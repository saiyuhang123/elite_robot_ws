#!/usr/bin/env python3
"""
Elite CS 机械臂笛卡尔空间控制示例
功能：
1. 获取末端位姿
2. 笛卡尔空间运动（通过修改硬件接口支持）
3. 关节空间运动
4. 夹爪控制

使用方法：
1. 启动机械臂驱动：
   ros2 launch eli_cs_robot_driver elite_control.launch.py robot_ip:=192.168.1.212 cs_type:=cs66

2. 运行本示例：
   ros2 run elite_robot_example robot_cartesian_control
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from eli_common_interface.srv import SetIO
from std_srvs.srv import Trigger
from std_msgs.msg import String

import math
import time
import numpy as np
from typing import List, Optional, Tuple, Dict


# CS66 关节名称
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# 预义的关节位置（弧度）
HOME_POSITION = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]  # 零位
READY_POSITION = [0.0, -1.2, 0.8, -1.57, -1.57, 0.0]  # 准备位置

# ==========================================================
# CS66 DH 参数与运动学
# ==========================================================
# DH 参数 (Modified DH convention)
# 出厂标定值（由控制器 30001 端口读取的 MDH，与 cartesian_move.cpp 一致）。
# 注意：不要使用型录标称值，旧值 d6=0.0920 与实测 0.112116 差约 2cm，
# 会直接导致末端定位偏差。
CS66_DH = {
    'd1': 0.160861,   # 底座高度
    'a2': -0.42752,   # 上臂长度
    'a3': -0.391601,  # 前臂长度
    'd4': 0.147568,   # 腕部偏移
    'd5': 0.0964976,  # 腕部2偏移
    'd6': 0.112116,   # 末端到法兰
}


def _rot_z(theta: float) -> np.ndarray:
    """绕 Z 轴旋转"""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _rot_x(theta: float) -> np.ndarray:
    """绕 X 轴旋转"""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def cs66_forward_kinematics(joint_angles: List[float]) -> Tuple[np.ndarray, np.ndarray]:
    """
    CS66 正运动学（Modified DH）

    Args:
        joint_angles: 6 个关节角度（弧度）

    Returns:
        (position[3], rotation_matrix[3x3]) TCP 在基座坐标系下的位姿
    """
    d = CS66_DH
    q1, q2, q3, q4, q5, q6 = joint_angles

    # T_01: base → shoulder_pan
    T = np.eye(4)
    T[:3, :3] = _rot_z(q1)
    T[:3, 3] = [0, 0, d['d1']]

    # T_12: shoulder_pan → shoulder_lift (R_x(π/2) * R_z(q2))
    T12 = np.eye(4)
    T12[:3, :3] = _rot_x(math.pi / 2) @ _rot_z(q2)
    T12[:3, 3] = [0, 0, 0]
    T = T @ T12

    # T_23: shoulder_lift → elbow
    T23 = np.eye(4)
    T23[:3, :3] = _rot_z(q3)
    T23[:3, 3] = [d['a2'], 0, 0]
    T = T @ T23

    # T_34: elbow → wrist_1
    T34 = np.eye(4)
    T34[:3, :3] = _rot_z(q4)
    T34[:3, 3] = [d['a3'], 0, d['d4']]
    T = T @ T34

    # T_45: wrist_1 → wrist_2 (R_x(π/2) * T_y(-d5) * R_z(q5))
    T45 = np.eye(4)
    T45[:3, :3] = _rot_x(math.pi / 2) @ _rot_z(q5)
    T45[:3, 3] = [0, -d['d5'], 0]
    T = T @ T45

    # T_56: wrist_2 → wrist_3 (R_x(-π/2) * T_y(d6) * R_z(q6))
    T56 = np.eye(4)
    T56[:3, :3] = _rot_x(-math.pi / 2) @ _rot_z(q6)
    T56[:3, 3] = [0, d['d6'], 0]
    T = T @ T56

    return T[:3, 3], T[:3, :3]


def cs66_inverse_kinematics(
    target_pos: np.ndarray,
    target_rot: np.ndarray,
    q_guess: List[float],
    max_iter: int = 200,
    tolerance: float = 1e-4,
    joint_margin: float = math.pi  # ±180° 范围
) -> Optional[List[float]]:
    """
    CS66 逆运动学（数值法，Levenberg-Marquardt + 正则化）

    Args:
        target_pos: 目标位置 [x, y, z]
        target_rot: 目标旋转矩阵 (3x3)
        q_guess: 初始关节角度猜测（弧度）
        max_iter: 最大迭代次数
        tolerance: 收敛精度
        joint_margin: 关节偏离初始值的最大范围（弧度），默认 ±π

    Returns:
        6 个关节角度（弧度），失败返回 None
    """
    from scipy.optimize import least_squares

    q0 = np.array(q_guess)

    def pose_error(q):
        pos, rot = cs66_forward_kinematics(q)
        pos_err = pos - target_pos
        rot_err = (rot - target_rot).flatten()
        # 弱正则化：轻微惩罚偏离初始值
        reg = 0.001 * (q - q0)
        return np.concatenate([10.0 * pos_err, rot_err, reg])

    # 边界：以当前位置为中心，±joint_margin 范围
    bounds = (
        [q0[i] - joint_margin for i in range(6)],
        [q0[i] + joint_margin for i in range(6)],
    )

    result = least_squares(
        pose_error,
        q0,
        bounds=bounds,
        method='trf',
        max_nfev=max_iter,
        ftol=tolerance,
        xtol=1e-8,
    )

    if result.success or np.linalg.norm(pose_error(result.x)[:3]) < 0.02:
        return list(result.x)
    return None


def quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """
    四元数转旋转矩阵

    Args:
        x, y, z, w: 四元数

    Returns:
        3x3 旋转矩阵
    """
    # 归一化四元数
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/norm, y/norm, z/norm, w/norm

    # 计算旋转矩阵
    R = np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
        [2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]
    ])
    return R


def rotation_matrix_to_quaternion(R: np.ndarray) -> Tuple[float, float, float, float]:
    """
    旋转矩阵转四元数

    Args:
        R: 3x3 旋转矩阵

    Returns:
        (x, y, z, w) 四元数
    """
    trace = np.trace(R)

    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    return x, y, z, w


class RobotCartesianControl(Node):
    """机械臂笛卡尔空间控制节点"""

    def __init__(self):
        super().__init__('robot_cartesian_control')

        # 使用多线程执行器
        self.callback_group = ReentrantCallbackGroup()

        # ========== 状态变量 ==========
        self._joint_positions: Optional[List[float]] = None
        self._joint_velocities: Optional[List[float]] = None
        self._tcp_pose: Optional[Tuple[List[float], List[float]]] = None  # (position, orientation)
        self._joint_name_map: Optional[Dict[str, str]] = None  # 期望关节名 -> 实际关节名（兼容 tf_prefix）

        # ========== 订阅器 ==========
        # 订阅关节状态
        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
            callback_group=self.callback_group
        )

        # 订阅末端位姿
        self._tcp_pose_sub = self.create_subscription(
            PoseStamped,
            '/tcp_pose_broadcaster/pose',
            self._tcp_pose_callback,
            10,
            callback_group=self.callback_group
        )

        # ========== Action 客户端 ==========
        # 轨迹执行客户端
        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )

        # ========== 服务客户端 ==========
        # IO 控制服务（用于夹爪）
        self._set_io_client = self.create_client(
            SetIO,
            '/io_and_status_controller/set_io',
            callback_group=self.callback_group
        )

        # Dashboard 服务
        self._power_on_client = self.create_client(
            Trigger,
            '/dashboard_client/power_on',
            callback_group=self.callback_group
        )

        self._power_off_client = self.create_client(
            Trigger,
            '/dashboard_client/power_off',
            callback_group=self.callback_group
        )

        self._brake_release_client = self.create_client(
            Trigger,
            '/dashboard_client/brake_release',
            callback_group=self.callback_group
        )

        # 控制权切换服务（笛卡尔运动用）
        self._hand_back_control_client = self.create_client(
            Trigger,
            '/io_and_status_controller/hand_back_control',
            callback_group=self.callback_group
        )

        self._resend_external_script_client = self.create_client(
            Trigger,
            '/io_and_status_controller/resend_external_script',
            callback_group=self.callback_group
        )

        # ========== 脚本命令发布者 ==========
        self._script_command_pub = self.create_publisher(
            String,
            '/script_sender/script_command',
            10
        )

        self.get_logger().info("Elite Robot Cartesian Control 已初始化")

    # ========== 回调函数 ==========

    def _joint_state_callback(self, msg: JointState):
        """关节状态回调（兼容带 tf_prefix 的关节名，如 cs66_shoulder_pan_joint）"""
        name_to_pos = dict(zip(msg.name, msg.position))
        name_to_vel = dict(zip(msg.name, msg.velocity))

        # 首次收到时建立 期望名 -> 实际名 映射（驱动带 tf_prefix 时关节名有前缀）
        if self._joint_name_map is None:
            mapping = {}
            for expected in JOINT_NAMES:
                for actual in msg.name:
                    if actual == expected or actual.endswith("_" + expected):
                        mapping[expected] = actual
                        break
            if len(mapping) != len(JOINT_NAMES):
                return  # 关节名还没认全，等下一帧
            self._joint_name_map = mapping
            self.get_logger().info(f"关节名映射: {self._joint_name_map}")

        try:
            self._joint_positions = [float(name_to_pos[self._joint_name_map[j]]) for j in JOINT_NAMES]
            self._joint_velocities = [float(name_to_vel[self._joint_name_map[j]]) for j in JOINT_NAMES]
        except KeyError:
            pass

    def _tcp_pose_callback(self, msg: PoseStamped):
        """末端位姿回调"""
        pos = msg.pose.position
        ori = msg.pose.orientation
        self._tcp_pose = (
            [pos.x, pos.y, pos.z],
            [ori.x, ori.y, ori.z, ori.w]
        )

    # ========== 状态获取 ==========

    def wait_for_state(self, timeout: float = 5.0) -> bool:
        """等待接收机械臂状态"""
        self.get_logger().info("等待机械臂状态...")
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._joint_positions is not None:
                self.get_logger().info("已接收到机械臂状态")
                return True
        self.get_logger().error("等待状态超时")
        return False

    def get_joint_positions(self) -> Optional[List[float]]:
        """获取当前关节位置（弧度）"""
        return self._joint_positions

    def get_joint_degrees(self) -> Optional[List[float]]:
        """获取当前关节位置（角度）"""
        if self._joint_positions is None:
            return None
        return [math.degrees(p) for p in self._joint_positions]

    def get_tcp_pose(self) -> Optional[Tuple[List[float], List[float]]]:
        """获取当前末端位姿 (position[x,y,z], quaternion[x,y,z,w])"""
        return self._tcp_pose

    def get_tcp_pose_dict(self) -> Optional[Dict[str, float]]:
        """
        获取末端位姿字典，方便使用

        Returns:
            字典包含：
            - x, y, z: 位置 (米)
            - qx, qy, qz, qw: 四元数
        """
        if self._tcp_pose is None:
            return None

        pos, ori = self._tcp_pose
        return {
            'x': pos[0],
            'y': pos[1],
            'z': pos[2],
            'qx': ori[0],
            'qy': ori[1],
            'qz': ori[2],
            'qw': ori[3],
        }

    def print_status(self):
        """打印当前状态"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("当前机械臂状态：")

        if self._joint_positions:
            self.get_logger().info("关节位置（度）：")
            for i, (name, pos) in enumerate(zip(JOINT_NAMES, self.get_joint_degrees())):
                self.get_logger().info(f"  {name}: {pos:.2f}°")

        if self._tcp_pose:
            pos, ori = self._tcp_pose
            self.get_logger().info(f"末端位置：x={pos[0]:.4f}, y={pos[1]:.4f}, z={pos[2]:.4f}")
            self.get_logger().info(f"末端姿态（四元数）：x={ori[0]:.4f}, y={ori[1]:.4f}, z={ori[2]:.4f}, w={ori[3]:.4f}")

        self.get_logger().info("=" * 50)

    # ========== 运动控制 ==========

    def move_to_joint_positions(
        self,
        target_positions: List[float],
        duration: float = 5.0,
        use_degrees: bool = False
    ) -> bool:
        """
        移动到目标关节位置

        Args:
            target_positions: 目标关节位置（6 个值）
            duration: 运动时间（秒）
            use_degrees: 是否使用角度（True）或弧度（False）

        Returns:
            是否成功
        """
        if len(target_positions) != 6:
            self.get_logger().error("目标位置必须是 6 个关节值")
            return False

        # 转换为弧度
        if use_degrees:
            target_rad = [math.radians(p) for p in target_positions]
        else:
            target_rad = target_positions

        # 等待状态
        if not self.wait_for_state():
            return False

        # 等待 action 服务器
        if not self._trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("轨迹服务器不可用")
            return False

        # 构建轨迹目标（用驱动实际的关节名，可能带 tf_prefix 前缀）
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [self._joint_name_map.get(j, j) for j in JOINT_NAMES]

        # 起点：当前位置
        start_point = JointTrajectoryPoint()
        start_point.positions = list(self._joint_positions)
        start_point.time_from_start = Duration(sec=0, nanosec=0)
        goal.trajectory.points.append(start_point)

        # 终点：目标位置
        end_point = JointTrajectoryPoint()
        end_point.positions = list(target_rad)
        end_point.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration % 1) * 1e9)
        )
        goal.trajectory.points.append(end_point)

        # 发送目标
        self.get_logger().info(f"发送运动指令，运动时间：{duration}秒")
        future = self._trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error("目标被拒绝")
            return False

        # 等待完成
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code == 0:
            self.get_logger().info("运动完成")
            return True
        else:
            self.get_logger().error(f"运动失败，错误码：{result.error_code}")
            return False

    def move_to_cartesian_pose(
        self,
        target_position: List[float],
        target_orientation: List[float],
        duration: float = 5.0,
        speed: float = 0.1,
        acceleration: float = 0.3
    ) -> bool:
        """
        笛卡尔空间运动 —— 通过 IK 解算 + 关节轨迹控制实现

        先尝试 IK + move_to_joint_positions（可靠）。
        失败则回退到 script_sender 方法。

        Args:
            target_position: 目标位置 [x, y, z]（米）
            target_orientation: 目标姿态 [x, y, z, w]（四元数）
            duration: 运动时间（秒）
            speed: 运动速度（米/秒，仅 script_sender 模式）
            acceleration: 运动加速度（米/秒²，仅 script_sender 模式）

        Returns:
            是否成功
        """
        self.get_logger().info(f"目标笛卡尔位姿：位置={target_position}, 姿态={target_orientation}")

        if not self.wait_for_state():
            return False

        if self._joint_positions is None:
            self.get_logger().error("无当前关节位置")
            return False

        # 目标旋转矩阵（从四元数）
        from scipy.spatial.transform import Rotation as Rot
        target_rot = Rot.from_quat(target_orientation).as_matrix()
        target_pos = np.array(target_position)

        # 当前关节角度作为 IK 初值
        q_guess = list(self._joint_positions)

        self.get_logger().info("计算逆运动学 (IK)...")
        joint_target = cs66_inverse_kinematics(target_pos, target_rot, q_guess)

        if joint_target is None:
            # 放宽约束重试（±2π 即 360°）
            self.get_logger().info("紧约束 IK 失败，尝试放宽边界...")
            joint_target = cs66_inverse_kinematics(
                target_pos, target_rot, q_guess,
                max_iter=500, tolerance=1e-3, joint_margin=2 * math.pi
            )

        if joint_target is not None:
            # 验证 IK 结果（位置 + 姿态）
            fk_pos, fk_rot = cs66_forward_kinematics(joint_target)
            fk_err = np.linalg.norm(fk_pos - target_pos)
            rot_err = np.linalg.norm(fk_rot - target_rot)
            self.get_logger().info(f"IK 成功，位置误差: {fk_err:.4f}m, 姿态误差: {rot_err:.4f}")
            self.get_logger().info(f"目标关节(度): {[f'{math.degrees(j):.1f}' for j in joint_target]}")

            if fk_err < 0.01 and rot_err < 0.05:  # FK 已用出厂标定参数，残差应在 mm 级
                self.get_logger().info("使用关节轨迹控制执行运动...")
                return self.move_to_joint_positions(joint_target, duration)
            else:
                self.get_logger().warn(f"IK 误差过大 (位置 {fk_err:.4f}m, 姿态 {rot_err:.4f})，目标可能不可达")

        # IK 完全失败
        self.get_logger().error("IK 解算失败，目标不可达")
        return False

    def _move_cartesian_via_script(
        self,
        target_position: List[float],
        target_orientation: List[float],
        duration: float = 5.0,
        speed: float = 0.1,
        acceleration: float = 0.3
    ) -> bool:
        """
        通过 script_sender 发送 movel 命令（回退方案）
        """
        quat = target_orientation
        from scipy.spatial.transform import Rotation as Rot
        r = Rot.from_quat(quat)
        rotvec = r.as_rotvec()

        x, y, z = target_position
        rx, ry, rz = rotvec

        # 步骤 1: 释放 ROS2 控制权
        self.get_logger().info("步骤 1: 释放 ROS2 控制权...")
        if not self._hand_back_control():
            return False
        time.sleep(0.5)

        # 步骤 2: 发送 movel
        self.get_logger().info("步骤 2: 发送 movel 指令...")
        script_cmd = (
            f"def eli_cartesian_move():\n"
            f"\tmovel(p[{x:.6f}, {y:.6f}, {z:.6f}, {rx:.6f}, {ry:.6f}, {rz:.6f}], "
            f"a={acceleration:.2f}, v={speed:.2f}, t=0, r=0)\n"
            f"end\n"
        )
        msg = String()
        msg.data = script_cmd
        self._script_command_pub.publish(msg)

        # 步骤 3: 等待到位
        self.get_logger().info("步骤 3: 等待运动完成...")
        target = np.array([x, y, z])
        start_time = time.time()
        timeout = duration * 3
        while time.time() - start_time < timeout:
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.05)
            if self._tcp_pose is not None:
                current_pos, _ = self._tcp_pose
                dist = np.linalg.norm(np.array(current_pos) - target)
                if dist < 0.01:
                    self.get_logger().info(f"已到达，距离: {dist:.4f}m")
                    break
            time.sleep(0.2)
        else:
            self.get_logger().warn("等待运动超时")

        # 步骤 4: 恢复外部控制
        self.get_logger().info("步骤 4: 恢复外部控制...")
        if not self._resend_external_script():
            self.get_logger().warn("恢复外部控制失败")
        time.sleep(2.0)
        self.wait_for_state(timeout=10.0)
        return True

    # ========== 控制权管理 ==========

    def _hand_back_control(self) -> bool:
        """释放 ROS2 外部控制权"""
        if not self._hand_back_control_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("hand_back_control 服务不可用")
            return False

        future = self._hand_back_control_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info("控制权已释放")
            return True
        else:
            self.get_logger().error("释放控制权失败")
            return False

    def _resend_external_script(self) -> bool:
        """重新发送外部控制脚本，恢复 ROS2 控制"""
        if not self._resend_external_script_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("resend_external_script 服务不可用")
            return False

        future = self._resend_external_script_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info("外部控制脚本已重新发送")
            return True
        else:
            self.get_logger().error("重新发送外部控制脚本失败")
            return False

    def move_to_home(self, duration: float = 5.0) -> bool:
        """移动到零位"""
        self.get_logger().info("移动到零位...")
        return self.move_to_joint_positions(HOME_POSITION, duration)

    def move_to_ready(self, duration: float = 5.0) -> bool:
        """移动到准备位置"""
        self.get_logger().info("移动到准备位置...")
        return self.move_to_joint_positions(READY_POSITION, duration)

    def move_joint_by_index(self, joint_index: int, target_angle: float, duration: float = 3.0) -> bool:
        """
        单独移动指定关节

        Args:
            joint_index: 关节索引（0-5）
            target_angle: 目标角度（度）
            duration: 运动时间（秒）
        """
        if joint_index < 0 or joint_index > 5:
            self.get_logger().error("关节索引必须在 0-5 之间")
            return False

        if not self.wait_for_state():
            return False

        # 复制当前位置，只修改目标关节
        target = list(self._joint_positions)
        target[joint_index] = math.radians(target_angle)

        return self.move_to_joint_positions(target, duration)

    # ========== IO 控制（夹爪）==========

    def set_digital_output(self, pin: int, value: bool) -> bool:
        """
        设置数字输出（用于控制夹爪）

        Args:
            pin: 输出引脚号（0-15）
            value: True=高电平, False=低电平
        """
        if not self._set_io_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("IO 服务不可用")
            return False

        request = SetIO.Request()
        request.fun = SetIO.Request.FUN_SET_DIGITAL_OUT
        request.pin = pin
        request.state = float(SetIO.Request.STATE_ON if value else SetIO.Request.STATE_OFF)

        future = self._set_io_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"数字输出 {pin} 设置为 {'高' if value else '低'}")
            return True
        else:
            self.get_logger().error("设置 IO 失败")
            return False

    def open_gripper(self, pin: int = 0) -> bool:
        """打开夹爪"""
        self.get_logger().info("打开夹爪...")
        return self.set_digital_output(pin, False)

    def close_gripper(self, pin: int = 0) -> bool:
        """关闭夹爪"""
        self.get_logger().info("关闭夹爪...")
        return self.set_digital_output(pin, True)

    # ========== 机器人管理 ==========

    def power_on(self) -> bool:
        """机器人上电"""
        if not self._power_on_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Dashboard 服务不可用")
            return False

        future = self._power_on_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info("机器人上电成功")
            return True
        return False

    def brake_release(self) -> bool:
        """释放刹车"""
        if not self._brake_release_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Dashboard 服务不可用")
            return False

        future = self._brake_release_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info("刹车释放成功")
            return True
        return False


def main(args=None):
    rclpy.init(args=args)

    # 使用多线程执行器
    executor = MultiThreadedExecutor()
    node = RobotCartesianControl()
    executor.add_node(node)

    try:
        # 等待状态
        if not node.wait_for_state(timeout=10.0):
            node.get_logger().error("无法连接机械臂，请检查：")
            node.get_logger().error("1. 机械臂驱动是否启动")
            node.get_logger().error("2. IP 地址是否正确 (192.168.1.212)")
            node.get_logger().error("3. 网络连接是否正常")
            return

        # 打印初始状态
        node.get_logger().info("===== 机械臂笛卡尔空间控制已就绪 =====")
        node.print_status()

        # 示例 1：移动到零位
        node.get_logger().info("\n示例 1：移动到零位")
        node.move_to_home(duration=5.0)
        time.sleep(1.0)
        node.print_status()

        # 示例 2：移动到准备位置
        node.get_logger().info("\n示例 2：移动到准备位置")
        node.move_to_ready(duration=5.0)
        time.sleep(1.0)
        node.print_status()

        # 示例 3：笛卡尔空间运动（需要安装 ikpy）
        node.get_logger().info("\n示例 3：笛卡尔空间运动")
        target_pos = [0.3, 0.3, 0.5]  # 目标位置
        target_ori = [0.0, 0.0, 0.0, 1.0]  # 目标姿态（四元数）
        node.move_to_cartesian_pose(target_pos, target_ori, duration=5.0)
        time.sleep(1.0)
        node.print_status()

        # 示例 4：夹爪控制
        node.get_logger().info("\n示例 4：夹爪控制")
        node.close_gripper(pin=0)
        time.sleep(2.0)
        node.open_gripper(pin=0)
        time.sleep(1.0)

        # 示例 5：回到零位
        node.get_logger().info("\n示例 5：回到零位")
        node.move_to_home(duration=5.0)

        node.get_logger().info("\n===== 所有示例执行完成 =====")

    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
