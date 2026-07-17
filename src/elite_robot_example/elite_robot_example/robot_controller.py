#!/usr/bin/env python3
"""
Elite CS 机械臂运动控制与状态反馈示例
功能：
1. 订阅机械臂状态（关节位置、末端位姿）
2. 发送关节运动指令
3. 控制夹爪（数字 IO）
4. 机器人管理（上电、下电等）

使用方法：
1. 启动机械臂驱动：
   ros2 launch eli_cs_robot_driver elite_control.launch.py robot_ip:=192.168.1.212 cs_type:=cs66

2. 运行本示例：
   ros2 run elite_robot_example robot_controller
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

import math
import time
from typing import List, Optional, Tuple


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


class EliteRobotController(Node):
    """Elite 机械臂控制器节点"""

    def __init__(self):
        super().__init__('elite_robot_controller')

        # 使用多线程执行器
        self.callback_group = ReentrantCallbackGroup()

        # ========== 状态变量 ==========
        self._joint_positions: Optional[List[float]] = None
        self._joint_velocities: Optional[List[float]] = None
        self._tcp_pose: Optional[Tuple[List[float], List[float]]] = None  # (position, orientation)

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
            '/tcp_pose_broadcaster/tcp_pose',
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

        self.get_logger().info("Elite Robot Controller 已初始化")

    # ========== 回调函数 ==========

    def _joint_state_callback(self, msg: JointState):
        """关节状态回调"""
        name_to_pos = dict(zip(msg.name, msg.position))
        name_to_vel = dict(zip(msg.name, msg.velocity))

        try:
            self._joint_positions = [float(name_to_pos[j]) for j in JOINT_NAMES]
            self._joint_velocities = [float(name_to_vel[j]) for j in JOINT_NAMES]
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

    def print_status(self):
        """打印当前状态"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("当前机械臂状态：")

        if self._joint_positions:
            self.get_logger().info("关节位置（度）：")
            for i, (name, pos) in enumerate(zip(JOINT_NAMES, self._joint_degrees())):
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

        # 构建轨迹目标
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES

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
    node = EliteRobotController()
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
        node.get_logger().info("===== 机械臂控制器已就绪 =====")
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

        # 示例 3：单独移动关节 0（底座旋转）
        node.get_logger().info("\n示例 3：单独移动关节 0 到 45 度")
        node.move_joint_by_index(0, 45.0, duration=3.0)
        time.sleep(1.0)

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
