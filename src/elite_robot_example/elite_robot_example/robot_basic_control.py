#!/usr/bin/env python3
"""
Elite CS 机械臂基础控制示例
功能：开机、关机、抱闸、获取位姿

使用方法：
ros2 run elite_robot_example robot_basic_control
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

import math
import time
from typing import Optional, List, Tuple, Dict


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """
    四元数转欧拉角 (roll, pitch, yaw)

    Args:
        x, y, z, w: 四元数

    Returns:
        (roll, pitch, yaw) 弧度
    """
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class RobotBasicControl(Node):
    """机械臂基础控制节点"""

    def __init__(self):
        super().__init__('robot_basic_control')

        self.callback_group = ReentrantCallbackGroup()

        # 状态变量
        self._joint_positions: Optional[List[float]] = None
        self._tcp_pose: Optional[Tuple[List[float], List[float]]] = None

        # 订阅器
        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
            callback_group=self.callback_group
        )

        self._tcp_pose_sub = self.create_subscription(
            PoseStamped,
            '/tcp_pose_broadcaster/pose',
            self._tcp_pose_callback,
            10,
            callback_group=self.callback_group
        )

        # 服务客户端
        self._power_on_client = self.create_client(
            Trigger, '/dashboard_client/power_on',
            callback_group=self.callback_group
        )

        self._power_off_client = self.create_client(
            Trigger, '/dashboard_client/power_off',
            callback_group=self.callback_group
        )

        self._brake_release_client = self.create_client(
            Trigger, '/dashboard_client/brake_release',
            callback_group=self.callback_group
        )

        self._brake_lock_client = self.create_client(
            Trigger, '/dashboard_client/power_off',  # power_off 会锁抱闸
            callback_group=self.callback_group
        )

        self.get_logger().info("Robot Basic Control 节点已初始化")

    # ========== 回调函数 ==========

    def _joint_state_callback(self, msg: JointState):
        """关节状态回调"""
        # 按照标准关节顺序排列
        joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self._joint_positions = [float(name_to_pos[j]) for j in joint_names]
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

    # ========== 等待状态 ==========

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

    # ========== 控制功能 ==========

    def power_on(self) -> bool:
        """开机上电"""
        if not self._power_on_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Dashboard 服务不可用")
            return False

        self.get_logger().info("正在上电...")
        future = self._power_on_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info("✅ 上电成功")
            return True
        else:
            self.get_logger().error("❌ 上电失败")
            return False

    def power_off(self) -> bool:
        """关机下电"""
        if not self._power_off_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Dashboard 服务不可用")
            return False

        self.get_logger().info("正在下电...")
        future = self._power_off_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info("✅ 下电成功")
            return True
        else:
            self.get_logger().error("❌ 下电失败")
            return False

    def brake_release(self) -> bool:
        """释放抱闸"""
        if not self._brake_release_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Dashboard 服务不可用")
            return False

        self.get_logger().info("正在释放抱闸...")
        future = self._brake_release_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info("✅ 抱闸已释放")
            return True
        else:
            self.get_logger().error("❌ 释放抱闸失败")
            return False

    def brake_lock(self) -> bool:
        """锁定抱闸（下电会自动锁抱闸）"""
        return self.power_off()

    def get_joint_positions(self) -> Optional[List[float]]:
        """获取关节位置（弧度）"""
        return self._joint_positions

    def get_joint_degrees(self) -> Optional[List[float]]:
        """获取关节位置（角度）"""
        if self._joint_positions is None:
            return None
        return [math.degrees(p) for p in self._joint_positions]

    def get_tcp_pose(self) -> Optional[Tuple[List[float], List[float]]]:
        """获取末端位姿 (position[x,y,z], quaternion[x,y,z,w])"""
        return self._tcp_pose

    def get_tcp_position(self) -> Optional[List[float]]:
        """获取末端位置 [x, y, z] (米)"""
        if self._tcp_pose is None:
            return None
        return self._tcp_pose[0]

    def get_tcp_orientation_euler(self) -> Optional[Tuple[float, float, float]]:
        """获取末端旋转角度 (roll, pitch, yaw) 弧度"""
        if self._tcp_pose is None:
            return None
        _, ori = self._tcp_pose
        return quaternion_to_euler(ori[0], ori[1], ori[2], ori[3])

    def get_tcp_orientation_euler_degrees(self) -> Optional[Tuple[float, float, float]]:
        """获取末端旋转角度 (roll, pitch, yaw) 角度"""
        euler = self.get_tcp_orientation_euler()
        if euler is None:
            return None
        return (math.degrees(euler[0]), math.degrees(euler[1]), math.degrees(euler[2]))

    def get_tcp_pose_dict(self) -> Optional[Dict[str, float]]:
        """
        获取末端位姿字典，方便使用

        Returns:
            字典包含：
            - x, y, z: 位置 (米)
            - roll, pitch, yaw: 旋转角度 (弧度)
            - roll_deg, pitch_deg, yaw_deg: 旋转角度 (角度)
            - qx, qy, qz, qw: 四元数
        """
        if self._tcp_pose is None:
            return None

        pos, ori = self._tcp_pose
        roll, pitch, yaw = quaternion_to_euler(ori[0], ori[1], ori[2], ori[3])

        return {
            'x': pos[0],
            'y': pos[1],
            'z': pos[2],
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'roll_deg': math.degrees(roll),
            'pitch_deg': math.degrees(pitch),
            'yaw_deg': math.degrees(yaw),
            'qx': ori[0],
            'qy': ori[1],
            'qz': ori[2],
            'qw': ori[3]
        }

    def print_status(self):
        """打印当前状态"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("当前机械臂状态：")

        if self._joint_positions:
            self.get_logger().info("关节位置（度）：")
            joint_names = ["J1 底座", "J2 肩部", "J3 肘部", "J4 腕部1", "J5 腕部2", "J6 腕部3"]
            for i, (name, pos) in enumerate(zip(joint_names, self.get_joint_degrees())):
                self.get_logger().info(f"  {name}: {pos:.2f}°")
        else:
            self.get_logger().info("关节位置：等待数据...")

        if self._tcp_pose:
            pos, ori = self._tcp_pose
            roll, pitch, yaw = quaternion_to_euler(ori[0], ori[1], ori[2], ori[3])

            self.get_logger().info("末端位姿（基座坐标系）：")
            self.get_logger().info(f"  位置 X: {pos[0]:.4f} m")
            self.get_logger().info(f"  位置 Y: {pos[1]:.4f} m")
            self.get_logger().info(f"  位置 Z: {pos[2]:.4f} m")
            self.get_logger().info(f"  旋转 Roll:  {math.degrees(roll):.2f}° ({roll:.4f} rad)")
            self.get_logger().info(f"  旋转 Pitch: {math.degrees(pitch):.2f}° ({pitch:.4f} rad)")
            self.get_logger().info(f"  旋转 Yaw:   {math.degrees(yaw):.2f}° ({yaw:.4f} rad)")
            self.get_logger().info(f"  四元数: [{ori[0]:.4f}, {ori[1]:.4f}, {ori[2]:.4f}, {ori[3]:.4f}]")
        else:
            self.get_logger().info("末端位姿：等待数据...")

        self.get_logger().info("=" * 60)


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    node = RobotBasicControl()
    executor.add_node(node)

    try:
        # 等待状态
        if not node.wait_for_state(timeout=10.0):
            node.get_logger().error("无法连接机械臂")
            return

        # 打印初始状态
        node.print_status()

        # 示例：开机流程
        node.get_logger().info("\n===== 开机流程 =====")
        node.power_on()
        time.sleep(2)
        node.brake_release()
        time.sleep(1)
        node.print_status()

        # 示例：获取末端位姿的多种方式
        node.get_logger().info("\n===== 获取末端位姿示例 =====")

        # 方式1：获取原始数据
        pose = node.get_tcp_pose()
        if pose:
            pos, ori = pose
            node.get_logger().info(f"方式1 - 原始数据: pos={pos}, ori={ori}")

        # 方式2：获取位置
        position = node.get_tcp_position()
        if position:
            node.get_logger().info(f"方式2 - 位置: x={position[0]:.4f}, y={position[1]:.4f}, z={position[2]:.4f}")

        # 方式3：获取旋转角度（弧度）
        euler = node.get_tcp_orientation_euler()
        if euler:
            node.get_logger().info(f"方式3 - 旋转角度(弧度): roll={euler[0]:.4f}, pitch={euler[1]:.4f}, yaw={euler[2]:.4f}")

        # 方式4：获取旋转角度（角度）
        euler_deg = node.get_tcp_orientation_euler_degrees()
        if euler_deg:
            node.get_logger().info(f"方式4 - 旋转角度(角度): roll={euler_deg[0]:.2f}°, pitch={euler_deg[1]:.2f}°, yaw={euler_deg[2]:.2f}°")

        # 方式5：获取字典格式
        pose_dict = node.get_tcp_pose_dict()
        if pose_dict:
            node.get_logger().info(f"方式5 - 字典格式:")
            node.get_logger().info(f"  位置: x={pose_dict['x']:.4f}, y={pose_dict['y']:.4f}, z={pose_dict['z']:.4f}")
            node.get_logger().info(f"  旋转: roll={pose_dict['roll_deg']:.2f}°, pitch={pose_dict['pitch_deg']:.2f}°, yaw={pose_dict['yaw_deg']:.2f}°")

        # 等待用户输入
        node.get_logger().info("\n机械臂已开机，按 Ctrl+C 关机")

        # 保持运行
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        node.get_logger().info("\n===== 关机流程 =====")
        node.power_off()
        node.get_logger().info("已关机")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
