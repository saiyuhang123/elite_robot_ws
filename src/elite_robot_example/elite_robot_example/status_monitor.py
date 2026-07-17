#!/usr/bin/env python3
"""
Elite CS 机械臂状态监控节点
功能：实时显示关节位置和末端位姿

使用方法：
ros2 run elite_robot_example status_monitor
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import math
from typing import List, Optional


JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class StatusMonitor(Node):
    """状态监控节点"""

    def __init__(self):
        super().__init__('status_monitor')

        self._joint_positions: Optional[List[float]] = None
        self._tcp_pose = None

        # 订阅关节状态
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_callback,
            10
        )

        # 订阅末端位姿
        self.create_subscription(
            PoseStamped,
            '/tcp_pose_broadcaster/tcp_pose',
            self._tcp_pose_callback,
            10
        )

        # 定时打印状态
        self.create_timer(1.0, self.print_status)

        self.get_logger().info("状态监控已启动，每秒更新一次...")

    def _joint_callback(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self._joint_positions = [float(name_to_pos[j]) for j in JOINT_NAMES]
        except KeyError:
            pass

    def _tcp_pose_callback(self, msg: PoseStamped):
        pos = msg.pose.position
        ori = msg.pose.orientation
        self._tcp_pose = (pos, ori)

    def print_status(self):
        print("\033[2J\033[H")  # 清屏
        print("=" * 60)
        print("       Elite CS 机械臂状态监控")
        print("=" * 60)

        if self._joint_positions:
            print("\n【关节位置】")
            print("-" * 40)
            for i, (name, pos) in enumerate(zip(JOINT_NAMES, self._joint_positions)):
                deg = math.degrees(pos)
                bar_len = int(abs(deg) / 5)
                bar = "█" * min(bar_len, 20)
                sign = "+" if deg >= 0 else "-"
                print(f"  J{i+1} {name:20s}: {deg:+8.2f}° {sign}{bar}")
        else:
            print("\n【关节位置】等待数据...")

        if self._tcp_pose:
            pos, ori = self._tcp_pose
            print(f"\n【末端位姿】")
            print("-" * 40)
            print(f"  位置 X: {pos.x:+.4f} m")
            print(f"  位置 Y: {pos.y:+.4f} m")
            print(f"  位置 Z: {pos.z:+.4f} m")
            print(f"  姿态 X: {ori.x:+.4f}")
            print(f"  姿态 Y: {ori.y:+.4f}")
            print(f"  姿态 Z: {ori.z:+.4f}")
            print(f"  姿态 W: {ori.w:+.4f}")
        else:
            print("\n【末端位姿】等待数据...")

        print("\n" + "=" * 60)
        print("按 Ctrl+C 退出")
        print("=" * 60)


def main(args=None):
    rclpy.init(args=args)
    node = StatusMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
