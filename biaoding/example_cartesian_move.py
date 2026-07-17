#!/usr/bin/env python3
"""
笛卡尔空间运动示例

功能：
1. 连接到 Elite Robot 机械臂
2. 获取当前末端位姿
3. 执行笛卡尔空间运动
4. 打印运动结果

使用方法：
1. 启动机械臂驱动：
   ros2 launch eli_cs_robot_driver elite_control.launch.py robot_ip:=192.168.1.212 cs_type:=cs66

2. 运行本示例：
   python3 example_cartesian_move.py
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor
import sys
import os
import time

# 添加 elite_robot_example 包路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'elite_robot_example'))

from elite_robot_example.robot_cartesian_control import RobotCartesianControl


def main():
    """主函数"""

    # 初始化 ROS2
    rclpy.init()
    executor = MultiThreadedExecutor()
    robot_node = RobotCartesianControl()
    executor.add_node(robot_node)

    try:
        # 等待机械臂状态
        print("正在连接机械臂...")
        if not robot_node.wait_for_state(timeout=10.0):
            print("错误：无法连接机械臂")
            return

        print("机械臂已连接！")
        robot_node.print_status()

        # 示例 1：获取当前位姿
        print("\n===== 示例 1：获取当前位姿 =====")
        tcp_pose = robot_node.get_tcp_pose()
        if tcp_pose:
            pos, ori = tcp_pose
            print(f"当前位置: x={pos[0]:.4f}, y={pos[1]:.4f}, z={pos[2]:.4f}")
            print(f"当前姿态: x={ori[0]:.4f}, y={ori[1]:.4f}, z={ori[2]:.4f}, w={ori[3]:.4f}")

        # 示例 2：移动到零位
        print("\n===== 示例 2：移动到零位 =====")
        robot_node.move_to_home(duration=5.0)
        time.sleep(1.0)
        robot_node.print_status()

        # 示例 3：笛卡尔空间运动
        print("\n===== 示例 3：笛卡尔空间运动 =====")
        # 目标位置：在当前位置基础上向上移动 10cm
        if tcp_pose:
            current_pos, current_ori = tcp_pose
            target_position = [current_pos[0], current_pos[1], current_pos[2] + 0.1]
            target_orientation = current_ori  # 保持当前姿态

            print(f"目标位置: x={target_position[0]:.4f}, y={target_position[1]:.4f}, z={target_position[2]:.4f}")
            print(f"目标姿态: x={target_orientation[0]:.4f}, y={target_orientation[1]:.4f}, z={target_orientation[2]:.4f}, w={target_orientation[3]:.4f}")

            success = robot_node.move_to_cartesian_pose(
                target_position,
                target_orientation,
                duration=5.0
            )

            if success:
                print("笛卡尔空间运动完成！")
            else:
                print("笛卡尔空间运动失败！")

            time.sleep(1.0)
            robot_node.print_status()

        # 示例 4：关节空间运动
        print("\n===== 示例 4：关节空间运动 =====")
        # 移动到准备位置
        robot_node.move_to_ready(duration=5.0)
        time.sleep(1.0)
        robot_node.print_status()

        # 示例 5：夹爪控制
        print("\n===== 示例 5：夹爪控制 =====")
        print("关闭夹爪...")
        robot_node.close_gripper(pin=0)
        time.sleep(2.0)

        print("打开夹爪...")
        robot_node.open_gripper(pin=0)
        time.sleep(1.0)

        # 示例 6：回到零位
        print("\n===== 示例 6：回到零位 =====")
        robot_node.move_to_home(duration=5.0)
        time.sleep(1.0)
        robot_node.print_status()

        print("\n===== 所有示例执行完成 =====")

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        # 清理资源
        robot_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
