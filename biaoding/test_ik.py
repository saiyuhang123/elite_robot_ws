#!/usr/bin/env python3
"""IK 解算测试工具：手动输入目标点（基座系），验证 5 维 IK 能否解算成功。

功能：
  - 输入 x y z（基座系，米），工具轴方向默认世界系"下"（45° 倾斜模型）
  - 打印：FK 与控制器上报的位置漂移、IK 结果关节角、FK 验证误差
  - 可选：确认后直接发送 movej 执行

前提：机械臂驱动已启动（/joint_states 和 /tcp_pose_broadcaster/pose 正常）

用法：
  cd ~/Documents/elite_robot_ws/biaoding
  python3 test_ik.py
"""

import math
import sys
import os

import numpy as np
from scipy.spatial.transform import Rotation as Rot

import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'elite_robot_example'))
from elite_robot_example.robot_cartesian_control import (
    RobotCartesianControl,
    cs66_inverse_kinematics_5dof,
    cs66_forward_kinematics,
)
from std_msgs.msg import String

# 倾斜安装：世界系"上"在基座系下的方向（实测值）
# 测量方法：物理法兰调水平后 tf2_echo cs66_base_link cs66_tool0，取旋转矩阵
# 第三列（法兰Z轴=世界的"下"）取反。实测倾斜角约 48°。
# 注意：不要用 cs66_tool0_controller（控制器上报的虚拟TCP），它和物理法兰
# 差一个约118°的固定旋转（TCP偏移），用它算出来的方向全是错的。
V_UP_IN_BASE = np.array([-0.7431, 0.0120, 0.6691])
DOWN_IN_BASE = -V_UP_IN_BASE

SCRIPT_TOPIC = "/script_sender/script_command"
MOVEJ_A = 1.0
MOVEJ_V = 0.2

# 工具偏移（米）：法兰面到夹爪掌心/指尖的距离，沿法兰 Z 轴（机械手约 11cm）。
# IK 目标 = 输入点 - L × 工具轴方向，使【掌心/指尖】到达输入点。
TOOL_TIP_LENGTH = 0.11


def spin_some(node, n=10, dt=0.05):
    for _ in range(n):
        rclpy.spin_once(node, timeout_sec=dt)


def main():
    rclpy.init()
    node = RobotCartesianControl()
    script_pub = node.create_publisher(String, SCRIPT_TOPIC, 10)

    if not node.wait_for_state(timeout=10.0):
        print("错误：收不到机械臂状态，请确认驱动已启动")
        node.destroy_node()
        rclpy.shutdown()
        return

    print("=" * 60)
    print("IK 解算测试工具")
    print("  输入: x y z        —— 目标点（基座系，米），IK 解算并验证")
    print("        x y z !      —— 同上，但解算成功后直接 movej 执行")
    print("        d dx dy dz   —— 自定义工具轴方向（默认世界系'下'）")
    print("        c            —— 打印当前 TCP 和关节角")
    print("        q            —— 退出")
    print(f"  工具轴方向: {np.round(DOWN_IN_BASE, 4)}（世界系'下'，实测倾斜模型）")
    print("=" * 60)

    target_dir = DOWN_IN_BASE.copy()

    try:
        while True:
            spin_some(node, 2)
            try:
                line = input("\ntarget> ").strip()
            except EOFError:
                break
            if not line:
                continue
            parts = line.split()
            cmd = parts[0].lower()

            if cmd == 'q':
                break
            elif cmd == 'c':
                spin_some(node)
                tcp = node.get_tcp_pose()
                joints = node.get_joint_degrees()
                if tcp:
                    print(f"  TCP 位置: {np.round(tcp[0], 4)}")
                    print(f"  TCP 姿态(四元数): {np.round(tcp[1], 4)}")
                if joints:
                    print(f"  关节角(度): {np.round(joints, 1)}")
                continue
            elif cmd == 'd':
                if len(parts) != 4:
                    print("  用法: d dx dy dz")
                    continue
                target_dir = np.array([float(v) for v in parts[1:]])
                target_dir /= np.linalg.norm(target_dir)
                print(f"  工具轴方向已设为: {np.round(target_dir, 4)}")
                continue

            # x y z [!]
            execute = parts[-1] == '!'
            if execute:
                parts = parts[:-1]
            if len(parts) != 3:
                print("  用法: x y z [!]")
                continue
            try:
                target_pos = np.array([float(v) for v in parts])
            except ValueError:
                print("  数字格式错误")
                continue

            spin_some(node)
            q_guess = node.get_joint_positions()
            tcp = node.get_tcp_pose()
            if q_guess is None or tcp is None:
                print("  收不到机械臂状态")
                continue

            # FK 与控制器上报的一致性检查（FK=URDF=物理法兰，无需姿态校正）
            fk_pos_now, _ = cs66_forward_kinematics(q_guess)
            drift = float(np.linalg.norm(fk_pos_now - np.array(tcp[0])))
            print(f"  FK与上报位置漂移: {drift*1000:.1f}mm" +
                  ("  !! 偏大，模型可能不一致" if drift > 10 else "  (正常)"))

            # 工具偏移：让掌心/指尖到达输入点，法兰停在后方 L 处
            flange_target = target_pos - TOOL_TIP_LENGTH * target_dir
            if TOOL_TIP_LENGTH > 0:
                print(f"  法兰目标(输入点-L): {np.round(flange_target, 4)}")

            # 可达性粗检
            dist = float(np.linalg.norm(flange_target - np.array([0.0, 0.0, 0.1625])))
            print(f"  目标距肩关节: {dist:.3f}m" +
                  ("  !! 超过臂展 0.92m，必然无解" if dist > 0.92 else ""))

            joint_target = cs66_inverse_kinematics_5dof(
                flange_target, target_dir, q_guess)
            if joint_target is None:
                print("  IK 解算失败（目标不可达）")
                continue

            fk_pos, fk_rot = cs66_forward_kinematics(joint_target)
            pos_err = float(np.linalg.norm(fk_pos - flange_target))
            achieved_dir = fk_rot[:, 2]
            dir_err = math.degrees(math.acos(
                float(np.clip(achieved_dir @ target_dir, -1.0, 1.0))))

            print(f"  IK 成功!")
            print(f"    关节角(rad): {np.round(joint_target, 4).tolist()}")
            print(f"    关节角(度):  {np.round(np.degrees(joint_target), 1).tolist()}")
            print(f"    位置误差: {pos_err*1000:.2f}mm   方向误差: {dir_err:.2f}°")

            if pos_err > 0.02 or dir_err > 5.0:
                print("  !! 误差过大，判定不可达，不执行")
                continue

            if execute:
                j = ", ".join(f"{x:.6f}" for x in joint_target)
                script = f"def prog():\n    movej([{j}], a={MOVEJ_A:.3f}, v={MOVEJ_V:.3f}, r=0)\nend"
                msg = String()
                msg.data = script
                script_pub.publish(msg)
                print("  已发送 movej，机械臂执行中...")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("已退出")


if __name__ == '__main__':
    main()
