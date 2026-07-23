#!/usr/bin/env python3
"""世界系竖直方向标定工具。

用法：
  1. 用水平尺把【物理法兰端面】调到严格水平（法兰面朝下正对桌面）
  2. 运行本脚本：
       python3 calibrate_vertical.py
  3. 脚本读取 TF (cs66_base_link -> cs66_tool0)，计算世界系"上"在基座系下
     的方向向量 V_UP_IN_BASE 和基座倾斜角，并打印可直接粘贴到代码里的常量

注意：
  - 必须用物理法兰 cs66_tool0，不要用 cs66_tool0_controller（虚拟 TCP，
    与物理法兰差约 118° 固定旋转）
  - 前提：机械臂驱动已启动（TF 正常发布）
"""

import math
import sys

import rclpy
from rclpy.node import Node

try:
    import tf2_ros
except ImportError:
    print("错误：需要 tf2_ros（sudo apt install ros-humble-tf2-ros）")
    sys.exit(1)

BASE_FRAME = "cs66_base_link"
TOOL_FRAME = "cs66_tool0"   # URDF 物理法兰


def main():
    rclpy.init()
    node = Node("calibrate_vertical")
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer, node)

    print(f"正在查询 TF: {BASE_FRAME} -> {TOOL_FRAME} ...")
    try:
        t = buffer.lookup_transform(
            BASE_FRAME, TOOL_FRAME, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=3.0))
    except Exception as e:
        print(f"查询失败: {e}")
        print("请确认：1. 机械臂驱动已启动  2. tf_prefix 为 cs66_")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n

    # 旋转矩阵第三列 = 法兰 Z 轴在基座系下的方向
    tool_z = (2 * (x * z + w * y),
              2 * (y * z - w * x),
              1 - 2 * (x * x + y * y))

    # 法兰水平时，法兰 Z 轴 = 世界的"下"，取反 = 世界的"上"
    v_up = (-tool_z[0], -tool_z[1], -tool_z[2])
    norm = math.sqrt(sum(v * v for v in v_up))
    v_up = tuple(v / norm for v in v_up)

    tilt = math.degrees(math.acos(max(-1.0, min(1.0, v_up[2]))))

    print()
    print("=" * 56)
    print(f"法兰 Z 轴方向（=世界的下）: [{tool_z[0]: .4f}, {tool_z[1]: .4f}, {tool_z[2]: .4f}]")
    print(f"世界系'上' V_UP_IN_BASE:    [{v_up[0]: .4f}, {v_up[1]: .4f}, {v_up[2]: .4f}]")
    print(f"基座倾斜角:                 {tilt:.2f}°")
    print("=" * 56)
    print()
    print("粘贴到 visual_grasp_test.py / test_ik.py：")
    print(f"V_UP_IN_BASE = np.array([{v_up[0]:.4f}, {v_up[1]:.4f}, {v_up[2]:.4f}])")
    print()
    print("粘贴到 joint_jog_gui.py：")
    print(f"V_UP_IN_BASE = ({v_up[0]:.4f}, {v_up[1]:.4f}, {v_up[2]:.4f})")
    print()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
