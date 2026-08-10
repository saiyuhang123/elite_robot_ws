#!/usr/bin/env python3
"""读取 cs66_base_link -> cs66_tool0 的 TF，沿 tool z 方向偏移打磨机长度，
输出打磨头尖端在 cs66_base_link 系下的坐标，并生成 pcd_box_tool.py 验证命令。

用法:
  1. 打磨头尖端轻触标定板/工件上某个特征点，保持不动
  2. 另开终端: python3 biaoding/grinder_tip.py
  3. 复制输出的 pcd_box_tool.py 命令执行，盒内应有成片的点
"""
import sys
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation
import numpy as np

TOOL_Z = 0.183  # 打磨机(含沙盘)沿 tool z 伸出长度（米），2026-07-27 平贴实测，按实际修改


def main():
    rclpy.init()
    node = Node('grinder_tip_tool')
    buf = Buffer()
    TransformListener(buf, node)  # 不开后台线程，下面手动 spin
    # 手动 spin 让 buffer 收到 TF（最多等 3 秒）
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
        if buf.can_transform('cs66_base_link', 'cs66_tool0', rclpy.time.Time()):
            break
    try:
        tf = buf.lookup_transform('cs66_base_link', 'cs66_tool0',
                                  rclpy.time.Time(), timeout=Duration(seconds=1))
    except Exception as e:
        print(f'查不到 TF: {e}')
        rclpy.shutdown()
        sys.exit(1)

    t = tf.transform.translation
    q = tf.transform.rotation
    p_tool0 = np.array([t.x, t.y, t.z])
    R = Rotation.from_quat([q.x, q.y, q.z, q.w])
    tip = p_tool0 + R.apply([0.0, 0.0, TOOL_Z])

    print(f'tool0 位置 (base_link): {p_tool0.round(4)}')
    print(f'打磨头尖端 (base_link): {tip.round(4)}  (tool z 偏移 {TOOL_Z}m)')

    d = 0.02
    box = [tip[0] - d, tip[1] - d, tip[2] - d, tip[0] + d, tip[1] + d, tip[2] + d]
    print('\n复制执行（盒内应有成片点云，质心≈尖端坐标）:')
    print('python3 biaoding/pcd_box_tool.py ' + ' '.join(f'{v:.3f}' for v in box))

    rclpy.shutdown()


if __name__ == '__main__':
    main()
