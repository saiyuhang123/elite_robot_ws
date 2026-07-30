#!/usr/bin/env python3
"""手眼标定稳定性验证（Eye-in-Hand，静态标记法）

标记固定不动，用示教器把机械臂移动到多个差异大的位姿，
每移动一次按一次回车采样；至少 5 组后按 Ctrl+C 或输入 q 回车，输出统计。

判定：标记在 base 系下各次位置的散布（max-min）
  < 5mm  优秀
  ~1cm   当前硬件正常水平
  > 2cm  标定有问题，重采

前置：
  终端1: 机械臂驱动（elite_control.launch.py）
  终端2: 相机 + aruco_single_tf.py（标记在视野内）
  终端3: ros2 launch easy_handeye2 publish.launch.py name:=elite_cs66_handeye
"""
import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener


class Verify(Node):
    def __init__(self, base_frame, marker_frame):
        super().__init__('verify_hand_eye')
        self.base_frame = base_frame
        self.marker_frame = marker_frame
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)

    def sample(self):
        t = self.buf.lookup_transform(
            self.base_frame, self.marker_frame, rclpy.time.Time(),
            timeout=Duration(seconds=2.0))
        p = t.transform.translation
        return (p.x, p.y, p.z)


def main():
    base_frame = sys.argv[1] if len(sys.argv) > 1 else 'base_link'
    marker_frame = sys.argv[2] if len(sys.argv) > 2 else 'aruco_marker_frame'
    rclpy.init()
    node = Verify(base_frame, marker_frame)
    samples = []
    print('机械臂移动到新位姿并停稳后，按回车采样（输入 q 回车结束并统计）')
    while True:
        s = input(f'[{len(samples)} 组] 回车=采样  q=统计 >> ').strip().lower()
        if s == 'q':
            break
        try:
            p = node.sample()
        except Exception as e:
            print(f'  TF 查询失败: {e}')
            continue
        samples.append(p)
        print(f'  {node.base_frame} 下标记位置: x={p[0]:.4f} y={p[1]:.4f} z={p[2]:.4f}')

    if len(samples) < 2:
        print('样本不足')
        return
    xs, ys, zs = zip(*samples)
    spread = [max(v) - min(v) for v in (xs, ys, zs)]
    mean = [sum(v) / len(v) for v in (xs, ys, zs)]
    std = [math.sqrt(sum((v - m) ** 2 for v in vals) / len(vals))
           for vals, m in ((xs, mean[0]), (ys, mean[1]), (zs, mean[2]))]
    worst = max(spread)
    print('=' * 50)
    print(f'共 {len(samples)} 组')
    print(f'散布(max-min): x={spread[0]*1000:.1f}mm y={spread[1]*1000:.1f}mm z={spread[2]*1000:.1f}mm')
    print(f'标准差:        x={std[0]*1000:.1f}mm y={std[1]*1000:.1f}mm z={std[2]*1000:.1f}mm')
    if worst < 0.005:
        print('判定: 优秀 (<5mm)')
    elif worst < 0.015:
        print('判定: 正常 (~1cm)，可用于抓取')
    else:
        print('判定: 偏差过大 (>1.5cm)，建议重标定')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
