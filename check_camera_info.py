#!/usr/bin/env python3
"""对比 color / depth 的 camera_info，验证深度对齐是否生效。Ctrl+C 退出。"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class CICheck(Node):
    def __init__(self):
        super().__init__('ci_check')
        self.got = {}
        # 驱动只在 image_raw 有订阅者时才发布 camera_info，必须先订上 image
        self.create_subscription(Image, '/camera/color/image_raw', lambda m: None, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', lambda m: None, 10)
        self.create_subscription(CameraInfo, '/camera/color/camera_info',
                                 lambda m: self.cb('color', m), 10)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info',
                                 lambda m: self.cb('depth', m), 10)

    def cb(self, name, msg):
        if name in self.got:
            return
        self.got[name] = msg
        k = [round(x, 3) for x in msg.k]
        print(f'[{name}] {msg.width}x{msg.height} frame={msg.header.frame_id}')
        print(f'  fx={k[0]} fy={k[4]} cx={k[2]} cy={k[5]}')
        print(f'  d={[round(x,4) for x in msg.d]}')
        if len(self.got) == 2:
            c, d = self.got['color'], self.got['depth']
            same_k = all(abs(a - b) < 1e-3 for a, b in zip(c.k, d.k))
            print('=' * 40)
            print('K 矩阵一致' if same_k else 'K 矩阵【不一致】')
            if same_k and (c.width, c.height) == (d.width, d.height):
                print('=> 对齐生效：深度图已在彩色坐标系，可用彩色标定结果')
            else:
                print('=> 对齐未生效或分辨率不同，不能混用彩色标定结果')
            raise SystemExit


rclpy.init()
try:
    rclpy.spin(CICheck())
except SystemExit:
    pass
