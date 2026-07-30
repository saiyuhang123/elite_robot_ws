#!/usr/bin/env python3
"""向 /camera/dynamic_config 发送 XML 配置（在线改相机参数，无需重启）。

用法:
  python3 set_cam_feature.py "<source name='Texture'><feature name='ExposureAuto'>1</feature></source>"

可用 source: Texture(彩色) / Left(左IR) / Right(右IR) / Depth / Laser / Device
常用 feature: ExposureAuto(0/1) ExposureTime(微秒) AnalogAll(模拟增益)
"""
import sys
import time
import rclpy
from std_msgs.msg import String

if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(1)

rclpy.init()
node = rclpy.create_node('set_cam_feature')
pub = node.create_publisher(String, '/camera/dynamic_config', 10)

msg = String()
msg.data = sys.argv[1]

# 等待订阅者（相机节点）发现本发布者
for _ in range(50):
    if pub.get_subscription_count() > 0:
        break
    time.sleep(0.1)
else:
    print('警告: 没有订阅者，相机节点可能在运行吗？仍然发送。')

pub.publish(msg)
print(f'已发送: {msg.data}')
time.sleep(0.5)
node.destroy_node()
rclpy.shutdown()
