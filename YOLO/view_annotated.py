#!/usr/bin/env python3
"""实时显示 YOLO 检测调试图像（/yolo/annotated_image）。

用法：
  python3 view_annotated.py            # 默认话题 /yolo/annotated_image
  python3 view_annotated.py <话题名>   # 自定义话题
按 Q 或关闭窗口退出。
"""

import sys

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class AnnotatedViewer(Node):
    def __init__(self, topic):
        super().__init__('annotated_viewer')
        self.bridge = CvBridge()
        self.latest = None
        self.create_subscription(Image, topic, self.cb, qos_profile_sensor_data)
        self.get_logger().info(f'订阅 {topic}，等待图像...')

    def cb(self, msg):
        try:
            self.latest = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'图像转换失败: {e}')


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/yolo/annotated_image'
    rclpy.init()
    node = AnnotatedViewer(topic)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if node.latest is not None:
                cv2.imshow('YOLO Grasp Debug', node.latest)
                if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q'), 27):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
