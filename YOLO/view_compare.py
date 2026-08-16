#!/usr/bin/env python3
"""YOLO / Qwen 两种后端识别结果并排显示（论文插图用）。

订阅：
  左：/yolo/annotated_image
  右：/qwen/annotated_image

用法：
  python3 view_compare.py                 # 默认两个话题
  python3 view_compare.py <左话题> <右话题>

按键：
  S      保存当前并排画面为 PNG（--out 目录，默认 ./compare_shots）
  Q/Esc  退出
"""

import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

HEIGHT = 720          # 并排画面统一高度
LEFT_LABEL = 'YOLO'
RIGHT_LABEL = 'Qwen'


def resize_to_height(img, h):
    scale = h / img.shape[0]
    return cv2.resize(img, (int(img.shape[1] * scale), h))


class CompareViewer(Node):
    def __init__(self, left_topic, right_topic):
        super().__init__('compare_viewer')
        self.bridge = CvBridge()
        self.left = None
        self.right = None
        self.create_subscription(
            Image, left_topic, lambda m: self._cb(m, 'left'),
            qos_profile_sensor_data)
        self.create_subscription(
            Image, right_topic, lambda m: self._cb(m, 'right'),
            qos_profile_sensor_data)
        self.get_logger().info(
            f'订阅 {left_topic} | {right_topic}，等待图像...')

    def _cb(self, msg, side):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'图像转换失败: {e}')
            return
        if side == 'left':
            self.left = img
        else:
            self.right = img


def placeholder(label):
    img = np.zeros((HEIGHT, int(HEIGHT * 4 / 3), 3), np.uint8)
    cv2.putText(img, f'{label}: waiting...', (40, HEIGHT // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (180, 180, 180), 2)
    return img


def compose(viewer):
    left = viewer.left if viewer.left is not None else placeholder(LEFT_LABEL)
    right = viewer.right if viewer.right is not None else placeholder(RIGHT_LABEL)
    left = resize_to_height(left, HEIGHT)
    right = resize_to_height(right, HEIGHT)
    for img, label in ((left, LEFT_LABEL), (right, RIGHT_LABEL)):
        cv2.putText(img, label, (20, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 255), 3)
    # 中间 4px 白缝分隔
    sep = np.full((HEIGHT, 4, 3), 255, np.uint8)
    return np.hstack((left, sep, right))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('left', nargs='?', default='/yolo/annotated_image')
    ap.add_argument('right', nargs='?', default='/qwen/annotated_image')
    ap.add_argument('--out', default='compare_shots',
                    help='截图保存目录（默认 ./compare_shots）')
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    rclpy.init()
    node = CompareViewer(args.left, args.right)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            frame = compose(node)
            cv2.imshow('YOLO vs Qwen', frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break
            if key in (ord('s'), ord('S')):
                path = os.path.join(
                    args.out, f'compare_{time.strftime("%Y%m%d_%H%M%S")}.png')
                cv2.imwrite(path, frame)
                node.get_logger().info(f'已保存 {path}')
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
