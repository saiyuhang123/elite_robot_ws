#!/usr/bin/env python3
"""彩色/深度一致性验证：同一块 ArUco 板，solvePnP(彩色几何) vs 深度图取点。

两者是独立测量体系：
  - PnP：彩色图角点 + marker_size，不依赖深度
  - 深度：对齐深度图中心窗口中值 × 0.25mm/LSB，反投影
两者差异即 彩色↔depth 配准误差 + 深度本身误差。

前置：相机带深度启动：
  ros2 launch percipio_camera percipio_camera.launch.py \
      color_resolution:=1280x960 depth_resolution:=1280x960

判定：同一时刻两路 Z 差异 <1cm 为正常（深度本身有 mm~cm 级噪声）。
"""
import threading
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

MARKER_SIZE = 0.123
MARKER_ID = 0
DEPTH_SCALE_M = 0.25 / 1000.0  # Percipio: 0.25mm/LSB -> 米


class CD(Node):
    def __init__(self):
        super().__init__('verify_color_depth')
        self.bridge = CvBridge()
        self.K = None
        self.color = None
        self.depth = None
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        s = MARKER_SIZE / 2
        self.obj_pts = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]],
                                dtype=np.float32)
        # 驱动只在 image_raw 有订阅者时才发 camera_info，三个都要订
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.info_cb, 10)
        self.create_subscription(Image, '/camera/color/image_raw', self.color_cb, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, 10)

    def info_cb(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f'camera_info: {msg.width}x{msg.height}')

    def color_cb(self, msg):
        self.color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def depth_cb(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, '16UC1')

    def compare_once(self):
        if self.K is None or self.color is None or self.depth is None:
            print('等待 color/depth/camera_info ...')
            return
        gray = cv2.cvtColor(self.color, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict,
                                                  parameters=self.aruco_params)
        if ids is None or MARKER_ID not in ids.flatten():
            print('未检测到标记')
            return
        idx = list(ids.flatten()).index(MARKER_ID)
        ok, rvec, tvec = cv2.solvePnP(self.obj_pts, corners[idx][0], self.K,
                                      np.zeros(5), flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return
        u, v = corners[idx][0].mean(axis=0).astype(int)
        r = 5
        h, w = self.depth.shape
        win = self.depth[max(0, v - r):v + r + 1, max(0, u - r):u + r + 1].astype(np.float32)
        valid = win[win > 0]
        if valid.size < 10:
            print('标记中心深度无效')
            return
        z_d = float(np.median(valid)) * DEPTH_SCALE_M
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        p_depth = np.array([(u - cx) * z_d / fx, (v - cy) * z_d / fy, z_d])
        p_pnp = tvec.flatten()
        diff = (p_pnp - p_depth) * 1000
        print(f'PnP  : x={p_pnp[0]:+.4f} y={p_pnp[1]:+.4f} z={p_pnp[2]:.4f}')
        print(f'深度 : x={p_depth[0]:+.4f} y={p_depth[1]:+.4f} z={p_depth[2]:.4f}')
        print(f'差值 : dx={diff[0]:+.1f}mm dy={diff[1]:+.1f}mm dz={diff[2]:+.1f}mm')


def main():
    rclpy.init()
    node = CD()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    time.sleep(2.0)
    print('标定板正对/侧对相机、换距离，每按一次回车比对一次（q 退出）')
    while True:
        s = input('>> ').strip().lower()
        if s == 'q':
            break
        node.compare_once()
    import os
    os._exit(0)


if __name__ == '__main__':
    main()
