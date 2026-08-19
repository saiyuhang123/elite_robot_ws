#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO-World 板面初检节点（打磨视觉 roi_mode: yolo 的第一级）.

工作方式：
  订阅彩色图（参数 color_topic），用 YOLO-World 开放词汇模型按 prompts
  检测板面。2026-08-19 现场实测：塑料膜包裹的板子在 YOLO-World 零样本下
  置信度随外观在 0.07~0.46 间剧烈波动，单靠 conf 阈值必然时有时无。
  因此引入多帧平滑：每帧取最高分框（含尺寸下限过滤），最近若干帧里
  命中达到 smooth_min_hits 帧才对外发布，发布值为命中帧的逐坐标中位数。
  发布到 /elite_polish/board_bbox（std_msgs/Float32MultiArray）：
      data = [x1, y1, x2, y2, conf, unix_timestamp_sec]
  无有效检测时不发布（C++ 侧按 yolo_bbox_max_age 超时自动回退 cluster/fixed）。

  注意：Float32MultiArray 元素为 float32，unix 时间戳(~1.7e9)会被量化到
  约百秒级精度，仅作日志参考；C++ 侧新鲜度判断以 bbox 消息到达时刻为准。

  调试图 /elite_polish/board_bbox_image 始终发布：绿框=平滑后输出，
  黄框=本帧原始检测，红字 no detection=平滑输出不可用。便于现场区分
  "相机没图"和"没检到"两种情况。

  注意：YOLO-World 的 set_classes 只能启动时调一次（运行时改触发
  ultralytics CUDA/CPU device bug），改 prompts 需重启节点。
"""

import time
from collections import deque
from statistics import median

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO


class YoloBoardDetectNode(Node):
    def __init__(self):
        super().__init__('yolo_board_detect_node')

        self.declare_parameter(
            'model_path',
            '/home/nvidia/Documents/elite_robot_ws/YOLO/yolov8x-worldv2.pt')
        # 2026-08-19 现场实测有效词组：包膜板子被识别为"纸箱/包裹"类
        self.declare_parameter('prompts',
                               'carton,box,package,board,metal plate')
        # 低阈值 + 多帧平滑：单帧阈值 0.05 放进候选，靠多帧投票滤掉偶发误检
        self.declare_parameter('conf', 0.05)
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('bbox_topic', '/elite_polish/board_bbox')
        self.declare_parameter('show_debug_image', False)
        self.declare_parameter('smooth_window', 5)      # 滑窗帧数
        self.declare_parameter('smooth_min_hits', 2)    # 窗内最少命中帧数
        self.declare_parameter('min_box_width', 60.0)   # 检测框最小宽(px)
        self.declare_parameter('min_box_height', 30.0)  # 检测框最小高(px)

        model_path = self.get_parameter('model_path').value
        self.prompts = [p.strip()
                        for p in self.get_parameter('prompts').value.split(',')
                        if p.strip()]
        self.conf = float(self.get_parameter('conf').value)
        color_topic = self.get_parameter('color_topic').value
        bbox_topic = self.get_parameter('bbox_topic').value
        self.show_debug_image = bool(self.get_parameter('show_debug_image').value)
        win = max(1, int(self.get_parameter('smooth_window').value))
        self.min_hits = max(1, int(self.get_parameter('smooth_min_hits').value))
        self.min_bw = float(self.get_parameter('min_box_width').value)
        self.min_bh = float(self.get_parameter('min_box_height').value)

        self.get_logger().info('loading YOLO-World model: %s' % model_path)
        self.model = YOLO(model_path)
        # YOLO-World 开放词汇：推理类别 = prompts（只能启动时调一次）
        self.model.set_classes(self.prompts)
        self.get_logger().info(
            'prompts: %s, conf: %.2f, smooth: %d/%d' %
            (str(self.prompts), self.conf, self.min_hits, win))

        self.bridge = CvBridge()
        self.bbox_pub = self.create_publisher(Float32MultiArray, bbox_topic, 1)
        self.debug_img_pub = None
        if self.show_debug_image:
            self.debug_img_pub = self.create_publisher(
                Image, '/elite_polish/board_bbox_image', 1)
        self._recent = deque(maxlen=win)  # 元素: None 或 (x1,y1,x2,y2,conf)
        # 队列深度 1：推理比相机帧慢时只保留最新帧，不积压
        self.sub = self.create_subscription(
            Image, color_topic, self.image_cb, 1)
        self.get_logger().info(
            'subscribing color: %s, publishing bbox: %s' % (color_topic, bbox_topic))

    def _smoothed(self):
        """最近若干帧命中框的逐坐标中位数；命中不足返回 None."""
        boxes = [b for b in self._recent if b is not None]
        if len(boxes) < self.min_hits:
            return None
        xs = list(zip(*boxes))
        return (median(xs[0]), median(xs[1]), median(xs[2]), median(xs[3]),
                median(xs[4]))

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn('cv_bridge convert failed: %s' % str(e))
            return

        results = self.model.predict(frame, conf=self.conf, verbose=False)

        # 本帧最佳框：置信度最高且尺寸过下限
        best = None  # (x1, y1, x2, y2, conf)
        if results and results[0].boxes is not None:
            for box in results[0].boxes:
                cls = int(box.cls[0])
                if cls >= len(self.prompts):
                    continue
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
                if (x2 - x1) < self.min_bw or (y2 - y1) < self.min_bh:
                    continue
                conf = float(box.conf[0])
                if best is None or conf > best[4]:
                    best = (x1, y1, x2, y2, conf)

        self._recent.append(best)
        sm = self._smoothed()

        if sm is not None:
            out = Float32MultiArray()
            out.data = [sm[0], sm[1], sm[2], sm[3], sm[4],
                        float(time.time())]
            self.bbox_pub.publish(out)

        if self.debug_img_pub is not None:
            if best is not None:
                cv2.rectangle(frame, (int(best[0]), int(best[1])),
                              (int(best[2]), int(best[3])), (0, 255, 255), 1)
            if sm is not None:
                cv2.rectangle(frame, (int(sm[0]), int(sm[1])),
                              (int(sm[2]), int(sm[3])), (0, 255, 0), 2)
                cv2.putText(frame, '%.2f' % sm[4],
                            (int(sm[0]), max(20, int(sm[1]) - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv2.putText(frame, 'no detection', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            self.debug_img_pub.publish(
                self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))


def main():
    rclpy.init()
    node = YoloBoardDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
