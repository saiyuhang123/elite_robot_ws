#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Qwen 独立绿框识别测试（不经过机械臂/TF/手眼）。

只订阅图漾彩色图 -> 调 Qwen -> 画绿色 bbox -> 屏幕显示 + 保存图片。

用法：
  cd ~/Documents/elite_robot_ws/biaoding
  python3 qwen_box_test.py --target apple

按键：
  s  手动触发一次识别
  t  切换目标（输入英文/中文名）
  q  退出

默认每 5 秒自动识别一次，可用 --auto 0 关闭自动识别。
保存目录：/tmp/qwen_box_test/annotated_*.png
"""

import argparse
import ast
import base64
import json
import os
import re
import threading
import time

import cv2
import requests
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


CONFIG_FILE = os.path.expanduser(
    '~/Documents/elite_robot_ws/src/qwen_vision/config/qwen_vision.yaml')
SAVE_DIR = '/tmp/qwen_box_test'


class QwenBoxTest(Node):
    def __init__(self, target, auto_interval):
        super().__init__('qwen_box_test')
        self.bridge = CvBridge()
        self.target = target
        self.auto_interval = auto_interval

        with open(CONFIG_FILE) as f:
            cfg = yaml.safe_load(f)['/**']['ros__parameters']
        self.api_key = cfg.get('api_key', '')
        self.base_url = cfg.get('base_url', '')
        self.model = cfg.get('model', 'qwen3.7-plus')
        self.api_timeout = float(cfg.get('api_timeout', 60.0))

        if not self.api_key or not self.base_url:
            self.get_logger().error('config 里 api_key/base_url 为空，请检查 qwen_vision.yaml')

        os.makedirs(SAVE_DIR, exist_ok=True)

        self.lock = threading.Lock()
        self.latest = None
        self.latest_stamp = None
        self.detecting = False
        self.last_auto = 0.0
        self.result_text = '等待图像...'
        self.last_boxes = []

        self.create_subscription(
            Image, '/camera/color/image_raw', self._color_cb, 10)
        self.get_logger().info(
            f'Qwen 绿框测试启动 | 目标: {self.target} | 自动间隔: {self.auto_interval}s')

    def _color_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'图像转换失败: {e}')
            return
        with self.lock:
            self.latest = frame
            self.latest_stamp = msg.header.stamp

    def trigger_detect(self):
        if self.detecting:
            self.get_logger().info('正在识别中，忽略本次触发')
            return
        threading.Thread(target=self._detect_worker, daemon=True).start()

    def _detect_worker(self):
        with self.lock:
            if self.latest is None:
                self.result_text = '没有图像'
                return
            frame = self.latest.copy()
            stamp = self.latest_stamp

        self.detecting = True
        self.result_text = '正在调用 Qwen...'
        try:
            stamp_ns = int(stamp.sec) * 10**9 + int(stamp.nanosec)
            temp_path = os.path.join(SAVE_DIR, f'capture_{stamp_ns}.png')
            cv2.imwrite(temp_path, frame)

            h, w = frame.shape[:2]
            boxes = self._call_qwen(temp_path, w, h)
            self.last_boxes = boxes

            vis = frame.copy()
            for name, conf, bbox in boxes:
                x1, y1, x2, y2 = map(int, bbox)
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 3)
                cv2.putText(vis, f'{name} {conf:.2f}', (x1, max(0, y1 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.circle(vis, ((x1 + x2) // 2, (y1 + y2) // 2), 4,
                           (0, 0, 255), -1)

            save_path = os.path.join(SAVE_DIR, f'annotated_{stamp_ns}.png')
            cv2.imwrite(save_path, vis)
            self.result_text = (
                f'{len(boxes)} 个框 -> {os.path.basename(save_path)}')
            self.get_logger().info(
                f'Qwen 原始返回: {json.dumps(boxes, ensure_ascii=False)}')
            self.get_logger().info(f'绿框已保存: {save_path}')
        except Exception as e:
            self.result_text = f'识别失败: {e}'
            self.get_logger().error(f'识别失败: {e}')
        finally:
            self.detecting = False

    def _call_qwen(self, image_path, img_w, img_h):
        with open(image_path, 'rb') as f:
            b64 = base64.b64encode(f.read()).decode('utf-8')

        prompt = (
            f'请只定位图像中的目标物：{self.target}。'
            '如果有多个该目标，请把每一个实例都返回bbox。'
            '如果没有找到，请返回空数组。'
            'bbox必须紧贴物体外轮廓，不能包含阴影、桌面、手或其他背景。'
            'bbox坐标使用0~1000的归一化坐标（相对图像宽高的千分比，'
            '例如图像正中心为[500,500]），不要输出像素坐标。'
            '只输出严格JSON，不要输出任何解释/Markdown。'
            'JSON格式：{"objects":[{"name":"TARGET",'
            '"bbox":[x_min,y_min,x_max,y_max],"confidence":0.0}]}'
            ' 或 {"objects":[]}'
        )
        payload = {
            'model': self.model,
            'messages': [
                {
                    'role': 'system',
                    'content': [{'type': 'text', 'text':
                                 '你是机器人视觉感知系统，需要严格输出可解析JSON。'}],
                },
                {
                    'role': 'user',
                    'content': [
                        {'type': 'image_url',
                         'image_url': {'url':
                                       f'data:image/png;base64,{b64}'}},
                        {'type': 'text', 'text': prompt},
                    ],
                },
            ],
        }
        headers = {
            'Content-Type': 'application/json',
            'Authorization': f'Bearer {self.api_key}',
        }
        resp = requests.post(
            f'{self.base_url}/chat/completions',
            headers=headers, json=payload, timeout=self.api_timeout)
        resp.raise_for_status()
        content = resp.json()['choices'][0]['message']['content']
        obj = self._extract_json(content)
        boxes = []
        for o in obj.get('objects', []):
            bbox = o.get('bbox')
            if not bbox or len(bbox) != 4:
                continue
            vals = [float(v) for v in bbox]
            # Qwen 返回的是 0~1000 归一化坐标，换算回图像像素。
            # 若模型偶发直接给了像素坐标（明显超出 1000），则不再缩放。
            if max(vals) <= 1000.0:
                vals = [vals[0] * img_w / 1000.0, vals[1] * img_h / 1000.0,
                        vals[2] * img_w / 1000.0, vals[3] * img_h / 1000.0]
            boxes.append((
                o.get('name', 'unknown'),
                float(o.get('confidence', 0.0)),
                [int(round(v)) for v in vals],
            ))
        return boxes

    @staticmethod
    def _extract_json(content):
        m = re.search(r'```(?:json)?\s*([\[\{].*?[\]\}])\s*```',
                      content, flags=re.S)
        if m:
            content = m.group(1)
        else:
            start = content.find('{')
            end = content.rfind('}')
            if start != -1 and end != -1:
                content = content[start:end + 1]
        try:
            obj = json.loads(content)
        except Exception:
            try:
                obj = ast.literal_eval(content)
            except Exception:
                return {'objects': []}
        if isinstance(obj, list):
            return {'objects': obj}
        return obj if isinstance(obj, dict) else {'objects': []}

    def set_target(self, target):
        self.target = target.strip()
        self.get_logger().info(f'目标切换为: {self.target}')


def main():
    parser = argparse.ArgumentParser(description='Qwen 独立绿框识别测试')
    parser.add_argument('--target', default='apple', help='识别目标，默认 apple')
    parser.add_argument('--auto', type=float, default=5.0,
                        help='自动识别间隔秒数，0 表示只手动')
    args = parser.parse_args()

    rclpy.init()
    node = QwenBoxTest(args.target, args.auto)

    display = bool(os.environ.get('DISPLAY'))
    if display:
        try:
            cv2.namedWindow('Qwen Box Test', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Qwen Box Test', 960, 720)
        except Exception:
            display = False

    print('=' * 60)
    print('Qwen 绿框测试')
    print('  s=识别  t=切换目标  q=退出')
    print('  保存目录:', SAVE_DIR)
    print('=' * 60)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

            if args.auto > 0 and not node.detecting \
                    and time.time() - node.last_auto >= args.auto:
                node.last_auto = time.time()
                node.trigger_detect()

            with node.lock:
                latest = None if node.latest is None else node.latest.copy()
            if latest is not None and display:
                cv2.putText(latest, node.result_text, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow('Qwen Box Test', latest)

            key = cv2.waitKey(1) & 0xFF if display else -1
            if key in (ord('q'), 27):
                break
            elif key == ord('s'):
                node.trigger_detect()
            elif key == ord('t'):
                try:
                    new_target = input('输入目标名: ').strip()
                    if new_target:
                        node.set_target(new_target)
                except (EOFError, KeyboardInterrupt):
                    pass
    except KeyboardInterrupt:
        pass
    finally:
        if display:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
