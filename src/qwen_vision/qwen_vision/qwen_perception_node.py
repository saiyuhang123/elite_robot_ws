#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Qwen-VL 大模型检测 + 深度定位节点（图漾 Percipio + Elite CS66）。

设计对齐当前项目的 YOLO 感知链路：
  /camera/color/image_raw + /camera/depth/image_raw + /camera/color/camera_info
  -> 大模型输出 bbox
  -> 深度图取 bbox 中心/中值深度
  -> 相机系 3D 点
  -> hand_eye_result.json（相机 -> tool0）
  -> TF（tool0 -> base）
  -> /target_object_pose（cs66_base_link 系）

提供：
  /qwen_perception/set_enabled        SetBool 持续识别开关
  /qwen_perception/locate_object      Trigger 异步触发一次
  /qwen_perception/locate_object_sync Trigger 同步触发一次
  /qwen_perception/status             Trigger 查询状态
  /target_object_pose                 与 YOLO 感知相同的输出话题
  /qwen/annotated_image               可视化调试图
  /qwen/object_name / /qwen/description
"""

import ast
import base64
import json
import os
import re
import tempfile
import threading
import time

import cv2
import numpy as np
import requests
import rclpy
import rclpy.duration
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


DEFAULT_HAND_EYE_JSON = os.path.expanduser(
    "~/Documents/elite_robot_ws/biaoding/hand_eye_result.json")


class QwenPerceptionNode(Node):
    def __init__(self):
        super().__init__('qwen_perception_node')

        # ---------- 参数 ----------
        self.declare_parameter('model', 'qwen3.7-plus')
        self.declare_parameter('prompt', '请只定位目标物体的位置（只返回bbox）')
        self.declare_parameter('image_save_dir', tempfile.gettempdir())
        self.declare_parameter('debug', True)
        self.declare_parameter('api_key', os.getenv('DASHSCOPE_API_KEY', ''))
        self.declare_parameter('base_url', os.getenv('DASHSCOPE_BASE_URL', ''))
        self.declare_parameter('target', 'apple')
        self.declare_parameter('hand_eye_json', DEFAULT_HAND_EYE_JSON)
        self.declare_parameter('base_frame', 'cs66_base_link')
        self.declare_parameter('tool_frame', 'cs66_tool0')
        self.declare_parameter('depth_scale', 0.00025)
        self.declare_parameter('min_depth_m', 0.10)
        self.declare_parameter('max_depth_m', 3.00)
        self.declare_parameter('patch_size', 7)
        self.declare_parameter('min_valid_depth', 10)
        self.declare_parameter('use_depth_centroid', True)
        self.declare_parameter('depth_tol_m', 0.05)
        self.declare_parameter('depth_color_tol_s', 0.5)
        self.declare_parameter('poll_interval', 1.0)
        self.declare_parameter('api_timeout', 15.0)
        self.declare_parameter('locate_timeout', 20.0)
        self.declare_parameter('confidence_threshold', 0.0)
        self.declare_parameter('use_model_center', False)
        self.declare_parameter('annotated_topic', '/qwen/annotated_image')
        self.declare_parameter('position_topic', '/target_object_pose')

        self.model_name = self.get_parameter('model').value
        self.default_prompt = self.get_parameter('prompt').value
        self.image_save_dir = self.get_parameter('image_save_dir').value
        self.debug_mode = self.get_parameter('debug').value
        self.api_key = self.get_parameter('api_key').value
        self.base_url = self.get_parameter('base_url').value
        self.target_name = (self.get_parameter('target').value or '').strip()
        self.hand_eye_json = self.get_parameter('hand_eye_json').value
        self.base_frame = self.get_parameter('base_frame').value
        self.tool_frame = self.get_parameter('tool_frame').value
        self.depth_scale = float(self.get_parameter('depth_scale').value)
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.patch_size = int(self.get_parameter('patch_size').value)
        self.min_valid_depth = int(self.get_parameter('min_valid_depth').value)
        self.use_depth_centroid = bool(self.get_parameter('use_depth_centroid').value)
        self.depth_tol_m = float(self.get_parameter('depth_tol_m').value)
        self.depth_color_tol_s = float(self.get_parameter('depth_color_tol_s').value)
        self.poll_interval = max(0.5, float(self.get_parameter('poll_interval').value))
        self.api_timeout = float(self.get_parameter('api_timeout').value)
        self.locate_timeout = float(self.get_parameter('locate_timeout').value)
        self.conf_threshold = float(self.get_parameter('confidence_threshold').value)
        self.use_model_center = bool(self.get_parameter('use_model_center').value)
        self.annotated_topic = self.get_parameter('annotated_topic').value
        self.position_topic = self.get_parameter('position_topic').value

        if not self.api_key:
            self.get_logger().warn('api_key 为空，请通过参数或 DASHSCOPE_API_KEY 设置')
        if not self.base_url:
            self.get_logger().warn('base_url 为空，请通过参数或 DASHSCOPE_BASE_URL 设置')
        if not os.path.exists(self.image_save_dir):
            os.makedirs(self.image_save_dir, exist_ok=True)

        # ---------- 手眼标定（相机 -> tool0） ----------
        self.T_cam_to_tool = self._load_hand_eye_matrix(self.hand_eye_json)
        self.get_logger().info(f'已加载手眼标定: {self.hand_eye_json}')

        # ---------- TF ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.bridge = CvBridge()

        # ---------- 目标别名/同义词 ----------
        self.synonyms = {
            "苹果": "apple",
            "香蕉": "banana",
            "橙子": "orange",
            "橘子": "orange",
            "梨": "pear",
            "桃子": "peach",
            "水": "water bottle",
            "水瓶": "water bottle",
            "矿泉水": "water bottle",
            "瓶子": "bottle",
            "杯子": "cup",
            "马克杯": "cup",
        }

        # ---------- 状态 ----------
        self._state_lock = threading.Lock()
        self._frame_lock = threading.Lock()
        self._target_lock = threading.Lock()
        self._result_lock = threading.Lock()

        self._enabled = False
        self._processing = False
        self._abort_processing = False
        self._last_process_started = 0.0
        self._enable_stamp = None
        self._done_event = threading.Event()

        self.latest_color = None
        self.latest_color_msg = None
        self.latest_depth = None
        self.latest_depth_msg = None
        self.latest_info = None
        self.latest_tool_pose = None
        self._tf_fallback_logged = False
        self._received = {'color': False, 'depth': False, 'info': False, 'tf': False}

        self._last_success = False
        self._last_message = ""
        self._last_name = ""
        self._last_pose = None

        self._init_ros_interfaces()

        self.create_timer(self.poll_interval, self._timer_cb)

        self.get_logger().info('Qwen 感知节点初始化完成')
        self.get_logger().info(
            f'目标: [{self.target_name or "all"}] | 深度倍率: {self.depth_scale} m/LSB')
        self.get_logger().info('服务: /qwen_perception/set_enabled, '
                               '/qwen_perception/locate_object(_sync)')
        self.get_logger().info('输出: /target_object_pose, /qwen/annotated_image')

    # ---------------------------------------------------------------
    # 初始化
    # ---------------------------------------------------------------
    def _init_ros_interfaces(self):
        self.create_subscription(
            Image, '/camera/color/image_raw', self._color_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/camera/depth/image_raw', self._depth_cb, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._info_cb, 1)

        # 同时兼容 YOLO 抓取主程序发的 /yolo/target_class 和通用 /vision/target_class
        self.create_subscription(String, '/yolo/target_class', self._target_cb, 10)
        self.create_subscription(String, '/vision/target_class', self._target_cb, 10)

        self.pose_pub = self.create_publisher(PoseStamped, self.position_topic, 10)
        self.annotated_pub = self.create_publisher(Image, self.annotated_topic, 10)
        self.name_pub = self.create_publisher(String, '/qwen/object_name', 10)
        self.desc_pub = self.create_publisher(String, '/qwen/description', 10)
        self.done_pub = self.create_publisher(String, '/qwen/perception_done', 10)

        self.create_service(SetBool, '/qwen_perception/set_enabled', self._set_enabled_cb)
        self.create_service(Trigger, '/qwen_perception/locate_object', self._locate_cb)
        self.create_service(Trigger, '/qwen_perception/locate_object_sync', self._locate_sync_cb)
        self.create_service(Trigger, '/qwen_perception/status', self._status_cb)

    def _load_hand_eye_matrix(self, path):
        with open(path, 'r') as f:
            calib = json.load(f)
        T = np.eye(4)
        T[:3, :3] = np.array(calib['R_cam2tool'])
        T[:3, 3] = np.array(calib['t_cam2tool']).flatten()
        return T

    # ---------------------------------------------------------------
    # 回调：图像/内参/目标
    # ---------------------------------------------------------------
    def _color_cb(self, msg):
        try:
            color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'彩色图转换失败: {e}')
            return
        # 拍照时刻立刻锁存机械臂位姿，避免 API 返回后再查 TF 拿到移动后的位姿
        tool_pose = self._lookup_tool_pose(msg.header.stamp)
        with self._frame_lock:
            self.latest_color = color
            self.latest_color_msg = msg
            self.latest_tool_pose = tool_pose
        if tool_pose is not None and not self._received['tf']:
            self._received['tf'] = True
            self.get_logger().info('[诊断] 首次获取 TF（拍照时刻）')
        if not self._received['color']:
            self._received['color'] = True
            self.get_logger().info('[诊断] 首次收到彩色图')
        self._maybe_auto_process()

    def _depth_cb(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'深度图转换失败: {e}')
            return
        with self._frame_lock:
            self.latest_depth = depth
            self.latest_depth_msg = msg
        if not self._received['depth']:
            self._received['depth'] = True
            self.get_logger().info('[诊断] 首次收到深度图')

    def _info_cb(self, msg):
        with self._frame_lock:
            self.latest_info = msg
        if not self._received['info']:
            self._received['info'] = True
            self.get_logger().info('[诊断] 首次收到相机内参')

    def _target_cb(self, msg):
        text = (msg.data or '').strip()
        if not text:
            return
        with self._target_lock:
            self.target_name = text
        self.get_logger().info(f'目标已更新: {text}')

    # ---------------------------------------------------------------
    # 触发逻辑：持续识别 / 单次触发 / 同步触发
    # ---------------------------------------------------------------
    def _timer_cb(self):
        self._maybe_auto_process()

    def _maybe_auto_process(self):
        ok, _ = self._try_begin_process(triggered=False)
        if not ok:
            return

    def _try_begin_process(self, triggered):
        """尝试开启一次检测。返回 (成功, 原因)。"""
        with self._state_lock:
            if self._processing:
                return False, 'previous request is still processing'
            if not triggered and not self._enabled:
                return False, 'perception is disabled'
            if not triggered and \
                    time.monotonic() - self._last_process_started < self.poll_interval:
                return False, 'poll interval not reached'

            with self._frame_lock:
                if self.latest_color is None or self.latest_depth is None \
                        or self.latest_depth_msg is None \
                        or self.latest_info is None:
                    return False, 'camera data not available'
                color = self.latest_color
                color_msg = self.latest_color_msg
                depth = self.latest_depth
                depth_msg = self.latest_depth_msg
                info = self.latest_info
                tool_pose = self.latest_tool_pose

            if not triggered and self._enable_stamp is not None:
                try:
                    if rclpy.time.Time.from_msg(color_msg.header.stamp) <= self._enable_stamp:
                        return False, 'waiting for frame newer than enable time'
                except Exception:
                    pass

            # 深度/彩色必须来自接近同一时刻，避免旧深度配新框
            try:
                dt_s = (rclpy.time.Time.from_msg(depth_msg.header.stamp)
                        - rclpy.time.Time.from_msg(color_msg.header.stamp)).nanoseconds / 1e9
                if abs(dt_s) > self.depth_color_tol_s:
                    return False, f'depth/color timestamp mismatch {dt_s:.2f}s'
                if self.debug_mode:
                    self.get_logger().info(
                        f'[时间戳] 彩色-深度时间差 {dt_s*1000:.0f}ms')
            except Exception:
                pass

            if tool_pose is None:
                tool_pose = self._lookup_tool_pose(color_msg.header.stamp)
                with self._frame_lock:
                    self.latest_tool_pose = tool_pose
            if tool_pose is None:
                return False, 'TF not available'

            self._processing = True
            self._abort_processing = False
            self._last_process_started = time.monotonic()
            self._done_event.clear()
            self._reset_result()

        threading.Thread(
            target=self._process_worker,
            args=(color, depth, info, color_msg.header.stamp, triggered, tool_pose),
            daemon=True,
        ).start()
        return True, 'started'

    def _set_enabled_cb(self, request, response):
        with self._state_lock:
            self._enabled = bool(request.data)
            if self._enabled:
                with self._frame_lock:
                    if self.latest_color_msg is not None:
                        self._enable_stamp = rclpy.time.Time.from_msg(
                            self.latest_color_msg.header.stamp)
                    else:
                        self._enable_stamp = None
                self._last_process_started = 0.0
            else:
                # 切换后端/关闭时，取消正在进行的持续识别结果发布
                self._abort_processing = True
        response.success = True
        response.message = '识别已开启' if self._enabled else '识别已关闭'
        self.get_logger().info(f'[开关] Qwen 持续识别: {"ON" if self._enabled else "OFF"}')
        if self._enabled:
            self._maybe_auto_process()
        return response

    def _locate_cb(self, request, response):
        ok, reason = self._try_begin_process(triggered=True)
        response.success = ok
        response.message = '检测已启动' if ok else reason
        return response

    def _locate_sync_cb(self, request, response):
        ok, reason = self._try_begin_process(triggered=True)
        if not ok:
            response.success = False
            response.message = reason
            return response

        if not self._done_event.wait(self.locate_timeout):
            response.success = False
            response.message = '目标检测失败（超时）'
            return response

        with self._result_lock:
            if not self._last_success or self._last_pose is None:
                response.success = False
                response.message = self._last_message or '目标检测失败'
                return response
            p = self._last_pose.pose.position
            response.success = True
            response.message = json.dumps({
                'name': self._last_name,
                'frame_id': self._last_pose.header.frame_id,
                'x': float(p.x),
                'y': float(p.y),
                'z': float(p.z),
            }, ensure_ascii=False)
            return response

    def _status_cb(self, request, response):
        with self._state_lock:
            enabled = self._enabled
            processing = self._processing
        with self._result_lock:
            has_result = self._last_success
            last_msg = self._last_message
        if processing:
            status = '识别中'
        elif enabled:
            status = '已开启，等待帧'
        else:
            status = '已关闭'
        if has_result:
            status += f' | 最近结果: {last_msg}'
        response.success = True
        response.message = status
        return response

    # ---------------------------------------------------------------
    # 检测主流程
    # ---------------------------------------------------------------
    def _process_worker(self, color, depth, info, stamp, triggered, tool_pose):
        try:
            self._run_detection(color, depth, info, stamp, triggered, tool_pose)
        except Exception as e:
            self.get_logger().error(f'检测流程异常: {e}')
            self._publish_fail(f'检测流程异常: {e}')
        finally:
            with self._state_lock:
                self._processing = False
            with self._result_lock:
                done_status = 'success' if self._last_success else 'failed'
            self.done_pub.publish(String(data=done_status))
            self._done_event.set()

    def _run_detection(self, color, depth, info, stamp, triggered, tool_pose):
        temp_path = None
        try:
            stamp_ns = int(stamp.sec) * 10**9 + int(stamp.nanosec)
            temp_path = os.path.join(self.image_save_dir, f'qwen_capture_{stamp_ns}.png')
            if not cv2.imwrite(temp_path, color):
                raise RuntimeError('保存检测图像失败')

            with self._target_lock:
                target = self.target_name
            fh, fw = color.shape[:2]
            result = self._detect_objects_with_vision_model(
                temp_path, self.default_prompt, target, fw, fh)
            if not result or 'objects' not in result:
                self._publish_fail('大模型返回内容无法解析')
                return

            objects = result.get('objects', [])
            objects = [
                o for o in objects
                if float(o.get('confidence', 0.0)) >= self.conf_threshold
            ]
            if not objects:
                self._publish_fail(f'大模型未找到目标: {target or "all"}')
                return

            positions = self._calculate_3d_positions(color, depth, info, objects)
            if not positions:
                self._publish_fail('检测到目标但深度无效或超出范围')
                return

            selected = self._select_best_target(positions, target)
            if selected is None:
                self._publish_fail(f'目标名称匹配失败: {target}')
                return

            p_cam = np.array([
                selected['camera_position'][0],
                selected['camera_position'][1],
                selected['camera_position'][2],
                1.0,
            ])
            p_base = self._camera_to_base(p_cam, tool_pose)
            if not triggered:
                with self._state_lock:
                    if self._abort_processing:
                        self._publish_fail('检测已取消（后端切换/关闭）')
                        return
            self._publish_result(selected, p_base, stamp, positions, color)
        finally:
            if temp_path and not self.debug_mode:
                try:
                    os.remove(temp_path)
                except OSError:
                    pass

    def _publish_fail(self, detail):
        self.get_logger().warn(detail)
        self.desc_pub.publish(String(data='目标检测失败'))
        with self._result_lock:
            self._last_success = False
            self._last_message = '目标检测失败'
            self._last_name = ''
            self._last_pose = None

    def _reset_result(self):
        with self._result_lock:
            self._last_success = False
            self._last_message = ''
            self._last_name = ''
            self._last_pose = None

    # ---------------------------------------------------------------
    # 3D 计算与坐标变换
    # ---------------------------------------------------------------
    def _depth_centroid_point(self, color, depth, fx, fy, cx, cy,
                              x_min, x_max, y_min, y_max):
        """用 bbox 内“与中值深度接近”的点群质心作为目标像素。

        大模型给出的 bbox 往往比 YOLO 松，直接用几何中心会把点取到物体边缘
        或背景上。这里取 bbox 内离相机最近的深度簇（通常是物体本体，而不是
        后面的桌面/背景），再对该簇做连通域并取质心，避免被背景拉偏。
        """
        fh, fw = color.shape[:2]
        dh, dw = depth.shape[:2]
        u0 = int(round(x_min * dw / fw))
        u1 = int(round(x_max * dw / fw))
        v0 = int(round(y_min * dh / fh))
        v1 = int(round(y_max * dh / fh))
        u0 = max(0, u0)
        u1 = min(dw - 1, u1)
        v0 = max(0, v0)
        v1 = min(dh - 1, v1)
        if u1 <= u0 or v1 <= v0:
            return None

        roi = depth[v0:v1 + 1, u0:u1 + 1].astype(np.float32)
        valid = np.isfinite(roi) & (roi > 0)
        if int(valid.sum()) < self.min_valid_depth:
            return None
        ys, xs = np.nonzero(valid)
        vals_m = roi[valid] * self.depth_scale
        # 用低分位作为“前景物体”深度种子：物体比桌面/背景更靠近相机
        seed_m = float(np.percentile(vals_m, 20))
        if seed_m < self.min_depth_m or seed_m > self.max_depth_m:
            return None

        tol = max(self.depth_tol_m, seed_m * 0.05)
        sel = np.abs(vals_m - seed_m) <= tol
        if int(sel.sum()) < self.min_valid_depth:
            return None

        # 取最大连通域，排除零星噪声和隔开的背景
        mask2d = np.zeros(roi.shape, dtype=np.uint8)
        mask2d[valid] = sel.astype(np.uint8)
        num_labels, labels = cv2.connectedComponents(mask2d)
        if num_labels <= 1:
            return None
        counts = np.bincount(labels.ravel())
        largest_label = 1 + int(np.argmax(counts[1:]))
        comp_vals = vals_m[labels[valid] == largest_label]
        if int(comp_vals.size) < self.min_valid_depth:
            return None
        ys, xs = np.nonzero(labels == largest_label)

        # 注意 xs/ys 是相对 bbox 左上角的坐标，必须加回 u0/v0 再换算到彩色图
        u_ref = float(np.mean(xs) + u0) * fw / dw
        v_ref = float(np.mean(ys + v0)) * fh / dh
        z_ref = float(np.median(comp_vals))
        if z_ref < self.min_depth_m or z_ref > self.max_depth_m:
            return None

        if self.debug_mode:
            self.get_logger().info(
                f'[深度修正] bbox=({x_min},{y_min},{x_max},{y_max}) '
                f'质心=({u_ref:.1f},{v_ref:.1f}) 前景簇={int(comp_vals.size)} '
                f'seed={seed_m*1000:.1f}mm z={z_ref*1000:.1f}mm')
        return (float((u_ref - cx) * z_ref / fx),
                float((v_ref - cy) * z_ref / fy),
                float(z_ref))

    def _depth_at(self, color, depth, fx, fy, cx, cy, u, v):
        """读取深度图中 (u, v) 附近有效中值，返回相机系 3D 点或 None。"""
        fh, fw = color.shape[:2]
        dh, dw = depth.shape[:2]
        ud = int(round(u * dw / fw))
        vd = int(round(v * dh / fh))
        half = max(1, self.patch_size // 2)
        v0, v1 = max(0, vd - half), min(dh, vd + half + 1)
        u0, u1 = max(0, ud - half), min(dw, ud + half + 1)
        patch = depth[v0:v1, u0:u1].astype(np.float32)
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if valid.size < self.min_valid_depth:
            if self.debug_mode:
                self.get_logger().warn(
                    f'[深度诊断] 中心({u:.0f},{v:.0f}) 有效深度点 '
                    f'{valid.size} < {self.min_valid_depth}'
                    f'（patch 区域 v[{v0},{v1}) u[{u0},{u1})）')
            return None
        raw = float(np.median(valid))
        z = raw * self.depth_scale
        if z < self.min_depth_m or z > self.max_depth_m:
            if self.debug_mode:
                self.get_logger().warn(
                    f'[深度诊断] 中心({u:.0f},{v:.0f}) raw={raw:.1f} LSB '
                    f'-> z={z*1000:.1f}mm，超出有效范围 '
                    f'[{self.min_depth_m*1000:.0f}, {self.max_depth_m*1000:.0f}]mm')
            return None
        return (float((u - cx) * z / fx),
                float((v - cy) * z / fy),
                float(z))

    def _calculate_3d_positions(self, color, depth, info, objects):
        if info is None:
            return []
        fx, fy = info.k[0], info.k[4]
        cx, cy = info.k[2], info.k[5]
        fh, fw = color.shape[:2]
        out = []

        for obj in objects:
            bbox = obj.get('bbox')
            raw_center = obj.get('center')
            center_ok = (isinstance(raw_center, (list, tuple))
                         and len(raw_center) == 2)
            if (not bbox or len(bbox) != 4) and not (center_ok and self.use_model_center):
                continue
            if (not bbox or len(bbox) != 4) and self.use_model_center:
                # 模型只返回中心点时，给它一个小的局部 bbox 用于深度/画框
                cu = int(round(float(raw_center[0])))
                cv_ = int(round(float(raw_center[1])))
                bbox = [cu - 20, cv_ - 20, cu + 20, cv_ + 20]
            x_min, y_min, x_max, y_max = map(int, bbox)
            x_min = max(0, min(x_min, fw - 1))
            y_min = max(0, min(y_min, fh - 1))
            x_max = max(0, min(x_max, fw - 1))
            y_max = max(0, min(y_max, fh - 1))
            if x_max <= x_min or y_max <= y_min:
                continue

            # 优先用模型直接返回的中心点；没有才退回 bbox 中心
            model_center = raw_center
            if center_ok and self.use_model_center:
                try:
                    u_center = int(round(float(model_center[0])))
                    v_center = int(round(float(model_center[1])))
                    u_center = max(0, min(u_center, fw - 1))
                    v_center = max(0, min(v_center, fh - 1))
                except Exception:
                    u_center = (x_min + x_max) // 2
                    v_center = (y_min + y_max) // 2
            else:
                u_center = (x_min + x_max) // 2
                v_center = (y_min + y_max) // 2

            p_cam = None
            if self.use_depth_centroid and not (center_ok and self.use_model_center):
                p_cam = self._depth_centroid_point(
                    color, depth, fx, fy, cx, cy,
                    x_min, x_max, y_min, y_max)
            if p_cam is None:
                p_cam = self._depth_at(
                    color, depth, fx, fy, cx, cy, u_center, v_center)

            if p_cam is None:
                # 中心无效时退化为 bbox 内中值
                dh, dw = depth.shape[:2]
                u0 = int(round(x_min * dw / fw))
                u1 = int(round(x_max * dw / fw))
                v0 = int(round(y_min * dh / fh))
                v1 = int(round(y_max * dh / fh))
                roi = depth[max(0, v0):min(dh, v1 + 1),
                            max(0, u0):min(dw, u1 + 1)].astype(np.float32)
                valid = roi[np.isfinite(roi) & (roi > 0)]
                if valid.size < self.min_valid_depth:
                    if self.debug_mode:
                        self.get_logger().warn(
                            f'[深度诊断] bbox=({x_min},{y_min},{x_max},{y_max}) '
                            f'中心({u_center},{v_center}) 退化为 bbox 内取中值仍无效: '
                            f'有效深度点 {valid.size} < {self.min_valid_depth}')
                    continue
                z = float(np.median(valid)) * self.depth_scale
                if z < self.min_depth_m or z > self.max_depth_m:
                    if self.debug_mode:
                        self.get_logger().warn(
                            f'[深度诊断] bbox=({x_min},{y_min},{x_max},{y_max}) '
                            f'退化中值 raw={float(np.median(valid)):.1f} LSB '
                            f'-> z={z*1000:.1f}mm，超出有效范围 '
                            f'[{self.min_depth_m*1000:.0f}, {self.max_depth_m*1000:.0f}]mm')
                    continue
                p_cam = (float((u_center - cx) * z / fx),
                         float((v_center - cy) * z / fy),
                         float(z))

            out.append({
                'name': obj.get('name', 'unknown'),
                'confidence': float(obj.get('confidence', 0.0)),
                'bbox': [x_min, y_min, x_max, y_max],
                'target_pixel': [u_center, v_center],
                'camera_position': p_cam,
            })
        return out

    def _target_aliases(self, target):
        aliases = set()
        if not target:
            return aliases
        parts = [p.strip() for p in target.replace('，', ',').split(',') if p.strip()]
        for part in parts:
            p = part.lower()
            if p in ('all', '全部', '任意'):
                continue
            aliases.add(p)
            aliases.add(self.synonyms.get(part, '').lower())
            for cn, en in self.synonyms.items():
                if p == en:
                    aliases.add(cn.lower())
        aliases.discard('')
        return aliases

    def _select_best_target(self, positions, target):
        aliases = self._target_aliases(target)

        def dist(o):
            x, y, z = o['camera_position']
            return float(np.sqrt(x * x + y * y + z * z))

        if not aliases:
            return min(positions, key=lambda o: (dist(o), -o['confidence']))

        candidates = []
        for o in positions:
            name = (o.get('name') or '').strip().lower()
            if not name:
                continue
            if any(name == a or a in name or name in a for a in aliases):
                candidates.append(o)
        if not candidates:
            return None
        return min(candidates, key=lambda o: (dist(o), -o['confidence']))

    def _lookup_tool_pose(self, color_stamp):
        """取拍照时刻的 tool0 -> base 位姿；取不到时退回最新 TF。"""
        try:
            try:
                return self.tf_buffer.lookup_transform(
                    self.base_frame, self.tool_frame,
                    rclpy.time.Time.from_msg(color_stamp),
                    timeout=rclpy.duration.Duration(seconds=0.05))
            except Exception:
                if not self._tf_fallback_logged:
                    self._tf_fallback_logged = True
                    self.get_logger().warn(
                        '[TF诊断] 拍照时刻 TF 不可用，已退回最新 TF'
                        '（机械臂若已移动会导致坐标偏）')
                return self.tf_buffer.lookup_transform(
                    self.base_frame, self.tool_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5))
        except Exception:
            return None

    def _camera_to_base(self, p_cam, tool_pose=None):
        p_tool = self.T_cam_to_tool @ p_cam
        try:
            trans = tool_pose
            if trans is None:
                trans = self._lookup_tool_pose(self.get_clock().now().to_msg())

            t = trans.transform.translation
            q = trans.transform.rotation
            r_mat = Rot.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            t_tool_to_base = np.eye(4)
            t_tool_to_base[:3, :3] = r_mat
            t_tool_to_base[:3, 3] = [t.x, t.y, t.z]
            p_base = t_tool_to_base @ p_tool
            if not self._received['tf']:
                self._received['tf'] = True
                self.get_logger().info('[诊断] 首次获取 TF')
            return p_base[:3]
        except Exception as e:
            raise RuntimeError(
                f'未获取到 TF ({self.base_frame} -> {self.tool_frame}): {e}')

    # ---------------------------------------------------------------
    # 结果发布
    # ---------------------------------------------------------------
    def _publish_result(self, selected, p_base, stamp, positions, color_img=None):
        x, y, z = [float(v) for v in p_base]
        now = self.get_clock().now().to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = self.base_frame
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.w = 1.0
        self.pose_pub.publish(pose_msg)

        name = selected.get('name', 'unknown')
        self.name_pub.publish(String(data=name))
        cam_x, cam_y, cam_z = selected['camera_position']
        dist = float(np.sqrt(cam_x * cam_x + cam_y * cam_y + cam_z * cam_z))
        desc = f'检测到 {name}，距离约 {dist:.2f} 米'
        self.desc_pub.publish(String(data=desc))

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.base_frame
        tf_msg.child_frame_id = f'target_{name}'
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = z
        tf_msg.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf_msg)

        # 用真正参与坐标计算的那一帧画框，避免显示最新帧导致框对不上
        vis_frame = color_img
        if vis_frame is None:
            with self._frame_lock:
                vis_frame = self.latest_color
        if vis_frame is not None:
            vis = self._visualize(vis_frame, positions, selected)
            vis_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            vis_msg.header.stamp = now
            self.annotated_pub.publish(vis_msg)
            if self.debug_mode:
                stamp_ns = int(stamp.sec) * 10**9 + int(stamp.nanosec)
                save_path = os.path.join(
                    self.image_save_dir, f'annotated_{stamp_ns}.png')
                cv2.imwrite(save_path, vis)
                self.get_logger().info(f'[可视化] 已保存: {save_path}')

        with self._result_lock:
            self._last_success = True
            self._last_message = desc
            self._last_name = name
            self._last_pose = pose_msg

        self.get_logger().info(
            f'识别到 [{name}] -> 相机系: [{cam_x:.3f}, {cam_y:.3f}, {cam_z:.3f}]m '
            f'| 基座系: [{x:.3f}, {y:.3f}, {z:.3f}]m | 距离: {dist:.2f}m')

    def _visualize(self, frame, positions, selected=None):
        vis = frame.copy()
        for obj in positions:
            x_min, y_min, x_max, y_max = map(int, obj['bbox'])
            cv2.rectangle(vis, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            cam = obj['camera_position']
            label = f"{obj['name']} {cam[2]:.2f}m"
            cv2.putText(vis, label, (x_min, max(0, y_min - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        if selected is not None:
            x_min, y_min, x_max, y_max = map(int, selected['bbox'])
            cv2.rectangle(vis, (x_min, y_min), (x_max, y_max), (0, 255, 0), 3)
            cam = selected['camera_position']
            label = f"selected {cam[2]:.2f}m"
            cv2.putText(vis, label, (x_min, max(0, y_min - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            target_pixel = selected.get('target_pixel')
            if target_pixel:
                cx, cy = map(int, target_pixel)
            else:
                cx = (x_min + x_max) // 2
                cy = (y_min + y_max) // 2
            cv2.circle(vis, (cx, cy), 5, (0, 0, 255), -1)
            cv2.line(vis, (cx - 10, cy), (cx + 10, cy), (0, 0, 255), 1)
            cv2.line(vis, (cx, cy - 10), (cx, cy + 10), (0, 0, 255), 1)
        return vis

    # ---------------------------------------------------------------
    # 大模型接口
    # ---------------------------------------------------------------
    def _encode_image(self, path):
        with open(path, 'rb') as f:
            return base64.b64encode(f.read()).decode('utf-8')

    def _extract_json_obj(self, content):
        if not content:
            return None
        m = re.search(r"```(?:json)?\s*([\[\{].*?[\]\}])\s*```", content, flags=re.S)
        if m:
            content = m.group(1)
        else:
            arr_start = content.find('[')
            arr_end = content.rfind(']')
            if arr_start != -1 and arr_end != -1 and arr_start < arr_end:
                content = content[arr_start:arr_end + 1]
            elif '{' in content and '}' in content:
                content = content[content.find('{'):content.rfind('}') + 1]
        try:
            obj = json.loads(content)
            if isinstance(obj, list):
                return {'objects': obj}
            if isinstance(obj, dict):
                return obj
        except Exception:
            pass
        try:
            obj = ast.literal_eval(content)
            if isinstance(obj, list):
                return {'objects': obj}
            if isinstance(obj, dict):
                return obj
        except Exception:
            return None
        return None

    def _detect_objects_with_vision_model(self, image_path, prompt, target,
                                          img_w, img_h):
        if not self.api_key:
            self.get_logger().error('api_key 为空，无法调用大模型')
            return None
        try:
            with self._target_lock:
                target = target or self.target_name
            target_text = (target or '').replace('，', ',').strip()
            if target_text and target_text.lower() not in ('all', '全部', '任意'):
                detailed_prompt = (
                    f'请只定位图像中的目标物：{target_text}。'
                    '如果有多个该目标，请把每一个实例都返回bbox。'
                    '如果没有找到，请返回空数组。'
                    'bbox必须紧贴物体外轮廓，不能包含阴影、桌面、手或其他背景；'
                    'bbox坐标使用0~1000的归一化坐标（相对图像宽高的千分比，'
                    '例如图像正中心为[500,500]），不要输出像素坐标；'
                    '只输出严格JSON，不要输出任何解释/Markdown。'
                    'JSON格式：'
                    '{"objects":[{"name":"TARGET","bbox":[x_min,y_min,x_max,y_max],'
                    '"confidence":0.0}]} 或 {"objects":[]}'
                )
            else:
                detailed_prompt = (
                    f'{prompt}。只输出严格JSON，不要输出任何解释/Markdown。'
                    'bbox必须紧贴物体外轮廓，不能包含阴影、桌面、手或其他背景；'
                    'bbox坐标使用0~1000的归一化坐标（相对图像宽高的千分比，'
                    '例如图像正中心为[500,500]），不要输出像素坐标；'
                    '{"objects":[{"name":"object","bbox":[x_min,y_min,x_max,y_max],'
                    '"confidence":0.0}]}'
                )

            payload = {
                'model': self.model_name,
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
                                           f"data:image/png;base64,{self._encode_image(image_path)}"}},
                            {'type': 'text', 'text': detailed_prompt},
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
                headers=headers,
                json=payload,
                timeout=self.api_timeout,
            )
            resp.raise_for_status()
            content = resp.json()['choices'][0]['message']['content']
            obj = self._extract_json_obj(content)
            if not obj or 'objects' not in obj:
                self.get_logger().error(f'模型输出解析失败: {content[:200]}')
                return None
            self._normalize_model_coords(obj, img_w, img_h)
            return obj
        except Exception as e:
            self.get_logger().error(f'调用大模型失败: {e}')
            return None

    @staticmethod
    def _normalize_model_coords(obj, img_w, img_h):
        """把模型返回的 0~1000 归一化坐标换算回图像像素。

        Qwen-VL 系列返回的 bbox/center 是相对图像宽高的千分比坐标；
        若某次返回值明显超出 1000（模型偶发直接给像素坐标），则不再缩放。
        """
        for o in obj.get('objects', []):
            bbox = o.get('bbox')
            if isinstance(bbox, (list, tuple)) and len(bbox) == 4:
                vals = [float(v) for v in bbox]
                if max(vals) <= 1000.0:
                    vals = [vals[0] * img_w / 1000.0, vals[1] * img_h / 1000.0,
                            vals[2] * img_w / 1000.0, vals[3] * img_h / 1000.0]
                o['bbox'] = [int(round(v)) for v in vals]
            center = o.get('center')
            if isinstance(center, (list, tuple)) and len(center) == 2:
                cvals = [float(v) for v in center]
                if max(cvals) <= 1000.0:
                    cvals = [cvals[0] * img_w / 1000.0,
                             cvals[1] * img_h / 1000.0]
                o['center'] = [int(round(v)) for v in cvals]


def main(args=None):
    rclpy.init(args=args)
    node = QwenPerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
