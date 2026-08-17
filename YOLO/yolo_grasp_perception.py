#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
import message_filters
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from scipy.spatial.transform import Rotation as R

import json
import os
import sys
import argparse
import time
import numpy as np
import cv2
from collections import deque
from ultralytics import YOLO

# 基座/法兰坐标系（驱动以 tf_prefix=cs66_ 启动，必须用带前缀的名字）
BASE_FRAME = 'cs66_base_link'
TOOL_FRAME = 'cs66_tool0'   # URDF 物理法兰，与手眼标定坐标系一致

# 手眼标定结果文件（与 visual_grasp_test.py 用同一份）
HAND_EYE_JSON = os.path.expanduser(
    '~/Documents/elite_robot_ws/biaoding/hand_eye_result.json')


# 默认目标类别（可通过 /yolo/target_class 话题动态修改）
# 空集合 = 处理所有类别
DEFAULT_TARGET_CLASSES = {'apple'}
# 置信度阈值：低于此值的检测框直接忽略
CONF_THRESHOLD = 0.25

# 倾斜安装实测：世界系"上"在基座系下的方向
# （与 biaoding/yolo_grasp.py 同一份，calibrate_vertical.py 可重测）。
# 二指模式估计躺倒瓶子的水平朝向时用。
V_UP_IN_BASE = np.array([-0.7431, 0.0120, 0.6691])


def _build_world_axes(v_up):
    up = np.asarray(v_up, dtype=float)
    up /= np.linalg.norm(up)
    y = np.array([0.0, 1.0, 0.0])
    y = y - (y @ up) * up
    y /= np.linalg.norm(y)
    return np.cross(y, up), y, up


WORLD_X_IN_BASE, WORLD_Y_IN_BASE, _ = _build_world_axes(V_UP_IN_BASE)

# ---- 二指模式：躺倒瓶子长轴朝向估计（离地高度掩码 + PCA）----
BOTTLE_AXIS_MIN_PIXELS = 60     # 瓶身掩码最少像素，少了不可靠
BOTTLE_AXIS_ABOVE_GROUND_M = 0.015  # 高出地面这么多(米)才算瓶身
BOTTLE_AXIS_RING_PX = 30        # bbox 外扩取样地面的环宽（像素）
BOTTLE_AXIS_MIN_ELONG = 1.8     # PCA 长/短轴标准差比，小了说明方向不可靠
BOTTLE_AXIS_STANDING_COS = 0.7  # 长轴与世界"上"的 |cos| 超过它 = 接近直立，
                                # 水平朝向无意义（抓取端回退默认朝向）

# 深度帧新鲜度：只要求是"识别开启之后新到的深度帧"。
# 拍照时机械臂已停稳，0.8fps 图漾的 color/depth 时间戳差天然很大，
# 做时间戳差校验只会把好帧丢掉（灵巧手时代也没有这道校验）。

# 灵巧手模式（--mode linkerhand，默认）：帧间一致性多帧判断 + 中心点深度，
# 与旧版一致；二指模式（--mode two_finger）用鲁棒深度 + 单帧发布。
CONSISTENT_FRAMES = 3
CONSISTENT_TOL_M = 0.015
CONSISTENT_MAX_TRIES = 4
CONSISTENT_MAX_SECONDS = 2.0

def load_hand_eye_matrix(path):
    """从 hand_eye_result.json 读取 相机->法兰 的 4x4 变换矩阵。"""
    with open(path, 'r') as f:
        calib = json.load(f)
    T = np.eye(4)
    T[:3, :3] = np.array(calib['R_cam2tool'])
    T[:3, 3] = np.array(calib['t_cam2tool']).flatten()
    return T


def estimate_bottle_axis(xyxy, depth_img, fx, fy, cx, cy, T_cam_to_base):
    """估计躺倒瓶子长轴的水平朝向（二指模式用）。

    做法：bbox 外环带像素反投影到基座系，取世界"上"方向投影的中位数
    作当地地面高度（用 3D 高度而非深度，天然免疫透视坡度）；bbox 内
    高出地面一截的像素作瓶身掩码 → PCA 第一主成分 = 瓶子长轴 →
    投影到世界水平面得朝向。瓶身完全缺深度时掩码为空返回 None，
    不会给出假象方向。

    参数:
      xyxy    目标检测框（像素）
      depth_img 深度图（原始值，0.25mm/LSB）
      T_cam_to_base 相机→基座 4x4 变换

    返回 (quat, yaw_deg, elong, axis_h)：
      quat    物体姿态四元数 (x,y,z,w)：R_obj 的 x=瓶子长轴(水平化)、
              z=世界"上"、y=z×x；抓取端只取 x 列算 yaw。
      yaw_deg 长轴在世界水平面内的角度（日志/画图用）
      elong   掩码长/短轴比（日志用）
      axis_h  水平化后的长轴单位向量（基座系，画图用）
    无法可靠估计（深度太少/掩码太圆/接近直立）返回全 None。
    """
    x1, y1, x2, y2 = [int(v) for v in xyxy]
    x1 = max(0, x1)
    y1 = max(0, y1)
    x2 = min(depth_img.shape[1], x2)
    y2 = min(depth_img.shape[0], y2)
    if x2 - x1 < 5 or y2 - y1 < 5:
        return None, None, None, None
    sub = depth_img[y1:y2, x1:x2].astype(np.float64) * 0.25 / 1000.0
    if int((sub > 0).sum()) < BOTTLE_AXIS_MIN_PIXELS:
        return None, None, None, None

    up = V_UP_IN_BASE / np.linalg.norm(V_UP_IN_BASE)
    R_cb = T_cam_to_base[:3, :3]
    t_cb = T_cam_to_base[:3, 3]

    def _heights(xa, ya, xb, yb):
        """区域有效像素反投影到基座系，返回世界"上"方向高度。"""
        roi = depth_img[ya:yb, xa:xb].astype(np.float64) * 0.25 / 1000.0
        ys, xs = np.nonzero(roi > 0)
        if ys.size == 0:
            return None
        Z = roi[roi > 0]
        u = xs.astype(np.float64) + xa
        v = ys.astype(np.float64) + ya
        pts = np.column_stack([(u - cx) * Z / fx,
                               (v - cy) * Z / fy, Z])
        return (pts @ R_cb.T + t_cb) @ up

    # 1. bbox 外环带 → 当地地面高度（中位数，抗零散干扰）
    r = BOTTLE_AXIS_RING_PX
    rx1, ry1 = max(0, x1 - r), max(0, y1 - r)
    rx2, ry2 = min(depth_img.shape[1], x2 + r), min(depth_img.shape[0], y2 + r)
    ring_parts = []
    if ry1 < y1:
        ring_parts.append(_heights(rx1, ry1, rx2, y1))     # 上
    if ry2 > y2:
        ring_parts.append(_heights(rx1, y2, rx2, ry2))     # 下
    if rx1 < x1:
        ring_parts.append(_heights(rx1, ry1, x1, ry2))     # 左
    if rx2 > x2:
        ring_parts.append(_heights(x2, ry1, rx2, ry2))     # 右
    ring_parts = [h for h in ring_parts if h is not None]
    if not ring_parts:
        return None, None, None, None
    h_ring = np.concatenate(ring_parts)
    if h_ring.size < BOTTLE_AXIS_MIN_PIXELS:
        return None, None, None, None
    h_ground = float(np.median(h_ring))

    # 2. bbox 内高出地面的像素 = 瓶身掩码
    ys, xs = np.nonzero(sub > 0)
    Z = sub[sub > 0]
    u = xs.astype(np.float64) + x1
    v = ys.astype(np.float64) + y1
    pts = np.column_stack([(u - cx) * Z / fx,
                           (v - cy) * Z / fy, Z])
    pts = pts @ R_cb.T + t_cb                    # 基座系
    keep = (pts @ up) > h_ground + BOTTLE_AXIS_ABOVE_GROUND_M
    n = int(keep.sum())
    if n < BOTTLE_AXIS_MIN_PIXELS:
        return None, None, None, None
    pts = pts[keep]

    # 3. PCA 第一主成分 = 瓶子长轴
    pts = pts - pts.mean(axis=0)
    eigvals, eigvecs = np.linalg.eigh(pts.T @ pts / n)   # 特征值升序
    if eigvals[1] <= 1e-12:
        return None, None, None, None
    elong = float(np.sqrt(eigvals[2] / eigvals[1]))
    if elong < BOTTLE_AXIS_MIN_ELONG:
        return None, None, None, None
    axis = eigvecs[:, 2]            # 瓶子长轴（基座系）

    if abs(float(axis @ up)) > BOTTLE_AXIS_STANDING_COS:
        return None, None, None, None   # 接近直立：水平朝向无意义
    axis_h = axis - (axis @ up) * up
    axis_h /= np.linalg.norm(axis_h)
    if float(axis_h @ WORLD_X_IN_BASE) < 0:
        axis_h = -axis_h            # 长轴有 180° 歧义，定号统一
    z = up
    x = axis_h
    y = np.cross(z, x)
    quat = R.from_matrix(np.column_stack([x, y, z])).as_quat()
    yaw_deg = float(np.degrees(np.arctan2(axis_h @ WORLD_Y_IN_BASE,
                                          axis_h @ WORLD_X_IN_BASE)))
    return quat, yaw_deg, elong, axis_h


class YoloGraspPerceptionNode(Node):
    def __init__(self, initial_classes=None, mode='linkerhand'):
        super().__init__('yolo_grasp_perception_node')
        self.get_logger().info('>>> YOLO 抓取感知节点正在启动...')
        self.mode = mode

        # ---------------- 1. 初始化参数与配置 ----------------
        self.bridge = CvBridge()

        # 目标类别集合（可通过 /yolo/target_class 话题动态修改）
        self.target_classes = set(
            initial_classes if initial_classes is not None
            else DEFAULT_TARGET_CLASSES)

        # 加载 YOLO 模型（YOLO-World 世界模型，开放词汇；可换成你自己的 pt 模型）
        self.get_logger().info('正在加载 YOLO 模型...')
        self.yolo_model = YOLO('yolov8x-worldv2.pt')

        # 保存模型默认类别（世界模型的默认 COCO 80 类），供“全部类别”时恢复
        self._default_world_classes = list(self.yolo_model.names.values())
        # 把目标类别作为世界模型提示词（开放词汇，只检测这些类别）
        self._apply_world_classes()

        # 启动时预热一次推理（CUDA 初始化），避免首次开启识别时卡顿 30~60s
        self.get_logger().info('预热推理中（CUDA 初始化）...')
        try:
            self.yolo_model(np.zeros((480, 640, 3), dtype=np.uint8), verbose=False)
            self.get_logger().info('预热完成')
        except Exception as e:
            self.get_logger().warn(f'预热推理失败（不影响使用）: {e}')

        # 手眼标定矩阵 (Camera -> Tool/Flange)，从 hand_eye_result.json 读取
        try:
            self.T_cam_to_tool = load_hand_eye_matrix(HAND_EYE_JSON)
            self.get_logger().info(f'已加载手眼标定: {HAND_EYE_JSON}')
        except Exception as e:
            self.get_logger().error(f'无法加载手眼标定文件 {HAND_EYE_JSON}: {e}')
            raise

        # ---------------- 2. TF 监听器 (用来获取末端法兰盘的实时位姿) ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- 3. 话题订阅 ----------------
        # 不用 message_filters 时间同步器。二指模式下自行做单向严格配对：
        # 只允许 depth_stamp >= color_stamp；彩色先到时缓存，等深度回调再处理。
        # 灵巧手保留原来的“彩色到达时取最新深度”行为，避免改变既有流程。
        self._received = {'color': False, 'depth': False, 'info': False, 'tf': False}
        self.latest_color = None
        self.latest_depth = None
        self.latest_info = None
        self._pending_color = None          # 二指：等待同时间或更新深度的彩色帧
        # 深度新鲜度/帧间一致性状态
        self._depth_stamp_at_enable = None  # 开启识别时刻缓存的深度帧时间戳
        self._depth_rejects_old = 0          # 因"识别开启前旧深度"弃帧计数
        self._depth_rejects_before_color = 0 # 二指：深度时间戳早于彩色
        self._last_reject_log = 0.0
        # 灵巧手模式：帧间一致性窗口状态
        self._pose_window = deque(maxlen=5)
        self._pose_cls = None
        self._pose_tries = 0
        self._pose_start = None
        self.create_subscription(
            Image, '/camera/color/image_raw', self._color_cb, 10)
        self.create_subscription(
            Image, '/camera/depth/image_raw', self._depth_cb, 10)
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._info_cb, 10)
        self.create_timer(5.0, self._report_status)

        # 订阅目标类别切换话题（可动态切换检测目标，如 "apple" -> "cup"）
        self.create_subscription(
            String, '/yolo/target_class', self._target_class_cb, 10)

        # 按需识别开关（默认关闭）：
        # 关闭时只缓存图像帧、不推理不发布 —— 省算力，也防止导航/摆臂
        # 过程中的旧检测结果污染抓取。抓取主程序在预备位姿停稳后通过
        # /yolo_perception/set_enabled 开启，锁存目标后关闭。
        self.enabled = False
        self.create_service(
            SetBool, '/yolo_perception/set_enabled', self._set_enabled_cb)

        # ---------------- 4. 话题发布与 TF 广播 ----------------
        # 发布算出的目标 3D 位姿 (基座坐标系下)
        self.pose_pub = self.create_publisher(PoseStamped, '/target_object_pose', 10)
        # 发布可视化画框后的图像 (方便 debug 查看)
        self.annotated_img_pub = self.create_publisher(Image, '/yolo/annotated_image', 10)
        # TF 广播器 (用来在 RViz2 里画出目标坐标轴)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('>>> 感知节点初始化完成，正在等待图像与 TF 数据...')

    # ---- 话题回调（缓存最新帧） ----
    def _color_cb(self, msg):
        self._mark('color')
        self.latest_color = msg
        if not self.enabled:
            return

        if self.mode == 'two_finger':
            # 0.8Hz 相机可能出现彩色回调先于同帧深度回调。不能拿上一帧
            # 深度配当前彩色；先缓存彩色，若深度未就绪则由 _depth_cb 重试。
            self._pending_color = msg
            self._try_process_pending_color()
            return

        # 灵巧手保留原逻辑：彩色图到达时使用最新且为识别开启后到达的深度。
        if (self.latest_depth is not None and self.latest_info is not None
                and self._depth_is_fresh(self.latest_depth, msg)):
            self._run_perception(msg, self.latest_depth, self.latest_info)

    def _depth_cb(self, msg):
        self._mark('depth')
        self.latest_depth = msg
        # 二指模式：彩色先到时，等到同时间戳或更新的深度帧再触发推理。
        if self.enabled and self.mode == 'two_finger':
            self._try_process_pending_color()

    def _info_cb(self, msg):
        self._mark('info')
        self.latest_info = msg

    def _run_perception(self, color_msg, depth_msg, info_msg):
        """统一执行感知并记录异常，避免彩色/深度回调重复异常处理代码。"""
        try:
            self.perception_callback(color_msg, depth_msg, info_msg)
        except Exception as e:
            import traceback
            self.get_logger().error(
                f'[诊断] 感知回调异常: {e}\n{traceback.format_exc()}')

    def _try_process_pending_color(self):
        """二指模式严格配对：深度时间戳不得早于待处理彩色帧。"""
        color_msg = self._pending_color
        if (not self.enabled or color_msg is None
                or self.latest_depth is None or self.latest_info is None):
            return False
        if not self._depth_is_fresh(self.latest_depth, color_msg):
            return False

        # 先清空，防止本次推理异常时重复处理同一彩色帧。
        self._pending_color = None
        self._run_perception(color_msg, self.latest_depth, self.latest_info)
        return True

    def _depth_is_fresh(self, depth_msg, color_msg):
        """深度帧新鲜度：识别开启前缓存（可能是停稳前/运动途中）的深度帧
        一律不用；二指还要求深度时间戳不早于当前彩色时间戳。"""
        depth_t = rclpy.time.Time.from_msg(depth_msg.header.stamp)
        if self._depth_stamp_at_enable is not None:
            if depth_t <= self._depth_stamp_at_enable:
                self._depth_rejects_old += 1
                self._log_depth_rejects()
                return False
        if self.mode == 'two_finger':
            color_t = rclpy.time.Time.from_msg(color_msg.header.stamp)
            if depth_t < color_t:
                self._depth_rejects_before_color += 1
                self._log_depth_rejects()
                return False
        return True

    def _log_depth_rejects(self):
        """定期打印深度弃帧计数，方便判断是不是校验把正常帧挡掉了。"""
        if time.time() - self._last_reject_log < 5.0:
            return
        self._last_reject_log = time.time()
        self.get_logger().warn(
            f'[诊断] 深度帧被弃用: 旧帧×{self._depth_rejects_old}, '
            f'早于彩色×{self._depth_rejects_before_color}')

    def _consistency_gate(self, P_base, cls_name):
        """灵巧手模式：帧间一致性。同一类别连续 CONSISTENT_FRAMES 帧位置
        一致才返回；连续 CONSISTENT_MAX_TRIES 帧或超 CONSISTENT_MAX_SECONDS
        仍不一致时按最新帧兜底（与旧版一致）。"""
        if self._pose_cls != cls_name:
            self._pose_cls = cls_name
            self._pose_window.clear()
            self._pose_tries = 0
            self._pose_start = time.time()
        self._pose_tries += 1
        self._pose_window.append(P_base[:3])

        if len(self._pose_window) >= CONSISTENT_FRAMES:
            arr = np.array(self._pose_window)
            spread = float(np.max(arr.max(axis=0) - arr.min(axis=0)))
            if spread <= CONSISTENT_TOL_M:
                return P_base[:3]
            elapsed = time.time() - (self._pose_start or time.time())
            if (self._pose_tries >= CONSISTENT_MAX_TRIES
                    or elapsed >= CONSISTENT_MAX_SECONDS):
                self.get_logger().warn(
                    f'[诊断] {self._pose_tries}帧/'
                    f'{elapsed:.1f}s 位置不一致(跨度{spread*1000:.1f}mm)，'
                    f'按最新帧发布')
                return P_base[:3]
        return None

    # ---- 按需识别开关服务 ----
    def _set_enabled_cb(self, request, response):
        self.enabled = bool(request.data)
        # 每次识别会话都从新的彩色帧开始，禁止沿用上次待配对的图像。
        self._pending_color = None
        if self.enabled:
            # 记录开启时刻已缓存的深度帧；更早的一律视为停稳前旧帧
            self._depth_stamp_at_enable = (
                rclpy.time.Time.from_msg(self.latest_depth.header.stamp)
                if self.latest_depth is not None else None)
            # 灵巧手模式：一致性窗口清零，防止沿用上一次识别会话的旧帧
            self._pose_window.clear()
            self._pose_cls = None
            self._pose_tries = 0
            self._pose_start = None
        response.success = True
        response.message = '识别已开启' if self.enabled else '识别已关闭'
        self.get_logger().info(
            f'[开关] 按需识别: {"ON" if self.enabled else "OFF"}')
        return response

    # ---- 目标类别切换回调 ----
    def _target_class_cb(self, msg):
        """动态切换检测目标类别。发送逗号分隔的类别名，如 "apple,cup" 或 "apple"。
           发送空字符串或 "all" 表示检测所有类别。"""
        text = msg.data.strip()
        if not text or text.lower() == 'all':
            self.target_classes = set()
            self.get_logger().info('[目标类别] 已切换为: 全部类别')
        else:
            self.target_classes = {c.strip() for c in text.split(',') if c.strip()}
            self.get_logger().info(f'[目标类别] 已切换为: {self.target_classes}')
        # 同步更新世界模型提示词，否则模型还在用旧的类别检测
        self._apply_world_classes()

    def _apply_world_classes(self):
        """把当前目标类别同步给 YOLO-World 的提示词。
        空集合（全部类别）= 恢复模型默认的 COCO 全部类别。"""
        try:
            if self.target_classes:
                classes = sorted(self.target_classes)
                log_text = f'提示词已设置为: {classes}'
            else:
                classes = list(self._default_world_classes)
                log_text = '提示词已恢复为默认全部类别'
            world = self.yolo_model.model
            # 预热后 YOLO 会整体搬到 GPU，缓存的 CLIP 权重跟着搬但 device 属性
            # 不会更新，导致文本在 CPU、权重在 GPU 的设备不一致。每次先丢弃重建，
            # 让 CLIP 按 YOLO 当前设备重新加载。
            if hasattr(world, 'clip_model'):
                del world.clip_model
            self.yolo_model.set_classes(classes)
            # 提示词嵌入与 YOLO 模型保持同一设备
            world.txt_feats = world.txt_feats.to(next(world.parameters()).device)
            self.get_logger().info(f'[世界模型] {log_text}')
        except Exception as e:
            self.get_logger().warn(
                f'[世界模型] 设置提示词失败（缺少 CLIP 依赖，使用模型默认类别）: {e}')

    # ---- 诊断辅助 ----
    def _mark(self, key):
        if not self._received[key]:
            self._received[key] = True
            self.get_logger().info(f'[诊断] 首次收到话题: {key}')

    def _report_status(self):
        # TF 状态
        try:
            self.tf_buffer.lookup_transform(
                BASE_FRAME, TOOL_FRAME, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            self._received['tf'] = True
        except Exception:
            self._received['tf'] = False
        missing = [k for k, v in self._received.items() if not v]
        if missing:
            self.get_logger().warn(f'[诊断] 仍缺: {missing}（同步器凑不齐就不会触发）')
        else:
            self.get_logger().info('[诊断] color/depth/info/TF 全部就绪')

    def perception_callback(self, color_msg, depth_msg, info_msg):
        import time as _time
        _t0 = _time.time()
        # 1. ROS 图像消息转换为 OpenCV 格式
        try:
            color_img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return

        # 2. 从 CameraInfo 提取相机内参
        fx = info_msg.k[0]
        fy = info_msg.k[4]
        cx = info_msg.k[2]
        cy = info_msg.k[5]

        # 3. YOLO 模型推理
        if not hasattr(self, '_infer_logged'):
            self.get_logger().info('[诊断] 首次推理开始（Jetson 首次 CUDA 推理可能要 30~60s）...')
        try:
            results = self.yolo_model(color_img, verbose=False)[0]
        except Exception as e:
            self.get_logger().error(f'[诊断] 推理异常: {e}')
            return
        if not hasattr(self, '_infer_logged'):
            self._infer_logged = True
            self.get_logger().info(f'[诊断] 首次推理完成，耗时 {_time.time()-_t0:.1f}s，'
                                   f'检出 {len(results.boxes)} 个框')
        elif int(_t0) % 10 < 1:
            self.get_logger().info(f'[诊断] 推理中... 本帧检出 {len(results.boxes)} 个框')

        # 如果没有检测到任何物体，发布原图并返回
        if len(results.boxes) == 0:
            self.annotated_img_pub.publish(self.bridge.cv2_to_imgmsg(color_img, encoding='bgr8'))
            return

        # 4. 类别/置信度过滤，多个目标取深度最近的一个
        candidates = []
        all_detections = []  # 调试：记录所有原始检测
        for box in results.boxes:
            cls_id = int(box.cls[0])
            cls_name = self.yolo_model.names[cls_id]
            conf = float(box.conf[0])
            all_detections.append(f'{cls_name}({conf:.2f})')
            if self.target_classes and cls_name not in self.target_classes:
                continue
            if conf < CONF_THRESHOLD:
                continue
            xyxy = box.xyxy[0].cpu().numpy().astype(int)
            candidates.append((xyxy, cls_name, conf))

        # 定期打印所有原始检测（帮助排查换环境后识别问题）
        if int(_time.time()) % 5 == 0 and not hasattr(self, '_last_diag_time'):
            self._last_diag_time = int(_time.time())
            self.get_logger().info(
                f'[诊断] 原始检测({len(all_detections)}个): {", ".join(all_detections[:10])}'
                f'{"..." if len(all_detections) > 10 else ""}'
                f' | 阈值={CONF_THRESHOLD} 候选={len(candidates)}')
        elif abs(int(_time.time()) - getattr(self, '_last_diag_time', 0)) >= 5:
            self._last_diag_time = int(_time.time())
            self.get_logger().info(
                f'[诊断] 原始检测({len(all_detections)}个): {", ".join(all_detections[:10])}'
                f'{"..." if len(all_detections) > 10 else ""}'
                f' | 阈值={CONF_THRESHOLD} 候选={len(candidates)}')

        if not candidates:
            # 没有目标类别的检测框：画出所有检测框（灰色）提示用户
            for box in results.boxes:
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                name = self.yolo_model.names[int(box.cls[0])]
                cv2.rectangle(color_img, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (128, 128, 128), 1)
                cv2.putText(color_img, name, (xyxy[0], xyxy[1] - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)
            self.annotated_img_pub.publish(self.bridge.cv2_to_imgmsg(color_img, encoding='bgr8'))
            return

        # 5. 遍历候选目标，取深度最近的一个
        best = None  # (Z_c, xyxy, cls_name, conf, X_c, Y_c)
        for xyxy, cls_name, conf in candidates:
            u_center = int((xyxy[0] + xyxy[2]) / 2)
            v_center = int((xyxy[1] + xyxy[3]) / 2)

            if self.mode == 'two_finger':
                # ---------------- 二指：鲁棒深度提取 ----------------
                # 深色/反光塑料瓶中心常缺深度（0），但瓶身边缘通常有值。
                # 取检测框中央偏下区域（60%宽 × 35%~95%高）的有效深度，
                # 用低分位(15%)选"最近的瓶子表面"，背景/地面更远不会被选中；
                # 再用这些最近像素的质心做投影点。
                bx1, by1, bx2, by2 = [int(v) for v in xyxy]
                bw = bx2 - bx1
                bh = by2 - by1
                r_x1 = max(0, int(bx1 + 0.20 * bw))
                r_x2 = min(depth_img.shape[1], int(bx2 - 0.20 * bw))
                r_y1 = max(0, int(by1 + 0.35 * bh))
                r_y2 = min(depth_img.shape[0], int(by2 - 0.05 * bh))

                u0, v0 = r_x1, r_y1
                roi = depth_img[v0:r_y2, u0:r_x2]
                valid_depths = roi[roi > 0]

                if valid_depths.size < 3:
                    # 回退：中心 7x7
                    u0 = max(0, u_center - 3)
                    v0 = max(0, v_center - 3)
                    roi = depth_img[v0:min(depth_img.shape[0], v_center + 4),
                                    u0:min(depth_img.shape[1], u_center + 4)]
                    valid_depths = roi[roi > 0]

                if valid_depths.size == 0:
                    self.get_logger().warn(
                        f'目标 [{cls_name}] 深度无效，跳过...')
                    continue

                # 低分位选最近表面
                thr = float(np.percentile(valid_depths, 15))
                mask = (roi > 0) & (roi <= thr)
                ys, xs = np.nonzero(mask)
                if xs.size > 0:
                    u_use = u0 + float(np.mean(xs))
                    v_use = v0 + float(np.mean(ys))
                    depth_raw = float(np.median(roi[mask]))
                else:
                    u_use, v_use = float(u_center), float(v_center)
                    depth_raw = float(np.percentile(valid_depths, 15))
            else:
                # ---------------- 灵巧手：中心 7x7 中位数（旧方案） ----------------
                v_start = max(0, v_center - 3)
                v_end = min(depth_img.shape[0], v_center + 4)
                u_start = max(0, u_center - 3)
                u_end = min(depth_img.shape[1], u_center + 4)
                depth_patch = depth_img[v_start:v_end, u_start:u_end]
                valid_depths = depth_patch[depth_patch > 0]

                if len(valid_depths) == 0:
                    self.get_logger().warn(
                        f'目标 [{cls_name}] 中心点深度无效，跳过...')
                    continue
                depth_raw = float(np.median(valid_depths))
                u_use, v_use = float(u_center), float(v_center)

            # Percipio 原始值 0.25mm/LSB -> 转换为 m
            Z_c = depth_raw * 0.25 / 1000.0

            # 过滤不合理的深度值 (比如小于10cm 或 大于 3m)
            if Z_c < 0.1 or Z_c > 3.0:
                continue

            # ---------------- 反推相机坐标系 3D 坐标 (X_c, Y_c, Z_c) ----------------
            X_c = (u_use - cx) * Z_c / fx
            Y_c = (v_use - cy) * Z_c / fy

            if best is None or Z_c < best[0]:
                best = (Z_c, xyxy, cls_name, conf, X_c, Y_c,
                        int(u_use), int(v_use))

        if best is None:
            self.annotated_img_pub.publish(self.bridge.cv2_to_imgmsg(color_img, encoding='bgr8'))
            return

        Z_c, xyxy, cls_name, conf, X_c, Y_c, u_center, v_center = best
        P_cam = np.array([X_c, Y_c, Z_c, 1.0])

        # ---------------- 利用手眼标定转换到 法兰盘坐标系 (P_tool) ----------------
        P_tool = self.T_cam_to_tool @ P_cam

        # ---------------- 结合机械臂实时 TF 转到 基座坐标系 (P_base) ----------------
        try:
            # 优先用图像拍摄时刻的 TF（机械臂运动时更准）；
            # 驱动 TF 时间戳落后于相机时钟（本机存在固定偏差）时退回最新 TF
            try:
                trans = self.tf_buffer.lookup_transform(
                    BASE_FRAME,
                    TOOL_FRAME,
                    rclpy.time.Time.from_msg(color_msg.header.stamp),
                    timeout=rclpy.duration.Duration(seconds=0.05)
                )
            except Exception:
                trans = self.tf_buffer.lookup_transform(
                    BASE_FRAME,
                    TOOL_FRAME,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )

            # 提取平移向量和四元数
            t_vec = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            q_vec = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]

            # 四元数转换为 3x3 旋转矩阵
            r_mat = R.from_quat(q_vec).as_matrix()

            # 拼合 4x4 T_tool_to_base 变换矩阵
            T_tool_to_base = np.eye(4)
            T_tool_to_base[:3, :3] = r_mat
            T_tool_to_base[:3, 3] = t_vec

            # 最终算出基座坐标系下的目标 3D 位置 P_base
            P_base = T_tool_to_base @ P_tool
            X_b, Y_b, Z_b = float(P_base[0]), float(P_base[1]), float(P_base[2])

        except Exception as e:
            self.get_logger().warn(f'未获取到 TF ({BASE_FRAME} -> {TOOL_FRAME})，请确认机械臂驱动是否启动: {e}')
            self.annotated_img_pub.publish(self.bridge.cv2_to_imgmsg(color_img, encoding='bgr8'))
            return

        # 二指模式：估计躺倒瓶子的水平朝向（长轴 PCA），随姿态四元数发布。
        # 估计失败/接近直立时 bottle_quat=None，姿态发 identity（旧行为）。
        bottle_quat = None
        axis_h = None
        if self.mode == 'two_finger':
            T_cam_to_base = T_tool_to_base @ self.T_cam_to_tool
            bottle_quat, yaw_deg, elong, axis_h = estimate_bottle_axis(
                xyxy, depth_img, fx, fy, cx, cy, T_cam_to_base)
            if bottle_quat is not None:
                self.get_logger().info(
                    f'瓶子长轴朝向: 世界水平面 {yaw_deg:.1f}°，'
                    f'长/短轴比 {elong:.1f}')

        self.get_logger().info(
            f'识别到 [{cls_name}] (可信度:{conf:.2f}) -> '
            f'相机系: [{X_c:.3f}, {Y_c:.3f}, {Z_c:.3f}]m | '
            f'基座系: [{X_b:.3f}, {Y_b:.3f}, {Z_b:.3f}]m'
        )

        # ---------------- 发布策略 ----------------
        # 二指：拍照时机械臂已停稳，单帧识别 + 有效深度直接发布
        #       （0.8fps 慢相机下多帧窗口会被坏帧污染）；
        # 灵巧手：恢复旧版多帧一致性判断（_consistency_gate）。
        if self.mode == 'two_finger':
            gate_pos = P_base[:3]
        else:
            gate_pos = self._consistency_gate(P_base, cls_name)

        # ---------------- 图像上绘制调试信息 ----------------
        cv2.rectangle(color_img, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (0, 255, 0), 2)
        cv2.circle(color_img, (u_center, v_center), 5, (0, 0, 255), -1)
        if axis_h is not None:
            # 画出估计的瓶子长轴（基座系中心 ±6cm 两端点反投影回图像）
            T_base_to_cam = np.linalg.inv(T_cam_to_base)
            pts_px = []
            for s in (-0.06, 0.06):
                p_cam = T_base_to_cam @ np.array(
                    [X_b + s * axis_h[0], Y_b + s * axis_h[1],
                     Z_b + s * axis_h[2], 1.0])
                if p_cam[2] > 0.01:
                    pts_px.append((int(fx * p_cam[0] / p_cam[2] + cx),
                                   int(fy * p_cam[1] / p_cam[2] + cy)))
            if len(pts_px) == 2:
                cv2.line(color_img, pts_px[0], pts_px[1], (0, 255, 255), 2)
        if gate_pos is not None:
            label = (f'{cls_name}: Base[{gate_pos[0]:.2f}, '
                     f'{gate_pos[1]:.2f}, {gate_pos[2]:.2f}]m')
        else:
            label = (f'{cls_name}: raw[{X_b:.3f},{Y_b:.3f},{Z_b:.3f}] '
                     f'wait')
        cv2.putText(color_img, label, (xyxy[0], xyxy[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 发布画框调试后的图像
        self.annotated_img_pub.publish(self.bridge.cv2_to_imgmsg(color_img, encoding='bgr8'))

        # 灵巧手：未达成一致时不发布位姿，等下一帧
        if gate_pos is None:
            return

        X_b, Y_b, Z_b = gate_pos
        # ---------------- 发布 PoseStamped 消息与 TF 广播 ----------------
        now = self.get_clock().now().to_msg()

        # 发布 Pose 消息（姿态：二指估计出的瓶子长轴朝向；无估计时 identity）
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = BASE_FRAME
        pose_msg.pose.position.x = X_b
        pose_msg.pose.position.y = Y_b
        pose_msg.pose.position.z = Z_b
        if bottle_quat is not None:
            pose_msg.pose.orientation.x = float(bottle_quat[0])
            pose_msg.pose.orientation.y = float(bottle_quat[1])
            pose_msg.pose.orientation.z = float(bottle_quat[2])
            pose_msg.pose.orientation.w = float(bottle_quat[3])
        else:
            pose_msg.pose.orientation.w = 1.0  # 无朝向信息（旧行为）
        self.pose_pub.publish(pose_msg)

        # 广播 TF Transform (方便在 RViz2 里可视化)
        t_tf = TransformStamped()
        t_tf.header.stamp = now
        t_tf.header.frame_id = BASE_FRAME
        t_tf.child_frame_id = f'target_{cls_name}'
        t_tf.transform.translation.x = X_b
        t_tf.transform.translation.y = Y_b
        t_tf.transform.translation.z = Z_b
        if bottle_quat is not None:
            t_tf.transform.rotation.x = float(bottle_quat[0])
            t_tf.transform.rotation.y = float(bottle_quat[1])
            t_tf.transform.rotation.z = float(bottle_quat[2])
            t_tf.transform.rotation.w = float(bottle_quat[3])
        else:
            t_tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_tf)


def main(args=None):
    parser = argparse.ArgumentParser(description='臂上 YOLO 感知')
    parser.add_argument(
        '--target-class', default=None,
        help='初始检测类别，逗号分隔，如 bottle 或 bottle,cup；'
             '默认 apple（兼容旧抓果流程）')
    parser.add_argument(
        '--mode', default='linkerhand', choices=['linkerhand', 'two_finger'],
        help='linkerhand=多帧一致性+中心点深度（旧方案，默认）；'
             'two_finger=鲁棒深度+单帧发布')
    parsed, unknown = parser.parse_known_args(sys.argv[1:])
    rclpy.init(args=unknown)
    initial = None
    if parsed.target_class:
        initial = {c.strip() for c in parsed.target_class.split(',')
                   if c.strip()}
    node = YoloGraspPerceptionNode(initial_classes=initial, mode=parsed.mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
