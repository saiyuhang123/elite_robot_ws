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
import numpy as np
import cv2
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


def load_hand_eye_matrix(path):
    """从 hand_eye_result.json 读取 相机->法兰 的 4x4 变换矩阵。"""
    with open(path, 'r') as f:
        calib = json.load(f)
    T = np.eye(4)
    T[:3, :3] = np.array(calib['R_cam2tool'])
    T[:3, 3] = np.array(calib['t_cam2tool']).flatten()
    return T


class YoloGraspPerceptionNode(Node):
    def __init__(self):
        super().__init__('yolo_grasp_perception_node')
        self.get_logger().info('>>> YOLO 抓取感知节点正在启动...')

        # ---------------- 1. 初始化参数与配置 ----------------
        self.bridge = CvBridge()

        # 目标类别集合（可通过 /yolo/target_class 话题动态修改）
        self.target_classes = set(DEFAULT_TARGET_CLASSES)

        # 加载 YOLO 模型 (首次运行会自动下载 yolov8n.pt，可换成你自己的 pt 模型)
        self.get_logger().info('正在加载 YOLO 模型...')
        self.yolo_model = YOLO('yolov8s.pt')

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
        # 不用时间同步器（本机相机各话题时间戳有固定偏差，同步器凑不齐）。
        # 各自缓存最新一帧，彩色图到达时用最新的深度+内参处理（静态场景够用）。
        self._received = {'color': False, 'depth': False, 'info': False, 'tf': False}
        self.latest_color = None
        self.latest_depth = None
        self.latest_info = None
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
        # 彩色图到达时驱动一次感知（仅在按需识别开启时；深度和内参需已就绪）
        if (self.enabled and self.latest_depth is not None
                and self.latest_info is not None):
            try:
                self.perception_callback(msg, self.latest_depth, self.latest_info)
            except Exception as e:
                import traceback
                self.get_logger().error(f'[诊断] 感知回调异常: {e}\n{traceback.format_exc()}')

    def _depth_cb(self, msg):
        self._mark('depth')
        self.latest_depth = msg

    def _info_cb(self, msg):
        self._mark('info')
        self.latest_info = msg

    # ---- 按需识别开关服务 ----
    def _set_enabled_cb(self, request, response):
        self.enabled = bool(request.data)
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

            # ---------------- 7x7 窗口中值滤波提取深度 ----------------
            patch_size = 7
            half = patch_size // 2

            # 边界保护
            v_start = max(0, v_center - half)
            v_end = min(depth_img.shape[0], v_center + half + 1)
            u_start = max(0, u_center - half)
            u_end = min(depth_img.shape[1], u_center + half + 1)

            depth_patch = depth_img[v_start:v_end, u_start:u_end]
            valid_depths = depth_patch[depth_patch > 0]  # 过滤掉 0 深度无效值

            if len(valid_depths) == 0:
                self.get_logger().warn(f'目标 [{cls_name}] 中心点深度无效，跳过...')
                continue

            # 取深度中位数 (Percipio 原始值 0.25mm/LSB -> 转换为 m)
            depth_raw = np.median(valid_depths)
            Z_c = float(depth_raw) * 0.25 / 1000.0

            # 过滤不合理的深度值 (比如小于10cm 或 大于 3m)
            if Z_c < 0.1 or Z_c > 3.0:
                continue

            # ---------------- 反推相机坐标系 3D 坐标 (X_c, Y_c, Z_c) ----------------
            X_c = (u_center - cx) * Z_c / fx
            Y_c = (v_center - cy) * Z_c / fy

            if best is None or Z_c < best[0]:
                best = (Z_c, xyxy, cls_name, conf, X_c, Y_c, u_center, v_center)

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

        self.get_logger().info(
            f'识别到 [{cls_name}] (可信度:{conf:.2f}) -> '
            f'相机系: [{X_c:.3f}, {Y_c:.3f}, {Z_c:.3f}]m | '
            f'基座系: [{X_b:.3f}, {Y_b:.3f}, {Z_b:.3f}]m'
        )

        # ---------------- 发布 PoseStamped 消息与 TF 广播 ----------------
        now = self.get_clock().now().to_msg()

        # 发布 Pose 消息
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = BASE_FRAME
        pose_msg.pose.position.x = X_b
        pose_msg.pose.position.y = Y_b
        pose_msg.pose.position.z = Z_b
        pose_msg.pose.orientation.w = 1.0  # 默认姿态
        self.pose_pub.publish(pose_msg)

        # 广播 TF Transform (方便在 RViz2 里可视化)
        t_tf = TransformStamped()
        t_tf.header.stamp = now
        t_tf.header.frame_id = BASE_FRAME
        t_tf.child_frame_id = f'target_{cls_name}'
        t_tf.transform.translation.x = X_b
        t_tf.transform.translation.y = Y_b
        t_tf.transform.translation.z = Z_b
        t_tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_tf)

        # ---------------- 图像上绘制调试信息 ----------------
        cv2.rectangle(color_img, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (0, 255, 0), 2)
        cv2.circle(color_img, (u_center, v_center), 5, (0, 0, 255), -1)
        label = f'{cls_name}: Base[{X_b:.2f}, {Y_b:.2f}, {Z_b:.2f}]m'
        cv2.putText(color_img, label, (xyxy[0], xyxy[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 发布画框调试后的图像
        self.annotated_img_pub.publish(self.bridge.cv2_to_imgmsg(color_img, encoding='bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = YoloGraspPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()