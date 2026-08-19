#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
基于深度图的地面差分与矩形板面检测.

节点在收到 /elite_vision_job_cmd=1 后处理接下来的深度帧：
  1. 用深度相机内参把深度图反投影为 3D 点；
  2. RANSAC 拟合占比最大的地面平面；
  3. 按点到地面的有符号距离提取凸起板面；
  4. 取最大外轮廓，拟合四角并校验真实长宽；
  5. 发布板面有效点云，供 ysCamera3DSolver 继续做精 RANSAC 和手眼变换。

检测失败时发布空点云，C++ 侧按 depth -> cluster -> fixed 的顺序回退。
"""

from dataclasses import dataclass
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, Int32


@dataclass
class DetectorConfig:
    depth_scale: float = 0.00025
    min_depth_m: float = 0.20
    max_depth_m: float = 2.50
    sample_stride: int = 4
    ransac_iterations: int = 160
    floor_distance_threshold_m: float = 0.004
    floor_min_inlier_ratio: float = 0.45
    floor_max_rms_m: float = 0.004
    board_height_min_m: float = 0.090
    board_height_max_m: float = 0.140
    board_plane_threshold_m: float = 0.003
    board_max_rms_m: float = 0.003
    board_max_parallel_angle_deg: float = 8.0
    board_min_pixels: int = 2500
    board_min_points: int = 1500
    board_min_rectangularity: float = 0.60
    board_length_min_m: float = 0.25
    board_length_max_m: float = 0.45
    board_width_min_m: float = 0.10
    board_width_max_m: float = 0.25
    morphology_close_px: int = 9
    morphology_open_px: int = 3


class DetectionError(ValueError):
    def __init__(self, message, mask=None, corners=None):
        super().__init__(message)
        self.mask = mask
        self.corners = corners


def _normalized_plane_from_points(points):
    """最小二乘拟合平面，返回朝向相机原点的 (normal, d, rms)."""
    if points.shape[0] < 3:
        raise ValueError('not enough points for plane fit')
    centroid = np.mean(points, axis=0)
    centered = points - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1].astype(np.float64)
    norm = np.linalg.norm(normal)
    if norm < 1e-9:
        raise ValueError('degenerate plane')
    normal /= norm
    d = -float(np.dot(normal, centroid))
    # 法向朝相机原点。板面比地面更靠近相机时，有符号距离应为正。
    if np.dot(normal, centroid) > 0.0:
        normal = -normal
        d = -d
    residual = points @ normal + d
    rms = float(np.sqrt(np.mean(residual * residual)))
    return normal, d, rms


def _fit_plane_ransac(points, threshold, iterations, rng):
    """对输入点拟合最大平面，返回精修后的平面和内点比例."""
    if points.shape[0] < 100:
        raise ValueError('too few points for floor RANSAC')
    best_mask = None
    best_count = 0
    for _ in range(iterations):
        ids = rng.choice(points.shape[0], 3, replace=False)
        p0, p1, p2 = points[ids]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-8:
            continue
        normal /= norm
        d = -float(np.dot(normal, p0))
        mask = np.abs(points @ normal + d) <= threshold
        count = int(np.count_nonzero(mask))
        if count > best_count:
            best_count = count
            best_mask = mask
    if best_mask is None or best_count < 100:
        raise ValueError('floor RANSAC found no plane')

    normal, d, _ = _normalized_plane_from_points(points[best_mask])
    refined_mask = np.abs(points @ normal + d) <= threshold
    if np.count_nonzero(refined_mask) >= 100:
        normal, d, rms = _normalized_plane_from_points(points[refined_mask])
    else:
        rms = float('inf')
    ratio = float(np.count_nonzero(refined_mask)) / float(points.shape[0])
    return normal, d, rms, ratio


def _order_corners_clockwise(corners):
    center = np.mean(corners, axis=0)
    angles = np.arctan2(corners[:, 1] - center[1], corners[:, 0] - center[0])
    ordered = corners[np.argsort(angles)]
    # 固定从图像左上侧开始，避免日志中的角点次序在相邻帧翻转。
    start = int(np.argmin(ordered[:, 0] + ordered[:, 1]))
    return np.roll(ordered, -start, axis=0)


def _intersect_pixel_with_plane(pixel, intrinsics, normal, d):
    fx, fy, cx, cy = intrinsics
    u, v = float(pixel[0]), float(pixel[1])
    ray = np.array([(u - cx) / fx, (v - cy) / fy, 1.0], dtype=np.float64)
    denom = float(np.dot(normal, ray))
    if abs(denom) < 1e-8:
        raise ValueError('corner ray parallel to board plane')
    scale = -d / denom
    if scale <= 0.0:
        raise ValueError('corner plane intersection behind camera')
    return ray * scale


def detect_board(depth_m, intrinsics, cfg, rng=None):
    """检测单帧板面；成功返回包含 points/corners/statistics 的字典."""
    if rng is None:
        rng = np.random.default_rng(7)
    if depth_m.ndim != 2:
        raise ValueError('depth image must be single-channel')
    height, width = depth_m.shape
    fx, fy, cx, cy = [float(v) for v in intrinsics]
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError('invalid camera intrinsics')

    valid = (np.isfinite(depth_m)
             & (depth_m >= cfg.min_depth_m)
             & (depth_m <= cfg.max_depth_m))
    stride = max(1, int(cfg.sample_stride))
    sample_v, sample_u = np.nonzero(valid[::stride, ::stride])
    sample_v = sample_v * stride
    sample_u = sample_u * stride
    if sample_u.size < 500:
        raise ValueError('too few valid depth samples')
    sample_z = depth_m[sample_v, sample_u].astype(np.float64)
    floor_points = np.column_stack((
        (sample_u.astype(np.float64) - cx) * sample_z / fx,
        (sample_v.astype(np.float64) - cy) * sample_z / fy,
        sample_z))

    floor_n, floor_d, floor_rms, floor_ratio = _fit_plane_ransac(
        floor_points,
        cfg.floor_distance_threshold_m,
        cfg.ransac_iterations,
        rng)
    if floor_ratio < cfg.floor_min_inlier_ratio:
        raise ValueError(
            f'floor inlier ratio too low: {floor_ratio:.3f}')
    if floor_rms > cfg.floor_max_rms_m:
        raise ValueError(f'floor RMS too high: {floor_rms:.4f}m')

    grid_v, grid_u = np.indices((height, width), dtype=np.float32)
    z = depth_m.astype(np.float32, copy=False)
    x = (grid_u - cx) * z / fx
    y = (grid_v - cy) * z / fy
    signed_height = (floor_n[0] * x + floor_n[1] * y
                     + floor_n[2] * z + floor_d)
    candidate = (valid
                 & (signed_height >= cfg.board_height_min_m)
                 & (signed_height <= cfg.board_height_max_m))
    mask = candidate.astype(np.uint8) * 255

    close_size = max(1, int(cfg.morphology_close_px))
    open_size = max(1, int(cfg.morphology_open_px))
    if close_size % 2 == 0:
        close_size += 1
    if open_size % 2 == 0:
        open_size += 1
    if close_size > 1:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (close_size, close_size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    if open_size > 1:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (open_size, open_size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    contours = [
        c for c in contours
        if cv2.contourArea(c) >= cfg.board_min_pixels
    ]
    if not contours:
        raise DetectionError(
            'no board-sized connected component', mask=(mask != 0))
    contour = max(contours, key=cv2.contourArea)
    contour_area = float(cv2.contourArea(contour))
    rect = cv2.minAreaRect(contour)
    rect_area = max(1.0, float(rect[1][0] * rect[1][1]))
    rectangularity = contour_area / rect_area
    if rectangularity < cfg.board_min_rectangularity:
        raise DetectionError(
            f'component rectangularity too low: {rectangularity:.3f}',
            mask=(mask != 0), corners=cv2.boxPoints(rect))

    hull = cv2.convexHull(contour)
    perimeter = cv2.arcLength(hull, True)
    approx = cv2.approxPolyDP(hull, 0.02 * perimeter, True)
    if len(approx) == 4:
        corners_px = approx.reshape(4, 2).astype(np.float64)
    else:
        corners_px = cv2.boxPoints(rect).astype(np.float64)
    corners_px = _order_corners_clockwise(corners_px)

    component_fill = np.zeros_like(mask)
    cv2.drawContours(component_fill, [contour], -1, 255, thickness=cv2.FILLED)
    board_valid = candidate & (component_fill != 0)
    board_v, board_u = np.nonzero(board_valid)
    if board_u.size < cfg.board_min_points:
        raise DetectionError(
            f'too few board depth points: {board_u.size}',
            mask=board_valid, corners=corners_px)
    board_z = depth_m[board_v, board_u].astype(np.float64)
    board_points = np.column_stack((
        (board_u.astype(np.float64) - cx) * board_z / fx,
        (board_v.astype(np.float64) - cy) * board_z / fy,
        board_z))

    board_n, board_d, _ = _normalized_plane_from_points(board_points)
    board_residual = np.abs(board_points @ board_n + board_d)
    board_inliers = board_residual <= cfg.board_plane_threshold_m
    if np.count_nonzero(board_inliers) < cfg.board_min_points:
        raise DetectionError(
            'too few board-plane inliers',
            mask=board_valid, corners=corners_px)
    board_points = board_points[board_inliers]
    board_n, board_d, board_rms = _normalized_plane_from_points(board_points)
    if board_rms > cfg.board_max_rms_m:
        raise DetectionError(
            f'board RMS too high: {board_rms:.4f}m',
            mask=board_valid, corners=corners_px)

    normal_dot = float(np.clip(np.dot(floor_n, board_n), -1.0, 1.0))
    parallel_angle = math.degrees(math.acos(normal_dot))
    if parallel_angle > cfg.board_max_parallel_angle_deg:
        raise DetectionError(
            f'board/floor angle too large: {parallel_angle:.2f}deg',
            mask=board_valid, corners=corners_px)

    corners_3d = np.vstack([
        _intersect_pixel_with_plane(p, intrinsics, board_n, board_d)
        for p in corners_px
    ])
    sides = np.linalg.norm(
        np.roll(corners_3d, -1, axis=0) - corners_3d, axis=1)
    dim_a = 0.5 * float(sides[0] + sides[2])
    dim_b = 0.5 * float(sides[1] + sides[3])
    board_length = max(dim_a, dim_b)
    board_width = min(dim_a, dim_b)
    if not cfg.board_length_min_m <= board_length <= cfg.board_length_max_m:
        raise DetectionError(
            f'board length out of range: {board_length:.3f}m',
            mask=board_valid, corners=corners_px)
    if not cfg.board_width_min_m <= board_width <= cfg.board_width_max_m:
        raise DetectionError(
            f'board width out of range: {board_width:.3f}m',
            mask=board_valid, corners=corners_px)

    board_height = float(np.median(signed_height[board_valid]))
    return {
        'points': board_points.astype(np.float32),
        'mask': board_valid,
        'corners_px': corners_px.astype(np.float32),
        'corners_3d': corners_3d.astype(np.float32),
        'floor_rms': floor_rms,
        'floor_ratio': floor_ratio,
        'board_rms': board_rms,
        'board_height': board_height,
        'parallel_angle_deg': parallel_angle,
        'length_m': board_length,
        'width_m': board_width,
        'rectangularity': rectangularity,
    }


def _point_cloud_message(header, points):
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = int(points.shape[0])
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
    return msg


def _debug_image(depth_m, mask=None, corners=None, status=''):
    valid = np.isfinite(depth_m) & (depth_m > 0.0)
    gray = np.zeros(depth_m.shape, dtype=np.uint8)
    if np.count_nonzero(valid) > 10:
        near, far = np.percentile(depth_m[valid], (2.0, 98.0))
        if far > near:
            normalized = np.clip((depth_m - near) / (far - near), 0.0, 1.0)
            gray[valid] = np.rint((1.0 - normalized[valid]) * 255.0).astype(
                np.uint8)
    debug = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if mask is not None and mask.shape == depth_m.shape:
        overlay = debug.copy()
        overlay[mask.astype(bool)] = (0, 180, 0)
        debug = cv2.addWeighted(debug, 0.45, overlay, 0.55, 0.0)
    failed = status.startswith('FAILED')
    if corners is not None:
        corner_array = np.rint(corners).astype(np.int32)
        color = (0, 0, 255) if failed else (0, 255, 255)
        cv2.polylines(debug, [corner_array], True, color, 2)
    if status:
        cv2.rectangle(debug, (0, 0), (debug.shape[1], 28), (0, 0, 0), -1)
        color = (0, 0, 255) if failed else (0, 255, 0)
        cv2.putText(debug, status[:95], (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.52, color, 1, cv2.LINE_AA)
    return debug


class DepthBoardDetectNode(Node):
    def __init__(self):
        super().__init__('depth_board_detect_node')
        defaults = DetectorConfig()
        for name, value in defaults.__dict__.items():
            self.declare_parameter(name, value)
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter(
            'camera_info_topic', '/camera/depth/camera_info')
        self.declare_parameter(
            'board_cloud_topic', '/elite_polish/depth_board_points')
        self.declare_parameter(
            'board_info_topic', '/elite_polish/depth_board_info')
        self.declare_parameter(
            'debug_mask_topic', '/elite_polish/depth_board_mask')
        self.declare_parameter(
            'request_topic', '/elite_polish/depth_board_request')
        self.declare_parameter('max_attempts', 5)
        self.declare_parameter('show_debug_image', True)

        values = {
            name: self.get_parameter(name).value
            for name in defaults.__dict__.keys()
        }
        self.cfg = DetectorConfig(**values)
        self.max_attempts = max(
            1, int(self.get_parameter('max_attempts').value))
        self.show_debug_image = bool(
            self.get_parameter('show_debug_image').value)
        depth_topic = self.get_parameter('depth_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        cloud_topic = self.get_parameter('board_cloud_topic').value
        board_info_topic = self.get_parameter('board_info_topic').value
        debug_topic = self.get_parameter('debug_mask_topic').value
        request_topic = self.get_parameter('request_topic').value

        self.bridge = CvBridge()
        self.camera_info = None
        self.pending = False
        self.attempts = 0
        self.last_process_time = 0.0
        self.cloud_pub = self.create_publisher(PointCloud2, cloud_topic, 1)
        self.info_pub = self.create_publisher(
            Float32MultiArray, board_info_topic, 1)
        self.mask_pub = (self.create_publisher(Image, debug_topic, 1)
                         if self.show_debug_image else None)
        self.create_subscription(CameraInfo, info_topic, self._camera_info_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, self._depth_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(
            Int32, request_topic, self._command_cb, 1)
        self.get_logger().info(
            f'depth board detector ready: depth={depth_topic}, '
            f'info={info_topic}, '
            f'request={request_topic}, output={cloud_topic}')

    def _camera_info_cb(self, msg):
        self.camera_info = msg

    def _command_cb(self, msg):
        if msg.data == 1:
            self.pending = True
            self.attempts = 0
            self.get_logger().info('depth board detection requested')

    def _publish_failure(self, header, reason):
        self.get_logger().error(f'depth board detection failed: {reason}')
        self.cloud_pub.publish(_point_cloud_message(
            header, np.empty((0, 3), dtype=np.float32)))
        self.pending = False

    def _depth_cb(self, msg):
        if not self.pending:
            return
        if self.camera_info is None:
            self.attempts += 1
            if self.attempts >= self.max_attempts:
                self._publish_failure(msg.header, 'camera_info not received')
            return
        # 防止同一帧在异常高频重入；相机当前为约 5Hz。
        now = time.monotonic()
        if now - self.last_process_time < 0.02:
            return
        self.last_process_time = now
        self.attempts += 1
        try:
            raw = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
            raw = np.asarray(raw)
            if raw.dtype == np.uint16:
                depth_m = raw.astype(np.float32) * self.cfg.depth_scale
            elif raw.dtype in (np.float32, np.float64):
                depth_m = raw.astype(np.float32)
            else:
                raise ValueError(f'unsupported depth dtype: {raw.dtype}')

            info = self.camera_info
            image_h, image_w = depth_m.shape
            info_w = int(info.width) if info.width else image_w
            info_h = int(info.height) if info.height else image_h
            sx = float(image_w) / float(info_w)
            sy = float(image_h) / float(info_h)
            intrinsics = (
                float(info.k[0]) * sx,
                float(info.k[4]) * sy,
                float(info.k[2]) * sx,
                float(info.k[5]) * sy,
            )
            result = detect_board(depth_m, intrinsics, self.cfg)
        except Exception as exc:
            self.get_logger().warn(
                f'depth detection attempt {self.attempts}/'
                f'{self.max_attempts}: {exc}')
            if self.mask_pub is not None and 'depth_m' in locals():
                debug = _debug_image(
                    depth_m,
                    mask=getattr(exc, 'mask', None),
                    corners=getattr(exc, 'corners', None),
                    status=(f'FAILED {self.attempts}/'
                            f'{self.max_attempts}: {exc}'))
                self.mask_pub.publish(
                    self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))
            if self.attempts >= self.max_attempts:
                self._publish_failure(msg.header, str(exc))
            return

        self.cloud_pub.publish(
            _point_cloud_message(msg.header, result['points']))
        info_msg = Float32MultiArray()
        info_msg.data = [
            *result['corners_px'].reshape(-1).tolist(),
            float(result['length_m']),
            float(result['width_m']),
            float(result['board_height']),
            float(result['floor_rms']),
            float(result['board_rms']),
            float(result['parallel_angle_deg']),
            float(result['rectangularity']),
            float(result['points'].shape[0]),
        ]
        self.info_pub.publish(info_msg)

        if self.mask_pub is not None:
            debug = _debug_image(
                depth_m,
                mask=result['mask'],
                corners=result['corners_px'],
                status=(f'OK {result["length_m"]:.3f}x'
                        f'{result["width_m"]:.3f}m '
                        f'h={result["board_height"]:.3f}m'))
            self.mask_pub.publish(
                self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))

        self.get_logger().info(
            'depth board OK: points=%d size=%.3fx%.3fm height=%.4fm '
            'floor_rms=%.4fm board_rms=%.4fm parallel=%.2fdeg rect=%.2f'
            % (result['points'].shape[0], result['length_m'],
               result['width_m'], result['board_height'],
               result['floor_rms'], result['board_rms'],
               result['parallel_angle_deg'], result['rectangularity']))
        self.pending = False


def main():
    rclpy.init()
    node = DepthBoardDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
