#!/usr/bin/env python3
"""
单 ArUco 标记检测 + TF 发布
替 aruco_opencv 的 board-based tracker，只检测一个标记
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import cv2
from cv_bridge import CvBridge
import numpy as np


class ArucoSingleTF(Node):
    def __init__(self):
        super().__init__('aruco_single_tf')
        self.declare_parameter('marker_size', 0.123)
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_dict', 'DICT_6X6_250')
        self.declare_parameter('parent_frame', 'camera_color_optical_frame')
        self.declare_parameter('child_frame', 'aruco_marker_frame')

        self.marker_size = self.get_parameter('marker_size').value
        self.marker_id = self.get_parameter('marker_id').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        dict_name = self.get_parameter('marker_dict').value
        self.dict_name = dict_name

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        dict_id = getattr(cv2.aruco, dict_name)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.obj_pts = np.array([
            [-self.marker_size/2,  self.marker_size/2, 0.0],
            [ self.marker_size/2,  self.marker_size/2, 0.0],
            [ self.marker_size/2, -self.marker_size/2, 0.0],
            [-self.marker_size/2, -self.marker_size/2, 0.0]
        ], dtype=np.float32)

        self.camera_matrix = None
        self.dist_coeffs = None

        # 诊断状态：记录最近一次检测到目标标记的时间、当前画面中的标记 id
        self.last_detect_time = None
        self.last_seen_ids = []
        self.diag_timer = self.create_timer(5.0, self.diagnostic_cb)

        # Percipio 相机以 RELIABLE QoS 发布，默认订阅 QoS 即可匹配。
        # 注意：该驱动只在 image_raw 有订阅者时才发布 camera_info，
        # 所以两个订阅缺一不可。
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info',
            self.camera_info_cb, 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw',
            self.image_cb, 10)

        self.get_logger().info(f'ArucoSingleTF ready (dict={dict_name}, id={self.marker_id}, size={self.marker_size}m)')

    def camera_info_cb(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera info received')

    def image_cb(self, msg):
        if self.camera_matrix is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        self.last_seen_ids = ids.flatten().tolist() if ids is not None else []
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for idx, mid in enumerate(ids.flatten()):
                if mid == self.marker_id:
                    ok, rvec, tvec = cv2.solvePnP(
                        self.obj_pts, corners[idx][0], self.camera_matrix, self.dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    if ok:
                        self.last_detect_time = self.get_clock().now()
                        cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)
                        cv2.putText(frame, f"ID:{mid} Z:{tvec[2][0]:.3f}m", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        t = TransformStamped()
                        t.header.stamp = msg.header.stamp
                        t.header.frame_id = self.parent_frame
                        t.child_frame_id = self.child_frame
                        t.transform.translation.x = float(tvec[0])
                        t.transform.translation.y = float(tvec[1])
                        t.transform.translation.z = float(tvec[2])
                        rmat, _ = cv2.Rodrigues(rvec)
                        from scipy.spatial.transform import Rotation as Rot
                        q = Rot.from_matrix(rmat).as_quat()
                        t.transform.rotation.x = float(q[0])
                        t.transform.rotation.y = float(q[1])
                        t.transform.rotation.z = float(q[2])
                        t.transform.rotation.w = float(q[3])
                        self.tf_broadcaster.sendTransform(t)
        cv2.imshow("ArUco Detector", frame)
        cv2.waitKey(1)

    def diagnostic_cb(self):
        if self.camera_matrix is None:
            self.get_logger().warn(
                '仍未收到 camera_info，请确认相机已启动、/camera/color/image_raw 有数据'
                '（该驱动只在 image_raw 有订阅者时才发布 camera_info；'
                '可用 ros2 topic hz /camera/color/image_raw 检查）')
            return
        if self.last_detect_time is not None:
            elapsed = (self.get_clock().now() - self.last_detect_time).nanoseconds / 1e9
            if elapsed < 5.0:
                return  # 目标标记检测正常
        if self.last_seen_ids:
            self.get_logger().warn(
                f'画面中检测到标记 {self.last_seen_ids}，但没有目标 ID {self.marker_id}'
                f'（字典 {self.dict_name}），请核对 marker_id / marker_dict 参数')
        else:
            self.get_logger().warn(
                f'已收到图像，但未检测到任何 {self.dict_name} 标记，请确认标记在相机视野内、成像清晰')


def main():
    rclpy.init()
    node = ArucoSingleTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
