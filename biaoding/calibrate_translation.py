#!/usr/bin/env python3
"""
单点平移标定：用机械臂末端物理接触标定板中心，直接算 t_cam2tool。

用法：
1. 先把标定板放桌面上
2. 手动操控机械臂，让夹具尖端（TCP）精确对准标定板中心
3. 运行本脚本，按 S 采集
4. 自动更新 hand_eye_result.json 的 t_cam2tool
"""

import cv2
import numpy as np
import json
import os
import sys
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'elite_robot_example'))
from elite_robot_example.robot_basic_control import RobotBasicControl

MARKER_SIZE = 0.123
TARGET_ID = 0
RESULT_FILE = "hand_eye_result.json"


def get_realsense_intrinsics():
    import pyrealsense2 as rs
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipe.start(cfg)
    for _ in range(30):
        pipe.wait_for_frames()
    profile = pipe.get_active_profile()
    cp = rs.video_stream_profile(profile.get_stream(rs.stream.color))
    intr = cp.get_intrinsics()
    pipe.stop()
    return (np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], dtype=np.float32),
            np.array(intr.coeffs, dtype=np.float32))


def main():
    # 加载已有的 R_cam2tool
    if not os.path.exists(RESULT_FILE):
        print(f"错误：找不到 {RESULT_FILE}，请先跑一次完整标定")
        return
    with open(RESULT_FILE) as f:
        calib = json.load(f)
    R_cam2tool = np.array(calib["R_cam2tool"])

    # 相机
    cam_matrix, dist = get_realsense_intrinsics()
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    aruco_params = cv2.aruco.DetectorParameters_create()
    obj_pts = np.array([
        [-MARKER_SIZE/2,  MARKER_SIZE/2, 0],
        [ MARKER_SIZE/2,  MARKER_SIZE/2, 0],
        [ MARKER_SIZE/2, -MARKER_SIZE/2, 0],
        [-MARKER_SIZE/2, -MARKER_SIZE/2, 0],
    ], dtype=np.float32)

    import pyrealsense2 as rs
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipe.start(cfg)
    for _ in range(30):
        pipe.wait_for_frames()

    # ROS2
    rclpy.init()
    executor = MultiThreadedExecutor()
    robot = RobotBasicControl()
    executor.add_node(robot)
    if not robot.wait_for_state(timeout=10):
        print("ROS2 连接失败")
        return

    print("\n" + "=" * 60)
    print("  单点平移标定")
    print("  1. 手动把夹具尖端对准标定板中心")
    print("  2. 确认标定板在画面中（看到绿色坐标轴）")
    print("  3. 按 S 自动计算并保存 t_cam2tool")
    print("=" * 60 + "\n")

    try:
        while True:
            frames = pipe.wait_for_frames()
            frame = np.asanyarray(frames.get_color_frame().get_data())
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

            detected = False
            rvec, tvec = None, None
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                for i, mid in enumerate(ids.flatten()):
                    if mid == TARGET_ID:
                        ok, rv, tv = cv2.solvePnP(
                            obj_pts, corners[i][0], cam_matrix, dist,
                            flags=cv2.SOLVEPNP_IPPE_SQUARE
                        )
                        if ok:
                            detected = True
                            rvec, tvec = rv, tv
                            cv2.drawFrameAxes(frame, cam_matrix, dist, rvec, tvec, 0.05)

            status = "READY" if detected else "NO MARKER"
            color = (0, 255, 0) if detected else (0, 0, 255)
            cv2.putText(frame, f"Status: {status}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.imshow("Translation Calibration", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s') and detected:
                # 获取当前 TCP 位姿
                for _ in range(10):
                    rclpy.spin_once(robot, timeout_sec=0.05)
                tcp = robot.get_tcp_pose()
                if tcp is None:
                    print("获取 TCP 失败")
                    continue
                pos, quat = tcp
                from scipy.spatial.transform import Rotation as Rot
                R_g2b = Rot.from_quat(quat).as_matrix()

                # p_cam: 标定板在相机坐标系下的位置
                p_cam = tvec.flatten()

                # 计算 t_cam2tool
                # TCP 对准标定板中心 → 标定板在基座 = TCP 在基座
                # R_g2b * (R_c2t * p_cam + t_c2t) + pos = pos
                # → R_c2t * p_cam + t_c2t = 0
                # → t_c2t = -R_c2t * p_cam
                t_new = -np.dot(R_cam2tool, p_cam).reshape(3, 1)

                # 更新并保存
                calib["t_cam2tool"] = t_new.tolist()
                with open(RESULT_FILE, 'w') as f:
                    json.dump(calib, f, indent=4)

                print(f"\n标定完成！")
                print(f"  p_cam = ({p_cam[0]:.4f}, {p_cam[1]:.4f}, {p_cam[2]:.4f})")
                print(f"  t_cam2tool (新) = ({t_new[0,0]:.4f}, {t_new[1,0]:.4f}, {t_new[2,0]:.4f})")
                print(f"  已保存至 {RESULT_FILE}\n")

    finally:
        pipe.stop()
        cv2.destroyAllWindows()
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
