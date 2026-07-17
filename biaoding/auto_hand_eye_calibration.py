#!/usr/bin/env python3
"""
自动化手眼标定脚本（眼在手上 Eye-in-Hand）

一键采集：对准标定板 → 移动机械臂到不同位姿 → 按 S 自动记录
自动获取：相机端（ArUco solvePnP） + 机械臂端（ROS2 实时位姿）

使用方法：
1. 启动机械臂驱动:
   ros2 launch eli_cs_robot_driver elite_control.launch.py robot_ip:=192.168.1.212 cs_type:=cs66

2. 运行标定脚本:
   python3 auto_hand_eye_calibration.py

3. 操作:
   - 移动机械臂到不同位置/角度（至少 15 个，建议 20+）
   - 确保相机能看到标定板（画面显示绿色坐标轴 = 检测成功）
   - 按 S 记录当前这组数据
   - 按 D 删除上一组数据
   - 按 C 计算标定并保存
   - 按 Q 退出
"""

import cv2
import numpy as np
import json
import os
import sys
import time
from scipy.spatial.transform import Rotation as Rot

import rclpy
from rclpy.executors import MultiThreadedExecutor

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'elite_robot_example'))
from elite_robot_example.robot_basic_control import RobotBasicControl


# ==========================================================
# 配置参数
# ==========================================================
ROBOT_IP = "192.168.1.212"
MARKER_SIZE = 0.123       # ArUco 标定板黑边物理尺寸（米），必须和实际打印一致！
TARGET_MARKER_ID = 0       # 标定板 ArUco ID
MIN_SAMPLES = 15            # 最少采集组数
OUTPUT_FILE = "hand_eye_result.json"


class AutoHandEyeCalibration:
    """自动化眼在手上标定"""

    def __init__(self):
        self.samples = []  # [(robot_pose_xyzrpy, cam_rvec, cam_tvec), ...]

        # ---------- 1. 初始化相机 ----------
        self._init_camera()

        # ---------- 2. 初始化 ROS2 ----------
        self._init_ros2()

        # ---------- 3. ArUco 检测器 ----------
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.obj_points = np.array([
            [-MARKER_SIZE / 2,  MARKER_SIZE / 2, 0.0],
            [ MARKER_SIZE / 2,  MARKER_SIZE / 2, 0.0],
            [ MARKER_SIZE / 2, -MARKER_SIZE / 2, 0.0],
            [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0.0]
        ], dtype=np.float32)

        # 当前帧检测结果
        self.current_rvec = None
        self.current_tvec = None
        self.marker_detected = False

    def _init_camera(self):
        """初始化 RealSense 相机并获取内参"""
        import pyrealsense2 as rs
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        for _ in range(30):
            self.pipeline.wait_for_frames()

        profile = self.pipeline.get_active_profile()
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        intr = color_profile.get_intrinsics()

        self.camera_matrix = np.array(
            [[intr.fx, 0, intr.ppx],
             [0, intr.fy, intr.ppy],
             [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.array(intr.coeffs, dtype=np.float32)

        print(f"\n{'='*60}")
        print(f"相机内参: fx={intr.fx:.2f}, fy={intr.fy:.2f}, "
              f"cx={intr.ppx:.2f}, cy={intr.ppy:.2f}")
        print(f"畸变系数: {intr.coeffs}")
        print(f"标定板尺寸: {MARKER_SIZE}m, 目标 ID: {TARGET_MARKER_ID}")
        print(f"{'='*60}")

    def _init_ros2(self):
        """初始化 ROS2 节点"""
        rclpy.init()
        self.executor = MultiThreadedExecutor()
        self.robot = RobotBasicControl()
        self.executor.add_node(self.robot)

        if not self.robot.wait_for_state(timeout=10.0):
            raise RuntimeError("无法连接机械臂，请检查驱动是否启动")

        print("ROS2 已连接，机械臂状态正常\n")

    def get_robot_pose(self):
        """获取机械臂当前末端位姿 [x, y, z, rx, ry, rz]（米, 弧度）"""
        for _ in range(10):
            rclpy.spin_once(self.robot, timeout_sec=0.05)

        pos = self.robot.get_tcp_position()
        euler = self.robot.get_tcp_orientation_euler()

        if pos is None or euler is None:
            return None

        return [pos[0], pos[1], pos[2], euler[0], euler[1], euler[2]]

    def capture_sample(self):
        """采集一组标定数据"""
        if not self.marker_detected:
            print("\n[错误] 未检测到标定板，无法采集！")
            return False

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            print("\n[错误] 无法获取机械臂位姿！")
            return False

        self.samples.append((
            robot_pose,
            self.current_rvec.copy(),
            self.current_tvec.copy()
        ))

        n = len(self.samples)
        rv = self.current_rvec.flatten()
        tv = self.current_tvec.flatten()
        print(f"\n{'='*50}")
        print(f"[采集成功] 第 {n} 组")
        print(f"  机械臂: pos=({robot_pose[0]:.4f},{robot_pose[1]:.4f},{robot_pose[2]:.4f})m")
        print(f"           euler(rx,ry,rz)=({robot_pose[3]:.4f},{robot_pose[4]:.4f},{robot_pose[5]:.4f})rad")
        print(f"  标定板: tvec=({tv[0]:.4f},{tv[1]:.4f},{tv[2]:.4f})m")
        print(f"           rvec=({rv[0]:.4f},{rv[1]:.4f},{rv[2]:.4f})")
        need = max(0, MIN_SAMPLES - n)
        if need > 0:
            print(f"  还需 {need} 组 (当前 {n}/{MIN_SAMPLES})")
        else:
            print(f"  已满足最小需求 ({n} 组)，可按 C 计算标定")
        print(f"{'='*50}\n")
        return True

    def delete_last_sample(self):
        """删除最后一组数据"""
        if self.samples:
            self.samples.pop()
            print(f"已删除，剩余 {len(self.samples)} 组")
        else:
            print("无数据可删")

    def compute_calibration(self):
        """计算手眼标定"""
        n = len(self.samples)
        if n < 10:
            print(f"\n[警告] 只有 {n} 组，建议至少 {MIN_SAMPLES} 组")
            return

        print(f"\n正在用 {n} 组数据计算手眼标定...")

        R_gripper2base = []
        t_gripper2base = []
        R_target2cam = []
        t_target2cam = []

        for robot_pose, cam_rvec, cam_tvec in self.samples:
            x, y, z, rx, ry, rz = robot_pose
            t_g2b = np.array([x, y, z]).reshape(3, 1)
            R_g2b = Rot.from_euler('xyz', [rx, ry, rz], degrees=False).as_matrix()

            R_t2c, _ = cv2.Rodrigues(cam_rvec)
            t_t2c = np.array(cam_tvec).reshape(3, 1)

            R_gripper2base.append(R_g2b)
            t_gripper2base.append(t_g2b)
            R_target2cam.append(R_t2c)
            t_target2cam.append(t_t2c)

        # 使用多种方法计算，选最佳
        methods = {
            'Tsai': cv2.CALIB_HAND_EYE_TSAI,
            'Park': cv2.CALIB_HAND_EYE_PARK,
            'Daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
        }

        best_result = None
        best_method = None
        best_error = float('inf')

        for name, method in methods.items():
            try:
                R, t = cv2.calibrateHandEye(
                    R_gripper2base, t_gripper2base,
                    R_target2cam, t_target2cam,
                    method=method
                )
                # 计算重投影误差
                error = self._compute_error(
                    R_gripper2base, t_gripper2base,
                    R_target2cam, t_target2cam, R, t
                )
                print(f"  方法 {name:12s}: 误差 = {error:.4f}m")
                if error < best_error:
                    best_error = error
                    best_result = (R, t)
                    best_method = name
            except Exception as e:
                print(f"  方法 {name:12s}: 失败 ({e})")

        if best_result is None:
            print("\n[错误] 所有方法均失败")
            return

        R_cam2tool, t_cam2tool = best_result

        # 保存到 JSON
        result = {
            "R_cam2tool": R_cam2tool.tolist(),
            "t_cam2tool": t_cam2tool.tolist(),
            "method": best_method,
            "error": best_error,
            "samples": n,
        }
        with open(OUTPUT_FILE, 'w') as f:
            json.dump(result, f, indent=4)

        print(f"\n{'='*60}")
        print(f"标定完成！(方法: {best_method}, 误差: {best_error:.4f}m)")
        print(f"R_cam2tool:\n{R_cam2tool}")
        print(f"t_cam2tool:\n{t_cam2tool}")
        print(f"结果已保存至: {OUTPUT_FILE}")
        print(f"{'='*60}\n")

    def _compute_error(self, R_g2b, t_g2b, R_t2c, t_t2c, R_c2t, t_c2t):
        """
        评估标定质量：
        标定板是静止的 → 各样本算出的标定板在基座坐标系下的位置应该一致。
        误差 = 各样本位置的标准差 / 均值距离
        """
        n = len(R_g2b)
        target_positions = np.zeros((n, 3))
        for i in range(n):
            # 标定板原点(0,0,0)在相机坐标系 → 转换到基座坐标系
            P_cam = t_t2c[i]  # 3x1
            P_tool = np.dot(R_c2t, P_cam) + t_c2t
            P_base = np.dot(R_g2b[i], P_tool) + t_g2b[i]
            target_positions[i] = P_base.flatten()

        # 各样本算出的标定板位置
        mean_pos = np.mean(target_positions, axis=0)
        std_pos = np.std(target_positions, axis=0)
        max_dev = np.max(np.linalg.norm(target_positions - mean_pos, axis=1))

        print(f"  标定板推算位置(基座):")
        print(f"    均值 X={mean_pos[0]:.4f} Y={mean_pos[1]:.4f} Z={mean_pos[2]:.4f}")
        print(f"    标准差 X={std_pos[0]:.4f} Y={std_pos[1]:.4f} Z={std_pos[2]:.4f}")
        print(f"    最大偏差: {max_dev:.4f}m")

        # 标准差越小越好（< 2cm 算合格）
        mean_std = np.mean(std_pos)
        return mean_std

    def run(self):
        """主循环"""
        print("\n" + "="*60)
        print("  自动化手眼标定 (Eye-in-Hand)")
        print("  S = 采集  |  D = 删除上组  |  C = 计算  |  Q = 退出")
        print(f"  目标: 至少 {MIN_SAMPLES} 组，越多越准")
        print("="*60 + "\n")

        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                frame = np.asanyarray(color_frame.get_data())
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                corners, ids, _ = cv2.aruco.detectMarkers(
                    gray, self.aruco_dict, parameters=self.aruco_params
                )

                self.marker_detected = False

                if ids is not None and len(ids) > 0:
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    for idx, marker_id in enumerate(ids.flatten()):
                        if marker_id == TARGET_MARKER_ID:
                            img_points = corners[idx][0]
                            success, rvec, tvec = cv2.solvePnP(
                                self.obj_points, img_points,
                                self.camera_matrix, self.dist_coeffs,
                                flags=cv2.SOLVEPNP_IPPE_SQUARE
                            )
                            if success:
                                self.marker_detected = True
                                self.current_rvec = rvec
                                self.current_tvec = tvec
                                cv2.drawFrameAxes(frame, self.camera_matrix,
                                                  self.dist_coeffs, rvec, tvec, 0.05)

                # HUD 信息
                status_color = (0, 255, 0) if self.marker_detected else (0, 0, 255)
                status_text = "READY" if self.marker_detected else "NO MARKER"
                cv2.putText(frame, f"Status: {status_text}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
                cv2.putText(frame, f"Samples: {len(self.samples)}/{MIN_SAMPLES}", (10, 65),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                if self.marker_detected:
                    z_cam = self.current_tvec[2][0]
                    cv2.putText(frame, f"Z_dist: {z_cam:.3f}m", (10, 100),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

                cv2.imshow("Hand-Eye Calibration", frame)
                key = cv2.waitKey(1) & 0xFF

                if key == ord('q') or key == ord('Q'):
                    print("\n退出标定程序")
                    break
                elif key == ord('s') or key == ord('S'):
                    self.capture_sample()
                elif key == ord('d') or key == ord('D'):
                    self.delete_last_sample()
                elif key == ord('c') or key == ord('C'):
                    self.compute_calibration()

        except KeyboardInterrupt:
            print("\n用户中断")
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()
            self.robot.destroy_node()
            rclpy.shutdown()
            print("资源已清理")


if __name__ == '__main__':
    calibrator = AutoHandEyeCalibration()
    calibrator.run()
