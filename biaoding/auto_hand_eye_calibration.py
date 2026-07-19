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
        """获取机械臂当前末端位姿 (pos_xyz, quat_xyzw) — 直接用四元数，和抓取脚本保持一致"""
        for _ in range(10):
            rclpy.spin_once(self.robot, timeout_sec=0.05)

        tcp = self.robot.get_tcp_pose()
        if tcp is None:
            return None

        pos, quat = tcp  # pos=[x,y,z], quat=[qx,qy,qz,qw]
        return [pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]]

    def _wait_stable(self, tolerance=0.002, timeout=3.0):
        """等待机械臂停稳（位置变化 < tolerance 米）"""
        prev = None
        start = time.time()
        while time.time() - start < timeout:
            pos = self.robot.get_tcp_position()
            if pos is None:
                time.sleep(0.2)
                continue
            if prev is not None:
                dist = np.linalg.norm(np.array(pos) - np.array(prev))
                if dist < tolerance:
                    return True
            prev = pos
            time.sleep(0.3)
        return False  # 超时

    def _compute_reproj_error(self, rvec, tvec, img_points):
        """计算 solvePnP 重投影误差（像素）"""
        proj, _ = cv2.projectPoints(
            self.obj_points, rvec, tvec,
            self.camera_matrix, self.dist_coeffs
        )
        return np.mean(np.linalg.norm(img_points - proj.reshape(-1, 2), axis=1))

    def _detect_marker(self):
        """取一帧新图像并检测标定板，返回 (rvec, tvec, img_points) 或 None"""
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        frame = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is not None:
            for idx, marker_id in enumerate(ids.flatten()):
                if marker_id == TARGET_MARKER_ID:
                    success, rvec, tvec = cv2.solvePnP(
                        self.obj_points, corners[idx][0],
                        self.camera_matrix, self.dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    if success:
                        return rvec, tvec, corners[idx][0]
        return None

    def capture_sample(self):
        """采集一组标定数据 —— 停稳后同时读取相机和机械臂数据"""
        if not self.marker_detected:
            print("\n[错误] 未检测到标定板，无法采集！")
            return False

        # 等机械臂停稳
        print("  等待机械臂停稳...", end="", flush=True)
        if not self._wait_stable():
            print(" 超时（可能仍在运动）")
        else:
            print(" 已停稳")

        # ★ 关键修复：停稳后重新取一帧相机数据，确保相机和机械臂是同一时刻
        result = self._detect_marker()
        if result is None:
            print("\n[错误] 停稳后未检测到标定板！")
            return False
        fresh_rvec, fresh_tvec, fresh_img_pts = result

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            print("\n[错误] 无法获取机械臂位姿！")
            return False

        # 计算重投影误差
        reproj_err = self._compute_reproj_error(fresh_rvec, fresh_tvec, fresh_img_pts)

        self.samples.append({
            'robot_pose': robot_pose,
            'rvec': fresh_rvec.copy(),
            'tvec': fresh_tvec.copy(),
            'reproj_err': reproj_err
        })

        n = len(self.samples)
        rv = fresh_rvec.flatten()
        tv = fresh_tvec.flatten()

        err_flag = " ⚠️ 重投影偏大!" if reproj_err > 1.0 else ""
        print(f"\n{'='*50}")
        print(f"[采集成功] 第 {n} 组")
        print(f"  机械臂: pos=({robot_pose[0]:.4f},{robot_pose[1]:.4f},{robot_pose[2]:.4f})m")
        print(f"           quat=({robot_pose[3]:.4f},{robot_pose[4]:.4f},{robot_pose[5]:.4f},{robot_pose[6]:.4f})")
        print(f"  标定板: tvec=({tv[0]:.4f},{tv[1]:.4f},{tv[2]:.4f})m")
        print(f"           rvec=({rv[0]:.4f},{rv[1]:.4f},{rv[2]:.4f})")
        print(f"  重投影误差: {reproj_err:.3f} px{err_flag}")
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
            s = self.samples.pop()
            print(f"已删除第 {len(self.samples)+1} 组 (reproj={s['reproj_err']:.3f}px)，剩余 {len(self.samples)} 组")
        else:
            print("无数据可删")

    def list_samples(self):
        """列出所有样本的重投影误差，标记异常"""
        if not self.samples:
            print("无样本")
            return
        print(f"\n{'='*60}")
        print(f"  样本列表 (共 {len(self.samples)} 组)")
        print(f"  {'索引':>4s}  {'重投影误差(px)':>16s}  {'状态'}")
        print(f"  {'-'*40}")
        errors = [s['reproj_err'] for s in self.samples]
        mean_err = np.mean(errors)
        std_err = np.std(errors)
        for i, s in enumerate(self.samples):
            e = s['reproj_err']
            flag = ""
            if e > mean_err + 3 * std_err:
                flag = " ❌ 离群! (建议删除)"
            elif e > 1.0:
                flag = " ⚠️ 偏大"
            print(f"  {i:4d}  {e:16.4f}{flag}")
        print(f"  {'-'*40}")
        print(f"  均值: {mean_err:.4f} px, 标准差: {std_err:.4f} px")
        print(f"  删除命令: 按 X 后输入索引号")
        print(f"{'='*60}\n")

    def delete_by_index(self, idx):
        """删除指定索引的样本"""
        if 0 <= idx < len(self.samples):
            s = self.samples.pop(idx)
            print(f"已删除第 {idx} 组 (reproj={s['reproj_err']:.3f}px)，剩余 {len(self.samples)} 组")
        else:
            print(f"索引 {idx} 无效 (范围 0-{len(self.samples)-1})")

    def compute_calibration(self):
        """计算手眼标定"""
        n = len(self.samples)
        if n < 10:
            print(f"\n[警告] 只有 {n} 组，建议至少 {MIN_SAMPLES} 组")
            return

        # 打印重投影误差汇总
        errors = [s['reproj_err'] for s in self.samples]
        bad_samples = [i for i, e in enumerate(errors) if e > 1.0]
        if bad_samples:
            print(f"\n⚠️  警告: 第 {bad_samples} 组重投影误差 > 1px，可能 solvePnP 解错（翻转歧义）")
            print(f"   建议: 先按 L 查看，再按 X 删除这几组重试\n")

        print(f"\n正在用 {n} 组数据计算手眼标定...")

        R_gripper2base = []
        t_gripper2base = []
        R_target2cam = []
        t_target2cam = []

        for s in self.samples:
            robot_pose = s['robot_pose']
            cam_rvec = s['rvec']
            cam_tvec = s['tvec']
            x, y, z, qx, qy, qz, qw = robot_pose
            t_g2b = np.array([x, y, z]).reshape(3, 1)
            R_g2b = Rot.from_quat([qx, qy, qz, qw]).as_matrix()

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
        print("  S = 采集  |  D = 删上组  |  L = 列表  |  X = 删指定")
        print("  C = 计算  |  Q = 退出")
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
                                self.current_img_points = img_points
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
                elif key == ord('l') or key == ord('L'):
                    self.list_samples()
                elif key == ord('x') or key == ord('X'):
                    try:
                        idx_str = input("输入要删除的样本索引: ").strip()
                        idx = int(idx_str)
                        self.delete_by_index(idx)
                    except ValueError:
                        print("无效输入")
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
