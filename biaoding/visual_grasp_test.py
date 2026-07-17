#!/usr/bin/env python3
"""
视觉引导抓取测试脚本（集成 Elite Robot SDK）

功能：
1. 检测 ArUco 标定板 (ID=2)
2. 通过手眼标定结果计算目标在基座坐标系下的位置
3. 使用 Elite Robot SDK (RobotCartesianControl) 控制机械臂运动到目标上方
4. 支持夹爪控制

使用方法：
1. 启动机械臂驱动：
   ros2 launch eli_cs_robot_driver elite_control.launch.py robot_ip:=192.168.1.212 cs_type:=cs66

2. 运行本脚本：
   python3 visual_grasp_test.py
"""

import cv2
import numpy as np
import json
from scipy.spatial.transform import Rotation as Rot

import rclpy
from rclpy.executors import MultiThreadedExecutor
import sys
import os
import time
import subprocess
import signal

# 添加 elite_robot_example 包路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'elite_robot_example'))

from elite_robot_example.robot_cartesian_control import RobotCartesianControl

# 机械臂 IP 与 C++ 工具路径
ROBOT_IP = "192.168.1.212"
CARTESIAN_TOOL = os.path.join(os.path.dirname(__file__), "cartesian_move")


# ==========================================================
# 1. 从 RealSense 相机获取真实内参
# ==========================================================
def get_realsense_intrinsics():
    """从 RealSense 相机获取出厂标定内参"""
    try:
        import pyrealsense2 as rs
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipe.start(cfg)
        # 等待几帧稳定
        for _ in range(30):
            pipe.wait_for_frames()
        profile = pipe.get_active_profile()
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        intr = color_profile.get_intrinsics()
        pipe.stop()
        camera_matrix = np.array(
            [[intr.fx, 0, intr.ppx],
             [0, intr.fy, intr.ppy],
             [0, 0, 1]], dtype=np.float32)
        dist_coeffs = np.array(intr.coeffs, dtype=np.float32)
        print(f"RealSense 内参: fx={intr.fx:.2f}, fy={intr.fy:.2f}, "
              f"cx={intr.ppx:.2f}, cy={intr.ppy:.2f}")
        print(f"畸变系数: {intr.coeffs}")
        return camera_matrix, dist_coeffs
    except ImportError:
        print("警告: pyrealsense2 未安装，使用默认内参")
    except Exception as e:
        print(f"警告: 获取 RealSense 内参失败 ({e})，使用默认内参")

    # 回退到默认值
    camera_matrix = np.array([[611.69867, 0.0, 326.48343],
                              [0.0, 610.4816, 242.60501],
                              [0.0, 0.0, 1.0]], dtype=np.float32)
    dist_coeffs = np.zeros((5, 1), dtype=np.float32)
    return camera_matrix, dist_coeffs


camera_matrix, dist_coeffs = get_realsense_intrinsics()

marker_size = 0.123  # 标定板物理尺寸 (米)
TARGET_MARKER_ID = 0  # 目标 ArUco 标定板 ID (根据实际标定板修改：0, 2, 等)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters_create()

obj_points = np.array([
    [-marker_size/2.0,  marker_size/2.0, 0.0],
    [ marker_size/2.0,  marker_size/2.0, 0.0],
    [ marker_size/2.0, -marker_size/2.0, 0.0],
    [-marker_size/2.0, -marker_size/2.0, 0.0]
], dtype=np.float32)


def main():
    """主函数"""

    # ==========================================================
    # 2. 加载手眼标定结果
    # ==========================================================
    try:
        with open("hand_eye_result.json", "r") as f:
            calib_data = json.load(f)
        R_cam2tool = np.array(calib_data["R_cam2tool"])
        t_cam2tool = np.array(calib_data["t_cam2tool"])
        print("成功加载手眼标定结果文件！")
    except Exception as e:
        print("错误：无法加载标定文件，请先运行上一步的计算脚本生成 hand_eye_result.json！", e)
        return

    # ==========================================================
    # 3. 初始化 ROS2 与机械臂 SDK 节点
    # ==========================================================
    rclpy.init()
    executor = MultiThreadedExecutor()
    robot_node = RobotCartesianControl()
    executor.add_node(robot_node)

    # 等待机械臂状态
    if not robot_node.wait_for_state(timeout=10.0):
        print("错误：无法连接机械臂，请检查：")
        print("1. 机械臂驱动是否启动")
        print("2. IP 地址是否正确 (192.168.1.212)")
        print("3. 网络连接是否正常")
        robot_node.destroy_node()
        rclpy.shutdown()
        return

    print("机械臂已连接！")
    robot_node.print_status()

    # 确保机械臂上电并释放抱闸
    print("\n正在检查机械臂状态...")
    if not robot_node.power_on():
        print("警告：上电失败，可能已上电")
    time.sleep(1.0)
    if not robot_node.brake_release():
        print("警告：释放抱闸失败，可能已释放")
    time.sleep(1.0)
    print("机械臂已就绪，可以执行运动指令。")

    # ==========================================================
    # 4. 初始化相机（尝试多个索引，自动选择可用的相机）
    # ==========================================================
    CAMERA_INDEX = 2  # 相机索引：2=Realsense RGB(1280x720), 4=Realsense(1920x1080)
    cap = cv2.VideoCapture(CAMERA_INDEX)

    if not cap.isOpened():
        print(f"警告：无法打开相机索引 {CAMERA_INDEX}，自动探测中...")
        # 尝试常见索引（本机可用: 2, 4）
        for test_idx in [2, 4, 0]:
            cap = cv2.VideoCapture(test_idx)
            if cap.isOpened():
                print(f"已自动选择相机索引: {test_idx}")
                break
        else:
            print("错误：所有相机索引均无法打开！请检查相机连接。")
            robot_node.destroy_node()
            rclpy.shutdown()
            return

    print("\n【运行提示】：")
    print("1. 让相机对准标定板 (ID=2)。")
    print("2. 在画面窗口按下键盘 'G' 键，机械臂将尝试移动到标定板上方。")
    print("3. 按 'C' 键关闭夹爪，按 'O' 键打开夹爪。")
    print("4. 按 'H' 键机械臂回到零位。")
    print("5. 按 'Q' 键退出程序。")

    # 存储最新检测到的目标
    detected_p_cam = None

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # 检查是否为灰度图，如果是则直接使用，否则转换
            if len(frame.shape) == 2:
                gray = frame
            else:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            target_detected = False

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                detected_ids = ids.flatten()
                # 调试：打印检测到的所有 ID
                print(f"\r检测到标记 IDs: {detected_ids}", end="", flush=True)

                for idx, marker_id in enumerate(detected_ids):
                    if marker_id == TARGET_MARKER_ID:
                        img_points = corners[idx][0]
                        success, rvec, tvec = cv2.solvePnP(
                            obj_points, img_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
                        )

                        if success:
                            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)

                            # 实时获取标定板在相机下的 3D 坐标
                            detected_p_cam = tvec.flatten()  # [X_c, Y_c, Z_c]
                            target_detected = True

                            # 在画面上显示检测状态和坐标
                            cv2.putText(frame, f"Target ID={TARGET_MARKER_ID}: [{detected_p_cam[0]:.3f}, {detected_p_cam[1]:.3f}, {detected_p_cam[2]:.3f}]",
                                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                            cv2.putText(frame, "Press 'G' to grasp", (10, 60),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                print("\r未检测到任何标记      ", end="", flush=True)

            # 显示画面
            cv2.imshow("Grasp Pipeline Testing", frame)

            # 等待按键
            key = cv2.waitKey(1) & 0xFF

            # 按 Q 退出
            if key == ord('q') or key == ord('Q'):
                print("退出程序")
                break

            # 按 G 触发视觉引导抓取运动
            if key == ord('g') or key == ord('G'):
                if detected_p_cam is None:
                    print(f"\n未检测到标定板 (目标 ID={TARGET_MARKER_ID})，请确认标定板 ID 是否正确！")
                    continue

                print("\n======= 触发视觉引导运动 =======")

                # ----------------------------------------------------
                # 【步骤一】：通过 SDK 获取机械臂当前位姿
                # ----------------------------------------------------
                print("1. 正在获取机械臂当前位姿...")

                # 先刷新 ROS2 回调，确保获取最新数据
                for _ in range(10):
                    rclpy.spin_once(robot_node, timeout_sec=0.05)

                tcp_pose = robot_node.get_tcp_pose()

                if tcp_pose is None:
                    print("错误：无法获取机械臂当前位姿！")
                    continue

                current_pos, current_ori = tcp_pose
                print(f"   当前位置 (基座坐标系): x={current_pos[0]:.4f}, y={current_pos[1]:.4f}, z={current_pos[2]:.4f}")
                print(f"   当前姿态 (四元数):      x={current_ori[0]:.4f}, y={current_ori[1]:.4f}, z={current_ori[2]:.4f}, w={current_ori[3]:.4f}")

                # ----------------------------------------------------
                # 【步骤二】：执行空间坐标系转换
                #   - 标定板在相机坐标系: P_cam
                #   - 手眼标定: 相机→末端: P_tool = R_cam2tool * P_cam + t_cam2tool
                #   - 末端→基座:  P_base = R_tool2base * P_tool + t_tool2base
                # ----------------------------------------------------
                # A. 构造当前末端到基座的变换矩阵
                t_tool2base = np.array(current_pos).reshape(3, 1)
                R_tool2base = Rot.from_quat(current_ori).as_matrix()

                # B. 将标定板坐标从相机系转换到末端系
                P_c = detected_p_cam.reshape(3, 1)
                P_tool = np.dot(R_cam2tool, P_c) + t_cam2tool

                # C. 将末端系坐标转换到基座坐标系
                P_base = np.dot(R_tool2base, P_tool) + t_tool2base
                target_in_base = P_base.flatten()

                # 为了安全，让机械臂在目标上方 10 厘米（0.1米）处停下
                safe_target_z = target_in_base[2] 

                print(f"2. 计算目标在基座坐标系: X={target_in_base[0]:.4f}, Y={target_in_base[1]:.4f}, Z={target_in_base[2]:.4f}")
                print(f"   安全高度 (目标上方+0.1m): X={target_in_base[0]:.4f}, Y={target_in_base[1]:.4f}, Z={safe_target_z:.4f}")

                # ----------------------------------------------------
                # 【步骤三】：通过 C++ SDK 工具发送笛卡尔运动指令
                #   保持当前姿态不变，只改变末端位置到目标上方
                # ----------------------------------------------------
                print("3. 正在通过 Elite SDK 发送笛卡尔运动指令...")

                # 先释放 ROS2 控制权，避免与 C++ 工具冲突
                print("   释放 ROS2 控制权...")
                robot_node._hand_back_control()
                time.sleep(0.5)

                # 姿态保持当前不变，转为旋转向量
                rotvec = Rot.from_quat(current_ori).as_rotvec()

                target_x = target_in_base[0]
                target_y = target_in_base[1]
                target_z = safe_target_z

                move_dist = np.linalg.norm(
                    np.array([target_x, target_y, target_z]) - np.array(current_pos)
                )
                move_time = max(8.0, move_dist / 0.05)  # 慢速：0.05 m/s，至少 8 秒
                print(f"   运动距离: {move_dist:.3f}m, 预计时间: {move_time:.1f}s")

                # 构建 C++ 工具命令
                cmd = [
                    CARTESIAN_TOOL,
                    ROBOT_IP,
                    f"{target_x:.6f}",
                    f"{target_y:.6f}",
                    f"{target_z:.6f}",
                    f"{rotvec[0]:.6f}",
                    f"{rotvec[1]:.6f}",
                    f"{rotvec[2]:.6f}",
                    "0.05",              # speed (m/s) — 慢速
                    "0.15",              # acceleration (m/s²) — 柔和加速
                    f"{move_time:.1f}",  # time (s)
                ]

                print(f"   执行: {' '.join(cmd)}")

                try:
                    result = subprocess.run(
                        cmd,
                        cwd=os.path.dirname(CARTESIAN_TOOL),
                        timeout=move_time + 30,
                        capture_output=True,
                        text=True
                    )
                    print(result.stdout)
                    if result.returncode == 0:
                        print("4. 机械臂已到达目标上方！")
                    else:
                        print(f"4. 运动失败 (exit code {result.returncode})")
                        if result.stderr:
                            print(f"   错误: {result.stderr.strip()}")
                except subprocess.TimeoutExpired:
                    print("4. 运动超时！")
                except FileNotFoundError:
                    print(f"4. 错误：找不到 cartesian_move 工具 ({CARTESIAN_TOOL})，请先编译")

                print("================================\n")

            # 按 C 关闭夹爪
            if key == ord('c') or key == ord('C'):
                print("关闭夹爪...")
                robot_node.close_gripper(pin=0)

            # 按 O 打开夹爪
            if key == ord('o') or key == ord('O'):
                print("打开夹爪...")
                robot_node.open_gripper(pin=0)

            # 按 H 回到零位
            if key == ord('h') or key == ord('H'):
                print("回到零位...")
                robot_node.move_to_home(duration=5.0)

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        # 清理资源
        cap.release()
        cv2.destroyAllWindows()
        robot_node.destroy_node()
        rclpy.shutdown()
        print("资源已清理，程序退出。")


if __name__ == '__main__':
    main()
