#!/usr/bin/env python3
"""
视觉引导抓取测试脚本（使用 Elite Robot SDK）

功能：
1. 检测 ArUco 标定板
2. 通过手眼标定结果计算目标位置
3. 使用 Elite Robot SDK 控制机械臂运动到目标位置

使用方法：
1. 启动机械臂驱动：
   ros2 launch eli_cs_robot_driver elite_control.launch.py robot_ip:=192.168.1.212 cs_type:=cs66

2. 运行本脚本：
   python3 visual_grasp_with_sdk.py
"""

import cv2
import numpy as np
import json
from scipy.spatial.transform import Rotation as Rot

import rclpy
from rclpy.executors import MultiThreadedExecutor
import sys
import os

# 添加 elite_robot_example 包路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'elite_robot_example'))

from elite_robot_example.robot_cartesian_control import RobotCartesianControl


# ==========================================================
# 1. 基础配置 (与标定板生成参数、相机内参保持一致)
# ==========================================================
camera_matrix = np.array([[611.69867, 0.0, 326.48343],
                          [0.0, 610.4816, 242.60501],
                          [0.0, 0.0, 1.0]], dtype=np.float32)  # 替换为您的真实内参
dist_coeffs = np.zeros((5, 1), dtype=np.float32)

marker_size = 0.123  # 标定板物理尺寸 (米)

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
    # 3. 初始化 ROS2 节点
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
        rclpy.shutdown()
        return

    print("机械臂已连接！")
    robot_node.print_status()

    # ==========================================================
    # 4. 初始化相机与主循环
    # ==========================================================
    cap = cv2.VideoCapture(2)  # 使用 /dev/video2

    print("\n【运行提示】：")
    print("1. 让相机对准标定板。")
    print("2. 在画面窗口按下键盘 'G' 键，机械臂将尝试移动到标定板上方。")
    print("3. 按 'Q' 键退出程序。")

    # 存储检测到的目标位置
    detected_target = None
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

            # 绘制检测到的标定板
            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                # 调试：打印检测到的所有 ID
                print(f"检测到标记 IDs: {ids.flatten()}")
                for idx, marker_id in enumerate(ids.flatten()):
                    if marker_id == 0:  # 判定 ID 为 0
                        img_points = corners[idx][0]
                        success, rvec, tvec = cv2.solvePnP(
                            obj_points, img_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
                        )

                        if success:
                            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)

                            # 实时获取标定板在相机下的 3D 坐标
                            detected_p_cam = tvec.flatten()  # [X_c, Y_c, Z_c]
                            detected_target = True

                            # 在画面上显示提示
                            cv2.putText(frame, "Press 'G' to grasp", (10, 30),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    else:
                        print(f"跳过标记 ID: {marker_id} (需要 ID=2)")
            else:
                print("未检测到任何标记")

            # 显示画面
            cv2.imshow("Grasp Pipeline Testing", frame)

            # 等待按键
            key = cv2.waitKey(1) & 0xFF

            # 按 Q 退出
            if key == ord('q') or key == ord('Q'):
                print("退出程序")
                break

            # 按 G 触发抓取
            if key == ord('g') or key == ord('G'):
                if detected_target is None or detected_p_cam is None:
                    print("未检测到标定板，请先对准标定板！")
                    continue

                print("\n======= 触发视觉引导运动 =======")

                # 【步骤一】：获取机械臂当前的位姿
                print("1. 正在获取机械臂当前位姿...")

                # 先处理一下回调，确保获取最新数据
                for _ in range(10):
                    rclpy.spin_once(robot_node, timeout_sec=0.1)

                tcp_pose = robot_node.get_tcp_pose()

                if tcp_pose is None:
                    print("错误：无法获取机械臂当前位姿")
                    continue

                current_pos, current_ori = tcp_pose
                print(f"   当前位置: {current_pos}")
                print(f"   当前姿态: {current_ori}")

                # 【步骤二】：执行空间坐标系转换
                # A. 构造当前末端到基座的变换
                t_tool2base = np.array(current_pos).reshape(3, 1)
                R_tool2base = Rot.from_quat(current_ori).as_matrix()

                # B. 将点转换到末端坐标系： P_tool = R_cam2tool * P_cam + t_cam2tool
                P_c = detected_p_cam.reshape(3, 1)
                P_tool = np.dot(R_cam2tool, P_c) + t_cam2tool

                # C. 将点转换到基座坐标系： P_base = R_tool2base * P_tool + t_tool2base
                P_base = np.dot(R_tool2base, P_tool) + t_tool2base
                target_in_base = P_base.flatten()

                # 为了安全，我们让机械臂在目标上方 10 厘米（0.1米）处停下，避免直接撞击桌面
                safe_target_z = target_in_base[2] + 0.10

                print(f"2. 计算出目标在基座下的坐标: X={target_in_base[0]:.4f}, Y={target_in_base[1]:.4f}, Z={safe_target_z:.4f}")
                print("3. 正在向机械臂发送运动指令...")

                # 【步骤三】：使用 SDK 控制机械臂运动
                # 保持当前姿态，只改变位置
                target_position = [target_in_base[0], target_in_base[1], safe_target_z]
                target_orientation = current_ori  # 保持当前姿态

                success = robot_node.move_to_cartesian_pose(
                    target_position,
                    target_orientation,
                    duration=5.0
                )

                if success:
                    print("4. 机械臂运动完成！")
                else:
                    print("4. 机械臂运动失败！")

                print("================================\n")

                # 重置检测状态
                detected_target = None
                detected_p_cam = None

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        # 清理资源
        cap.release()
        cv2.destroyAllWindows()
        robot_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
