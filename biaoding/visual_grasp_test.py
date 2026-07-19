#!/usr/bin/env python3
"""
视觉引导抓取测试脚本（集成 Elite Robot SDK）

功能：
1. 检测 ArUco 标定板 (ID=0)
2. 通过手眼标定结果计算目标在基座坐标系下的位置
3. 使用 Elite Robot SDK (RobotCartesianControl) 控制机械臂运动到目标上方
4. 支持夹爪控制

使用方法：
1. 启动机械臂驱动：
   ros2 launch eli_cs_robot_driver elite_control.launch.py robot_ip:=192.168.1.212 cs_type:=cs66

2. 启动 ROS 相机（内参由驱动按分辨率自动匹配，与标定同一条链路）：
   ros2 launch realsense2_camera rs_launch.py camera_namespace:=camera \
     enable_color:=true enable_depth:=false rgb_camera.color_profile:=1280x720x30 align_depth.enable:=false

3. 运行本脚本：
   python3 visual_grasp_test.py
"""

import cv2
import numpy as np
import json
from scipy.spatial.transform import Rotation as Rot

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
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

# 手动补偿量（米）：改正 TCP/标定的固定偏移
# 正值为正方向偏移，试出来之后填这里，不用重新标定
MANUAL_OFFSET_X = 0.00  # 机器人基座 X 方向
MANUAL_OFFSET_Y = 0.00  # 机器人基座 Y 方向
MANUAL_OFFSET_Z = 0.00  # 机器人基座 Z 方向（上下准就保持 0）

# 接近高度（米）：移动到目标点"正上方"这么高；设为 0 = 直接移动到目标点（用于验证到位精度）
APPROACH_HEIGHT = 0.00

# CS66 几何参数（用于可达性预检，见 default_kinematics.yaml）
SHOULDER_Z = 0.1625   # 肩关节距基座高度
ARM_REACH = 0.92      # 大臂0.427 + 小臂0.3905 + 腕部余量，约等于最大臂展


# ==========================================================
# 1. ArUco 检测节点（走 ROS 相机话题，内参来自 camera_info，
#    与手眼标定/验证用的是同一条链路，分辨率自动匹配）
# ==========================================================
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


class CameraDetector(Node):
    """订阅相机话题，检测目标标记并解算其在相机坐标系下的位置"""

    def __init__(self):
        super().__init__('grasp_camera_detector')
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_frame = None
        self.latest_p_cam = None
        self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self.camera_info_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/camera/camera/color/image_raw',
            self.image_cb, qos_profile_sensor_data)

    def camera_info_cb(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info(f'Camera info received ({msg.width}x{msg.height})')

    def image_cb(self, msg):
        if self.camera_matrix is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        self.latest_p_cam = None
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for idx, mid in enumerate(ids.flatten()):
                if mid == TARGET_MARKER_ID:
                    ok, rvec, tvec = cv2.solvePnP(
                        obj_points, corners[idx][0], self.camera_matrix, self.dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    if ok:
                        cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)
                        self.latest_p_cam = tvec.flatten()
                        cv2.putText(frame,
                                    f"Target ID={TARGET_MARKER_ID}: [{tvec[0][0]:.3f}, {tvec[1][0]:.3f}, {tvec[2][0]:.3f}]",
                                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        cv2.putText(frame, "Press 'G' to grasp", (10, 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        self.latest_frame = frame


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
    # 4. 初始化相机检测节点（订阅 ROS 相机话题，需先启动相机驱动）
    # ==========================================================
    cam_node = CameraDetector()
    # 等 camera_info 到达（内参）
    for _ in range(100):  # 最多等 10 秒
        rclpy.spin_once(cam_node, timeout_sec=0.1)
        if cam_node.camera_matrix is not None:
            break
    if cam_node.camera_matrix is None:
        print("错误：收不到相机 camera_info，请检查相机驱动是否已启动：")
        print("  ros2 launch realsense2_camera rs_launch.py camera_namespace:=camera \\")
        print("    enable_color:=true enable_depth:=false rgb_camera.color_profile:=1280x720x30 align_depth.enable:=false")
        robot_node.destroy_node()
        cam_node.destroy_node()
        rclpy.shutdown()
        return
    print("相机已就绪（内参来自 camera_info，与标定链路一致）")

    print("\n【运行提示】：")
    print(f"1. 让相机对准标定板 (ID={TARGET_MARKER_ID})。")
    print("2. 在画面窗口按下键盘 'G' 键，机械臂将尝试移动到标定板上方。")
    print("3. 按 'C' 键关闭夹爪，按 'O' 键打开夹爪。")
    print("4. 按 'H' 键机械臂回到零位。")
    print("5. 按 'Q' 键退出程序。")

    try:
        while True:
            # 泵相机回调，刷新画面与检测结果
            rclpy.spin_once(cam_node, timeout_sec=0.001)
            frame = cam_node.latest_frame
            if frame is None:
                continue

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
                detected_p_cam = cam_node.latest_p_cam
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

                # 应用手动补偿
                target_in_base[0] += MANUAL_OFFSET_X
                target_in_base[1] += MANUAL_OFFSET_Y
                target_in_base[2] += MANUAL_OFFSET_Z

                # 接近高度：0 = 直接到目标点；>0 = 抬高到目标上方
                if APPROACH_HEIGHT > 0:
                    target_in_base[2] += APPROACH_HEIGHT

                print(f"2. 计算目标在基座坐标系: X={target_in_base[0]:.4f}, Y={target_in_base[1]:.4f}, Z={target_in_base[2]:.4f}")

                # 可达性预检：目标到肩关节的斜距超过臂展就必然无解，直接给出人话提示
                slant = np.linalg.norm(target_in_base - np.array([0.0, 0.0, SHOULDER_Z]))
                if slant > ARM_REACH:
                    print(f"   !! 目标不可达：距肩关节 {slant:.2f}m，超过 CS66 臂展约 {ARM_REACH}m")
                    print("   !! 请把目标放近一点（建议距基座水平 0.7m 以内）或降低目标高度，本次不运动")
                    continue

                # ----------------------------------------------------
                # 【步骤三】：通过 C++ SDK 工具发送笛卡尔运动指令
                #   保持当前姿态不变，只改变末端位置到目标上方
                # ----------------------------------------------------
                print("3. 正在通过 Elite SDK 发送笛卡尔运动指令...")

                # 先释放 ROS2 控制权，避免与 C++ 工具冲突
                print("   释放 ROS2 控制权...")
                robot_node._hand_back_control()
                time.sleep(3.0)  # 等 ROS2 dashboard 完全释放

                # 姿态保持当前不变，转为旋转向量
                rotvec = Rot.from_quat(current_ori).as_rotvec()

                target_x = target_in_base[0]
                target_y = target_in_base[1]
                target_z = target_in_base[2]

                move_dist = np.linalg.norm(
                    np.array([target_x, target_y, target_z]) - np.array(current_pos)
                )
                move_time = max(8.0, move_dist / 0.05)  # 慢速：0.05 m/s，至少 8 秒
                print(f"   运动距离: {move_dist:.3f}m, 预计时间: {move_time:.1f}s")

                # 构建 C++ 工具命令；先试笛卡尔，失败则换关节空间模式重开进程再试
                base_cmd = [
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
                joints = robot_node._joint_positions  # rad，JOINT_NAMES 顺序

                def run_move(mode):
                    cmd = base_cmd + [mode]
                    if mode == "joint":
                        cmd += [f"{j:.6f}" for j in joints]
                    print(f"   执行({mode}): {' '.join(cmd)}")
                    return subprocess.run(
                        cmd,
                        cwd=os.path.dirname(CARTESIAN_TOOL),
                        timeout=move_time + 30,
                        capture_output=True,
                        text=True
                    )

                try:
                    result = run_move("cartesian")
                    print(result.stdout)
                    if result.returncode != 0:
                        if joints is not None and len(joints) == 6:
                            print("   笛卡尔失败（多为奇异位姿），切换关节空间模式重试...")
                            result = run_move("joint")
                            print(result.stdout)
                        else:
                            print("   警告：拿不到当前关节角，无法切换关节空间模式")
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
        cv2.destroyAllWindows()
        cam_node.destroy_node()
        robot_node.destroy_node()
        rclpy.shutdown()
        print("资源已清理，程序退出。")


if __name__ == '__main__':
    main()
