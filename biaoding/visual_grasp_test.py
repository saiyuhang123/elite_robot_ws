#!/usr/bin/env python3
"""
视觉引导抓取测试脚本（控制器原生 movej 版）

功能：
1. 检测 ArUco 标定板 (ID=0)
2. 通过手眼标定结果计算目标在基座坐标系下的位置
3. 通过 /script_sender/script_command 向控制器直接发 movej 脚本
   （不经过 MoveIt；IK 和轨迹规划都在控制器内部完成）
4. 支持夹爪控制

使用方法：
1. 启动机械臂驱动（本脚本依赖驱动的 script_sender 节点和 TF）：
   ros2 launch eli_cs_robot_driver elite_control.launch.py robot_ip:=192.168.1.212 cs_type:=cs66

2. 启动 ROS 相机（开深度并与彩色对齐，目标点坐标优先从深度图直接读取）：
   ros2 launch realsense2_camera rs_launch.py camera_namespace:=camera \
     enable_color:=true enable_depth:=true rgb_camera.color_profile:=1280x720x30 \
     depth_module.depth_profile:=640x480x30 align_depth.enable:=true

3. 运行本脚本：
   python3 visual_grasp_test.py

注意：movej 走 def 主脚本，会打断驱动的外部控制程序；脚本退出时会自动
调用 resend_external_script 恢复（也可以手动恢复后再用 MoveIt）。
"""

import cv2
import math
import numpy as np
import json
from scipy.spatial.transform import Rotation as Rot

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import sys
import os
import time
import signal

# 添加 elite_robot_example 包路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'elite_robot_example'))

from elite_robot_example.robot_cartesian_control import (
    RobotCartesianControl,
    cs66_inverse_kinematics_5dof,
    cs66_forward_kinematics,
)

# 手动补偿量（米）：改正 TCP/标定的固定偏移
# 正值为正方向偏移，试出来之后填这里，不用重新标定
MANUAL_OFFSET_X = 0.00  # 机器人基座 X 方向
MANUAL_OFFSET_Y = 0.00  # 机器人基座 Y 方向
MANUAL_OFFSET_Z = 0.00  # 机器人基座 Z 方向（上下准就保持 0）

# ---- 倾斜安装补偿 ----
# 基座倾斜安装，世界系"竖直向上"在基座系下的方向（实测值，倾斜角约48°）。
# 测量方法：物理法兰调水平后 tf2_echo cs66_base_link cs66_tool0，
# 取旋转矩阵第三列（法兰Z轴=世界的"下"）取反。
# 注意：不要用 cs66_tool0_controller（控制器上报的虚拟TCP）——它和物理
# 法兰差一个约118°的固定旋转（TCP偏移），姿态数据全是错的；位置不受影响。
# 预抓取点 = 目标点 + APPROACH_OFFSET_WORLD（世界系偏移）
V_UP_IN_BASE = np.array([-0.7431, 0.0120, 0.6691])


def _build_world_axes(v_up):
    """由世界的"上"构建正交的世界系 X/Y 轴（以基座 Y 为参考）。"""
    up = np.asarray(v_up, dtype=float)
    up /= np.linalg.norm(up)
    y = np.array([0.0, 1.0, 0.0])
    y = y - (y @ up) * up
    y /= np.linalg.norm(y)
    x = np.cross(y, up)
    return x, y, up


# 世界系 X/Y/Z 在基座系下的方向（X/Y 为世界系水平方向）
WORLD_X_IN_BASE, WORLD_Y_IN_BASE, _ = _build_world_axes(V_UP_IN_BASE)

# 接近偏移（世界系，米）：预抓取点 = 目标点 + 偏移
#   (0, 0, 0.20)  = 世界系正上方 20cm（默认）
#   (-0.20, 0, 0) = 世界系 -X 侧 20cm（侧向接近）
#   (0, 0, 0)     = 直接到目标点（验证到位精度用）
APPROACH_OFFSET_WORLD = np.array([-0.20, 0, 0])

# 工具偏移（米）：法兰面到夹爪掌心/指尖的距离，沿法兰 Z 轴（机械手约 11cm）。
# IK 求解的是法兰位置：法兰目标 = 指尖目标 - TOOL_TIP_LENGTH × 工具轴方向，
# 使【掌心/指尖】到达目标点而非法兰面。设为 0 = 规划到法兰面。
TOOL_TIP_LENGTH = 0.11

# 运动方式：本地 IK（scipy 数值法，当前关节角做初值取就近解）
# + 控制器原生 movej（/script_sender/script_command 发脚本）。
# 无碰撞检查；IK 失败或 FK 验证误差过大时不运动。
SCRIPT_TOPIC = "/script_sender/script_command"
MOVEJ_A = 1.0    # 关节加速度 (rad/s^2)
MOVEJ_V = 0.2    # 关节速度 (rad/s)，想更快加大

# 零位关节角（rad，H 键回零用）
HOME_JOINTS = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]

# 终点姿态策略（5 维 IK：只约束工具 Z 轴方向，绕法兰的自转放开）：
#   "down" - 工具 Z 轴朝世界系的"下"（法兰竖直朝下，从上方抓取，默认）
#   "side" - 工具 Z 轴朝世界系 +X 水平方向（从世界系 -X 侧水平伸过去抓）
#   "keep" - 保持当前工具 Z 轴方向
# （"fixed"/"fixed_nearest" 旧值等同 "down"）
GRASP_ORIENTATION_MODE = "side"

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
    """订阅相机话题，检测目标标记并解算其在相机坐标系下的位置。
    视角正(<50°)用 solvePnP（与标定同一测量体系）；斜视角用对齐深度图直接取点。"""

    def __init__(self):
        super().__init__('grasp_camera_detector')
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_frame = None
        self.latest_depth = None     # 16UC1，单位 mm，与彩色图对齐
        self.latest_p_cam = None
        self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self.camera_info_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/camera/camera/color/image_raw',
            self.image_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_cb, qos_profile_sensor_data)

    def camera_info_cb(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info(f'Camera info received ({msg.width}x{msg.height})')

    def depth_cb(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except Exception:
            pass

    def position_from_depth(self, u, v, fw, fh):
        """用对齐深度图取 (u,v) 处的 3D 坐标（小窗口中值滤波，去零值）"""
        if self.latest_depth is None or self.camera_matrix is None:
            return None
        d = self.latest_depth
        dh, dw = d.shape
        ud = int(u * dw / fw)
        vd = int(v * dh / fh)
        r = 7
        ud = min(max(ud, r), dw - r - 1)
        vd = min(max(vd, r), dh - r - 1)
        win = d[vd - r:vd + r + 1, ud - r:ud + r + 1].astype(np.float32)
        valid = win[win > 0]
        if valid.size < 20:
            return None
        z = float(np.median(valid)) / 1000.0
        if z < 0.15 or z > 3.0:
            return None
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])

    def image_cb(self, msg):
        if self.camera_matrix is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return
        fh, fw = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        self.latest_p_cam = None
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for idx, mid in enumerate(ids.flatten()):
                if mid == TARGET_MARKER_ID:
                    pts = corners[idx][0]
                    u = float(np.mean(pts[:, 0]))
                    v = float(np.mean(pts[:, 1]))

                    # 先跑 PnP（与手眼标定同一测量体系，正视时最准）
                    p_cam = None
                    source = None
                    face_angle_deg = None
                    ok, rvec, tvec = cv2.solvePnP(
                        obj_points, pts, self.camera_matrix, self.dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    if ok:
                        cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)
                        # 入射角：标记法线与"标记->相机"视线的夹角，越小越正视
                        normal = cv2.Rodrigues(rvec)[0][:, 2]
                        ray = -tvec.flatten() / np.linalg.norm(tvec)
                        face_angle_deg = float(np.degrees(np.arccos(np.clip(normal @ ray, -1, 1))))

                    # 视角正(<50°)用 PnP；斜视角 PnP 误差爆炸，换深度图直接取点
                    if ok and face_angle_deg is not None :  # and face_angle_deg < 50.0 去掉深度图
                        p_cam = tvec.flatten()
                        source = f'pnp {face_angle_deg:.0f}°'
                    else:
                        p_cam = self.position_from_depth(u, v, fw, fh)
                        if p_cam is not None:
                            source = f'depth{"" if face_angle_deg is None else f" {face_angle_deg:.0f}°"}'
                        elif ok:
                            # 深度无效，只能退回 PnP（斜视角下误差大，提示用户）
                            p_cam = tvec.flatten()
                            source = f'pnp {face_angle_deg:.0f}°(斜视角,慎用)'

                    if p_cam is not None:
                        self.latest_p_cam = p_cam
                        cv2.putText(frame,
                                    f"ID={TARGET_MARKER_ID}[{source}]: [{p_cam[0]:.3f}, {p_cam[1]:.3f}, {p_cam[2]:.3f}]",
                                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        cv2.putText(frame, "Press 'G' to grasp", (10, 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        self.latest_frame = frame


# ==========================================================
# 2. 控制器原生 movej：通过 /script_sender/script_command 发脚本
# ==========================================================
class ControllerMover(Node):
    """向控制器直接发送 movej 脚本（def 主脚本，IK/规划在控制器内部）。"""

    def __init__(self):
        super().__init__('grasp_script_mover')
        self.script_pub = self.create_publisher(String, SCRIPT_TOPIC, 10)

    def _send_script(self, script):
        msg = String()
        msg.data = script
        self.script_pub.publish(msg)
        self.get_logger().info(f"已发送脚本:\n{script}")

    def send_movej_joints(self, joints_rad, a=MOVEJ_A, v=MOVEJ_V):
        j = ", ".join(f"{x:.6f}" for x in joints_rad)
        self._send_script(f"def prog():\n    movej([{j}], a={a:.3f}, v={v:.3f}, r=0)\nend")

    def wait_motion_done(self, robot_node, timeout=30.0, settle_eps=0.0008):
        """轮询 TCP 位姿，连续多次基本不动视为运动结束。
        前 1 秒为宽限期（等脚本到达控制器、机器人启动），期间不算完成。"""
        start = time.time()
        last = None
        stable = 0
        while time.time() - start < timeout:
            for _ in range(5):
                rclpy.spin_once(robot_node, timeout_sec=0.02)
            tcp = robot_node.get_tcp_pose()
            if tcp is None:
                continue
            pos = np.array(tcp[0])
            if (last is not None and time.time() - start > 1.0
                    and np.linalg.norm(pos - last) < settle_eps):
                stable += 1
                if stable >= 5:
                    return True
            else:
                stable = 0
            last = pos
            time.sleep(0.05)
        return False

    def resend_external_script(self):
        """恢复驱动的外部控制程序（def 脚本会把它打断）。"""
        cli = self.create_client(Trigger, "/io_and_status_controller/resend_external_script")
        if not cli.wait_for_service(timeout_sec=2.0):
            return False
        future = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        return bool(resp and resp.success)


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

    # 就绪检查：本脚本【不】调用 power_on/brake_release！
    # 这两个 dashboard 命令会停止机器人侧正在运行的外部控制脚本，
    # 导致 controller_stopper 随即停用 scaled_joint_trajectory_controller
    # （表现为机械臂接受指令却纹丝不动）。
    # 上电/松闸请在启动前于示教器完成（驱动启动时也会自行上电），
    # 确认示教器显示 RUNNING 后再使用本脚本。
    print("\n提示：本脚本不执行上电/松闸（避免停止机器人侧脚本）。")
    print("      请确认示教器上机械臂处于 RUNNING 状态。")

    # 控制器 movej 脚本发送节点（只需驱动在跑，不需要 move_group）
    mover = ControllerMover()
    executor.add_node(mover)

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
        print("    enable_color:=true enable_depth:=true rgb_camera.color_profile:=1280x720x30 \\")
        print("    depth_module.depth_profile:=640x480x30 align_depth.enable:=true")
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
                #     （标定时用的是 URDF tool0 = 物理法兰系）
                #   - 末端→基座:  P_base = R_tool2base * P_tool + t_tool2base
                #     （因此这里也必须用物理法兰系的姿态 = FK；
                #      不能用控制器上报的 tool0_controller，它带 ~118° TCP 旋转偏移）
                # ----------------------------------------------------
                # A. 构造当前末端到基座的变换矩阵
                #    位置：控制器上报即可（TCP偏移平移为零，与FK一致）
                #    姿态：用当前关节角的 FK（物理法兰系，与手眼标定一致）
                q_now = robot_node.get_joint_positions()
                if q_now is None:
                    print("错误：无法获取当前关节角！")
                    continue
                _, R_tool2base = cs66_forward_kinematics(q_now)
                t_tool2base = np.array(current_pos).reshape(3, 1)

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

                # 接近偏移：沿【世界系】偏移（倾斜安装补偿），
                # (0,0,0) = 直接到目标点；非零 = 停在目标点世界系偏移处
                offset_base = (APPROACH_OFFSET_WORLD[0] * WORLD_X_IN_BASE +
                               APPROACH_OFFSET_WORLD[1] * WORLD_Y_IN_BASE +
                               APPROACH_OFFSET_WORLD[2] * V_UP_IN_BASE)
                if np.linalg.norm(offset_base) > 0:
                    target_in_base = target_in_base + offset_base

                print(f"2. 计算目标在基座坐标系: X={target_in_base[0]:.4f}, Y={target_in_base[1]:.4f}, Z={target_in_base[2]:.4f}"
                      f"（世界系偏移 {APPROACH_OFFSET_WORLD}）")

                # 可达性预检：目标到肩关节的斜距超过臂展就必然无解，直接给出人话提示  去掉
                # slant = np.linalg.norm(target_in_base - np.array([0.0, 0.0, SHOULDER_Z]))
                # if slant > ARM_REACH:
                #     print(f"   !! 目标不可达：距肩关节 {slant:.2f}m，超过 CS66 臂展约 {ARM_REACH}m")
                #     print("   !! 请把目标放近一点（建议距基座水平 0.7m 以内）或降低目标高度，本次不运动")
                #     continue

                # ----------------------------------------------------
                # 【步骤三】：本地 5 维 IK 解算关节角 + 控制器 movej
                #   只约束位置 + 工具轴方向（绕法兰自转放开，求解器自取最优），
                #   EliRobot 脚本的 movej([6个数]) 只认关节角，故先 IK 再发。
                # ----------------------------------------------------
                print("3. 正在本地 IK 解算并发送 movej 指令...")

                # 目标工具轴方向（基座系）：down = 世界系的"下"；
                # side = 世界系 +X（侧向水平抓取）；keep = 保持当前方向
                if GRASP_ORIENTATION_MODE in ("down", "fixed", "fixed_nearest"):
                    target_dir = -V_UP_IN_BASE
                    print(f"   终点方向: 工具轴朝世界系下方 {np.round(target_dir, 3)}（自转放开）")
                elif GRASP_ORIENTATION_MODE == "side":
                    target_dir = WORLD_X_IN_BASE
                    print(f"   终点方向: 工具轴朝世界系+X {np.round(target_dir, 3)}（水平侧抓）")
                else:
                    _, cur_rot = cs66_forward_kinematics(robot_node.get_joint_positions())
                    target_dir = cur_rot[:, 2]  # FK=物理法兰，当前工具Z轴方向
                    print("   终点方向: 保持当前工具轴方向")

                # 工具偏移：IK 求解的是法兰位置，让【掌心/指尖】到达目标点。
                # 法兰目标 = 指尖目标 - L × 工具轴方向
                # （down: 法兰在指尖上方 L；side: 法兰在指尖后方 L）
                flange_target = (target_in_base - TOOL_TIP_LENGTH * target_dir
                                 if TOOL_TIP_LENGTH > 0 else target_in_base)
                target_x, target_y, target_z = flange_target
                if TOOL_TIP_LENGTH > 0:
                    print(f"   法兰目标(指尖目标-L): [{target_x:.4f}, {target_y:.4f}, {target_z:.4f}]")

                move_dist = np.linalg.norm(
                    np.array([target_x, target_y, target_z]) - np.array(current_pos)
                )
                print(f"   运动距离: {move_dist:.3f}m")

                q_guess = robot_node.get_joint_positions()
                if q_guess is None:
                    print("4. 无法获取当前关节角，本次不运动")
                    continue

                # FK 模型 = URDF = 物理法兰坐标系，姿态天然一致，无需校正。
                # （控制器上报的 tool0_controller 带 TCP 旋转偏移，仅用于位置，
                #  位置偏移为零，不影(响)FK 对比）
                fk_pos_now, _ = cs66_forward_kinematics(q_guess)
                pos_drift = float(np.linalg.norm(fk_pos_now - np.array(current_pos)))
                if pos_drift > 0.01:
                    print(f"   !! 警告: FK 位置与控制器上报偏差 {pos_drift*1000:.1f}mm，模型可能不一致")

                joint_target = cs66_inverse_kinematics_5dof(
                    np.array([target_x, target_y, target_z]), target_dir, q_guess)
                if joint_target is None:
                    print("4. IK 解算失败，目标不可达，本次不运动")
                    continue

                # FK 验证 IK 结果：位置误差 + 工具轴方向误差（自转不检查）
                fk_pos, fk_rot = cs66_forward_kinematics(joint_target)
                pos_err = float(np.linalg.norm(fk_pos - np.array([target_x, target_y, target_z])))
                achieved_dir = fk_rot[:, 2]
                dir_err = math.degrees(math.acos(
                    float(np.clip(achieved_dir @ target_dir, -1.0, 1.0))))
                print(f"   IK 成功，位置误差: {pos_err:.4f}m, 方向误差: {dir_err:.2f}°")
                if pos_err > 0.02 or dir_err > 5.0:
                    print("4. IK 误差过大（目标可能不可达），本次不运动")
                    continue

                mover.send_movej_joints(joint_target)
                if mover.wait_motion_done(robot_node):
                    print("4. 机械臂已到达目标上方！")
                else:
                    print("4. 等待运动结束超时（看示教器是否有报错）")

                print("================================\n")

            # 按 C 关闭夹爪
            if key == ord('c') or key == ord('C'):
                print("关闭夹爪...")
                robot_node.close_gripper(pin=0)

            # 按 O 打开夹爪
            if key == ord('o') or key == ord('O'):
                print("打开夹爪...")
                robot_node.open_gripper(pin=0)

            # 按 H 回到零位（控制器 movej，关节角目标）
            if key == ord('h') or key == ord('H'):
                print("回到零位...")
                mover.send_movej_joints(HOME_JOINTS)
                if mover.wait_motion_done(robot_node):
                    print("已回到零位")
                else:
                    print("回零等待超时（看示教器是否有报错）")

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        # 清理资源；恢复驱动的外部控制程序（def 脚本把它打断了）
        cv2.destroyAllWindows()
        if mover.resend_external_script():
            print("已恢复驱动外部控制程序")
        cam_node.destroy_node()
        robot_node.destroy_node()
        mover.destroy_node()
        rclpy.shutdown()
        print("资源已清理，程序退出。")


if __name__ == '__main__':
    main()
