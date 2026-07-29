#!/usr/bin/env python3
"""YOLO 视觉抓取主程序（支持多种夹爪）。

流程（按 G 触发）：
  1. 取 YOLO 感知节点发布的目标点（/target_object_pose，基座系）
  2. 抓取点 = 目标点 + 世界系偏移（由夹爪类型决定语义）
  3. 抓取姿态由夹爪定义（gripper.grasp_rotation）：直装夹爪法兰 Z
     朝下竖直抓；灵巧手法兰面朝世界 X+、手水平伸出、手心朝下
  4. movej 到预抓取点 = 目标点正上方 10cm（世界系）
  5. movel 竖直下降到抓取点
  6. 闭合夹爪
  7. movel 上升退回预抓取点
  8. movej 回零位（h 键）

前提：
  - 机械臂驱动已启动（start_robot.launch.py）
  - YOLO 感知节点已启动（YOLO/yolo_grasp_perception.py）
  - 对应夹爪的控制节点已启动

用法：
  cd ~/Documents/elite_robot_ws/biaoding
  python3 yolo_grasp.py                          # 默认 linkerhand
  python3 yolo_grasp.py --gripper two_finger     # 二指夹爪
  python3 yolo_grasp.py --gripper soft_touch     # 柔触三指
  python3 yolo_grasp.py --headless               # 无人值守：仅服务驱动

调度集成：
  调 /yolo_grasp/grasp（Trigger）触发抓取：先到预备位姿（相机视野
  最佳）→ 锁存目标 → 抓取，结束后无论成败都收拢到 Home2。
  response.success/message 为真实结果。
  调 /yolo_grasp/place（Trigger）执行放置：movej 到示教放置位姿
  （按 j 示教，存 place_pose.json）→ 张手放下 → 退回 Home2。
  底盘导航期间机械臂必须处于 Home2 收拢位姿（/yolo_grasp/home2）。
"""

import argparse
import math
import os
import sys
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation as Rot

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'elite_robot_example'))
from elite_robot_example.robot_cartesian_control import (
    RobotCartesianControl,
    cs66_inverse_kinematics,
    cs66_inverse_kinematics_5dof,
    cs66_forward_kinematics,
)

from grippers import create_gripper

# ---------------- 通用配置 ----------------
SCRIPT_TOPIC = "/script_sender/script_command"
TARGET_TOPIC = "/target_object_pose"

# 倾斜安装实测：世界系"上"在基座系下的方向（calibrate_vertical.py 可重测）
V_UP_IN_BASE = np.array([-0.7431, 0.0120, 0.6691])


def _build_world_axes(v_up):
    up = np.asarray(v_up, dtype=float)
    up /= np.linalg.norm(up)
    y = np.array([0.0, 1.0, 0.0])
    y = y - (y @ up) * up
    y /= np.linalg.norm(y)
    return np.cross(y, up), y, up


WORLD_X_IN_BASE, WORLD_Y_IN_BASE, _ = _build_world_axes(V_UP_IN_BASE)

# 预抓取点偏移（世界系，米）：目标点 + 偏移 = 预抓取点
PRE_GRASP_OFFSET_WORLD = np.array([0.0, 0.0, 0.10])

# 目标新鲜度/稳定性门限（调度集成：防止抓旧目标或移动中的目标）
TARGET_MAX_AGE = 5.0        # 目标点最大龄期（秒），超龄视为无目标
TARGET_STABLE_WINDOW = 1.2  # 稳定采样窗口（秒）
TARGET_STABLE_TOL = 0.05   # 窗口内允许的最大漂移（米）

MOVEJ_A, MOVEJ_V = 1.0, 0.2
MOVEL_A, MOVEL_V = 0.3, 0.05

HOME_JOINTS = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
# 第二 Home 位姿（角度: -2.2, 19.7, -154.8, -86.3, 94.1, 84.2）
HOME2_JOINTS = [-0.0384, 0.3438, -2.7018, -1.5062, 1.6424, 1.4696]
# 抓取预备位姿（角度: -2.2, -38.3, -124.8, -16.3, 102.1, 94.2）
READY_JOINTS = [-0.0384, -0.6685, -2.1782, -0.2845, 1.7820, 1.6441]
SHOULDER_Z = 0.1625
ARM_REACH = 0.92

# 放置位姿示教文件（关节角，按 j 示教，place 服务/ l 键使用）
PLACE_POSE_FILE = os.path.join(os.path.dirname(__file__), 'place_pose.json')


def quat_to_rotvec(x, y, z, w):
    """四元数 -> 旋转向量（movel 指令的姿态格式）。"""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    angle = 2.0 * math.acos(max(-1.0, min(1.0, w)))
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-9:
        return 0.0, 0.0, 0.0
    if angle > math.pi:
        angle -= 2.0 * math.pi
    return x / s * angle, y / s * angle, z / s * angle


class YoloGrasp:
    def __init__(self, gripper_name: str = "linkerhand"):
        self.robot = RobotCartesianControl()
        self.cb_group = ReentrantCallbackGroup()

        # ---- 夹爪（按名称创建）----
        self.gripper = create_gripper(gripper_name, self.robot)

        # ---- 控制器通信 ----
        self.script_pub = self.robot.create_publisher(String, SCRIPT_TOPIC, 10)

        # ---- 目标感知 ----
        self.target_class_pub = self.robot.create_publisher(
            String, '/yolo/target_class', 10)
        self.latest_target = None
        self.latest_target_time = 0.0
        self._locked_target = None
        self._target_locked = False
        self.robot.create_subscription(
            PoseStamped, TARGET_TOPIC, self._target_cb, 10)

        # ---- ROS 服务 ----
        self.robot.create_service(Trigger, '/yolo_grasp/grasp', self._srv_grasp,
                                  callback_group=self.cb_group)
        self.robot.create_service(Trigger, '/yolo_grasp/open', self._srv_open,
                                  callback_group=self.cb_group)
        self.robot.create_service(Trigger, '/yolo_grasp/close', self._srv_close,
                                  callback_group=self.cb_group)
        self.robot.create_service(Trigger, '/yolo_grasp/home', self._srv_home,
                                  callback_group=self.cb_group)
        self.robot.create_service(Trigger, '/yolo_grasp/home2', self._srv_home2,
                                  callback_group=self.cb_group)
        self.robot.create_service(Trigger, '/yolo_grasp/ready', self._srv_ready,
                                  callback_group=self.cb_group)
        self.robot.create_service(Trigger, '/yolo_grasp/place', self._srv_place,
                                  callback_group=self.cb_group)
        self.robot.create_service(Trigger, '/yolo_grasp/status', self._srv_status,
                                  callback_group=self.cb_group)

        # ---- 放置位姿（关节角，按 j 示教）----
        self.place_joints = None
        if os.path.exists(PLACE_POSE_FILE):
            try:
                import json as _json
                with open(PLACE_POSE_FILE) as f:
                    self.place_joints = list(_json.load(f))
                print(f"已从 {PLACE_POSE_FILE} 加载放置位姿")
            except Exception as e:
                print(f"读取 {PLACE_POSE_FILE} 失败: {e}（按 j 重新示教）")

    # ---------------- 放置位姿示教 ----------------
    def teach_place_pose(self):
        """把机械臂（手动/GUI）摆到放置位姿后调用，记录当前关节角。"""
        self.spin(10)
        q = self.robot.get_joint_positions()
        if q is None:
            print("  无法获取关节角，示教失败")
            return
        self.place_joints = [float(v) for v in q]
        import json as _json
        with open(PLACE_POSE_FILE, 'w') as f:
            _json.dump(self.place_joints, f)
        print(f"  放置位姿已记录到 {PLACE_POSE_FILE}")
        print(f"  关节角 = {np.round(self.place_joints, 4)}")


    # ---------------- ROS 服务回调 ----------------
    def _srv_grasp(self, request, response):
        self.robot.get_logger().info('[服务] 收到抓取指令')
        ok, msg = self.grasp()
        response.success = ok
        response.message = msg
        return response

    def _srv_open(self, request, response):
        self.robot.get_logger().info('[服务] 收到张开指令')
        self.gripper.open()
        response.success = True
        response.message = '已张开'
        return response

    def _srv_close(self, request, response):
        self.robot.get_logger().info('[服务] 收到闭合指令')
        self.gripper.close()
        response.success = True
        response.message = '已闭合'
        return response

    def _srv_home(self, request, response):
        self.robot.get_logger().info('[服务] 收到回零指令')
        self.home()
        response.success = True
        response.message = '已回零位'
        return response

    def _srv_home2(self, request, response):
        self.robot.get_logger().info('[服务] 收到回 Home2 指令')
        # 同步执行：调度方（mission_executor）需要等到位后再让底盘导航
        self.home2()
        response.success = True
        response.message = '已回 Home2 位姿'
        return response

    def _srv_ready(self, request, response):
        self.robot.get_logger().info('[服务] 收到预备位姿指令')
        self.go_ready()
        response.success = True
        response.message = '已到抓取预备位姿'
        return response

    def _srv_place(self, request, response):
        self.robot.get_logger().info('[服务] 收到放置指令')
        ok, msg = self.place()
        response.success = ok
        response.message = msg
        return response

    def _srv_status(self, request, response):
        if self.latest_target is None:
            response.success = False
            response.message = '无目标（感知节点未检测到物体）'
        else:
            t = self.latest_target
            age = time.time() - self.latest_target_time
            response.success = True
            response.message = (
                f'目标: [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}]（{age:.1f}s 前）')
        return response

    def set_target_class(self, class_name: str):
        msg = String()
        msg.data = class_name
        self.target_class_pub.publish(msg)
        print(f"  已发送目标类别切换: '{class_name}'")

    def _target_cb(self, msg):
        self.latest_target = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.latest_target_time = time.time()

    def spin(self, n=10, dt=0.05):
        # 后台 MultiThreadedExecutor 已在全程处理订阅/服务回调，
        # 这里不能再 rclpy.spin_once 同一节点（一个节点挂两个执行器会
        # 互踩 wait set，导致服务回调卡死），只需等待数据更新。
        time.sleep(n * dt)

    # ---------------- 运动原语 ----------------
    def send_movej(self, joints_rad, a=MOVEJ_A, v=MOVEJ_V):
        j = ", ".join(f"{x:.6f}" for x in joints_rad)
        self._send(f"def prog():\n    movej([{j}], a={a:.3f}, v={v:.3f}, r=0)\nend")

    def send_movel_keep_orientation(self, pos, a=MOVEL_A, v=MOVEL_V):
        self.spin(5)
        tcp = self.robot.get_tcp_pose()
        if tcp is None:
            return False
        q = tcp[1]
        rx, ry, rz = quat_to_rotvec(*q)
        p = ", ".join(f"{x:.6f}" for x in (*pos, rx, ry, rz))
        self._send(f"def prog():\n    movel([{p}], a={a:.3f}, v={v:.3f})\nend")
        return True

    def _send(self, script):
        msg = String()
        msg.data = script
        self.script_pub.publish(msg)
        print(f"  >> 已发送:\n{script}")

    def wait_motion_done(self, timeout=30.0, settle_eps=0.0008):
        start = time.time()
        last, stable = None, 0
        while time.time() - start < timeout:
            self.spin(5, 0.02)
            tcp = self.robot.get_tcp_pose()
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

    # ---------------- 抓取流程 ----------------
    def get_stable_target(self, window=TARGET_STABLE_WINDOW,
                          tol=TARGET_STABLE_TOL):
        """眼在手上：拍照位姿下快速确认目标新鲜即可，不要求运动过程中持续跟踪。
        短暂采样若干帧取均值（过滤单帧噪声），返回稳定目标均值（基座系）。"""
        samples = []
        start = time.time()
        while time.time() - start < window:
            t = self.latest_target
            age = time.time() - self.latest_target_time
            if t is not None and age < TARGET_MAX_AGE:
                samples.append(t.copy())
            time.sleep(0.05)
        if len(samples) < 2:
            print(f"   [诊断] 稳定采样: 窗口{window}s内仅{len(samples)}个有效样本")
            return None
        arr = np.array(samples)
        spread = float(np.max(np.linalg.norm(arr - arr.mean(axis=0), axis=1)))
        mean = arr.mean(axis=0)
        print(f"   [诊断] 稳定采样: OK, 漂移 {spread*1000:.1f}mm, "
              f"样本{len(samples)}个, 均值[{mean[0]:.3f},{mean[1]:.3f},{mean[2]:.3f}]")
        if spread > tol:
            print(f"   [诊断] 注意: 漂移偏大 ({spread*1000:.1f}mm > {tol*1000:.0f}mm)，"
                  f"但仍使用均值（眼在手上模式）")
        # 锁存目标，后续运动不再依赖感知
        self._locked_target = mean.copy()
        self._target_locked = True
        return mean

    def _check_fresh_target(self):
        """快速检查：是否有新鲜目标（不采样，不回 None 时直接返回最新值）。"""
        if self.latest_target is None:
            return None
        if time.time() - self.latest_target_time > TARGET_MAX_AGE:
            return None
        return self.latest_target.copy()

    def grasp(self):
        """执行一次抓取流程：先到预备位姿（相机视野最佳）→ 抓取 →
        无论成败都收拢到 Home2（底盘导航期间的安全姿态）。
        返回 (成功与否, 结果描述)。"""
        # 0. 先到抓取预备位姿，此位姿下相机视野最好
        print("0. movej 到抓取预备位姿...")
        self.send_movej(READY_JOINTS)
        if not self.wait_motion_done():
            print("   !! 到预备位姿超时，放弃")
            return False, "到预备位姿超时"
        self.spin(10)  # 等感知在新位姿下刷新几帧

        ok, msg = self._grasp_impl()

        # 无论成败，收拢到 Home2，保证底盘导航期间机械臂处于安全姿态
        self.home2()
        return ok, msg

    def _grasp_impl(self):
        """抓取流程本体（在预备位姿下调用）。返回 (成功与否, 结果描述)。"""
        gripper = self.gripper
        print(f"\n======= 开始抓取流程 [{gripper.name}] =======")

        # 0. 拍照位姿下锁存目标位置（眼在手上：后续运动不依赖感知持续跟踪）
        self._target_locked = False
        self._locked_target = None

        # 先快速检查是否有新鲜目标
        obj = self._check_fresh_target()
        if obj is None:
            print("没有目标（感知未检测到或目标超龄）")
            return False, "无目标"

        # 短暂采样取均值，过滤单帧噪声（不要求全程稳定）
        obj = self.get_stable_target()
        if obj is None:
            print("目标采样不足（感知帧率过低或窗口内目标丢失次数过多）")
            return False, "目标采样不足"
        print(f"1. 目标点(基座系): [{obj[0]:.4f}, {obj[1]:.4f}, {obj[2]:.4f}]")

        # 1. 抓取点 = 目标点 + 夹爪定义的偏移
        g_off = gripper.grasp_offset_world
        offset_base = (g_off[0] * WORLD_X_IN_BASE +
                       g_off[1] * WORLD_Y_IN_BASE +
                       g_off[2] * V_UP_IN_BASE)
        grasp_tip = obj + offset_base
        print(f"2. 抓取点(TCP): [{grasp_tip[0]:.4f}, {grasp_tip[1]:.4f}, "
              f"{grasp_tip[2]:.4f}]（偏移 {g_off}）")

        # 2. 预抓取点
        pre_offset = (PRE_GRASP_OFFSET_WORLD[0] * WORLD_X_IN_BASE +
                      PRE_GRASP_OFFSET_WORLD[1] * WORLD_Y_IN_BASE +
                      PRE_GRASP_OFFSET_WORLD[2] * V_UP_IN_BASE)
        pre_tip = obj + pre_offset

        # 抓取姿态由夹爪定义：直装夹爪法兰 Z 朝下竖直抓；
        # 灵巧手法兰面朝世界 X+、手水平伸出、手心朝下
        grasp_rot = gripper.grasp_rotation(WORLD_X_IN_BASE, V_UP_IN_BASE)
        tool_dir = grasp_rot[:, 2]   # 法兰 Z 轴 = 工具伸出方向
        L = gripper.tool_length
        print(f"   [诊断] 法兰Z轴: {np.round(tool_dir, 3)}  "
              f"法兰Y轴: {np.round(grasp_rot[:, 1], 3)}  "
              f"长度: {L:.3f}m")

        pre_flange = pre_tip - L * tool_dir
        print(f"3. 预抓取点(法兰): [{pre_flange[0]:.4f}, {pre_flange[1]:.4f}, "
              f"{pre_flange[2]:.4f}]")

        dist = float(np.linalg.norm(pre_flange - np.array([0, 0, SHOULDER_Z])))
        if dist > ARM_REACH:
            print(f"   !! 距肩关节 {dist:.2f}m 超臂展，放弃")
            return False, f"目标不可达（距肩关节 {dist:.2f}m）"

        self.spin(10)
        q_guess = self.robot.get_joint_positions()
        if q_guess is None:
            print("   !! 无法获取当前关节角，放弃")
            return False, "无法获取当前关节角"

        # 根据夹爪类型选择 IK
        ik_func = (cs66_inverse_kinematics_5dof if gripper.ik_mode == "5dof"
                   else cs66_inverse_kinematics)
        if gripper.ik_mode == "5dof":
            joint_target = ik_func(pre_flange, tool_dir, q_guess)
        else:
            joint_target = ik_func(pre_flange, grasp_rot, q_guess)

        if joint_target is None:
            print("   !! IK 解算失败，放弃")
            return False, "IK 解算失败"
        fk_pos, fk_rot = cs66_forward_kinematics(joint_target)
        pos_err = float(np.linalg.norm(fk_pos - pre_flange))
        if gripper.ik_mode == "6dof":
            rot_err = math.degrees(float(
                Rot.from_matrix(fk_rot.T @ grasp_rot).magnitude()))
        else:
            rot_err = math.degrees(math.acos(float(
                np.clip(fk_rot[:, 2] @ tool_dir, -1.0, 1.0))))
        print(f"   IK({gripper.ik_mode}): 位置误差 {pos_err*1000:.1f}mm,  "
              f"方向误差 {rot_err:.2f}°")
        if pos_err > 0.02 or rot_err > 5.0:
            print("   !! IK 误差过大，放弃")
            return False, f"IK 误差过大（位置 {pos_err*1000:.1f}mm，方向 {rot_err:.2f}°）"

        # 3. 张开，movej 到预抓取点
        print(f"4. 张开 [{gripper.name}]，movej 到预抓取点...")
        gripper.open()
        time.sleep(0.5)
        self.send_movej(joint_target)
        if not self.wait_motion_done():
            print("   !! 运动超时，放弃")
            return False, "movej 运动超时"

        self.spin(5)
        actual_tcp = self.robot.get_tcp_pose()
        if actual_tcp is not None:
            tcp_err = np.linalg.norm(np.array(actual_tcp[0]) - pre_flange)
            print(f"   [诊断] movej 后偏差: {tcp_err*1000:.1f}mm")

        # 4. movel 下降
        reach_flange = grasp_tip - L * tool_dir
        print(f"   TCP目标: [{grasp_tip[0]:.4f}, {grasp_tip[1]:.4f}, "
              f"{grasp_tip[2]:.4f}]")
        print(f"   法兰目标: [{reach_flange[0]:.4f}, {reach_flange[1]:.4f}, "
              f"{reach_flange[2]:.4f}]")
        print("5. movel 下降...")
        if not self.send_movel_keep_orientation(reach_flange):
            print("   !! 无法读取当前位姿，放弃")
            return False, "无法读取当前位姿"
        if not self.wait_motion_done():
            print("   !! movel 超时，放弃")
            return False, "movel 下降超时"

        self.spin(5)
        actual_tcp = self.robot.get_tcp_pose()
        if actual_tcp is not None:
            tcp_err = np.linalg.norm(np.array(actual_tcp[0]) - reach_flange)
            print(f"   [诊断] movel 后偏差: {tcp_err*1000:.1f}mm")

        # 5. 闭合
        print(f"6. 闭合 [{gripper.name}]...")
        gripper.close()
        time.sleep(gripper.close_delay)

        # 6. movel 退回
        print("7. movel 退回...")
        if not self.send_movel_keep_orientation(pre_flange):
            print("   !! 无法读取当前位姿")
            return False, "退回失败（无法读取位姿，物体可能已夹住）"
        if not self.wait_motion_done():
            return False, "退回超时（物体可能已夹住，注意检查）"

        print(f"======= 抓取完成 [{gripper.name}] =======\n")
        return True, "抓取完成"

    def home(self):
        print("回零位...")
        self.send_movej(HOME_JOINTS)
        if self.wait_motion_done():
            print("已回零")

    def home2(self):
        print("回 Home2 位姿...")
        self.send_movej(HOME2_JOINTS)
        if self.wait_motion_done():
            print("已到 Home2 位姿")

    def go_ready(self):
        print("移动到抓取预备位姿...")
        self.send_movej(READY_JOINTS)
        if self.wait_motion_done():
            print("已到预备位姿")

    def place(self):
        """移动到示教放置位姿 → 张手放下 → 退回 Home2 收拢位姿。返回 (成功与否, 描述)。"""
        print(f"\n======= 开始放置流程 [{self.gripper.name}] =======")
        if self.place_joints is None:
            print("   !! 放置位姿未示教，请先按 j 示教")
            return False, "放置位姿未示教"

        # 1. movej 到放置位姿
        print("1. movej 到放置位姿...")
        self.send_movej(self.place_joints)
        if not self.wait_motion_done():
            print("   !! 运动超时，放弃")
            return False, "movej 到放置位姿超时"

        # 2. 张手放下
        print(f"2. 张开 [{self.gripper.name}] 放下物体...")
        self.gripper.open()
        time.sleep(self.gripper.close_delay)

        # 3. 退回 Home2 收拢位姿（底盘导航期间的安全姿态）
        print("3. 退回 Home2 位姿...")
        self.send_movej(HOME2_JOINTS)
        if not self.wait_motion_done():
            return False, "退回 Home2 超时（物体已放下）"

        print(f"======= 放置完成 [{self.gripper.name}] =======\n")
        return True, "放置完成"

    def resend_external_script(self):
        cli = self.robot.create_client(
            Trigger, "/io_and_status_controller/resend_external_script")
        if cli.wait_for_service(timeout_sec=2.0):
            future = cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self.robot, future)


def main():
    parser = argparse.ArgumentParser(description="YOLO 视觉抓取主程序")
    parser.add_argument("--gripper", default="linkerhand",
                        choices=["linkerhand", "two_finger", "soft_touch"],
                        help="夹爪类型（默认 linkerhand）")
    parser.add_argument("--headless", action="store_true",
                        help="无人值守模式：跳过键盘交互，仅 ROS 服务驱动")
    args = parser.parse_args()

    rclpy.init()
    g = YoloGrasp(gripper_name=args.gripper)

    if not g.robot.wait_for_state(timeout=10.0):
        print("错误：收不到机械臂状态，请确认驱动已启动")
        g.robot.destroy_node()
        rclpy.shutdown()
        return

    # 后台线程：处理 ROS 服务请求
    executor = MultiThreadedExecutor()
    executor.add_node(g.robot)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # 初始化夹爪
    g.gripper.setup()
    g.gripper.validate()

    print("=" * 60)
    print(f"YOLO 抓取主程序（夹爪: {g.gripper.name}）")
    print(f"  IK: {g.gripper.ik_mode}  偏移: {g.gripper.grasp_offset_world}")
    print(f"  工具长度: {g.gripper.tool_length:.3f}m")
    print("  键盘: g=抓取  o=张开  c=闭合  p=打印目标  h=回零  2=Home2  r=预备位姿")
    print("        j=示教放置位姿  l=放置  t=切换目标类别  q=退出")
    print("  ROS服务: /yolo_grasp/grasp /open /close /home /home2 /ready /place /status")
    print("=" * 60)

    try:
        if args.headless:
            print("headless 模式：仅 ROS 服务驱动，Ctrl+C 退出")
            try:
                while True:
                    time.sleep(1.0)
            except KeyboardInterrupt:
                pass
        else:
            try:
                while True:
                    g.spin(2)
                    try:
                        cmd = input("cmd> ").strip().lower()
                    except EOFError:
                        break
                    if cmd == 'q':
                        break
                    elif cmd == 'g':
                        ok, msg = g.grasp()
                        print(f"  结果: {'成功' if ok else '失败'} - {msg}")
                    elif cmd == 'o':
                        g.gripper.open()
                        print("  已张开")
                    elif cmd == 'c':
                        g.gripper.close()
                        print("  已闭合")
                    elif cmd == 'p':
                        if g.latest_target is None:
                            print("  无目标")
                        else:
                            print(f"  目标: {np.round(g.latest_target, 4)}"
                                  f"（{time.time()-g.latest_target_time:.1f}s 前）")
                    elif cmd == 'h':
                        g.home()
                    elif cmd == '2':
                        g.home2()
                    elif cmd == 'r':
                        g.go_ready()
                    elif cmd == 'j':
                        g.teach_place_pose()
                    elif cmd == 'l':
                        ok, msg = g.place()
                        print(f"  结果: {'成功' if ok else '失败'} - {msg}")
                    elif cmd.startswith('t'):
                        parts = cmd.split(maxsplit=1)
                        cls = parts[1] if len(parts) > 1 else 'apple'
                        g.set_target_class(cls)
                    elif cmd == '':
                        pass
                    else:
                        print("  未知命令。g=抓取 o=张开 c=闭合 p=打印 "
                              "h=回零 2=Home2 r=预备 j=示教放置 l=放置 "
                              "t=切换目标 q=退出")
            except KeyboardInterrupt:
                pass
    finally:
        executor.shutdown()
        g.resend_external_script()
        g.robot.destroy_node()
        rclpy.shutdown()
        print("已退出（外部控制程序已恢复）")


if __name__ == '__main__':
    main()
