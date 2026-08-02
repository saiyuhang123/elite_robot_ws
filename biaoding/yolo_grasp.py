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
  最佳）→ 开启按需识别（/yolo_perception/set_enabled）→ 只用停稳后
  的新帧锁存目标 → 关闭识别 → 抓取，结束后无论成败都收拢到 Home2。
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
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool

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

# 抓取点整体下探偏移（世界系，米）：在目标点基础上再往下 0.5cm，
# 桌面模式与悬挂模式都生效（沿世界"下"方向 -V_UP_IN_BASE 平移）
GRASP_DOWN_OFFSET = 0.000

# 目标新鲜度门限（调度集成：防止抓旧目标或移动中的目标）
TARGET_MAX_AGE = 5.0        # 目标点最大龄期（秒），超龄视为无目标

MOVEJ_A, MOVEJ_V = 1.0, 0.2
MOVEL_A, MOVEL_V = 0.3, 0.05

# 力控下探（guarded move）：最后几厘米慢速分段下降，触力即停，
# 不再依赖精确的目标高度，防止压坏物体触发力报警
FT_TOPIC = '/force_torque_sensor_broadcaster/wrench'
FORCE_THRESHOLD = 2.0       # 硬保护阈值（N，力变化模长），任何方向大力即停
FORCE_PROJ_THRESHOLD = 1.2   # 软接触阈值（N，下压方向投影），轻触也能检出
FORCE_PROJ_HITS = 3          # 投影连续超阈值次数，滤毛刺
FORCE_APPROACH_H = 0.03      # 快速接近段终点 = 抓取点上方 3cm
FORCE_DIVE_OVERSHOOT = 0.03   # 下探过冲：力反馈是必须条件，给足竖直搜索深度
FORCE_DIVE_STEP = 0.008      # 分段下探步长 8mm（stopl 失效时过冲也不超一步）
FORCE_DIVE_V = 0.03          # 下探速度 m/s（越慢触力后过冲越小）
LIFT_BEFORE_CLOSE = 0.0120    # 触力后先上抬再闭合（米），避免收拢挤压物体触发力报警
# 闭合卸力：闭合过程中挤压力超阈值就自动上抬一点（防收拢时力控报警）
FORCE_RELIEF_THRESHOLD = 2.0  # 闭合挤压力阈值（N，力变化模长）
FORCE_RELIEF_STEP = 0.004     # 每次卸力上抬 4mm
FORCE_RELIEF_MAX = 0.02       # 卸力累计上抬上限 2cm

# 补拍精定位：首次估计后移动相机到更陡的视角再拍一次，用第二次结果抓取。
# 相机在臂展允许内尽量抬高（俯角大则检测点高度误差小），光轴对准首次估计点。
# 2026-08-02 停用：耗时长且副作用多，恢复时改回 True 即可。
RESHOOT_ENABLED = False
RESHOOT_DIST = 0.40           # 相机到目标的拍照距离（米）
RESHOOT_ELEVATIONS = [90, 70, 55]  # 俯角候选（度），逐个尝试直到 IK 可达
RESHOOT_SETTLE = 0.8          # 到位后停稳时间（秒）
# 手眼标定文件（与感知节点同一份），用于把相机位姿换算成法兰位姿
HAND_EYE_JSON = os.path.join(os.path.dirname(__file__), 'hand_eye_result.json')

# 抓取模式：'table'=桌面（力控下探）  'hanging'=悬挂（侧抓包络+拉拽摘取）
GRASP_MODE = 'table'
HANG_PRE_DIST = 0.12       # 预抓取点在目标后方（世界 X- 方向，米）
HANG_PALM_SIDE = 0.035      # 掌心在目标点右侧偏移（米，世界 Y-，手心朝左
                           # 从右侧包住果实；贴太紧调大、包不住调小）
HANG_PALM_BELOW = 0.0      # 掌心在目标点下方偏移（米，侧抓时一般取 0）
HANG_DETACH_PULL = 0.05    # 摘取下拉（米）
HANG_RESHOOT_DIST = 0.30   # 悬挂模式补拍距离（米，沿光轴近拍，当前未启用）

# 观察位姿：预备位姿等待 OBSERVE_WAIT 秒仍检测不到目标时，
# 转到该观察位姿寻找，检测到目标即停并在该位姿继续抓取
OBSERVE_POSES = [
    [-0.038397, 0.308923, -1.619919, -1.680604, 1.712094, 1.504874],
]
OBSERVE_WAIT = 3.0         # 观察位姿等待检测的时间（秒）

HOME_JOINTS = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
# 第二 Home 位姿（角度: -2.2, 19.7, -154.8, -86.3, 94.1, 84.2）
HOME2_JOINTS = [-0.0384, 0.3438, -2.7018, -1.5062, 1.6424, 1.4696]
# 抓取预备位姿（角度: -2.2, -38.3, -124.8, -16.3, 102.1, 94.2）
READY_JOINTS = [-0.0384, 0.4503, -2.7262, -0.1693, 1.6424, 1.4696]
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

        # ---- 末端力传感器（力控下探用）----
        self.latest_force = None
        self.latest_force_time = 0.0
        self.robot.create_subscription(
            WrenchStamped, FT_TOPIC, self._ft_cb, 10)

        # ---- 手眼标定（补拍位姿规划用：相机系→法兰系）----
        self.T_cam_to_tool = None
        if os.path.exists(HAND_EYE_JSON):
            try:
                import json as _json
                with open(HAND_EYE_JSON) as f:
                    calib = _json.load(f)
                self.T_cam_to_tool = np.eye(4)
                self.T_cam_to_tool[:3, :3] = np.array(calib['R_cam2tool'])
                self.T_cam_to_tool[:3, 3] = np.array(
                    calib['t_cam2tool']).flatten()
            except Exception as e:
                print(f"读取手眼标定 {HAND_EYE_JSON} 失败: {e}（补拍将禁用）")

        # 按需识别开关（感知节点默认关闭识别，抓取前才临时开启）
        self.perception_enable_cli = self.robot.create_client(
            SetBool, '/yolo_perception/set_enabled')

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

    # ---------------- 末端力传感器 ----------------
    def _ft_cb(self, msg):
        self.latest_force = np.array([
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        self.latest_force_time = time.time()

    def _force_baseline(self, duration=0.5):
        """静止时采力均值作零点（补偿残余零漂）。无数据返回 None。"""
        if self.latest_force is None or time.time() - self.latest_force_time > 1.0:
            return None
        samples = []
        start = time.time()
        while time.time() - start < duration:
            if self.latest_force is not None:
                samples.append(self.latest_force.copy())
            time.sleep(0.02)
        return np.mean(samples, axis=0) if samples else None

    def send_stopl(self, a=0.5):
        """急停当前直线运动。"""
        self._send(f"def prog():\n    stopl({a})\nend")

    def force_guided_descend(self, down_dir, max_dist, step=FORCE_DIVE_STEP,
                             v=FORCE_DIVE_V, threshold=FORCE_THRESHOLD,
                             contact_dir_flange=None,
                             proj_threshold=FORCE_PROJ_THRESHOLD):
        """慢速分段下探：每走一小步检查力变化，触力即停。
        停止判据（任一满足）：
          - 力变化模长 > threshold（硬保护，任意方向大力）
          - 力变化在接触方向上的投影 > proj_threshold 连续 FORCE_PROJ_HITS 次
            （软接触，轻触也能检出；contact_dir_flange 为接触力方向在法兰系
            下的单位向量，None 则关闭投影判据）
        返回 True=触力停止，False=走满行程未触力，None=出错。"""
        base = self._force_baseline()
        if base is None:
            print(f"   !! 力传感器无数据（{FT_TOPIC}）")
            return None
        print(f"   [力控] 零点 {np.round(base, 2)}，模长阈值 {threshold}N，"
              f"投影阈值 {proj_threshold}N，"
              f"步长 {step*1000:.0f}mm，行程 {max_dist*1000:.0f}mm")
        traveled = 0.0
        max_df = 0.0
        proj_min, proj_max = 0.0, 0.0
        proj_hits = 0
        while traveled < max_dist - 1e-6:
            d = min(step, max_dist - traveled)
            tcp = self.robot.get_tcp_pose()
            if tcp is None:
                return None
            nxt = np.array(tcp[0]) + d * down_dir
            if not self.send_movel_keep_orientation(nxt, a=0.3, v=v):
                return None
            # 等本步走完，中途触力立即停
            start = time.time()
            while time.time() - start < 10.0:
                if self.latest_force is not None:
                    dvec = self.latest_force - base
                    df = float(np.linalg.norm(dvec))
                    max_df = max(max_df, df)
                    if df > threshold:
                        print(f"   [力控] 硬触发 {df:.1f}N > {threshold}N，停止下探"
                              f"（已下探 {traveled*1000:.0f}mm）")
                        self.send_stopl()
                        time.sleep(0.3)
                        return True
                    if contact_dir_flange is not None:
                        proj = float(dvec @ contact_dir_flange)
                        proj_min = min(proj_min, proj)
                        proj_max = max(proj_max, proj)
                        # abs：传感器力方向约定不明，接触/拉扯都触发
                        proj_hits = proj_hits + 1 \
                            if abs(proj) > proj_threshold else 0
                        if proj_hits >= FORCE_PROJ_HITS:
                            print(f"   [力控] 软接触触发（投影 {proj:.1f}N "
                                  f"连续 {proj_hits} 次），停止下探"
                                  f"（已下探 {traveled*1000:.0f}mm）")
                            self.send_stopl()
                            time.sleep(0.3)
                            return True
                tcp2 = self.robot.get_tcp_pose()
                if tcp2 is not None and \
                        np.linalg.norm(np.array(tcp2[0]) - nxt) < 0.002:
                    break
                time.sleep(0.01)
            else:
                print("   !! 下探单步超时")
                return None
            traveled += d
        print(f"   [力控] 走满行程未触力，最大力变化 {max_df:.1f}N，"
              f"投影范围 [{proj_min:.1f}, {proj_max:.1f}]N")
        return False

    def close_with_force_relief(self, up_dir_base,
                                threshold=FORCE_RELIEF_THRESHOLD,
                                step=FORCE_RELIEF_STEP,
                                max_lift=FORCE_RELIEF_MAX):
        """闭合夹爪（攥紧段），闭合过程中监测挤压力（模长），超阈值就上抬
        一小段卸力。防止收拢时挤压物体导致力控报警。返回累计上抬量（米）。"""
        base = self._force_baseline(0.3)
        self.gripper.close()
        lifted = 0.0
        t_end = time.time() + self.gripper.close_delay
        while time.time() < t_end:
            if base is not None and self.latest_force is not None \
                    and lifted < max_lift:
                df = float(np.linalg.norm(self.latest_force - base))
                if df > threshold:
                    tcp = self.robot.get_tcp_pose()
                    if tcp is not None:
                        target = np.array(tcp[0]) + step * up_dir_base
                        print(f"   [卸力] 闭合挤压力 {df:.1f}N > {threshold}N，"
                              f"上抬 {step*1000:.0f}mm")
                        self.send_movel_keep_orientation(target, a=0.3, v=0.02)
                        self.wait_motion_done(timeout=5.0)
                        lifted += step
                        t_end = time.time() + 0.5  # 抬完留点时间让夹爪走完
            time.sleep(0.02)
        if lifted > 0:
            print(f"   [卸力] 累计上抬 {lifted*1000:.0f}mm")
        return lifted

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
    def _check_fresh_target(self):
        """取最后一帧感知结果：有新鲜目标（龄期 < TARGET_MAX_AGE）就返回
        最新一帧的 3D 目标点副本，否则返回 None。
        不做跨帧平均：多目标时感知每帧发布当帧"深度最近"的目标，帧间切换
        目标时平均会把不同苹果的位置混在一起导致抓取点跑偏。"""
        if self.latest_target is None:
            return None
        if time.time() - self.latest_target_time > TARGET_MAX_AGE:
            return None
        return self.latest_target.copy()

    # ---------------- 补拍精定位 ----------------
    def _plan_reshoot_pose(self, obj):
        """规划补拍关节角，按抓取模式生成候选相机位姿，逐个试 IK：
        table   - 相机在目标上方尽量陡（俯角候选自动降级），修正视角滑动；
        hanging - 相机在目标世界 X- 侧，光轴沿 X+ 平视近拍（挂果高，
                  俯视臂展不够，平视近拍同样能精化水平/高度）。
        返回关节角或 None。"""
        if self.T_cam_to_tool is None:
            return None
        tcp = self.robot.get_tcp_pose()
        if tcp is None:
            return None
        q_guess = self.robot.get_joint_positions()
        if q_guess is None:
            return None

        # 候选列表：(标签, 相机位姿 T_cam)。两种模式锚定轴不同：
        # 俯视时 z≈-V_UP 不能用 V_UP 投影（退化），平视时 z=WORLD_X
        # 不能用 WORLD_X 投影（退化），所以各造各的。
        candidates = []
        if GRASP_MODE == 'hanging':
            # 不旋转相机：保持当前相机姿态（本来就水平对着目标），
            # 只沿当前光轴平移到拍照距离，法兰几乎不用转
            R_fl = Rot.from_quat(tcp[1]).as_matrix()
            R_cam = R_fl @ self.T_cam_to_tool[:3, :3]
            z = R_cam[:, 2] / np.linalg.norm(R_cam[:, 2])   # 当前光轴
            T_cam = np.eye(4)
            T_cam[:3, :3] = R_cam
            T_cam[:3, 3] = obj - HANG_RESHOOT_DIST * z
            candidates.append(('原位姿态近拍', T_cam))
        else:
            # 水平方向取"目标指向当前相机一侧"：移动量小、臂展压力小
            h = np.array(tcp[0]) - obj
            h = h - (h @ V_UP_IN_BASE) * V_UP_IN_BASE
            if np.linalg.norm(h) < 1e-3:
                h = WORLD_X_IN_BASE.copy()
            h = h / np.linalg.norm(h)
            for el in RESHOOT_ELEVATIONS:
                elr = math.radians(el)
                cam_pos = obj + RESHOOT_DIST * (
                    math.sin(elr) * V_UP_IN_BASE + math.cos(elr) * h)
                z = obj - cam_pos
                z /= np.linalg.norm(z)      # 光轴对准目标
                x = WORLD_X_IN_BASE - (WORLD_X_IN_BASE @ z) * z
                if np.linalg.norm(x) < 1e-3:
                    continue
                x /= np.linalg.norm(x)
                y = np.cross(z, x)
                T_cam = np.eye(4)
                T_cam[:3, :3] = np.column_stack([x, y, z])
                T_cam[:3, 3] = cam_pos
                candidates.append((f'俯角{el}°', T_cam))

        T_tool_cam = np.linalg.inv(self.T_cam_to_tool)
        for label, T_cam in candidates:
            T_fl = T_cam @ T_tool_cam       # 相机位姿 → 法兰位姿
            joints = cs66_inverse_kinematics(T_fl[:3, 3], T_fl[:3, :3], q_guess)
            if joints is None:
                continue
            fk_pos, fk_rot = cs66_forward_kinematics(joints)
            pos_err = float(np.linalg.norm(fk_pos - T_fl[:3, 3]))
            rot_err = math.degrees(float(
                Rot.from_matrix(fk_rot.T @ T_fl[:3, :3]).magnitude()))
            if pos_err > 0.02 or rot_err > 5.0:
                continue
            if np.linalg.norm(T_fl[:3, 3]
                              - np.array([0, 0, SHOULDER_Z])) > ARM_REACH:
                continue
            print(f"   [补拍] {label} 可达，IK误差 "
                  f"{pos_err*1000:.1f}mm/{rot_err:.1f}°")
            return joints
        return None

    def _reshoot_refine(self, obj):
        """移动到补拍位姿重新检测目标。成功返回新目标点，失败返回 None
        （调用方退回用首次估计，流程不死）。移动过机械臂时置
        self._reshoot_moved，调用方负责抓前回预备位姿。"""
        self._reshoot_moved = False
        joints = self._plan_reshoot_pose(obj)
        if joints is None:
            print("   [补拍] 所有俯角候选不可达，沿用首次估计")
            return None
        print("   [补拍] 移动到补拍位姿...")
        self._reshoot_moved = True
        self.send_movej(joints)
        if not self.wait_motion_done():
            print("   [补拍] 移动超时，沿用首次估计")
            return None
        time.sleep(RESHOOT_SETTLE)
        # 等补拍位姿下的新检测帧
        self.latest_target = None
        start = time.time()
        while self.latest_target is None and time.time() - start < 3.0:
            time.sleep(0.05)
        if self.latest_target is None:
            print("   [补拍] 补拍位姿下未检测到目标，沿用首次估计")
            return None
        new_obj = self._check_fresh_target()
        if new_obj is None:
            print("   [补拍] 补拍位姿未检测到目标，沿用首次估计")
            return None
        diff = float(np.linalg.norm(new_obj - obj))
        print(f"   [补拍] 精定位 [{np.round(new_obj, 4)}]，"
              f"与首次估计差 {diff*1000:.1f}mm")
        return new_obj

    def _set_perception(self, enabled: bool) -> bool:
        """开关感知节点的按需识别。返回服务是否调用成功。"""
        if not self.perception_enable_cli.wait_for_service(timeout_sec=2.0):
            print("   !! 感知开关服务不可用（yolo_grasp_perception.py 未启动？）")
            return False
        req = SetBool.Request()
        req.data = bool(enabled)
        future = self.perception_enable_cli.call_async(req)
        # 节点由后台 MultiThreadedExecutor  spinning，轮询等待即可，
        # 不能再 spin_until_future_complete（一个节点挂两个执行器会互踩）
        start = time.time()
        while not future.done() and time.time() - start < 3.0:
            time.sleep(0.02)
        if not future.done():
            print("   !! 感知开关服务超时")
            return False
        return True

    def _wait_target(self, timeout):
        """清空旧目标，等待新检测结果。返回是否在超时内检测到。"""
        self.latest_target = None
        self.latest_target_time = 0.0
        start = time.time()
        while self.latest_target is None and time.time() - start < timeout:
            time.sleep(0.05)
        return self.latest_target is not None

    def _search_observe_poses(self):
        """依次转到观察位姿找目标，检测到即停（留在该位姿）。"""
        for i, pose in enumerate(OBSERVE_POSES):
            print(f"   [观察] 预备位姿无目标，转观察位姿 "
                  f"{i+1}/{len(OBSERVE_POSES)}...")
            self.send_movej(pose)
            if not self.wait_motion_done():
                print("   [观察] 移动超时，试下一个")
                continue
            time.sleep(0.5)  # 停稳
            if self._wait_target(OBSERVE_WAIT):
                print(f"   [观察] 位姿 {i+1} 检测到目标")
                return
        print("   [观察] 所有观察位姿均未检测到目标")

    def grasp(self):
        """执行一次抓取流程：先到预备位姿（相机视野最佳）→ 开启按需识别
        → 锁存目标后关闭识别 → 抓取 → 无论成败都收拢到 Home2。
        返回 (成功与否, 结果描述)。"""
        # 0. 先到抓取预备位姿，此位姿下相机视野最好
        print("0. movej 到抓取预备位姿...")
        self.send_movej(READY_JOINTS)
        if not self.wait_motion_done():
            print("   !! 到预备位姿超时，放弃")
            return False, "到预备位姿超时"
        self.spin(10)  # 等机械臂完全停稳

        # 1. 清空旧目标，开启按需识别：只用开启后拍的新帧，
        #    避免混入导航/摆臂过程中的旧检测结果导致抓取点跑偏
        self.latest_target = None
        self.latest_target_time = 0.0
        if not self._set_perception(True):
            return False, "感知节点未响应（识别未开启，放弃）"
        try:
            # 2. 默认（预备）位姿等 5s 检测；检测不到则遍历观察位姿，
            #    哪个位姿检测到就在哪个位姿继续抓取
            if not self._wait_target(5.0):
                self._search_observe_poses()
            if self.latest_target is None:
                ok, msg = False, "所有位姿均未检测到目标"
            else:
                ok, msg = self._grasp_impl()
        finally:
            # 目标已锁存（或抓取失败），关闭识别省算力
            self._set_perception(False)

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

        # 只使用最后一帧感知结果（不做跨帧平均）：
        # 多目标时感知每帧发布当帧"深度最近"的目标，若帧间切换目标，
        # 平均会把不同苹果的位置混在一起导致抓取点跑偏
        obj = self._check_fresh_target()
        if obj is None:
            print("没有目标（感知未检测到或目标超龄）")
            return False, "无目标"
        print(f"1. 目标点(基座系): [{obj[0]:.4f}, {obj[1]:.4f}, {obj[2]:.4f}]")

        # 1.5 补拍精定位（仅桌面模式）：移动到更陡的视角重拍一次，
        # 修正"检测点随视角在物体表面滑动"导致的高度/水平偏差
        if RESHOOT_ENABLED and GRASP_MODE != 'hanging':
            new_obj = self._reshoot_refine(obj)
            if new_obj is not None:
                obj = new_obj
                print(f"1'. 精定位目标点(基座系): [{obj[0]:.4f}, "
                      f"{obj[1]:.4f}, {obj[2]:.4f}]")
            if getattr(self, '_reshoot_moved', False):
                # 从拍照位姿直接 IK 预抓取点容易出翻转/奇异解，
                # 先回预备位姿再走正常抓取流程
                print("   [补拍] 回预备位姿...")
                self.go_ready()

        # 悬挂模式：走专用流程（下托包络 + 拉拽摘取，不用力控下探）
        if GRASP_MODE == 'hanging':
            return self._grasp_hanging(obj)

        # 1. 抓取点 = 目标点 + 夹爪定义的偏移
        g_off = gripper.grasp_offset_world
        offset_base = (g_off[0] * WORLD_X_IN_BASE +
                       g_off[1] * WORLD_Y_IN_BASE +
                       g_off[2] * V_UP_IN_BASE)
        grasp_tip = obj + offset_base - GRASP_DOWN_OFFSET * V_UP_IN_BASE
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

        # 4. movel 下降：先快速接近到抓取点上方，再慢速力控下探（触力即停）
        reach_flange = grasp_tip - L * tool_dir
        approach_tip = grasp_tip + FORCE_APPROACH_H * V_UP_IN_BASE
        approach_flange = approach_tip - L * tool_dir
        print(f"   抓取TCP: [{grasp_tip[0]:.4f}, {grasp_tip[1]:.4f}, "
              f"{grasp_tip[2]:.4f}]")
        print(f"5. movel 快速接近（抓取点上方 {FORCE_APPROACH_H*100:.0f}cm）...")
        if not self.send_movel_keep_orientation(approach_flange):
            print("   !! 无法读取当前位姿，放弃")
            return False, "无法读取当前位姿"
        if not self.wait_motion_done():
            print("   !! movel 接近超时，放弃")
            return False, "movel 接近超时"

        print("6. 慢速力控下探（触力即停）...")
        # 接触力方向（世界上）转到法兰系，用于软接触投影判据
        contact_dir_flange = grasp_rot.T @ V_UP_IN_BASE
        contact_dir_flange /= np.linalg.norm(contact_dir_flange)
        contact = self.force_guided_descend(
            -V_UP_IN_BASE, FORCE_APPROACH_H + FORCE_DIVE_OVERSHOOT,
            contact_dir_flange=contact_dir_flange)
        if contact is None:
            return False, "力传感器异常或无法读取位姿"
        if not contact:
            # 力反馈是抓取的必要条件：探到底都没力说明目标不在，不闭合
            print("   !! 下探到底未触到力，退回预抓取点")
            self.send_movel_keep_orientation(pre_flange)
            self.wait_motion_done()
            return False, "未触到物体（抓空）"

        # 触到力后上抬卸掉接触力，再进入闭合流程
        print(f"   [力控] 触力到位，上抬 {LIFT_BEFORE_CLOSE*1000:.0f}mm 再闭合...")
        tcp = self.robot.get_tcp_pose()
        if tcp is None:
            return False, "无法读取当前位姿"
        lift_flange = np.array(tcp[0]) + LIFT_BEFORE_CLOSE * V_UP_IN_BASE
        if not self.send_movel_keep_orientation(lift_flange):
            return False, "无法读取当前位姿"
        if not self.wait_motion_done():
            print("   !! 上抬超时，放弃")
            return False, "触力后上抬超时"

        # 5. 闭合（一次性攥紧，带力控卸力兜底）
        print(f"7. 闭合 [{gripper.name}]...")
        self.close_with_force_relief(V_UP_IN_BASE)

        # 6. movel 退回
        print("8. movel 退回...")
        if not self.send_movel_keep_orientation(pre_flange):
            print("   !! 无法读取当前位姿")
            return False, "退回失败（无法读取位姿，物体可能已夹住）"
        if not self.wait_motion_done():
            return False, "退回超时（物体可能已夹住，注意检查）"

        print(f"======= 抓取完成 [{gripper.name}] =======\n")
        return True, "抓取完成"

    # ---------------- 悬挂抓取（室内挂果） ----------------
    def _grasp_hanging(self, obj):
        """悬挂果实抓取（侧抓）：法兰面朝世界 X+ 水平伸出，手心朝世界左，
        从右侧包住果实 → 闭合 → 下拉摘取 → 退回。
        不用力控下探：果实无桌面支撑，一碰就让，力建立不起来。"""
        gripper = self.gripper
        print(f"\n--- 悬挂抓取流程 [{gripper.name}] ---")

        # 抓取姿态（固定）：法兰面（法兰Z）= 世界 X+，手沿 X+ 水平伸出；
        # 手心（法兰Y）= 世界左（Y+），从右侧包住果实
        z = WORLD_X_IN_BASE / np.linalg.norm(WORLD_X_IN_BASE)
        y = WORLD_Y_IN_BASE / np.linalg.norm(WORLD_Y_IN_BASE)
        x = np.cross(y, z)
        grasp_rot = np.column_stack([x, y, z])
        print(f"   法兰面(法兰Z): {np.round(z, 3)}  "
              f"手心(法兰Y): {np.round(y, 3)}")

        L = gripper.tool_length
        # 掌心 = 目标点右侧（手心朝左对着果实）+ 竖直微调
        grasp_tip = (obj - HANG_PALM_SIDE * y
                     - HANG_PALM_BELOW * V_UP_IN_BASE
                     - GRASP_DOWN_OFFSET * V_UP_IN_BASE)
        pre_tip = grasp_tip - HANG_PRE_DIST * z           # 预抓取：X- 后方
        reach_flange = grasp_tip - L * z
        pre_flange = pre_tip - L * z

        dist = float(np.linalg.norm(pre_flange - np.array([0, 0, SHOULDER_Z])))
        if dist > ARM_REACH:
            return False, f"目标不可达（距肩关节 {dist:.2f}m）"

        self.spin(10)
        q_guess = self.robot.get_joint_positions()
        if q_guess is None:
            return False, "无法获取当前关节角"
        joint_target = cs66_inverse_kinematics(pre_flange, grasp_rot, q_guess)
        if joint_target is None:
            return False, "IK 解算失败"
        fk_pos, fk_rot = cs66_forward_kinematics(joint_target)
        pos_err = float(np.linalg.norm(fk_pos - pre_flange))
        rot_err = math.degrees(float(
            Rot.from_matrix(fk_rot.T @ grasp_rot).magnitude()))
        print(f"   IK: 位置误差 {pos_err*1000:.1f}mm, 方向误差 {rot_err:.2f}°")
        if pos_err > 0.02 or rot_err > 5.0:
            return False, (f"IK 误差过大（位置 {pos_err*1000:.1f}mm，"
                           f"方向 {rot_err:.2f}°）")

        # 1. 张开，movej 到侧后方预抓取点
        print("1. 张开，movej 到预抓取点...")
        gripper.open()
        time.sleep(0.5)
        self.send_movej(joint_target)
        if not self.wait_motion_done():
            return False, "movej 运动超时"

        # 2. movel 水平前伸到托取位
        print("2. movel 前伸下托...")
        if not self.send_movel_keep_orientation(reach_flange, v=0.03):
            return False, "无法读取当前位姿"
        if not self.wait_motion_done():
            return False, "movel 前伸超时"

        # 3. 闭合（无桌面，直接握紧即可）
        print(f"3. 闭合 [{gripper.name}]...")
        gripper.close()
        time.sleep(gripper.close_delay)

        # 4. 摘取：movel 下拉拽断果柄（已注释：不需要下拉动作）
        # print("4. 摘取（下拉）...")
        # cur = np.array(self.robot.get_tcp_pose()[0])
        # if not self.send_movel_keep_orientation(
        #         cur - HANG_DETACH_PULL * V_UP_IN_BASE, v=0.03):
        #     return False, "摘取失败（无法读取位姿，物体可能已夹住）"
        # self.wait_motion_done()

        # 5. movel 退回预抓取点
        print("5. movel 退回...")
        if not self.send_movel_keep_orientation(pre_flange):
            return False, "退回失败（物体可能已夹住）"
        if not self.wait_motion_done():
            return False, "退回超时（物体可能已夹住，注意检查）"

        print(f"--- 悬挂抓取完成 [{gripper.name}] ---\n")
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
    print("  键盘: g=抓取  o=张开  c=闭合  p=打印目标  f=打印力  h=回零  2=Home2  r=预备位姿")
    print("        j=示教放置位姿  l=放置  e=开关持续识别(调试)  t=切换目标类别  q=退出")
    print("  ROS服务: /yolo_grasp/grasp /open /close /home /home2 /ready /place /status")
    print("  注意: 识别默认关闭（按需识别），抓取时自动临时开启；"
          "调试看图像/目标请先按 e 开启")
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
                    elif cmd == 'f':
                        if g.latest_force is None:
                            print(f"  无力数据（{FT_TOPIC}）")
                        else:
                            print(f"  力: {np.round(g.latest_force, 2)}N  "
                                  f"幅值: {np.linalg.norm(g.latest_force):.2f}N")
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
                    elif cmd == 'e':
                        g.perception_on = not getattr(g, 'perception_on', False)
                        if g._set_perception(g.perception_on):
                            print(f"  持续识别: {'开' if g.perception_on else '关'}")
                        else:
                            g.perception_on = False
                    elif cmd.startswith('t'):
                        parts = cmd.split(maxsplit=1)
                        cls = parts[1] if len(parts) > 1 else 'apple'
                        g.set_target_class(cls)
                    elif cmd == '':
                        pass
                    else:
                        print("  未知命令。g=抓取 o=张开 c=闭合 p=打印 f=打印力 "
                              "h=回零 2=Home2 r=预备 j=示教放置 l=放置 "
                              "e=开关识别 t=切换目标 q=退出")
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
