#!/usr/bin/env python3
"""YOLO 视觉抓取主程序（机械手版）。

流程（按 G 触发）：
  1. 取 YOLO 感知节点发布的目标点（/target_object_pose，基座系）
  2. 抓取点 = 目标点 + 世界系偏移（默认上方 1cm，防碰）
  3. movej 到预抓取点 = 目标点正上方 10cm（世界系），
     工具轴朝世界系 +X（法兰面朝向），掌心到达（含 11cm 工具偏移）
  4. movel 竖直下降到抓取点
  5. 闭合机械手（LinkerHand O6）
  6. movel 上升退回预抓取点
  7. movej 回零位（h 键）

前提：
  - 机械臂驱动已启动（start_robot.launch.py）
  - YOLO 感知节点已启动（YOLO/yolo_grasp_perception.py）

用法：
  cd ~/Documents/elite_robot_ws/biaoding
  python3 yolo_grasp.py
  按键: g=抓取  p=打印当前目标  h=回零  q=退出
"""

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
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'elite_robot_example'))
from elite_robot_example.robot_cartesian_control import (
    RobotCartesianControl,
    cs66_inverse_kinematics,
    cs66_inverse_kinematics_5dof,
    cs66_forward_kinematics,
)

# ---------------- 配置 ----------------
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

# 抓取点偏移（世界系，米）：目标点 + 偏移 = 抓取点（掌心到达处）
# 负 Z = 掌心低于物体表面（包住物体），值根据物体大小调整
# 苹果 ~8cm → -0.04；小物件 → -0.02；大物件 → -0.06
GRASP_OFFSET_WORLD = np.array([0.0, 0.0, 0.04])

# 预抓取点偏移（世界系，米）：目标点 + 偏移 = 预抓取点
# (0, 0, 0.10) = 目标正上方 10cm，movej 到此后再 movel 竖直下降
PRE_GRASP_OFFSET_WORLD = np.array([0.0, 0.0, 0.10])

# 工具轴方向（世界系）：手心朝下抓取 = 世界的"下"
TOOL_AXIS_DIR = -V_UP_IN_BASE / np.linalg.norm(V_UP_IN_BASE)

# 抓取姿态（完整旋转矩阵，法兰朝世界系+X 且手心朝下）。
# 启动后按 k 键示教：把机械臂摆到目标姿态（法兰朝+X、手心朝下），
# 自动记录当前 FK 旋转矩阵并存入 grasp_orientation.json；
# 也可以把测得的 3x3 矩阵直接写在这里（跳过示教）。
GRASP_TARGET_ROT = None
GRASP_ROT_FILE = os.path.join(os.path.dirname(__file__), 'grasp_orientation.json')

# 工具偏移（米）：法兰面到掌心，沿手心法线方向
TOOL_TIP_LENGTH = 0.13

MOVEJ_A, MOVEJ_V = 1.0, 0.2    # movej 关节加速度/速度 (rad/s^2, rad/s)
MOVEL_A, MOVEL_V = 0.3, 0.05   # movel 加速度/速度 (m/s^2, m/s)

HOME_JOINTS = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
# 抓取预备位姿（角度: -2.2, -38.3, -124.8, -16.3, 102.1, 94.2）
READY_JOINTS = [-0.0384, -0.6685, -2.1782, -0.2845, 1.7820, 1.6441]
SHOULDER_Z = 0.1625
ARM_REACH = 0.92

# ---------------- LinkerHand O6 机械手 ----------------
# 控制 topic：position 0~255，0=弯曲(闭合)，255=伸直(张开)
# 关节顺序: [大拇指弯曲, 大拇指横摆, 食指, 中指, 无名指, 小拇指]
HAND_CMD_TOPIC = "/cb_right_hand_control_cmd"
HAND_SETTING_TOPIC = "/cb_right_hand_setting_cmd"
HAND_OPEN_POSE = [255.0] * 6                    # 五指张开
HAND_CLOSE_POSE = [0.0, 25.0, 0.0, 0.0, 0.0, 0.0]  # 握拳（大拇指横摆 70，便于包握）
HAND_SPEED = 30        # 速度 0~255
HAND_TORQUE = 80       # 扭矩上限 0~255（太小夹不住，太大伤物体）

GRIPPER_CLOSE_DELAY = 1.5   # 闭合机械手后的等待时间（秒）


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
    def __init__(self):
        self.robot = RobotCartesianControl()
        self.cb_group = ReentrantCallbackGroup()
        self.script_pub = self.robot.create_publisher(String, SCRIPT_TOPIC, 10)
        # LinkerHand 控制
        self.hand_pub = self.robot.create_publisher(JointState, HAND_CMD_TOPIC, 10)
        self.hand_setting_pub = self.robot.create_publisher(String, HAND_SETTING_TOPIC, 10)
        # 目标类别发布（发给 YOLO 感知节点）
        self.target_class_pub = self.robot.create_publisher(String, '/yolo/target_class', 10)
        self.latest_target = None       # 最新的目标点（基座系，米）
        self.latest_target_time = 0.0
        self.robot.create_subscription(
            PoseStamped, TARGET_TOPIC, self._target_cb, 10)

        # ---- ROS 服务（可通过命令行远程调用）----
        self.robot.create_service(Trigger, '/yolo_grasp/grasp', self._srv_grasp,
                                  callback_group=self.cb_group)
        self.robot.create_service(Trigger, '/yolo_grasp/open', self._srv_open,
                                  callback_group=self.cb_group)
        self.robot.create_service(Trigger, '/yolo_grasp/close', self._srv_close,
                                  callback_group=self.cb_group)
        self.robot.create_service(Trigger, '/yolo_grasp/home', self._srv_home,
                                  callback_group=self.cb_group)
        self.robot.create_service(Trigger, '/yolo_grasp/ready', self._srv_ready,
                                  callback_group=self.cb_group)
        self.robot.create_service(Trigger, '/yolo_grasp/status', self._srv_status,
                                  callback_group=self.cb_group)

        # ---- 抓取姿态（完整旋转矩阵）----
        self.grasp_rot = GRASP_TARGET_ROT
        if self.grasp_rot is None and os.path.exists(GRASP_ROT_FILE):
            try:
                import json as _json
                with open(GRASP_ROT_FILE) as f:
                    self.grasp_rot = np.array(_json.load(f), dtype=float)
                print(f"已从 {GRASP_ROT_FILE} 加载抓取姿态")
            except Exception as e:
                print(f"读取 {GRASP_ROT_FILE} 失败: {e}（按 k 重新示教）")

    # ---------------- 抓取姿态示教 ----------------
    def calibrate_grasp_orientation(self):
        """把机械臂摆到目标抓取姿态（法兰朝+X、手心朝下）后调用：
        记录当前 FK 旋转矩阵作为 IK 的完整姿态目标。"""
        self.spin(10)
        q = self.robot.get_joint_positions()
        if q is None:
            print("  无法获取关节角，示教失败")
            return
        _, r_flange = cs66_forward_kinematics(q)
        self.grasp_rot = r_flange
        import json as _json
        with open(GRASP_ROT_FILE, 'w') as f:
            _json.dump([[float(v) for v in row] for row in r_flange], f)
        print(f"  抓取姿态已记录（FK 旋转矩阵），保存到 {GRASP_ROT_FILE}")
        print(f"  法兰Z方向(基座系) = {np.round(r_flange[:, 2], 3)}（应≈世界+X方向）")

    # ---------------- LinkerHand 控制 ----------------
    def hand_open(self):
        self._hand_cmd(HAND_OPEN_POSE)

    def hand_close(self):
        self._hand_cmd(HAND_CLOSE_POSE)

    def hand_setup(self):
        """设置速度和扭矩上限（SDK 运行期间有效）。"""
        import json as _json
        for cmd, params in (("set_speed", {"speed": [HAND_SPEED] * 6}),
                            ("set_torque", {"torque": [HAND_TORQUE] * 6})):
            msg = String()
            msg.data = _json.dumps({"setting_cmd": cmd, "params": params})
            self.hand_setting_pub.publish(msg)

    def _hand_cmd(self, positions):
        msg = JointState()
        msg.position = [float(p) for p in positions]
        self.hand_pub.publish(msg)

    # ---------------- ROS 服务回调 ----------------
    def _srv_grasp(self, request, response):
        """服务 /yolo_grasp/grasp：触发一次抓取流程"""
        self.robot.get_logger().info('[服务] 收到抓取指令')
        # 在回调里直接执行抓取（会阻塞直到完成）
        self.grasp()
        response.success = True
        response.message = '抓取流程已完成'
        return response

    def _srv_open(self, request, response):
        """服务 /yolo_grasp/open：张开机械手"""
        self.robot.get_logger().info('[服务] 收到张开手指令')
        self.hand_open()
        response.success = True
        response.message = '机械手已张开'
        return response

    def _srv_close(self, request, response):
        """服务 /yolo_grasp/close：闭合机械手"""
        self.robot.get_logger().info('[服务] 收到闭合手指令')
        self.hand_close()
        response.success = True
        response.message = '机械手已闭合'
        return response

    def _srv_home(self, request, response):
        """服务 /yolo_grasp/home：回零位"""
        self.robot.get_logger().info('[服务] 收到回零指令')
        self.home()
        response.success = True
        response.message = '已回零位'
        return response

    def _srv_ready(self, request, response):
        """服务 /yolo_grasp/ready：到抓取预备位姿"""
        self.robot.get_logger().info('[服务] 收到预备位姿指令')
        self.go_ready()
        response.success = True
        response.message = '已到抓取预备位姿'
        return response

    def _srv_status(self, request, response):
        """服务 /yolo_grasp/status：获取当前状态"""
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
        """发布目标类别切换指令给 YOLO 感知节点。"""
        msg = String()
        msg.data = class_name
        self.target_class_pub.publish(msg)
        print(f"  已发送目标类别切换: '{class_name}'")

    def _target_cb(self, msg):
        self.latest_target = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.latest_target_time = time.time()

    def spin(self, n=10, dt=0.05):
        for _ in range(n):
            rclpy.spin_once(self.robot, timeout_sec=dt)

    # ---------------- 运动原语 ----------------
    def send_movej(self, joints_rad, a=MOVEJ_A, v=MOVEJ_V):
        j = ", ".join(f"{x:.6f}" for x in joints_rad)
        self._send(f"def prog():\n    movej([{j}], a={a:.3f}, v={v:.3f}, r=0)\nend")

    def send_movel_keep_orientation(self, pos, a=MOVEL_A, v=MOVEL_V):
        """movel 到指定位置，姿态保持当前（读当前 TCP 姿态原样回发）。"""
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
        """轮询 TCP 位置，连续稳定视为运动结束（前 1 秒宽限期）。"""
        start = time.time()
        last = None
        stable = 0
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
    def grasp(self):
        print("\n======= 开始抓取流程 =======")
        # 0. 目标检查
        if self.latest_target is None:
            print("没有目标点！请先确认 YOLO 感知节点检测到物体")
            return
        age = time.time() - self.latest_target_time
        if age > 3.0:
            print(f"警告：目标点已 {age:.0f}s 未更新（感知节点可能在跑但没检测到）")
        obj = self.latest_target
        print(f"1. 目标点(基座系): [{obj[0]:.4f}, {obj[1]:.4f}, {obj[2]:.4f}]")

        # 1. 抓取点 = 目标点 + 世界系偏移
        offset_base = (GRASP_OFFSET_WORLD[0] * WORLD_X_IN_BASE +
                       GRASP_OFFSET_WORLD[1] * WORLD_Y_IN_BASE +
                       GRASP_OFFSET_WORLD[2] * V_UP_IN_BASE)
        grasp_tip = obj + offset_base
        print(f"2. 抓取点(掌心到达): [{grasp_tip[0]:.4f}, {grasp_tip[1]:.4f}, {grasp_tip[2]:.4f}]"
              f"（世界系偏移 {GRASP_OFFSET_WORLD}）")

        # 2. 预抓取点 = 目标点 + 世界系偏移（默认正上方 10cm，指尖参考系）
        pre_offset = (PRE_GRASP_OFFSET_WORLD[0] * WORLD_X_IN_BASE +
                      PRE_GRASP_OFFSET_WORLD[1] * WORLD_Y_IN_BASE +
                      PRE_GRASP_OFFSET_WORLD[2] * V_UP_IN_BASE)
        pre_tip = obj + pre_offset

        # 工具方向 = 法兰 Z 轴（从示教抓取姿态读取），手沿法兰 Z 轴安装
        tool_dir = self.grasp_rot[:, 2]  # 法兰 Z 轴在基座系下的方向
        print(f"   [诊断] 工具方向(法兰Z轴): {np.round(tool_dir, 3)}")

        # IK 求法兰位置：掌心目标 - L × 工具方向
        pre_flange = pre_tip - TOOL_TIP_LENGTH * tool_dir
        print(f"3. 预抓取点(法兰): [{pre_flange[0]:.4f}, {pre_flange[1]:.4f}, {pre_flange[2]:.4f}]")

        dist = float(np.linalg.norm(pre_flange - np.array([0, 0, SHOULDER_Z])))
        if dist > ARM_REACH:
            print(f"   !! 预抓取点距肩关节 {dist:.2f}m 超臂展，不可达，放弃")
            return

        self.spin(10)
        q_guess = self.robot.get_joint_positions()
        if q_guess is None:
            print("   !! 无法获取当前关节角，放弃")
            return

        if self.grasp_rot is None:
            print("   !! 抓取姿态未示教：先把机械臂摆到目标姿态（法兰朝+X、手心朝下），再按 k 示教")
            return

        # 完整 6 维 IK：位置 + 完整姿态（法兰朝+X 且手心朝下，姿态不再放开）
        joint_target = cs66_inverse_kinematics(
            pre_flange, self.grasp_rot, q_guess)
        if joint_target is None:
            print("   !! IK 解算失败，放弃")
            return
        fk_pos, fk_rot = cs66_forward_kinematics(joint_target)
        pos_err = float(np.linalg.norm(fk_pos - pre_flange))
        rot_err = math.degrees(float(Rot.from_matrix(fk_rot.T @ self.grasp_rot).magnitude()))
        print(f"   IK: 位置误差 {pos_err*1000:.1f}mm, 姿态误差 {rot_err:.2f}°")
        if pos_err > 0.02 or rot_err > 5.0:
            print("   !! IK 误差过大，放弃")
            return

        # 3. 张开机械手，movej 到预抓取点
        print("4. 张开机械手，movej 到预抓取点...")
        self.hand_open()
        time.sleep(0.5)
        self.send_movej(joint_target)
        if not self.wait_motion_done():
            print("   !! 等待运动结束超时，放弃")
            return

        # 诊断：读取控制器反馈的实际 TCP 位置
        self.spin(5)
        actual_tcp = self.robot.get_tcp_pose()
        if actual_tcp is not None:
            print(f"   [诊断] movej 后实际 TCP: [{actual_tcp[0][0]:.4f}, {actual_tcp[0][1]:.4f}, {actual_tcp[0][2]:.4f}]")
            print(f"   [诊断] 期望法兰位置:   [{pre_flange[0]:.4f}, {pre_flange[1]:.4f}, {pre_flange[2]:.4f}]")
            tcp_err = np.linalg.norm(np.array(actual_tcp[0]) - pre_flange)
            print(f"   [诊断] 偏差: {tcp_err*1000:.1f}mm")

        # 4. movel 竖直下降到抓取点（法兰 = 掌心目标 - L × 工具方向）
        reach_flange = grasp_tip - TOOL_TIP_LENGTH * tool_dir
        print(f"   掌心目标: [{grasp_tip[0]:.4f}, {grasp_tip[1]:.4f}, {grasp_tip[2]:.4f}]")
        print(f"   法兰目标(movel): [{reach_flange[0]:.4f}, {reach_flange[1]:.4f}, {reach_flange[2]:.4f}]")
        print(f"   工具偏移: {TOOL_TIP_LENGTH:.3f}m  方向: {np.round(TOOL_AXIS_DIR, 3)}")
        print("5. movel 下降到抓取点...")
        if not self.send_movel_keep_orientation(reach_flange):
            print("   !! 无法读取当前位姿，放弃")
            return
        if not self.wait_motion_done():
            print("   !! movel 超时，放弃")
            return

        # 诊断：movel 后实际位置
        self.spin(5)
        actual_tcp = self.robot.get_tcp_pose()
        if actual_tcp is not None:
            print(f"   [诊断] movel 后实际 TCP: [{actual_tcp[0][0]:.4f}, {actual_tcp[0][1]:.4f}, {actual_tcp[0][2]:.4f}]")
            tcp_err = np.linalg.norm(np.array(actual_tcp[0]) - reach_flange)
            print(f"   [诊断] 与法兰目标偏差: {tcp_err*1000:.1f}mm")

        # 5. 闭合机械手
        print("6. 闭合机械手...")
        self.hand_close()
        time.sleep(GRIPPER_CLOSE_DELAY)

        # 6. movel 退回预抓取点
        print("7. movel 退回...")
        if not self.send_movel_keep_orientation(pre_flange):
            print("   !! 无法读取当前位姿")
            return
        if not self.wait_motion_done():
            print("   !! 退回超时（注意检查机械手是否夹住物体）")
            return

        print("======= 抓取完成 =======\n")

    def home(self):
        print("回零位...")
        self.send_movej(HOME_JOINTS)
        if self.wait_motion_done():
            print("已回零")

    def go_ready(self):
        print("移动到抓取预备位姿...")
        self.send_movej(READY_JOINTS)
        if self.wait_motion_done():
            print("已到预备位姿")

    def resend_external_script(self):
        cli = self.robot.create_client(
            Trigger, "/io_and_status_controller/resend_external_script")
        if cli.wait_for_service(timeout_sec=2.0):
            future = cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self.robot, future)


def main():
    rclpy.init()
    g = YoloGrasp()
    if not g.robot.wait_for_state(timeout=10.0):
        print("错误：收不到机械臂状态，请确认驱动已启动")
        g.robot.destroy_node()
        rclpy.shutdown()
        return

    # 后台线程：处理 ROS 服务请求
    executor = MultiThreadedExecutor()
    executor.add_node(g.robot)
    import threading
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("=" * 60)
    print("YOLO 抓取主程序（LinkerHand O6 机械手版）")
    print("  键盘: g=抓取  o=张开手  c=闭合手  p=打印目标  h=回零  r=预备位姿  q=退出")
    print("        k=示教抓取姿态（摆到 法兰朝+X+手心朝下 后按）")
    print("        t=切换目标类别（如 t apple / t cup / t all）")
    print("  ROS服务: /yolo_grasp/grasp  /open  /close  /home  /ready  /status")
    print("  ROS话题: /yolo/target_class (发布类别名切换检测目标)")
    print(f"  工具轴: 世界系下(手心朝下)   抓取偏移: {GRASP_OFFSET_WORLD}")
    print("=" * 60)
    g.hand_setup()  # 设置机械手速度和扭矩

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
                g.grasp()
            elif cmd == 'o':
                g.hand_open()
                print("  已张开")
            elif cmd == 'c':
                g.hand_close()
                print("  已闭合")
            elif cmd == 'p':
                if g.latest_target is None:
                    print("  无目标")
                else:
                    print(f"  目标: {np.round(g.latest_target, 4)}"
                          f"（{time.time()-g.latest_target_time:.1f}s 前）")
            elif cmd == 'h':
                g.home()
            elif cmd == 'r':
                g.go_ready()
            elif cmd == 'k':
                g.calibrate_grasp_orientation()
            elif cmd.startswith('t'):
                # t apple  /  t cup  /  t all  /  t apple,cup
                parts = cmd.split(maxsplit=1)
                cls = parts[1] if len(parts) > 1 else 'apple'
                g.set_target_class(cls)
            elif cmd == '':
                pass
            else:
                print("  未知命令。可用: g=抓取 o=张开 c=闭合 p=打印 h=回零 r=预备位姿 t=切换目标 q=退出")
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
