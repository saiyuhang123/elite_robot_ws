#!/usr/bin/env python3
"""YOLO 视觉抓取主程序（支持多种夹爪）。

流程（按 G 触发）：
  1. 取 YOLO 感知节点发布的目标点（/target_object_pose，基座系）
  2. 抓取点 = 目标点 + 世界系偏移（由夹爪类型决定语义）
  3. movej 到预抓取点 = 目标点正上方 10cm（世界系）
  4. movel 竖直下降到抓取点
  5. 闭合夹爪
  6. movel 上升退回预抓取点
  7. movej 回零位（h 键）

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
  调 /yolo_grasp/grasp（Trigger）触发抓取，response.success/message
  为真实结果（失败原因如 无稳定目标/IK 解算失败/movel 下降超时）。
  抓取前会检查目标新鲜度（<1s）与稳定性（0.6s 内漂移 <5mm）。
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
TARGET_MAX_AGE = 1.0        # 目标点最大龄期（秒），超龄视为无目标
TARGET_STABLE_WINDOW = 0.6  # 稳定采样窗口（秒）
TARGET_STABLE_TOL = 0.005   # 窗口内允许的最大漂移（米）

MOVEJ_A, MOVEJ_V = 1.0, 0.2
MOVEL_A, MOVEL_V = 0.3, 0.05

HOME_JOINTS = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
# 抓取预备位姿（角度: -2.2, -38.3, -124.8, -16.3, 102.1, 94.2）
READY_JOINTS = [-0.0384, -0.6685, -2.1782, -0.2845, 1.7820, 1.6441]
SHOULDER_Z = 0.1625
ARM_REACH = 0.92

# 示教文件基名（不同夹爪存不同文件）
GRASP_ROT_FILE = os.path.join(os.path.dirname(__file__), 'grasp_orientation.json')


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


def _gripper_rot_file(name: str) -> str:
    """按夹爪名称返回示教文件路径。"""
    return os.path.join(os.path.dirname(__file__),
                        f'grasp_orientation_{name}.json')


class YoloGrasp:
    def __init__(self, gripper_name: str = "linkerhand"):
        self.robot = RobotCartesianControl()
        self.cb_group = ReentrantCallbackGroup()

        # ---- 夹爪（按名称创建）----
        self.gripper = create_gripper(gripper_name, self.robot)
        rot_file = _gripper_rot_file(gripper_name)

        # ---- 控制器通信 ----
        self.script_pub = self.robot.create_publisher(String, SCRIPT_TOPIC, 10)

        # ---- 目标感知 ----
        self.target_class_pub = self.robot.create_publisher(
            String, '/yolo/target_class', 10)
        self.latest_target = None
        self.latest_target_time = 0.0
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
        self.robot.create_service(Trigger, '/yolo_grasp/ready', self._srv_ready,
                                  callback_group=self.cb_group)
        self.robot.create_service(Trigger, '/yolo_grasp/status', self._srv_status,
                                  callback_group=self.cb_group)

        # ---- 抓取姿态：示教文件优先，直装爪缺文件时自动构造兜底 ----
        self.grasp_rot = None

        if os.path.exists(rot_file):
            try:
                import json as _json
                with open(rot_file) as f:
                    self.grasp_rot = np.array(_json.load(f), dtype=float)
                print(f"已从 {rot_file} 加载抓取姿态")
            except Exception as e:
                print(f"读取 {rot_file} 失败: {e}（按 k 重新示教）")

        if self.grasp_rot is None and not self.gripper.needs_orientation_calibration:
            # 直装型夹爪：自动构造（Z=世界下方），不依赖文件/启动姿态
            r = self.gripper.default_grasp_rotation(V_UP_IN_BASE)
            if r is not None:
                self.grasp_rot = r
                import json as _json
                with open(rot_file, 'w') as f:
                    _json.dump([[float(v) for v in row] for row in r], f)
                print(f"[自动] 抓取姿态已构造: Z={np.round(r[:, 2], 3)} → {rot_file}")

    # ---------------- 抓取姿态示教 ----------------
    def calibrate_grasp_orientation(self):
        """把机械臂摆到目标抓取姿态后调用，记录当前 FK 旋转矩阵。"""
        self.spin(10)
        q = self.robot.get_joint_positions()
        if q is None:
            print("  无法获取关节角，示教失败")
            return
        _, r_flange = cs66_forward_kinematics(q)
        self.grasp_rot = r_flange
        rot_file = _gripper_rot_file(self.gripper.name)
        import json as _json
        with open(rot_file, 'w') as f:
            _json.dump([[float(v) for v in row] for row in r_flange], f)
        print(f"  抓取姿态已记录到 {rot_file}")
        print(f"  法兰Z方向(基座系) = {np.round(r_flange[:, 2], 3)}")


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

    def _srv_ready(self, request, response):
        self.robot.get_logger().info('[服务] 收到预备位姿指令')
        self.go_ready()
        response.success = True
        response.message = '已到抓取预备位姿'
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
        for _ in range(n):
            rclpy.spin_once(self.robot, timeout_sec=dt)

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
        """在 window 内持续采样目标点：要求感知持续更新且位置漂移 < tol。
        返回稳定目标均值（基座系），任一条件不满足返回 None。"""
        samples = []
        first_stamp = self.latest_target_time
        start = time.time()
        while time.time() - start < window:
            t = self.latest_target
            if (t is None
                    or time.time() - self.latest_target_time > TARGET_MAX_AGE):
                return None
            samples.append(t.copy())
            time.sleep(0.05)
        if not samples or self.latest_target_time == first_stamp:
            return None  # 窗口内没有新帧（感知停发）
        arr = np.array(samples)
        spread = float(np.max(np.linalg.norm(arr - arr.mean(axis=0), axis=1)))
        if spread > tol:
            return None
        return arr.mean(axis=0)

    def grasp(self):
        """执行一次抓取流程。返回 (成功与否, 结果描述)。"""
        gripper = self.gripper
        print(f"\n======= 开始抓取流程 [{gripper.name}] =======")

        # 0. 目标检查：新鲜 + 稳定（调度场景：防旧目标/移动目标）
        obj = self.get_stable_target()
        if obj is None:
            print("没有稳定目标（感知未检测到、目标超龄或在移动）")
            return False, "无稳定目标"
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

        # 工具方向 = 法兰 Z 轴，长度 = 夹爪的 tool_length
        if self.grasp_rot is None:
            print("   !! 抓取姿态未示教，请先按 k 示教")
            return False, "抓取姿态未示教"
        tool_dir = self.grasp_rot[:, 2]
        L = gripper.tool_length
        print(f"   [诊断] 工具方向(法兰Z轴): {np.round(tool_dir, 3)}  "
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
            joint_target = ik_func(pre_flange, self.grasp_rot, q_guess)

        if joint_target is None:
            print("   !! IK 解算失败，放弃")
            return False, "IK 解算失败"
        fk_pos, fk_rot = cs66_forward_kinematics(joint_target)
        pos_err = float(np.linalg.norm(fk_pos - pre_flange))
        if gripper.ik_mode == "6dof":
            rot_err = math.degrees(float(
                Rot.from_matrix(fk_rot.T @ self.grasp_rot).magnitude()))
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
    print("  键盘: g=抓取  o=张开  c=闭合  p=打印目标  h=回零  r=预备位姿")
    print("        k=示教姿态  t=切换目标类别  q=退出")
    print("  ROS服务: /yolo_grasp/grasp /open /close /home /ready /status")
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
                    elif cmd == 'r':
                        g.go_ready()
                    elif cmd == 'k':
                        g.calibrate_grasp_orientation()
                    elif cmd.startswith('t'):
                        parts = cmd.split(maxsplit=1)
                        cls = parts[1] if len(parts) > 1 else 'apple'
                        g.set_target_class(cls)
                    elif cmd == '':
                        pass
                    else:
                        print("  未知命令。g=抓取 o=张开 c=闭合 p=打印 "
                              "h=回零 r=预备 k=示教 t=切换目标 q=退出")
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
