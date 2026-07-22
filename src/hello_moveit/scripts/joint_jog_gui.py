#!/usr/bin/env python3
"""CS66 控制器原生 movej / movel 测试 GUI（Tkinter + rclpy）。

不经过 MoveIt，直接通过 /script_sender/script_command 把 EliRobot 脚本
发给控制器执行。

注意：movej/movel 只能放在主脚本(def)里，控制器不允许辅脚本(sec)执行运动。
def 会打断驱动的外部控制程序（external_control），用完点界面上的
“恢复驱动外部控制”按钮（或 ros2 service call
/io_and_status_controller/resend_external_script std_srvs/srv/Trigger）
恢复，之后 MoveIt/轨迹控制器才能继续用。

功能：
  - 上半部分：六轴关节 movej（滑条 / 输入 / 点动 / 按住连续点动）
  - 下半部分：末端 movel 直线（显示当前 TCP 位姿，X/Y/Z 相对步进或绝对目标）

前提：只需启动机器人驱动（start_robot.launch.py），不需要 move_group。

用法：
  ros2 run hello_moveit joint_jog_gui.py
"""

import math
import queue
import threading

import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    import tf2_ros
except ImportError:
    tf2_ros = None

SCRIPT_TOPIC = "/script_sender/script_command"
BASE_FRAME = "cs66_base"              # tcp_pose_broadcaster 的父坐标系（控制器基座系）
TCP_FRAME = "cs66_tool0_controller"   # 控制器上报的 TCP 位姿
JOINT_LIMIT_DEG = 360.0
N_JOINTS = 6
# CS66 六轴在控制器 movej 里的规范顺序（/joint_states 发布顺序是乱的，不能用）
JOINT_ORDER = [
    "cs66_shoulder_pan_joint",   # 关节1
    "cs66_shoulder_lift_joint",  # 关节2
    "cs66_elbow_joint",          # 关节3
    "cs66_wrist_1_joint",        # 关节4
    "cs66_wrist_2_joint",        # 关节5
    "cs66_wrist_3_joint",        # 关节6
]

# ---- 倾斜安装补偿 ----
# 基座倾斜安装（实测倾斜角约48°）。世界系"上"在基座系下的方向（实测值）：
# 物理法兰调水平后 tf2_echo cs66_base_link cs66_tool0，取矩阵第三列取反。
# 注意：不要用 cs66_tool0_controller（虚拟TCP，与物理法兰差约118°旋转）。
# 世界系 X/Y/Z 由该向量构建正交基；MoveL 步进沿世界系方向。
V_UP_IN_BASE = (-0.7431, 0.0120, 0.6691)


def _build_world_axes(v_up):
    """由世界的'上'构建正交的世界系 X/Y 轴（以基座 Y 为参考）。"""
    import numpy as _np
    up = _np.array(v_up, dtype=float)
    up /= _np.linalg.norm(up)
    y = _np.array([0.0, 1.0, 0.0])
    y = y - (y @ up) * up
    y /= _np.linalg.norm(y)
    x = _np.cross(y, up)
    return tuple(x), tuple(y), tuple(up)


V_X_IN_BASE, V_Y_IN_BASE, _ = _build_world_axes(V_UP_IN_BASE)
WORLD_AXES_IN_BASE = (V_X_IN_BASE, V_Y_IN_BASE, V_UP_IN_BASE)


def quat_to_rotvec(x, y, z, w):
    """四元数 -> 旋转向量 (rx, ry, rz)，模长归一化到 <= pi。
    艾利特控制器上报和脚本指令的位姿姿态部分都是旋转向量
    （驱动 hardware_interface 按轴角解析上报的 TCP 姿态）。"""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    angle = 2.0 * math.acos(max(-1.0, min(1.0, w)))
    s = math.sqrt(max(0.0, 1.0 - w * w))  # sin(angle/2)
    if s < 1e-9:
        return 0.0, 0.0, 0.0
    if angle > math.pi:  # 取短边
        angle -= 2.0 * math.pi
    return x / s * angle, y / s * angle, z / s * angle


class JogNode(Node):
    """后台 ROS 节点：订阅当前状态 + 向控制器发 movej/movel 脚本。"""

    def __init__(self, ui_queue):
        super().__init__("joint_jog_gui")
        self.ui_queue = ui_queue
        self.script_pub = self.create_publisher(String, SCRIPT_TOPIC, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        self.joint_names = list(JOINT_ORDER)  # 固定顺序，不依赖 /joint_states 的发布顺序
        self.tf_buffer = tf2_ros.Buffer() if tf2_ros else None
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self) if tf2_ros else None

    def on_joint_states(self, msg):
        positions = []
        for name in self.joint_names:
            try:
                i = msg.name.index(name)
                positions.append(msg.position[i])
            except (ValueError, IndexError):
                return
        self.ui_queue.put(("current", [math.degrees(p) for p in positions]))

    def current_tcp(self):
        """返回当前 TCP (x,y,z,rx,ry,rz)，rx/ry/rz 为旋转向量，查询失败返回 None。"""
        if not self.tf_buffer:
            return None
        try:
            t = self.tf_buffer.lookup_transform(
                BASE_FRAME, TCP_FRAME, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().warn(f"查询 TCP 失败: {e}")
            return None
        p = t.transform.translation
        q = t.transform.rotation
        rx, ry, rz = quat_to_rotvec(q.x, q.y, q.z, q.w)
        return p.x, p.y, p.z, rx, ry, rz

    def current_tcp_quat(self):
        """返回当前 TCP 四元数 (x,y,z,w)，查询失败返回 None。
        用于标定竖直方向：四元数->矩阵无解释歧义，比解析示教器数字可靠。"""
        if not self.tf_buffer:
            return None
        try:
            t = self.tf_buffer.lookup_transform(
                BASE_FRAME, TCP_FRAME, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().warn(f"查询 TCP 失败: {e}")
            return None
        q = t.transform.rotation
        return q.x, q.y, q.z, q.w

    def send_script(self, script):
        msg = String()
        msg.data = script
        self.script_pub.publish(msg)

    def send_movej(self, joints_deg, a, v):
        joints = ", ".join(f"{math.radians(d):.6f}" for d in joints_deg)
        self.send_script(f"def prog():\n    movej([{joints}], a={a:.3f}, v={v:.3f}, r=0)\nend")
        self.ui_queue.put(("status", "已发送 movej: [" +
                           ", ".join(f"{d:.1f}" for d in joints_deg) + "]"))

    def send_movel(self, pose, a, v):
        # EliRobot 脚本里位姿是 6 元素列表 [x,y,z,rx,ry,rz]，姿态部分为旋转向量(rad)。
        # 不要用 URScript 的 p[...] 写法（p 是内置函数，会报“不支持下标操作”）
        p = ", ".join(f"{x:.6f}" for x in pose)
        self.send_script(f"def prog():\n    movel([{p}], a={a:.3f}, v={v:.3f})\nend")
        self.ui_queue.put(("status", f"已发送 movel: [{p}]"))

    def resend_external_script(self):
        """恢复驱动的外部控制程序（def 脚本会把它打断）。"""
        cli = self.create_client(Trigger, "/io_and_status_controller/resend_external_script")
        if not cli.wait_for_service(timeout_sec=2.0):
            self.ui_queue.put(("status", "resend_external_script 服务不可用"))
            return
        future = cli.call_async(Trigger.Request())
        while rclpy.ok() and not future.done():
            threading.Event().wait(0.05)
        resp = future.result()
        if resp and resp.success:
            self.ui_queue.put(("status", "外部控制程序已恢复"))
        else:
            self.ui_queue.put(("status", "恢复失败: " + (resp.message if resp else "无响应")))


class JogGUI:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.ui_queue = node.ui_queue
        self.joint_names = list(JOINT_ORDER)
        self.name_labels = []
        self.sliders = []
        self.entries = []
        self.current_labels = []

        root.title("CS66 控制器 movej / movel 测试")
        root.geometry("860x760")

        # ============ MoveJ 关节部分 ============
        ttk.Label(root, text="— MoveJ（关节空间）—", padding=4).pack(fill=tk.X)

        top = ttk.Frame(root, padding=4)
        top.pack(fill=tk.X)
        ttk.Label(top, text="a(rad/s²)").pack(side=tk.LEFT)
        self.ja_var = tk.StringVar(value="1.0")
        ttk.Entry(top, textvariable=self.ja_var, width=6).pack(side=tk.LEFT, padx=(2, 10))
        ttk.Label(top, text="v(rad/s)").pack(side=tk.LEFT)
        self.jv_var = tk.StringVar(value="0.2")
        ttk.Entry(top, textvariable=self.jv_var, width=6).pack(side=tk.LEFT, padx=(2, 10))
        ttk.Label(top, text="点动步长(°)").pack(side=tk.LEFT)
        self.step_var = tk.StringVar(value="2")
        ttk.Entry(top, textvariable=self.step_var, width=6).pack(side=tk.LEFT, padx=2)

        grid = ttk.Frame(root, padding=4)
        grid.pack(fill=tk.X)
        ttk.Label(grid, text="关节", width=22).grid(row=0, column=0)
        ttk.Label(grid, text="当前(°)", width=10).grid(row=0, column=1)
        ttk.Label(grid, text="目标(°)").grid(row=0, column=2, columnspan=4)

        for j in range(N_JOINTS):
            r = j + 1
            short = self.joint_names[j].replace("cs66_", "").replace("_joint", "")
            name_lbl = ttk.Label(grid, text=f"关节{j+1} ({short})", width=22)
            name_lbl.grid(row=r, column=0, sticky=tk.W)
            self.name_labels.append(name_lbl)

            cur_lbl = ttk.Label(grid, text="--", width=10)
            cur_lbl.grid(row=r, column=1)
            self.current_labels.append(cur_lbl)

            var = tk.DoubleVar(value=0.0)
            slider = ttk.Scale(grid, from_=-JOINT_LIMIT_DEG, to=JOINT_LIMIT_DEG,
                               variable=var, orient=tk.HORIZONTAL, length=260,
                               command=lambda v, j=j: self.on_slider(j, v))
            slider.grid(row=r, column=2, padx=4)
            self.sliders.append(var)

            entry = ttk.Entry(grid, width=8)
            entry.insert(0, "0.0")
            entry.bind("<Return>", lambda e, j=j: self.on_entry(j))
            entry.grid(row=r, column=3, padx=4)
            self.entries.append(entry)

            for sign, text, col in ((-1, "-", 4), (+1, "+", 5)):
                btn = ttk.Button(grid, text=text, width=3)
                # 按下开始持续点动，松开停止；单击也会点动一次
                btn.bind("<ButtonPress>", lambda e, j=j, s=sign: self.start_hold(j, s))
                btn.bind("<ButtonRelease>", lambda e: self.stop_hold())
                btn.grid(row=r, column=col)

        jbtns = ttk.Frame(root, padding=4)
        jbtns.pack(fill=tk.X)
        ttk.Button(jbtns, text="当前 → 目标", command=self.copy_current).pack(side=tk.LEFT, padx=4)
        ttk.Button(jbtns, text="全部回零", command=self.zero_all).pack(side=tk.LEFT, padx=4)
        ttk.Button(jbtns, text="恢复驱动外部控制",
                   command=lambda: threading.Thread(
                       target=self.node.resend_external_script, daemon=True).start()
                   ).pack(side=tk.LEFT, padx=12)
        ttk.Button(jbtns, text="执行 MoveJ", command=self.execute_movej).pack(side=tk.RIGHT, padx=4)

        # ============ MoveL 笛卡尔部分 ============
        ttk.Separator(root, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=6)
        ttk.Label(root, text="— MoveL（末端直线，基座系）—", padding=4).pack(fill=tk.X)

        ltop = ttk.Frame(root, padding=4)
        ltop.pack(fill=tk.X)
        ttk.Label(ltop, text="a(m/s²)").pack(side=tk.LEFT)
        self.la_var = tk.StringVar(value="0.5")
        ttk.Entry(ltop, textvariable=self.la_var, width=6).pack(side=tk.LEFT, padx=(2, 10))
        ttk.Label(ltop, text="v(m/s)").pack(side=tk.LEFT)
        self.lv_var = tk.StringVar(value="0.05")
        ttk.Entry(ltop, textvariable=self.lv_var, width=6).pack(side=tk.LEFT, padx=(2, 10))
        ttk.Label(ltop, text="步长(mm)").pack(side=tk.LEFT)
        self.lstep_var = tk.StringVar(value="10")
        ttk.Entry(ltop, textvariable=self.lstep_var, width=6).pack(side=tk.LEFT, padx=2)

        cur_frame = ttk.Frame(root, padding=4)
        cur_frame.pack(fill=tk.X)
        ttk.Label(cur_frame, text="当前 TCP:").pack(side=tk.LEFT)
        self.tcp_var = tk.StringVar(value="--")
        ttk.Label(cur_frame, textvariable=self.tcp_var).pack(side=tk.LEFT, padx=6)
        ttk.Button(cur_frame, text="刷新", command=self.refresh_tcp).pack(side=tk.LEFT, padx=4)
        ttk.Button(cur_frame, text="当前 → 目标", command=self.copy_tcp).pack(side=tk.LEFT, padx=4)

        tgt_frame = ttk.Frame(root, padding=4)
        tgt_frame.pack(fill=tk.X)
        ttk.Label(tgt_frame, text="目标 TCP:  x y z rx ry rz").pack(side=tk.LEFT)
        self.tcp_entries = []
        defaults = ("0.0", "0.0", "0.5", "0.0", "0.0", "0.0")
        for i in range(6):
            e = ttk.Entry(tgt_frame, width=9)
            e.insert(0, defaults[i])
            e.pack(side=tk.LEFT, padx=2)
            self.tcp_entries.append(e)
        ttk.Button(tgt_frame, text="执行 MoveL", command=self.execute_movel).pack(side=tk.LEFT, padx=8)

        jog_frame = ttk.Frame(root, padding=4)
        jog_frame.pack(fill=tk.X)
        ttk.Label(jog_frame, text="直线步进:").pack(side=tk.LEFT)
        for axis, idx in (("X", 0), ("Y", 1), ("Z", 2)):
            ttk.Button(jog_frame, text=f"{axis}-", width=4,
                       command=lambda i=idx: self.jog_cartesian(i, -1)).pack(side=tk.LEFT, padx=2)
            ttk.Button(jog_frame, text=f"{axis}+", width=4,
                       command=lambda i=idx: self.jog_cartesian(i, +1)).pack(side=tk.LEFT, padx=2)

        tilt_frame = ttk.Frame(root, padding=4)
        tilt_frame.pack(fill=tk.X)
        # Z 步进的竖直方向：默认用文件顶部常量，点“标定”按钮用实测姿态重算
        self.v_up = tuple(V_UP_IN_BASE)
        self.use_tilt = tk.BooleanVar(value=True)
        ttk.Checkbutton(tilt_frame, text="倾斜补偿-世界系XYZ（关=基座系）",
                        variable=self.use_tilt).pack(side=tk.LEFT, padx=4)
        ttk.Button(tilt_frame, text="标定竖直方向（法兰先调水平）",
                   command=self.calibrate_up).pack(side=tk.LEFT, padx=8)
        self.tilt_var = tk.StringVar(value=self._tilt_text(self.v_up))
        ttk.Label(tilt_frame, textvariable=self.tilt_var).pack(side=tk.LEFT, padx=8)

        self.status_var = tk.StringVar(value="等待 /joint_states ...")
        ttk.Label(root, textvariable=self.status_var, relief=tk.SUNKEN,
                  anchor=tk.W, padding=4).pack(fill=tk.X, side=tk.BOTTOM)

        self.latest_current = None
        self.latest_tcp = None
        self._targets_initialized = False  # 第一次收到当前角度时同步到目标
        self._hold_after_id = None         # 持续点动的 after 定时器
        self._hold_joint = None
        self._hold_dir = 0
        root.after(100, self.poll_queue)
        root.after(1000, self.refresh_tcp)

    # ---- MoveJ UI ----
    def on_slider(self, j, value):
        self.entries[j].delete(0, tk.END)
        self.entries[j].insert(0, f"{float(value):.1f}")

    def on_entry(self, j):
        try:
            v = max(-JOINT_LIMIT_DEG, min(JOINT_LIMIT_DEG, float(self.entries[j].get())))
        except ValueError:
            return
        self.sliders[j].set(v)

    def jog(self, j, direction):
        try:
            step = abs(float(self.step_var.get()))
        except ValueError:
            step = 2.0
        v = self.sliders[j].get() + direction * step
        v = max(-JOINT_LIMIT_DEG, min(JOINT_LIMIT_DEG, v))
        self.sliders[j].set(v)
        self.on_slider(j, v)

    def start_hold(self, j, direction):
        self.stop_hold()
        self._hold_joint = j
        self._hold_dir = direction
        self.jog(j, direction)
        self.execute_movej()
        self._hold_after_id = self.root.after(300, self._hold_repeat)

    def _hold_repeat(self):
        if self._hold_joint is None:
            return
        # 实际角度跟上目标后再发下一步，避免命令堆积
        try:
            step = abs(float(self.step_var.get()))
        except ValueError:
            step = 2.0
        if self.latest_current is not None:
            diff = abs(self.latest_current[self._hold_joint] -
                       self.sliders[self._hold_joint].get())
            if diff < step:
                self.jog(self._hold_joint, self._hold_dir)
                self.execute_movej()
        self._hold_after_id = self.root.after(300, self._hold_repeat)

    def stop_hold(self):
        if self._hold_after_id is not None:
            self.root.after_cancel(self._hold_after_id)
            self._hold_after_id = None
        self._hold_joint = None
        self._hold_dir = 0

    def copy_current(self):
        if self.latest_current:
            for j, deg in enumerate(self.latest_current):
                self.sliders[j].set(deg)
                self.on_slider(j, deg)

    def zero_all(self):
        for j in range(N_JOINTS):
            self.sliders[j].set(0.0)
            self.on_slider(j, 0.0)

    def execute_movej(self):
        try:
            a = float(self.ja_var.get())
            v = float(self.jv_var.get())
            assert a > 0 and v > 0
        except (ValueError, AssertionError):
            self.status_var.set("movej 的 a/v 必须是正数")
            return
        joints = [self.sliders[j].get() for j in range(N_JOINTS)]
        self.node.send_movej(joints, a, v)

    # ---- MoveL UI ----
    def refresh_tcp(self):
        tcp = self.node.current_tcp()
        if tcp:
            self.latest_tcp = tcp
            self.tcp_var.set("  ".join(f"{v:.4f}" for v in tcp))

    def copy_tcp(self):
        self.refresh_tcp()
        if self.latest_tcp:
            for i, v in enumerate(self.latest_tcp):
                self.tcp_entries[i].delete(0, tk.END)
                self.tcp_entries[i].insert(0, f"{v:.4f}")

    def get_lav(self):
        try:
            a = float(self.la_var.get())
            v = float(self.lv_var.get())
            assert a > 0 and v > 0
            return a, v
        except (ValueError, AssertionError):
            self.status_var.set("movel 的 a/v 必须是正数")
            return None

    def execute_movel(self):
        lav = self.get_lav()
        if not lav:
            return
        try:
            pose = [float(e.get()) for e in self.tcp_entries]
        except ValueError:
            self.status_var.set("目标 TCP 格式错误（6 个数字: x y z rx ry rz）")
            return
        self.node.send_movel(pose, *lav)

    def jog_cartesian(self, axis, direction):
        """从当前 TCP 出发走一条短直线，姿态不变。
        补偿开启时 X/Y/Z 均沿世界系方向（倾斜安装），关闭时沿基座系。"""
        lav = self.get_lav()
        if not lav:
            return
        tcp = self.node.current_tcp()
        if not tcp:
            self.status_var.set("查询当前 TCP 失败，无法直线步进")
            return
        try:
            step_m = abs(float(self.lstep_var.get())) / 1000.0
        except ValueError:
            step_m = 0.01
        pose = list(tcp)
        if self.use_tilt.get():
            # 世界系方向（基座倾斜安装）；Z 可被“标定竖直方向”覆盖
            vec = self.v_up if axis == 2 else WORLD_AXES_IN_BASE[axis]
            for i in range(3):
                pose[i] += direction * step_m * vec[i]
        else:
            pose[axis] += direction * step_m
        self.node.send_movel(pose, *lav)
        # 运动结束后自动刷新当前 TCP 显示
        self.root.after(1500, self.refresh_tcp)

    # ---- 竖直方向标定 ----
    @staticmethod
    def _tilt_text(v_up):
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, v_up[2]))))
        return f"竖直方向=[{v_up[0]:.3f}, {v_up[1]:.3f}, {v_up[2]:.3f}]  倾斜角={tilt:.1f}°"

    def calibrate_up(self):
        """用当前姿态标定世界竖直方向（前提：物理法兰已用水平尺调水平）。
        查 URDF 物理法兰（cs66_tool0）的 TF——不要用 tool0_controller
        （控制器上报的虚拟TCP，带约118°的 TCP 旋转偏移）。"""
        if not self.node.tf_buffer:
            return
        try:
            t = self.node.tf_buffer.lookup_transform(
                BASE_FRAME, "cs66_tool0", rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception as e:
            self.status_var.set(f"查询 cs66_tool0 失败: {e}")
            return
        q = t.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        n = math.sqrt(x * x + y * y + z * z + w * w)
        x, y, z, w = x / n, y / n, z / n, w / n
        # 旋转矩阵第三列 = 物理法兰Z轴在基座系下的方向 = 世界的"下"（法兰水平时）
        tool_z = (2 * (x * z + w * y),
                  2 * (y * z - w * x),
                  1 - 2 * (x * x + y * y))
        self.v_up = (-tool_z[0], -tool_z[1], -tool_z[2])
        self.tilt_var.set(self._tilt_text(self.v_up))
        self.status_var.set("竖直方向已用物理法兰姿态重新标定")

    # ---- 从 ROS 线程更新 UI ----
    def poll_queue(self):
        try:
            while True:
                kind, data = self.ui_queue.get_nowait()
                if kind == "current":
                    self.latest_current = data
                    for j, deg in enumerate(data):
                        self.current_labels[j].config(text=f"{deg:.1f}")
                    if not self._targets_initialized:
                        # 目标角度初始化为当前角度，方便直接微调
                        self._targets_initialized = True
                        for j, deg in enumerate(data):
                            self.sliders[j].set(deg)
                            self.on_slider(j, deg)
                elif kind == "status":
                    self.status_var.set(data)
        except queue.Empty:
            pass
        self.root.after(100, self.poll_queue)


def main():
    rclpy.init()
    ui_queue = queue.Queue()
    node = JogNode(ui_queue)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    gui = JogGUI(root, node)

    def on_close():
        rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
