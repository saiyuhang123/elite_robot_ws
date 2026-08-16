#!/usr/bin/env python3
"""Inspire 4B4C 二指夹爪控制。

控制通道：12 个 ROS 2 服务（串口→ROS 桥接节点 inspire_gripper）。
读写命令统一走 rclpy 持久化 client（首次使用时创建，之后复用），
避免每次命令启动 ros2 CLI 子进程；读反馈（开口度/状态码）用于
动作到位确认与空夹检测。反馈通道不可用时自动降级为盲发。

重试策略：开/合命令无应答（回执丢失）或未确认到位时自动重发，
最多 COMMAND_RETRIES 次。重发幂等（重复开/合无副作用），
真到位与否以开口度反馈为准，回执丢失不视为失败。
"""

import threading
import time
import numpy as np

from .base import GripperBase

# ---- 动作确认阈值（按本台夹爪实测/经验值，不准时改这里）----
APERTURE_OPEN_MIN = 900     # 开口度 ≥ 此值认为已张开到位
APERTURE_EMPTY_MAX = 100    # 开口度 ≤ 此值认为空夹（未夹到物体）
CONFIRM_OPEN_TIMEOUT = 4.0  # 每次张开尝试的到位确认窗口（秒）
CONFIRM_POLL = 0.15         # 确认轮询间隔（秒）
COMMAND_RETRIES = 10        # 开/合无应答或未到位时的最大尝试次数
CMD_ACK_TIMEOUT = 3.0       # 单次发令回执等待（秒）: 回执仅用于尽早发现
                            # 死服务, 丢了靠重发补上, 到位判断看开口度


class TwoFingerGripper(GripperBase):
    """Inspire 4B4C 二指电动夹爪。

    依赖: inspire_gripper 包的 Gripper_control_node 已运行。
    默认串口 /dev/ttyGripper（原 ttyCH341USB1）, 115200 baud, gripper_id=1。

    参数:
        gripper_id:  夹爪 ID (1-254)
        open_speed:  张开速度 (1-1000)
        close_speed: 闭合速度 (1-1000)
        close_force: 闭合力度 (50-1000)
        tip_length:  法兰面到指尖中点的工具偏移（米）
    """

    def __init__(self, robot_node,
                 gripper_id: int = 1,
                 open_speed: int = 400,
                 close_speed: int = 500,
                 close_force: int = 600,
                 tip_length: float = 0.12):
        self._node = robot_node
        self._id = gripper_id
        self._open_speed = open_speed
        self._close_speed = close_speed
        self._close_force = close_force
        self._tip_length = tip_length
        # 夹爪 service 请求串行锁（RLock：open 内部轮询开口度时同线程可重入）
        self._cmd_lock = threading.RLock()
        # 直接 rclpy 客户端（读写均用，首次使用时惰性创建）
        self._cli_ready = None
        self._getcopen_cli = None
        self._getstatus_cli = None
        self._setclearerror_cli = None
        self._setmovemax_cli = None
        self._setmovemin_cli = None
        self._setmoveminhold_cli = None
        self._Getcopen = None
        self._Getstatus = None
        self._Setclearerror = None
        self._Setmovemax = None
        self._Setmovemin = None
        self._Setmoveminhold = None

    @property
    def name(self) -> str:
        return "two_finger"

    @property
    def ik_mode(self) -> str:
        return "6dof"  # 二指方向重要（指缝朝向）

    @property
    def close_delay(self) -> float:
        return 1.0

    @property
    def grasp_offset_world(self) -> np.ndarray:
        # 指尖中点对准物体中心
        return np.array([0.0, 0.0, 0.0])

    @property
    def tool_length(self) -> float:
        return self._tip_length

    # ---------------- 持久化 service client 管理 ----------------

    def _ensure_clients(self) -> bool:
        """惰性创建读写服务客户端，之后复用。失败时当前命令返回失败。"""
        if self._cli_ready is True:
            return True
        with self._cmd_lock:
            if self._cli_ready is True:
                return True
            try:
                from service_interfaces.srv import (
                    Getcopen, Getstatus,
                    Setclearerror, Setmovemax, Setmovemin, Setmoveminhold,
                )
                self._Getcopen = Getcopen
                self._Getstatus = Getstatus
                self._Setclearerror = Setclearerror
                self._Setmovemax = Setmovemax
                self._Setmovemin = Setmovemin
                self._Setmoveminhold = Setmoveminhold
            except Exception as exc:
                self._node.get_logger().warn(
                    f"[{self.name}] service_interfaces 导入失败({exc})")
                self._cli_ready = False
                return False

            try:
                if self._getcopen_cli is None:
                    self._getcopen_cli = self._node.create_client(
                        Getcopen, '/Getcopen')
                if self._getstatus_cli is None:
                    self._getstatus_cli = self._node.create_client(
                        Getstatus, '/Getstatus')
                if self._setclearerror_cli is None:
                    self._setclearerror_cli = self._node.create_client(
                        Setclearerror, '/Setclearerror')
                if self._setmovemax_cli is None:
                    self._setmovemax_cli = self._node.create_client(
                        Setmovemax, '/Setmovemax')
                if self._setmovemin_cli is None:
                    self._setmovemin_cli = self._node.create_client(
                        Setmovemin, '/Setmovemin')
                if self._setmoveminhold_cli is None:
                    self._setmoveminhold_cli = self._node.create_client(
                        Setmoveminhold, '/Setmoveminhold')
                self._cli_ready = True
            except Exception as exc:
                self._node.get_logger().warn(
                    f"[{self.name}] 夹爪服务客户端创建失败({exc})")
                self._cli_ready = False
            return self._cli_ready

    def _rc_call(self, client, req, timeout=1.5):
        """阻塞等响应（后台 MultiThreadedExecutor 在 spin），超时返回 None。

        夹爪所有 service 请求通过 _cmd_lock 串行发送，避免开/合命令穿插。
        """
        if client is None:
            return None
        with self._cmd_lock:
            try:
                fut = client.call_async(req)
            except Exception:
                return None
            t0 = time.time()
            while not fut.done():
                if time.time() - t0 > timeout:
                    return None
                time.sleep(0.02)
            try:
                return fut.result()
            except Exception:
                return None

    def _call_client(self, client, request, ack_field,
                     timeout=CMD_ACK_TIMEOUT) -> bool:
        """调用持久化 client，并检查服务端 ack 字段。"""
        if not self._ensure_clients() or client is None:
            return False
        res = self._rc_call(client, request, timeout)
        if res is None:
            return False
        try:
            ack = bool(getattr(res, ack_field))
        except AttributeError:
            ack = True
        return ack

    # ---------------- 读反馈 ----------------

    def get_aperture(self, timeout=1.5):
        """读当前开口度（0~1000）。失败返回 None。"""
        if not self._ensure_clients():
            return None
        req = self._Getcopen.Request()
        req.gripper_id = self._id
        req.status = 'get_copen'
        res = self._rc_call(self._getcopen_cli, req, timeout)
        if res is None:
            return None
        return float(res.copen)

    def get_status(self, timeout=1.5):
        """读状态码/故障码/温度。失败返回 None。"""
        if not self._ensure_clients():
            return None
        req = self._Getstatus.Request()
        req.gripper_id = self._id
        req.status = 'get_status'
        res = self._rc_call(self._getstatus_cli, req, timeout)
        if res is None:
            return None
        return (int(res.status), int(res.error), int(res.temp))

    def wait_opened(self, timeout=CONFIRM_OPEN_TIMEOUT) -> bool:
        """轮询开口度直到 ≥ APERTURE_OPEN_MIN。
        反馈读不到时不阻塞流程（按盲发处理返回 True）。"""
        t0 = time.time()
        read_fail = 0
        while time.time() - t0 < timeout:
            a = self.get_aperture()
            if a is None:
                read_fail += 1
                if read_fail >= 3:
                    self._node.get_logger().warn(
                        f"[{self.name}] 开口度读取失败，跳过张开确认")
                    return True
            elif a >= APERTURE_OPEN_MIN:
                self._node.get_logger().info(
                    f"[{self.name}] 张开到位（开口度 {a:.0f}）")
                return True
            time.sleep(CONFIRM_POLL)
        return False

    def is_grasping(self) -> bool:
        """判断是否夹住物体：现读开口度，> APERTURE_EMPTY_MAX 视为夹到。
        第一次读数 ≤ 阈值时复测一次（可能还在闭合途中）。读不到不阻塞流程。"""
        a = self.get_aperture()
        if a is None:
            return True
        self._node.get_logger().info(f"[{self.name}] 当前开口度 {a:.0f}")
        if a > APERTURE_EMPTY_MAX:
            return True
        time.sleep(0.4)
        a = self.get_aperture()
        if a is None:
            return True
        self._node.get_logger().info(f"[{self.name}] 复测开口度 {a:.0f}")
        return a > APERTURE_EMPTY_MAX

    # ---------------- 指令（写命令） ----------------

    def _send_with_retry(self, service_name, client, request, ack_field,
                         retries=COMMAND_RETRIES) -> bool:
        """发命令，无应答/未确认时自动重发，最多 retries 次。"""
        for attempt in range(1, retries + 1):
            if self._call_client(client, request, ack_field):
                return True
            if attempt < retries:
                self._node.get_logger().warn(
                    f"[{self.name}] {service_name} 无应答/未确认，重发 "
                    f"{attempt}/{retries - 1}...")
        return False

    def setup(self):
        """清除故障（如需开口限位，请另行调用 Setopenlimit 服务）。"""
        with self._cmd_lock:
            if not self._ensure_clients():
                self._node.get_logger().error(
                    f"[{self.name}] 夹爪服务客户端不可用，无法执行 setup")
                return
            req = self._Setclearerror.Request()
            req.gripper_id = self._id
            req.status = 'set_clearerror'
            if self._call_client(self._setclearerror_cli, req,
                                 'clearerror_accepted'):
                self._node.get_logger().info(f"[{self.name}] 已清除故障")
            else:
                self._node.get_logger().warn(f"[{self.name}] 清除故障失败")

    def open(self):
        """张开并确认到位；未确认自动重发重试，最多 COMMAND_RETRIES 次。
        返回是否最终确认到位（反馈不可用时按盲发成功处理）。"""
        with self._cmd_lock:
            if not self._ensure_clients():
                self._node.get_logger().error(
                    f"[{self.name}] 夹爪服务客户端不可用，无法张开")
                return False
            for attempt in range(1, COMMAND_RETRIES + 1):
                req = self._Setmovemax.Request()
                req.speed = self._open_speed
                req.gripper_id = self._id
                req.status = 'set_movemax'
                ack_ok = self._call_client(self._setmovemax_cli, req,
                                           'movemax_accepted')
                if not ack_ok:
                    self._node.get_logger().warn(
                        f"[{self.name}] Setmovemax 无应答/未确认"
                        f"（第 {attempt}/{COMMAND_RETRIES} 次）")
                if self.wait_opened():
                    if attempt > 1:
                        self._node.get_logger().info(
                            f"[{self.name}] 第 {attempt} 次尝试张开到位")
                    return True
                self._node.get_logger().warn(
                    f"[{self.name}] 张开未到位，重试 "
                    f"{attempt}/{COMMAND_RETRIES}...")
            self._node.get_logger().warn(
                f"[{self.name}] 张开未确认到位（已重试 {COMMAND_RETRIES} 次）")
            return False

    def close(self):
        # 闭到最小位置（力度 600，范围 50~1000；原来 200 偏小，瓶子容易滑）
        with self._cmd_lock:
            if not self._ensure_clients():
                self._node.get_logger().error(
                    f"[{self.name}] 夹爪服务客户端不可用，无法闭合")
                return
            req = self._Setmovemin.Request()
            req.speed = self._close_speed
            req.power = self._close_force
            req.gripper_id = self._id
            req.status = 'set_movemin'
            ok = self._send_with_retry(
                "/Setmovemin", self._setmovemin_cli, req, 'movemin_accepted')
            if not ok:
                self._node.get_logger().warn(
                    f"[{self.name}] 闭合命令无应答/未确认"
                    f"（已重试 {COMMAND_RETRIES} 次）")
            # 闭合后持续保持抓取力，防止搬运/放置过程中回退松手
            time.sleep(0.1)
            req = self._Setmoveminhold.Request()
            req.speed = self._close_speed
            req.power = self._close_force
            req.gripper_id = self._id
            req.status = 'set_moveminhold'
            ok = self._send_with_retry(
                "/Setmoveminhold", self._setmoveminhold_cli, req,
                'moveminhold_accepted')
            if not ok:
                self._node.get_logger().warn(
                    f"[{self.name}] 保持力命令无应答/未确认"
                    f"（已重试 {COMMAND_RETRIES} 次）")

    def validate(self) -> bool:
        """检查夹爪服务是否在线。"""
        with self._cmd_lock:
            if not self._ensure_clients():
                self._node.get_logger().warn(
                    f"[{self.name}] service_interfaces 不可用，"
                    "请确认已安装并 source install/setup.bash")
                return False
            req = self._Getstatus.Request()
            req.gripper_id = self._id
            req.status = 'get_status'
            res = self._rc_call(self._getstatus_cli, req, timeout=10.0)
            if res is not None:
                self._node.get_logger().info(f"[{self.name}] 服务在线")
                return True
            self._node.get_logger().warn(
                f"[{self.name}] 服务不在线，请确认 Gripper_control_node 已启动")
            return False
