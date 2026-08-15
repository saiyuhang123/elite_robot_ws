#!/usr/bin/env python3
"""Inspire 4B4C 二指夹爪控制。

控制通道：12 个 ROS 2 服务（串口→ROS 桥接节点 inspire_gripper）。
写命令通过 subprocess 调用 ros2 service call，不依赖服务接口包编译；
读反馈（开口度/状态码）走直接 rclpy 客户端（毫秒级，可轮询），用于
动作到位确认与空夹检测。反馈通道不可用时自动降级为盲发（旧行为）。

重试策略：开/合命令无应答（回执丢失）或未确认到位时自动重发，
最多 COMMAND_RETRIES 次。重发幂等（重复开/合无副作用），
真到位与否以开口度反馈为准，回执丢失不视为失败。
"""

import subprocess
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


def _call_service(service: str, srv_type: str, request: str,
                  timeout: float = 10.0) -> bool:
    """调用 ros2 service call，返回是否成功。
    超时 10s: 高负载下仅子进程启动 + DDS 发现就要 2~5s，
    超时过短会丢回执误报失败（命令实际已执行）。"""
    cmd = ["ros2", "service", "call", service, srv_type, request]
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout)
        return result.returncode == 0
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False


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
        # 直接 rclpy 客户端（读反馈用，首次使用时惰性创建）
        self._cli_ready = None
        self._getcopen_cli = None
        self._getstatus_cli = None
        self._Getcopen = None
        self._Getstatus = None

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

    # ---------------- 读反馈（直接 rclpy 客户端） ----------------

    def _ensure_clients(self) -> bool:
        """惰性创建 Getcopen/Getstatus 客户端。失败则降级为盲发。"""
        if self._cli_ready is not None:
            return self._cli_ready
        try:
            from service_interfaces.srv import Getcopen, Getstatus
            self._Getcopen = Getcopen
            self._Getstatus = Getstatus
            self._getcopen_cli = self._node.create_client(
                Getcopen, '/Getcopen')
            self._getstatus_cli = self._node.create_client(
                Getstatus, '/Getstatus')
            self._cli_ready = True
        except Exception as exc:
            self._node.get_logger().warn(
                f"[{self.name}] 反馈通道不可用({exc})，动作确认降级为盲发")
            self._cli_ready = False
        return self._cli_ready

    def _rc_call(self, client, req, timeout=1.5):
        """阻塞等响应（后台 MultiThreadedExecutor 在 spin），超时返回 None。"""
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

    def _send_with_retry(self, service, srv_type, request,
                         retries=COMMAND_RETRIES) -> bool:
        """发命令，无应答自动重发，最多 retries 次。返回最终是否有应答。
        重发幂等；单次回执只等 CMD_ACK_TIMEOUT（回执丢了靠重发补上）。"""
        for attempt in range(1, retries + 1):
            if _call_service(service, srv_type, request,
                             timeout=CMD_ACK_TIMEOUT):
                return True
            if attempt < retries:
                self._node.get_logger().warn(
                    f"[{self.name}] {service} 无应答，重发 "
                    f"{attempt}/{retries - 1}...")
        return False

    def setup(self):
        """清除故障、设置张开限位。"""
        _call_service("/Setclearerror",
                      "service_interfaces/srv/Setclearerror",
                      f"{{gripper_id: {self._id}, status: 'set_clearerror'}}")
        self._node.get_logger().info(f"[{self.name}] 已清除故障")

    def open(self):
        """张开并确认到位；未确认自动重发重试，最多 COMMAND_RETRIES 次。
        返回是否最终确认到位（反馈不可用时按盲发成功处理）。"""
        req = (f"{{speed: {self._open_speed}, "
               f"gripper_id: {self._id}, status: 'set_movemax'}}")
        for attempt in range(1, COMMAND_RETRIES + 1):
            _call_service("/Setmovemax", "service_interfaces/srv/Setmovemax",
                          req, timeout=CMD_ACK_TIMEOUT)
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
        ok = self._send_with_retry(
            "/Setmovemin", "service_interfaces/srv/Setmovemin",
            f"{{speed: {self._close_speed}, "
            f"power: {self._close_force}, "
            f"gripper_id: {self._id}, status: 'set_movemin'}}")
        if not ok:
            self._node.get_logger().warn(
                f"[{self.name}] 闭合命令无应答（已重试 {COMMAND_RETRIES} 次）")
        # 闭合后持续保持抓取力，防止搬运/放置过程中回退松手
        time.sleep(0.1)
        ok = self._send_with_retry(
            "/Setmoveminhold", "service_interfaces/srv/Setmoveminhold",
            f"{{speed: {self._close_speed}, "
            f"power: {self._close_force}, "
            f"gripper_id: {self._id}, status: 'set_moveminhold'}}")
        if not ok:
            self._node.get_logger().warn(
                f"[{self.name}] 保持力命令无应答（已重试 {COMMAND_RETRIES} 次）")

    def validate(self) -> bool:
        """检查夹爪服务是否在线。"""
        ok = _call_service("/Getstatus",
                           "service_interfaces/srv/Getstatus",
                           f"{{gripper_id: {self._id}, status: 'get_status'}}",
                           timeout=10.0)
        if ok:
            self._node.get_logger().info(f"[{self.name}] 服务在线")
        else:
            self._node.get_logger().warn(
                f"[{self.name}] 服务不在线，请确认 Gripper_control_node 已启动")
        return ok
