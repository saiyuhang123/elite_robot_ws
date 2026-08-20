#!/usr/bin/env python3
"""柔触三指气动夹爪控制。

控制通道：/gripper_command 服务（Modbus TCP→ROS 桥接节点 gripper_server）。
使用 rclpy 长驻 client 调用服务，不再每次命令启动 ros2 CLI 子进程，
避免高 CPU 负载下临时进程创建/DDS 发现被 3 秒超时误杀。
"""

import threading
import time

import numpy as np
from std_msgs.msg import Float32

from .base import GripperBase


# 服务调用超时参数（秒）。CPU 高时服务响应可能变慢，这里留足余量。
SERVICE_READY_TIMEOUT = 5.0
CMD_ACK_TIMEOUT = 8.0
SERVICE_READY_POLL = 0.05

# 张开确认：正压压力低于该值才认为已经松气（防止带压回 Home2）。
OPEN_RELEASE_PRESSURE_MAX_KPA = 80.0
OPEN_RELEASE_TIMEOUT = 10.0
OPEN_RELEASE_POLL = 0.1


class SoftTouchGripper(GripperBase):
    """柔触三指气动夹爪。

    依赖: gripper_control 包的 gripper_server 已运行。
    默认 Modbus TCP: 192.168.3.200:502。

    参数:
        pressure_limit:  正压上限 (kPa)
        vacuum_value:    负压值 (kPa, 负数)
        tip_length:      法兰面到指尖平面的工具偏移（米）
    """

    def __init__(self, robot_node,
                 pressure_limit: int = 280,
                 vacuum_value: int = -50,
                 tip_length: float = 0.19):
        self._node = robot_node
        self._pressure_limit = pressure_limit
        self._vacuum_value = vacuum_value
        self._tip_length = tip_length
        self._pressure_value = None
        self._pressure_time = 0.0
        self._pressure_seq = 0
        self._pressure_sub = robot_node.create_subscription(
            Float32, '/gripper_pressure', self._pressure_cb, 10)

        # 柔触命令串行锁（RLock：open/close 整体持锁时内部调用可重入）。
        self._cmd_lock = threading.RLock()

        # 长驻 rclpy client：创建一次，后续所有命令复用。
        # srv 类型在这里按需导入并保存到实例，避免未构建 gripper_control 时
        # 影响二指/灵巧手等其他末端模式的 import，同时保证 _request() 可用。
        self._gripper_cli = None
        self._srv_type = None
        try:
            from gripper_control.srv import GripperCommand
            self._srv_type = GripperCommand
            self._gripper_cli = robot_node.create_client(
                self._srv_type, '/gripper_command')
        except Exception as exc:
            self._node.get_logger().error(
                f'[{self.name}] /gripper_command 客户端创建失败: {exc}')

    def _pressure_cb(self, msg):
        """缓存柔触控制器轮询发布的实际正压值（kPa）。"""
        self._pressure_value = float(msg.data)
        self._pressure_time = time.time()
        self._pressure_seq += 1

    @property
    def name(self) -> str:
        return "soft_touch"

    @property
    def ik_mode(self) -> str:
        # 锁定完整法兰姿态，避免低位抓取时 5dof 自由自转选到翻腕解。
        return "6dof"

    @property
    def close_delay(self) -> float:
        return 2.0  # 气动建立正压需要时间

    @property
    def pressure_limit(self) -> float:
        return float(self._pressure_limit)

    @property
    def pressure_tolerance(self) -> float:
        return 5.0

    @property
    def grasp_offset_world(self) -> np.ndarray:
        # 与二指一致：指尖平面（TCP）对准目标点，不上抬
        return np.array([0.0, 0.0, 0.0])

    @property
    def tool_length(self) -> float:
        return self._tip_length

    # ---------------- 长驻 service client 调用 ----------------

    def _wait_service_ready(self, timeout: float = SERVICE_READY_TIMEOUT) -> bool:
        """等待 /gripper_command 服务上线。后台 MultiThreadedExecutor 在 spin，
        这里只轮询，不临时启动任何进程。"""
        if self._gripper_cli is None:
            self._node.get_logger().error(
                f'[{self.name}] /gripper_command 客户端不可用')
            return False
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                if self._gripper_cli.service_is_ready():
                    return True
            except Exception:
                pass
            time.sleep(SERVICE_READY_POLL)
        self._node.get_logger().error(
            f'[{self.name}] 等待 /gripper_command 服务超时'
            f'（{timeout:.1f}s），请确认 gripper_server 已启动')
        return False

    def _request(self, command: int, value: int = 0, slave_id: int = 1,
                 timeout: float = CMD_ACK_TIMEOUT):
        """向 /gripper_command 发送一次请求，返回 Response 或 None。"""
        if not self._wait_service_ready():
            return None
        if self._srv_type is None:
            self._node.get_logger().error(
                f'[{self.name}] GripperCommand 类型未初始化')
            return None

        req = self._srv_type.Request()
        req.command = command
        req.value = value
        req.slave_id = slave_id

        try:
            future = self._gripper_cli.call_async(req)
        except Exception as exc:
            self._node.get_logger().error(
                f'[{self.name}] /gripper_command 请求发送失败: {exc}')
            return None

        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            self._node.get_logger().error(
                f'[{self.name}] /gripper_command 无应答'
                f'（command={command}, timeout={timeout:.1f}s）')
            try:
                self._gripper_cli.remove_pending_request(future)
            except Exception:
                pass
            return None

        try:
            res = future.result()
        except Exception as exc:
            self._node.get_logger().error(
                f'[{self.name}] /gripper_command 响应异常: {exc}')
            return None

        if not res.success:
            self._node.get_logger().error(
                f'[{self.name}] /gripper_command 执行失败'
                f'（command={command}）: {res.message}')
            return None
        return res

    def _call_gripper(self, command: int, value: int = 0, slave_id: int = 1,
                      timeout: float = CMD_ACK_TIMEOUT) -> bool:
        """调用 /gripper_command 服务。返回服务端 ack 是否成功。"""
        return self._request(command, value, slave_id, timeout) is not None

    # ---------------- 指令 ----------------

    def setup(self):
        """设置压力参数。返回是否两条参数命令均 ack 成功。"""
        with self._cmd_lock:
            ok1 = self._call_gripper(4, self._pressure_limit)  # 正压上限
            ok2 = self._call_gripper(5, self._vacuum_value)     # 负压值
            if ok1 and ok2:
                self._node.get_logger().info(
                    f'[{self.name}] 压力参数已设置: +{self._pressure_limit}/'
                    f'{self._vacuum_value} kPa')
                return True
            self._node.get_logger().error(
                f'[{self.name}] 压力参数设置失败，柔触可能未初始化')
            return False

    def wait_pressure_released(self,
                               timeout: float = OPEN_RELEASE_TIMEOUT,
                               max_pressure: float = OPEN_RELEASE_PRESSURE_MAX_KPA) -> bool:
        """等待实际正压降到 max_pressure 以下（松气确认）。"""
        deadline = time.time() + timeout
        last_log = 0.0
        while time.time() < deadline:
            now = time.time()
            value = self._pressure_value
            fresh = value is not None and now - self._pressure_time <= 1.0
            if fresh and value <= max_pressure:
                self._node.get_logger().info(
                    f'[{self.name}] 已松气（当前 {value:.0f} kPa ≤ '
                    f'{max_pressure:.0f} kPa）')
                return True
            if now - last_log >= 1.0:
                last_log = now
                text = (f'{value:.0f} kPa' if fresh else '无新鲜气压数据')
                self._node.get_logger().info(
                    f'[{self.name}] 等待正压释放到 {max_pressure:.0f} kPa '
                    f'以下，当前 {text}')
            time.sleep(OPEN_RELEASE_POLL)
        self._node.get_logger().error(
            f'[{self.name}] 正压未在 {timeout:.0f}s 内释放，'
            f'禁止后续运动')
        return False

    def open(self):
        """负压张开（松掉正压，短暂抽负压把手指撑开，再松气）。

        返回 True 仅当三条命令均 ack 且 /gripper_pressure 确认正压已释放；
        否则返回 False，调用方不得继续带压运动。
        """
        with self._cmd_lock:
            ok = True
            ok &= self._call_gripper(1)  # 正压松气（防止还保持着闭合压力）
            ok &= self._call_gripper(2)  # 启动负压，手指张开
            time.sleep(0.3)
            ok &= self._call_gripper(3)  # 负压松气
            if not ok:
                self._node.get_logger().error(f'[{self.name}] 张开命令失败')
                return False
            if not self.wait_pressure_released():
                self._node.get_logger().error(
                    f'[{self.name}] 张开后未确认松气，按失败处理')
                return False
            self._node.get_logger().info(f'[{self.name}] 已张开')
            return True

    def close(self):
        """正压闭合（手指充气卷曲，保持压力夹住物体）。

        返回 True 表示两条命令均 ack；实际压力是否达标由
        wait_pressure_ready() 在抓取流程中进一步确认。
        """
        with self._cmd_lock:
            ok = True
            ok &= self._call_gripper(3)  # 负压松气（防止还保持着张开负压）
            ok &= self._call_gripper(0)  # 启动正压，保持不松气
            if not ok:
                self._node.get_logger().error(f'[{self.name}] 闭合命令失败')
                return False
            self._node.get_logger().info(f'[{self.name}] 已闭合（正压保持）')
            return True

    def wait_pressure_ready(self, timeout: float = 20.0,
                            stable_samples: int = 3,
                            tolerance=None) -> bool:
        """等待实际正压连续多帧进入目标 ± tolerance 区间。"""
        target = float(self._pressure_limit)
        tolerance = (self.pressure_tolerance
                     if tolerance is None else float(tolerance))
        lower = target - tolerance
        upper = target + tolerance
        deadline = time.time() + timeout
        hits = 0
        last_log = 0.0
        last_seq = self._pressure_seq
        while time.time() < deadline:
            now = time.time()
            value = self._pressure_value
            fresh = value is not None and now - self._pressure_time <= 1.0
            if self._pressure_seq != last_seq:
                last_seq = self._pressure_seq
                if fresh and lower <= value <= upper:
                    hits += 1
                    if hits >= stable_samples:
                        self._node.get_logger().info(
                            f"[{self.name}] 实际正压已达标: "
                            f"{value:.0f} kPa（允许 {lower:.0f}~{upper:.0f} kPa）")
                        return True
                else:
                    hits = 0
            if now - last_log >= 1.0:
                last_log = now
                text = (f"{value:.0f} kPa" if fresh else "无新鲜气压数据")
                self._node.get_logger().info(
                    f"[{self.name}] 等待正压进入 "
                    f"{lower:.0f}~{upper:.0f} kPa，"
                    f"当前 {text}")
            time.sleep(0.05)
        value_text = (f"{self._pressure_value:.0f} kPa"
                      if self._pressure_value is not None else "无数据")
        self._node.get_logger().error(
            f"[{self.name}] {timeout:.0f}s 内正压未进入 "
            f"{lower:.0f}~{upper:.0f} kPa（当前 {value_text}）")
        return False

    def validate(self) -> bool:
        """检查服务是否在线。"""
        ok = self._call_gripper(6, timeout=2.0)  # 读正压反馈
        if ok:
            self._node.get_logger().info(f"[{self.name}] 服务在线")
        else:
            self._node.get_logger().warn(
                f"[{self.name}] 服务不在线，请确认 gripper_server 已启动")
        return ok
