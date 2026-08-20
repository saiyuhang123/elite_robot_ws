#!/usr/bin/env python3
"""柔触三指气动夹爪控制。

控制通道：/gripper_command 服务（Modbus TCP→ROS 桥接节点 gripper_server）。
通过 subprocess 调用 ros2 service call。
"""

import subprocess
import time
import numpy as np
from std_msgs.msg import Float32

from .base import GripperBase


def _call_gripper(command: int, value: int = 0, slave_id: int = 1,
                  timeout: float = 3.0) -> bool:
    """调用 /gripper_command 服务。"""
    cmd = ["ros2", "service", "call", "/gripper_command",
           "gripper_control/srv/GripperCommand",
           f"{{command: {command}, value: {value}, slave_id: {slave_id}}}"]
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout)
        return result.returncode == 0
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False


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

    def setup(self):
        """设置压力参数。"""
        _call_gripper(4, self._pressure_limit)  # 正压上限
        _call_gripper(5, self._vacuum_value)     # 负压值
        self._node.get_logger().info(
            f"[{self.name}] 压力参数已设置: +{self._pressure_limit}/"
            f"{self._vacuum_value} kPa")

    def open(self):
        """负压张开（松掉正压，短暂抽负压把手指撑开，再松气）。"""
        _call_gripper(1)  # 正压松气（防止还保持着闭合压力）
        _call_gripper(2)  # 启动负压，手指张开
        time.sleep(0.3)
        _call_gripper(3)  # 负压松气
        self._node.get_logger().info(f"[{self.name}] 已张开")

    def close(self):
        """正压闭合（手指充气卷曲，保持压力夹住物体）。"""
        _call_gripper(3)  # 负压松气（防止还保持着张开负压）
        _call_gripper(0)  # 启动正压，保持不松气
        self._node.get_logger().info(f"[{self.name}] 已闭合（正压保持）")

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
        ok = _call_gripper(6, timeout=2.0)  # 读正压反馈
        if ok:
            self._node.get_logger().info(f"[{self.name}] 服务在线")
        else:
            self._node.get_logger().warn(
                f"[{self.name}] 服务不在线，请确认 gripper_server 已启动")
        return ok
