#!/usr/bin/env python3
"""柔触三指气动夹爪控制。

控制通道：/gripper_command 服务（Modbus TCP→ROS 桥接节点 gripper_server）。
通过 subprocess 调用 ros2 service call。
"""

import subprocess
import time
import numpy as np

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
                 pressure_limit: int = 150,
                 vacuum_value: int = -50,
                 tip_length: float = 0.19):
        self._node = robot_node
        self._pressure_limit = pressure_limit
        self._vacuum_value = vacuum_value
        self._tip_length = tip_length

    @property
    def name(self) -> str:
        return "soft_touch"

    @property
    def ik_mode(self) -> str:
        return "5dof"  # 三指 120° 旋转对称，放开自转，可达域更大

    @property
    def close_delay(self) -> float:
        return 2.0  # 气动建立正压需要时间

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

    def validate(self) -> bool:
        """检查服务是否在线。"""
        ok = _call_gripper(6, timeout=2.0)  # 读正压反馈
        if ok:
            self._node.get_logger().info(f"[{self.name}] 服务在线")
        else:
            self._node.get_logger().warn(
                f"[{self.name}] 服务不在线，请确认 gripper_server 已启动")
        return ok
