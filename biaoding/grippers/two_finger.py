#!/usr/bin/env python3
"""Inspire 4B4C 二指夹爪控制。

控制通道：12 个 ROS 2 服务（串口→ROS 桥接节点 inspire_gripper）。
通过 subprocess 调用 ros2 service call，不依赖服务接口包编译。
"""

import subprocess
import time
import numpy as np

from .base import GripperBase


def _call_service(service: str, srv_type: str, request: str,
                  timeout: float = 3.0) -> bool:
    """调用 ros2 service call，返回是否成功。"""
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

    def setup(self):
        """清除故障、设置张开限位。"""
        _call_service("/Setclearerror",
                      "service_interfaces/srv/Setclearerror",
                      f"{{gripper_id: {self._id}, status: 'set_clearerror'}}")
        self._node.get_logger().info(f"[{self.name}] 已清除故障")

    def open(self):
        ok = _call_service("/Setmovemax",
                           "service_interfaces/srv/Setmovemax",
                           f"{{speed: {self._open_speed}, "
                           f"gripper_id: {self._id}, status: 'set_movemax'}}")
        if not ok:
            self._node.get_logger().warn(f"[{self.name}] 张开命令可能失败")

    def close(self):
        # 闭到最小位置（力度 600，范围 50~1000；原来 200 偏小，瓶子容易滑）
        ok = _call_service("/Setmovemin",
                           "service_interfaces/srv/Setmovemin",
                           f"{{speed: {self._close_speed}, "
                           f"power: {self._close_force}, "
                           f"gripper_id: {self._id}, status: 'set_movemin'}}")
        if not ok:
            self._node.get_logger().warn(f"[{self.name}] 闭合命令可能失败")
        # 闭合后持续保持抓取力，防止搬运/放置过程中回退松手
        time.sleep(0.1)
        ok = _call_service("/Setmoveminhold",
                           "service_interfaces/srv/Setmoveminhold",
                           f"{{speed: {self._close_speed}, "
                           f"power: {self._close_force}, "
                           f"gripper_id: {self._id}, status: 'set_moveminhold'}}")
        if not ok:
            self._node.get_logger().warn(f"[{self.name}] 保持力命令可能失败")

    def validate(self) -> bool:
        """检查夹爪服务是否在线。"""
        ok = _call_service("/Getstatus",
                           "service_interfaces/srv/Getstatus",
                           f"{{gripper_id: {self._id}, status: 'get_status'}}",
                           timeout=2.0)
        if ok:
            self._node.get_logger().info(f"[{self.name}] 服务在线")
        else:
            self._node.get_logger().warn(
                f"[{self.name}] 服务不在线，请确认 Gripper_control_node 已启动")
        return ok
