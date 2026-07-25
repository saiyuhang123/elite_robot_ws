#!/usr/bin/env python3
"""夹爪工厂。"""

from .base import GripperBase
from .linkerhand import LinkerHandGripper
from .two_finger import TwoFingerGripper
from .soft_touch import SoftTouchGripper

GRIPPER_REGISTRY = {
    "linkerhand": LinkerHandGripper,
    "two_finger": TwoFingerGripper,
    "soft_touch": SoftTouchGripper,
}


def create_gripper(name: str, robot_node, **kwargs) -> GripperBase:
    """创建夹爪实例。

    Args:
        name: 夹爪名称（linkerhand / two_finger / soft_touch）
        robot_node: ROS 2 Node 实例（用于创建 publisher/client）
        **kwargs: 传递给具体夹爪类的额外参数
    """
    cls = GRIPPER_REGISTRY.get(name)
    if cls is None:
        raise ValueError(
            f"未知夹爪: '{name}'，可用: {list(GRIPPER_REGISTRY.keys())}")
    return cls(robot_node, **kwargs)
