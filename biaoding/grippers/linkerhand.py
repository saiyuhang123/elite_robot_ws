#!/usr/bin/env python3
"""LinkerHand O6 灵巧手控制。"""

import json
import time
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .base import GripperBase


class LinkerHandGripper(GripperBase):
    """LinkerHand O6 六指灵巧手。

    控制通道:
      - /cb_right_hand_control_cmd   (JointState, 0~255)
      - /cb_hand_setting_cmd         (String, JSON，SDK 的公共设置入口)
      - /cb_right_hand_state         (JointState，SDK/CAN 实时状态)
    """

    HAND_TYPE = "right"
    HAND_CMD_TOPIC = "/cb_right_hand_control_cmd"
    HAND_SETTING_TOPIC = "/cb_hand_setting_cmd"
    HAND_STATE_TOPIC = "/cb_right_hand_state"

    # 关节顺序: [大拇指弯曲, 大拇指横摆, 食指, 中指, 无名指, 小拇指]
    HAND_OPEN_POSE = [255.0] * 6                    # 五指张开
    HAND_CAGE_POSE = [180.0, 25.0, 180.0, 180.0, 180.0, 180.0]  # 半闭合成笼（不发力）
    HAND_CLOSE_POSE = [0.0, 5.0, 50.0, 50.0, 50.0, 50.0]  # 握拳

    def __init__(self, robot_node, speed: int = 20, torque: int = 30):
        self._node = robot_node
        self._speed = speed
        self._torque = torque
        self._last_state_at = 0.0
        self._hand_pub = robot_node.create_publisher(
            JointState, self.HAND_CMD_TOPIC, 10)
        self._setting_pub = robot_node.create_publisher(
            String, self.HAND_SETTING_TOPIC, 10)
        self._state_sub = robot_node.create_subscription(
            JointState, self.HAND_STATE_TOPIC, self._on_hand_state, 10)

    def default_grasp_rotation(self, v_up_in_base):
        return None  # 灵巧手必须手动示教，不能自动构造

    def grasp_rotation(self, world_x_in_base, v_up_in_base):
        """灵巧手固定抓取姿态：法兰面（法兰 Z）朝世界 X+，
        手沿法兰 Z 水平伸出；法兰 Y = 世界正下方 → 手心朝下。"""
        z = np.asarray(world_x_in_base, dtype=float)
        z /= np.linalg.norm(z)
        y = -np.asarray(v_up_in_base, dtype=float)
        y /= np.linalg.norm(y)
        x = np.cross(y, z)
        return np.column_stack([x, y, z])

    @property
    def needs_orientation_calibration(self) -> bool:
        return True  # 手心朝向有要求，必须示教

    @property
    def name(self) -> str:
        return "linkerhand"

    @property
    def ik_mode(self) -> str:
        return "6dof"  # 手心朝向重要 

    @property
    def close_delay(self) -> float:
        """闭合后等待手指抓稳，再允许机械臂上抬。"""
        return 4.0

    @property
    def grasp_offset_world(self) -> np.ndarray:
        # 掌心 Z 偏移（相对 YOLO 检测到的物体上表面）。
        # 手水平伸出、手心朝下姿态下，握拳后指尖会低于手心平面，
        # 偏移太小会压住物体。苹果可先试 0.06，压物就调大、抓空就调小。
        return np.array([0.0, 0.0, 0.03])  # 手心在物体上表面上方 6cm

    @property
    def tool_length(self) -> float:
        return 0.13

    def setup(self):
        # 当前 LinkerHand SDK 的设置入口要求 params.hand_type，并使用
        # set_max_torque_limits 作为扭矩命令名。
        for cmd, values in (
                ("set_speed", {"speed": [self._speed] * 6}),
                ("set_max_torque_limits",
                 {"torque": [self._torque] * 6})):
            msg = String()
            params = {"hand_type": self.HAND_TYPE}
            params.update(values)
            msg.data = json.dumps({"setting_cmd": cmd, "params": params})
            self._setting_pub.publish(msg)
        return True

    def open(self):
        self._publish_positions(self.HAND_OPEN_POSE)

    def close_cage(self):
        """半闭合成笼：手指弯成笼子罩住物体但不发力，
        用于两段式闭合的第一段（笼住→抬起→攥紧）。"""
        self._publish_positions(self.HAND_CAGE_POSE)

    def close(self):
        self._publish_positions(self.HAND_CLOSE_POSE)

    def _publish_positions(self, positions):
        msg = JointState()
        msg.position = [float(p) for p in positions]
        self._hand_pub.publish(msg)

    def _on_hand_state(self, _msg):
        self._last_state_at = time.monotonic()

    def validate(self) -> bool:
        """命令有 SDK 接收且持续收到状态帧，才认为 SDK/CAN 已就绪。"""
        state_age = time.monotonic() - self._last_state_at
        return (self._node.count_subscribers(self.HAND_CMD_TOPIC) > 0
                and self._last_state_at > 0.0
                and state_age <= 3.0)
