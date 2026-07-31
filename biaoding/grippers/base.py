#!/usr/bin/env python3
"""夹爪抽象基类。所有夹爪实现必须继承此类。"""

from abc import ABC, abstractmethod
import numpy as np


class GripperBase(ABC):
    """夹爪控制接口。

    子类只需实现：tcp_transform, open, close, setup, validate。
    主程序（yolo_grasp.py）通过本接口统一操作不同夹爪。
    """

    # ---- 子类必须覆盖的属性 ----

    @property
    @abstractmethod
    def name(self) -> str:
        """夹爪名称（linkerhand / two_finger / soft_touch）。"""
        ...

    @property
    @abstractmethod
    def ik_mode(self) -> str:
        """IK 模式: "5dof"（放开自转）或 "6dof"（完整约束）。"""
        ...

    @property
    def close_delay(self) -> float:
        """闭合后等待时间（秒）。"""
        return 1.5

    @property
    def grasp_offset_world(self) -> np.ndarray:
        """世界系抓取偏移 [dx, dy, dz]（米），掌心/指尖相对于目标点的偏移。
        正值 Z = 高于物体表面，负值 = 低于表面（包握）。"""
        return np.array([0.0, 0.0, 0.01])

    def default_grasp_rotation(self, v_up_in_base: "np.ndarray") -> "np.ndarray | None":
        """构造默认抓取姿态旋转矩阵。
        直装型夹爪：Z 轴 = 世界下方，X/Y 自动正交。
        返回 None 表示无法自动构造（必须手动示教）。

        Args:
            v_up_in_base: 世界"上"在基座系下的单位方向
        """
        import numpy as np
        z = -np.asarray(v_up_in_base, dtype=float)
        z /= np.linalg.norm(z)
        # X 从世界 X 投影到与 Z 垂直的平面
        x = np.array([1.0, 0.0, 0.0])
        x = x - (x @ z) * z
        x /= np.linalg.norm(x)
        y = np.cross(z, x)
        return np.column_stack([x, y, z])

    def grasp_rotation(self, world_x_in_base: "np.ndarray",
                       v_up_in_base: "np.ndarray") -> "np.ndarray":
        """抓取姿态旋转矩阵（基座系，列为法兰 X/Y/Z 轴）。
        默认（直装夹爪）：法兰 Z = 世界下方，竖直向下抓。
        安装方式特殊的夹爪（如灵巧手）应覆盖此方法。"""
        return self.default_grasp_rotation(v_up_in_base)

    @property
    def needs_orientation_calibration(self) -> bool:
        """是否需要 k 键示教抓取姿态。
        True  = 工具方向不在法兰 Z 轴上（如灵巧手），必须示教。
        False = 工具沿法兰 Z 轴直出（二指/柔触），可自动推导。"""
        return False

    @property
    def tool_length(self) -> float:
        """法兰面到 TCP（掌心/指尖中点）沿工具轴的距离（米）。"""
        return 0.11

    @property
    def tcp_transform(self) -> np.ndarray:
        """法兰→TCP 的 4x4 齐次变换矩阵。
        默认：法兰 Z 轴方向延伸 tool_length 的纯平移。
        子类可覆盖（如 LinkerHand 有横向拐弯）。"""
        return np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, self.tool_length],
            [0, 0, 0, 1],
        ])

    # ---- 子类必须实现的方法 ----

    @abstractmethod
    def setup(self):
        """初始化（设置速度/扭矩/压力等参数，SDK/节点运行期间有效）。"""
        ...

    @abstractmethod
    def open(self):
        """张开/释放。"""
        ...

    def close_cage(self):
        """半闭合成笼（两段式闭合第一段）。默认不支持半闭合，
        退化为直接闭合；支持的夹爪（如灵巧手）应覆盖。"""
        self.close()

    @abstractmethod
    def close(self):
        """闭合/抓取。"""
        ...

    def validate(self) -> bool:
        """启动自检：检查控制通道是否连通。返回 True 表示就绪。"""
        return True

    def is_grasping(self) -> bool:
        """可选：判断是否夹住物体。默认返回 True（不阻塞流程）。"""
        return True
