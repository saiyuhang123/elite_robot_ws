#!/usr/bin/env python3
"""世界系偏移 -> 基座系坐标 换算器。

已知目标点的基座系坐标，计算"沿世界系某方向偏移一段距离"后的基座系坐标。
典型用途：
  - 世界系竖直抬高 10cm 的预抓取点在哪
  - 世界系 -X 侧移 20cm 的点在哪

用法：
  python3 world_offset_calc.py
  然后按提示输入目标点（基座系）和世界系偏移量

世界系定义（倾斜安装实测，与 visual_grasp_test.py 一致）：
  世界 Z（竖直向上）= V_UP_IN_BASE
  世界 X/Y = 由 V_UP 构建的正交水平基
"""

import numpy as np

# 世界系"上"在基座系下的方向（实测值，可用 calibrate_vertical.py 重新测量）
V_UP_IN_BASE = np.array([-0.7431, 0.0120, 0.6691])


def build_world_axes(v_up):
    """由世界的"上"构建正交的世界系 X/Y 轴（以基座 Y 为参考）。"""
    up = np.asarray(v_up, dtype=float)
    up /= np.linalg.norm(up)
    y = np.array([0.0, 1.0, 0.0])
    y = y - (y @ up) * up
    y /= np.linalg.norm(y)
    x = np.cross(y, up)
    return x, y, up


WX, WY, WZ = build_world_axes(V_UP_IN_BASE)


def world_offset_to_base(target_base, offset_world):
    """目标点(基座系) + 世界系偏移 -> 结果点(基座系)"""
    dx, dy, dz = offset_world
    return np.asarray(target_base, dtype=float) + dx * WX + dy * WY + dz * WZ


def main():
    print("=" * 60)
    print("世界系偏移 -> 基座系坐标 换算器")
    print(f"世界 X 在基座系: {np.round(WX, 4)}")
    print(f"世界 Y 在基座系: {np.round(WY, 4)}")
    print(f"世界 Z 在基座系: {np.round(WZ, 4)}  (倾斜角 "
          f"{np.degrees(np.arccos(np.clip(WZ[2], -1, 1))):.1f}°)")
    print("=" * 60)
    print("输入目标点的基座系坐标 x y z（米），回车；q 退出\n")

    while True:
        try:
            line = input("目标点(基座系 x y z)> ").strip()
        except EOFError:
            break
        if not line or line.lower() == 'q':
            break
        try:
            target = np.array([float(v) for v in line.split()])
            assert target.shape == (3,)
        except (ValueError, AssertionError):
            print("  格式错误，输入 3 个数字，如: 0.55 -0.08 0.79")
            continue

        while True:
            try:
                line2 = input("  世界系偏移(dx dy dz, 直接回车=竖直抬高)> ").strip()
            except EOFError:
                return
            if not line2:
                off = np.array([0.0, 0.0, 0.10])  # 默认抬高 10cm
            else:
                try:
                    off = np.array([float(v) for v in line2.split()])
                    assert off.shape == (3,)
                except (ValueError, AssertionError):
                    print("  格式错误，输入 3 个数字，如: 0 0 0.1 或 -0.2 0 0")
                    continue

            result = world_offset_to_base(target, off)
            delta = result - target
            print(f"  基座系位移量: [{delta[0]: .4f}, {delta[1]: .4f}, {delta[2]: .4f}]")
            print(f"  结果点(基座系): [{result[0]:.4f}, {result[1]:.4f}, {result[2]:.4f}]")
            print(f"  距肩关节: {np.linalg.norm(result - np.array([0, 0, 0.1625])):.3f}m"
                  + ("  !! 超臂展0.92m" if np.linalg.norm(result - np.array([0, 0, 0.1625])) > 0.92 else ""))
            print()


if __name__ == '__main__':
    main()
