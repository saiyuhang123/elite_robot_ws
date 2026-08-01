#!/usr/bin/env python3
"""多姿态实测拟合工具重力向量（替代 7-27 预留值）。

原理: 原始力 = R_fts^T * g_base + tare(传感器系常数)
      R_fts = FK(base_link -> ft_frame)，由关节角 + URDF 计算。
      多姿态最小二乘解出 g_base(工具重力在 base 系) 和 tare。

用法:
  1. 机械臂静止、无接触，取 5 个差异大的姿态，每个姿态记录:
       ros2 topic echo /force_torque_sensor_broadcaster/wrench --once   # 原始力 fx fy fz
       ros2 topic echo /joint_states --once                             # 6 个 cs66_* 关节角(rad)
  2. 写入 /tmp/gravity_meas.txt，每行: 6 个关节角(rad) + fx fy fz
  3. python3 biaoding/fit_tool_gravity.py
  4. 把输出的 g_base 填到 polish_params.yaml 的 tool_gravity
"""
import sys
import numpy as np

URDF_XACRO = "/home/nvidia/Documents/elite_robot_ws/src/kybot_elite_robot_cell_description/urdf/kybot_elite_robot_cell.urdf.xacro"
JOINT_ORDER = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def load_robot():
    import subprocess, tempfile, os
    from urdf_parser_py.urdf import URDF
    with tempfile.NamedTemporaryFile(suffix=".urdf", delete=False) as f:
        subprocess.run(["xacro", URDF_XACRO], stdout=f, check=True)
        urdf_path = f.name
    try:
        return URDF.from_xml_file(urdf_path)
    finally:
        os.unlink(urdf_path)


def rot_rpy(rpy):
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def fk_to_tip(robot, root, tip, qmap):
    """URDF 手写 FK: 从 root 沿关节树走到 tip，返回 4x4 齐次矩阵。"""
    T = np.eye(4)

    def walk(link, T):
        if link == tip:
            return T
        for j in robot.joints:
            if j.parent != link:
                continue
            Tj = np.eye(4)
            Tj[0:3, 0:3] = rot_rpy(j.origin.rpy)
            Tj[0:3, 3] = j.origin.xyz
            if j.type == "revolute" or j.type == "continuous":
                q = qmap.get(j.name, 0.0)
                Rq = np.eye(4)
                Rq[0:3, 0:3] = np.array([[np.cos(q), -np.sin(q), 0],
                                         [np.sin(q), np.cos(q), 0],
                                         [0, 0, 1]])
                Tj = Tj @ Rq
            r = walk(j.child, T @ Tj)
            if r is not None:
                return r
        return None

    result = walk(root, T)
    if result is None:
        raise RuntimeError(f"URDF 中找不到链 {root} -> {tip}")
    return result


def load_measurements(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            v = [float(x) for x in line.split()]
            if len(v) != 9:
                print(f"跳过格式错误行: {line}")
                continue
            rows.append(v)
    return rows


def main():
    if len(sys.argv) > 1:
        meas_path = sys.argv[1]
    else:
        meas_path = "/tmp/gravity_meas.txt"
    rows = load_measurements(meas_path)
    if len(rows) < 2:
        print(f"至少需要 2 组测量（建议 5 组），当前 {len(rows)} 组")
        return

    robot = load_robot()

    A = []   # 每行: [R^T 3x3 | I 3x3] -> 9 列
    b = []
    print("=== 测量输入 ===")
    for r in rows:
        qmap = {JOINT_ORDER[i]: r[i] for i in range(len(JOINT_ORDER))}
        T = fk_to_tip(robot, "cs66_base_link", "cs66_ft_frame", qmap)
        R = T[0:3, 0:3]
        f = np.array(r[6:9])
        # raw = R^T g + c  ->  [R^T | I] * [g; c] = raw
        A.append(np.hstack([R.T, np.eye(3)]))
        b.append(f)
        print(f"  关节 {np.round(r[0:6], 3)}  原始力 {np.round(f, 3)}")

    A = np.vstack(A)
    b = np.concatenate(b)
    sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    g_base = sol[0:3]
    c = sol[3:6]

    print("\n=== 拟合结果 ===")
    print(f"tool_gravity (base 系) = [{g_base[0]:.4f}, {g_base[1]:.4f}, {g_base[2]:.4f}]")
    print(f"  模长 {np.linalg.norm(g_base):.3f} N  (≈质量 {np.linalg.norm(g_base)/9.81:.3f} kg)")
    print(f"tare (传感器系常数) = [{c[0]:.4f}, {c[1]:.4f}, {c[2]:.4f}]")

    print("\n=== 拟合后残差（应≈0）===")
    for i, r in enumerate(rows):
        qmap = {JOINT_ORDER[j]: r[j] for j in range(len(JOINT_ORDER))}
        T = fk_to_tip(robot, "cs66_base_link", "cs66_ft_frame", qmap)
        R = T[0:3, 0:3]
        resid = np.array(r[6:9]) - R.T @ g_base - c
        print(f"  姿态 {i+1} 残差 {np.round(resid, 3)}")


if __name__ == "__main__":
    main()
