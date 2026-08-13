#!/usr/bin/env python3
"""分析留底点云: 以尖端为板面锚点, 沿世界向下法向分层, 量板面范围/地面位置, 给出 target_box 建议。"""
import numpy as np

PCD = "/tmp/base_calib.pcd"
TIP = np.array([0.8687, -0.1098, 0.2706])       # grinder_tip 实测板心
N = np.array([0.7431, -0.0120, -0.6691])       # 世界向下(压入方向), base 系
N = N / np.linalg.norm(N)

with open(PCD, "rb") as f:
    data = f.read()
idx = data.find(b"DATA binary\n") + len(b"DATA binary\n")
pts = np.frombuffer(data[idx:], dtype=np.float32).reshape(-1, 3)
print(f"总点数: {len(pts)}")

# 沿法向距离: d<0 在板面上方(相机侧), d>0 在板面之下
d = (pts - TIP) @ N

# 分层统计, 找板面层和地面层
for lo, hi, name in [(-0.30, -0.02, "板上方(应少)"), (-0.02, 0.005, "板面层"),
                     (0.005, 0.03, "板下1(地面?)"), (0.03, 0.10, "板下2"), (0.10, 0.40, "更下")]:
    print(f"  d∈[{lo:+.2f},{hi:+.2f}] {name}: {np.sum((d>=lo)&(d<hi))}")

# 板面候选: 锚点附近薄层
board = pts[(d > -0.012) & (d < 0.012)]
print(f"\n板面候选点数(d∈±12mm): {len(board)}")

# 平面内坐标: up_ref=世界Y
up = np.array([0.0, 1.0, 0.0])
py = up - (up @ N) * N; py /= np.linalg.norm(py)
px = np.cross(py, N); px /= np.linalg.norm(px)
u = (board - TIP) @ px
v = (board - TIP) @ py

def pct(a, q): return np.percentile(a, q)
u1, u99 = pct(u, 1), pct(u, 99)
v1, v99 = pct(v, 1), pct(v, 99)
print(f"板面范围: u(扫掠向) [{u1:.3f},{u99:.3f}] 宽 {u99-u1:.3f}m")
print(f"          v(另一向) [{v1:.3f},{v99:.3f}] 宽 {v99-v1:.3f}m")
print(f"板心相对尖端偏移: du={(u1+u99)/2:+.3f} dv={(v1+v99)/2:+.3f}")

# 板面四角(1%/99%分位)换算回 base 系, 加余量生成 target_box
m = 0.015  # x/y余量(收紧, 减少地面混入)
corners = []
for uu in (u1 - m, u99 + m):
    for vv in (v1 - m, v99 + m):
        corners.append(TIP + uu * px + vv * py)
corners = np.array(corners)
bmin = corners.min(axis=0)
bmax = corners.max(axis=0)

# 板面在 base 系是斜的(扫掠向 z 落差大), z 下限必须按最低板面角算, 否则切掉扫掠远端;
# 同时不能低于地面太多(地面沿法向在板下2~3cm, 会混入)。
surf_z_min = corners[:, 2].min()
zmax = board[:, 2].max() + 0.05
zmin = surf_z_min - 0.008

# 检查: 该盒内地面点(d>15mm)混入多少
box = pts[(pts[:,0]>bmin[0])&(pts[:,0]<bmax[0])&(pts[:,1]>bmin[1])&(pts[:,1]<bmax[1])&(pts[:,2]>zmin)&(pts[:,2]<zmax)]
db = (box - TIP) @ N
print(f"\n建议 target_box:")
print(f"  target_box_min: [{bmin[0]:.3f}, {bmin[1]:.3f}, {zmin:.3f}]")
print(f"  target_box_max: [{bmax[0]:.3f}, {bmax[1]:.3f}, {zmax:.3f}]")
print(f"盒内总点 {len(box)}, 其中板面(d±12mm) {np.sum(np.abs(db)<0.012)}, 深层(d>15mm, 地面/杂物) {np.sum(db>0.015)}")

print(f"\nplane_point_o: [{TIP[0]:.4f}, {TIP[1]:.4f}]")
