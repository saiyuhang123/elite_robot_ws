#!/usr/bin/env python3
"""对 /tmp/base.pcd 做体素连通域聚类，把空间上分离的结构分开，
报告每一团的点数/包围盒/质心，用于识别工件、机械臂本体、噪声鬼影。

用法: python3 biaoding/pcd_cluster.py [pcd路径] [体素尺寸m，默认0.03]
"""
import sys
import numpy as np
from scipy import ndimage

sys.path.insert(0, '/home/nvidia/Documents/elite_robot_ws/biaoding')
from pcd_box_tool import load_pcd

pcd_path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/base.pcd'
voxel = float(sys.argv[2]) if len(sys.argv) > 2 else 0.03

pts = load_pcd(pcd_path)
pts = pts[~np.isnan(pts).any(axis=1)]
r = np.linalg.norm(pts, axis=1)
pts = pts[r < 2.0]
print(f'{pcd_path}: 2m 内 {len(pts)} 点, 体素 {voxel}m')

# 体素化
idx = np.floor(pts / voxel).astype(int)
origin = idx.min(axis=0)
idx -= origin
dims = idx.max(axis=0) + 1
grid = np.zeros(dims, dtype=bool)
grid[tuple(idx.T)] = True

# 26 邻接连通域
lbl, n = ndimage.label(grid, structure=np.ones((3, 3, 3)))
print(f'共 {n} 个连通域\n')

# 每个点属于哪个域
pt_lbl = lbl[tuple(idx.T)]
rows = []
for c in range(1, n + 1):
    m = pt_lbl == c
    cnt = int(m.sum())
    if cnt < 300:
        continue
    p = pts[m]
    rows.append((cnt, p.min(axis=0), p.max(axis=0), p.mean(axis=0)))
rows.sort(key=lambda x: -x[0])

print(f'点数 >= 300 的团（按点数排序，共 {len(rows)} 个）:')
for cnt, mn, mx, mean in rows[:15]:
    size = mx - mn
    print(f'  {cnt:>7} 点 | 中心 ({mean[0]:6.3f},{mean[1]:6.3f},{mean[2]:6.3f}) '
          f'| 尺寸 {size[0]:.2f}x{size[1]:.2f}x{size[2]:.2f}m '
          f'| 范围 x[{mn[0]:.2f},{mx[0]:.2f}] y[{mn[1]:.2f},{mx[1]:.2f}] z[{mn[2]:.2f},{mx[2]:.2f}]')
