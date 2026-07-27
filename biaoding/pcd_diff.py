#!/usr/bin/env python3
"""差分标定: 对比"空治具"和"有工件"两张点云，提取纯工件点云。

用法:
  1. 拍照位不动，拿掉工件拍一帧: cp /tmp/base.pcd /tmp/empty.pcd
  2. 放回工件拍一帧:            cp /tmp/base.pcd /tmp/with_part.pcd
  3. python3 biaoding/pcd_diff.py
  4. 输出工件的包围盒（大盒初值），并保存 /tmp/diff.pcd 供 pcd_box_tool.py 精调
"""
import numpy as np
import sys

sys.path.insert(0, '/home/nvidia/Documents/elite_robot_ws/biaoding')
from pcd_box_tool import load_pcd

RES = 0.005  # 匹配分辨率 5mm


def voxel_keys(pts):
    idx = np.floor(pts / RES).astype(np.int64)
    return idx[:, 0] * 100000000 + idx[:, 1] * 10000 + idx[:, 2] + (1 << 62)


def main():
    empty = load_pcd('/tmp/empty.pcd')
    withp = load_pcd('/tmp/with_part.pcd')
    empty = empty[~np.isnan(empty).any(axis=1)]
    withp = withp[~np.isnan(withp).any(axis=1)]
    # 只看 1.5m 以内：远处的墙/天花板两张之间抖动大，会污染差分
    empty = empty[np.linalg.norm(empty, axis=1) < 1.5]
    withp = withp[np.linalg.norm(withp, axis=1) < 1.5]
    print(f'空治具(1.5m内): {len(empty)} 点, 有工件(1.5m内): {len(withp)} 点')

    # 空治具云膨胀一层体素（吸收两次拍摄的小抖动）
    keys = set()
    base_idx = np.floor(empty / RES).astype(np.int64)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                i = base_idx + np.array([dx, dy, dz])
                keys.update(i[:, 0] * 100000000 + i[:, 1] * 10000 + i[:, 2])

    wk = voxel_keys(withp) - (1 << 62)
    diff = withp[~np.isin(wk, list(keys))]
    print(f'差分后点: {len(diff)}')
    if len(diff) < 500:
        print('点太少，检查两张 pcd 是否拍对（拍照位/治具不能动）')
        return

    # 差分结果再聚类，取最大团（去掉零散噪声和误差点）
    from scipy import ndimage
    vox = 0.03
    idx = np.floor(diff / vox).astype(int)
    idx -= idx.min(axis=0)
    grid = np.zeros(idx.max(axis=0) + 1, dtype=bool)
    grid[tuple(idx.T)] = True
    lbl, n = ndimage.label(grid, structure=np.ones((3, 3, 3)))
    sizes = ndimage.sum(np.ones_like(lbl), lbl, range(1, n + 1))
    best = int(np.argmax(sizes)) + 1
    diff = diff[lbl[tuple(idx.T)] == best]
    print(f'最大团（=工件）: {len(diff)} 点 / 共 {n} 团')

    print('\n=== 工件包围盒（大盒初值，已含 2cm 余量）===')
    mn, mx = diff.min(axis=0), diff.max(axis=0)
    print(f'x: {mn[0]-0.02:.3f} ~ {mx[0]+0.02:.3f}')
    print(f'y: {mn[1]-0.02:.3f} ~ {mx[1]+0.02:.3f}')
    print(f'z: {mn[2]-0.02:.3f} ~ {mx[2]+0.02:.3f}')
    print(f'\n原始范围 x[{mn[0]:.3f},{mx[0]:.3f}] y[{mn[1]:.3f},{mx[1]:.3f}] z[{mn[2]:.3f},{mx[2]:.3f}]')

    # 保存供 pcd_box_tool.py / pcl_viewer 精调
    hdr = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
           f'COUNT 1 1 1\nWIDTH {len(diff)}\nHEIGHT 1\nPOINTS {len(diff)}\nDATA ascii\n')
    with open('/tmp/diff.pcd', 'w') as f:
        f.write(hdr)
        np.savetxt(f, diff, fmt='%.5f')
    print('\n已保存 /tmp/diff.pcd，可用以下命令精调盒子:')
    print('  python3 biaoding/pcd_box_tool.py /tmp/diff.pcd')
    print('  pcl_viewer /tmp/diff.pcd')


if __name__ == '__main__':
    main()
