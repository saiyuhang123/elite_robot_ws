#!/usr/bin/env python3
"""查看 /tmp/base.pcd（cs66_base_link 系点云）的数值范围，用于：
1. 验证手眼变换（点云坐标 vs 卷尺实测）
2. 标定裁剪盒（直接读 min/max 填进 pclCalcTransform.hpp）

用法:
  python3 biaoding/pcd_box_tool.py                          # 整体范围（默认读 /tmp/base.pcd）
  python3 biaoding/pcd_box_tool.py -0.6 -1.4 -0.3 0.4 -0.6 0.4      # 盒内子云范围
  python3 biaoding/pcd_box_tool.py /path/to/xx.pcd [6个盒参数]      # 读指定 pcd
"""
import sys
import numpy as np


def load_pcd(path):
    try:
        import open3d as o3d
        pc = o3d.io.read_point_cloud(path)
        return np.asarray(pc.points)
    except ImportError:
        pass
    # 无 open3d 时手工解析 pcd（支持 ascii 和 binary）
    with open(path, 'rb') as f:
        data = f.read()
    header_end = data.index(b'DATA')
    header = data[:header_end].decode('ascii', errors='ignore')
    body = data[header_end:].split(b'\n', 1)[1]
    fields = None
    npoints = None
    for line in header.splitlines():
        if line.startswith('FIELDS'):
            fields = line.split()[1:]
        if line.startswith('POINTS'):
            npoints = int(line.split()[1])
    xi, yi, zi = fields.index('x'), fields.index('y'), fields.index('z')
    if header.split('DATA')[1].strip().startswith('binary'):
        arr = np.frombuffer(body, dtype=np.float32, count=npoints * len(fields))
        arr = arr.reshape(npoints, len(fields))
        return arr[:, [xi, yi, zi]]
    else:
        rows = [l.split() for l in body.decode().splitlines() if l.strip()]
        arr = np.array(rows, dtype=np.float32)
        return arr[:, [xi, yi, zi]]


def hist_mode(pts, voxel=0.1, max_range=2.0, topn=12):
    """把 max_range 范围内的空间划成 voxel 大小的格子，打印点数最多的格子中心。"""
    r = np.linalg.norm(pts, axis=1)
    near = pts[r < max_range]
    print(f'{max_range}m 内点数: {len(near)}（远处的墙/天花板已忽略）')
    if len(near) == 0:
        return
    idx = np.floor(near / voxel).astype(int)
    uniq, counts = np.unique(idx, axis=0, return_counts=True)
    order = np.argsort(-counts)[:topn]
    print(f'\n点数最多的 {topn} 个 {voxel}m 格子（中心坐标 / 点数）:')
    for k in order:
        c = (uniq[k] + 0.5) * voxel
        print(f'  ({c[0]:6.2f}, {c[1]:6.2f}, {c[2]:6.2f})  {counts[k]} 点')


def main():
    args = sys.argv[1:]
    pcd_path = '/tmp/base.pcd'
    if args and args[0].endswith('.pcd'):
        pcd_path = args.pop(0)
    pts = load_pcd(pcd_path)
    pts = pts[~np.isnan(pts).any(axis=1)]
    print(f'{pcd_path} 总点数: {len(pts)}')

    if args and args[0] == '--hist':
        hist_mode(pts)
        return

    if len(args) == 6:
        xmin, ymin, zmin, xmax, ymax, zmax = map(float, args)
        m = (pts[:, 0] >= xmin) & (pts[:, 0] <= xmax) \
          & (pts[:, 1] >= ymin) & (pts[:, 1] <= ymax) \
          & (pts[:, 2] >= zmin) & (pts[:, 2] <= zmax)
        pts = pts[m]
        print(f'盒内点数: {len(pts)}')
        if len(pts) == 0:
            print('盒子是空的！')
            return

    print('min:', pts.min(axis=0).round(4))
    print('max:', pts.max(axis=0).round(4))
    print('质心:', pts.mean(axis=0).round(4))


if __name__ == '__main__':
    main()
