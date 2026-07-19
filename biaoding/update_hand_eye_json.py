#!/usr/bin/env python3
"""
把 easy_handeye2 保存的标定结果转换成 visual_grasp_test.py 需要的 json。

用法：
  cd ~/Documents/elite_robot_ws/biaoding
  python3 update_hand_eye_json.py

每次重新标定（在 rqt 里 Save Calibration）之后都要运行一次本脚本。
"""
import json
import os
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R

CALIB_FILE = os.path.expanduser('~/.ros2/easy_handeye2/calibrations/elite_cs66_handeye.calib')
OUT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'hand_eye_result.json')


def main():
    with open(CALIB_FILE) as f:
        calib = yaml.safe_load(f)

    t = calib['transform']['translation']
    q = calib['transform']['rotation']
    R_mat = R.from_quat([q['x'], q['y'], q['z'], q['w']]).as_matrix()

    out = {
        # 相机系 -> 末端(tool0)系：P_tool = R_cam2tool @ P_cam + t_cam2tool
        "R_cam2tool": R_mat.tolist(),
        "t_cam2tool": [[t['x']], [t['y']], [t['z']]],
    }
    with open(OUT_FILE, 'w') as f:
        json.dump(out, f, indent=4)

    print(f"已从 {CALIB_FILE}")
    print(f"更新 {OUT_FILE}")
    print(f"t = [{t['x']:.6f}, {t['y']:.6f}, {t['z']:.6f}]")
    print(f"q = [{q['x']:.6f}, {q['y']:.6f}, {q['z']:.6f}, {q['w']:.6f}]")


if __name__ == '__main__':
    main()
