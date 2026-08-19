#!/usr/bin/env python3
"""不依赖相机的深度板面检测回归测试."""

import importlib.util
from pathlib import Path
import unittest

import cv2
import numpy as np


SCRIPT = (Path(__file__).resolve().parents[1]
          / 'scripts' / 'depth_board_detect_node.py')
SPEC = importlib.util.spec_from_file_location(
    'depth_board_detect_node', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class DepthBoardDetectorTest(unittest.TestCase):
    def setUp(self):
        self.fx = 300.0
        self.fy = 300.0
        self.cx = 160.0
        self.cy = 120.0
        self.config = MODULE.DetectorConfig(
            board_height_min_m=0.005,
            board_height_max_m=0.080,
            board_min_pixels=1500,
            board_min_points=1000,
            board_length_min_m=0.25,
            board_length_max_m=0.40,
            board_width_min_m=0.10,
            board_width_max_m=0.22,
        )

    def test_rectangle_with_large_depth_hole(self):
        depth = np.ones((240, 320), dtype=np.float32)
        depth[95:145, 110:210] = 0.97
        depth[108:132, 145:180] = 0.0

        result = MODULE.detect_board(
            depth, (self.fx, self.fy, self.cx, self.cy), self.config)

        self.assertGreater(result['points'].shape[0], 3000)
        self.assertAlmostEqual(result['board_height'], 0.03, delta=0.002)
        self.assertAlmostEqual(result['length_m'], 0.32, delta=0.02)
        self.assertAlmostEqual(result['width_m'], 0.16, delta=0.02)
        self.assertLess(result['board_rms'], 0.001)

    def test_floor_only_is_rejected(self):
        depth = np.ones((240, 320), dtype=np.float32)
        with self.assertRaises(ValueError):
            MODULE.detect_board(
                depth, (self.fx, self.fy, self.cx, self.cy), self.config)

    def test_rotated_board_on_tilted_floor(self):
        image_h, image_w = 240, 320
        grid_v, grid_u = np.indices((image_h, image_w), dtype=np.float64)
        rays = np.stack(((grid_u - self.cx) / self.fx,
                         (grid_v - self.cy) / self.fy,
                         np.ones_like(grid_u)), axis=-1)
        floor_n = np.array([0.05, -0.03, -1.0], dtype=np.float64)
        floor_n /= np.linalg.norm(floor_n)
        floor_d = 1.0
        ray_dot_n = rays @ floor_n
        floor_depth = -floor_d / ray_dot_n

        board_height = 0.03
        board_depth = -(floor_d - board_height) / ray_dot_n
        board_mask = np.zeros((image_h, image_w), dtype=np.uint8)
        rect = ((self.cx, self.cy), (100.0, 50.0), 18.0)
        box = np.rint(cv2.boxPoints(rect)).astype(np.int32)
        cv2.fillConvexPoly(board_mask, box, 255)

        depth = floor_depth.astype(np.float32)
        depth[board_mask != 0] = board_depth[board_mask != 0]
        depth[110:132, 145:178] = 0.0

        result = MODULE.detect_board(
            depth, (self.fx, self.fy, self.cx, self.cy), self.config)

        self.assertAlmostEqual(
            result['board_height'], board_height, delta=0.002)
        self.assertAlmostEqual(result['length_m'], 0.32, delta=0.03)
        self.assertAlmostEqual(result['width_m'], 0.16, delta=0.03)
        self.assertLess(result['parallel_angle_deg'], 0.3)
        self.assertLess(result['floor_rms'], 0.001)


if __name__ == '__main__':
    unittest.main()
