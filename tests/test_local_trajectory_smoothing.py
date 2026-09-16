import math
import unittest
import numpy as np

from planner.config import DEFAULT_CONFIG
from planner.physical import PhysicalPose, PhysicalTrajectory, TrajectorySample
from planner.local_trajectory_smoothing import (
    detect_curvature_junctions,
    apply_corridor_safe_local_bspline_smoothing,
)
from planner.roi import load_roi
from planner.terrain import TerrainQuery


class TestLocalTrajectorySmoothing(unittest.TestCase):
    def test_junction_detection(self):
        # Create a straight primitive followed by a right turn primitive
        s1 = [TrajectorySample(0.0, float(y), 200.0, 0.0, float(y)) for y in range(0, 61, 5)]
        t1 = PhysicalTrajectory(s1[0].pose, s1[-1].pose, 60.0, tuple(s1))
        
        # 15 degree right turn, R = 350m
        R = 350.0
        arc_len = 91.6
        s2 = []
        for i in range(13):
            s_val = (i / 12.0) * arc_len
            ang = s_val / R
            x = R - R * math.cos(ang)
            y = 60.0 + R * math.sin(ang)
            h = math.degrees(ang)
            s2.append(TrajectorySample(x, y, 200.0, h, s_val))
        t2 = PhysicalTrajectory(s2[0].pose, s2[-1].pose, arc_len, tuple(s2))
        
        curvs, junctions = detect_curvature_junctions([t1, t2])
        self.assertEqual(len(curvs), 2)
        self.assertEqual(curvs[0], 0.0)
        self.assertGreater(curvs[1], 0.0)
        self.assertEqual(len(junctions), 1)
        self.assertEqual(junctions[0].kind, "STRAIGHT_TO_TURN")
        self.assertAlmostEqual(junctions[0].s_m, 60.0)

    def test_smoothing_preserves_corridor_safety(self):
        # Verify that smoothing stays within bounds and minimum AGL
        config = DEFAULT_CONFIG
        terrain = TerrainQuery(load_roi(config))
        # Basic straight line
        s1 = [TrajectorySample(257000.0, 4467000.0 + float(y), 500.0, 0.0, float(y)) for y in range(0, 61, 5)]
        t1 = PhysicalTrajectory(s1[0].pose, s1[-1].pose, 60.0, tuple(s1))
        res = apply_corridor_safe_local_bspline_smoothing([t1], terrain, config=config)
        self.assertTrue(res.corridor_safe)
        self.assertEqual(res.num_junctions_total, 0)


if __name__ == "__main__":
    unittest.main()
