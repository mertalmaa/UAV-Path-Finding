"""Contracts for the production vertical-reachability heuristic."""
from __future__ import annotations

import math
import unittest

from planner.pose_search import vertical_reachability_heuristic
from planner.physical import PhysicalPose
from planner.pose_search import GoalPose, GoalTolerance, _heuristic


class VerticalReachabilityHeuristicTests(unittest.TestCase):
    def setUp(self) -> None:
        self.goal = GoalPose(1000.0, 1000.0, 2000.0)
        self.tolerance = GoalTolerance(90.0, 10.0)

    def test_dominates_current_euclidean_goal_region_bound(self) -> None:
        for pose in (
            PhysicalPose(0.0, 0.0, 1500.0, 90.0),
            PhysicalPose(950.0, 1000.0, 1995.0, 90.0),
            PhysicalPose(910.0, 910.0, 2010.0, 90.0),
        ):
            self.assertGreaterEqual(vertical_reachability_heuristic(pose, self.goal, self.tolerance),
                                    _heuristic(pose, self.goal, self.tolerance))

    def test_vertical_requirement_controls_when_lateral_distance_is_small(self) -> None:
        pose = PhysicalPose(1000.0, 1000.0, 1500.0, 0.0)
        dz = 490.0  # 500 m exact difference less the 10 m acceptance tolerance.
        expected = math.hypot(dz * 40.0 / 5.0, dz)
        self.assertAlmostEqual(vertical_reachability_heuristic(pose, self.goal, self.tolerance), expected)

    def test_zero_inside_goal_region(self) -> None:
        self.assertEqual(vertical_reachability_heuristic(
            PhysicalPose(1080.0, 1000.0, 2009.0, 0.0), self.goal, self.tolerance,
        ), 0.0)


if __name__ == "__main__":
    unittest.main()
