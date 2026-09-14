"""Deterministic first-baseline regressions for pose-aware fixed-wing A*."""
from __future__ import annotations

import dataclasses
import math
import unittest
from unittest.mock import patch

import numpy as np

from planner.aircraft_profile import load_aircraft_profile
from planner.config import DEFAULT_CONFIG
from planner.physical import PhysicalPose, PhysicalTrajectory, TrajectorySample, build_straight_level_trajectory
from planner.pose_search import (
    GoalPose, GoalTolerance, navigation_bearing_deg, pose_aware_astar_search,
    pose_in_goal, search_key_for_pose,
)
from tests.test_current_contracts import PROFILE_PATH, terrain_from_array


class PoseAwareSearchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.profile = load_aircraft_profile(PROFILE_PATH)
        cls.config = dataclasses.replace(
            DEFAULT_CONFIG, min_agl_m=100.0, primitive_sample_spacing_m=10.0,
            search_xy_bin_m=60.0, search_z_bin_m=5.0, search_heading_bin_deg=15.0,
            lateral_buffer_m=0.0,
        )

    def flat(self, size=100):
        return terrain_from_array(np.full((size, size), 1000.0, dtype=np.float32), 60.0)

    def search(self, start, goal, terrain, tolerance=GoalTolerance(90.0, 10.0), limit=3000):
        return pose_aware_astar_search(start, goal, terrain, self.profile, goal_tolerance=tolerance,
                                       config=self.config, max_expansions=limit)

    def test_floor_key_bins_and_wrapped_heading_preserve_pose(self) -> None:
        pose = PhysicalPose(120.0, 60.0, 1005.0, 0.0)
        key = search_key_for_pose(pose, self.config)
        self.assertEqual((key.x_bin, key.y_bin, key.z_bin, key.heading_bin), (2, 1, 201, 0))
        self.assertEqual(pose.x_m, 120.0)
        self.assertEqual(pose.heading_deg, 0.0)
        self.assertEqual(search_key_for_pose(PhysicalPose(0, 0, 0, -1), self.config).heading_bin, 23)

    def test_goal_uses_actual_pose_not_key(self) -> None:
        goal, tolerance = GoalPose(100.0, 100.0, 1200.0), GoalTolerance(10.0, 2.0)
        self.assertTrue(pose_in_goal(PhysicalPose(109.0, 100.0, 1201.9, 22.0), goal, tolerance))
        self.assertFalse(pose_in_goal(PhysicalPose(111.0, 100.0, 1200.0, 22.0), goal, tolerance))

    def test_open_flat_terrain_can_travel_and_turn_with_continuous_path(self) -> None:
        terrain = self.flat()
        result = self.search(PhysicalPose(300, 300, 1200, 90), GoalPose(700, 300, 1200), terrain)
        self.assertTrue(result.success)
        self.assertGreater(result.continuous_path_length_m, 0.0)
        self.assertEqual(result.nodes[0].end_pose, PhysicalPose(300, 300, 1200, 90))
        self.assertTrue(all(t.max_sample_spacing_m <= 10.0 for t in result.trajectories))

    def test_turn_arc_hazard_rejected_even_when_endpoint_is_safe(self) -> None:
        terrain = terrain_from_array(np.full((100, 100), 1000.0, dtype=np.float32), 10.0)
        # Right 15 degree arc from north bends across cell (59, 51), while a
        # northward straight remains in column 50 and the arc endpoint is in
        # a different row.  Thus a safe endpoint/chord claim cannot hide it.
        terrain.roi.elevation[59, 51] = 1150.0
        result = self.search(PhysicalPose(500, 300, 1200, 0), GoalPose(500, 500, 1200), terrain,
                             tolerance=GoalTolerance(30.0, 10.0), limit=200)
        self.assertGreater(result.rejected_reason_counts.get("BELOW_MIN_AGL", 0), 0)

    def test_required_climb_is_continuous_not_a_z_bin_teleport(self) -> None:
        terrain = self.flat()
        start = PhysicalPose(300, 300, 1200, 90)
        result = self.search(start, GoalPose(360, 300, 1207.0), terrain, GoalTolerance(5.0, 1.0), 300)
        self.assertTrue(result.success)
        climbs = [node for node in result.nodes if node.incoming_primitive == "STRAIGHT_CLIMB"]
        self.assertTrue(climbs)
        self.assertNotEqual(climbs[0].end_pose.z_msl_m - start.z_msl_m, self.config.search_z_bin_m)

    def test_same_key_self_transition_is_measured_without_loop_insertion(self) -> None:
        terrain = self.flat()
        # A climb has a continuous, LUT-derived delta; it may remain inside the
        # source bucket for some source altitudes.  The accounting is stable
        # whether or not this particular profile row creates one in this run.
        result = self.search(PhysicalPose(300, 300, 1200, 90), GoalPose(1800, 300, 1200), terrain,
                             GoalTolerance(30.0, 10.0), 100)
        self.assertEqual(result.same_key_self_transition_count,
                         sum(result.same_key_self_transition_by_primitive.values()))
        self.assertLessEqual(result.same_key_self_transition_count, result.generated_neighbors)

    def test_lower_g_physical_representative_replaces_same_key_revision(self) -> None:
        terrain = self.flat()
        start = PhysicalPose(100, 100, 1200, 90)
        via = PhysicalPose(100, 160, 1200, 0)
        goal = PhysicalPose(160, 100, 1200, 90)

        def long_safe_path(first: PhysicalPose, last: PhysicalPose) -> PhysicalTrajectory:
            # A deliberately long, continuous physical detour used to exercise
            # dominance replacement, sampled at the required 10 m density.
            points = [(100 + 10 * i, 100) for i in range(7)]
            points += [(160, 100 + 10 * i) for i in range(1, 7)]
            points += [(160 - 10 * i, 160) for i in range(1, 7)]
            points += [(100, 160 - 10 * i) for i in range(1, 7)]
            points += [(100 + 10 * i, 100) for i in range(1, 7)]
            samples = tuple(TrajectorySample(x, y, 1200, 90, 10.0 * index) for index, (x, y) in enumerate(points))
            return PhysicalTrajectory(first, last, samples[-1].horizontal_distance_along_path_m, samples)

        high = long_safe_path(start, goal)
        to_via = build_straight_level_trajectory(PhysicalPose(100, 100, 1200, 0), 60, 10).trajectory
        distance = math.hypot(60, -60)
        intervals = math.ceil(distance / 10.0)
        samples = tuple(TrajectorySample(
            via.x_m + (goal.x_m - via.x_m) * index / intervals,
            via.y_m + (goal.y_m - via.y_m) * index / intervals,
            1200, 0 if index < intervals else 90, distance * index / intervals,
        ) for index in range(intervals + 1))
        to_goal = PhysicalTrajectory(via, goal, distance, samples)

        def candidates(pose, _envelope, _config):
            if pose == start:
                return (("STRAIGHT_LEVEL", high), ("LEFT_LEVEL_TURN", to_via))
            if pose == via:
                return (("STRAIGHT_LEVEL", to_goal),)
            return ()

        # The second arrival to `goal` has lower g. Its new node revision must
        # replace the long-detour representative rather than expanding stale
        # heap state for the same SearchKey.
        with patch("planner.pose_search._candidate_trajectories", candidates):
            result = self.search(start, GoalPose(goal.x_m, goal.y_m, goal.z_msl_m), terrain,
                                 GoalTolerance(0.1, 0.1), 20)
        self.assertTrue(result.success)
        self.assertEqual(result.same_key_replaced_lower_g, 1)
        self.assertEqual(result.nodes[-1].end_pose, goal)

    def test_navigation_bearing_is_north_clockwise(self) -> None:
        self.assertEqual(navigation_bearing_deg(0, 0, 0, 1), 0.0)
        self.assertEqual(navigation_bearing_deg(0, 0, 1, 0), 90.0)
        self.assertEqual(navigation_bearing_deg(0, 0, 0, -1), 180.0)
        self.assertEqual(navigation_bearing_deg(0, 0, -1, 0), 270.0)


if __name__ == "__main__":
    unittest.main()
