"""Terrain-following integration against real terrain safety and fixed-wing kinematic model."""
from __future__ import annotations

from dataclasses import replace
import math
import unittest

import numpy as np

from planner.config import DEFAULT_CONFIG
from planner.fixed_wing_envelope import FixedWingKinematicEnvelope, MAX_CLIMB_RATE_MPS, MAX_DESCENT_RATE_MPS
from planner.physical import (
    FIXED_PLANAR_SPEED_MPS, PhysicalPose,
    build_level_turn_trajectory, build_straight_level_trajectory,
    build_straight_vertical_trajectory,
)
from planner.pose_search import GoalPose, GoalTolerance
from planner.terrain_following import optimize_terrain_following_altitudes, plan_terrain_following
from planner.trajectory_safety import evaluate_physical_trajectory_safety
from tests.test_current_contracts import NODATA, terrain_from_array


class TerrainFollowingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.envelope = FixedWingKinematicEnvelope()
        cls.config = replace(DEFAULT_CONFIG, min_agl_m=100.0,
                             primitive_sample_spacing_m=10.0, lateral_buffer_m=0.0)

    @staticmethod
    def terrain(elevation=1000.0, width=1000, height=120):
        return terrain_from_array(np.full((height, width), elevation, dtype=np.float32), 10.0)

    @staticmethod
    def straight(length=8000.0, altitude=1300.0):
        return build_straight_level_trajectory(
            PhysicalPose(500.0, 600.5, altitude, 90.0), length, 10.0,
        ).trajectory

    def optimize(self, source, terrain, **kwargs):
        return optimize_terrain_following_altitudes(
            source, terrain, config=kwargs.pop("config", self.config),
            envelope=self.envelope, **kwargs,
        )

    def assert_safe_profile(self, source, result, terrain, config=None):
        """Check returned geometry, continuous DEM safety, and authoritative fixed-wing rates."""
        config = self.config if config is None else config
        self.assertTrue(result.success, result.status)
        self.assertEqual(result.trajectories[0].start_pose, source[0].start_pose)
        self.assertEqual(result.trajectories[-1].end_pose, source[-1].end_pose)
        self.assertEqual(len(source), len(result.trajectories))
        for original, trajectory in zip(source, result.trajectories):
            self.assertEqual(original.horizontal_arc_length_m, trajectory.horizontal_arc_length_m)
            self.assertEqual(
                [(s.x_m, s.y_m, s.heading_deg, s.horizontal_distance_along_path_m)
                 for s in original.samples],
                [(s.x_m, s.y_m, s.heading_deg, s.horizontal_distance_along_path_m)
                 for s in trajectory.samples],
            )
            safety = evaluate_physical_trajectory_safety(
                trajectory, terrain, config.min_agl_m, config.primitive_sample_spacing_m,
                lateral_buffer_m=config.lateral_buffer_m, planning_bounds=terrain.roi.bounds,
            )
            self.assertTrue(safety.is_safe, safety.failure_reason)
            self.assertGreaterEqual(safety.min_agl_m, config.min_agl_m)
            for a, b in zip(trajectory.samples, trajectory.samples[1:]):
                dz = b.z_msl_m - a.z_msl_m
                if abs(dz) < 1e-9:
                    continue
                dt = ((b.horizontal_distance_along_path_m - a.horizontal_distance_along_path_m)
                      / FIXED_PLANAR_SPEED_MPS)
                mode = "CLIMB" if dz > 0 else "DESCENT"
                cap = MAX_CLIMB_RATE_MPS if dz > 0 else MAX_DESCENT_RATE_MPS
                limit = self.envelope.straight_vertical(a.z_msl_m, mode)
                self.assertEqual(limit.availability, "AVAILABLE")
                self.assertLessEqual(abs(dz / dt), cap + 1e-7)
        for a, b in zip(result.trajectories, result.trajectories[1:]):
            self.assertEqual(a.end_pose, b.start_pose)

    @staticmethod
    def vertical_rates(trajectory):
        return [
            (b.z_msl_m - a.z_msl_m) * FIXED_PLANAR_SPEED_MPS
            / (b.horizontal_distance_along_path_m - a.horizontal_distance_along_path_m)
            for a, b in zip(trajectory.samples, trajectory.samples[1:])
        ]

    def test_flat_high_endpoints_descend_to_target_then_climb_with_envelope_limit(self):
        terrain, source = self.terrain(), (self.straight(),)
        for target in (100.0, 120.0):
            with self.subTest(target=target):
                result = self.optimize(source, terrain, target_agl_m=target)
                self.assert_safe_profile(source, result, terrain)
                self.assertAlmostEqual(result.minimum_agl_m, target)
                rates = self.vertical_rates(result.trajectories[0])
                self.assertAlmostEqual(max(rates), MAX_CLIMB_RATE_MPS)
                self.assertAlmostEqual(min(rates), -MAX_DESCENT_RATE_MPS)

    def test_high_altitude_descent_respects_the_5_0_cap(self):
        terrain, source = self.terrain(3000.0), (self.straight(altitude=3300.0),)
        result = self.optimize(source, terrain)
        self.assert_safe_profile(source, result, terrain)
        rates = self.vertical_rates(result.trajectories[0])
        self.assertAlmostEqual(min(rates), -MAX_DESCENT_RATE_MPS)
        self.assertAlmostEqual(max(rates), MAX_CLIMB_RATE_MPS)
        self.assertAlmostEqual(result.minimum_agl_m, 120.0)

    def test_ridge_triggers_climb_while_ground_is_still_low(self):
        terrain = self.terrain(width=1400)
        terrain.roi.elevation[:, 650:670] = 1100.0
        source = (self.straight(length=12000.0),)
        result = self.optimize(source, terrain)
        self.assert_safe_profile(source, result, terrain)
        samples = result.trajectories[0].samples
        approach = [s for s in samples if 5800.0 <= s.x_m <= 6400.0]
        self.assertGreater(approach[0].z_msl_m, 1120.0)
        self.assertGreater(approach[-1].z_msl_m, approach[0].z_msl_m)
        self.assertTrue(all(s.z_msl_m >= 1220.0 for s in samples if 6500 <= s.x_m <= 6700))
        self.assertAlmostEqual(min(s.z_msl_m for s in samples), 1120.0)

    def test_short_terrain_dip_is_bridged_by_limited_vertical_rates(self):
        terrain = self.terrain(1150.0)
        terrain.roi.elevation[:, 300:320] = 1000.0
        source = (self.straight(altitude=1270.0),)
        result = self.optimize(source, terrain)
        self.assert_safe_profile(source, result, terrain)
        minimum = min(s.z_msl_m for s in result.trajectories[0].samples)
        self.assertLess(minimum, 1270.0)
        self.assertGreater(minimum, 1240.0)

    def test_lateral_buffer_raises_altitude_for_adjacent_hazard(self):
        terrain = self.terrain()
        terrain.roi.elevation[56, 400:402] = 1200.0  # ~30 m north of the track.
        source = (self.straight(),)
        centerline = self.optimize(source, terrain)
        buffered_config = replace(self.config, lateral_buffer_m=40.0)
        buffered = self.optimize(source, terrain, config=buffered_config)
        self.assert_safe_profile(source, centerline, terrain)
        self.assert_safe_profile(source, buffered, terrain, buffered_config)
        center_z = min(s.z_msl_m for s in centerline.trajectories[0].samples if 4000 <= s.x_m <= 4020)
        buffer_z = min(s.z_msl_m for s in buffered.trajectories[0].samples if 4000 <= s.x_m <= 4020)
        self.assertAlmostEqual(center_z, 1120.0)
        self.assertGreaterEqual(buffer_z, 1320.0)

    def test_nodata_on_track_fails_closed(self):
        terrain = self.terrain()
        terrain.roi.elevation[59, 400] = NODATA
        result = self.optimize((self.straight(),), terrain)
        self.assertFalse(result.success)
        self.assertEqual(result.status, "INVALID_TERRAIN_COVERAGE")
        self.assertEqual(result.trajectories, ())

    def test_basic_turn_remains_level_and_preserves_exact_xy_arc(self):
        terrain = self.terrain(height=240)
        first = self.straight(length=3000.0)
        limit = self.envelope.level_turn(first.end_pose.z_msl_m, "LEFT")
        turn = build_level_turn_trajectory(first.end_pose, limit.level_turn, 15.0, 10.0).trajectory
        last = build_straight_level_trajectory(turn.end_pose, 4000.0, 10.0).trajectory
        source = (first, turn, last)
        result = self.optimize(source, terrain)
        self.assert_safe_profile(source, result, terrain)
        optimized_turn = result.trajectories[1]
        self.assertEqual(len({s.z_msl_m for s in optimized_turn.samples}), 1)
        self.assertLess(optimized_turn.start_pose.z_msl_m, turn.start_pose.z_msl_m)

    def test_invalid_target_and_missing_hard_floor_are_rejected(self):
        terrain, source = self.terrain(), (self.straight(length=100.0),)
        for target in (99.0, math.nan, math.inf):
            with self.subTest(target=target), self.assertRaises(ValueError):
                self.optimize(source, terrain, target_agl_m=target)
        with self.assertRaises(ValueError):
            self.optimize(source, terrain, config=replace(self.config, min_agl_m=None))

    def test_wrapper_keeps_search_and_profile_results_separate(self):
        terrain = self.terrain(width=200)
        start, goal = PhysicalPose(500.0, 600.5, 1300.0, 90.0), GoalPose(860.0, 600.5, 1300.0)
        result = plan_terrain_following(
            start, goal, terrain, goal_tolerance=GoalTolerance(1.0, 1.0),
            config=self.config, max_expansions=200, envelope=self.envelope,
        )
        self.assertTrue(result.search_result.success, result.search_result.status)
        self.assertIsNotNone(result.profile_result)
        self.assertTrue(result.success, result.profile_result.status)
        self.assert_safe_profile(result.search_result.trajectories, result.profile_result, terrain)
        self.assertEqual(result.trajectories, result.profile_result.trajectories)
        self.assertLess(min(s.z_msl_m for t in result.trajectories for s in t.samples), 1300.0)
        limited = plan_terrain_following(
            start, goal, terrain, goal_tolerance=GoalTolerance(1.0, 1.0),
            config=self.config, max_expansions=0, envelope=self.envelope,
        )
        self.assertFalse(limited.success)
        self.assertEqual(limited.search_result.status, "search_limit_reached")
        self.assertIsNone(limited.profile_result)
        self.assertEqual(limited.trajectories, ())


if __name__ == "__main__":
    unittest.main()
