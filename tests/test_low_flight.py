"""Regression tests for low-flight planning and the vertical search bottleneck."""
from dataclasses import replace
import math
import unittest

import numpy as np

from planner.config import DEFAULT_CONFIG
from planner.fixed_wing_envelope import FixedWingKinematicEnvelope, FixedWingKinematicModel
from planner.physical import PhysicalPose, build_level_turn_trajectory, build_straight_level_trajectory
from planner.pose_search import GoalPose, GoalTolerance, _TerrainGuidance, _trajectory_edge_cost, pose_aware_astar_search
from planner.terrain_following import optimize_terrain_following_altitudes
from planner.trajectory_safety import TerrainInfluenceCache, evaluate_physical_trajectory_safety
from tests.terrain_helpers import terrain_from_array


class LowFlightTests(unittest.TestCase):
    config = replace(DEFAULT_CONFIG, min_agl_m=100.0)

    def test_long_descent_with_combined_turns_stays_within_budget(self):
        terrain = terrain_from_array(np.full((30, 170), 1000.0), 60)
        result = pose_aware_astar_search(
            PhysicalPose(300, 900, 1600, 90), GoalPose(9600, 900, 1100), terrain,
            config=self.config, goal_tolerance=GoalTolerance(90, 10), max_expansions=30000)
        self.assertTrue(result.success, result.termination_reason)
        self.assertIn("STRAIGHT_DESCENT", result.path_primitives)
        self.assertGreaterEqual(result.minimum_agl_m, 100)

    def test_invalid_start_cannot_succeed_inside_goal(self):
        terrain = terrain_from_array(np.full((10, 10), 1000.0), 60)
        result = pose_aware_astar_search(
            PhysicalPose(300, 300, 1050, 90), GoalPose(300, 300, 1050), terrain,
            config=self.config, goal_tolerance=GoalTolerance(90, 10), max_expansions=10)
        self.assertFalse(result.success)
        self.assertTrue(result.termination_reason.startswith("INVALID_START_"))

    def test_linear_cost_prefers_lower_safe_flight_without_saturation(self):
        terrain = terrain_from_array(np.full((20, 20), 1000.0), 60)
        config = replace(self.config, enable_low_altitude_cost=True,
                         low_altitude_cost_shape="linear", lambda_agl=.25, agl_cost_scale_m=1000)
        costs = []
        for altitude in (1120, 2120, 3120):
            trajectory = build_straight_level_trajectory(PhysicalPose(300, 300, altitude, 90), 60, 10).trajectory
            safety = evaluate_physical_trajectory_safety(trajectory, terrain, 100, 10, record_sample_terrain=True)
            costs.append(_trajectory_edge_cost(trajectory, safety, config))
        self.assertEqual(costs, [60, 75, 90])

    def test_backward_guidance_anticipates_ridge_and_masks_nodata(self):
        data = np.full((20, 50), 1000.0)
        data[:, 30:32] = 1200.0
        data[0, :] = -9999
        terrain = terrain_from_array(data, 60)
        goal = GoalPose(2700, 600, 1120)
        guide = _TerrainGuidance(terrain, goal, GoalTolerance(90, 10), self.config,
                                 FixedWingKinematicEnvelope(), TerrainInfluenceCache(terrain))
        self.assertGreater(guide.target(PhysicalPose(1740, 600, 1120, 90)), 1120)
        self.assertTrue(np.all(np.isinf(guide.distance[0])))

    def test_profile_respects_reduced_aircraft_rates(self):
        terrain = terrain_from_array(np.full((30, 170), 1000.0), 60)
        trajectory = build_straight_level_trajectory(PhysicalPose(300, 900, 1400, 90), 9000, 10).trajectory
        envelope = FixedWingKinematicEnvelope(FixedWingKinematicModel(max_climb_rate_mps=2, max_descent_rate_mps=3))
        result = optimize_terrain_following_altitudes((trajectory,), terrain, config=self.config, envelope=envelope)
        self.assertTrue(result.success, result.status)
        rates = [(b.z_msl_m - a.z_msl_m) * 40 / (b.horizontal_distance_along_path_m - a.horizontal_distance_along_path_m)
                 for a, b in zip(result.trajectories[0].samples, result.trajectories[0].samples[1:])]
        self.assertLessEqual(max(rates), 2 + 1e-8)
        self.assertGreaterEqual(min(rates), -3 - 1e-8)

    def test_combined_turn_can_descend_but_basic_turn_stays_level(self):
        terrain = terrain_from_array(np.full((100, 100), 1000.0), 60)
        start = PhysicalPose(1000, 1000, 1300, 90)
        envelope = FixedWingKinematicEnvelope()
        turn = build_level_turn_trajectory(start, envelope.level_turn(1300, "LEFT").level_turn, 90, 10).trajectory
        straight = build_straight_level_trajectory(turn.end_pose, 3000, 10).trajectory
        for enabled in (False, True):
            config = replace(self.config, enable_combined_turns=enabled)
            result = optimize_terrain_following_altitudes((turn, straight), terrain, config=config)
            self.assertTrue(result.success, result.status)
            dz = result.trajectories[0].end_pose.z_msl_m - start.z_msl_m
            self.assertLess(dz, 0) if enabled else self.assertEqual(dz, 0)

    def test_low_flight_below_sea_level_has_no_artificial_zero_msl_floor(self):
        terrain = terrain_from_array(np.full((30, 170), -400.0), 60)
        trajectory = build_straight_level_trajectory(PhysicalPose(300, 900, -100, 90), 9000, 10).trajectory
        result = optimize_terrain_following_altitudes((trajectory,), terrain, config=self.config, target_agl_m=100)
        self.assertTrue(result.success, result.status)
        self.assertEqual(min(p.z_msl_m for p in result.trajectories[0].samples), -300)

    def test_invalid_search_weights_fail_early(self):
        terrain = terrain_from_array(np.full((10, 10), 1000.0), 60)
        for weight in (math.nan, math.inf, .99):
            with self.subTest(weight=weight), self.assertRaises(ValueError):
                pose_aware_astar_search(PhysicalPose(120, 120, 1200, 90), GoalPose(300, 120, 1200), terrain,
                    config=replace(self.config, search_heuristic_weight=weight), goal_tolerance=GoalTolerance(90, 10))


if __name__ == "__main__":
    unittest.main()
