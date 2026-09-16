"""Soft terrain guidance: vertical direction, fallback and safety contracts."""
import dataclasses
import unittest

import numpy as np

from planner.config import DEFAULT_CONFIG
from planner.fixed_wing_envelope import FixedWingKinematicEnvelope, FixedWingKinematicModel
from planner.physical import PhysicalPose
from planner.pose_search import _TerrainGuidance, GoalPose, GoalTolerance, pose_aware_astar_search
from planner.trajectory_safety import TerrainInfluenceCache
from tests.test_current_contracts import terrain_from_array


class TerrainGuidanceTests(unittest.TestCase):
    def guide(self, terrain, goal, descent=5, stride=1):
        config = dataclasses.replace(DEFAULT_CONFIG, min_agl_m=100,
            enable_terrain_guidance=True, terrain_guidance_stride=stride)
        envelope = FixedWingKinematicEnvelope(FixedWingKinematicModel(max_descent_rate_mps=descent))
        return _TerrainGuidance(terrain, goal, GoalTolerance(10, 10), config,
                                envelope, TerrainInfluenceCache(terrain))

    def test_slow_descent_increases_reverse_cost_from_high_terrain(self):
        terrain = terrain_from_array(np.array([[500, 300, 100]], dtype=np.float32))
        goal = GoalPose(150, 30, 210)
        pose = PhysicalPose(30, 30, 610, 90)
        fast = self.guide(terrain, goal, descent=5)
        slow = self.guide(terrain, goal, descent=2)
        self.assertGreater(slow.value(slow.distance, pose, 0), fast.value(fast.distance, pose, 0))
        self.assertGreater(slow.estimate(pose, 0), fast.estimate(pose, 0))

    def test_excess_altitude_adds_descent_distance(self):
        terrain = terrain_from_array(np.zeros((3, 5)))
        goal = GoalPose(270, 90, 100)
        guide = self.guide(terrain, goal)
        low = PhysicalPose(30, 90, 100, 90)
        high = PhysicalPose(30, 90, 500, 90)
        self.assertGreater(guide.estimate(high, 0), guide.estimate(low, 0))

    def test_missing_coarse_connection_uses_fallback_not_rejection(self):
        terrain = terrain_from_array(np.array([[0, -9999, 0]], dtype=np.float32))
        guide = self.guide(terrain, GoalPose(150, 30, 100))
        self.assertEqual(guide.estimate(PhysicalPose(30, 30, 50, 90), 123), 123)
        self.assertEqual(guide.estimate(PhysicalPose(-1, 30, 50, 90), 123), 123)

    def test_partial_stride_cell_at_map_edge_is_supported(self):
        terrain = terrain_from_array(np.zeros((4, 5)))
        guide = self.guide(terrain, GoalPose(270, 30, 100), stride=3)
        self.assertTrue(np.isfinite(guide.estimate(PhysicalPose(270, 30, 100, 90), 0)))

    def test_ordering_works_without_changing_geometric_cost_or_safety(self):
        terrain = terrain_from_array(np.zeros((20, 20)))
        config = dataclasses.replace(DEFAULT_CONFIG, min_agl_m=100,
                                    enable_terrain_guidance=True, enable_low_altitude_cost=False)
        goal = GoalPose(900, 300, 150)
        result = pose_aware_astar_search(PhysicalPose(300, 300, 150, 90), goal, terrain,
            goal_tolerance=GoalTolerance(30, 10), config=config, max_expansions=1000)
        self.assertTrue(result.success)
        self.assertAlmostEqual(result.total_cost, result.continuous_path_length_m)
        self.assertGreaterEqual(result.minimum_agl_m, 100)
        invalid = pose_aware_astar_search(PhysicalPose(300, 300, 99, 90), goal, terrain,
            goal_tolerance=GoalTolerance(30, 10), config=config, max_expansions=1000)
        self.assertFalse(invalid.success)
        self.assertTrue(invalid.termination_reason.startswith("INVALID_START"))

    def test_invalid_stride_rejected(self):
        terrain = terrain_from_array(np.zeros((4, 5)))
        for stride in (0, -1, 1.5, True):
            with self.subTest(stride=stride), self.assertRaises(ValueError):
                self.guide(terrain, GoalPose(270, 30, 100), stride=stride)

    def test_feedback_penalties_reroute_around_bottleneck(self):
        elev = np.full((100, 200), 1000.0, dtype=np.float32)
        terrain = terrain_from_array(elev, 10.0)
        envelope = FixedWingKinematicEnvelope()
        config = dataclasses.replace(DEFAULT_CONFIG, min_agl_m=100.0, primitive_sample_spacing_m=10.0,
                                     lateral_buffer_m=0.0, enable_combined_turns=False,
                                     enable_low_altitude_cost=False, enable_terrain_guidance=True,
                                     search_heuristic_weight=1.05)
        start = PhysicalPose(200.0, 500.0, 1100.0, 90.0)
        goal = GoalPose(1500.0, 500.0, 1100.0)
        search_plain = pose_aware_astar_search(start, goal, terrain, goal_tolerance=GoalTolerance(30.0, 20.0),
                                              config=config, envelope=envelope, max_expansions=2000)
        min_dist_plain = min(np.hypot(s.x_m - 800.0, s.y_m - 500.0) for t in search_plain.trajectories for s in t.samples)
        self.assertAlmostEqual(min_dist_plain, 0.0, delta=1.0)

        search_penalized = pose_aware_astar_search(start, goal, terrain, goal_tolerance=GoalTolerance(30.0, 20.0),
                                                  config=config, envelope=envelope, max_expansions=2000,
                                                  feedback_penalties=[(800.0, 500.0, 150.0)])
        min_dist_penalized = min(np.hypot(s.x_m - 800.0, s.y_m - 500.0) for t in search_penalized.trajectories for s in t.samples)
        self.assertGreaterEqual(min_dist_penalized, 150.0)
