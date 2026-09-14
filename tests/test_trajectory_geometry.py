"""Geometry-only contracts for continuous straight and level-turn paths."""

from __future__ import annotations

import math
import unittest

from planner.physical import (
    FIXED_PLANAR_SPEED_MPS,
    LevelTurnKinematics,
    PhysicalPose,
    build_level_turn_trajectory,
    build_straight_level_trajectory,
    level_turn_center,
)


RADIUS_M = 100.0
TURN_RATE_DEG_S = FIXED_PLANAR_SPEED_MPS / RADIUS_M * 180.0 / math.pi
TOL = 1e-9


def assert_close(test: unittest.TestCase, actual: float, expected: float, tolerance: float = TOL) -> None:
    test.assertTrue(math.isclose(actual, expected, rel_tol=0.0, abs_tol=tolerance), (actual, expected))


class ContinuousTrajectoryGeometryTests(unittest.TestCase):
    def setUp(self) -> None:
        self.start = PhysicalPose(1000.25, 2000.5, 1500.0, 0.0)
        self.left = LevelTurnKinematics("LEFT", RADIUS_M, -TURN_RATE_DEG_S)
        self.right = LevelTurnKinematics("RIGHT", RADIUS_M, TURN_RATE_DEG_S)

    def assert_turn_contract(self, result, kinematics, expected_arc_length: float) -> None:
        trajectory = result.trajectory
        assert_close(self, trajectory.horizontal_arc_length_m, expected_arc_length)
        assert_close(self, trajectory.duration_s, expected_arc_length / FIXED_PLANAR_SPEED_MPS)
        expected_rate_rad_s = abs(kinematics.signed_turn_rate_deg_s) * math.pi / 180.0
        assert_close(self, expected_rate_rad_s, FIXED_PLANAR_SPEED_MPS / kinematics.radius_m)
        assert_close(self, expected_arc_length / kinematics.radius_m, expected_rate_rad_s * trajectory.duration_s)
        distances = [sample.horizontal_distance_along_path_m for sample in trajectory.samples]
        self.assertEqual(distances, sorted(distances))
        self.assertEqual(trajectory.samples[0].pose, self.start)
        self.assertEqual(trajectory.samples[-1].pose, trajectory.end_pose)

        center_x, center_y = level_turn_center(self.start, kinematics)
        radial_errors = [
            abs(math.hypot(sample.x_m - center_x, sample.y_m - center_y) - kinematics.radius_m)
            for sample in trajectory.samples
        ]
        self.assertLessEqual(max(radial_errors), TOL)

    def test_straight_north_east_and_arbitrary_heading(self) -> None:
        north = build_straight_level_trajectory(self.start, 80.0, 30.0).trajectory
        assert_close(self, north.end_pose.x_m, self.start.x_m)
        assert_close(self, north.end_pose.y_m, self.start.y_m + 80.0)
        self.assertEqual(north.end_pose.heading_deg, 0.0)
        assert_close(self, north.duration_s, 2.0)

        east_start = PhysicalPose(1000.25, 2000.5, 1500.0, 90.0)
        east = build_straight_level_trajectory(east_start, 80.0, 30.0).trajectory
        assert_close(self, east.end_pose.x_m, east_start.x_m + 80.0)
        assert_close(self, east.end_pose.y_m, east_start.y_m)

        arbitrary_start = PhysicalPose(0.0, 0.0, 1500.0, 30.0)
        arbitrary = build_straight_level_trajectory(arbitrary_start, 20.0, 7.0).trajectory
        assert_close(self, arbitrary.end_pose.x_m, 10.0)
        assert_close(self, arbitrary.end_pose.y_m, 10.0 * math.sqrt(3.0))
        self.assertTrue(all(sample.z_msl_m == arbitrary_start.z_msl_m for sample in arbitrary.samples))

    def test_left_and_right_quarter_turns_are_mirrored(self) -> None:
        arc_length = math.pi * RADIUS_M / 2.0
        right = build_level_turn_trajectory(self.start, self.right, 90.0, 30.0)
        left = build_level_turn_trajectory(self.start, self.left, 90.0, 30.0)
        self.assert_turn_contract(right, self.right, arc_length)
        self.assert_turn_contract(left, self.left, arc_length)

        assert_close(self, right.end_pose.x_m, self.start.x_m + RADIUS_M)
        assert_close(self, right.end_pose.y_m, self.start.y_m + RADIUS_M)
        self.assertEqual(right.end_pose.heading_deg, 90.0)
        assert_close(self, left.end_pose.x_m, self.start.x_m - RADIUS_M)
        assert_close(self, left.end_pose.y_m, self.start.y_m + RADIUS_M)
        self.assertEqual(left.end_pose.heading_deg, 270.0)

    def test_half_turns_and_full_circle(self) -> None:
        half_length = math.pi * RADIUS_M
        right_half = build_level_turn_trajectory(self.start, self.right, 180.0, 35.0)
        left_half = build_level_turn_trajectory(self.start, self.left, 180.0, 35.0)
        self.assert_turn_contract(right_half, self.right, half_length)
        self.assert_turn_contract(left_half, self.left, half_length)
        assert_close(self, right_half.end_pose.x_m, self.start.x_m + 2.0 * RADIUS_M)
        assert_close(self, left_half.end_pose.x_m, self.start.x_m - 2.0 * RADIUS_M)
        assert_close(self, right_half.end_pose.y_m, self.start.y_m)
        assert_close(self, left_half.end_pose.y_m, self.start.y_m)
        self.assertEqual(right_half.end_pose.heading_deg, 180.0)
        self.assertEqual(left_half.end_pose.heading_deg, 180.0)

        full = build_level_turn_trajectory(self.start, self.right, 360.0, 25.0)
        self.assert_turn_contract(full, self.right, 2.0 * math.pi * RADIUS_M)
        assert_close(self, full.end_pose.x_m, self.start.x_m)
        assert_close(self, full.end_pose.y_m, self.start.y_m)
        self.assertEqual(full.end_pose.heading_deg, 0.0)

    def test_turn_kinematics_rejects_inconsistent_or_wrong_sign_pairs(self) -> None:
        with self.assertRaises(ValueError):
            LevelTurnKinematics("RIGHT", RADIUS_M, TURN_RATE_DEG_S * 0.9)
        with self.assertRaises(ValueError):
            LevelTurnKinematics("LEFT", RADIUS_M, TURN_RATE_DEG_S)
