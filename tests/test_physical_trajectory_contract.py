"""Focused contracts for the passive continuous physical-motion data model."""

from __future__ import annotations

import unittest

import numpy as np

from planner.physical import (
    FIXED_PLANAR_SPEED_MPS,
    PhysicalPose,
    PhysicalPrimitiveResult,
    PhysicalTrajectory,
    TrajectorySample,
    angular_distance_deg,
    normalize_heading_deg,
)
from tests.test_current_contracts import terrain_from_array


class PhysicalTrajectoryContracts(unittest.TestCase):
    def test_heading_normalization_and_wrapped_distance(self) -> None:
        self.assertEqual(normalize_heading_deg(360.0), 0.0)
        self.assertEqual(normalize_heading_deg(-1.0), 359.0)
        self.assertEqual(normalize_heading_deg(721.5), 1.5)
        self.assertEqual(angular_distance_deg(359.0, 1.0), 2.0)
        self.assertEqual(angular_distance_deg(90.0, 270.0), 180.0)

    def test_continuous_pose_and_trajectory_are_grid_independent(self) -> None:
        start = PhysicalPose(123.48, 487.19, 1320.25, -1.0)
        end = PhysicalPose(163.48, 487.19, 1330.25, 359.0)
        trajectory = PhysicalTrajectory(
            start_pose=start,
            end_pose=end,
            horizontal_arc_length_m=40.0,
            samples=(
                TrajectorySample(123.48, 487.19, 1320.25, -1.0, 0.0),
                TrajectorySample(143.48, 487.19, 1325.25, 359.0, 20.0),
                TrajectorySample(163.48, 487.19, 1330.25, 359.0, 40.0),
            ),
        )
        result = PhysicalPrimitiveResult(trajectory)

        self.assertEqual(start.x_m, 123.48)
        self.assertEqual(start.y_m, 487.19)
        self.assertEqual(start.heading_deg, 359.0)
        self.assertFalse(hasattr(start, "row"))
        self.assertFalse(hasattr(start, "col"))
        self.assertEqual(trajectory.duration_s, 40.0 / FIXED_PLANAR_SPEED_MPS)
        self.assertIs(result.start_pose, start)
        self.assertIs(result.end_pose, end)

    def test_terrain_query_receives_true_float_sample_coordinates(self) -> None:
        terrain = terrain_from_array(np.full((20, 20), 1000.0, dtype=np.float32), resolution_m=30.0)
        sample = TrajectorySample(123.48, 487.19, 1320.25, 17.0, 0.0)

        terrain_result = terrain.query(sample.x_m, sample.y_m)

        self.assertTrue(terrain_result.valid)
        self.assertEqual(terrain_result.x, 123.48)
        self.assertEqual(terrain_result.y, 487.19)
        self.assertNotEqual((terrain_result.x, terrain_result.y), terrain.rowcol_to_xy(terrain_result.row, terrain_result.col))

    def test_trajectory_rejects_non_endpoint_or_non_monotonic_samples(self) -> None:
        start = PhysicalPose(1.0, 2.0, 3.0, 0.0)
        end = PhysicalPose(11.0, 2.0, 3.0, 0.0)
        with self.assertRaises(ValueError):
            PhysicalTrajectory(
                start, end, 10.0,
                (TrajectorySample(1.0, 2.0, 3.0, 0.0, 1.0), TrajectorySample(11.0, 2.0, 3.0, 0.0, 10.0)),
            )
        with self.assertRaises(ValueError):
            PhysicalTrajectory(
                start, end, 10.0,
                (
                    TrajectorySample(1.0, 2.0, 3.0, 0.0, 0.0),
                    TrajectorySample(6.0, 2.0, 3.0, 0.0, 8.0),
                    TrajectorySample(11.0, 2.0, 3.0, 0.0, 7.0),
                ),
            )
