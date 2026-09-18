"""Continuous centerline terrain/AGL contracts for PhysicalTrajectory."""
from __future__ import annotations

import math
import unittest

import numpy as np

from planner.physical import (
    FIXED_PLANAR_SPEED_MPS,
    CombinedTurnKinematics,
    LevelTurnKinematics,
    PhysicalPose,
    build_helical_turn_trajectory,
    build_level_turn_trajectory,
    build_straight_level_trajectory,
    build_straight_vertical_trajectory,
)
from planner.trajectory_safety import evaluate_physical_trajectory_safety
from tests.terrain_helpers import NODATA, terrain_from_array


RESOLUTION_M = 10.0
MIN_AGL_M = 100.0


def flat_terrain(size: int = 60, elevation_m: float = 1000.0):
    return terrain_from_array(np.full((size, size), elevation_m, dtype=np.float32), RESOLUTION_M)


def right_turn(radius_m: float = 100.0) -> LevelTurnKinematics:
    return LevelTurnKinematics(
        "RIGHT", radius_m, FIXED_PLANAR_SPEED_MPS / radius_m * 180.0 / math.pi
    )


class ContinuousTrajectorySafetyTests(unittest.TestCase):
    def evaluate(self, trajectory, terrain, spacing: float = RESOLUTION_M, bounds=None):
        return evaluate_physical_trajectory_safety(
            trajectory, terrain, MIN_AGL_M, spacing, planning_bounds=bounds
        )

    def test_straight_safe_below_threshold_and_exact_boundary(self) -> None:
        terrain = flat_terrain()
        safe = build_straight_level_trajectory(PhysicalPose(15.25, 250.5, 1120.0, 0.0), 80.0, 10.0).trajectory
        result = self.evaluate(safe, terrain)
        self.assertTrue(result.is_safe)
        self.assertEqual(result.min_agl_m, 120.0)

        below = build_straight_level_trajectory(PhysicalPose(15.25, 250.5, 1099.0, 0.0), 80.0, 10.0).trajectory
        result = self.evaluate(below, terrain)
        self.assertFalse(result.is_safe)
        self.assertEqual(result.failure_reason, "BELOW_MIN_AGL")
        self.assertEqual(result.first_failure_sample.x_m, 15.25)
        self.assertEqual(result.first_failure_sample.y_m, 250.5)

        boundary = build_straight_level_trajectory(PhysicalPose(15.25, 250.5, 1100.0, 0.0), 80.0, 10.0).trajectory
        result = self.evaluate(boundary, terrain)
        self.assertTrue(result.is_safe)
        self.assertEqual(result.min_agl_m, MIN_AGL_M)

    def test_level_arc_safe_endpoints_but_unsafe_middle_is_rejected(self) -> None:
        terrain = flat_terrain()
        start = PhysicalPose(50.0, 400.0, 1200.0, 0.0)
        arc = build_level_turn_trajectory(start, right_turn(), 180.0, 10.0).trajectory
        self.assertTrue(self.evaluate(arc, terrain).is_safe)

        # The 90-degree arc midpoint is exactly (150, 500); endpoints retain
        # 200 m AGL while the actual circular middle crosses this high cell.
        terrain.roi.elevation[10, 15] = 1150.0
        result = self.evaluate(arc, terrain)
        self.assertFalse(result.is_safe)
        self.assertEqual(result.failure_reason, "BELOW_MIN_AGL")
        self.assertEqual((result.first_failure_cell.row, result.first_failure_cell.col), (10, 15))
        self.assertEqual(result.min_agl_m, 50.0)

    def test_straight_climb_and_descent_use_sampled_altitude(self) -> None:
        terrain_data = np.full((100, 100), 1000.0, dtype=np.float32)
        terrain_data[:40, :] = 1040.0  # northward final portion of the line
        terrain = terrain_from_array(terrain_data, RESOLUTION_M)
        climb = build_straight_vertical_trajectory(
            PhysicalPose(50.0, 250.0, 1120.0, 0.0), 400.0, 5.0, 10.0
        ).trajectory
        self.assertTrue(self.evaluate(climb, terrain).is_safe)

        descent = build_straight_vertical_trajectory(
            PhysicalPose(50.0, 50.0, 1200.0, 0.0), 900.0, -5.0, 10.0
        ).trajectory
        result = self.evaluate(descent, flat_terrain(size=100))
        self.assertFalse(result.is_safe)
        self.assertEqual(result.failure_reason, "BELOW_MIN_AGL")
        self.assertGreater(result.first_failure_sample.index, 0)

    def test_helix_safe_and_unsafe_middle(self) -> None:
        terrain = flat_terrain()
        start = PhysicalPose(50.0, 400.0, 1200.0, 0.0)
        helix = build_helical_turn_trajectory(
            start, CombinedTurnKinematics(right_turn(), 2.0), 360.0, 10.0
        ).trajectory
        self.assertTrue(self.evaluate(helix, terrain).is_safe)

        # At 90 degrees x/y are (150, 500); the helix has climbed 5 m there.
        terrain.roi.elevation[10, 15] = 1110.0
        result = self.evaluate(helix, terrain)
        self.assertFalse(result.is_safe)
        self.assertEqual(result.failure_reason, "BELOW_MIN_AGL")
        self.assertEqual((result.first_failure_cell.row, result.first_failure_cell.col), (10, 15))

    def test_roi_nodata_float_coordinates_and_sample_density(self) -> None:
        terrain = flat_terrain(size=20)
        leaves_roi = build_straight_level_trajectory(PhysicalPose(195.25, 100.5, 1200.0, 90.0), 20.0, 10.0).trajectory
        result = self.evaluate(leaves_roi, terrain)
        self.assertFalse(result.is_safe)
        self.assertEqual(result.failure_reason, "OUTSIDE_ROI")

        outside_dem = self.evaluate(
            leaves_roi, terrain, bounds=(0.0, 0.0, 300.0, 200.0)
        )
        self.assertFalse(outside_dem.is_safe)
        self.assertEqual(outside_dem.failure_reason, "OUTSIDE_DEM")

        terrain.roi.elevation[8, 3] = NODATA
        nodata = build_straight_level_trajectory(PhysicalPose(35.25, 105.5, 1200.0, 0.0), 30.0, 10.0).trajectory
        result = self.evaluate(nodata, terrain)
        self.assertFalse(result.is_safe)
        self.assertEqual(result.failure_reason, "NODATA")
        # Check the immutable float physical coordinate itself, independently
        # of row/col terrain indexing.
        self.assertEqual(nodata.samples[0].x_m, 35.25)
        self.assertEqual(nodata.samples[0].y_m, 105.5)

        sparse = build_straight_level_trajectory(PhysicalPose(15.25, 100.5, 1200.0, 0.0), 40.0, 20.0).trajectory
        result = self.evaluate(sparse, terrain, spacing=10.0)
        self.assertFalse(result.is_safe)
        self.assertEqual(result.failure_reason, "INSUFFICIENT_SAMPLE_DENSITY")
        self.assertFalse(result.sampling_sufficient)

    def test_supercover_catches_diagonal_cell_missed_by_sparse_samples(self) -> None:
        terrain = flat_terrain(size=10)
        # Start/end samples occupy (row=9,col=0) and (row=6,col=3); this
        # diagonal still intersects the high intermediate (row=8,col=1) cell.
        terrain.roi.elevation[8, 1] = 1150.0
        diagonal = build_straight_level_trajectory(
            PhysicalPose(5.0, 5.0, 1200.0, 45.0), 40.0, 40.0
        ).trajectory
        result = self.evaluate(diagonal, terrain, spacing=40.0)
        self.assertFalse(result.is_safe)
        self.assertEqual(result.failure_reason, "BELOW_MIN_AGL")
        self.assertEqual((result.first_failure_cell.row, result.first_failure_cell.col), (8, 1))
        self.assertGreater(result.covered_cell_count, len(diagonal.samples))

    def test_sagitta_expansion_catches_arc_and_helix_cells_outside_chord(self) -> None:
        terrain = flat_terrain()
        # A 90-degree right arc from (50,400) to (150,500) is sampled only
        # at endpoints. Its chord misses this cell, while the true arc passes
        # through it; 29.29 m sagitta expansion must cover it.
        terrain.roi.elevation[12, 7] = 1150.0
        start = PhysicalPose(50.0, 400.0, 1200.0, 0.0)
        arc = build_level_turn_trajectory(start, right_turn(), 90.0, 200.0).trajectory
        result = self.evaluate(arc, terrain, spacing=200.0)
        self.assertFalse(result.is_safe)
        self.assertEqual((result.first_failure_cell.row, result.first_failure_cell.col), (12, 7))
        self.assertGreater(result.max_curve_to_chord_deviation_m, 29.0)

        helix = build_helical_turn_trajectory(
            start, CombinedTurnKinematics(right_turn(), 2.0), 90.0, 200.0
        ).trajectory
        result = self.evaluate(helix, terrain, spacing=200.0)
        self.assertFalse(result.is_safe)
        self.assertEqual((result.first_failure_cell.row, result.first_failure_cell.col), (12, 7))

    def test_lateral_buffer_is_independent_of_agl_and_cacheable(self) -> None:
        terrain = flat_terrain(size=10)
        terrain.roi.elevation[4, 7] = 1150.0  # near, but not on, x=55 centerline
        path = build_straight_level_trajectory(PhysicalPose(55.0, 45.0, 1200.0, 0.0), 20.0, 10.0).trajectory
        from planner.trajectory_safety import TerrainInfluenceCache
        cache = TerrainInfluenceCache(terrain)
        self.assertTrue(self.evaluate(path, terrain, spacing=10.0).is_safe)
        buffered = evaluate_physical_trajectory_safety(path, terrain, MIN_AGL_M, 10.0,
                                                        lateral_buffer_m=15.0, terrain_influence_cache=cache)
        self.assertFalse(buffered.is_safe)
        self.assertEqual(buffered.failure_reason, "BELOW_MIN_AGL")
        self.assertEqual(buffered.lateral_buffer_m, 15.0)
        # Same AGL threshold with a distinct, static lateral model layer.
        self.assertEqual(buffered.first_failure_cell.terrain_elevation_msl, 1150.0)

        outside = flat_terrain(size=10)
        outside.roi.elevation[4, 9] = 1150.0  # 30 m cell-footprint gap: outside 15 m buffer
        outside_result = evaluate_physical_trajectory_safety(path, outside, MIN_AGL_M, 10.0, lateral_buffer_m=15.0)
        self.assertTrue(outside_result.is_safe)
