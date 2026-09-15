"""Small, canonical regression suite for the current planner contracts."""

from __future__ import annotations

import dataclasses
import unittest

import numpy as np
from affine import Affine
from rasterio.transform import rowcol as _rasterio_rowcol

from planner.agl import evaluate_agl
from planner.fixed_wing_envelope import FixedWingKinematicEnvelope
from planner.physical import PhysicalPose, build_straight_vertical_trajectory
from planner.pose_search import GoalPose, GoalTolerance, pose_aware_astar_search
from planner.candidate_z import CandidateZGenerator, MissionContext, TerrainMetadataStore
from planner.config import DEFAULT_CONFIG
from planner.primitives import (
    MotionPrimitive,
    evaluate_primitive,
    primitive_for_target_altitude,
)
from planner.roi import ROIData
from planner.terrain import TerrainQuery
NODATA = -9999.0


def terrain_from_array(elevation: np.ndarray, resolution_m: float = 60.0) -> TerrainQuery:
    height, width = elevation.shape
    transform = Affine(resolution_m, 0.0, 0.0, 0.0, -resolution_m, height * resolution_m)
    roi = ROIData(
        elevation=elevation.astype(np.float32),
        transform=transform,
        crs="EPSG:32636",
        width=width,
        height=height,
        bounds=(0.0, 0.0, width * resolution_m, height * resolution_m),
        resolution=(resolution_m, resolution_m),
        nodata=NODATA,
    )
    return TerrainQuery(roi)


class CurrentArchitectureContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.envelope = FixedWingKinematicEnvelope()

    def test_candidate_z_is_deterministic_and_preserves_off_lattice_events(self) -> None:
        terrain = terrain_from_array(np.full((8, 8), 1000.0, dtype=np.float32))
        mission = MissionContext(
            start_rowcol=(2, 2),
            start_z_msl=1203.25,
            goal_rowcol=(5, 5),
            goal_z_msl=1287.75,
            ceiling_msl=1303.5,
            min_agl_m=200.0,
        )
        generator = CandidateZGenerator(TerrainMetadataStore(terrain), mission)

        first = generator.generate(2, 2)
        second = generator.generate(2, 2)
        self.assertEqual(first, second)
        self.assertEqual(first, tuple(sorted(set(first))))
        self.assertIn(1203.25, first)
        self.assertTrue(generator.is_representable(2, 2, 1203.25))

    def test_candidate_z_timing_diagnostics_are_constant_memory(self) -> None:
        terrain = terrain_from_array(np.full((4, 4), 1000.0, dtype=np.float32))
        mission = MissionContext((0, 0), 1200.0, (3, 3), 1240.0, 1400.0, 200.0)
        generator = CandidateZGenerator(TerrainMetadataStore(terrain), mission)

        for _ in range(10):
            generator.generate(1, 1)

        stats = generator.stats
        self.assertEqual(stats.call_count, 10)
        self.assertGreater(stats.total_time_s, 0.0)
        self.assertGreaterEqual(stats.max_time_s, stats.min_time_s)
        self.assertEqual(stats.average_time_s, stats.total_time_s / stats.call_count)
        self.assertEqual(
            set(stats.__dict__),
            {"call_count", "total_time_s", "max_time_s", "min_time_s"},
        )

    def test_candidate_z_terrain_floor_is_conservative(self) -> None:
        terrain = terrain_from_array(np.full((4, 4), 1007.0, dtype=np.float32))
        mission = MissionContext((0, 0), 1220.0, (3, 3), 1240.0, 1400.0, 200.0)
        generator = CandidateZGenerator(TerrainMetadataStore(terrain), mission)

        self.assertEqual(generator.floor_for(1, 1), 1220.0)
        self.assertFalse(generator.is_representable(1, 1, 1219.999))
        self.assertTrue(generator.is_representable(1, 1, 1220.0))

    def test_xy_to_rowcol_matches_rasterio_affine_equivalence(self) -> None:
        """PERF-5: TerrainQuery.xy_to_rowcol's precomputed-inverse-affine
        formula must match rasterio.transform.rowcol(transform, x, y)
        (its default op=None, i.e. floor) exactly -- boundary points and
        negative/off-lattice coordinates included, where naive int()
        truncation would silently diverge from floor() semantics."""
        resolution_m = 60.0
        height, width = 12, 15
        terrain = terrain_from_array(np.zeros((height, width), dtype=np.float32), resolution_m)
        transform = terrain.roi.transform

        points = []
        # Exact pixel centers and exact pixel corners (boundaries) across
        # the raster, plus tiny +-epsilon nudges on both sides of each
        # boundary to probe floor-vs-truncation divergence for negative
        # fractional inputs.
        for row in range(height + 2):
            for col in range(width + 2):
                cx = col * resolution_m
                cy = row * resolution_m
                center_x = cx + resolution_m / 2.0
                center_y = cy + resolution_m / 2.0
                points.append((center_x, center_y))
                points.append((cx, cy))
                for eps in (1e-9, 1e-6, 1e-3):
                    points.append((cx + eps, cy + eps))
                    points.append((cx - eps, cy - eps))
        # Raster corners and clearly out-of-bounds / negative coordinates.
        minx, miny, maxx, maxy = terrain.roi.bounds
        points.extend([
            (minx, miny), (minx, maxy), (maxx, miny), (maxx, maxy),
            (-500.0, -500.0), (-1.0, -1.0), (maxx + 1000.0, maxy + 1000.0),
        ])

        mismatches = []
        for x, y in points:
            expected_row, expected_col = _rasterio_rowcol(transform, x, y)
            actual_row, actual_col = terrain.xy_to_rowcol(x, y)
            if (actual_row, actual_col) != (int(expected_row), int(expected_col)):
                mismatches.append((x, y, expected_row, expected_col, actual_row, actual_col))

        self.assertEqual(mismatches, [], f"xy_to_rowcol diverged from rasterio.transform.rowcol at: {mismatches[:5]}")

    def test_explicit_target_primitives_preserve_arbitrary_altitudes(self) -> None:
        primitive = primitive_for_target_altitude("E", 4127.0, 4163.0, DEFAULT_CONFIG)
        self.assertIsNotNone(primitive)
        assert primitive is not None
        self.assertEqual(primitive.primitive_type, "climb")
        self.assertEqual(primitive.dz_m, 36.0)
        self.assertNotEqual(primitive.dz_m % DEFAULT_CONFIG.z_step_m, 0.0)

        repeated = primitive_for_target_altitude("E", 4127.0, 4163.0, DEFAULT_CONFIG)
        self.assertEqual(primitive, repeated)

    def test_vertical_motion_uses_current_constant_envelope(self) -> None:
        for altitude in (0.0, 1200.0, 3500.0, 5000.0):
            for mode, expected in (("CLIMB", 5.0), ("DESCENT", -5.0)):
                limit = self.envelope.straight_vertical(altitude, mode)
                self.assertEqual(limit.availability, "AVAILABLE")
                self.assertEqual(limit.signed_vertical_rate_mps, expected)
                trajectory = build_straight_vertical_trajectory(
                    PhysicalPose(300, 300, altitude, 90), 800, expected, 10
                ).trajectory
                self.assertAlmostEqual(trajectory.end_pose.z_msl_m, altitude + expected * 20)

    def test_envelope_requires_no_external_aircraft_profile(self) -> None:
        self.assertEqual(self.envelope.horizontal_speed_mps, 40.0)
        self.assertAlmostEqual(self.envelope.turn_radius_m, 349.89, places=2)
        with self.assertRaises(ValueError):
            self.envelope.straight_vertical(1200, "INVALID")

    def test_agl_and_along_path_terrain_safety(self) -> None:
        cfg = dataclasses.replace(DEFAULT_CONFIG, xy_resolution_m=30.0, min_agl_m=200.0)
        flat = terrain_from_array(np.full((5, 7), 1000.0, dtype=np.float32), 30.0)
        x, y = flat.rowcol_to_xy(2, 2)
        self.assertTrue(evaluate_agl(flat, x, y, 1200.0, cfg).valid)
        self.assertFalse(evaluate_agl(flat, x, y, 1199.0, cfg).valid)

        ridge_values = np.full((5, 7), 1000.0, dtype=np.float32)
        ridge_values[2, 3] = 1250.0
        ridge = terrain_from_array(ridge_values, 30.0)
        start_x, start_y = ridge.rowcol_to_xy(2, 1)
        long_level = MotionPrimitive("E", 0, 4, 0.0, 120.0, "level")
        result = evaluate_primitive((start_x, start_y, 1400.0), long_level, ridge, cfg)
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "below_min_agl")

    def test_tiny_production_planner_smoke(self) -> None:
        cfg = dataclasses.replace(DEFAULT_CONFIG, min_agl_m=200.0)
        terrain = terrain_from_array(np.full((20, 20), 1000.0, dtype=np.float32), 60.0)
        result = pose_aware_astar_search(
            PhysicalPose(150, 300, 1203, 90), GoalPose(900, 300, 1203), terrain,
            envelope=self.envelope, goal_tolerance=GoalTolerance(60, 10),
            config=cfg, max_expansions=2000,
        )
        self.assertTrue(result.success)
        self.assertEqual(result.termination_reason, "FOUND")
        self.assertGreaterEqual(result.minimum_agl_m, 200.0)


if __name__ == "__main__":
    unittest.main()
