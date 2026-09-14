"""Contracts for the LUT-derived, ideal C172P planner envelope."""
from __future__ import annotations

import math
import unittest
from pathlib import Path

from planner.aircraft_profile import load_aircraft_profile
from planner.derived_c172p import (
    DERIVED_CONSERVATIVE,
    LUT_SAFE_SOURCE,
    DerivedC172PEnvelope,
)
from planner.physical import (
    FIXED_PLANAR_SPEED_MPS,
    PhysicalPose,
    build_helical_turn_trajectory,
    build_straight_vertical_trajectory,
)


ROOT = Path(__file__).resolve().parents[1]
PROFILE_PATH = ROOT / "jsbsim" / "results" / "c172p_aircraft_profile_planner_safe_v3.json"


class DerivedC172PEnvelopeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.envelope = DerivedC172PEnvelope(load_aircraft_profile(PROFILE_PATH))

    def test_turn_uses_worst_hand_lut_radius_and_fixed_40_rate(self) -> None:
        turn = self.envelope.level_turn(0.0, "LEFT")
        self.assertEqual(turn.availability, "AVAILABLE")
        self.assertEqual(turn.provenance, DERIVED_CONSERVATIVE)
        self.assertIsNotNone(turn.level_turn)
        assert turn.level_turn is not None
        # Right is the restrictive validated 20-degree hand at this altitude.
        self.assertGreaterEqual(turn.level_turn.radius_m, 454.8144083982382)
        rate_rad_s = abs(turn.level_turn.signed_turn_rate_deg_s) * math.pi / 180.0
        self.assertAlmostEqual(rate_rad_s, FIXED_PLANAR_SPEED_MPS / turn.level_turn.radius_m)

    def test_five_mps_is_a_ceiling_not_an_unsafe_override(self) -> None:
        climb = self.envelope.straight_vertical(1500.0, "CLIMB")
        descent = self.envelope.straight_vertical(3000.0, "DESCENT")
        self.assertEqual(climb.provenance, LUT_SAFE_SOURCE)
        self.assertLessEqual(abs(climb.signed_vertical_rate_mps), 5.0)
        self.assertLessEqual(abs(descent.signed_vertical_rate_mps), 5.0)
        self.assertGreater(climb.signed_vertical_rate_mps, 0.0)
        self.assertLess(descent.signed_vertical_rate_mps, 0.0)
        self.assertEqual(self.envelope.straight_vertical(5000.0, "CLIMB").availability, "UNAVAILABLE")

    def test_derived_combined_and_spiral_follow_source_climb_availability(self) -> None:
        spiral = self.envelope.spiral_up(1500.0, "RIGHT")
        self.assertEqual(spiral.availability, "AVAILABLE")
        self.assertEqual(spiral.provenance, DERIVED_CONSERVATIVE)
        self.assertIsNotNone(spiral.combined_turn)
        assert spiral.combined_turn is not None
        metrics = self.envelope.full_turn_metrics(spiral)
        self.assertGreater(metrics.altitude_change_m, 0.0)
        self.assertAlmostEqual(metrics.duration_s, metrics.horizontal_arc_length_m / FIXED_PLANAR_SPEED_MPS)
        self.assertGreater(metrics.orbit_radius_m, 0.0)
        self.assertEqual(self.envelope.spiral_up(5000.0, "RIGHT").availability, "UNAVAILABLE")

        loiter = self.envelope.loiter_orbit(5000.0, "LEFT")
        self.assertEqual(loiter.family, "LOITER_ORBIT")
        self.assertEqual(loiter.availability, "AVAILABLE")
        self.assertIsNotNone(loiter.level_turn)

    def test_vertical_and_helical_geometry_keep_continuous_coordinates(self) -> None:
        start = PhysicalPose(123.48, 487.19, 1500.0, 0.0)
        climb = self.envelope.straight_vertical(1500.0, "CLIMB")
        straight = build_straight_vertical_trajectory(start, 80.0, climb.signed_vertical_rate_mps, 30.0)
        self.assertEqual(straight.start_pose.x_m, 123.48)
        self.assertGreater(straight.end_pose.z_msl_m, start.z_msl_m)

        spiral = self.envelope.spiral_up(1500.0, "RIGHT")
        result = build_helical_turn_trajectory(start, spiral.combined_turn, 360.0, 30.0)
        metrics = self.envelope.full_turn_metrics(spiral)
        self.assertAlmostEqual(result.trajectory.horizontal_arc_length_m, metrics.horizontal_arc_length_m)
        self.assertAlmostEqual(result.end_pose.z_msl_m - start.z_msl_m, metrics.altitude_change_m)
        self.assertAlmostEqual(result.end_pose.x_m, start.x_m)
        self.assertAlmostEqual(result.end_pose.y_m, start.y_m)
