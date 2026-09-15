"""Tests for the generic constant-performance FixedWingKinematicEnvelope."""
from __future__ import annotations

import math
import unittest

from planner.fixed_wing_envelope import (
    HORIZONTAL_SPEED_MPS,
    BANK_ANGLE_DEG,
    MAX_CLIMB_RATE_MPS,
    MAX_DESCENT_RATE_MPS,
    TURN_RADIUS_M,
    TURN_RATE_DEG_S,
    PRIMITIVE_HORIZONTAL_DISTANCE_M,
    PRIMITIVE_DURATION_S,
    PRIMITIVE_VERTICAL_DELTA_M,
    FixedWingKinematicEnvelope,
    FixedWingKinematicModel,
)
from planner.physical import (
    FIXED_PLANAR_SPEED_MPS,
    PhysicalPose,
    build_helical_turn_trajectory,
    build_straight_vertical_trajectory,
)


class FixedWingKinematicEnvelopeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.envelope = FixedWingKinematicEnvelope()

    def test_single_source_of_truth_constants(self) -> None:
        self.assertEqual(HORIZONTAL_SPEED_MPS, 40.0)
        self.assertEqual(MAX_CLIMB_RATE_MPS, 5.0)
        self.assertEqual(MAX_DESCENT_RATE_MPS, 5.0)
        self.assertEqual(BANK_ANGLE_DEG, 25.0)
        expected_radius = (40.0 ** 2) / (9.80665 * math.tan(math.radians(25.0)))
        self.assertAlmostEqual(TURN_RADIUS_M, expected_radius, places=3)
        expected_rate = (40.0 / expected_radius) * 180.0 / math.pi
        self.assertAlmostEqual(TURN_RATE_DEG_S, expected_rate, places=3)
        self.assertEqual(PRIMITIVE_HORIZONTAL_DISTANCE_M, 60.0)
        self.assertEqual(PRIMITIVE_DURATION_S, 1.5)
        self.assertEqual(PRIMITIVE_VERTICAL_DELTA_M, 7.5)

    def test_turn_kinematics_constant_across_altitudes(self) -> None:
        for alt in (0.0, 1500.0, 3000.0, 5000.0, 8000.0):
            left = self.envelope.level_turn(alt, "LEFT")
            right = self.envelope.level_turn(alt, "RIGHT")
            self.assertEqual(left.availability, "AVAILABLE")
            self.assertEqual(right.availability, "AVAILABLE")
            assert left.level_turn is not None and right.level_turn is not None
            self.assertAlmostEqual(left.level_turn.radius_m, TURN_RADIUS_M, places=3)
            self.assertAlmostEqual(right.level_turn.radius_m, TURN_RADIUS_M, places=3)
            self.assertAlmostEqual(left.level_turn.signed_turn_rate_deg_s, -TURN_RATE_DEG_S, places=3)
            self.assertAlmostEqual(right.level_turn.signed_turn_rate_deg_s, TURN_RATE_DEG_S, places=3)

    def test_vertical_rates_constant_across_altitudes(self) -> None:
        for alt in (0.0, 1500.0, 3000.0, 5000.0, 8000.0):
            climb = self.envelope.straight_vertical(alt, "CLIMB")
            descent = self.envelope.straight_vertical(alt, "DESCENT")
            self.assertEqual(climb.availability, "AVAILABLE")
            self.assertEqual(descent.availability, "AVAILABLE")
            self.assertEqual(climb.signed_vertical_rate_mps, 5.0)
            self.assertEqual(descent.signed_vertical_rate_mps, -5.0)

    def test_combined_and_spiral_turn_kinematics(self) -> None:
        spiral = self.envelope.spiral_up(2000.0, "RIGHT")
        self.assertEqual(spiral.availability, "AVAILABLE")
        self.assertIsNotNone(spiral.combined_turn)
        assert spiral.combined_turn is not None
        metrics = self.envelope.full_turn_metrics(spiral)
        self.assertGreater(metrics.altitude_change_m, 0.0)
        self.assertAlmostEqual(metrics.duration_s, metrics.horizontal_arc_length_m / FIXED_PLANAR_SPEED_MPS)
        self.assertAlmostEqual(metrics.orbit_radius_m, TURN_RADIUS_M, places=3)


if __name__ == "__main__":
    unittest.main()
