"""Authoritative Generic Constant-Performance Fixed-Wing Kinematic Model & Envelope.

Single source of truth for aircraft flight parameters across all planner components:
- Horizontal Kinematic Speed: 40.0 m/s
- Climb rate: +5.0 m/s (at all altitudes)
- Descent rate: -5.0 m/s (at all altitudes)
- Bank angle: 25.0 deg (turn radius ≈ 349.89 m, turn rate ≈ 6.55 deg/s)
- Zero wind convention
- Altitude dependence: NONE (same limits at all altitudes)
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from planner.physical import (
    FIXED_PLANAR_SPEED_MPS,
    CombinedTurnKinematics,
    LevelTurnKinematics,
)

# =============================================================================
# SINGLE SOURCE OF TRUTH — AUTHORITATIVE CONSTANTS
# =============================================================================
HORIZONTAL_SPEED_MPS: float = 40.0
MAX_CLIMB_RATE_MPS: float = 5.0
MAX_DESCENT_RATE_MPS: float = 5.0
BANK_ANGLE_DEG: float = 25.0
GRAVITY_MPS2: float = 9.80665

# Authoritative Derived Kinematics:
TURN_RADIUS_M: float = (HORIZONTAL_SPEED_MPS ** 2) / (GRAVITY_MPS2 * math.tan(math.radians(BANK_ANGLE_DEG)))  # 349.8864 m
TURN_RATE_DEG_S: float = (HORIZONTAL_SPEED_MPS / TURN_RADIUS_M) * 180.0 / math.pi  # 6.5505 deg/s

# 60m Primitive Step Constants:
PRIMITIVE_HORIZONTAL_DISTANCE_M: float = 60.0
PRIMITIVE_DURATION_S: float = PRIMITIVE_HORIZONTAL_DISTANCE_M / HORIZONTAL_SPEED_MPS  # 1.5 s
PRIMITIVE_VERTICAL_DELTA_M: float = MAX_CLIMB_RATE_MPS * PRIMITIVE_DURATION_S  # 7.5 m
PRIMITIVE_TURN_HEADING_DELTA_DEG: float = 15.0  # Standard search discrete heading delta


@dataclass(frozen=True)
class FixedWingKinematicModel:
    """Declared parameter set for generic constant-performance fixed-wing aircraft."""

    horizontal_speed_mps: float = HORIZONTAL_SPEED_MPS
    max_climb_rate_mps: float = MAX_CLIMB_RATE_MPS
    max_descent_rate_mps: float = MAX_DESCENT_RATE_MPS
    bank_angle_deg: float = BANK_ANGLE_DEG
    combined_radius_factor: float = 1.00
    combined_vertical_rate_factor: float = 1.00

    @property
    def turn_radius_m(self) -> float:
        return (self.horizontal_speed_mps ** 2) / (GRAVITY_MPS2 * math.tan(math.radians(self.bank_angle_deg)))

    @property
    def turn_rate_deg_s(self) -> float:
        return (self.horizontal_speed_mps / self.turn_radius_m) * 180.0 / math.pi


@dataclass(frozen=True)
class DerivedManeuverLimit:
    """Kinematic maneuver limit query result."""

    family: str
    altitude_m: float
    availability: str = "AVAILABLE"  # AVAILABLE | UNAVAILABLE
    provenance: str = "GENERIC_FIXED_WING_KINEMATIC"
    altitude_resolution: str = "constant_all_altitudes"
    reason_unavailable: Optional[str] = None
    level_turn: Optional[LevelTurnKinematics] = None
    signed_vertical_rate_mps: Optional[float] = None

    @property
    def combined_turn(self) -> Optional[CombinedTurnKinematics]:
        if self.level_turn is None or self.signed_vertical_rate_mps is None:
            return None
        return CombinedTurnKinematics(self.level_turn, self.signed_vertical_rate_mps)


@dataclass(frozen=True)
class SpiralTurnMetrics:
    """Physical facts for one full ideal helical revolution."""

    orbit_radius_m: float
    duration_s: float
    horizontal_arc_length_m: float
    altitude_change_m: float


class FixedWingKinematicEnvelope:
    """Authoritative fixed-wing kinematic envelope across all altitudes."""

    def __init__(self, model: FixedWingKinematicModel = FixedWingKinematicModel()):
        self.model = model
        self._turn_radius_m = model.turn_radius_m
        self._turn_rate_deg_s = model.turn_rate_deg_s

    @property
    def horizontal_speed_mps(self) -> float:
        return self.model.horizontal_speed_mps

    @property
    def turn_radius_m(self) -> float:
        return self._turn_radius_m

    @property
    def turn_rate_deg_s(self) -> float:
        return self._turn_rate_deg_s

    @property
    def max_climb_rate_mps(self) -> float:
        return self.model.max_climb_rate_mps

    @property
    def max_descent_rate_mps(self) -> float:
        return self.model.max_descent_rate_mps

    def level_turn(self, altitude_m: float, direction: str) -> DerivedManeuverLimit:
        if direction not in ("LEFT", "RIGHT"):
            raise ValueError(f"direction must be LEFT or RIGHT, got {direction!r}")
        radius = self._turn_radius_m
        sign = -1.0 if direction == "LEFT" else 1.0
        rate = sign * self._turn_rate_deg_s
        return DerivedManeuverLimit(
            "LEVEL_TURN",
            altitude_m,
            "AVAILABLE",
            "GENERIC_FIXED_WING_KINEMATIC",
            "constant_25deg_bank",
            level_turn=LevelTurnKinematics(direction, radius, rate),
        )

    def straight_vertical(self, altitude_m: float, mode: str) -> DerivedManeuverLimit:
        if mode not in ("CLIMB", "DESCENT"):
            raise ValueError(f"mode must be CLIMB or DESCENT, got {mode!r}")
        rate = self.model.max_climb_rate_mps if mode == "CLIMB" else -self.model.max_descent_rate_mps
        return DerivedManeuverLimit(
            f"STRAIGHT_{mode}",
            altitude_m,
            "AVAILABLE",
            "GENERIC_FIXED_WING_KINEMATIC",
            "constant_5mps_vertical",
            signed_vertical_rate_mps=rate,
        )

    def combined_turn(self, altitude_m: float, direction: str, mode: str) -> DerivedManeuverLimit:
        """Derive climbing/descending turn kinematics from unified parameters."""
        if mode not in ("CLIMB", "DESCENT"):
            raise ValueError(f"mode must be CLIMB or DESCENT, got {mode!r}")
        radius = self._turn_radius_m * self.model.combined_radius_factor
        sign = -1.0 if direction == "LEFT" else 1.0
        rate = sign * (self.model.horizontal_speed_mps / radius) * 180.0 / math.pi
        magnitude = (
            self.model.max_climb_rate_mps if mode == "CLIMB" else self.model.max_descent_rate_mps
        ) * self.model.combined_vertical_rate_factor
        signed_vz = magnitude if mode == "CLIMB" else -magnitude
        family = "CLIMBING_TURN" if mode == "CLIMB" else "DESCENDING_TURN"
        return DerivedManeuverLimit(
            family,
            altitude_m,
            "AVAILABLE",
            "GENERIC_FIXED_WING_KINEMATIC",
            "constant_combined_turn",
            level_turn=LevelTurnKinematics(direction, radius, rate),
            signed_vertical_rate_mps=signed_vz,
        )

    def spiral_up(self, altitude_m: float, direction: str) -> DerivedManeuverLimit:
        """Return a full-circle macro built from a climbing turn."""
        result = self.combined_turn(altitude_m, direction, "CLIMB")
        return DerivedManeuverLimit(
            "SPIRAL_UP",
            altitude_m,
            result.availability,
            result.provenance,
            result.altitude_resolution,
            result.reason_unavailable,
            result.level_turn,
            result.signed_vertical_rate_mps,
        )

    def loiter_orbit(self, altitude_m: float, direction: str) -> DerivedManeuverLimit:
        """Return a level turn under an explicit full-orbit macro name."""
        result = self.level_turn(altitude_m, direction)
        return DerivedManeuverLimit(
            "LOITER_ORBIT",
            altitude_m,
            result.availability,
            result.provenance,
            result.altitude_resolution,
            result.reason_unavailable,
            result.level_turn,
        )

    @staticmethod
    def full_turn_metrics(limit: DerivedManeuverLimit) -> SpiralTurnMetrics:
        combined = limit.combined_turn
        if limit.availability != "AVAILABLE" or combined is None:
            raise ValueError("full_turn_metrics requires an available combined maneuver")
        radius = combined.level_turn.radius_m
        horizontal_arc_length = 2.0 * math.pi * radius
        duration = horizontal_arc_length / FIXED_PLANAR_SPEED_MPS
        return SpiralTurnMetrics(
            orbit_radius_m=radius,
            duration_s=duration,
            horizontal_arc_length_m=horizontal_arc_length,
            altitude_change_m=combined.signed_vertical_rate_mps * duration,
        )
