"""Derived C172P planner envelope with explicit conservative provenance.

This is a reference-aircraft binding above the aircraft-neutral V3 profile.
It makes ideal fixed-wing primitives available to the future pose-aware layer;
derived combined manoeuvres are not represented as JSBSim-validated flights.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from planner.aircraft_profile import AircraftProfile
from planner.physical import (
    FIXED_PLANAR_SPEED_MPS,
    CombinedTurnKinematics,
    LevelTurnKinematics,
)


LUT_SAFE_SOURCE = "LUT_SAFE_SOURCE"
DERIVED_CONSERVATIVE = "DERIVED_CONSERVATIVE"
UNSUPPORTED = "UNSUPPORTED"


@dataclass(frozen=True)
class DerivedC172PPolicy:
    """The declared assumptions behind the ideal fixed-wing reference model.

    V3 has level-turn evidence at 20 degrees, not 25.  The 25-degree reference
    is thus an analytic fixed-40 bound, never a claim of 25-degree validation.
    The 5 m/s vertical number is a ceiling and cannot increase a lower safe
    rate supplied by the LUT.
    """

    source_turn_bank_deg: float = 20.0
    reference_bank_deg: float = 25.0
    max_abs_vertical_rate_mps: float = 5.0
    combined_radius_factor: float = 1.25
    combined_vertical_rate_factor: float = 0.50

    def __post_init__(self) -> None:
        names = (
            "source_turn_bank_deg", "reference_bank_deg", "max_abs_vertical_rate_mps",
            "combined_radius_factor", "combined_vertical_rate_factor",
        )
        for name in names:
            value = float(getattr(self, name))
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
            object.__setattr__(self, name, value)
        if not 0.0 < self.source_turn_bank_deg < 90.0:
            raise ValueError("source_turn_bank_deg must be in (0, 90)")
        if not 0.0 < self.reference_bank_deg < 90.0:
            raise ValueError("reference_bank_deg must be in (0, 90)")
        if self.max_abs_vertical_rate_mps <= 0.0:
            raise ValueError("max_abs_vertical_rate_mps must be positive")
        if self.combined_radius_factor < 1.0:
            raise ValueError("combined_radius_factor must not shrink the turn radius")
        if not 0.0 < self.combined_vertical_rate_factor <= 1.0:
            raise ValueError("combined_vertical_rate_factor must be in (0, 1]")


@dataclass(frozen=True)
class DerivedManeuverLimit:
    """One altitude-local answer whose origin can never be mistaken later."""

    family: str
    altitude_m: float
    availability: str  # AVAILABLE | UNAVAILABLE | OUT_OF_DOMAIN
    provenance: str  # LUT_SAFE_SOURCE | DERIVED_CONSERVATIVE | UNSUPPORTED
    altitude_resolution: str
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


class DerivedC172PEnvelope:
    """Turn V3 source values into a complete conservative planner envelope.

    This object is passive: it has no grid state, DEM dependency, or A* hook.
    """

    def __init__(self, profile: AircraftProfile, policy: DerivedC172PPolicy = DerivedC172PPolicy()):
        if not math.isclose(profile.manifest.nominal_ias_context_mps, FIXED_PLANAR_SPEED_MPS,
                            rel_tol=0.0, abs_tol=1e-9):
            raise ValueError("DerivedC172PEnvelope requires the frozen 40 m/s profile context")
        self.profile = profile
        self.policy = policy

    def level_turn(self, altitude_m: float, direction: str) -> DerivedManeuverLimit:
        if direction not in ("LEFT", "RIGHT"):
            raise ValueError(f"direction must be LEFT or RIGHT, got {direction!r}")
        left = self.profile.turn_query(altitude_m, "LEFT", self.policy.source_turn_bank_deg)
        right = self.profile.turn_query(altitude_m, "RIGHT", self.policy.source_turn_bank_deg)
        if left.availability != "AVAILABLE" or right.availability != "AVAILABLE":
            failed = left if left.availability != "AVAILABLE" else right
            return DerivedManeuverLimit(
                "LEVEL_TURN", altitude_m, failed.availability, UNSUPPORTED,
                failed.altitude_resolution, failed.reason_unavailable,
            )

        # One symmetric planner radius must respect the restrictive hand.
        lut_radius = max(float(left.planner_safe["turn_radius_m"]),
                         float(right.planner_safe["turn_radius_m"]))
        bank_radius = FIXED_PLANAR_SPEED_MPS ** 2 / (
            9.80665 * math.tan(math.radians(self.policy.reference_bank_deg))
        )
        radius = max(lut_radius, bank_radius)
        sign = -1.0 if direction == "LEFT" else 1.0
        rate = sign * FIXED_PLANAR_SPEED_MPS / radius * 180.0 / math.pi
        return DerivedManeuverLimit(
            "LEVEL_TURN", altitude_m, "AVAILABLE", DERIVED_CONSERVATIVE,
            f"worst_of_{left.altitude_resolution}_{right.altitude_resolution}",
            level_turn=LevelTurnKinematics(direction, radius, rate),
        )

    def straight_vertical(self, altitude_m: float, mode: str) -> DerivedManeuverLimit:
        if mode not in ("CLIMB", "DESCENT"):
            raise ValueError(f"mode must be CLIMB or DESCENT, got {mode!r}")
        source = self.profile.vertical_query(altitude_m, mode)
        if source.availability != "AVAILABLE":
            return DerivedManeuverLimit(
                f"STRAIGHT_{mode}", altitude_m, source.availability, UNSUPPORTED,
                source.altitude_resolution, source.reason_unavailable,
            )
        key = "climb_vz_mps" if mode == "CLIMB" else "descent_vz_mps"
        source_rate = float(source.planner_safe[key])
        magnitude = min(abs(source_rate), self.policy.max_abs_vertical_rate_mps)
        rate = magnitude if mode == "CLIMB" else -magnitude
        provenance = LUT_SAFE_SOURCE if math.isclose(magnitude, abs(source_rate), rel_tol=0.0, abs_tol=1e-12) \
            else DERIVED_CONSERVATIVE
        return DerivedManeuverLimit(
            f"STRAIGHT_{mode}", altitude_m, "AVAILABLE", provenance,
            source.altitude_resolution, signed_vertical_rate_mps=rate,
        )

    def combined_turn(self, altitude_m: float, direction: str, mode: str) -> DerivedManeuverLimit:
        """Derive a conservative combined turn from separate LUT-safe inputs."""
        if mode not in ("CLIMB", "DESCENT"):
            raise ValueError(f"mode must be CLIMB or DESCENT, got {mode!r}")
        turn = self.level_turn(altitude_m, direction)
        vertical = self.straight_vertical(altitude_m, mode)
        family = "CLIMBING_TURN" if mode == "CLIMB" else "DESCENDING_TURN"
        if turn.availability != "AVAILABLE" or vertical.availability != "AVAILABLE":
            failed = turn if turn.availability != "AVAILABLE" else vertical
            return DerivedManeuverLimit(
                family, altitude_m, failed.availability, UNSUPPORTED,
                failed.altitude_resolution, failed.reason_unavailable,
            )
        assert turn.level_turn is not None and vertical.signed_vertical_rate_mps is not None
        radius = turn.level_turn.radius_m * self.policy.combined_radius_factor
        sign = -1.0 if direction == "LEFT" else 1.0
        rate = sign * FIXED_PLANAR_SPEED_MPS / radius * 180.0 / math.pi
        return DerivedManeuverLimit(
            family, altitude_m, "AVAILABLE", DERIVED_CONSERVATIVE,
            f"derived_from_{turn.altitude_resolution}_{vertical.altitude_resolution}",
            level_turn=LevelTurnKinematics(direction, radius, rate),
            signed_vertical_rate_mps=(vertical.signed_vertical_rate_mps * self.policy.combined_vertical_rate_factor),
        )

    def spiral_up(self, altitude_m: float, direction: str) -> DerivedManeuverLimit:
        """Return a full-circle macro built from a derived climbing turn."""
        result = self.combined_turn(altitude_m, direction, "CLIMB")
        return DerivedManeuverLimit(
            "SPIRAL_UP", altitude_m, result.availability, result.provenance,
            result.altitude_resolution, result.reason_unavailable,
            result.level_turn, result.signed_vertical_rate_mps,
        )

    def loiter_orbit(self, altitude_m: float, direction: str) -> DerivedManeuverLimit:
        """Return the level-turn envelope under an explicit full-orbit macro name.

        Geometry creates the actual complete circle by requesting 360 degrees;
        keeping that magnitude outside the envelope lets a future planner use
        the same limit for partial turns without a duplicate capability.
        """
        result = self.level_turn(altitude_m, direction)
        return DerivedManeuverLimit(
            "LOITER_ORBIT", altitude_m, result.availability, result.provenance,
            result.altitude_resolution, result.reason_unavailable,
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
