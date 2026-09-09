"""Geometric climb/descent transition feasibility between two aircraft states.

Given start=(x1, y1, z1_msl) and end=(x2, y2, z2_msl), checks whether the
straight-line flight-path angle between them is within the configured
max climb/descent angle. Purely geometric: no terrain sampling along the
path, no AGL check, no motion primitive, no cost. Those are separate
concerns and stay in their own modules.

    d_xy   = hypot(x2-x1, y2-y1)
    delta_z = z2 - z1
    angle  = degrees(atan2(abs(delta_z), d_xy))
"""
import math
from dataclasses import dataclass
from typing import Tuple

from planner.config import DEFAULT_CONFIG, PlannerConfig

Point3 = Tuple[float, float, float]  # (x, y, z_msl)


@dataclass(frozen=True)
class TransitionResult:
    horizontal_distance_m: float
    delta_z_m: float
    flight_path_angle_deg: float
    transition_type: str  # "level" | "climb" | "descent"
    valid: bool
    reason: str  # "ok" | "no_motion" | "vertical_jump" | "exceeds_max_climb_angle" | "exceeds_max_descent_angle"


def evaluate_transition(
    start: Point3,
    end: Point3,
    config: PlannerConfig = DEFAULT_CONFIG,
) -> TransitionResult:
    x1, y1, z1 = start
    x2, y2, z2 = end

    d_xy = math.hypot(x2 - x1, y2 - y1)
    delta_z = z2 - z1

    if delta_z > 0:
        transition_type = "climb"
    elif delta_z < 0:
        transition_type = "descent"
    else:
        transition_type = "level"

    if d_xy == 0.0 and delta_z == 0.0:
        # Degenerate: start and end are the same state. Not a failure, but
        # kept distinct from a normal "ok" transition via its own reason.
        return TransitionResult(
            horizontal_distance_m=0.0,
            delta_z_m=0.0,
            flight_path_angle_deg=0.0,
            transition_type=transition_type,
            valid=True,
            reason="no_motion",
        )

    if d_xy == 0.0 and delta_z != 0.0:
        # Pure vertical jump with no horizontal travel -- undefined for a
        # fixed-wing flight path, always infeasible regardless of angle limits.
        angle = math.degrees(math.atan2(abs(delta_z), d_xy))  # == 90.0
        return TransitionResult(
            horizontal_distance_m=0.0,
            delta_z_m=delta_z,
            flight_path_angle_deg=angle,
            transition_type=transition_type,
            valid=False,
            reason="vertical_jump",
        )

    angle = math.degrees(math.atan2(abs(delta_z), d_xy))

    if transition_type == "climb":
        if config.max_climb_angle_deg is None:
            raise ValueError("config.max_climb_angle_deg is not set -- cannot evaluate climb feasibility")
        if angle > config.max_climb_angle_deg:
            return TransitionResult(d_xy, delta_z, angle, transition_type, False, "exceeds_max_climb_angle")
    elif transition_type == "descent":
        if config.max_descent_angle_deg is None:
            raise ValueError("config.max_descent_angle_deg is not set -- cannot evaluate descent feasibility")
        if angle > config.max_descent_angle_deg:
            return TransitionResult(d_xy, delta_z, angle, transition_type, False, "exceeds_max_descent_angle")

    return TransitionResult(d_xy, delta_z, angle, transition_type, True, "ok")
