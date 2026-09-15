"""Admissible lower-bound candidate for the BASIC pose-aware experiment.

This module is intentionally outside ``planner``: production continues to use
the Euclidean heuristic unless an experiment explicitly patches a single run.
"""
from __future__ import annotations

import math

from planner.physical import FIXED_PLANAR_SPEED_MPS, PhysicalPose
from planner.pose_search import GoalPose, GoalTolerance


GLOBAL_OPTIMISTIC_VZ_UPPER_MPS = 5.0


def goal_region_residuals(pose: PhysicalPose, goal: GoalPose,
                          tolerance: GoalTolerance) -> tuple[float, float]:
    """Conservative horizontal/vertical displacement to the accepted goal region.

    The horizontal term deliberately matches the production Euclidean
    heuristic's axis-aligned lower bound.  Using raw distance to the exact
    goal would overestimate when the physical goal tolerance already accepts
    the pose.
    """
    dx = max(abs(pose.x_m - goal.x_m) - tolerance.xy_m, 0.0)
    dy = max(abs(pose.y_m - goal.y_m) - tolerance.xy_m, 0.0)
    dxy = math.hypot(dx, dy)
    dz = max(abs(pose.z_msl_m - goal.z_msl_m) - tolerance.altitude_m, 0.0)
    return dxy, dz


def vertical_reachability_heuristic(
    pose: PhysicalPose,
    goal: GoalPose,
    tolerance: GoalTolerance,
    *,
    vz_upper_mps: float = GLOBAL_OPTIMISTIC_VZ_UPPER_MPS,
) -> float:
    """Return a fixed-40 vertical-reachability lower-bound candidate.

    BASIC has only level primitives plus straight vertical primitives.  The
    latter preserve a 40 m/s horizontal speed and are capped at 5 m/s in the
    derived envelope.  Thus for a net remaining vertical correction ``Dz``:

    ``H >= Dz * 40 / Vz_upper`` and ``H >= Dxy``.

    Any path's sampled 3D length is at least
    ``sqrt(max(Dxy, Dz * 40 / Vz_upper)^2 + Dz^2)`` by the triangle
    inequality.  Goal-region residuals are used so acceptance tolerances do
    not turn the bound into an overestimate.
    """
    if not math.isfinite(vz_upper_mps) or vz_upper_mps <= 0.0:
        raise ValueError("vz_upper_mps must be finite and positive")
    dxy, dz = goal_region_residuals(pose, goal, tolerance)
    vertical_horizontal_min = dz * FIXED_PLANAR_SPEED_MPS / vz_upper_mps
    horizontal_min = max(dxy, vertical_horizontal_min)
    return math.hypot(horizontal_min, dz)


__all__ = [
    "GLOBAL_OPTIMISTIC_VZ_UPPER_MPS", "goal_region_residuals",
    "vertical_reachability_heuristic",
]
