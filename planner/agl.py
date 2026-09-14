"""AGL (above-ground-level) feasibility gate for a single (x, y, altitude) point.

AGL is used here purely as a hard safety/feasibility check:

    agl_m = aircraft_altitude_msl - terrain_elevation_msl
    agl_m <  min_agl_m  -> INVALID ("below_min_agl")
    agl_m >= min_agl_m  -> VALID

This is NOT a cost function and does not express any altitude preference.
A future low-altitude preference will be a separate MSL-altitude-based cost
term added to the planner's cost function later -- not this module, and not
by preferring a particular AGL value here.

Terrain access goes entirely through TerrainQuery (planner/terrain.py); this
module does not touch the ROI array or DEM directly.
"""
from dataclasses import dataclass
import math
from typing import Tuple

from planner.config import DEFAULT_CONFIG, PlannerConfig
from planner.terrain import TerrainQuery


@dataclass(frozen=True)
class AGLResult:
    x: float
    y: float
    aircraft_altitude_msl: float
    terrain_elevation_msl: float  # NaN when terrain lookup failed
    agl_m: float  # NaN when terrain lookup failed
    valid: bool
    reason: str  # "ok" | "out_of_bounds" | "nodata" | "below_min_agl"


def _evaluate_agl_payload(
    terrain: TerrainQuery,
    x: float,
    y: float,
    aircraft_altitude_msl: float,
    config: PlannerConfig = DEFAULT_CONFIG,
) -> Tuple[float, float, bool, str]:
    """Return evaluate_agl's scalar result without allocating AGLResult.

    This is the single authoritative terrain/AGL calculation shared by the
    public result-object API and primitive safety's streaming hot path.
    """
    if config.min_agl_m is None:
        raise ValueError(
            "config.min_agl_m is not set -- cannot evaluate AGL feasibility without a threshold"
        )
    return _evaluate_agl_with_min_payload(terrain, x, y, aircraft_altitude_msl, config.min_agl_m)


def _evaluate_agl_with_min_payload(
    terrain: TerrainQuery,
    x: float,
    y: float,
    aircraft_altitude_msl: float,
    effective_min_agl_m: float,
) -> Tuple[float, float, bool, str]:
    """AGL scalar helper with an explicit threshold for physical trajectories.

    The existing grid primitive path supplies its config through
    :func:`_evaluate_agl_payload`; the continuous path supplies a mission's
    effective threshold directly.  Both retain exactly the same terrain lookup
    and inclusive AGL-boundary semantics.
    """
    if not math.isfinite(effective_min_agl_m) or effective_min_agl_m < 0.0:
        raise ValueError("effective_min_agl_m must be finite and non-negative")
    _, _, terrain_elevation_msl, terrain_valid, terrain_reason = terrain._query_payload(x, y)
    if not terrain_valid:
        return float("nan"), float("nan"), False, terrain_reason

    agl_m = aircraft_altitude_msl - terrain_elevation_msl
    if agl_m < effective_min_agl_m:
        return terrain_elevation_msl, agl_m, False, "below_min_agl"
    return terrain_elevation_msl, agl_m, True, "ok"


def evaluate_agl(
    terrain: TerrainQuery,
    x: float,
    y: float,
    aircraft_altitude_msl: float,
    config: PlannerConfig = DEFAULT_CONFIG,
) -> AGLResult:
    terrain_elevation_msl, agl_m, valid, reason = _evaluate_agl_payload(
        terrain, x, y, aircraft_altitude_msl, config
    )
    return AGLResult(
        x=x, y=y,
        aircraft_altitude_msl=aircraft_altitude_msl,
        terrain_elevation_msl=terrain_elevation_msl,
        agl_m=agl_m,
        valid=valid,
        reason=reason,
    )
