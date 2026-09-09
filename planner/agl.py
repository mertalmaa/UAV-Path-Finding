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


def evaluate_agl(
    terrain: TerrainQuery,
    x: float,
    y: float,
    aircraft_altitude_msl: float,
    config: PlannerConfig = DEFAULT_CONFIG,
) -> AGLResult:
    terrain_result = terrain.query(x, y)

    if not terrain_result.valid:
        # terrain_result.reason is "out_of_bounds" or "nodata" -- both are
        # hard INVALID here, we can't evaluate AGL without a real elevation.
        return AGLResult(
            x=x,
            y=y,
            aircraft_altitude_msl=aircraft_altitude_msl,
            terrain_elevation_msl=float("nan"),
            agl_m=float("nan"),
            valid=False,
            reason=terrain_result.reason,
        )

    if config.min_agl_m is None:
        raise ValueError(
            "config.min_agl_m is not set -- cannot evaluate AGL feasibility without a threshold"
        )

    terrain_elevation_msl = terrain_result.elevation
    agl_m = aircraft_altitude_msl - terrain_elevation_msl

    if agl_m < config.min_agl_m:
        return AGLResult(
            x=x, y=y,
            aircraft_altitude_msl=aircraft_altitude_msl,
            terrain_elevation_msl=terrain_elevation_msl,
            agl_m=agl_m,
            valid=False,
            reason="below_min_agl",
        )

    return AGLResult(
        x=x, y=y,
        aircraft_altitude_msl=aircraft_altitude_msl,
        terrain_elevation_msl=terrain_elevation_msl,
        agl_m=agl_m,
        valid=True,
        reason="ok",
    )
