"""Disposable pre-PERF-9B primitive-safety reference implementation."""

import math
from typing import List

from planner.agl import AGLResult
from planner.config import DEFAULT_CONFIG, PlannerConfig
from planner.primitives import MotionPrimitive, Point3, PrimitiveEvalResult, PrimitiveSample
from planner.terrain import TerrainQuery
from planner.transition import evaluate_transition


def evaluate_agl_reference(
    terrain: TerrainQuery,
    x: float,
    y: float,
    aircraft_altitude_msl: float,
    config: PlannerConfig = DEFAULT_CONFIG,
) -> AGLResult:
    """The TerrainQueryResult/AGLResult path that preceded PERF-9B."""
    terrain_result = terrain.query(x, y)
    if not terrain_result.valid:
        return AGLResult(
            x=x, y=y, aircraft_altitude_msl=aircraft_altitude_msl,
            terrain_elevation_msl=float("nan"), agl_m=float("nan"),
            valid=False, reason=terrain_result.reason,
        )
    if config.min_agl_m is None:
        raise ValueError(
            "config.min_agl_m is not set -- cannot evaluate AGL feasibility without a threshold"
        )
    terrain_elevation_msl = terrain_result.elevation
    agl_m = aircraft_altitude_msl - terrain_elevation_msl
    if agl_m < config.min_agl_m:
        return AGLResult(
            x=x, y=y, aircraft_altitude_msl=aircraft_altitude_msl,
            terrain_elevation_msl=terrain_elevation_msl, agl_m=agl_m,
            valid=False, reason="below_min_agl",
        )
    return AGLResult(
        x=x, y=y, aircraft_altitude_msl=aircraft_altitude_msl,
        terrain_elevation_msl=terrain_elevation_msl, agl_m=agl_m,
        valid=True, reason="ok",
    )


def evaluate_primitive_reference(
    start: Point3,
    primitive: MotionPrimitive,
    terrain: TerrainQuery,
    config: PlannerConfig = DEFAULT_CONFIG,
) -> PrimitiveEvalResult:
    """The list-building evaluator that preceded PERF-9B."""
    x1, y1, z1 = start
    dx = primitive.dcol * config.xy_resolution_m
    dy = -primitive.drow * config.xy_resolution_m
    end = (x1 + dx, y1 + dy, z1 + primitive.dz_m)

    transition = evaluate_transition(start, end, config)
    if not transition.valid:
        return PrimitiveEvalResult(
            valid=False,
            reason="transition_invalid",
            start=start,
            end=end,
            primitive_type=primitive.primitive_type,
            horizontal_distance_m=transition.horizontal_distance_m,
            delta_z_m=transition.delta_z_m,
            sample_count=0,
            min_agl_m=float("nan"),
            min_agl_sample=None,
            first_failure=None,
        )

    n_intervals = max(1, math.ceil(transition.horizontal_distance_m / config.primitive_sample_spacing_m))
    samples: List[PrimitiveSample] = []
    for i in range(n_intervals + 1):
        t = i / n_intervals
        x, y = x1 + dx * t, y1 + dy * t
        altitude_msl = z1 + primitive.dz_m * t
        agl = evaluate_agl_reference(terrain, x, y, altitude_msl, config)
        samples.append(PrimitiveSample(
            index=i, t=t, x=x, y=y, altitude_msl=altitude_msl,
            terrain_elevation_msl=agl.terrain_elevation_msl,
            agl_m=agl.agl_m, valid=agl.valid, reason=agl.reason,
        ))

    numeric = [sample for sample in samples if not math.isnan(sample.agl_m)]
    min_sample = min(numeric, key=lambda sample: sample.agl_m) if numeric else samples[0]
    first_failure = next((sample for sample in samples if not sample.valid), None)
    return PrimitiveEvalResult(
        valid=(first_failure is None),
        reason="ok" if first_failure is None else first_failure.reason,
        start=start,
        end=end,
        primitive_type=primitive.primitive_type,
        horizontal_distance_m=transition.horizontal_distance_m,
        delta_z_m=transition.delta_z_m,
        sample_count=len(samples),
        min_agl_m=min_sample.agl_m,
        min_agl_sample=min_sample,
        first_failure=first_failure,
    )
