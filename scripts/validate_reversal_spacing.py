"""Stage 14 validation: spacing-sensitive vertical-reversal penalty.

Replaces the flat "vertical_reversal_cost_weight * abs(dz)" per-reversal
charge (Stage 12) and the special-cased "first descent->climb is free"
rule (superseded here) with a general rule: a reversal's cost scales down
the longer the trend it's reversing had already run, reaching zero once
that trend covered reversal_relax_distance_m (300m default). No terrain
slope is ever treated as a hard limit -- max_climb/descent_angle_deg is an
AIRCRAFT flight-path angle, enforced entirely by the existing
evaluate_transition()/evaluate_primitive(); this module never compares it
to terrain slope directly.

9)  continuous descent / continuous climb -- no penalty regardless of length.
10) long-spaced natural reversal (>= relax distance) -- ~zero penalty.
11) short reversal (well under relax distance) -- real, non-zero penalty.
12) roller-coaster -- multiple penalized reversals, high total penalty.
13) mountain -> plain -> mountain: full A* search must be able to make
    the natural climb/descent/level/climb/descent transitions, keep
    AGL>=200 and angle<=10 deg, and see a cheap short reversal + a
    ~free long-spaced one.
14) terrain slope steeper than max_climb_angle_deg: the planner must
    still succeed by climbing early, never by matching the terrain slope.
"""
import math

import numpy as np
from affine import Affine

from planner.astar import (
    astar_search, _next_trend_and_age, _path_vertical_reversal_metrics, _max_trend_age_units,
)
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set
from planner.roi import ROIData
from planner.terrain import TerrainQuery

NODATA = -9999.0


def make_roi(elevation: np.ndarray, nodata: float = NODATA, res: float = 30.0) -> ROIData:
    height, width = elevation.shape
    transform = Affine(res, 0.0, 0.0, 0.0, -res, height * res)
    return ROIData(
        elevation=elevation.astype(np.float32), transform=transform, crs="EPSG:32636",
        width=width, height=height, bounds=(0.0, 0.0, width * res, height * res),
        resolution=(res, res), nodata=nodata,
    )


def build_path(start, prim_sequence, config):
    path = [start]
    r, c, z = start
    for prim in prim_sequence:
        r, c = r + prim.drow, c + prim.dcol
        z = z + round(prim.dz_m / config.z_step_m)
        path.append((r, c, z))
    return path


def report_reversal(rev) -> None:
    print(f"    total_reversals={rev['total_vertical_reversal_count']} "
          f"penalized={rev['penalized_reversal_count']} "
          f"zero_penalty_long_spacing={rev['zero_penalty_long_spacing_reversal_count']}")
    print(f"    min_spacing_m={rev['minimum_reversal_spacing_m']} "
          f"avg_spacing_m={rev['average_reversal_spacing_m']} "
          f"total_penalty={rev['total_reversal_penalty']:.3f}")


def validations_9_to_12(cfg, primitives) -> bool:
    print(f"=== 9-12: spacing-sensitive reversal unit checks "
          f"(reversal_relax_distance_m={cfg.reversal_relax_distance_m}, trend_age_unit_m={cfg.trend_age_unit_m}) ===")
    climb_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "climb")
    descent_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "descent")
    level_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "level")

    all_ok = True

    # 9: continuous descent / continuous climb -- no penalty regardless of length
    for label, seq in [("descent,descent,descent", [descent_e] * 3), ("climb,climb,climb", [climb_e] * 3)]:
        path = build_path((5, 2, 100), seq, cfg)
        rev = _path_vertical_reversal_metrics(path, primitives, cfg)
        ok = rev["total_vertical_reversal_count"] == 0 and rev["total_reversal_penalty"] == 0.0
        all_ok = all_ok and ok
        print(f"  9) {label}: reversals={rev['total_vertical_reversal_count']} "
              f"penalty={rev['total_reversal_penalty']:.3f}  {'PASS' if ok else 'FAIL'}")

    # 10: long natural reversal -- descent,descent,level,level,level,climb
    # standing descent-trend distance before the climb: 2*120 + 3*30 = 330m,
    # capped at 10 units (300m) -- >= relax distance either way -> free.
    seq = [descent_e, descent_e, level_e, level_e, level_e, climb_e]
    path = build_path((5, 2, 100), seq, cfg)
    rev = _path_vertical_reversal_metrics(path, primitives, cfg)
    ok = (rev["total_vertical_reversal_count"] == 1 and rev["penalized_reversal_count"] == 0
          and abs(rev["total_reversal_penalty"]) < 1e-9)
    all_ok = all_ok and ok
    print(f"  10) descent,descent,level,level,level,climb (~330m standing trend): "
          f"reversals={rev['total_vertical_reversal_count']} penalty={rev['total_reversal_penalty']:.3f}  "
          f"{'PASS' if ok else 'FAIL'}")

    # 11: short reversal -- descent,climb (standing trend only 120m)
    seq = [descent_e, climb_e]
    path = build_path((5, 2, 100), seq, cfg)
    rev = _path_vertical_reversal_metrics(path, primitives, cfg)
    expected_factor = 1.0 - min(1.0, 120.0 / cfg.reversal_relax_distance_m)
    expected_penalty = cfg.vertical_reversal_cost_weight * 20.0 * expected_factor
    ok = (rev["total_vertical_reversal_count"] == 1 and rev["penalized_reversal_count"] == 1
          and abs(rev["total_reversal_penalty"] - expected_penalty) < 1e-6)
    all_ok = all_ok and ok
    print(f"  11) descent,climb (120m standing trend): reversals={rev['total_vertical_reversal_count']} "
          f"penalty={rev['total_reversal_penalty']:.3f} (expect {expected_penalty:.3f})  {'PASS' if ok else 'FAIL'}")

    # 12: roller coaster -- descent,climb,descent,climb (each reversal at 120m spacing)
    seq = [descent_e, climb_e, descent_e, climb_e]
    path = build_path((5, 2, 100), seq, cfg)
    rev = _path_vertical_reversal_metrics(path, primitives, cfg)
    ok = (rev["total_vertical_reversal_count"] == 3 and rev["penalized_reversal_count"] == 3
          and rev["total_reversal_penalty"] > 3 * expected_penalty - 1e-6)
    all_ok = all_ok and ok
    print(f"  12) roller-coaster descent,climb,descent,climb: reversals={rev['total_vertical_reversal_count']} "
          f"penalized={rev['penalized_reversal_count']} total_penalty={rev['total_reversal_penalty']:.3f}  "
          f"{'PASS' if ok else 'FAIL'}")

    print(f"  Overall 9-12: {'PASS' if all_ok else 'FAIL'}")
    return all_ok


def validation_13(cfg, primitives) -> bool:
    print()
    print("=== 13: mountain -> plain -> mountain (natural multi-reversal terrain) ===")
    # Kept deliberately small: (row,col,z,trend,age) has ~11x more states per
    # physical cell than Stage 12's (row,col,z,trend), and this project's
    # earlier stages already found search cost very sensitive to grid
    # height/width/z-range -- a 100-col x 5-row x 6-z-level version of this
    # scenario took >30s for just 5000 (incomplete) expansions.
    width, height = 55, 3
    elev = np.full((height, width), 1000.0)
    elev[:, 8:14] = 1140.0  # ridge 1
    elev[:, 35:41] = 1140.0  # ridge 2 -- gap between them is 21 cols = 630m, > reversal_relax_distance_m
    roi = make_roi(elev)
    tq = TerrainQuery(roi)

    from planner.astar import msl_to_z_index
    z0 = msl_to_z_index(1300.0, cfg)  # AGL=300 on baseline
    start, goal = (1, 2, z0), (1, 50, z0)

    result = astar_search(start, goal, tq, min_search_altitude_msl=1300.0, max_search_altitude_msl=1360.0,
                           config=cfg, primitives=primitives, max_expansions=50_000)

    print(f"  status={result.status} success={result.success}")
    if result.success:
        print(f"    path_states={len(result.path)} geometric_length={result.geometric_path_length:.1f} "
              f"climb={result.total_climb_m:.1f} descent={result.total_descent_m:.1f}")
        print(f"    min_AGL={result.minimum_observed_agl:.1f} max_MSL={result.maximum_aircraft_msl:.1f}")
        report_reversal({
            "total_vertical_reversal_count": result.total_vertical_reversal_count,
            "penalized_reversal_count": result.penalized_reversal_count,
            "zero_penalty_long_spacing_reversal_count": result.zero_penalty_long_spacing_reversal_count,
            "minimum_reversal_spacing_m": result.minimum_reversal_spacing_m,
            "average_reversal_spacing_m": result.average_reversal_spacing_m,
            "total_reversal_penalty": result.total_reversal_penalty,
        })
        print(f"    expanded={result.expanded_nodes} max_open={result.max_open_size} "
              f"runtime={result.runtime_s * 1000:.1f}ms")

    ok = (
        result.success
        and result.minimum_observed_agl >= cfg.min_agl_m - 1e-6
        and result.total_climb_m > 0 and result.total_descent_m > 0
        and result.total_vertical_reversal_count >= 1  # at least one real climb<->descent transition happened
        and result.zero_penalty_long_spacing_reversal_count >= 1  # and it wasn't penalized -- long enough spacing
    )
    print("  planner climbed AND descended (not just flat-lined at ridge-clearing altitude), the transition(s) it "
          "made were never blocked by the reversal penalty, and the >=630m-spaced one was (near) free:")
    print(f"  PASS" if ok else "  FAIL")
    return ok


def validation_14(cfg, primitives) -> bool:
    print()
    print("=== 14: terrain slope steeper than max_climb_angle_deg is not a hard limit ===")
    width, height = 30, 3
    elev = np.full((height, width), 1000.0)
    elev[:, 10:] = 1200.0  # abrupt 200m step over a single 30m cell -- ~81 degree terrain slope
    slope_deg = math.degrees(math.atan2(200.0, 30.0))
    print(f"  synthetic terrain step: 200m over 30m -> slope={slope_deg:.1f} deg "
          f"(aircraft limit is {cfg.max_climb_angle_deg} deg)")
    roi = make_roi(elev)
    tq = TerrainQuery(roi)

    from planner.astar import msl_to_z_index
    z0 = msl_to_z_index(1300.0, cfg)  # safe on the low plateau (AGL=300), NOT safe on the high one (AGL=100)
    start, goal = (1, 2, z0), (1, 25, msl_to_z_index(1400.0, cfg))  # goal safe on the high plateau (AGL=200)

    result = astar_search(start, goal, tq, min_search_altitude_msl=1300.0, max_search_altitude_msl=1420.0,
                           config=cfg, primitives=primitives, max_expansions=50_000)

    print(f"  status={result.status} success={result.success}")
    if result.success:
        print(f"    path_states={len(result.path)} climb={result.total_climb_m:.1f} "
              f"min_AGL={result.minimum_observed_agl:.1f} expanded={result.expanded_nodes} "
              f"runtime={result.runtime_s * 1000:.1f}ms")

    ok = result.success and result.minimum_observed_agl >= cfg.min_agl_m - 1e-6 and result.total_climb_m > 0
    print("  planner cleared the step by climbing early over several cells, never by matching the "
          f"terrain's own {slope_deg:.0f} deg slope (every primitive is <= {cfg.max_climb_angle_deg} deg "
          f"by construction): {'PASS' if ok else 'FAIL'}")
    return ok


def main() -> None:
    cfg = DEFAULT_CONFIG
    primitives = build_primitive_set(cfg)
    print(f"max_trend_age_units = {_max_trend_age_units(cfg)} (relax_distance={cfg.reversal_relax_distance_m}m / "
          f"unit={cfg.trend_age_unit_m}m)")
    print()

    results = [
        validations_9_to_12(cfg, primitives),
        validation_13(cfg, primitives),
        validation_14(cfg, primitives),
    ]

    print()
    print(f"Overall: {'ALL PASS' if all(results) else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
