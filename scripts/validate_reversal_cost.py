"""Stage 12 validation: vertical-direction reversal penalty replacing the
absolute abs(delta_z) vertical-motion penalty.

A-D: direct reversal-counting checks on hand-built physical paths (using
    the augmented-state trend state machine via _path_vertical_reversal_metrics
    / _is_reversal / _next_vertical_trend directly -- these are "private"
    helpers in planner/astar.py, reached into here the same way earlier
    stages reached into _altitude_scaled).
E:  roller-coaster (3 reversals) vs smoother (1 reversal) path to the exact
    same endpoint, same total vertical motion -- cost comparison.
F:  full A* search behavior test: does the search itself avoid reversals
    when a low-MSL dive makes some vertical motion worthwhile.
G:  low-MSL + continuous-descent: does removing the old abs(delta_z) term
    let the search commit to a smooth descent into a safe low corridor.
H:  real Aladaglar ROI check (same scenario as Stage 9/10/11).
"""
import dataclasses
import math
import time

import numpy as np
from affine import Affine

from planner.astar import (
    astar_search, compute_edge_cost, msl_to_z_index,
    _is_reversal, _next_vertical_trend, _path_vertical_reversal_metrics, _path_altitude_metrics,
)
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set
from planner.roi import ROIData, load_roi
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


def build_path(start, prim_sequence):
    """Chain primitives from a physical (row,col,z_index) start into a path."""
    path = [start]
    r, c, z = start
    for prim in prim_sequence:
        r, c = r + prim.drow, c + prim.dcol
        z = z + round(prim.dz_m / DEFAULT_CONFIG.z_step_m)
        path.append((r, c, z))
    return path


def report(label, result) -> None:
    print(f"  [{label}] status={result.status} success={result.success}")
    if not result.success:
        return
    print(f"    path_states={len(result.path)} geometric_length={result.geometric_path_length:.2f} "
          f"total_cost={result.total_cost:.2f}")
    print(f"    climb={result.total_climb_m:.1f} descent={result.total_descent_m:.1f} "
          f"vertical_motion={result.total_vertical_motion_m:.1f} "
          f"reversals={result.vertical_reversal_count} reversal_penalty={result.total_reversal_penalty:.2f}")
    print(f"    avg_MSL={result.average_aircraft_msl:.1f} min_AGL={result.minimum_observed_agl:.1f} "
          f"expanded={result.expanded_nodes} max_open={result.max_open_size} runtime={result.runtime_s * 1000:.1f}ms")


def validations_a_to_d(cfg, primitives) -> bool:
    print("=== A-D: direct reversal counting on hand-built paths ===")
    climb_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "climb")
    descent_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "descent")
    level_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "level")

    all_ok = True
    cases = [
        ("A) descent,descent,descent", [descent_e, descent_e, descent_e], 0, 0.0, 60.0, 0.0),
        ("B) climb,climb,climb", [climb_e, climb_e, climb_e], 0, 0.0, 0.0, 60.0),
        ("C1) descent,climb", [descent_e, climb_e], 1, cfg.vertical_reversal_cost_weight * 20.0, 20.0, 20.0),
        ("C2) climb,descent", [climb_e, descent_e], 1, cfg.vertical_reversal_cost_weight * 20.0, 20.0, 20.0),
        ("D1) descent,level,climb", [descent_e, level_e, climb_e], 1, cfg.vertical_reversal_cost_weight * 20.0, 20.0, 20.0),
        ("D2) descent,level,descent", [descent_e, level_e, descent_e], 0, 0.0, 0.0, 40.0),
    ]

    for label, seq, expect_count, expect_penalty, expect_climb, expect_descent in cases:
        path = build_path((5, 2, 100), seq)
        rev = _path_vertical_reversal_metrics(path, primitives, cfg)
        ok = (
            rev["vertical_reversal_count"] == expect_count
            and abs(rev["total_reversal_penalty"] - expect_penalty) < 1e-9
        )
        all_ok = all_ok and ok
        print(f"  {label}: reversals={rev['vertical_reversal_count']} (expect {expect_count}), "
              f"penalty={rev['total_reversal_penalty']:.2f} (expect {expect_penalty:.2f})  "
              f"{'PASS' if ok else 'FAIL'}")

    print(f"  Overall A-D: {'PASS' if all_ok else 'FAIL'}")
    return all_ok


def validation_e(cfg) -> bool:
    print()
    print("=== E: roller-coaster (3 reversals) vs smoother (1 reversal), same endpoint & vertical motion ===")
    primitives = build_primitive_set(cfg)
    climb_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "climb")
    descent_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "descent")

    start = (5, 2, 100)
    path_a = build_path(start, [descent_e, climb_e, descent_e, climb_e])  # zigzag
    path_b = build_path(start, [descent_e, descent_e, climb_e, climb_e])  # smoother -- no level hop needed,
    # each primitive (climb/descent/level) has its own horizontal reach, so a level step here would shift
    # the endpoint; 4 non-level hops in a different order keeps both paths at the identical endpoint.

    print(f"  path A endpoint={path_a[-1]}  path B endpoint={path_b[-1]}  (same endpoint: {path_a[-1] == path_b[-1]})")

    rev_a = _path_vertical_reversal_metrics(path_a, primitives, cfg)
    rev_b = _path_vertical_reversal_metrics(path_b, primitives, cfg)
    alt_a = _path_altitude_metrics(path_a, TerrainQuery(make_roi(np.full((10, 25), 1000.0))), cfg)
    alt_b = _path_altitude_metrics(path_b, TerrainQuery(make_roi(np.full((10, 25), 1000.0))), cfg)

    print(f"  Path A (zigzag):   vertical_motion={alt_a['total_vertical_motion_m']:.1f} "
          f"reversals={rev_a['vertical_reversal_count']} penalty={rev_a['total_reversal_penalty']:.2f}")
    print(f"  Path B (smoother): vertical_motion={alt_b['total_vertical_motion_m']:.1f} "
          f"reversals={rev_b['vertical_reversal_count']} penalty={rev_b['total_reversal_penalty']:.2f}")

    ok = (
        path_a[-1] == path_b[-1]
        and abs(alt_a["total_vertical_motion_m"] - alt_b["total_vertical_motion_m"]) < 1e-9
        and rev_a["vertical_reversal_count"] == 3 and rev_b["vertical_reversal_count"] == 1
        and rev_a["total_reversal_penalty"] > rev_b["total_reversal_penalty"]
    )
    print(f"  same endpoint, same total vertical motion, zigzag strictly more expensive: {'PASS' if ok else 'FAIL'}")
    return ok


def validation_f(cfg, primitives) -> bool:
    print()
    print("=== F: A* search behavior -- avoids reversals when some vertical motion is worthwhile ===")
    flat = np.full((3, 55), 1000.0)
    roi = make_roi(flat)
    tq = TerrainQuery(roi)
    z0 = msl_to_z_index(1400.0, cfg)
    start, goal = (1, 2, z0), (1, 52, z0)
    c = dataclasses.replace(cfg, msl_cost_weight=0.25)  # same scenario shape as Stage 9/10 Scenario B

    result = astar_search(start, goal, tq, min_search_altitude_msl=1300.0, max_search_altitude_msl=1400.0,
                           config=c, primitives=primitives, max_expansions=100_000)
    report(f"msl={c.msl_cost_weight}, reversal_weight={c.vertical_reversal_cost_weight}", result)

    ok = result.success and result.vertical_reversal_count == 0
    print(f"  chose a route with vertical_reversal_count={result.vertical_reversal_count} "
          f"(dove to save MSL, but never flip-flopped): {'PASS' if ok else 'FAIL'}")
    return ok


def validation_g(cfg, primitives) -> bool:
    print()
    print("=== G: low-MSL + continuous descent (the original motivating problem) ===")
    flat = np.full((3, 60), 1000.0)
    roi = make_roi(flat)
    tq = TerrainQuery(roi)
    z_top = msl_to_z_index(1400.0, cfg)
    start, goal = (1, 2, z_top), (1, 2, msl_to_z_index(1300.0, cfg))  # same xy, purely a descent

    ok_any = False
    for w_msl in (0.025, 0.05):
        c = dataclasses.replace(cfg, msl_cost_weight=w_msl)
        result = astar_search(start, goal, tq, min_search_altitude_msl=1300.0, max_search_altitude_msl=1400.0,
                               config=c, primitives=primitives, max_expansions=50_000)
        report(f"msl={w_msl}", result)
        if result.success and result.vertical_reversal_count == 0 and result.total_descent_m > 0:
            ok_any = True
            print(f"  w_MSL={w_msl}: smooth continuous descent achieved, 0 reversals, "
                  f"descent={result.total_descent_m:.1f}m  PASS")
            break
        else:
            print(f"  w_MSL={w_msl}: reversals={result.vertical_reversal_count if result.success else 'n/a'} "
                  f"-- trying next weight" if w_msl == 0.025 else "  no further weight to try")

    print(f"  Overall G: {'PASS' if ok_any else 'FAIL'}")
    return ok_any


def validation_h(cfg, primitives) -> bool:
    print()
    print("=== H: real Aladaglar ROI check (same scenario as Stage 9/10/11) ===")
    roi = load_roi(cfg)
    tq = TerrainQuery(roi)

    row_idx, c_start, c_goal = 80, 90, 128
    seg = roi.elevation[row_idx, c_start:c_goal + 1]
    seg_min, seg_max = float(seg.min()), float(seg.max())
    cruise_msl = math.ceil((seg_max + cfg.min_agl_m) / cfg.z_step_m) * cfg.z_step_m + 20.0
    min_search = math.ceil((seg_min + cfg.min_agl_m) / cfg.z_step_m) * cfg.z_step_m
    max_search = cruise_msl + 20.0
    z0 = msl_to_z_index(cruise_msl, cfg)
    start, goal = (row_idx, c_start, z0), (row_idx, c_goal, z0)
    print(f"  segment terrain min={seg_min:.1f}m max={seg_max:.1f}m, cruise_msl={cruise_msl:.0f}m, "
          f"bounds=[{min_search:.0f},{max_search:.0f}]m -- PROTOTYPE scenario, not a real mission altitude/route.")

    ok = True
    for label, w_msl in (("Case A", 0.0), ("Case B", 0.025)):
        c = dataclasses.replace(cfg, msl_cost_weight=w_msl)
        t0 = time.perf_counter()
        r = astar_search(start, goal, tq, min_search_altitude_msl=min_search, max_search_altitude_msl=max_search,
                          config=c, primitives=primitives, max_expansions=150_000)
        dt = time.perf_counter() - t0
        report(f"{label} (msl={w_msl}) [wall={dt:.1f}s]", r)
        if not (r.success and r.minimum_observed_agl >= cfg.min_agl_m - 1e-6):
            ok = False
    print(f"  PASS" if ok else "  FAIL")
    return ok


def main() -> None:
    cfg = DEFAULT_CONFIG
    assert cfg.vertical_reversal_cost_weight == 1.0
    primitives = build_primitive_set(cfg)

    results = [
        validations_a_to_d(cfg, primitives),
        validation_e(cfg),
        validation_f(cfg, primitives),
        validation_g(cfg, primitives),
        validation_h(cfg, primitives),
    ]

    print()
    print(f"Overall: {'ALL PASS' if all(results) else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
