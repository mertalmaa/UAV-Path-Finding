"""Stage 35.1: Coarse Primitive Safety Precomputation -- speeds up ONLY
the safety-check step inside the Stage 35 coarse A* (evaluate_primitive's
per-sample terrain query replaced by an O(1) precomputed-cache lookup).
Search algorithm, state, primitives, cost, heuristic, epsilon=1.0, cap,
and endpoint policy are all UNCHANGED from Stage 35 -- see planner.
coarse_astar.precompute_coarse_primitive_safety's docstring for the exact
equivalence argument this script empirically verifies.
"""
import dataclasses
import math
import random

import numpy as np
from affine import Affine

from planner.astar import state_to_xyz
from planner.coarse import build_coarse_dem
from planner.coarse_astar import (
    coarse_astar_search, compute_coarse_distance_reference, lift_endpoint_if_unsafe,
    precompute_coarse_primitive_safety, precomputed_primitive_validity,
)
from planner.primitives import build_primitive_set, evaluate_primitive
from planner.roi import ROIData, load_roi
from planner.terrain import TerrainQuery
from scripts.validate_coarse_astar import COARSE_CONFIG, fine_replay

NODATA = -9999.0
START_ROW, START_COL = 48, 276
GOAL_ROW, GOAL_COL = 264, 276
START_COARSE = (16, 92)
GOAL_COARSE = (88, 92)
ORIGINAL_MSL = 3760.0
MAX_EXPANSIONS = 30_000
ALTITUDE_REFERENCE_MSL = 3240.0
ALTITUDE_SCALE_M = 1000.0
W_DISTANCE = 1.0
W_ALTITUDE = 1.25

# Stage 35's recorded baseline (project.md) -- NOT re-derived, only compared against.
STAGE35_BASELINE = {
    "status": "success", "expanded_nodes": 24_959, "runtime_s": 247.76, "path_node_count": 31,
    "xy_length_m": 6480.0, "min_msl": 3360.0, "max_msl": 3800.0, "min_coarse_max_agl_m": 200.0,
    "fine_replay_min_agl_m": 202.31, "fine_replay_violations": 0, "total_cost": 1.409649,
}


def make_synthetic_roi(elevation: np.ndarray, nodata=NODATA, res: float = 90.0) -> ROIData:
    height, width = elevation.shape
    transform = Affine(res, 0.0, 500000.0, 0.0, -res, 4200000.0)
    return ROIData(
        elevation=elevation.astype(np.float32), transform=transform, crs="EPSG:32636",
        width=width, height=height, bounds=(500000.0, 4200000.0 - height * res, 500000.0 + width * res, 4200000.0),
        resolution=(res, res), nodata=nodata,
    )


def _reference_validity(terrain, prim, row, col, start_msl, config):
    start_xyz = (*terrain.rowcol_to_xy(row, col), start_msl)
    result = evaluate_primitive(start_xyz, prim, terrain, config)
    return result.valid, result.reason


def _compare_all(terrain, primitives, cache, config, start_msls, label) -> int:
    """Compare old (evaluate_primitive) vs new (precomputed cache) validity
    for every (row, col, primitive, start_msl) combination in range.
    Returns mismatch count."""
    mismatches = 0
    checked = 0
    for row in range(terrain.roi.height):
        for col in range(terrain.roi.width):
            for prim in primitives:
                for start_msl in start_msls:
                    checked += 1
                    old_valid, old_reason = _reference_validity(terrain, prim, row, col, start_msl, config)
                    new_valid, new_reason = precomputed_primitive_validity(cache, row, col, prim, start_msl)
                    if old_valid != new_valid:
                        mismatches += 1
                        if mismatches <= 5:
                            print(f"    MISMATCH row={row} col={col} prim={prim.direction}/{prim.primitive_type} "
                                  f"start_msl={start_msl}: old=({old_valid},{old_reason}) new=({new_valid},{new_reason})")
    print(f"  [{label}] checked={checked} mismatches={mismatches}")
    return mismatches


def test_1_flat() -> bool:
    print("=== 1: synthetic flat terrain, old==new ===")
    elev = np.full((10, 10), 1000.0)
    tq = TerrainQuery(make_synthetic_roi(elev))
    prims = build_primitive_set(COARSE_CONFIG)
    cache, _ = precompute_coarse_primitive_safety(tq, prims, COARSE_CONFIG)
    mismatches = _compare_all(tq, prims, cache, COARSE_CONFIG, [1100.0, 1200.0, 1300.0], "flat")
    return mismatches == 0


def test_2_ridge() -> bool:
    print()
    print("=== 2: mid-segment ridge, old==new ===")
    elev = np.full((6, 6), 1000.0)
    elev[2, 2] = 1250.0
    tq = TerrainQuery(make_synthetic_roi(elev))
    prims = build_primitive_set(COARSE_CONFIG)
    cache, _ = precompute_coarse_primitive_safety(tq, prims, COARSE_CONFIG)
    mismatches = _compare_all(tq, prims, cache, COARSE_CONFIG, [1150.0, 1300.0, 1450.0], "ridge")
    return mismatches == 0


def test_3_nodata() -> bool:
    print()
    print("=== 3: NoData crossing, old==new ===")
    elev = np.full((6, 6), 1000.0)
    elev[3, 3] = NODATA
    tq = TerrainQuery(make_synthetic_roi(elev, nodata=NODATA))
    prims = build_primitive_set(COARSE_CONFIG)
    cache, _ = precompute_coarse_primitive_safety(tq, prims, COARSE_CONFIG)
    mismatches = _compare_all(tq, prims, cache, COARSE_CONFIG, [1150.0, 1300.0], "nodata")
    return mismatches == 0


def test_4_bounds() -> bool:
    print()
    print("=== 4: bounds crossing (edge of a small grid), old==new ===")
    elev = np.full((4, 4), 1000.0)
    tq = TerrainQuery(make_synthetic_roi(elev))
    prims = build_primitive_set(COARSE_CONFIG)
    cache, _ = precompute_coarse_primitive_safety(tq, prims, COARSE_CONFIG)
    mismatches = _compare_all(tq, prims, cache, COARSE_CONFIG, [1150.0, 1300.0], "bounds")
    return mismatches == 0


def test_5_types() -> bool:
    print()
    print("=== 5: level/climb/descent all match ===")
    elev = np.random.RandomState(42).uniform(2900.0, 3100.0, size=(8, 8))
    tq = TerrainQuery(make_synthetic_roi(elev))
    prims = build_primitive_set(COARSE_CONFIG)
    cache, _ = precompute_coarse_primitive_safety(tq, prims, COARSE_CONFIG)
    all_ok = True
    for kind in ("level", "climb", "descent"):
        kind_prims = [p for p in prims if p.primitive_type == kind]
        mismatches = _compare_all(tq, kind_prims, cache, COARSE_CONFIG, [3050.0, 3150.0, 3250.0], kind)
        all_ok = all_ok and mismatches == 0
    return all_ok


def test_6_start_msl_levels() -> bool:
    print()
    print("=== 6: many different start MSL levels, old/new validity identical ===")
    elev = np.random.RandomState(7).uniform(3000.0, 3400.0, size=(8, 8))
    tq = TerrainQuery(make_synthetic_roi(elev))
    prims = build_primitive_set(COARSE_CONFIG)
    cache, _ = precompute_coarse_primitive_safety(tq, prims, COARSE_CONFIG)
    start_msls = [3200.0 + 40.0 * i for i in range(15)]  # 3200..3760 step 40
    mismatches = _compare_all(tq, prims, cache, COARSE_CONFIG, start_msls, "multi-level")
    return mismatches == 0


def test_7_real_random(fine_roi) -> bool:
    print()
    print("=== 7: real coarse ROI, random states/primitives, broad comparison ===")
    coarse_result = build_coarse_dem(fine_roi, factor=3)
    tq = TerrainQuery(coarse_result.roi)
    prims = build_primitive_set(COARSE_CONFIG)
    cache, stats = precompute_coarse_primitive_safety(tq, prims, COARSE_CONFIG)

    rng = random.Random(123)
    mismatches = 0
    n_checks = 4000
    for _ in range(n_checks):
        row = rng.randrange(tq.roi.height)
        col = rng.randrange(tq.roi.width)
        prim = rng.choice(prims)
        start_msl = rng.choice([3280.0 + 40.0 * i for i in range(16)])
        old_valid, old_reason = _reference_validity(tq, prim, row, col, start_msl, COARSE_CONFIG)
        new_valid, new_reason = precomputed_primitive_validity(cache, row, col, prim, start_msl)
        if old_valid != new_valid:
            mismatches += 1
            if mismatches <= 5:
                print(f"    MISMATCH row={row} col={col} prim={prim.direction}/{prim.primitive_type} "
                      f"start_msl={start_msl}: old=({old_valid},{old_reason}) new=({new_valid},{new_reason})")
    print(f"  random checks={n_checks} mismatches={mismatches}")
    print(f"  precompute: entries={stats.entry_count} preprocessing_time={stats.preprocessing_runtime_s:.3f}s "
          f"static_invalid={stats.static_invalid_count} approx_memory_MB={stats.approx_memory_bytes / 1e6:.2f}")
    return mismatches == 0


def main() -> None:
    results = [
        test_1_flat(), test_2_ridge(), test_3_nodata(), test_4_bounds(),
        test_5_types(), test_6_start_msl_levels(),
    ]

    fine_cfg_default = None
    from planner.config import DEFAULT_CONFIG
    fine_cfg_default = DEFAULT_CONFIG
    fine_roi = load_roi(fine_cfg_default)
    results.append(test_7_real_random(fine_roi))

    print()
    print(f"Equivalence tests: {'ALL PASS (0 mismatches everywhere)' if all(results) else 'SOME FAILED -- mismatch found'}")
    if not all(results):
        print("Precomputation does NOT match the original evaluator -- stopping before the real benchmark.")
        return

    print()
    print("=" * 70)
    print("=== REAL 6.48km COARSE BENCHMARK (precomputed safety) ===")
    fine_terrain = TerrainQuery(fine_roi)
    coarse_result = build_coarse_dem(fine_roi, factor=3)
    coarse_terrain = TerrainQuery(coarse_result.roi)
    prims = build_primitive_set(COARSE_CONFIG)

    cache, precomp_stats = precompute_coarse_primitive_safety(coarse_terrain, prims, COARSE_CONFIG)
    print(f"  precompute: entries={precomp_stats.entry_count} "
          f"preprocessing_time={precomp_stats.preprocessing_runtime_s:.3f}s "
          f"static_invalid_count={precomp_stats.static_invalid_count} "
          f"approx_memory_MB={precomp_stats.approx_memory_bytes / 1e6:.2f}")

    start_lift = lift_endpoint_if_unsafe(START_COARSE[0], START_COARSE[1], ORIGINAL_MSL, coarse_terrain, COARSE_CONFIG)
    goal_lift = lift_endpoint_if_unsafe(GOAL_COARSE[0], GOAL_COARSE[1], ORIGINAL_MSL, coarse_terrain, COARSE_CONFIG)
    start = (start_lift.row, start_lift.col, start_lift.z_index)
    goal = (goal_lift.row, goal_lift.col, goal_lift.z_index)
    print(f"  START -> msl={start_lift.msl} (lifted={start_lift.was_lifted})  "
          f"GOAL -> msl={goal_lift.msl} (lifted={goal_lift.was_lifted})")

    seg = coarse_result.roi.elevation[min(START_COARSE[0], GOAL_COARSE[0]):max(START_COARSE[0], GOAL_COARSE[0]) + 1,
                                       START_COARSE[1]]
    seg_min = float(seg.min())
    min_search = math.ceil((seg_min + COARSE_CONFIG.min_agl_m) / COARSE_CONFIG.z_step_m) * COARSE_CONFIG.z_step_m
    max_search = max(start_lift.msl, goal_lift.msl) + 2 * COARSE_CONFIG.z_step_m
    d_ref = compute_coarse_distance_reference(start, goal, coarse_terrain, COARSE_CONFIG)
    print(f"  search bounds=[{min_search},{max_search}]  D_ref={d_ref:.2f}m  cap={MAX_EXPANSIONS} (1 run, no retry)")
    print()

    import time
    t0 = time.perf_counter()
    result = coarse_astar_search(
        start, goal, coarse_terrain, min_search, max_search, COARSE_CONFIG,
        primitives=prims, max_expansions=MAX_EXPANSIONS, distance_reference_m=d_ref,
        altitude_reference_msl=ALTITUDE_REFERENCE_MSL, altitude_scale_m=ALTITUDE_SCALE_M,
        w_distance=W_DISTANCE, w_altitude=W_ALTITUDE, precomputed_safety=cache,
    )
    search_wall = time.perf_counter() - t0
    total_wall = precomp_stats.preprocessing_runtime_s + search_wall

    print(f"  status={result.status}  search_runtime={result.runtime_s:.3f}s  "
          f"expanded={result.expanded_nodes}  max_open={result.max_open_size}")
    print(f"  preprocessing_time={precomp_stats.preprocessing_runtime_s:.3f}s  "
          f"search_time={search_wall:.3f}s  total_time={total_wall:.3f}s")
    speedup = STAGE35_BASELINE["runtime_s"] / total_wall if total_wall > 0 else float("nan")
    print(f"  Stage 35 baseline runtime={STAGE35_BASELINE['runtime_s']:.2f}s  "
          f"speedup={speedup:.2f}x  expansions/sec={result.expanded_nodes / search_wall:.1f}")

    if not result.success:
        print()
        print("FAIL -- does not match Stage 35 baseline (which was success). NOT counting this as PASS.")
        return

    path = result.path
    xyz = [state_to_xyz(s, coarse_terrain, COARSE_CONFIG) for s in path]
    xy_length = sum(math.hypot(xyz[i + 1][0] - xyz[i][0], xyz[i + 1][1] - xyz[i][1]) for i in range(len(xyz) - 1))
    msl_values = [p[2] for p in xyz]

    print()
    print("=== Comparison vs Stage 35 baseline ===")
    b = STAGE35_BASELINE
    same_expansions = result.expanded_nodes == b["expanded_nodes"]
    same_path_len = len(path) == b["path_node_count"]
    same_xy_length = abs(xy_length - b["xy_length_m"]) < 1e-6
    same_min_msl = abs(min(msl_values) - b["min_msl"]) < 1e-6
    same_max_msl = abs(max(msl_values) - b["max_msl"]) < 1e-6
    same_cost = abs(result.total_cost - b["total_cost"]) < 1e-4
    print(f"  expanded: new={result.expanded_nodes} baseline={b['expanded_nodes']} identical={same_expansions}")
    print(f"  path_node_count: new={len(path)} baseline={b['path_node_count']} identical={same_path_len}")
    print(f"  xy_length_m: new={xy_length:.2f} baseline={b['xy_length_m']} identical={same_xy_length}")
    print(f"  min_MSL: new={min(msl_values):.1f} baseline={b['min_msl']} identical={same_min_msl}")
    print(f"  max_MSL: new={max(msl_values):.1f} baseline={b['max_msl']} identical={same_max_msl}")
    print(f"  total_cost: new={result.total_cost:.6f} baseline={b['total_cost']} identical={same_cost}")
    behavior_preserved = same_expansions and same_path_len and same_xy_length and same_min_msl and same_max_msl and same_cost
    print(f"  BEHAVIOR PRESERVED: {behavior_preserved}")

    print()
    print("=== FINE REPLAY (30m DEM, <=10m sampling) ===")
    replay = fine_replay(xyz, fine_terrain, fine_cfg_default, sample_spacing_m=10.0)
    print(f"  min_AGL_m={replay['min_agl_m']:.2f}  max_angle_deg={replay['max_angle_deg']:.2f}  "
          f"violations={len(replay['violations'])}")
    same_fine_replay = len(replay["violations"]) == 0 and abs(replay["min_agl_m"] - b["fine_replay_min_agl_m"]) < 1e-2
    print(f"  fine replay matches Stage 35 baseline: {same_fine_replay}")

    print()
    overall = behavior_preserved and same_fine_replay
    print(f"Overall Stage 35.1: {'PASS' if overall else 'FAIL'}")


if __name__ == "__main__":
    main()
