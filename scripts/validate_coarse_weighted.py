"""Stage 35.2: Coarse Weighted A* Expansion Reduction -- ONLY the open-heap
priority (f = g + epsilon*h) changes; state, grid, primitives, safety
(precomputed cache reused, built ONCE), cost, heuristic itself, and
start/goal surrogate policy are all untouched from Stage 35/35.1.

epsilon=1.0 is NOT re-run -- Stage 35.1's recorded baseline (project.md)
is used directly. epsilon in {1.10, 1.25, 1.50} each get exactly one run.
Only the best epsilon (per the stated preference criteria) gets a fine-DEM
replay.
"""
import math
import time

from planner.astar import state_to_xyz
from planner.coarse import build_coarse_dem
from planner.coarse_astar import (
    coarse_astar_search, compute_coarse_distance_reference, lift_endpoint_if_unsafe,
    precompute_coarse_primitive_safety,
)
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from scripts.validate_coarse_astar import COARSE_CONFIG, fine_replay

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
EPSILON_VALUES = [1.10, 1.25, 1.50]

# Stage 35.1's recorded baseline (project.md) -- epsilon=1.0, NOT re-run.
BASELINE = {
    "epsilon": 1.0, "status": "success", "expanded_nodes": 24_959, "search_runtime_s": 4.70,
    "max_open_size": 12_348, "path_node_count": 31, "xy_length_m": 6480.0, "length_3d_m": 6541.9,
    "min_msl": 3360.0, "mean_msl": 3545.8, "max_msl": 3800.0,
    "total_climb_m": 440.0, "total_descent_m": 400.0, "total_cost": 1.409649,
    "min_coarse_max_agl_m": 200.0, "max_angle_deg": 8.43,
}


def path_metrics(path, coarse_terrain):
    xyz = [state_to_xyz(s, coarse_terrain, COARSE_CONFIG) for s in path]
    xy_length = sum(math.hypot(xyz[i + 1][0] - xyz[i][0], xyz[i + 1][1] - xyz[i][1]) for i in range(len(xyz) - 1))
    length_3d = sum(math.sqrt((xyz[i + 1][0] - xyz[i][0]) ** 2 + (xyz[i + 1][1] - xyz[i][1]) ** 2
                               + (xyz[i + 1][2] - xyz[i][2]) ** 2) for i in range(len(xyz) - 1))
    msl_values = [p[2] for p in xyz]
    total_climb = sum(max(0.0, xyz[i + 1][2] - xyz[i][2]) for i in range(len(xyz) - 1))
    total_descent = sum(max(0.0, xyz[i][2] - xyz[i + 1][2]) for i in range(len(xyz) - 1))
    return {
        "xyz": xyz, "xy_length_m": xy_length, "length_3d_m": length_3d,
        "min_msl": min(msl_values), "mean_msl": sum(msl_values) / len(msl_values), "max_msl": max(msl_values),
        "total_climb_m": total_climb, "total_descent_m": total_descent,
    }


def min_coarse_max_agl(path, coarse_terrain):
    from planner.primitives import MotionPrimitive, evaluate_primitive
    min_agl = math.inf
    max_angle = 0.0
    for (r1, c1, z1), (r2, c2, z2) in zip(path, path[1:]):
        p1 = state_to_xyz((r1, c1, z1), coarse_terrain, COARSE_CONFIG)
        dz_m = state_to_xyz((r2, c2, z2), coarse_terrain, COARSE_CONFIG)[2] - p1[2]
        horiz = math.hypot((c2 - c1) * COARSE_CONFIG.xy_resolution_m, (r2 - r1) * COARSE_CONFIG.xy_resolution_m)
        prim = MotionPrimitive("_", r2 - r1, c2 - c1, dz_m, horiz, "_")
        result = evaluate_primitive(p1, prim, coarse_terrain, COARSE_CONFIG)
        if result.valid:
            min_agl = min(min_agl, result.min_agl_m)
        angle = math.degrees(math.atan2(abs(dz_m), horiz)) if horiz > 0 else 0.0
        max_angle = max(max_angle, angle)
    return min_agl, max_angle


def main() -> None:
    fine_cfg = DEFAULT_CONFIG
    fine_roi = load_roi(fine_cfg)
    fine_terrain = TerrainQuery(fine_roi)
    coarse_result = build_coarse_dem(fine_roi, factor=3)
    coarse_terrain = TerrainQuery(coarse_result.roi)
    prims = build_primitive_set(COARSE_CONFIG)

    print("=== Building precompute cache ONCE (reused for every epsilon) ===")
    t0 = time.perf_counter()
    cache, precomp_stats = precompute_coarse_primitive_safety(coarse_terrain, prims, COARSE_CONFIG)
    print(f"  entries={precomp_stats.entry_count} preprocessing_time={precomp_stats.preprocessing_runtime_s:.3f}s "
          f"static_invalid={precomp_stats.static_invalid_count}")
    print()

    start_lift = lift_endpoint_if_unsafe(START_COARSE[0], START_COARSE[1], ORIGINAL_MSL, coarse_terrain, COARSE_CONFIG)
    goal_lift = lift_endpoint_if_unsafe(GOAL_COARSE[0], GOAL_COARSE[1], ORIGINAL_MSL, coarse_terrain, COARSE_CONFIG)
    start = (start_lift.row, start_lift.col, start_lift.z_index)
    goal = (goal_lift.row, goal_lift.col, goal_lift.z_index)

    seg = coarse_result.roi.elevation[min(START_COARSE[0], GOAL_COARSE[0]):max(START_COARSE[0], GOAL_COARSE[0]) + 1,
                                       START_COARSE[1]]
    seg_min = float(seg.min())
    min_search = math.ceil((seg_min + COARSE_CONFIG.min_agl_m) / COARSE_CONFIG.z_step_m) * COARSE_CONFIG.z_step_m
    max_search = max(start_lift.msl, goal_lift.msl) + 2 * COARSE_CONFIG.z_step_m
    d_ref = compute_coarse_distance_reference(start, goal, coarse_terrain, COARSE_CONFIG)

    rows = []
    best = None
    for eps in EPSILON_VALUES:
        print(f"=== epsilon_search={eps} ===")
        t0 = time.perf_counter()
        result = coarse_astar_search(
            start, goal, coarse_terrain, min_search, max_search, COARSE_CONFIG,
            primitives=prims, max_expansions=MAX_EXPANSIONS, distance_reference_m=d_ref,
            altitude_reference_msl=ALTITUDE_REFERENCE_MSL, altitude_scale_m=ALTITUDE_SCALE_M,
            w_distance=W_DISTANCE, w_altitude=W_ALTITUDE, precomputed_safety=cache, epsilon_search=eps,
        )
        wall = time.perf_counter() - t0

        row = {"epsilon": eps, "status": result.status, "expanded": result.expanded_nodes,
               "runtime_s": wall, "max_open": result.max_open_size, "cost": result.total_cost}
        print(f"  status={result.status} expanded={result.expanded_nodes} runtime={wall:.3f}s "
              f"max_open={result.max_open_size}")

        if result.success:
            pm = path_metrics(result.path, coarse_terrain)
            min_agl, max_angle = min_coarse_max_agl(result.path, coarse_terrain)
            cost_delta_pct = (result.total_cost / BASELINE["total_cost"] - 1.0) * 100.0
            print(f"  path_nodes={len(result.path)} xy_length={pm['xy_length_m']:.1f} "
                  f"3d_length={pm['length_3d_m']:.1f}")
            print(f"  min_MSL={pm['min_msl']:.1f} mean_MSL={pm['mean_msl']:.1f} max_MSL={pm['max_msl']:.1f}")
            print(f"  total_climb={pm['total_climb_m']:.1f} total_descent={pm['total_descent_m']:.1f}")
            print(f"  cost={result.total_cost:.6f}  cost_delta_vs_eps1.0={cost_delta_pct:+.2f}%")
            print(f"  min_coarse_MAX_AGL={min_agl:.1f}  max_flight_path_angle={max_angle:.2f}")
            row.update({"path_nodes": len(result.path), "xy_length": pm["xy_length_m"],
                        "cost_delta_pct": cost_delta_pct, "min_agl": min_agl, "path": result.path})
        else:
            print("  FAIL -- no path metrics.")
            row.update({"path_nodes": None, "cost_delta_pct": None, "min_agl": None, "path": None})
        rows.append(row)
        print()

    print("=== Selecting best epsilon (safety guaranteed regardless; then success; "
          "expansions<5000; runtime<=~1s; cost not badly degraded) ===")
    candidates = [r for r in rows if r["status"] == "success"]
    for r in candidates:
        print(f"  eps={r['epsilon']}: expanded={r['expanded']} runtime={r['runtime_s']:.3f}s "
              f"cost_delta={r['cost_delta_pct']:+.2f}%")
    if candidates:
        # Prefer expansions<5000 AND runtime<=1.0s among successes; tie-break by lowest cost_delta.
        strong = [r for r in candidates if r["expanded"] < 5000 and r["runtime_s"] <= 1.5]
        pool = strong if strong else candidates
        best = min(pool, key=lambda r: r["cost_delta_pct"])
        print(f"  >> BEST: epsilon={best['epsilon']} (expanded={best['expanded']}, "
              f"runtime={best['runtime_s']:.3f}s, cost_delta={best['cost_delta_pct']:+.2f}%)")
    else:
        print("  No epsilon succeeded.")

    print()
    print("=== COMPARISON TABLE ===")
    header = f"{'eps':>5} {'expansions':>10} {'runtime':>8} {'cost':>10} {'cost_delta':>11} {'path':>6} {'status':>9}"
    print(header)
    b = BASELINE
    print(f"{b['epsilon']:>5} {b['expanded_nodes']:>10} {b['search_runtime_s']:>7.2f}s {b['total_cost']:>10.6f} "
          f"{'baseline':>11} {b['path_node_count']:>6} {'success':>9}")
    for r in rows:
        cost_str = f"{r['cost']:.6f}" if r["status"] == "success" else "nan"
        delta_str = f"{r['cost_delta_pct']:+.2f}%" if r["cost_delta_pct"] is not None else "n/a"
        path_str = str(r["path_nodes"]) if r["path_nodes"] is not None else "n/a"
        print(f"{r['epsilon']:>5} {r['expanded']:>10} {r['runtime_s']:>7.2f}s {cost_str:>10} "
              f"{delta_str:>11} {path_str:>6} {r['status']:>9}")

    if best is not None and best["path"] is not None:
        print()
        print(f"=== FINE REPLAY for the BEST epsilon ({best['epsilon']}) ONLY ===")
        xyz = [state_to_xyz(s, coarse_terrain, COARSE_CONFIG) for s in best["path"]]
        replay = fine_replay(xyz, fine_terrain, fine_cfg, sample_spacing_m=10.0)
        print(f"  min_AGL_m={replay['min_agl_m']:.2f}  max_flight_path_angle_deg={replay['max_angle_deg']:.2f}  "
              f"violations={len(replay['violations'])}")
        replay_ok = len(replay["violations"]) == 0 and replay["min_agl_m"] >= COARSE_CONFIG.min_agl_m
        print(f"  {'PASS' if replay_ok else 'FAIL'}")


if __name__ == "__main__":
    main()
