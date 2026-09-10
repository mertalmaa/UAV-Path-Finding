"""Stage 37.3: History-Free Fine Weighted A* Sweep -- epsilon in
{1.30, 1.50, 1.70}, all with freeze_history=True (Stage 37.2), on top of
the same 3D corridor (XY +/-300m, Z +/-200m) and normalized cost as
Stage 37.1/37.2. Stage 37.2's own epsilon=1.10 baseline (project.md) is
NOT re-run. A single primitive_cache dict is built ONCE and reused
(external_primitive_cache) across all three runs.
"""
import csv
import dataclasses
import math
import os
import time

import numpy as np

from planner.astar import (
    _path_altitude_metrics, _path_min_observed_agl, _path_vertical_reversal_metrics,
    astar_search, compute_distance_reference, msl_to_z_index, state_to_xyz, validate_and_cost_path,
)
from planner.config import DEFAULT_CONFIG
from planner.corridor import build_z_guide_grid
from planner.primitives import build_primitive_set
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from scripts.benchmark_long_valley import verify_path_safety
from scripts.calibrate_low_msl_behavior import ascii_profile, compute_vertical_profile_metrics
from scripts.validate_cost_function_ranking import build_direct_level

START_ROW, START_COL = 48, 276
GOAL_ROW, GOAL_COL = 264, 276
AIRCRAFT_MSL = 3760.0
ALTITUDE_REFERENCE_MSL = 3240.0
NORMALIZED_ALTITUDE_SCALE_M = 1000.0
NORMALIZED_W_ALTITUDE = 1.25
NORMALIZED_W_DISTANCE = 1.0
NORMALIZED_W_REVERSAL = 1.0
MAX_EXPANSIONS = 30_000
TARGET_SUBOPTIMALITY = 1.05
GOAL_TOLERANCE_XY_M = 35.0
GOAL_TOLERANCE_Z_M = 25.0
XY_MASK_NPY = "outputs/stage36_corridor_mask.npy"
COARSE_PATH_CSV = "outputs/stage36_coarse_path_eps1.5.csv"
Z_TOLERANCE_M = 200.0
EPSILON_VALUES = [1.30, 1.50, 1.70]
OUTPUT_CSV = "outputs/stage37_3_best_path.csv"

# Stage 37.2's own recorded real-benchmark result (project.md, epsilon=1.10) -- NOT re-run.
STAGE37_2_REFERENCE = {
    "epsilon": 1.10, "status": "search_limit_reached", "expanded_nodes": 30_000, "wall_s": 132.33,
    "closest_distance_to_goal_region_m": 1499.49, "reopened_states": 463, "cache_hit_rate": 0.013,
}


def main() -> None:
    cfg = dataclasses.replace(
        DEFAULT_CONFIG,
        cost_mode="normalized",
        altitude_reference_msl=ALTITUDE_REFERENCE_MSL,
        normalized_altitude_scale_m=NORMALIZED_ALTITUDE_SCALE_M,
        normalized_w_distance=NORMALIZED_W_DISTANCE,
        normalized_w_altitude=NORMALIZED_W_ALTITUDE,
        normalized_w_reversal=NORMALIZED_W_REVERSAL,
        goal_tolerance_xy_m=GOAL_TOLERANCE_XY_M,
        goal_tolerance_z_m=GOAL_TOLERANCE_Z_M,
    )
    primitives = build_primitive_set(cfg)
    roi = load_roi(cfg)
    tq = TerrainQuery(roi)
    xy_mask = np.load(XY_MASK_NPY)

    coarse_path_xyz = []
    with open(COARSE_PATH_CSV) as f:
        for row in csv.DictReader(f):
            coarse_path_xyz.append((float(row["x"]), float(row["y"]), float(row["z_msl"])))
    z_guide_grid = build_z_guide_grid(roi, coarse_path_xyz)

    z0 = msl_to_z_index(AIRCRAFT_MSL, cfg)
    start, goal = (START_ROW, START_COL, z0), (GOAL_ROW, GOAL_COL, z0)
    for r, c in [(START_ROW, START_COL), (GOAL_ROW, GOAL_COL)]:
        diff = abs(AIRCRAFT_MSL - float(z_guide_grid[r, c]))
        if not (bool(xy_mask[r, c]) and diff <= Z_TOLERANCE_M):
            z_guide_grid[r, c] = AIRCRAFT_MSL
            xy_mask[r, c] = True

    seg = roi.elevation[START_ROW:GOAL_ROW + 1, START_COL]
    seg_min = float(seg.min())
    min_search = math.ceil((seg_min + cfg.min_agl_m) / cfg.z_step_m) * cfg.z_step_m
    max_search = AIRCRAFT_MSL + 20.0
    d_ref = compute_distance_reference(start, goal, tq, cfg)

    direct_level_path = build_direct_level(z0)
    ok, incumbent_cost = validate_and_cost_path(direct_level_path, primitives, tq, cfg, d_ref)
    print(f"initial incumbent (direct 216-edge level chain): valid={ok} cost={incumbent_cost:.6f}")
    if not ok:
        print("!! Direct level path rejected -- aborting.")
        return

    shared_cache = {}
    print(f"Shared primitive_cache built once, reused across epsilon={EPSILON_VALUES}\n")

    rows = []
    for eps in EPSILON_VALUES:
        print(f"=== epsilon_search={eps} (freeze_history=True) ===")
        t0 = time.perf_counter()
        result = astar_search(
            start, goal, tq, min_search_altitude_msl=min_search, max_search_altitude_msl=max_search,
            config=cfg, primitives=primitives, max_expansions=MAX_EXPANSIONS,
            use_primitive_cache=True, use_dominance_pruning=False,
            use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False,
            use_incumbent_pruning=True, initial_incumbent_cost=incumbent_cost,
            initial_incumbent_path=direct_level_path,
            epsilon_search=eps, target_suboptimality=TARGET_SUBOPTIMALITY,
            corridor_mask=xy_mask, z_guide_grid=z_guide_grid, z_guide_tolerance_m=Z_TOLERANCE_M,
            freeze_history=True, external_primitive_cache=shared_cache,
        )
        wall = time.perf_counter() - t0
        reopen_ratio = result.reopened_states / result.expanded_nodes if result.expanded_nodes else float("nan")

        # IMPORTANT: result.status=="success" requires the FULL %5 certificate
        # (target_suboptimality) to have been proven -- a genuine, safe, COMPLETE
        # path can exist (first_solution_cost finite / incumbent updated) well
        # before that certificate is reached, and max_expansions can be hit before
        # the certificate completes even though a real solution is already in hand.
        # "found a real path" == first_solution_cost < inf, NOT result.success.
        first_found = result.first_solution_cost < math.inf
        effective_cost = result.final_incumbent_cost if first_found else None
        effective_path = result.incumbent_path if first_found else None

        row = {
            "epsilon": eps, "status": result.status, "expanded": result.expanded_nodes,
            "first_solution_expanded": result.first_solution_expanded if first_found else None,
            "wall_s": wall, "max_open": result.max_open_size, "reopened": result.reopened_states,
            "reopen_ratio": reopen_ratio, "cache_hit_rate": result.primitive_cache_hit_rate,
            "unique_xy": result.unique_expanded_xy, "unique_xyz": result.unique_expanded_xyz,
            "found_path": first_found, "cost": effective_cost, "path": effective_path,
            "certified": result.bounded_termination_triggered, "result": result,
        }
        rows.append(row)

        print(f"  status={result.status} expanded={result.expanded_nodes} "
              f"first_solution_expanded={row['first_solution_expanded']} wall={wall:.2f}s "
              f"max_open={result.max_open_size}")
        print(f"  reopened_states={result.reopened_states} reopen_ratio={reopen_ratio:.4f} "
              f"cache_hit_rate={result.primitive_cache_hit_rate:.3f} (shared cache size={len(shared_cache)})")
        print(f"  unique_expanded_xy={result.unique_expanded_xy} unique_expanded_xyz={result.unique_expanded_xyz} "
              f"avg_history_states_per_xyz={result.avg_history_states_per_xyz:.3f}")
        if first_found:
            min_agl = _path_min_observed_agl(effective_path, primitives, tq, cfg)
            safety = verify_path_safety(effective_path, primitives, tq, cfg)
            print(f"  COMPLETE PATH FOUND (certified={result.bounded_termination_triggered}): "
                  f"cost={effective_cost:.6f} path_nodes={len(effective_path)} "
                  f"min_AGL={min_agl:.1f} max_angle={safety.get('max_angle_deg', 'n/a')} "
                  f"incumbent_updates={result.incumbent_updates}")
        else:
            print(f"  NO PATH FOUND: closest_distance_to_goal_region_m={result.closest_distance_to_goal_region_m:.2f} "
                  f"closest_state={result.closest_state_to_goal}")
        print()

    print("=== COMPARISON TABLE ===")
    print("(NOTE: 'status' is the CERTIFIED (%5 bound) outcome -- 'found_path' is whether a real, safe, "
          "complete path was found at all, which can be True even when status stays search_limit_reached "
          "because the certificate itself wasn't proven before the cap. found_path is what matters here.)")
    print(f"{'eps':>5} {'status':>19} {'found_path':>10} {'expansions':>10} {'runtime':>9} "
          f"{'reopened':>9} {'cost':>12}")
    print(f"{STAGE37_2_REFERENCE['epsilon']:>5} {STAGE37_2_REFERENCE['status']:>19} {'False':>10} "
          f"{STAGE37_2_REFERENCE['expanded_nodes']:>10} {STAGE37_2_REFERENCE['wall_s']:>8.2f}s "
          f"{STAGE37_2_REFERENCE['reopened_states']:>9} {'n/a':>12}  (Stage 37.2 baseline, NOT re-run)")
    for r in rows:
        cost_str = f"{r['cost']:.6f}" if r["cost"] is not None else "n/a"
        print(f"{r['epsilon']:>5} {r['status']:>19} {str(r['found_path']):>10} {r['expanded']:>10} "
              f"{r['wall_s']:>8.2f}s {r['reopened']:>9} {cost_str:>12}")

    found = [r for r in rows if r["found_path"]]
    print()
    if not found:
        print("No epsilon in this sweep found a complete path -- no fine replay performed.")
        return

    if len(found) > 1:
        print("Cost deltas between candidates that found a path:")
        for i in range(len(found)):
            for j in range(i + 1, len(found)):
                a, b = found[i], found[j]
                delta_pct = (b["cost"] / a["cost"] - 1.0) * 100.0
                print(f"  eps={a['epsilon']} (cost={a['cost']:.6f}) vs eps={b['epsilon']} (cost={b['cost']:.6f}): "
                      f"{delta_pct:+.2f}%")

    # Pick the best: prefer fewer expansions (== found sooner), but reject if
    # reopen_ratio or cost is badly degraded relative to the cheapest candidate found.
    cheapest_cost = min(r["cost"] for r in found)
    candidates = []
    for r in found:
        cost_delta_pct = (r["cost"] / cheapest_cost - 1.0) * 100.0
        candidates.append((r, cost_delta_pct))
        print(f"  eps={r['epsilon']}: first_solution_expanded={r['first_solution_expanded']} "
              f"reopen_ratio={r['reopen_ratio']:.4f} cost_delta_vs_cheapest={cost_delta_pct:+.2f}%")

    # Reasonable acceptance bar: reopen_ratio not extreme (<0.98, all three runs here
    # thrash heavily regardless of epsilon -- see project.md) and cost_delta modest (<10%).
    acceptable = [(r, d) for r, d in candidates if r["reopen_ratio"] < 0.98 and d < 10.0]
    pool = acceptable if acceptable else candidates
    best_row, best_delta = min(pool, key=lambda rd: rd[0]["first_solution_expanded"])
    print(f"\n>> BEST CANDIDATE: epsilon={best_row['epsilon']} "
          f"(first_solution_expanded={best_row['first_solution_expanded']}, "
          f"reopen_ratio={best_row['reopen_ratio']:.4f}, cost_delta_vs_cheapest={best_delta:+.2f}%)")

    best_path = best_row["path"]
    print(f"\n=== FINE DEM REPLAY/VALIDATION for the BEST candidate (epsilon={best_row['epsilon']}) ONLY ===")
    safety = verify_path_safety(best_path, primitives, tq, cfg)
    min_agl = _path_min_observed_agl(best_path, primitives, tq, cfg)
    alt_metrics = _path_altitude_metrics(best_path, tq, cfg)
    reversal_metrics = _path_vertical_reversal_metrics(best_path, primitives, cfg)
    profile = compute_vertical_profile_metrics(best_path, primitives, tq, cfg)
    print(f"  path_node_count={len(best_path)}  3d_length={alt_metrics['geometric_path_length']:.1f}m")
    print(f"  min_MSL={alt_metrics['minimum_aircraft_msl']:.1f}  max_MSL={alt_metrics['maximum_aircraft_msl']:.1f}  "
          f"min_AGL={min_agl:.1f}")
    print(f"  total_climb={alt_metrics['total_climb_m']:.1f}  total_descent={alt_metrics['total_descent_m']:.1f}")
    print(f"  SAFETY CHECK: {'PASS' if safety['ok'] else 'FAIL -- ' + safety.get('reason', '')}  "
          f"(min_agl={safety.get('min_agl', 'n/a')}, max_primitive_angle={safety.get('max_angle_deg', 'n/a')})")
    print(f"  ascii: {ascii_profile(profile, profile['total_horizontal_distance_m'])}")

    os.makedirs("outputs", exist_ok=True)
    with open(OUTPUT_CSV, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "row", "col", "z_index", "x", "y", "z_msl"])
        for i, s in enumerate(best_path):
            x, y, zm = state_to_xyz(s, tq, cfg)
            writer.writerow([i, s[0], s[1], s[2], f"{x:.2f}", f"{y:.2f}", f"{zm:.2f}"])
    print(f"\nBest path written to {OUTPUT_CSV}")


if __name__ == "__main__":
    main()
