"""Stage 37.2: the ONE real 6.48km Aladaglar benchmark run with the fine
A*'s diagnostic-only freeze_history=True mode -- vertical_trend/trend_age
_bucket removed from the effective search state (frozen forever at their
initial values), reversal cost forced to 0.0. Every other setting is
IDENTICAL to Stage 37.1's own real run (3D corridor: XY +/-300m, Z
+/-200m, epsilon=1.10, normalized distance+altitude cost, primitive cache
ON, goal tolerance +/-35m XY/+/-25m Z, 30k cap). Stage 37.1's baseline
(project.md) is NOT re-run here. This is a diagnostic mode only -- NOT a
production default (freeze_history defaults to False everywhere else).
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
from scripts.benchmark_long_valley import last_climb_start_distance, verify_path_safety
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
EPSILON_SEARCH = 1.10
TARGET_SUBOPTIMALITY = 1.05
GOAL_TOLERANCE_XY_M = 35.0
GOAL_TOLERANCE_Z_M = 25.0
XY_MASK_NPY = "outputs/stage36_corridor_mask.npy"
COARSE_PATH_CSV = "outputs/stage36_coarse_path_eps1.5.csv"
Z_TOLERANCE_M = 200.0
OUTPUT_CSV = "outputs/stage37_2_history_free_path.csv"

# Stage 37.1's own recorded real-benchmark result (project.md) -- NOT re-run.
STAGE37_1_REFERENCE = {
    "status": "search_limit_reached", "expanded_nodes": 30_000, "wall_s": 37.80,
    "max_open_size": 34_435, "closest_distance_to_goal_region_m": 4918.11,
    "unique_expanded_xy": 1012, "unique_expanded_xyz": 7089, "unique_full_states": 30_000,
    "cache_hit_rate": 0.768,
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

    print(f"=== Stage 37.2 REAL Aladaglar benchmark -- history-free diagnostic (freeze_history=True) "
          f"(SINGLE run, cap={MAX_EXPANSIONS}, no retry) ===")
    print(f"  D_ref={d_ref:.4f}m  epsilon_search={EPSILON_SEARCH}\n")

    direct_level_path = build_direct_level(z0)
    ok, incumbent_cost = validate_and_cost_path(direct_level_path, primitives, tq, cfg, d_ref)
    print(f"initial incumbent (direct 216-edge level chain): valid={ok} cost={incumbent_cost:.6f}\n")
    if not ok:
        print("!! Direct level path rejected -- aborting.")
        return

    t0 = time.perf_counter()
    result = astar_search(
        start, goal, tq, min_search_altitude_msl=min_search, max_search_altitude_msl=max_search,
        config=cfg, primitives=primitives, max_expansions=MAX_EXPANSIONS,
        use_primitive_cache=True, use_dominance_pruning=False,
        use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False,
        use_incumbent_pruning=True, initial_incumbent_cost=incumbent_cost,
        initial_incumbent_path=direct_level_path,
        epsilon_search=EPSILON_SEARCH, target_suboptimality=TARGET_SUBOPTIMALITY,
        corridor_mask=xy_mask, z_guide_grid=z_guide_grid, z_guide_tolerance_m=Z_TOLERANCE_M,
        freeze_history=True,
    )
    wall = time.perf_counter() - t0

    print("=== SEARCH RESULT ===")
    print(f"  status={result.status}  wall={wall:.2f}s  runtime={result.runtime_s:.2f}s")
    print(f"  expanded={result.expanded_nodes}  max_open={result.max_open_size}  "
          f"cache_hit_rate={result.primitive_cache_hit_rate:.3f}  reopened_states={result.reopened_states}")
    print(f"  xy_corridor_reject_count={result.corridor_reject_count}  "
          f"z_corridor_reject_count={result.z_corridor_reject_count}")
    print()

    print("=== STATE-SPACE COMPOSITION DIAGNOSTICS ===")
    print(f"  unique_expanded_xy={result.unique_expanded_xy}")
    print(f"  unique_expanded_xyz={result.unique_expanded_xyz}")
    print(f"  unique_full_states={result.unique_full_states}")
    print(f"  avg_z_states_per_xy={result.avg_z_states_per_xy:.3f}")
    print(f"  avg_history_states_per_xyz={result.avg_history_states_per_xyz:.3f}  (expect exactly 1.0 -- no history)")
    print(f"  max_z_states_in_one_xy={result.max_z_states_in_one_xy}")
    print()

    print("=== COMPARISON vs Stage 37.1 baseline (NOT re-run) ===")
    b = STAGE37_1_REFERENCE
    print(f"  Stage 37.1: status={b['status']} expanded={b['expanded_nodes']} wall={b['wall_s']:.2f}s "
          f"max_open={b['max_open_size']} unique_xy={b['unique_expanded_xy']} "
          f"unique_xyz={b['unique_expanded_xyz']} full_states={b['unique_full_states']} "
          f"closest_goal_region={b['closest_distance_to_goal_region_m']:.2f}m cache_hit={b['cache_hit_rate']:.3f}")
    print(f"  Stage 37.2: status={result.status} expanded={result.expanded_nodes} wall={wall:.2f}s "
          f"max_open={result.max_open_size} unique_xy={result.unique_expanded_xy} "
          f"unique_xyz={result.unique_expanded_xyz} full_states={result.unique_full_states} "
          f"closest_goal_region={result.closest_distance_to_goal_region_m:.2f}m "
          f"cache_hit={result.primitive_cache_hit_rate:.3f}")
    print()

    if not result.success:
        print("=== FAIL -- no path/incumbent to report ===")
        cs = result.closest_state_to_goal
        print(f"  closest_state_to_goal={cs}  distance_to_goal_region_m={result.closest_distance_to_goal_region_m:.2f}")
        return

    report_path = result.path
    alt_metrics = _path_altitude_metrics(report_path, tq, cfg)
    min_agl = _path_min_observed_agl(report_path, primitives, tq, cfg)
    reversal_metrics = _path_vertical_reversal_metrics(report_path, primitives, cfg)
    profile = compute_vertical_profile_metrics(report_path, primitives, tq, cfg)
    safety = verify_path_safety(report_path, primitives, tq, cfg)

    xyz = [state_to_xyz(s, tq, cfg) for s in report_path]
    xy_length = sum(math.hypot(xyz[i + 1][0] - xyz[i][0], xyz[i + 1][1] - xyz[i][1]) for i in range(len(xyz) - 1))

    # Recompute the normalized distance/altitude components directly (reporting only,
    # same production formula) since compute_edge_cost only returns the combined total.
    total_distance_component = 0.0
    total_altitude_component = 0.0
    for i in range(len(xyz) - 1):
        x1, y1, z1 = xyz[i]
        x2, y2, z2 = xyz[i + 1]
        geom = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
        dC_distance = geom / d_ref
        mean_msl = (z1 + z2) / 2.0
        excess = max(0.0, mean_msl - ALTITUDE_REFERENCE_MSL)
        dC_altitude = dC_distance * (excess / NORMALIZED_ALTITUDE_SCALE_M)
        total_distance_component += NORMALIZED_W_DISTANCE * dC_distance
        total_altitude_component += NORMALIZED_W_ALTITUDE * dC_altitude

    print("=== PATH PHYSICAL ANALYSIS ===")
    print(f"  path_node_count={len(report_path)}")
    print(f"  xy_length={xy_length:.1f}m  3d_length={alt_metrics['geometric_path_length']:.1f}m")
    print(f"  min_MSL={alt_metrics['minimum_aircraft_msl']:.1f}  mean_MSL={alt_metrics['average_aircraft_msl']:.1f}  "
          f"max_MSL={alt_metrics['maximum_aircraft_msl']:.1f}  min_AGL={min_agl:.1f}")
    print(f"  total_climb={alt_metrics['total_climb_m']:.1f}  total_descent={alt_metrics['total_descent_m']:.1f}")
    print(f"  normalized_distance_component={total_distance_component:.6f}  "
          f"normalized_altitude_component={total_altitude_component:.6f}  "
          f"total_diagnostic_cost={result.total_cost:.6f}  (reversal component forced to 0.0)")
    print(f"  SAFETY CHECK: {'PASS' if safety['ok'] else 'FAIL -- ' + safety.get('reason', '')}  "
          f"(min_agl={safety.get('min_agl', 'n/a')}, max_primitive_angle={safety.get('max_angle_deg', 'n/a')})")
    print(f"  ascii: {ascii_profile(profile, profile['total_horizontal_distance_m'])}")

    os.makedirs("outputs", exist_ok=True)
    with open(OUTPUT_CSV, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "row", "col", "z_index", "x", "y", "z_msl"])
        for i, s in enumerate(report_path):
            x, y, zm = state_to_xyz(s, tq, cfg)
            writer.writerow([i, s[0], s[1], s[2], f"{x:.2f}", f"{y:.2f}", f"{zm:.2f}"])
    print(f"\nPath written to {OUTPUT_CSV}")


if __name__ == "__main__":
    main()
