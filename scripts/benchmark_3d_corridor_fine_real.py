"""Stage 37.1: the ONE real 6.48km Aladaglar benchmark run with the fine
A* constrained to a full 3D guidance tube: Stage 36's +/-300m XY corridor
AND a +/-200m Z tube around the epsilon=1.5 coarse path's own
(linearly-interpolated) altitude. Every other setting is IDENTICAL to
Stage 37's own real run -- ONLY z_guide_grid/z_guide_tolerance_m are new.
Stage 37's baseline (project.md) is NOT re-run here.
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
OUTPUT_CSV = "outputs/stage37_1_fine_3d_corridor_path.csv"

# Stage 37's own recorded real-benchmark result (project.md) -- NOT re-run.
STAGE37_REFERENCE = {
    "status": "search_limit_reached", "expanded_nodes": 30_000, "max_open_size": 34_435,
    "wall_s": 37.00, "corridor_reject_count": 53_790,
    "closest_distance_to_goal_region_m": 4918.11, "closest_row": 99,
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

    # Confirm start/goal fall inside the 3D tube -- force-include (widen just that
    # cell's z_guide, never move the mission endpoint) only if actually needed.
    for label, (r, c) in [("START", (START_ROW, START_COL)), ("GOAL", (GOAL_ROW, GOAL_COL))]:
        diff = abs(AIRCRAFT_MSL - float(z_guide_grid[r, c]))
        in_tube = bool(xy_mask[r, c]) and diff <= Z_TOLERANCE_M
        print(f"  {label} tube check: xy_in_corridor={bool(xy_mask[r, c])} z_guide={z_guide_grid[r, c]:.1f} "
              f"diff={diff:.1f} in_tube={in_tube}")
        if not in_tube:
            print(f"  !! {label} not naturally in the 3D tube -- forcing z_guide[{r},{c}]=aircraft_msl "
                  f"(endpoint itself unchanged) so the mission always starts/ends inside it.")
            z_guide_grid[r, c] = AIRCRAFT_MSL
            xy_mask[r, c] = True

    seg = roi.elevation[START_ROW:GOAL_ROW + 1, START_COL]
    seg_min = float(seg.min())
    min_search = math.ceil((seg_min + cfg.min_agl_m) / cfg.z_step_m) * cfg.z_step_m
    max_search = AIRCRAFT_MSL + 20.0

    d_ref = compute_distance_reference(start, goal, tq, cfg)

    print(f"\n=== Stage 37.1 REAL Aladaglar benchmark -- 3D corridor "
          f"(XY +/-300m, Z +/-{Z_TOLERANCE_M:.0f}m) (SINGLE run, cap={MAX_EXPANSIONS}, no retry) ===")
    print(f"  D_ref={d_ref:.4f}m  epsilon_search={EPSILON_SEARCH}  target_suboptimality={TARGET_SUBOPTIMALITY}\n")

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
    )
    wall = time.perf_counter() - t0

    print("=== SEARCH RESULT ===")
    print(f"  status={result.status}  wall={wall:.2f}s  runtime={result.runtime_s:.2f}s")
    print(f"  expanded={result.expanded_nodes}  max_open={result.max_open_size}  "
          f"cache_hit_rate={result.primitive_cache_hit_rate:.3f}  reopened_states={result.reopened_states}")
    print(f"  xy_corridor_reject_count={result.corridor_reject_count}  "
          f"z_corridor_reject_count={result.z_corridor_reject_count}")
    print(f"  initial_incumbent_cost={incumbent_cost:.6f}  final_incumbent_cost={result.final_incumbent_cost:.6f}  "
          f"incumbent_updates={result.incumbent_updates}")
    first_found = result.first_solution_cost < math.inf
    print(f"  first_solution_found={first_found}  bounded_termination_triggered={result.bounded_termination_triggered}")
    print()

    print("=== STATE-SPACE COMPOSITION DIAGNOSTICS ===")
    print(f"  unique_expanded_xy={result.unique_expanded_xy}")
    print(f"  unique_expanded_xyz={result.unique_expanded_xyz}")
    print(f"  unique_full_states={result.unique_full_states}")
    print(f"  avg_z_states_per_xy={result.avg_z_states_per_xy:.3f}")
    print(f"  avg_history_states_per_xyz={result.avg_history_states_per_xyz:.3f}")
    print(f"  max_z_states_in_one_xy={result.max_z_states_in_one_xy}")
    print()

    print("=== GOAL-REGION DIAGNOSTIC ===")
    print(f"  closest_distance_to_goal_region_m={result.closest_distance_to_goal_region_m:.2f}")
    print(f"  closest_distance_to_goal_center_m={result.closest_distance_to_goal_center_m:.2f}")
    if result.closest_state_to_goal is not None:
        cr, cc, cz = result.closest_state_to_goal
        cx, cy, cmsl = state_to_xyz(result.closest_state_to_goal, tq, cfg)
        terrain_elev = roi.elevation[cr, cc]
        print(f"  closest_state: row={cr} col={cc} z_index={cz}  msl={cmsl:.1f}  "
              f"point_AGL={cmsl - float(terrain_elev):.1f}")
    print()

    print("=== COMPARISON vs Stage 37 baseline (NOT re-run) ===")
    b = STAGE37_REFERENCE
    print(f"  Stage 37:   status={b['status']} expanded={b['expanded_nodes']} max_open={b['max_open_size']} "
          f"wall={b['wall_s']:.2f}s closest_goal_region={b['closest_distance_to_goal_region_m']:.2f}m "
          f"xy_rejects={b['corridor_reject_count']}")
    print(f"  Stage 37.1: status={result.status} expanded={result.expanded_nodes} max_open={result.max_open_size} "
          f"wall={wall:.2f}s closest_goal_region={result.closest_distance_to_goal_region_m:.2f}m "
          f"xy_rejects={result.corridor_reject_count} z_rejects={result.z_corridor_reject_count}")
    if b["expanded_nodes"]:
        print(f"  expansion change vs Stage 37: {(1 - result.expanded_nodes / b['expanded_nodes']) * 100.0:+.2f}%  "
              f"wall-clock change: {(1 - wall / b['wall_s']) * 100.0:+.2f}%")
    print()

    report_path = result.path if (result.success and result.path) else (
        result.incumbent_path if first_found else []
    )
    report_cost = result.total_cost if (result.success and result.path) else result.final_incumbent_cost

    if report_path:
        alt_metrics = _path_altitude_metrics(report_path, tq, cfg)
        min_agl = _path_min_observed_agl(report_path, primitives, tq, cfg)
        reversal_metrics = _path_vertical_reversal_metrics(report_path, primitives, cfg)
        profile = compute_vertical_profile_metrics(report_path, primitives, tq, cfg)
        safety = verify_path_safety(report_path, primitives, tq, cfg)
        last_climb = last_climb_start_distance(report_path, primitives, cfg)

        reached = report_path[-1]
        rx, ry, rz = state_to_xyz(reached, tq, cfg)
        gx, gy, gz = state_to_xyz(goal, tq, cfg)
        dx, dy, dz = rx - gx, ry - gy, rz - gz
        horizontal_error = math.hypot(dx, dy)
        total_3d_error = math.sqrt(dx * dx + dy * dy + dz * dz)
        xy_length = sum(math.hypot(state_to_xyz(report_path[i + 1], tq, cfg)[0] - state_to_xyz(report_path[i], tq, cfg)[0],
                                    state_to_xyz(report_path[i + 1], tq, cfg)[1] - state_to_xyz(report_path[i], tq, cfg)[1])
                         for i in range(len(report_path) - 1))

        print("=== DEVIATION FROM GOAL CENTER ===")
        print(f"  dx={dx:.2f}  dy={dy:.2f}  dz={dz:.2f}")
        print(f"  horizontal_error={horizontal_error:.2f}m  total_3d_error={total_3d_error:.2f}m\n")

        print("=== PATH PHYSICAL ANALYSIS ===")
        print(f"  path_node_count={len(report_path)}")
        print(f"  xy_length={xy_length:.1f}m  3d_length={alt_metrics['geometric_path_length']:.1f}m")
        print(f"  final_total_cost={report_cost:.6f}")
        print(f"  avg_MSL={alt_metrics['average_aircraft_msl']:.1f}  min_MSL={alt_metrics['minimum_aircraft_msl']:.1f}  "
              f"max_MSL={alt_metrics['maximum_aircraft_msl']:.1f}  min_AGL={min_agl:.1f}")
        print(f"  total_climb={alt_metrics['total_climb_m']:.1f}  total_descent={alt_metrics['total_descent_m']:.1f}  "
              f"reversal_count={reversal_metrics['total_vertical_reversal_count']}")
        print(f"  SAFETY CHECK: {'PASS' if safety['ok'] else 'FAIL -- ' + safety.get('reason', '')}  "
              f"(min_agl={safety.get('min_agl', 'n/a')}, max_primitive_angle={safety.get('max_angle_deg', 'n/a')})")
        print(f"  ascii: {ascii_profile(profile, profile['total_horizontal_distance_m'])}")

        if result.success:
            os.makedirs("outputs", exist_ok=True)
            with open(OUTPUT_CSV, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["index", "row", "col", "z_index", "x", "y", "z_msl"])
                for i, s in enumerate(report_path):
                    x, y, zm = state_to_xyz(s, tq, cfg)
                    writer.writerow([i, s[0], s[1], s[2], f"{x:.2f}", f"{y:.2f}", f"{zm:.2f}"])
            print(f"\nPath written to {OUTPUT_CSV}")
    else:
        print("=== FAIL -- no path/incumbent to report ===")
        print(f"  closest_state_to_goal={result.closest_state_to_goal}  "
              f"distance_to_goal_region_m={result.closest_distance_to_goal_region_m:.2f}")


if __name__ == "__main__":
    main()
