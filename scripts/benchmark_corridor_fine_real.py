"""Stage 37: the ONE real 6.48km Aladaglar benchmark run with the fine A*
now constrained to Stage 36's +/-300m XY corridor (outputs/
stage36_corridor_mask.npy) around the epsilon=1.5 coarse guide path.
Every other setting is IDENTICAL to Stage 33's own real run
(scripts/benchmark_goal_region_real.py) -- cost_mode="normalized",
altitude_reference_msl=3240, H_scale=1000, w_altitude=1.25, w_distance=1.0,
w_reversal=1.0, epsilon_search=1.10, target_suboptimality=1.05, dominance
OFF, primitive cache ON, incumbent pruning ON, goal region +/-35m XY/+/-25m
Z -- ONLY corridor_mask is new. Single run, 30,000-expansion cap, no retry.
Stage 33's own baseline (project.md) is NOT re-run here.
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
MASK_NPY = "outputs/stage36_corridor_mask.npy"
OUTPUT_CSV = "outputs/stage37_fine_corridor_path.csv"

# Stage 33's own recorded real-benchmark result (project.md) -- NOT re-run.
STAGE33_REFERENCE = {
    "status": "search_limit_reached", "expanded_nodes": 30_000, "max_open_size": 51_475,
    "cache_hit_rate": 0.757, "reopened_states": 0, "initial_incumbent_cost": 1.650000,
    "first_solution_found": False, "final_bound_ratio": 1.5148,
    "closest_distance_to_goal_region_m": 5066.80, "closest_distance_to_goal_center_m": 5102.51,
    "wall_s": 98.95,
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
    corridor_mask = np.load(MASK_NPY)

    z0 = msl_to_z_index(AIRCRAFT_MSL, cfg)
    start, goal = (START_ROW, START_COL, z0), (GOAL_ROW, GOAL_COL, z0)

    seg = roi.elevation[START_ROW:GOAL_ROW + 1, START_COL]
    seg_min = float(seg.min())
    min_search = math.ceil((seg_min + cfg.min_agl_m) / cfg.z_step_m) * cfg.z_step_m
    max_search = AIRCRAFT_MSL + 20.0

    d_ref = compute_distance_reference(start, goal, tq, cfg)

    print(f"=== Stage 37 REAL Aladaglar benchmark -- corridor-constrained fine A* "
          f"(mask={MASK_NPY}, {int(corridor_mask.sum())} cells) (SINGLE run, cap={MAX_EXPANSIONS}, no retry) ===")
    print(f"  start=({START_ROW},{START_COL}) goal_center=({GOAL_ROW},{GOAL_COL}) aircraft_msl={AIRCRAFT_MSL}")
    print(f"  goal_tolerance_xy_m={GOAL_TOLERANCE_XY_M}  goal_tolerance_z_m={GOAL_TOLERANCE_Z_M}")
    print(f"  D_ref={d_ref:.4f}m  epsilon_search={EPSILON_SEARCH}  target_suboptimality={TARGET_SUBOPTIMALITY}  "
          f"dominance=OFF  cache=ON  incumbent_pruning=ON\n")

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
        corridor_mask=corridor_mask,
    )
    wall = time.perf_counter() - t0

    print("=== SEARCH RESULT ===")
    print(f"  status={result.status}  wall={wall:.2f}s  runtime={result.runtime_s:.2f}s")
    print(f"  expanded={result.expanded_nodes}  max_open={result.max_open_size}  "
          f"cache_hit_rate={result.primitive_cache_hit_rate:.3f}  reopened_states={result.reopened_states}")
    print(f"  corridor_reject_count={result.corridor_reject_count}")
    print(f"  initial_incumbent_cost={incumbent_cost:.6f}  final_incumbent_cost={result.final_incumbent_cost:.6f}  "
          f"incumbent_updates={result.incumbent_updates}")
    first_found = result.first_solution_cost < math.inf
    print(f"  first_solution_found={first_found}  bounded_termination_triggered={result.bounded_termination_triggered}")
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

    print("=== COMPARISON vs Stage 33 baseline (NOT re-run) ===")
    b = STAGE33_REFERENCE
    print(f"  Stage 33: status={b['status']} expanded={b['expanded_nodes']} max_open={b['max_open_size']} "
          f"wall={b['wall_s']:.2f}s closest_goal_region={b['closest_distance_to_goal_region_m']:.2f}m")
    print(f"  Stage 37: status={result.status} expanded={result.expanded_nodes} max_open={result.max_open_size} "
          f"wall={wall:.2f}s closest_goal_region={result.closest_distance_to_goal_region_m:.2f}m")
    if b["expanded_nodes"]:
        exp_reduction = (1 - result.expanded_nodes / b["expanded_nodes"]) * 100.0
        print(f"  expansion change vs Stage 33: {exp_reduction:+.2f}%  wall-clock change: "
              f"{(1 - wall / b['wall_s']) * 100.0:+.2f}%")
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
        print(f"  target: x={gx:.1f} y={gy:.1f} z={gz:.1f}")
        print(f"  reached: x={rx:.1f} y={ry:.1f} z={rz:.1f}")
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
        print(f"  max_flight_path_angle from SAFETY CHECK below")
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
        print(f"  expanded={result.expanded_nodes}  max_open={result.max_open_size}")


if __name__ == "__main__":
    main()
