"""Stage 33: the ONE real 6.48km Aladaglar benchmark run with the new Safe
Goal Region (70x70x50m box: +/-35m XY, +/-25m Z) enabled, on top of the
Stage 32 normalized-cost baseline (unchanged): cost_mode="normalized",
altitude_reference_msl=3240, H_scale=1000, w_altitude=1.25, w_distance=1.0,
w_reversal=1.0, epsilon_search=1.10, target_suboptimality=1.05, dominance
OFF, primitive cache ON, incumbent pruning ON. Single run, 30,000-expansion
cap, NO retry.

Hypothesis being tested: does the search actually get close to the exact
goal state but just miss the single discretized cell, so a small tolerance
box lets it succeed?
"""
import dataclasses
import math
import time

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

# Stage 32's own (goal_tolerance=0) real-benchmark result, for comparison -- NOT re-run.
STAGE32_REFERENCE = {
    "status": "search_limit_reached", "expanded_nodes": 30_000, "max_open_size": 51_492,
    "cache_hit_rate": 0.757, "reopened_states": 0, "initial_incumbent_cost": 1.650000,
    "first_solution_found": False, "current_lower_bound": 1.094752, "final_bound_ratio": 1.5072,
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

    z0 = msl_to_z_index(AIRCRAFT_MSL, cfg)
    start, goal = (START_ROW, START_COL, z0), (GOAL_ROW, GOAL_COL, z0)

    seg = roi.elevation[START_ROW:GOAL_ROW + 1, START_COL]
    seg_min = float(seg.min())
    min_search = math.ceil((seg_min + cfg.min_agl_m) / cfg.z_step_m) * cfg.z_step_m
    max_search = AIRCRAFT_MSL + 20.0

    d_ref = compute_distance_reference(start, goal, tq, cfg)

    print(f"=== Stage 33 REAL Aladaglar benchmark -- Safe Goal Region ({2*GOAL_TOLERANCE_XY_M:.0f}x"
          f"{2*GOAL_TOLERANCE_XY_M:.0f}x{2*GOAL_TOLERANCE_Z_M:.0f}m box) (SINGLE run, cap={MAX_EXPANSIONS}, no retry) ===")
    print(f"  start=({START_ROW},{START_COL}) goal_center=({GOAL_ROW},{GOAL_COL}) aircraft_msl={AIRCRAFT_MSL}")
    print(f"  goal_tolerance_xy_m={GOAL_TOLERANCE_XY_M}  goal_tolerance_z_m={GOAL_TOLERANCE_Z_M}")
    print(f"  D_ref={d_ref:.4f}m  altitude_reference_msl={ALTITUDE_REFERENCE_MSL}  "
          f"H_scale={NORMALIZED_ALTITUDE_SCALE_M}  w_altitude={NORMALIZED_W_ALTITUDE}  "
          f"w_distance={NORMALIZED_W_DISTANCE}  w_reversal={NORMALIZED_W_REVERSAL}")
    print(f"  search bounds=[{min_search},{max_search}]  epsilon_search={EPSILON_SEARCH}  "
          f"target_suboptimality={TARGET_SUBOPTIMALITY}  dominance=OFF  cache=ON  incumbent_pruning=ON\n")

    direct_level_path = build_direct_level(z0)
    ok, incumbent_cost = validate_and_cost_path(direct_level_path, primitives, tq, cfg, d_ref)
    print(f"initial incumbent (direct 216-edge level chain, re-validated under normalized cost + "
          f"goal region): valid={ok} cost={incumbent_cost:.6f}\n")
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
    )
    wall = time.perf_counter() - t0

    reopen_ratio = result.reopened_states / result.expanded_nodes if result.expanded_nodes else float("nan")
    print("=== SEARCH RESULT ===")
    print(f"  status={result.status}  wall={wall:.2f}s  runtime={result.runtime_s:.2f}s")
    print(f"  expanded={result.expanded_nodes}  max_open={result.max_open_size}  "
          f"cache_hit_rate={result.primitive_cache_hit_rate:.3f}  reopened_states={result.reopened_states}  "
          f"reopen_ratio={reopen_ratio:.4f}")
    print(f"  initial_incumbent_cost={incumbent_cost:.6f}  final_incumbent_cost={result.final_incumbent_cost:.6f}  "
          f"incumbent_updates={result.incumbent_updates}  incumbent_pruned_candidates={result.incumbent_pruned_candidates}")
    first_found = result.first_solution_cost < math.inf
    print(f"  first_solution_found={first_found}  first_solution_cost="
          f"{result.first_solution_cost if first_found else 'inf'}  "
          f"first_solution_expanded={result.first_solution_expanded if first_found else 'n/a'}  "
          f"first_solution_runtime={result.first_solution_runtime_s if first_found else float('nan')}")
    print(f"  current_lower_bound={result.current_lower_bound:.6f}  final_bound_ratio={result.final_bound_ratio:.4f}")
    print(f"  bounded_termination_triggered={result.bounded_termination_triggered}")
    if result.bounded_termination_triggered:
        print(f"  >> %5 CERTIFIED bounded-suboptimal solution: at most "
              f"%{(result.final_bound_ratio - 1) * 100:.2f} suboptimal vs the current discretized search graph optimum.")
    elif first_found:
        gap_pct = (result.final_bound_ratio - 1) * 100 if not math.isnan(result.final_bound_ratio) else float("nan")
        print(f"  >> Solution found but %5 certificate not reached. Certified gap ~= %{gap_pct:.2f}")
    else:
        print("  >> No internal complete solution found within this cap.")
    print()

    print("=== GOAL-REGION DIAGNOSTIC (closest expanded state to goal) ===")
    print(f"  closest_distance_to_goal_region_m={result.closest_distance_to_goal_region_m:.2f}  "
          f"(0.0 means some state DID enter the box)")
    print(f"  closest_distance_to_goal_center_m={result.closest_distance_to_goal_center_m:.2f}")
    if result.closest_state_to_goal is not None:
        cr, cc, cz = result.closest_state_to_goal
        cx, cy, cmsl = state_to_xyz(result.closest_state_to_goal, tq, cfg)
        terrain_elev = roi.elevation[cr, cc]
        point_agl = cmsl - float(terrain_elev)
        print(f"  closest_state: row={cr} col={cc} z_index={cz}  x={cx:.1f} y={cy:.1f} msl={cmsl:.1f}  "
              f"terrain={terrain_elev:.1f}  point_AGL={point_agl:.1f}")
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
        improvement_pct = (1.0 - report_cost / incumbent_cost) * 100.0 if incumbent_cost < math.inf else float("nan")

        reached = report_path[-1]
        rx, ry, rz = state_to_xyz(reached, tq, cfg)
        gx, gy, gz = state_to_xyz(goal, tq, cfg)
        dx, dy, dz = rx - gx, ry - gy, rz - gz
        horizontal_error = math.hypot(dx, dy)
        total_3d_error = math.sqrt(dx * dx + dy * dy + dz * dz)

        print("=== DEVIATION FROM GOAL CENTER (SUCCESS case) ===")
        print(f"  target: row={goal[0]} col={goal[1]} z_index={goal[2]}  x={gx:.1f} y={gy:.1f} z={gz:.1f}")
        print(f"  reached: row={reached[0]} col={reached[1]} z_index={reached[2]}  x={rx:.1f} y={ry:.1f} z={rz:.1f}")
        print(f"  dx={dx:.2f}  dy={dy:.2f}  dz={dz:.2f}")
        print(f"  horizontal_error={horizontal_error:.2f}m  total_3d_error={total_3d_error:.2f}m\n")

        print("=== PATH PHYSICAL ANALYSIS ===")
        print(f"  final_total_cost={report_cost:.6f}  improvement_vs_direct_incumbent={improvement_pct:.2f}%")
        print(f"  geometric_path_length={alt_metrics['geometric_path_length']:.1f}m")
        print(f"  avg_MSL={alt_metrics['average_aircraft_msl']:.1f}  min_MSL={alt_metrics['minimum_aircraft_msl']:.1f}  "
              f"max_MSL={alt_metrics['maximum_aircraft_msl']:.1f}  min_AGL={min_agl:.1f}")
        print(f"  total_climb={alt_metrics['total_climb_m']:.1f}  total_descent={alt_metrics['total_descent_m']:.1f}  "
              f"reversal_count={reversal_metrics['total_vertical_reversal_count']}")
        print(f"  first_descent_at={profile['first_descent_distance_m']}m  "
              f"last_climb_start_distance_m={last_climb}")
        print(f"  SAFETY CHECK: {'PASS' if safety['ok'] else 'FAIL -- ' + safety.get('reason', '')}  "
              f"(min_agl={safety.get('min_agl', 'n/a')}, max_primitive_angle={safety.get('max_angle_deg', 'n/a')})")
        print(f"  ascii: {ascii_profile(profile, profile['total_horizontal_distance_m'])}")
    else:
        print("=== No path (neither success nor a first internal solution) -- no physical/deviation analysis possible. ===")

    print()
    print("=== COMPARISON vs Stage 32 (goal_tolerance=0, same everything else) -- from project.md, NOT re-run ===")
    ref = STAGE32_REFERENCE
    print(f"  Stage 32 (exact goal): status={ref['status']} expanded={ref['expanded_nodes']} "
          f"max_open={ref['max_open_size']} first_solution_found={ref['first_solution_found']} "
          f"final_bound_ratio={ref['final_bound_ratio']}")
    print(f"  Stage 33 (region goal): status={result.status} expanded={result.expanded_nodes} "
          f"max_open={result.max_open_size} first_solution_found={first_found} "
          f"final_bound_ratio={result.final_bound_ratio:.4f}")


if __name__ == "__main__":
    main()
