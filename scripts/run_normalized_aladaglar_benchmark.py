"""Stage 32: the ONE real 6.48km Aladaglar benchmark run under the new
production normalized cost mode. Single run, 30,000-expansion cap, NO
retry (per spec) -- reuses the exact same coordinates/aircraft/hard-safety
as Stage 19/25/26/27/28/29/30/31.

Settings (all per project.md "Stage 32" section 17):
    cost_mode = "normalized"
    altitude_reference_msl = 3240.0 (explicit config field now, NOT derived
        from min_search_altitude_msl -- see planner/config.py)
    normalized_altitude_scale_m = 1000.0, normalized_w_altitude = 1.25
        (the calibration-selected candidate from
        scripts/calibrate_normalized_production_cost.py)
    normalized_w_distance = 1.0, normalized_w_reversal = 1.0
    epsilon_search = 1.10, target_suboptimality = 1.05
    dominance OFF, primitive cache ON, incumbent pruning ON
    heuristic: automatic minimal normalized heuristic (cost_mode="normalized"
        bypasses the legacy MSL-lower-bound/vertical-reachability machinery
        entirely -- see planner/astar.py _heuristic docstring)
"""
import dataclasses
import math
import time

from planner.astar import (
    _path_altitude_metrics, _path_min_observed_agl, _path_vertical_reversal_metrics,
    astar_search, compute_distance_reference, msl_to_z_index, validate_and_cost_path,
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

# Stage 26's legacy reference for the final physical-behavior comparison (project.md, NOT re-run)
LEGACY_STAGE26_EPS110_REFERENCE = {
    "first_solution_expanded": 685, "final_incumbent_cost": 21668.85,
    "minimum_aircraft_msl": 3660.0, "total_descent_m": 100.0, "total_climb_m": 100.0,
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

    print(f"=== Stage 32 REAL Aladaglar benchmark -- cost_mode='normalized' (SINGLE run, cap={MAX_EXPANSIONS}, no retry) ===")
    print(f"  start=({START_ROW},{START_COL}) goal=({GOAL_ROW},{GOAL_COL}) aircraft_msl={AIRCRAFT_MSL}")
    print(f"  D_ref={d_ref:.4f}m  altitude_reference_msl={ALTITUDE_REFERENCE_MSL}  "
          f"H_scale={NORMALIZED_ALTITUDE_SCALE_M}  w_altitude={NORMALIZED_W_ALTITUDE}  "
          f"w_distance={NORMALIZED_W_DISTANCE}  w_reversal={NORMALIZED_W_REVERSAL}")
    print(f"  search bounds=[{min_search},{max_search}]  epsilon_search={EPSILON_SEARCH}  "
          f"target_suboptimality={TARGET_SUBOPTIMALITY}  dominance=OFF  cache=ON  incumbent_pruning=ON\n")

    direct_level_path = build_direct_level(z0)
    ok, incumbent_cost = validate_and_cost_path(direct_level_path, primitives, tq, cfg, d_ref)
    print(f"initial incumbent (direct 216-edge level chain, NEW normalized cost): valid={ok} "
          f"cost={incumbent_cost:.6f}  (legacy cost was 21829.82 -- NOT reused, this is the new formula's own value)\n")
    if not ok:
        print("!! Direct level path rejected under normalized cost -- aborting.")
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
          f"cache_hit_rate={result.primitive_cache_hit_rate:.3f}")
    print(f"  reopened_states={result.reopened_states}  reopen_ratio={reopen_ratio:.4f}")
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
        print(f"  >> %5 CERTIFICATE ACHIEVED: at most %{(result.final_bound_ratio - 1) * 100:.2f} "
              f"suboptimal vs the current discretized search graph optimum.")
    elif first_found:
        gap_pct = (result.final_bound_ratio - 1) * 100 if not math.isnan(result.final_bound_ratio) else float("nan")
        print(f"  >> Solution found but %5 certificate not reached. Certified gap ~= %{gap_pct:.2f}")
    else:
        print("  >> No internal complete solution found within this cap.")
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
        distance_ratio = alt_metrics["geometric_path_length"] / d_ref

        print("=== PATH PHYSICAL ANALYSIS ===")
        print(f"  final_total_cost={report_cost:.6f}  improvement_vs_direct_incumbent={improvement_pct:.2f}%")
        print(f"  geometric_path_length={alt_metrics['geometric_path_length']:.1f}m  "
              f"distance_ratio_vs_direct={distance_ratio:.4f}")
        print(f"  avg_MSL={alt_metrics['average_aircraft_msl']:.1f}  min_MSL={alt_metrics['minimum_aircraft_msl']:.1f}  "
              f"max_MSL={alt_metrics['maximum_aircraft_msl']:.1f}  min_AGL={min_agl:.1f}")
        print(f"  total_climb={alt_metrics['total_climb_m']:.1f}  total_descent={alt_metrics['total_descent_m']:.1f}  "
              f"reversal_count={reversal_metrics['total_vertical_reversal_count']}")
        print(f"  low_MSL_dwell_distance={profile['low_msl_dwell_distance_m']:.1f}m "
              f"({profile['low_msl_dwell_ratio']:.3f} ratio)  first_descent_at={profile['first_descent_distance_m']}m")
        print(f"  last_climb_start_distance_m={last_climb}")
        print(f"  SAFETY CHECK: {'PASS' if safety['ok'] else 'FAIL -- ' + safety.get('reason', '')}  "
              f"(min_agl={safety.get('min_agl', 'n/a')}, max_primitive_angle={safety.get('max_angle_deg', 'n/a')})")
        print(f"  ascii: {ascii_profile(profile, profile['total_horizontal_distance_m'])}\n")

        print("=== COMPARISON vs LEGACY Stage 26 (eps=1.10) -- from project.md, NOT re-run ===")
        ref = LEGACY_STAGE26_EPS110_REFERENCE
        print(f"  legacy:     first_solution_expanded={ref['first_solution_expanded']}  "
              f"final_cost={ref['final_incumbent_cost']}  min_MSL={ref['minimum_aircraft_msl']}m  "
              f"descent={ref['total_descent_m']}m")
        print(f"  normalized: first_solution_expanded={result.first_solution_expanded if first_found else 'n/a'}  "
              f"final_cost={report_cost:.6f}  min_MSL={alt_metrics['minimum_aircraft_msl']:.1f}m  "
              f"descent={alt_metrics['total_descent_m']:.1f}m")
        deeper = alt_metrics['minimum_aircraft_msl'] < ref['minimum_aircraft_msl']
        print(f"  >> normalized cost dove DEEPER than legacy eps=1.10's 3660m: {deeper}")
    else:
        print("=== No path (neither success nor a first internal solution) -- no physical analysis possible. ===")


if __name__ == "__main__":
    main()
