"""Follow-up to Stage 32: same real 6.48km Aladaglar benchmark, same
normalized cost mode / H_scale / weights / heuristic / config -- ONLY
epsilon_search changed, to 1.20 and 1.30. NO code, cost, heuristic, or
config change. Stage 32's own epsilon=1.10 run is NOT re-run here (its
recorded numbers from project.md are the reference). 150k retry NOT done.
Epsilon values beyond 1.30 NOT tried.

Reuses every constant/setting from scripts/run_normalized_aladaglar_benchmark.py
unchanged (imported, not retyped) -- only the epsilon_search argument to
astar_search() varies between the two runs below.
"""
import math
import time

from planner.astar import (
    _path_altitude_metrics, _path_min_observed_agl, _path_vertical_reversal_metrics,
    astar_search, compute_distance_reference, msl_to_z_index, validate_and_cost_path,
)
from planner.primitives import build_primitive_set
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from scripts.benchmark_long_valley import last_climb_start_distance, verify_path_safety
from scripts.calibrate_low_msl_behavior import ascii_profile, compute_vertical_profile_metrics
from scripts.run_normalized_aladaglar_benchmark import (
    ALTITUDE_REFERENCE_MSL, AIRCRAFT_MSL, GOAL_COL, GOAL_ROW, MAX_EXPANSIONS,
    NORMALIZED_ALTITUDE_SCALE_M, NORMALIZED_W_ALTITUDE, NORMALIZED_W_DISTANCE,
    NORMALIZED_W_REVERSAL, START_COL, START_ROW, TARGET_SUBOPTIMALITY,
)
from scripts.validate_cost_function_ranking import build_direct_level

import dataclasses
from planner.config import DEFAULT_CONFIG

EPSILON_VALUES = [1.20, 1.30]

STAGE32_EPS110_REFERENCE = {
    "epsilon_search": 1.10, "status": "search_limit_reached", "runtime_s": 64.30,
    "expanded_nodes": 30_000, "max_open_size": 51_492, "cache_hit_rate": 0.757,
    "reopened_states": 0, "reopen_ratio": 0.0, "first_solution_found": False,
    "first_solution_expanded": "n/a", "first_solution_cost": math.inf,
    "initial_incumbent_cost": 1.650000,
    "final_incumbent_cost": 1.650000, "incumbent_updates": 0, "incumbent_pruned_candidates": 0,
    "current_lower_bound": 1.094752, "final_bound_ratio": 1.5072,
    "bounded_termination_triggered": False, "min_MSL": None,
}


def run_one(epsilon, cfg, primitives, tq, start, goal, min_search, max_search, d_ref, incumbent_cost, direct_level_path):
    t0 = time.perf_counter()
    result = astar_search(
        start, goal, tq, min_search_altitude_msl=min_search, max_search_altitude_msl=max_search,
        config=cfg, primitives=primitives, max_expansions=MAX_EXPANSIONS,
        use_primitive_cache=True, use_dominance_pruning=False,
        use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False,
        use_incumbent_pruning=True, initial_incumbent_cost=incumbent_cost,
        initial_incumbent_path=direct_level_path,
        epsilon_search=epsilon, target_suboptimality=TARGET_SUBOPTIMALITY,
    )
    wall = time.perf_counter() - t0
    reopen_ratio = result.reopened_states / result.expanded_nodes if result.expanded_nodes else float("nan")

    print(f"=== epsilon_search={epsilon} (cap={MAX_EXPANSIONS}) ===")
    print(f"  status={result.status}  wall={wall:.2f}s  runtime={result.runtime_s:.2f}s")
    print(f"  expanded={result.expanded_nodes}  max_open={result.max_open_size}  "
          f"cache_hit_rate={result.primitive_cache_hit_rate:.3f}")
    print(f"  reopened_states={result.reopened_states}  reopen_ratio={reopen_ratio:.4f}")
    first_found = result.first_solution_cost < math.inf
    print(f"  first_solution_found={first_found}  first_solution_cost="
          f"{result.first_solution_cost if first_found else 'inf'}  "
          f"first_solution_expanded={result.first_solution_expanded if first_found else 'n/a'}  "
          f"first_solution_runtime={result.first_solution_runtime_s if first_found else float('nan')}")
    print(f"  initial_incumbent_cost={incumbent_cost:.6f}  final_incumbent_cost={result.final_incumbent_cost:.6f}  "
          f"incumbent_updates={result.incumbent_updates}  incumbent_pruned_candidates={result.incumbent_pruned_candidates}")
    print(f"  current_lower_bound={result.current_lower_bound:.6f}  final_bound_ratio={result.final_bound_ratio:.4f}")
    print(f"  bounded_termination_triggered={result.bounded_termination_triggered}")
    if result.bounded_termination_triggered:
        print(f"  >> %5 CERTIFICATE ACHIEVED: at most %{(result.final_bound_ratio - 1) * 100:.2f} suboptimal.")
    elif first_found:
        gap_pct = (result.final_bound_ratio - 1) * 100 if not math.isnan(result.final_bound_ratio) else float("nan")
        print(f"  >> Solution found but %5 certificate not reached. Certified gap ~= %{gap_pct:.2f}")
    else:
        print("  >> No internal complete solution found within this cap.")

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
        distance_ratio = alt_metrics["geometric_path_length"] / d_ref
        print(f"  final_total_cost={report_cost:.6f}  geometric_path_length={alt_metrics['geometric_path_length']:.1f}m  "
              f"distance_ratio_vs_direct={distance_ratio:.4f}")
        print(f"  avg_MSL={alt_metrics['average_aircraft_msl']:.1f}  min_MSL={alt_metrics['minimum_aircraft_msl']:.1f}  "
              f"min_AGL={min_agl:.1f}")
        print(f"  total_climb={alt_metrics['total_climb_m']:.1f}  total_descent={alt_metrics['total_descent_m']:.1f}  "
              f"reversal_count={reversal_metrics['total_vertical_reversal_count']}")
        print(f"  last_climb_start_distance_m={last_climb}")
        print(f"  SAFETY CHECK: {'PASS' if safety['ok'] else 'FAIL -- ' + safety.get('reason', '')}  "
              f"(min_agl={safety.get('min_agl', 'n/a')}, max_primitive_angle={safety.get('max_angle_deg', 'n/a')})")
        print(f"  ascii: {ascii_profile(profile, profile['total_horizontal_distance_m'])}")
    print()
    return result, reopen_ratio


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

    direct_level_path = build_direct_level(z0)
    ok, incumbent_cost = validate_and_cost_path(direct_level_path, primitives, tq, cfg, d_ref)
    print(f"initial incumbent (direct 216-edge level chain, normalized cost, identical to Stage 32): "
          f"valid={ok} cost={incumbent_cost:.6f}\n")
    assert ok

    rows = [dict(STAGE32_EPS110_REFERENCE)]
    for eps in EPSILON_VALUES:
        result, reopen_ratio = run_one(eps, cfg, primitives, tq, start, goal, min_search, max_search,
                                        d_ref, incumbent_cost, direct_level_path)
        first_found = result.first_solution_cost < math.inf
        alt_metrics = _path_altitude_metrics(result.path, tq, cfg) if result.success and result.path else None
        rows.append({
            "epsilon_search": eps, "status": result.status, "runtime_s": result.runtime_s,
            "expanded_nodes": result.expanded_nodes, "max_open_size": result.max_open_size,
            "cache_hit_rate": result.primitive_cache_hit_rate, "reopened_states": result.reopened_states,
            "reopen_ratio": reopen_ratio, "first_solution_found": first_found,
            "first_solution_expanded": result.first_solution_expanded if first_found else "n/a",
            "first_solution_cost": result.first_solution_cost,
            "final_incumbent_cost": result.final_incumbent_cost,
            "current_lower_bound": result.current_lower_bound, "final_bound_ratio": result.final_bound_ratio,
            "bounded_termination_triggered": result.bounded_termination_triggered,
            "min_MSL": alt_metrics["minimum_aircraft_msl"] if alt_metrics else None,
        })

    print("=== COMPARISON TABLE ===")
    header = (f"{'eps':>5} {'status':>19} {'goal_exp':>9} {'runtime':>8} {'max_open':>8} {'reopen':>7} "
              f"{'incumbent':>10} {'LB':>10} {'ratio':>7} {'min_MSL':>8}")
    print(header)
    for r in rows:
        exp_str = str(r.get("first_solution_expanded", "n/a")) if r.get("first_solution_found") else "n/a"
        min_msl_str = f"{r['min_MSL']:.1f}" if r.get("min_MSL") is not None else "n/a"
        print(f"{r['epsilon_search']:>5} {r['status']:>19} {exp_str:>9} {r['runtime_s']:>8.2f} "
              f"{r['max_open_size']:>8} {r['reopened_states']:>7} {r['final_incumbent_cost']:>10.6f} "
              f"{r['current_lower_bound']:>10.6f} {r['final_bound_ratio']:>7.4f} {min_msl_str:>8}")


if __name__ == "__main__":
    main()
