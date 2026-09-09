"""Stage 26: real 6.48km Aladagalar benchmark -- epsilon_search sweep at
1.20 and 1.10 (target_suboptimality=1.05 fixed), testing whether Stage 25's
epsilon=1.5 thrashing (reopened_states~=26,714/30,000, no first solution)
eases at a less aggressive weighted bias. NO astar.py/heuristic/cost/reopen
changes this stage -- benchmark parameters only, reusing Stage 25's exact
astar_search(...) call shape unchanged.

Same coordinates/aircraft/w_MSL/settings as Stage 25: START row=48,col=276
GOAL row=264,col=276 AIRCRAFT_MSL=3760, dominance OFF, cache ON, msl_lower
_bound_heuristic ON, vertical_reachability_heuristic OFF, incumbent_pruning
ON with the same validated direct-216-edge-level-chain incumbent
(cost~=21829.82), 30,000-expansion cap, no retry. epsilon=1.5's own result
is NOT re-run -- Stage 25's recorded numbers (project.md) are the reference.
"""
import dataclasses
import math
import time

from planner.astar import (
    _path_altitude_metrics, _path_min_observed_agl, _path_vertical_reversal_metrics,
    astar_search, msl_to_z_index, validate_and_cost_path,
)
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from scripts.benchmark_long_valley import verify_path_safety, last_climb_start_distance
from scripts.calibrate_low_msl_behavior import ascii_profile, compute_vertical_profile_metrics, decompose_cost

START_ROW, START_COL = 48, 276
GOAL_ROW, GOAL_COL = 264, 276
AIRCRAFT_MSL = 3760.0
W_MSL = 0.63
MAX_EXPANSIONS = 30_000
TARGET_SUBOPTIMALITY = 1.05
EPSILON_VALUES = [1.20, 1.10]

# Stage 25's already-recorded epsilon=1.5 result (project.md) -- NOT re-run.
STAGE25_REFERENCE = {
    "epsilon_search": 1.5, "status": "search_limit_reached", "expanded_nodes": 30_000,
    "runtime_s": 12.99, "max_open_size": 1061, "cache_hit_rate": 0.965,
    "reopened_states": 26_714, "first_solution_cost": math.inf,
    "initial_incumbent_cost": 21829.82, "final_incumbent_cost": 21829.82,
    "incumbent_updates": 0, "current_lower_bound": 19716.80, "final_bound_ratio": 1.1072,
    "bounded_termination_triggered": False,
}


def run_one(epsilon, cfg, primitives, tq, start, goal, min_search, max_search, incumbent_cost, incumbent_path):
    t0 = time.perf_counter()
    result = astar_search(
        start, goal, tq, min_search_altitude_msl=min_search, max_search_altitude_msl=max_search,
        config=cfg, primitives=primitives, max_expansions=MAX_EXPANSIONS,
        use_primitive_cache=True, use_dominance_pruning=False,
        use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False,
        use_incumbent_pruning=True, initial_incumbent_cost=incumbent_cost, initial_incumbent_path=incumbent_path,
        epsilon_search=epsilon, target_suboptimality=TARGET_SUBOPTIMALITY,
    )
    wall = time.perf_counter() - t0
    reopen_ratio = result.reopened_states / result.expanded_nodes if result.expanded_nodes else float("nan")

    print(f"=== epsilon_search={epsilon} (cap={MAX_EXPANSIONS}, target_suboptimality={TARGET_SUBOPTIMALITY}) ===")
    print(f"  status={result.status} wall={wall:.2f}s runtime={result.runtime_s:.2f}s "
          f"expanded={result.expanded_nodes} max_open={result.max_open_size} "
          f"cache_hit_rate={result.primitive_cache_hit_rate:.3f}")
    print(f"  reopened_states={result.reopened_states}  reopen_ratio={reopen_ratio:.4f}")
    print(f"  initial_incumbent_cost={incumbent_cost:.2f}  final_incumbent_cost={result.final_incumbent_cost:.2f}  "
          f"incumbent_updates={result.incumbent_updates}  incumbent_pruned_candidates={result.incumbent_pruned_candidates}")
    first_found = result.first_solution_cost < math.inf
    print(f"  first_solution_found={first_found}  first_solution_cost="
          f"{result.first_solution_cost if first_found else 'inf'}  "
          f"first_solution_expanded={result.first_solution_expanded if first_found else 'n/a'}  "
          f"first_solution_runtime={result.first_solution_runtime_s if first_found else float('nan')}")
    print(f"  current_lower_bound={result.current_lower_bound:.2f}  final_bound_ratio={result.final_bound_ratio:.4f}")
    print(f"  bounded_termination_triggered={result.bounded_termination_triggered}")

    if result.bounded_termination_triggered:
        print(f"  >> %5 CERTIFICATE ACHIEVED: mevcut discretized search graph optimumuna gore en fazla "
              f"%{(result.final_bound_ratio - 1) * 100:.2f} suboptimal.")
    elif first_found:
        gap_pct = (result.final_bound_ratio - 1) * 100 if not math.isnan(result.final_bound_ratio) else float("nan")
        print(f"  >> Solution found but %5 certificate not reached. Certified gap ~= %{gap_pct:.2f} "
              f"(incumbent={result.final_incumbent_cost:.2f}, LB={result.current_lower_bound:.2f}).")
    else:
        print("  >> No internal complete solution found within this cap.")

    # Report path-metrics for whatever the BEST known solution is -- the certified/exact
    # result.path when status=="success", or (Section 8) the internally-found
    # incumbent_path when a first/improved solution exists but the certificate itself
    # didn't complete (result.success alone would incorrectly skip this).
    report_path = result.path if (result.success and result.path) else (
        result.incumbent_path if first_found else []
    )
    report_cost = result.total_cost if (result.success and result.path) else result.final_incumbent_cost

    if report_path:
        # Recompute metrics directly from report_path -- SearchResult's own
        # geometric_path_length/average_aircraft_msl/etc. are only populated when
        # status=="success" (NaN here for a search_limit_reached run), so they can't
        # be trusted for the "first solution found, certificate not yet done" case.
        alt_metrics = _path_altitude_metrics(report_path, tq, cfg)
        min_agl = _path_min_observed_agl(report_path, primitives, tq, cfg)
        reversal_metrics = _path_vertical_reversal_metrics(report_path, primitives, cfg)
        profile = compute_vertical_profile_metrics(report_path, primitives, tq, cfg)
        decomp = decompose_cost(report_path, primitives, tq, cfg)
        safety = verify_path_safety(report_path, primitives, tq, cfg)
        last_climb = last_climb_start_distance(report_path, primitives, cfg)
        improvement_pct = (1.0 - report_cost / incumbent_cost) * 100.0 if incumbent_cost < math.inf else float("nan")
        print(f"  final_total_cost={report_cost:.2f}  improvement_vs_direct_incumbent={improvement_pct:.2f}%")
        print(f"  geometric_path_length={alt_metrics['geometric_path_length']:.1f}  "
              f"avg_MSL={alt_metrics['average_aircraft_msl']:.1f} min_MSL={alt_metrics['minimum_aircraft_msl']:.1f} "
              f"min_AGL={min_agl:.1f}")
        print(f"  total_climb={alt_metrics['total_climb_m']:.1f} total_descent={alt_metrics['total_descent_m']:.1f} "
              f"reversal_count={reversal_metrics['total_vertical_reversal_count']}")
        print(f"  cost decomposition: G={decomp['G']:.1f} M={decomp['M']:.1f} R={decomp['R']:.2f}")
        print(f"  last_climb_start_distance_m={last_climb}")
        print(f"  SAFETY CHECK: {'PASS' if safety['ok'] else 'FAIL -- ' + safety.get('reason', '')} "
              f"(min_agl={safety.get('min_agl', 'n/a')}, max_primitive_angle={safety.get('max_angle_deg', 'n/a')})")
        print(f"  ascii: {ascii_profile(profile, profile['total_horizontal_distance_m'])}")
    print()
    return result, reopen_ratio


def main() -> None:
    cfg = dataclasses.replace(DEFAULT_CONFIG, msl_cost_weight=W_MSL)
    primitives = build_primitive_set(cfg)

    roi = load_roi(cfg)
    tq = TerrainQuery(roi)

    seg = roi.elevation[START_ROW:GOAL_ROW + 1, START_COL]
    seg_min = float(seg.min())

    z0 = msl_to_z_index(AIRCRAFT_MSL, cfg)
    start, goal = (START_ROW, START_COL, z0), (GOAL_ROW, GOAL_COL, z0)

    min_search = math.ceil((seg_min + cfg.min_agl_m) / cfg.z_step_m) * cfg.z_step_m
    max_search = AIRCRAFT_MSL + 20.0

    print(f"search bounds: min={min_search} max={max_search} "
          f"({(max_search - min_search) / cfg.z_step_m:.0f} z-steps)")
    print(f"w_MSL={W_MSL}, target_suboptimality={TARGET_SUBOPTIMALITY}, dominance_pruning=OFF, "
          f"incumbent_pruning=ON, cache=ON, cap={MAX_EXPANSIONS} (2 runs, no retry)")
    print()

    direct_level_path = [(r, START_COL, z0) for r in range(START_ROW, GOAL_ROW + 1)]
    ok, incumbent_cost = validate_and_cost_path(direct_level_path, primitives, tq, cfg)
    print(f"initial incumbent (direct 216-edge level chain): valid={ok} "
          f"edges={len(direct_level_path) - 1} cost={incumbent_cost:.2f}")
    if not ok:
        print("!! Direct level path rejected -- falling back to incumbent_cost=inf")
        incumbent_cost, direct_level_path = math.inf, None
    print()

    rows = [dict(STAGE25_REFERENCE, reopen_ratio=STAGE25_REFERENCE["reopened_states"] / STAGE25_REFERENCE["expanded_nodes"])]
    for eps in EPSILON_VALUES:
        result, reopen_ratio = run_one(eps, cfg, primitives, tq, start, goal, min_search, max_search,
                                        incumbent_cost, direct_level_path)
        rows.append({
            "epsilon_search": eps, "status": result.status, "expanded_nodes": result.expanded_nodes,
            "runtime_s": result.runtime_s, "max_open_size": result.max_open_size,
            "cache_hit_rate": result.primitive_cache_hit_rate, "reopened_states": result.reopened_states,
            "reopen_ratio": reopen_ratio, "first_solution_cost": result.first_solution_cost,
            "final_incumbent_cost": result.final_incumbent_cost, "current_lower_bound": result.current_lower_bound,
            "final_bound_ratio": result.final_bound_ratio,
        })

    print("=== COMPARISON TABLE ===")
    header = f"{'eps':>5} {'status':>19} {'expanded':>8} {'runtime':>8} {'max_open':>8} {'reopened':>9} {'reopen%':>8} {'1st_sol':>8} {'incumbent':>10} {'LB':>10} {'ratio':>7}"
    print(header)
    for r in rows:
        first_sol = "yes" if r["first_solution_cost"] < math.inf else "no"
        print(f"{r['epsilon_search']:>5} {r['status']:>19} {r['expanded_nodes']:>8} {r['runtime_s']:>8.2f} "
              f"{r['max_open_size']:>8} {r['reopened_states']:>9} {r['reopen_ratio'] * 100:>7.1f}% {first_sol:>8} "
              f"{r['final_incumbent_cost']:>10.2f} {r['current_lower_bound']:>10.2f} {r['final_bound_ratio']:>7.4f}")


if __name__ == "__main__":
    main()
