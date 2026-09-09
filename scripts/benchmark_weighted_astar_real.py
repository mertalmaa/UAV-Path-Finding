"""Stage 25: real 6.48km Aladagalar benchmark -- Weighted A* (epsilon_search
=1.5) + certified 5%-bounded-suboptimal termination (target_suboptimality=
1.05), ONE main run, 30,000-expansion cap, no retry.

Same coordinates/aircraft/w_MSL as Stage 19-24: START row=48,col=276 GOAL
row=264,col=276 AIRCRAFT_MSL=3760, distance=6480.0m. Initial incumbent: the
same validated direct 216-edge level chain used in Stage 24
(cost~=21829.82, re-validated here, not assumed).

Settings: dominance_pruning=OFF (per spec -- isolate weighted A*'s own
effect), cache ON, msl_lower_bound_heuristic ON, vertical_reachability
_heuristic OFF, 3-bucket state, current 10 deg primitives, min_agl_m=200.
"""
import dataclasses
import math
import time

from planner.astar import astar_search, msl_to_z_index, validate_and_cost_path
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
EPSILON_SEARCH = 1.5
TARGET_SUBOPTIMALITY = 1.05


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
    print(f"w_MSL={W_MSL}, epsilon_search={EPSILON_SEARCH}, target_suboptimality={TARGET_SUBOPTIMALITY}, "
          f"dominance_pruning=OFF, incumbent_pruning=ON, cache=ON, msl_lower_bound_heuristic=ON, "
          f"vertical_reachability_heuristic=OFF, cap={MAX_EXPANSIONS} (1 run, no retry)")
    print()

    direct_level_path = [(r, START_COL, z0) for r in range(START_ROW, GOAL_ROW + 1)]
    ok, incumbent_cost = validate_and_cost_path(direct_level_path, primitives, tq, cfg)
    print(f"initial incumbent (direct 216-edge level chain): valid={ok} "
          f"edges={len(direct_level_path) - 1} cost={incumbent_cost:.2f}")
    if not ok:
        print("!! Direct level path rejected -- falling back to incumbent_cost=inf")
        incumbent_cost, direct_level_path = math.inf, None
    print()

    t0 = time.perf_counter()
    result = astar_search(
        start, goal, tq, min_search_altitude_msl=min_search, max_search_altitude_msl=max_search,
        config=cfg, primitives=primitives, max_expansions=MAX_EXPANSIONS,
        use_primitive_cache=True, use_dominance_pruning=False,
        use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False,
        use_incumbent_pruning=True, initial_incumbent_cost=incumbent_cost, initial_incumbent_path=direct_level_path,
        epsilon_search=EPSILON_SEARCH, target_suboptimality=TARGET_SUBOPTIMALITY,
    )
    wall = time.perf_counter() - t0

    print(f"=== MAIN RUN (cap={MAX_EXPANSIONS}) ===")
    print(f"  status={result.status} wall={wall:.2f}s runtime={result.runtime_s:.2f}s "
          f"expanded={result.expanded_nodes} max_open={result.max_open_size} "
          f"cache_hit_rate={result.primitive_cache_hit_rate:.3f} multiplier={result.heuristic_cost_multiplier:.4f} "
          f"reopened_states={result.reopened_states}")
    print()
    print("  --- A) FIRST IMPROVED SOLUTION (vs initial incumbent) ---")
    if result.first_solution_cost < math.inf:
        improved = result.first_solution_cost < incumbent_cost - 1e-6
        print(f"  first_solution_cost={result.first_solution_cost:.2f}  "
              f"first_solution_expanded={result.first_solution_expanded}  "
              f"first_solution_runtime_s={result.first_solution_runtime_s:.2f}s")
        print(f"  improved over initial incumbent ({incumbent_cost:.2f})? {improved}  "
              f"improvement={(1 - result.first_solution_cost / incumbent_cost) * 100:.2f}%")
    else:
        print("  No internal solution found at all within the 30k cap.")
    print()
    print("  --- B) CERTIFIED 5% SOLUTION ---")
    print(f"  bounded_termination_triggered={result.bounded_termination_triggered}")
    print(f"  final_incumbent_cost={result.final_incumbent_cost:.2f}  "
          f"current_lower_bound={result.current_lower_bound:.2f}  "
          f"final_bound_ratio={result.final_bound_ratio:.4f}  incumbent_updates={result.incumbent_updates}")
    if result.bounded_termination_triggered:
        print(f"  CERTIFICATE ACHIEVED: solution is provably within {(result.final_bound_ratio - 1) * 100:.2f}% "
              f"of optimal (target was <=5%).")
    else:
        certified_gap_pct = (result.final_bound_ratio - 1) * 100 if not math.isnan(result.final_bound_ratio) else float("nan")
        print(f"  Certificate NOT reached within {MAX_EXPANSIONS} expansions. "
              f"Currently provable gap: {certified_gap_pct:.2f}% "
              f"(best incumbent={result.final_incumbent_cost:.2f}, current LB={result.current_lower_bound:.2f}).")
    print()

    if result.success and result.path:
        profile = compute_vertical_profile_metrics(result.path, primitives, tq, cfg)
        decomp = decompose_cost(result.path, primitives, tq, cfg)
        safety = verify_path_safety(result.path, primitives, tq, cfg)
        last_climb = last_climb_start_distance(result.path, primitives, cfg)
        improvement_pct = (1.0 - result.total_cost / incumbent_cost) * 100.0 if incumbent_cost < math.inf else float("nan")

        print(f"  final_total_cost={result.total_cost:.2f}  initial_incumbent_cost={incumbent_cost:.2f}  "
              f"improvement={improvement_pct:.2f}%")
        print(f"  geometric_path_length={result.geometric_path_length:.1f}")
        print(f"  avg_MSL={result.average_aircraft_msl:.1f} min_MSL={result.minimum_aircraft_msl:.1f} "
              f"max_MSL={result.maximum_aircraft_msl:.1f} min_AGL={result.minimum_observed_agl:.1f}")
        print(f"  total_climb={result.total_climb_m:.1f} total_descent={result.total_descent_m:.1f} "
              f"reversal_count={result.total_vertical_reversal_count} "
              f"total_reversal_penalty={result.total_reversal_penalty:.3f}")
        print(f"  cost decomposition: G={decomp['G']:.1f} M={decomp['M']:.1f} R={decomp['R']:.2f}")
        print(f"  first_descent_distance_m={profile['first_descent_distance_m']}")
        print(f"  low_msl_dwell_distance_m={profile['low_msl_dwell_distance_m']:.1f} "
              f"({profile['low_msl_dwell_distance_m'] / 1000:.2f} km) ratio={profile['low_msl_dwell_ratio']:.3f}")
        print(f"  last_climb_start_distance_m={last_climb}")
        print(f"  SAFETY CHECK: {'PASS' if safety['ok'] else 'FAIL -- ' + safety.get('reason', '')} "
              f"(min_agl={safety.get('min_agl', 'n/a')}, max_primitive_angle={safety.get('max_angle_deg', 'n/a')})")
        print(f"  ascii: {ascii_profile(profile, profile['total_horizontal_distance_m'])}")
    else:
        print("  Overall status is not a certified/exact success -- no final path-metrics block "
              "(best incumbent numbers above are still the honest answer).")


if __name__ == "__main__":
    main()
