"""Stage 24: real 6.48km Aladagalar benchmark with incumbent upper-bound
pruning ON, comparing dominance OFF vs dominance ON (exactly two new runs,
30,000-expansion cap each, no retry -- the 150k baselines from Stage
20-23 are already recorded in project.md and are NOT re-run here).

Initial incumbent: the direct 216-edge level chain (row 48->264, col=276
fixed, z=3760m constant) that Stage 20 already proved is 216/216 VALID
with min_AGL~=203.2m -- but re-validated here through
planner.astar.validate_and_cost_path() (the real production safety +
cost authority), not assumed.

Settings (both runs): 3-bucket state, primitive cache ON, global MSL
lower-bound heuristic ON, vertical-reachability heuristic OFF, w_MSL=0.63,
current 10 deg primitives, min_agl_m=200, z_step=20m, incumbent pruning ON.
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


def run(label, use_dominance, cfg, primitives, tq, start, goal, min_search, max_search,
        incumbent_cost, incumbent_path):
    t0 = time.perf_counter()
    result = astar_search(
        start, goal, tq, min_search_altitude_msl=min_search, max_search_altitude_msl=max_search,
        config=cfg, primitives=primitives, max_expansions=MAX_EXPANSIONS,
        use_primitive_cache=True, use_dominance_pruning=use_dominance,
        use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False,
        use_incumbent_pruning=True, initial_incumbent_cost=incumbent_cost, initial_incumbent_path=incumbent_path,
    )
    wall = time.perf_counter() - t0

    print(f"=== {label} (dominance={'ON' if use_dominance else 'OFF'}, cap={MAX_EXPANSIONS}) ===")
    print(f"  status={result.status} wall={wall:.2f}s runtime={result.runtime_s:.2f}s "
          f"expanded={result.expanded_nodes} max_open={result.max_open_size} "
          f"cache_hit_rate={result.primitive_cache_hit_rate:.3f} multiplier={result.heuristic_cost_multiplier:.4f}")
    print(f"  incumbent: initial_available={result.initial_incumbent_available} "
          f"initial_cost={result.initial_incumbent_cost:.2f} final_cost={result.final_incumbent_cost:.2f} "
          f"updates={result.incumbent_updates} pruned_candidates={result.incumbent_pruned_candidates} "
          f"heap_pops_skipped={result.incumbent_heap_pops_skipped} "
          f"termination_triggered={result.incumbent_termination_triggered}")
    if use_dominance:
        print(f"  dominance: checks={result.dominance_checks} pruned={result.dominance_pruned_candidates} "
              f"frontier_removed={result.dominance_frontier_entries_removed} "
              f"pop_skipped={result.dominated_heap_pops_skipped} max_frontier={result.max_dominance_frontier_size} "
              f"avg_frontier={result.average_dominance_frontier_size:.3f}")

    if not result.success:
        print()
        return result

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
    print()
    return result


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
    print(f"w_MSL={W_MSL}, incumbent_pruning=ON, cache=ON, msl_lower_bound_heuristic=ON, "
          f"vertical_reachability_heuristic=OFF, cap={MAX_EXPANSIONS} (2 runs, no retry)")
    print()

    # Initial incumbent: direct level chain, row 48..264, col=276, z=3760m constant.
    direct_level_path = [(r, START_COL, z0) for r in range(START_ROW, GOAL_ROW + 1)]
    ok, incumbent_cost = validate_and_cost_path(direct_level_path, primitives, tq, cfg)
    print(f"initial incumbent (direct 216-edge level chain): valid={ok} "
          f"edges={len(direct_level_path) - 1} cost={incumbent_cost:.2f}")
    if not ok:
        print("!! Direct level path rejected by validate_and_cost_path -- falling back to incumbent_cost=inf")
        incumbent_cost, direct_level_path = math.inf, None
    print()

    run("RUN A", False, cfg, primitives, tq, start, goal, min_search, max_search, incumbent_cost, direct_level_path)
    run("RUN B", True, cfg, primitives, tq, start, goal, min_search, max_search, incumbent_cost, direct_level_path)


if __name__ == "__main__":
    main()
