"""Stage 23: real 6.48km Aladagalar benchmark, 3-bucket state + exact
dominance pruning ON, single 30,000-expansion-cap run (no retry -- per
spec, the 150k baselines are already recorded in project.md).

Same coordinates/aircraft/w_MSL as Stage 19-22: START row=48,col=276
GOAL row=264,col=276 AIRCRAFT_MSL=3760, distance=6480.0m.

Settings: dominance_pruning=ON (the only change from Stage 22's
dominance-OFF 30k run), cache ON, msl_lower_bound_heuristic ON,
vertical_reachability_heuristic OFF, w_MSL=0.63, current 10 deg
primitives, min_agl_m=200 (all unchanged).
"""
import dataclasses
import math
import time

from planner.astar import astar_search, msl_to_z_index
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
    print(f"w_MSL={W_MSL}, dominance_pruning=ON, msl_lower_bound_heuristic=ON, "
          f"vertical_reachability_heuristic=OFF, cache=ON, cap={MAX_EXPANSIONS} (single run, no retry)")
    print()

    t0 = time.perf_counter()
    result = astar_search(
        start, goal, tq, min_search_altitude_msl=min_search, max_search_altitude_msl=max_search,
        config=cfg, primitives=primitives, max_expansions=MAX_EXPANSIONS,
        use_primitive_cache=True, use_dominance_pruning=True,
        use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False,
    )
    wall = time.perf_counter() - t0

    print(f"status={result.status} wall={wall:.2f}s runtime={result.runtime_s:.2f}s "
          f"expanded={result.expanded_nodes} max_open={result.max_open_size} "
          f"cache_hit_rate={result.primitive_cache_hit_rate:.3f} multiplier={result.heuristic_cost_multiplier:.4f}")
    print(f"dominance: checks={result.dominance_checks} pruned={result.dominance_pruned_candidates} "
          f"frontier_removed={result.dominance_frontier_entries_removed} "
          f"pop_skipped={result.dominated_heap_pops_skipped} "
          f"max_frontier={result.max_dominance_frontier_size} "
          f"avg_frontier={result.average_dominance_frontier_size:.3f}")

    if not result.success:
        print()
        print("SEARCH_LIMIT_REACHED -- no path metrics to report (matches Stage 19-22's same limitation).")
        return

    profile = compute_vertical_profile_metrics(result.path, primitives, tq, cfg)
    decomp = decompose_cost(result.path, primitives, tq, cfg)
    safety = verify_path_safety(result.path, primitives, tq, cfg)
    last_climb = last_climb_start_distance(result.path, primitives, cfg)

    print(f"geometric_path_length={result.geometric_path_length:.1f} total_cost={result.total_cost:.2f}")
    print(f"avg_MSL={result.average_aircraft_msl:.1f} min_MSL={result.minimum_aircraft_msl:.1f} "
          f"max_MSL={result.maximum_aircraft_msl:.1f} min_AGL={result.minimum_observed_agl:.1f}")
    print(f"total_climb={result.total_climb_m:.1f} total_descent={result.total_descent_m:.1f} "
          f"reversal_count={result.total_vertical_reversal_count} "
          f"penalized_reversals={result.penalized_reversal_count} "
          f"total_reversal_penalty={result.total_reversal_penalty:.3f}")
    print(f"cost decomposition: G={decomp['G']:.1f} M={decomp['M']:.1f} R={decomp['R']:.2f}  "
          f"(G+w*M+R={decomp['G'] + W_MSL * decomp['M'] + decomp['R']:.2f} vs total_cost={result.total_cost:.2f})")
    print(f"first_descent_distance_m={profile['first_descent_distance_m']}")
    print(f"deepest_point_distance_from_start_m={profile['deepest_point_distance_from_start_m']:.1f}")
    print(f"descent_depth_m={AIRCRAFT_MSL - result.minimum_aircraft_msl:.1f}")
    print(f"low_msl_dwell_distance_m={profile['low_msl_dwell_distance_m']:.1f} "
          f"({profile['low_msl_dwell_distance_m'] / 1000:.2f} km) ratio={profile['low_msl_dwell_ratio']:.3f}")
    print(f"last_climb_start_distance_m={last_climb}")
    print(f"SAFETY CHECK: {'PASS' if safety['ok'] else 'FAIL -- ' + safety.get('reason', '')} "
          f"(min_agl={safety.get('min_agl', 'n/a')}, max_primitive_angle={safety.get('max_angle_deg', 'n/a')})")
    print(f"ascii: {ascii_profile(profile, profile['total_horizontal_distance_m'])}")


if __name__ == "__main__":
    main()
