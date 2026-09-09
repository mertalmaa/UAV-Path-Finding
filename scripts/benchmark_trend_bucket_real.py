"""Stage 22 real-terrain benchmark: does the 3-bucket trend-age compression
(vs the old 0..10 trend_age_units, see planner/astar.py _next_trend_and_bucket)
shrink the 6.48km HIGH -> VALLEY -> HIGH search that Stage 20/21 could not
converge?

Same coordinates/terrain/aircraft as scripts/benchmark_long_valley.py
(Stage 19) and scripts/validate_vertical_reachability_heuristic.py
(Stage 21): START row=48,col=276  GOAL row=264,col=276  AIRCRAFT_MSL=3760,
distance=6480.0m.

Settings for THIS run (per Stage 22 spec, explicit sign-off from the user):
  use_dominance_pruning=False
  use_msl_lower_bound_heuristic=True   (Stage 17 global heuristic)
  use_vertical_reachability_heuristic=False  (Stage 21 heuristic OFF --
      shown not to help this benchmark, classification C)
  w_MSL=0.63 (this run only, config default unchanged)
  use_primitive_cache=True
  min_agl_m unchanged (200), current 10 deg primitives unchanged

The ONLY thing that changed since Stage 20/21's runs of this exact
benchmark is that astar_search's internal augmented state now uses the
3-bucket trend_age_bucket instead of the 11-value trend_age_units -- same
production astar_search, same cost formula, same primitives/cache/heuristic
code paths.
"""
import dataclasses
import math
import time

from planner.astar import astar_search, msl_to_z_index, state_to_xyz
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set, evaluate_primitive
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from scripts.benchmark_long_valley import verify_path_safety, last_climb_start_distance
from scripts.calibrate_low_msl_behavior import ascii_profile, compute_vertical_profile_metrics, decompose_cost

START_ROW, START_COL = 48, 276
GOAL_ROW, GOAL_COL = 264, 276
AIRCRAFT_MSL = 3760.0
W_MSL = 0.63


def run(cfg, primitives, tq, start, goal, min_search, max_search, max_expansions, label):
    t0 = time.perf_counter()
    result = astar_search(
        start, goal, tq, min_search_altitude_msl=min_search, max_search_altitude_msl=max_search,
        config=cfg, primitives=primitives, max_expansions=max_expansions,
        use_primitive_cache=True, use_dominance_pruning=False,
        use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False,
    )
    wall = time.perf_counter() - t0

    print(f"=== {label} (cap={max_expansions}) ===")
    print(f"  status={result.status} wall={wall:.2f}s runtime={result.runtime_s:.2f}s "
          f"expanded={result.expanded_nodes} max_open={result.max_open_size} "
          f"cache_hit_rate={result.primitive_cache_hit_rate:.3f} multiplier={result.heuristic_cost_multiplier:.4f}")

    if not result.success:
        print()
        return result, None, None, None

    profile = compute_vertical_profile_metrics(result.path, primitives, tq, cfg)
    decomp = decompose_cost(result.path, primitives, tq, cfg)
    safety = verify_path_safety(result.path, primitives, tq, cfg)
    last_climb = last_climb_start_distance(result.path, primitives, cfg)

    print(f"  geometric_path_length={result.geometric_path_length:.1f} total_cost={result.total_cost:.2f}")
    print(f"  avg_MSL={result.average_aircraft_msl:.1f} min_MSL={result.minimum_aircraft_msl:.1f} "
          f"max_MSL={result.maximum_aircraft_msl:.1f} min_AGL={result.minimum_observed_agl:.1f}")
    print(f"  total_climb={result.total_climb_m:.1f} total_descent={result.total_descent_m:.1f} "
          f"reversal_count={result.total_vertical_reversal_count} "
          f"penalized_reversals={result.penalized_reversal_count} "
          f"total_reversal_penalty={result.total_reversal_penalty:.3f}")
    print(f"  cost decomposition: G={decomp['G']:.1f} M={decomp['M']:.1f} R={decomp['R']:.2f}  "
          f"(G+w*M+R={decomp['G'] + W_MSL * decomp['M'] + decomp['R']:.2f} vs total_cost={result.total_cost:.2f})")
    print(f"  first_descent_distance_m={profile['first_descent_distance_m']}")
    print(f"  deepest_point_distance_from_start_m={profile['deepest_point_distance_from_start_m']:.1f}")
    print(f"  descent_depth_m={AIRCRAFT_MSL - result.minimum_aircraft_msl:.1f}")
    print(f"  low_msl_dwell_distance_m={profile['low_msl_dwell_distance_m']:.1f} "
          f"({profile['low_msl_dwell_distance_m'] / 1000:.2f} km)  ratio={profile['low_msl_dwell_ratio']:.3f}")
    print(f"  last_climb_start_distance_m={last_climb}")
    print(f"  SAFETY CHECK: {'PASS' if safety['ok'] else 'FAIL -- ' + safety.get('reason', '')} "
          f"(min_agl={safety.get('min_agl', 'n/a')}, max_primitive_angle={safety.get('max_angle_deg', 'n/a')})")
    print(f"  aircraft profile: START {AIRCRAFT_MSL:.0f} -> min {result.minimum_aircraft_msl:.0f} -> "
          f"GOAL {AIRCRAFT_MSL:.0f}")
    print(f"  ascii: {ascii_profile(profile, profile['total_horizontal_distance_m'])}")
    print()
    return result, profile, decomp, safety


def main() -> None:
    cfg = dataclasses.replace(DEFAULT_CONFIG, msl_cost_weight=W_MSL)
    primitives = build_primitive_set(cfg)

    roi = load_roi(cfg)
    tq = TerrainQuery(roi)

    seg = roi.elevation[START_ROW:GOAL_ROW + 1, START_COL]
    seg_min = float(seg.min())
    start_t = float(roi.elevation[START_ROW, START_COL])
    goal_t = float(roi.elevation[GOAL_ROW, START_COL])
    sx, sy = tq.rowcol_to_xy(START_ROW, START_COL)
    gx, gy = tq.rowcol_to_xy(GOAL_ROW, GOAL_COL)
    dist = math.hypot(gx - sx, gy - sy)

    print("=== Coordinate / terrain verification (same as Stage 19-21) ===")
    print(f"  start=({START_ROW},{START_COL}) terrain={start_t:.1f}m  goal=({GOAL_ROW},{GOAL_COL}) terrain={goal_t:.1f}m")
    print(f"  horizontal_distance={dist:.1f}m  valley_min={seg_min:.1f}m")
    print()

    z0 = msl_to_z_index(AIRCRAFT_MSL, cfg)
    start, goal = (START_ROW, START_COL, z0), (GOAL_ROW, GOAL_COL, z0)

    min_search = math.ceil((seg_min + cfg.min_agl_m) / cfg.z_step_m) * cfg.z_step_m
    max_search = AIRCRAFT_MSL + 20.0
    print(f"  search bounds: min={min_search} max={max_search} "
          f"({(max_search - min_search) / cfg.z_step_m:.0f} z-steps)")
    print(f"  w_MSL={W_MSL}, dominance_pruning=OFF, msl_lower_bound_heuristic=ON, "
          f"vertical_reachability_heuristic=OFF, cache=ON")
    print()

    result, *_ = run(cfg, primitives, tq, start, goal, min_search, max_search, 30_000,
                      "Stage 22 3-bucket -- 6.48km benchmark")

    if result.status == "search_limit_reached":
        print(">> 30,000-cap SEARCH_LIMIT_REACHED -- one permitted retry at 150,000 cap.")
        run(cfg, primitives, tq, start, goal, min_search, max_search, 150_000,
            "Stage 22 3-bucket -- 6.48km benchmark RETRY")


if __name__ == "__main__":
    main()
