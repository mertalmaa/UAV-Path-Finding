"""Stage 19: 6.48 km real-terrain HIGH -> VALLEY -> HIGH benchmark.

No architecture change. Same production system as Stage 17/18: primitive
cache, MSL-aware admissible heuristic, spacing-sensitive reversal model,
z_step=20m, max climb/descent angle=10 deg, min_agl_m=200 (all unchanged).
w_MSL=0.63 for this run only (config default is NOT changed).

Coordinates (planner 10x10km ROI row/col, verified against the real DEM
before any search was attempted):
    START row=48,  col=276  terrain~3530.5m
    GOAL  row=264, col=276  terrain~3539.5m
    straight-line horizontal distance = 6480.0m (exact)
    intervening terrain: min=3036.3m max=3556.8m mean=3207.8m, 0 NoData

Runs dominance pruning both OFF (per the written spec) and ON (requested
ad hoc), for comparison.
"""
import dataclasses
import math
import time

from planner.astar import astar_search, msl_to_z_index, state_to_xyz
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set, evaluate_primitive
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from scripts.calibrate_low_msl_behavior import ascii_profile, compute_vertical_profile_metrics, decompose_cost

START_ROW, START_COL = 48, 276
GOAL_ROW, GOAL_COL = 264, 276
AIRCRAFT_MSL = 3760.0
W_MSL = 0.63


def terrain_ascii(seg_min, seg_max, start_t, goal_t) -> str:
    return (
        f"START ~{start_t:.0f}\n"
        f"   \\\n"
        f"    \\______________\n"
        f"       valley ~{seg_min:.0f}\n"
        f"                     /\n"
        f"                    /\n"
        f"               GOAL ~{goal_t:.0f}\n"
        f"  (max intervening terrain ~{seg_max:.0f})"
    )


def last_climb_start_distance(path, primitives, config) -> float:
    by_delta = {(p.drow, p.dcol, round(p.dz_m / config.z_step_m)): p for p in primitives}
    cum = 0.0
    last_start = None
    for (r1, c1, z1), (r2, c2, z2) in zip(path, path[1:]):
        prim = by_delta.get((r2 - r1, c2 - c1, z2 - z1))
        if prim.primitive_type == "climb":
            last_start = cum
        cum += prim.horizontal_distance_m
    return last_start


def verify_path_safety(path, primitives, terrain, config) -> dict:
    by_delta = {(p.drow, p.dcol, round(p.dz_m / config.z_step_m)): p for p in primitives}
    max_angle = 0.0
    min_agl = math.inf
    for (r1, c1, z1), (r2, c2, z2) in zip(path, path[1:]):
        prim = by_delta.get((r2 - r1, c2 - c1, z2 - z1))
        if prim is None:
            return {"ok": False, "reason": "no matching primitive for path edge"}
        start_xyz = state_to_xyz((r1, c1, z1), terrain, config)
        result = evaluate_primitive(start_xyz, prim, terrain, config)
        if not result.valid:
            return {"ok": False, "reason": f"edge invalid: {result.reason}"}
        min_agl = min(min_agl, result.min_agl_m)
        angle = math.degrees(math.atan2(abs(prim.dz_m), prim.horizontal_distance_m)) if prim.horizontal_distance_m else 0.0
        max_angle = max(max_angle, angle)
    return {"ok": True, "min_agl": min_agl, "max_angle_deg": max_angle}


def run_one(label, use_dominance, cfg, primitives, tq, start, goal, min_search, max_search, max_expansions):
    t0 = time.perf_counter()
    result = astar_search(start, goal, tq, min_search_altitude_msl=min_search, max_search_altitude_msl=max_search,
                           config=cfg, primitives=primitives, max_expansions=max_expansions,
                           use_primitive_cache=True, use_dominance_pruning=use_dominance,
                           use_msl_lower_bound_heuristic=True)
    wall = time.perf_counter() - t0

    print(f"=== {label} (dominance={'ON' if use_dominance else 'OFF'}) ===")
    print(f"  status={result.status} wall={wall:.2f}s runtime={result.runtime_s:.2f}s "
          f"expanded={result.expanded_nodes} max_open={result.max_open_size} "
          f"cache_hit_rate={result.primitive_cache_hit_rate:.3f} multiplier={result.heuristic_cost_multiplier:.4f}")
    if use_dominance:
        print(f"  dominance: checks={result.dominance_checks} pruned={result.dominance_pruned_candidates} "
              f"pop_skipped={result.dominated_heap_pops_skipped} max_frontier={result.max_dominance_frontier_size}")
    if result.runtime_s > 30.0 or result.status == "search_limit_reached":
        print(f"  !! FLAG: runtime={result.runtime_s:.1f}s status={result.status}")

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
          f"({profile['low_msl_dwell_distance_m'] / 1000:.2f} km)")
    print(f"  low_msl_dwell_ratio={profile['low_msl_dwell_ratio']:.3f}")
    print(f"  last_climb_start_distance_m={last_climb}")
    print(f"  SAFETY CHECK: {'PASS' if safety['ok'] else 'FAIL -- ' + safety.get('reason','')} "
          f"(min_agl={safety.get('min_agl', 'n/a')}, max_primitive_angle={safety.get('max_angle_deg', 'n/a')})")
    print(f"  aircraft profile: START {AIRCRAFT_MSL:.0f} -> min {result.minimum_aircraft_msl:.0f} -> "
          f"GOAL {AIRCRAFT_MSL:.0f}")
    print(f"  ascii: {ascii_profile(profile, profile['total_horizontal_distance_m'])}")
    print()
    return result, profile, decomp, safety


def main() -> None:
    cfg_base = DEFAULT_CONFIG
    cfg = dataclasses.replace(cfg_base, msl_cost_weight=W_MSL)
    primitives = build_primitive_set(cfg)

    roi = load_roi(cfg)
    tq = TerrainQuery(roi)

    seg = roi.elevation[START_ROW:GOAL_ROW + 1, START_COL]
    seg_min, seg_max, seg_mean = float(seg.min()), float(seg.max()), float(seg.mean())
    start_t = float(roi.elevation[START_ROW, START_COL])
    goal_t = float(roi.elevation[GOAL_ROW, START_COL])
    sx, sy = tq.rowcol_to_xy(START_ROW, START_COL)
    gx, gy = tq.rowcol_to_xy(GOAL_ROW, GOAL_COL)
    dist = math.hypot(gx - sx, gy - sy)

    print("=== Coordinate / terrain verification ===")
    print(f"  start=({START_ROW},{START_COL}) x={sx:.1f} y={sy:.1f} terrain={start_t:.1f}m")
    print(f"  goal=({GOAL_ROW},{GOAL_COL}) x={gx:.1f} y={gy:.1f} terrain={goal_t:.1f}m")
    print(f"  horizontal_distance={dist:.1f}m")
    print(f"  intervening terrain: min={seg_min:.1f} max={seg_max:.1f} mean={seg_mean:.1f} "
          f"relief={seg_max - seg_min:.1f}  NoData_count={int((seg == roi.nodata).sum()) if roi.nodata is not None else 0}")
    print()
    print(terrain_ascii(seg_min, seg_max, start_t, goal_t))
    print()

    agl_start = AIRCRAFT_MSL - start_t
    agl_goal = AIRCRAFT_MSL - goal_t
    agl_worst = AIRCRAFT_MSL - seg_max
    print(f"  AIRCRAFT_MSL={AIRCRAFT_MSL}: AGL@start={agl_start:.1f} AGL@goal={agl_goal:.1f} "
          f"AGL@worst_intervening={agl_worst:.1f}  (all must be >= {cfg.min_agl_m})")
    assert agl_start >= cfg.min_agl_m and agl_goal >= cfg.min_agl_m and agl_worst >= cfg.min_agl_m
    print()

    z0 = msl_to_z_index(AIRCRAFT_MSL, cfg)
    start, goal = (START_ROW, START_COL, z0), (GOAL_ROW, GOAL_COL, z0)

    min_search = math.ceil((seg_min + cfg.min_agl_m) / cfg.z_step_m) * cfg.z_step_m
    max_search = AIRCRAFT_MSL + 20.0
    print(f"  search bounds: min_search_altitude_msl={min_search} max_search_altitude_msl={max_search} "
          f"({(max_search - min_search) / cfg.z_step_m:.0f} z-steps) -- PROTOTYPE search bound, not an aircraft ceiling")
    print(f"  w_MSL={W_MSL} (this run only -- config default unchanged), z_step={cfg.z_step_m}, "
          f"max_climb/descent_angle={cfg.max_climb_angle_deg}/{cfg.max_descent_angle_deg} deg, "
          f"min_agl_m={cfg.min_agl_m}")
    print()

    run_one("Long valley benchmark", False, cfg, primitives, tq, start, goal, min_search, max_search, 30_000)
    run_one("Long valley benchmark", True, cfg, primitives, tq, start, goal, min_search, max_search, 30_000)


if __name__ == "__main__":
    main()
