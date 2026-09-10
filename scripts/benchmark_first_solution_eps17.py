"""Stage 37.4: First-Solution Speed + Route Quality -- epsilon_search=1.7,
freeze_history=True, 3D corridor (XY +/-300m, Z +/-200m), stop_on_first_
solution=True (new Stage 37.4 flag: break at the very first complete safe
path, never pursue the %5 certificate). ONE run only. Stage 37.3's own
epsilon=1.7 "refined" result (which searched all the way to 30k
expansions) is NOT re-run -- project.md's recorded numbers are the
reference for that comparison.
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
from planner.corridor import build_z_guide_grid
from planner.primitives import build_primitive_set, evaluate_primitive
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from scripts.benchmark_long_valley import verify_path_safety
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
EPSILON_SEARCH = 1.70
TARGET_SUBOPTIMALITY = 1.05
GOAL_TOLERANCE_XY_M = 35.0
GOAL_TOLERANCE_Z_M = 25.0
XY_MASK_NPY = "outputs/stage36_corridor_mask.npy"
COARSE_PATH_CSV = "outputs/stage36_coarse_path_eps1.5.csv"
Z_TOLERANCE_M = 200.0
OUTPUT_CSV = "outputs/stage37_4_first_solution_eps17.csv"

# Reference numbers from project.md -- NOT re-derived/re-run here.
DIRECT_LEVEL_COST = 1.6500
DEEP_CANDIDATE_COST = 1.3714
DEEP_CANDIDATE_MIN_MSL = 3400.0
STAGE37_3_REFINED_EPS17 = {
    "cost": 1.497221, "length_3d": 6534.4, "min_msl": 3520.0, "min_agl": 200.8, "path_nodes": 63,
}


def vertical_smoothness(path, primitives, config):
    """Diagnostic only -- no history/cost added. Reversal/spacing reuse
    _path_vertical_reversal_metrics (replays the real (trend,bucket)
    machine fresh from the path's own geometry, independent of the
    search's own freeze_history internals). Longest-continuous-run and
    total-variation are computed directly here from the path's own
    primitive sequence."""
    by_delta = {(p.drow, p.dcol, round(p.dz_m / config.z_step_m)): p for p in primitives}
    reversal_metrics = _path_vertical_reversal_metrics(path, primitives, config)

    longest_climb_run = 0.0
    longest_descent_run = 0.0
    current_run_type = None
    current_run_total = 0.0
    total_climb = 0.0
    total_descent = 0.0

    for (r1, c1, z1), (r2, c2, z2) in zip(path, path[1:]):
        prim = by_delta.get((r2 - r1, c2 - c1, z2 - z1))
        if prim is None:
            continue
        if prim.dz_m > 0:
            kind = "climb"
            total_climb += prim.dz_m
        elif prim.dz_m < 0:
            kind = "descent"
            total_descent += -prim.dz_m
        else:
            kind = "level"

        if kind in ("climb", "descent"):
            if kind == current_run_type:
                current_run_total += abs(prim.dz_m)
            else:
                current_run_type = kind
                current_run_total = abs(prim.dz_m)
            if current_run_type == "climb":
                longest_climb_run = max(longest_climb_run, current_run_total)
            else:
                longest_descent_run = max(longest_descent_run, current_run_total)
        # level primitives don't reset a standing climb/descent run (matches
        # the production trend model's own "level never resets" rule) but
        # also don't add to it -- current_run_total stays as-is.

    return {
        "reversal_count": reversal_metrics["total_vertical_reversal_count"],
        "short_reversal_count": reversal_metrics["penalized_reversal_count"],  # spacing < 300m -> non-zero penalty
        "longest_continuous_climb_m": longest_climb_run,
        "longest_continuous_descent_m": longest_descent_run,
        "total_vertical_variation_m": total_climb + total_descent,
    }


def fine_replay(path, primitives, terrain, config):
    """Re-validate every edge of the found path against the real fine
    terrain via evaluate_primitive -- same production safety authority,
    reused, not reimplemented."""
    violations = []
    min_agl = math.inf
    max_angle = 0.0
    for (r1, c1, z1), (r2, c2, z2) in zip(path, path[1:]):
        by_delta = {(p.drow, p.dcol, round(p.dz_m / config.z_step_m)): p for p in primitives}
        prim = by_delta.get((r2 - r1, c2 - c1, z2 - z1))
        if prim is None:
            violations.append({"reason": "no_matching_primitive", "edge": ((r1, c1, z1), (r2, c2, z2))})
            continue
        start_xyz = state_to_xyz((r1, c1, z1), terrain, config)
        result = evaluate_primitive(start_xyz, prim, terrain, config)
        if not result.valid:
            violations.append({"reason": result.reason, "edge": ((r1, c1, z1), (r2, c2, z2))})
        else:
            min_agl = min(min_agl, result.min_agl_m)
        angle = math.degrees(math.atan2(abs(prim.dz_m), prim.horizontal_distance_m)) if prim.horizontal_distance_m else 0.0
        max_angle = max(max_angle, angle)
    return {"violations": violations, "min_agl": min_agl, "max_angle": max_angle}


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
    xy_mask = np.load(XY_MASK_NPY)

    coarse_path_xyz = []
    with open(COARSE_PATH_CSV) as f:
        for row in csv.DictReader(f):
            coarse_path_xyz.append((float(row["x"]), float(row["y"]), float(row["z_msl"])))
    z_guide_grid = build_z_guide_grid(roi, coarse_path_xyz)

    z0 = msl_to_z_index(AIRCRAFT_MSL, cfg)
    start, goal = (START_ROW, START_COL, z0), (GOAL_ROW, GOAL_COL, z0)
    for r, c in [(START_ROW, START_COL), (GOAL_ROW, GOAL_COL)]:
        diff = abs(AIRCRAFT_MSL - float(z_guide_grid[r, c]))
        if not (bool(xy_mask[r, c]) and diff <= Z_TOLERANCE_M):
            z_guide_grid[r, c] = AIRCRAFT_MSL
            xy_mask[r, c] = True

    seg = roi.elevation[START_ROW:GOAL_ROW + 1, START_COL]
    seg_min = float(seg.min())
    min_search = math.ceil((seg_min + cfg.min_agl_m) / cfg.z_step_m) * cfg.z_step_m
    max_search = AIRCRAFT_MSL + 20.0
    d_ref = compute_distance_reference(start, goal, tq, cfg)

    direct_level_path = build_direct_level(z0)
    ok, incumbent_cost = validate_and_cost_path(direct_level_path, primitives, tq, cfg, d_ref)
    print(f"initial incumbent (direct 216-edge level chain): valid={ok} cost={incumbent_cost:.6f}")
    if not ok:
        print("!! Direct level path rejected -- aborting.")
        return

    print(f"\n=== Stage 37.4: epsilon_search={EPSILON_SEARCH}, freeze_history=True, "
          f"stop_on_first_solution=True (SINGLE run) ===\n")

    t0 = time.perf_counter()
    result = astar_search(
        start, goal, tq, min_search_altitude_msl=min_search, max_search_altitude_msl=max_search,
        config=cfg, primitives=primitives, max_expansions=MAX_EXPANSIONS,
        use_primitive_cache=True, use_dominance_pruning=False,
        use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False,
        use_incumbent_pruning=True, initial_incumbent_cost=incumbent_cost,
        initial_incumbent_path=direct_level_path,
        epsilon_search=EPSILON_SEARCH, target_suboptimality=TARGET_SUBOPTIMALITY,
        corridor_mask=xy_mask, z_guide_grid=z_guide_grid, z_guide_tolerance_m=Z_TOLERANCE_M,
        freeze_history=True, stop_on_first_solution=True,
    )
    wall = time.perf_counter() - t0

    print("=== SEARCH ===")
    print(f"  status={result.status}  first_solution_expanded={result.expanded_nodes}  "
          f"runtime_to_first_solution={wall:.4f}s (internal timer={result.runtime_s:.4f}s)")
    print(f"  max_open={result.max_open_size}  reopened_states_before_first_solution={result.reopened_states}  "
          f"generated_states={result.generated_neighbors}  cache_hit_rate={result.primitive_cache_hit_rate:.3f}")

    if not result.success:
        print("\nFAIL -- stop_on_first_solution=True but no complete path was found at all. "
              "(Unexpected given Stage 37.3's own eps=1.7 result -- reporting honestly.)")
        print(f"  closest_distance_to_goal_region_m={result.closest_distance_to_goal_region_m:.2f}")
        return

    path = result.path
    xyz = [state_to_xyz(s, tq, cfg) for s in path]

    # SAFETY.
    replay = fine_replay(path, primitives, tq, cfg)
    safety = verify_path_safety(path, primitives, tq, cfg)
    min_agl_direct = _path_min_observed_agl(path, primitives, tq, cfg)

    # DISTANCE.
    xy_length = sum(math.hypot(xyz[i + 1][0] - xyz[i][0], xyz[i + 1][1] - xyz[i][1]) for i in range(len(xyz) - 1))
    length_3d = sum(math.sqrt((xyz[i + 1][0] - xyz[i][0]) ** 2 + (xyz[i + 1][1] - xyz[i][1]) ** 2
                               + (xyz[i + 1][2] - xyz[i][2]) ** 2) for i in range(len(xyz) - 1))
    direct_distance = d_ref
    detour_pct = (xy_length / direct_distance - 1.0) * 100.0

    # LOW-MSL QUALITY.
    alt_metrics = _path_altitude_metrics(path, tq, cfg)
    total_distance_component = 0.0
    total_altitude_component = 0.0
    for i in range(len(xyz) - 1):
        x1, y1, z1 = xyz[i]
        x2, y2, z2 = xyz[i + 1]
        geom = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
        dC_distance = geom / d_ref
        mean_msl = (z1 + z2) / 2.0
        excess = max(0.0, mean_msl - ALTITUDE_REFERENCE_MSL)
        dC_altitude = dC_distance * (excess / NORMALIZED_ALTITUDE_SCALE_M)
        total_distance_component += NORMALIZED_W_DISTANCE * dC_distance
        total_altitude_component += NORMALIZED_W_ALTITUDE * dC_altitude

    # VERTICAL SMOOTHNESS.
    smoothness = vertical_smoothness(path, primitives, cfg)

    print("\n=== SAFETY ===")
    print(f"  min_AGL={min_agl_direct:.2f}m  max_flight_path_angle={replay['max_angle']:.2f} deg")
    print(f"  NoData/bounds violations={len(replay['violations'])}")
    print(f"  fine DEM replay: {'PASS' if len(replay['violations']) == 0 else 'FAIL'}")
    print(f"  SAFETY CHECK (verify_path_safety): {'PASS' if safety['ok'] else 'FAIL -- ' + safety.get('reason', '')}")

    print("\n=== DISTANCE ===")
    print(f"  xy_length={xy_length:.1f}m  3d_length={length_3d:.1f}m  direct_distance={direct_distance:.1f}m  "
          f"detour={detour_pct:.2f}%")

    print("\n=== LOW-MSL QUALITY ===")
    print(f"  min_MSL={alt_metrics['minimum_aircraft_msl']:.1f}  max_MSL={alt_metrics['maximum_aircraft_msl']:.1f}  "
          f"distance_weighted_mean_MSL={alt_metrics['average_aircraft_msl']:.1f}")
    print(f"  total_descent={alt_metrics['total_descent_m']:.1f}  total_climb={alt_metrics['total_climb_m']:.1f}")
    print(f"  normalized_distance_component={total_distance_component:.6f}  "
          f"normalized_altitude_component={total_altitude_component:.6f}")
    print(f"  total_diagnostic_cost={result.total_cost:.6f}  (reversal component forced to 0.0 -- freeze_history)")

    print("\n=== VERTICAL SMOOTHNESS DIAGNOSTIC (measured only, not added to cost) ===")
    print(f"  vertical_reversal_count={smoothness['reversal_count']}")
    print(f"  short_reversal_count (<300m separation)={smoothness['short_reversal_count']}")
    print(f"  longest_continuous_descent_m={smoothness['longest_continuous_descent_m']:.1f}")
    print(f"  longest_continuous_climb_m={smoothness['longest_continuous_climb_m']:.1f}")
    print(f"  total_vertical_variation_m={smoothness['total_vertical_variation_m']:.1f}")

    print("\n=== REFERENCE COMPARISON (project.md numbers, not re-run) ===")
    cost_vs_direct_pct = (result.total_cost / DIRECT_LEVEL_COST - 1.0) * 100.0
    cost_vs_deep_pct = (result.total_cost / DEEP_CANDIDATE_COST - 1.0) * 100.0
    cost_vs_refined_pct = (result.total_cost / STAGE37_3_REFINED_EPS17["cost"] - 1.0) * 100.0
    print(f"  vs direct level (cost~={DIRECT_LEVEL_COST}): {cost_vs_direct_pct:+.2f}%")
    print(f"  vs deep candidate (cost~={DEEP_CANDIDATE_COST}, min_MSL~={DEEP_CANDIDATE_MIN_MSL} -- SAME normalized "
          f"distance+altitude objective only, NOT a global-optimum claim): {cost_vs_deep_pct:+.2f}%  "
          f"(min_MSL diff: {alt_metrics['minimum_aircraft_msl'] - DEEP_CANDIDATE_MIN_MSL:+.1f}m)")
    print(f"  vs Stage 37.3 refined eps=1.7 (cost={STAGE37_3_REFINED_EPS17['cost']}, "
          f"3d_length={STAGE37_3_REFINED_EPS17['length_3d']}, min_MSL={STAGE37_3_REFINED_EPS17['min_msl']}): "
          f"{cost_vs_refined_pct:+.2f}%  (3d_length diff: {length_3d - STAGE37_3_REFINED_EPS17['length_3d']:+.1f}m, "
          f"min_MSL diff: {alt_metrics['minimum_aircraft_msl'] - STAGE37_3_REFINED_EPS17['min_msl']:+.1f}m)")

    profile = compute_vertical_profile_metrics(path, primitives, tq, cfg)
    print(f"\n  ascii: {ascii_profile(profile, profile['total_horizontal_distance_m'])}")

    os.makedirs("outputs", exist_ok=True)
    with open(OUTPUT_CSV, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "row", "col", "z_index", "x", "y", "z_msl"])
        for i, s in enumerate(path):
            x, y, zm = state_to_xyz(s, tq, cfg)
            writer.writerow([i, s[0], s[1], s[2], f"{x:.2f}", f"{y:.2f}", f"{zm:.2f}"])
    print(f"\nPath written to {OUTPUT_CSV}")


if __name__ == "__main__":
    main()
