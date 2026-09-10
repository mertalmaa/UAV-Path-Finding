"""Stage 38: Fine ARA* Refinement -- epsilon_schedule=(1.70, 1.50, 1.30, 1.10),
freeze_history=True, SAME 3D corridor (XY +/-300m, Z +/-200m) and normalized
cost as Stage 37.3/37.4, ONE genuine ARA* run (planner.astar.ara_star_search)
with a single CUMULATIVE 30,000-expansion budget across all four epsilon
phases -- never four independent astar_search() calls. Stage 37.3/37.4's own
baselines (57 expansions / 0.3427s / cost=1.509232 for eps=1.7's first
solution) are NOT re-run -- project.md's recorded numbers are the reference.
"""
import csv
import dataclasses
import math
import os

import numpy as np

from planner.astar import (
    _path_min_observed_agl, ara_star_search, compute_distance_reference, msl_to_z_index, state_to_xyz,
    validate_and_cost_path,
)
from planner.config import DEFAULT_CONFIG
from planner.corridor import build_z_guide_grid
from planner.primitives import build_primitive_set, evaluate_primitive
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from scripts.validate_cost_function_ranking import build_direct_level

START_ROW, START_COL = 48, 276
GOAL_ROW, GOAL_COL = 264, 276
AIRCRAFT_MSL = 3760.0
ALTITUDE_REFERENCE_MSL = 3240.0
NORMALIZED_ALTITUDE_SCALE_M = 1000.0
NORMALIZED_W_ALTITUDE = 1.25
NORMALIZED_W_DISTANCE = 1.0
NORMALIZED_W_REVERSAL = 1.0
MAX_EXPANSIONS_CUMULATIVE = 30_000
EPSILON_SCHEDULE = (1.70, 1.50, 1.30, 1.10)
GOAL_TOLERANCE_XY_M = 35.0
GOAL_TOLERANCE_Z_M = 25.0
XY_MASK_NPY = "outputs/stage36_corridor_mask.npy"
COARSE_PATH_CSV = "outputs/stage36_coarse_path_eps1.5.csv"
Z_TOLERANCE_M = 200.0
FIRST_OUTPUT_CSV = "outputs/stage38_ara_first_path.csv"
FINAL_OUTPUT_CSV = "outputs/stage38_ara_final_path.csv"

# Reference from project.md "Stage 37.4" -- NOT re-run here.
STAGE37_4_FIRST_SOLUTION = {"expanded": 57, "runtime_s": 0.3427, "cost": 1.509232}


def fine_replay(path, primitives, terrain, config):
    """Independent re-validation of every edge against the real fine
    terrain via evaluate_primitive -- same production safety authority,
    reused (not reimplemented), same pattern as scripts/
    benchmark_first_solution_eps17.py's own fine_replay."""
    violations = []
    min_agl = math.inf
    max_angle = 0.0
    by_delta = {(p.drow, p.dcol, round(p.dz_m / config.z_step_m)): p for p in primitives}
    for (r1, c1, z1), (r2, c2, z2) in zip(path, path[1:]):
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


def path_quality(path, primitives, terrain, config, d_ref):
    xyz = [state_to_xyz(s, terrain, config) for s in path]
    xy_length = sum(math.hypot(xyz[i + 1][0] - xyz[i][0], xyz[i + 1][1] - xyz[i][1]) for i in range(len(xyz) - 1))
    length_3d = sum(math.sqrt((xyz[i + 1][0] - xyz[i][0]) ** 2 + (xyz[i + 1][1] - xyz[i][1]) ** 2
                               + (xyz[i + 1][2] - xyz[i][2]) ** 2) for i in range(len(xyz) - 1))
    altitudes = [p[2] for p in xyz]
    total_climb = sum(max(0.0, xyz[i + 1][2] - xyz[i][2]) for i in range(len(xyz) - 1))
    total_descent = sum(max(0.0, xyz[i][2] - xyz[i + 1][2]) for i in range(len(xyz) - 1))
    replay = fine_replay(path, primitives, terrain, config)
    min_agl_direct = _path_min_observed_agl(path, primitives, terrain, config)
    return {
        "xy_length": xy_length, "length_3d": length_3d,
        "min_msl": min(altitudes), "max_msl": max(altitudes),
        "mean_msl": sum(altitudes) / len(altitudes),
        "total_climb": total_climb, "total_descent": total_descent,
        "min_agl": min_agl_direct, "max_angle": replay["max_angle"],
        "violations": len(replay["violations"]),
    }


def write_path_csv(path, terrain, config, out_path):
    os.makedirs("outputs", exist_ok=True)
    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "row", "col", "z_index", "x", "y", "z_msl"])
        for i, s in enumerate(path):
            x, y, zm = state_to_xyz(s, terrain, config)
            writer.writerow([i, s[0], s[1], s[2], f"{x:.2f}", f"{y:.2f}", f"{zm:.2f}"])


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
    ok, direct_cost = validate_and_cost_path(direct_level_path, primitives, tq, cfg, d_ref)
    print(f"initial reference (direct 216-edge level chain): valid={ok} cost={direct_cost:.6f}  (NOT the ARA* incumbent seed)")

    print(f"\n=== Stage 38: genuine ARA*, epsilon_schedule={EPSILON_SCHEDULE}, "
          f"freeze_history=True, max_expansions_cumulative={MAX_EXPANSIONS_CUMULATIVE} ===\n")

    result = ara_star_search(
        start, goal, tq, min_search_altitude_msl=min_search, max_search_altitude_msl=max_search,
        config=cfg, primitives=primitives, epsilon_schedule=EPSILON_SCHEDULE,
        max_expansions_cumulative=MAX_EXPANSIONS_CUMULATIVE,
        corridor_mask=xy_mask, z_guide_grid=z_guide_grid, z_guide_tolerance_m=Z_TOLERANCE_M,
        use_primitive_cache=True,
    )

    print("=== PER-EPSILON PHASE REPORT ===")
    header = (f"  {'eps':>5} {'added_exp':>9} {'cum_exp':>8} {'cum_t(s)':>9} {'OPEN':>7} {'INCONS':>7} "
              f"{'CLOSED':>7} {'g_impr':>7} {'incumbent':>10} {'min_MSL':>8} {'mean_MSL':>9} {'diag_LB':>9} "
              f"{'inc/LB':>7} {'complete':>8}")
    print(header)
    for p in result.phases:
        inc_str = f"{p.incumbent_cost:.6f}" if p.incumbent_available else "n/a"
        lb_str = f"{p.diagnostic_lower_bound:.4f}" if p.diagnostic_lower_bound < math.inf else "inf"
        ratio_str = f"{p.diagnostic_bound_ratio:.4f}" if not math.isnan(p.diagnostic_bound_ratio) else "n/a"
        min_msl_str = f"{p.min_msl:.1f}" if not math.isnan(p.min_msl) else "n/a"
        mean_msl_str = f"{p.mean_msl:.1f}" if not math.isnan(p.mean_msl) else "n/a"
        print(f"  {p.epsilon:>5.2f} {p.added_expansions:>9} {p.cumulative_expansions:>8} "
              f"{p.cumulative_runtime_s:>9.3f} {p.open_size_at_end:>7} {p.incons_size_at_end:>7} "
              f"{p.closed_size_this_phase:>7} {p.g_value_improvement_count:>7} {inc_str:>10} "
              f"{min_msl_str:>8} {mean_msl_str:>9} {lb_str:>9} {ratio_str:>7} {str(p.phase_complete):>8}")
        print(f"        xy_length={p.xy_length_m:.1f}m  3d_length={p.length_3d_m:.1f}m  "
              f"max_MSL={p.max_msl:.1f}  climb={p.total_climb_m:.1f}  descent={p.total_descent_m:.1f}  "
              f"reversal_count(diagnostic)={p.vertical_reversal_count}")

    print(f"\n=== SUMMARY ===")
    print(f"  path_found={result.path_found}  refinement_limit_reached={result.refinement_limit_reached}")
    print(f"  total_expanded={result.total_expanded}  total_runtime_s={result.total_runtime_s:.4f}")

    if not result.path_found:
        print("\nFAIL -- ARA* never found a complete safe path in any phase within the cumulative budget.")
        return

    first_path = result.first_incumbent_path
    final_path = result.final_incumbent_path

    first_q = path_quality(first_path, primitives, tq, cfg, d_ref)
    final_q = path_quality(final_path, primitives, tq, cfg, d_ref)

    print("\n=== FIRST SOLUTION (epsilon=1.7 phase, first incumbent) vs Stage 37.4 baseline (NOT re-run) ===")
    print(f"  ARA* first incumbent: expanded={result.first_incumbent_expanded}  "
          f"runtime_s={result.first_incumbent_runtime_s:.4f}  cost={result.first_incumbent_cost:.6f}")
    print(f"  Stage 37.4 baseline:  expanded={STAGE37_4_FIRST_SOLUTION['expanded']}  "
          f"runtime_s={STAGE37_4_FIRST_SOLUTION['runtime_s']}  cost={STAGE37_4_FIRST_SOLUTION['cost']}")

    print("\n=== INDEPENDENT FINE DEM REPLAY: FIRST vs FINAL ===")
    for label, q, cost in [("FIRST", first_q, result.first_incumbent_cost), ("FINAL", final_q, result.final_incumbent_cost)]:
        print(f"  [{label}] cost={cost:.6f}  min_MSL={q['min_msl']:.1f}  mean_MSL={q['mean_msl']:.1f}  "
              f"3d_length={q['length_3d']:.1f}m  xy_length={q['xy_length']:.1f}m  "
              f"climb={q['total_climb']:.1f}  descent={q['total_descent']:.1f}  "
              f"min_AGL={q['min_agl']:.2f}m  max_angle={q['max_angle']:.2f}deg  "
              f"violations={q['violations']}  SAFETY={'PASS' if q['violations'] == 0 and q['min_agl'] >= cfg.min_agl_m and q['max_angle'] <= cfg.max_climb_angle_deg else 'FAIL'}")

    cost_improvement_pct = (1.0 - result.final_incumbent_cost / result.first_incumbent_cost) * 100.0
    print(f"\n  cost improvement FIRST->FINAL: {cost_improvement_pct:+.2f}%")
    print(f"  min_MSL change: {final_q['min_msl'] - first_q['min_msl']:+.1f}m")
    print(f"  3d_length change: {final_q['length_3d'] - first_q['length_3d']:+.1f}m")

    write_path_csv(first_path, tq, cfg, FIRST_OUTPUT_CSV)
    write_path_csv(final_path, tq, cfg, FINAL_OUTPUT_CSV)
    print(f"\nFirst path written to {FIRST_OUTPUT_CSV}")
    print(f"Final path written to {FINAL_OUTPUT_CSV}")


if __name__ == "__main__":
    main()
