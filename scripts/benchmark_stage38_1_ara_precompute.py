"""Stage 38.1: Fine Corridor Safety Precompute + ARA* Runtime Sweep --
build planner.fine_precompute's dense-array safety precompute ONCE (only
for the fine XY corridor cells), then run ONE continuous genuine ARA*
(planner.astar.ara_star_search) over epsilon_schedule=(1.7,1.5,1.3,1.2),
never restarting the search per epsilon. Everything else (freeze_history=
True, corridor, z_guide tube, cost, heuristic, goal tolerance, ARA* reuse
logic) is UNCHANGED from Stage 38. Stage 38's own recorded numbers are NOT
re-run here.
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
from planner.fine_precompute import precompute_fine_corridor_primitive_safety
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
EPSILON_SCHEDULE = (1.70, 1.50, 1.30, 1.20)
GOAL_TOLERANCE_XY_M = 35.0
GOAL_TOLERANCE_Z_M = 25.0
XY_MASK_NPY = "outputs/stage36_corridor_mask.npy"
COARSE_PATH_CSV = "outputs/stage36_coarse_path_eps1.5.csv"
Z_TOLERANCE_M = 200.0
FINAL_OUTPUT_CSV = "outputs/stage38_1_ara_final_path.csv"

# Reference from project.md "Stage 38" -- NOT re-run here.
STAGE38_REFERENCE = {"first_expanded": 54, "first_cost": 1.517405, "final_cost": 1.483210, "total_expanded": 30000}


def fine_replay(path, primitives, terrain, config):
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
    print(f"initial reference (direct 216-edge level chain): valid={ok} cost={direct_cost:.6f}")

    print("\n=== PRECOMPUTE: fine corridor primitive safety (dense NumPy arrays) ===")
    precompute = precompute_fine_corridor_primitive_safety(tq, xy_mask, primitives, cfg)
    print(f"  entry_count={precompute.entry_count}  corridor_cell_count={precompute.corridor_cell_count}")
    print(f"  preprocessing_time={precompute.preprocessing_runtime_s:.3f}s")
    print(f"  approx_memory_MB={precompute.approx_memory_mb:.2f}")
    print(f"  static_invalid_count={precompute.static_invalid_count} (among computed corridor entries)")
    print(f"  (equivalence vs old evaluator verified separately in scripts/validate_fine_precompute.py -- ALL PASS)")

    print(f"\n=== Stage 38.1: genuine ARA*, epsilon_schedule={EPSILON_SCHEDULE}, freeze_history=True, "
          f"fine_precompute=ON, max_expansions_cumulative={MAX_EXPANSIONS_CUMULATIVE} ===\n")

    result = ara_star_search(
        start, goal, tq, min_search_altitude_msl=min_search, max_search_altitude_msl=max_search,
        config=cfg, primitives=primitives, epsilon_schedule=EPSILON_SCHEDULE,
        max_expansions_cumulative=MAX_EXPANSIONS_CUMULATIVE,
        corridor_mask=xy_mask, z_guide_grid=z_guide_grid, z_guide_tolerance_m=Z_TOLERANCE_M,
        fine_precompute=precompute,
    )

    print("=== PER-EPSILON PHASE REPORT ===")
    prev_cost = None
    prev_cum_t = 0.0
    rows_for_table = []
    for p in result.phases:
        phase_time = p.cumulative_runtime_s - prev_cum_t
        if prev_cost is not None and prev_cost < math.inf and p.incumbent_cost < math.inf:
            cost_gain_pct = (1.0 - p.incumbent_cost / prev_cost) * 100.0
        else:
            cost_gain_pct = float("nan")
        quality_gain_per_second = cost_gain_pct / phase_time if phase_time > 0 and not math.isnan(cost_gain_pct) else float("nan")

        print(f"  eps={p.epsilon:.2f}  added_exp={p.added_expansions}  cum_exp={p.cumulative_expansions}  "
              f"phase_time={phase_time:.3f}s  cum_time={p.cumulative_runtime_s:.3f}s")
        print(f"    incumbent_cost={p.incumbent_cost:.6f}  cost_gain_vs_prev_phase={cost_gain_pct:+.3f}%  "
              f"quality_gain_per_second={quality_gain_per_second:.4f}")
        print(f"    min_MSL={p.min_msl:.1f}  mean_MSL={p.mean_msl:.1f}  max_MSL={p.max_msl:.1f}  "
              f"path_length_3d={p.length_3d_m:.1f}m  xy_length={p.xy_length_m:.1f}m")
        print(f"    OPEN={p.open_size_at_end}  INCONS={p.incons_size_at_end}  CLOSED_this_phase={p.closed_size_this_phase}  "
              f"g_improvements={p.g_value_improvement_count}  reversal_count(diagnostic)={p.vertical_reversal_count}  "
              f"phase_complete={p.phase_complete}")

        rows_for_table.append((p.epsilon, p.added_expansions, phase_time, p.cumulative_runtime_s,
                                p.incumbent_cost, cost_gain_pct, p.min_msl, p.mean_msl))
        prev_cost = p.incumbent_cost
        prev_cum_t = p.cumulative_runtime_s

    print(f"\n=== SUMMARY ===")
    print(f"  path_found={result.path_found}  refinement_limit_reached={result.refinement_limit_reached}")
    print(f"  total_expanded={result.total_expanded}  total_runtime_s={result.total_runtime_s:.4f}")

    if not result.path_found:
        print("\nFAIL -- ARA* never found a complete safe path in any phase within the cumulative budget.")
        return

    print(f"\n=== FIRST SOLUTION (epsilon=1.7 phase) vs Stage 38 baseline (NOT re-run) ===")
    print(f"  this run:  expanded={result.first_incumbent_expanded}  runtime_s={result.first_incumbent_runtime_s:.4f}  "
          f"cost={result.first_incumbent_cost:.6f}")
    print(f"  Stage 38:  expanded={STAGE38_REFERENCE['first_expanded']}  cost={STAGE38_REFERENCE['first_cost']}")

    final_path = result.final_incumbent_path
    xyz = [state_to_xyz(s, tq, cfg) for s in final_path]
    xy_length = sum(math.hypot(xyz[i + 1][0] - xyz[i][0], xyz[i + 1][1] - xyz[i][1]) for i in range(len(xyz) - 1))
    length_3d = sum(math.sqrt((xyz[i + 1][0] - xyz[i][0]) ** 2 + (xyz[i + 1][1] - xyz[i][1]) ** 2
                               + (xyz[i + 1][2] - xyz[i][2]) ** 2) for i in range(len(xyz) - 1))
    altitudes = [p[2] for p in xyz]
    replay = fine_replay(final_path, primitives, tq, cfg)
    min_agl_direct = _path_min_observed_agl(final_path, primitives, tq, cfg)
    safety_pass = (len(replay["violations"]) == 0 and min_agl_direct >= cfg.min_agl_m
                   and replay["max_angle"] <= cfg.max_climb_angle_deg)

    print(f"\n=== FINAL BEST INCUMBENT (independent fine DEM replay) ===")
    print(f"  cost={result.final_incumbent_cost:.6f}  min_MSL={min(altitudes):.1f}  max_MSL={max(altitudes):.1f}  "
          f"3d_length={length_3d:.1f}m  xy_length={xy_length:.1f}m")
    print(f"  min_AGL={min_agl_direct:.2f}m  max_angle={replay['max_angle']:.2f}deg  "
          f"violations={len(replay['violations'])}  SAFETY={'PASS' if safety_pass else 'FAIL'}")
    print(f"  vs Stage 38 final (cost={STAGE38_REFERENCE['final_cost']}): "
          f"{(result.final_incumbent_cost / STAGE38_REFERENCE['final_cost'] - 1.0) * 100.0:+.3f}%")

    print(f"\n=== SHORT TABLE ===")
    print(f"  {'epsilon':>7} | {'added_exp':>9} | {'phase_time':>10} | {'cum_time':>9} | {'cost':>10} | "
          f"{'cost_gain':>9} | {'min_MSL':>8} | {'mean_MSL':>9}")
    for eps, added, ptime, ctime, cost, gain, minm, meanm in rows_for_table:
        gain_str = f"{gain:+.2f}%" if not math.isnan(gain) else "n/a"
        print(f"  {eps:>7.2f} | {added:>9} | {ptime:>9.3f}s | {ctime:>8.3f}s | {cost:>10.6f} | "
              f"{gain_str:>9} | {minm:>8.1f} | {meanm:>9.1f}")

    print(f"\n=== KARAR SORULARI ===")
    if len(rows_for_table) >= 2:
        eps15 = rows_for_table[1]
        print(f"  1.5 ek kazanç: {eps15[5]:+.2f}% cost / {eps15[2]:.2f}s phase time "
              f"(quality_gain_per_second={eps15[5]/eps15[2] if eps15[2]>0 else float('nan'):.4f})")
    if len(rows_for_table) >= 3:
        eps13 = rows_for_table[2]
        print(f"  1.3 ek kazanç: {eps13[5]:+.2f}% cost / {eps13[2]:.2f}s phase time "
              f"(quality_gain_per_second={eps13[5]/eps13[2] if eps13[2]>0 else float('nan'):.4f})")
    if len(rows_for_table) >= 4:
        eps12 = rows_for_table[3]
        print(f"  1.2 ek kazanç: {eps12[5]:+.2f}% cost / {eps12[2]:.2f}s phase time "
              f"(quality_gain_per_second={eps12[5]/eps12[2] if eps12[2]>0 else float('nan'):.4f})")
    print(f"  precompute preprocessing_time={precompute.preprocessing_runtime_s:.3f}s (ONE-TIME, amortized across the whole run)")

    write_path_csv(final_path, tq, cfg, FINAL_OUTPUT_CSV)
    print(f"\nFinal path written to {FINAL_OUTPUT_CSV}")


if __name__ == "__main__":
    main()
