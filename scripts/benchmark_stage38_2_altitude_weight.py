"""Stage 38.2: altitude-weight A/B point.

Run exactly one continuous fine ARA* search with w_distance=1.0,
w_altitude=1.50 and epsilon_schedule=(1.70, 1.50), reusing the Stage 38.1
fine safety precompute.  All geometry, safety, corridor, heuristic, goal,
and ARA* settings stay unchanged.  Recorded older paths are replayed only;
no older search/benchmark is rerun.
"""
import csv
import dataclasses
import math
import os

import numpy as np

from planner.astar import (
    _path_altitude_metrics,
    _path_vertical_reversal_metrics,
    ara_star_search,
    compute_distance_reference,
    msl_to_z_index,
    state_to_xyz,
)
from planner.config import DEFAULT_CONFIG
from planner.corridor import build_z_guide_grid
from planner.fine_precompute import precompute_fine_corridor_primitive_safety
from planner.mission import mission_policy_from_config
from planner.primitives import build_primitive_set, evaluate_primitive
from planner.roi import load_roi
from planner.terrain import TerrainQuery


START_ROW, START_COL = 48, 276
GOAL_ROW, GOAL_COL = 264, 276
AIRCRAFT_MSL = 3760.0
ALTITUDE_REFERENCE_MSL = 3240.0
NORMALIZED_ALTITUDE_SCALE_M = 1000.0
NORMALIZED_W_DISTANCE = 1.0
NORMALIZED_W_ALTITUDE = 1.50
NORMALIZED_W_REVERSAL = 1.0  # unchanged; freeze_history makes its contribution zero
MAX_EXPANSIONS_CUMULATIVE = 30_000
EPSILON_SCHEDULE = (1.70, 1.50)
GOAL_TOLERANCE_XY_M = 35.0
GOAL_TOLERANCE_Z_M = 25.0
XY_MASK_NPY = "outputs/stage36_corridor_mask.npy"
COARSE_PATH_CSV = "outputs/stage36_coarse_path_eps1.5.csv"
Z_TOLERANCE_M = 200.0
OLD_EPS13_PATH_CSV = "outputs/stage38_1_ara_final_path.csv"
OUTPUT_PREFIX = "outputs/stage38_2"


def read_path_csv(path):
    with open(path, newline="") as f:
        return [(int(row["row"]), int(row["col"]), int(row["z_index"])) for row in csv.DictReader(f)]


def write_path_csv(path, terrain, config, out_path):
    os.makedirs("outputs", exist_ok=True)
    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "row", "col", "z_index", "x", "y", "z_msl"])
        for i, state in enumerate(path):
            x, y, z_msl = state_to_xyz(state, terrain, config)
            writer.writerow([i, state[0], state[1], state[2], f"{x:.2f}", f"{y:.2f}", f"{z_msl:.2f}"])


def replay_safety(path, primitives, terrain, config):
    by_delta = {(p.drow, p.dcol, round(p.dz_m / config.z_step_m)): p for p in primitives}
    violations = []
    min_agl = math.inf
    max_angle = 0.0
    for s1, s2 in zip(path, path[1:]):
        prim = by_delta.get((s2[0] - s1[0], s2[1] - s1[1], s2[2] - s1[2]))
        if prim is None:
            violations.append((s1, s2, "no_matching_primitive"))
            continue
        start_xyz = state_to_xyz(s1, terrain, config)
        evaluated = evaluate_primitive(start_xyz, prim, terrain, config)
        if not evaluated.valid:
            violations.append((s1, s2, evaluated.reason))
        else:
            min_agl = min(min_agl, evaluated.min_agl_m)
        if prim.horizontal_distance_m:
            max_angle = max(
                max_angle,
                math.degrees(math.atan2(abs(prim.dz_m), prim.horizontal_distance_m)),
            )
    return min_agl, max_angle, len(violations)


def cost_components(path, primitives, terrain, config, d_ref):
    """Exact freeze-history normalized objective split into its two active terms."""
    by_delta = {(p.drow, p.dcol, round(p.dz_m / config.z_step_m)): p for p in primitives}
    policy = mission_policy_from_config(config)
    distance_component = 0.0
    altitude_component = 0.0
    for s1, s2 in zip(path, path[1:]):
        prim = by_delta[(s2[0] - s1[0], s2[1] - s1[1], s2[2] - s1[2])]
        z1 = state_to_xyz(s1, terrain, config)[2]
        z2 = z1 + prim.dz_m
        geometric = math.hypot(prim.horizontal_distance_m, prim.dz_m)
        components = policy.edge_components(geometric, z1, z2, d_ref)
        distance_component += components.distance
        altitude_component += components.altitude
    return distance_component, altitude_component, distance_component + altitude_component


def path_report(path, primitives, terrain, config, d_ref):
    alt = _path_altitude_metrics(path, terrain, config)
    reversals = _path_vertical_reversal_metrics(path, primitives, config)
    xyz = [state_to_xyz(s, terrain, config) for s in path]
    xy_length = sum(
        math.hypot(x2 - x1, y2 - y1)
        for (x1, y1, _), (x2, y2, _) in zip(xyz, xyz[1:])
    )
    min_agl, max_angle, violations = replay_safety(path, primitives, terrain, config)
    distance_cost, altitude_cost, total_cost = cost_components(path, primitives, terrain, config, d_ref)
    return {
        "min_msl": alt["minimum_aircraft_msl"],
        "mean_msl": alt["average_aircraft_msl"],
        "xy_length": xy_length,
        "length_3d": alt["geometric_path_length"],
        "climb": alt["total_climb_m"],
        "descent": alt["total_descent_m"],
        "reversals": reversals["total_vertical_reversal_count"],
        "min_agl": min_agl,
        "max_angle": max_angle,
        "violations": violations,
        "distance_cost": distance_cost,
        "altitude_cost": altitude_cost,
        "total_cost": total_cost,
    }


def print_route(label, q):
    print(f"[{label}]")
    print(
        f"  min_MSL={q['min_msl']:.1f}  distance_weighted_mean_MSL={q['mean_msl']:.1f}  "
        f"XY={q['xy_length']:.1f}m  3D={q['length_3d']:.1f}m"
    )
    print(
        f"  climb={q['climb']:.1f}m  descent={q['descent']:.1f}m  reversals={q['reversals']}  "
        f"min_AGL={q['min_agl']:.2f}m  max_angle={q['max_angle']:.2f}deg  violations={q['violations']}"
    )
    print(
        f"  distance_component={q['distance_cost']:.9f}  altitude_component={q['altitude_cost']:.9f}  "
        f"total_cost={q['total_cost']:.9f}"
    )


def main():
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
    terrain = TerrainQuery(roi)
    xy_mask = np.load(XY_MASK_NPY)

    coarse_path_xyz = []
    with open(COARSE_PATH_CSV, newline="") as f:
        for row in csv.DictReader(f):
            coarse_path_xyz.append((float(row["x"]), float(row["y"]), float(row["z_msl"])))
    z_guide_grid = build_z_guide_grid(roi, coarse_path_xyz)

    z0 = msl_to_z_index(AIRCRAFT_MSL, cfg)
    start = (START_ROW, START_COL, z0)
    goal = (GOAL_ROW, GOAL_COL, z0)
    for row, col in ((START_ROW, START_COL), (GOAL_ROW, GOAL_COL)):
        if not (bool(xy_mask[row, col]) and abs(AIRCRAFT_MSL - float(z_guide_grid[row, col])) <= Z_TOLERANCE_M):
            xy_mask[row, col] = True
            z_guide_grid[row, col] = AIRCRAFT_MSL

    segment = roi.elevation[START_ROW:GOAL_ROW + 1, START_COL]
    min_search = math.ceil((float(segment.min()) + cfg.min_agl_m) / cfg.z_step_m) * cfg.z_step_m
    max_search = AIRCRAFT_MSL + 20.0
    d_ref = compute_distance_reference(start, goal, terrain, cfg)

    print("=== Stage 38.2 fixed configuration ===")
    print(
        f"w_distance={NORMALIZED_W_DISTANCE} w_altitude={NORMALIZED_W_ALTITUDE} H_ref={ALTITUDE_REFERENCE_MSL:.0f} "
        f"H_scale={NORMALIZED_ALTITUDE_SCALE_M:.0f} epsilon_schedule={EPSILON_SCHEDULE} "
        f"freeze_history=True XY=+/-300m Z=+/-{Z_TOLERANCE_M:.0f}m fine_precompute=ON"
    )

    print("\n=== Replay existing recorded paths only (NO SEARCH) ===")
    print("old epsilon=1.5 ARA* phase path: unavailable on disk (Stage 38/38.1 did not save that phase snapshot)")
    old_eps13_path = read_path_csv(OLD_EPS13_PATH_CSV)
    old_eps13 = path_report(old_eps13_path, primitives, terrain, cfg, d_ref)
    print_route("old epsilon=1.3 path replayed at w_altitude=1.50", old_eps13)

    print("\n=== One-time fine safety precompute ===")
    precompute = precompute_fine_corridor_primitive_safety(terrain, xy_mask, primitives, cfg)
    print(
        f"entries={precompute.entry_count} cells={precompute.corridor_cell_count} "
        f"time={precompute.preprocessing_runtime_s:.3f}s memory={precompute.approx_memory_mb:.2f}MB "
        f"static_invalid={precompute.static_invalid_count}"
    )

    print("\n=== ONE continuous ARA* run: 1.70 -> 1.50 -> STOP ===")
    result = ara_star_search(
        start,
        goal,
        terrain,
        min_search_altitude_msl=min_search,
        max_search_altitude_msl=max_search,
        config=cfg,
        primitives=primitives,
        epsilon_schedule=EPSILON_SCHEDULE,
        max_expansions_cumulative=MAX_EXPANSIONS_CUMULATIVE,
        corridor_mask=xy_mask,
        z_guide_grid=z_guide_grid,
        z_guide_tolerance_m=Z_TOLERANCE_M,
        fine_precompute=precompute,
    )

    previous_cumulative_time = 0.0
    phase_rows = []
    for phase in result.phases:
        phase_time = phase.cumulative_runtime_s - previous_cumulative_time
        q = path_report(phase.incumbent_path, primitives, terrain, cfg, d_ref)
        phase_rows.append((phase, phase_time, q))
        print(f"\n--- epsilon={phase.epsilon:.2f} ---")
        print(
            f"first_improvement_expansion={phase.first_incumbent_improvement_expansion} "
            f"incumbent_last_improved_expansion={phase.last_incumbent_improvement_expansion} "
            f"added_expansions={phase.added_expansions} cumulative_expansions={phase.cumulative_expansions}"
        )
        print(
            f"phase_runtime={phase_time:.3f}s cumulative_runtime={phase.cumulative_runtime_s:.3f}s "
            f"phase_complete={phase.phase_complete}"
        )
        print_route(f"epsilon={phase.epsilon:.2f} phase incumbent", q)
        print(f"  search_incumbent_cost={phase.incumbent_cost:.9f} component_delta={q['total_cost'] - phase.incumbent_cost:+.3e}")
        previous_cumulative_time = phase.cumulative_runtime_s

        write_path_csv(phase.incumbent_path, terrain, cfg, f"{OUTPUT_PREFIX}_eps{int(round(phase.epsilon * 10)):02d}_path.csv")

    first_q = path_report(result.first_incumbent_path, primitives, terrain, cfg, d_ref)
    print("\n=== First solution inside epsilon=1.70 phase ===")
    print(
        f"expansion={result.first_incumbent_expanded} runtime={result.first_incumbent_runtime_s:.3f}s "
        f"search_cost={result.first_incumbent_cost:.9f}"
    )
    print_route("epsilon=1.70 first incumbent", first_q)

    final_phase, _, final_q = phase_rows[-1]
    final_safe = (
        final_q["violations"] == 0
        and final_q["min_agl"] >= cfg.min_agl_m
        and final_q["max_angle"] <= cfg.max_climb_angle_deg
    )
    print("\n=== FINAL epsilon=1.50 fine replay ===")
    print(
        f"AGL>={cfg.min_agl_m:.0f}: {final_q['min_agl']:.2f}  angle<={cfg.max_climb_angle_deg:.0f}: "
        f"{final_q['max_angle']:.2f}  violation={final_q['violations']}  SAFETY={'PASS' if final_safe else 'FAIL'}"
    )

    print("\n=== REQUIRED TABLE ===")
    print("phase | expansions | time | min MSL | mean MSL | XY length | cost")
    for phase, phase_time, q in phase_rows:
        print(
            f"{phase.epsilon:.2f} | {phase.added_expansions} (cum {phase.cumulative_expansions}) | "
            f"{phase_time:.3f}s (cum {phase.cumulative_runtime_s:.3f}s) | {q['min_msl']:.1f} | "
            f"{q['mean_msl']:.1f} | {q['xy_length']:.1f}m | {q['total_cost']:.9f}"
        )

    print("\n=== Run status ===")
    print(
        f"path_found={result.path_found} total_expanded={result.total_expanded} "
        f"search_runtime={result.total_runtime_s:.3f}s preprocessing={precompute.preprocessing_runtime_s:.3f}s "
        f"end_to_end={result.total_runtime_s + precompute.preprocessing_runtime_s:.3f}s"
    )


if __name__ == "__main__":
    main()
