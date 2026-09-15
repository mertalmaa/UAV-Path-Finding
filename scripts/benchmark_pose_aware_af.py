"""Produce the canonical controlled BASIC-vs-COMBINED pose-aware A--F evidence.

This runner changes only ``enable_combined_turns`` between modes.  Search keys,
cost, heuristic, safety, tolerances, primitive distances, and budgets are all
the shared canonical values from ``scripts.benchmark_missions``.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import sys
from pathlib import Path
from typing import Dict, Iterable, Tuple

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from planner.fixed_wing_envelope import FixedWingKinematicEnvelope
from planner.physical import PhysicalPose
from planner.pose_search import (
    GoalPose,
    active_primitive_names,
    navigation_bearing_deg,
    pose_aware_astar_search,
)
from planner.roi import load_roi
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from scripts.benchmark_far_missions import FAR_MISSIONS
from scripts.benchmark_missions import (
    CACHE_DIR,
    CONFIG,
    FACTOR,
    GOAL_TOLERANCE,
    SOURCE_DEM_PATH,
    mission_definitions,
)


RESULTS_DIR = ROOT / "results"
BASIC_OUTPUT = RESULTS_DIR / "pose_aware_basic_af.json"
COMBINED_OUTPUT = RESULTS_DIR / "pose_aware_combined_af.json"
COMPARISON_OUTPUT = RESULTS_DIR / "pose_aware_basic_vs_combined.md"


def _mission_definitions(cache) -> Dict[str, dict]:
    """Return A--F in a single stable order without changing their contracts."""
    missions = dict(mission_definitions(cache))
    for name, definition in FAR_MISSIONS.items():
        missions[name] = dict(definition)
    return missions


def _mission_altitudes(definition: dict) -> Tuple[float, float]:
    start = float(definition["start_z"] if "start_z" in definition else definition["z_msl_m"])
    goal = definition["goal_z"] if "goal_z" in definition else definition.get("goal_z_msl_m", start)
    return start, float(goal)


def _search_statistics(result) -> dict:
    return {
        "generated_by_primitive": result.generated_by_primitive,
        "open_inserted_by_primitive": result.open_inserted_by_primitive,
        "expanded_arrivals_by_primitive": result.expanded_arrivals_by_primitive,
    }


def _path_payload(result) -> dict | None:
    if not result.success:
        return None
    return {
        "physical_length_m": result.continuous_path_length_m,
        "segments": len(result.trajectories),
        "primitives": list(result.path_primitives),
        "primitive_counts": result.path_primitive_counts,
        "minimum_agl_m": result.minimum_agl_m,
        "goal_xy_error_m": result.goal_xy_error_m,
        "goal_z_error_m": result.goal_z_error_m,
        "final_heading_deg": result.final_heading_deg,
    }


def _run_case(name: str, definition: dict, terrain, envelope, config) -> dict:
    start_z, goal_z = _mission_altitudes(definition)
    sx, sy = terrain.rowcol_to_xy(*definition["start_rc"])
    gx, gy = terrain.rowcol_to_xy(*definition["goal_rc"])
    heading = navigation_bearing_deg(sx, sy, gx, gy)
    start = PhysicalPose(sx, sy, start_z, heading)
    goal = GoalPose(gx, gy, goal_z)
    result = pose_aware_astar_search(
        start, goal, terrain, envelope=envelope, goal_tolerance=GOAL_TOLERANCE, config=config,
        max_expansions=30_000, max_search_time_s=300.0,
    )
    best_end = result.best_nodes[-1].end_pose
    return {
        "name": name,
        "criterion": definition["criterion"],
        "start_rowcol": definition["start_rc"],
        "goal_rowcol": definition["goal_rc"],
        "physical_separation_m": math.hypot(gx - sx, gy - sy),
        "altitude_contract": "explicit_roi_entry_cruise_msl",
        "start_z_msl_m": start_z,
        "goal_z_msl_m": goal_z,
        "start_heading_deg": heading,
        "status": result.status,
        "termination_reason": result.termination_reason,
        "success": result.success,
        "expanded": result.expanded_nodes,
        "generated": result.generated_neighbors,
        "generated_per_expanded": result.generated_neighbors / result.expanded_nodes if result.expanded_nodes else float("nan"),
        "rejected": result.rejected_neighbors,
        "peak_open": result.max_open_size,
        "runtime_s": result.runtime_s,
        "unique_search_keys": result.unique_search_keys,
        "same_key": {
            "collisions": result.same_key_collision_count,
            "replaced_lower_g": result.same_key_replaced_lower_g,
            "rejected_existing_better": result.same_key_rejected_existing_better,
            "self_transitions": result.same_key_self_transition_count,
            "self_transitions_by_primitive": result.same_key_self_transition_by_primitive,
        },
        "reject_reasons": result.rejected_reason_counts,
        "search_statistics": _search_statistics(result),
        "minimum_agl_m": result.minimum_agl_m if result.success else None,
        "closest_xy_distance_m": result.closest_xy_distance_to_goal_m,
        "best_partial_goal_z_error_m": abs(best_end.z_msl_m - goal.z_msl_m),
        "best_partial_goal_3d_error_m": result.closest_3d_distance_to_goal_m,
        "best_partial_end_pose": {
            "x_m": best_end.x_m,
            "y_m": best_end.y_m,
            "z_msl_m": best_end.z_msl_m,
            "heading_deg": best_end.heading_deg,
        },
        "path": _path_payload(result),
    }


def run_suite(enable_combined_turns: bool) -> dict:
    """Run unchanged A--F missions with only the activation flag varied."""
    cache = load_terrain_cache(CACHE_DIR, load_roi(CONFIG), SOURCE_DEM_PATH)
    terrain = build_terrain_query_from_cache(cache, load_roi(CONFIG), FACTOR)
    envelope = FixedWingKinematicEnvelope()
    config = dataclasses.replace(CONFIG, enable_combined_turns=enable_combined_turns)
    missions = _mission_definitions(cache)
    return {
        "architecture": "pose_aware_fixed_wing_single_representative_approximate_search",
        "mode": "COMBINED" if enable_combined_turns else "BASIC",
        "enable_combined_turns": enable_combined_turns,
        "active_primitives": list(active_primitive_names(config)),
        "aircraft_model": "generic_constant_performance_fixed_wing",
        "speed_mps": 40.0,
        "effective_min_agl_m": config.min_agl_m,
        "lateral_buffer_m": config.lateral_buffer_m,
        "search_key": {
            "xy_m": config.search_xy_bin_m,
            "z_m": config.search_z_bin_m,
            "heading_deg": config.search_heading_bin_deg,
        },
        "goal_tolerance": {"xy_m": GOAL_TOLERANCE.xy_m, "altitude_m": GOAL_TOLERANCE.altitude_m},
        "budgets": {"max_expansions": 30_000, "max_search_time_s": 300.0},
        "missions": {name: _run_case(name, definition, terrain, envelope, config)
                     for name, definition in missions.items()},
    }


def _value(value, digits: int = 2) -> str:
    if value is None:
        return "-"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def _path_summary(entry: dict) -> str:
    path = entry["path"]
    return "-" if path is None else ", ".join(f"{name}:{count}" for name, count in path["primitive_counts"].items())


def _mission_e_classification(basic: dict, combined: dict) -> str:
    if combined["success"] and not basic["success"]:
        return "IMPROVE"
    if basic["success"] and not combined["success"]:
        return "REGRESS"
    if (combined["generated"] > basic["generated"] and
            combined["peak_open"] > basic["peak_open"] and
            combined["closest_xy_distance_m"] > basic["closest_xy_distance_m"]):
        return "REGRESS"
    if (combined["generated"] < basic["generated"] and
            combined["closest_xy_distance_m"] <= basic["closest_xy_distance_m"]):
        return "IMPROVE"
    return "NEUTRAL"


def render_comparison(basic: dict, combined: dict) -> str:
    lines = [
        "# Pose-aware BASIC vs COMBINED benchmark (A-F)",
        "",
        "Only `enable_combined_turns` changes between these runs. Search key, heuristic, cost, safety, goal tolerance, primitive distance, turn angle, and budgets are identical.",
        "",
        "| Mission | Mode | Status | Expanded | Generated | Gen/expanded | Rejected | Peak OPEN | Runtime s | Path m | Min AGL m | Best XY / Z error m | Path primitive counts |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for name in basic["missions"]:
        for payload in (basic["missions"][name], combined["missions"][name]):
            path = payload["path"]
            path_length = None if path is None else path["physical_length_m"]
            min_agl = None if path is None else path["minimum_agl_m"]
            z_error = (path["goal_z_error_m"] if path is not None
                       else payload["best_partial_goal_z_error_m"])
            lines.append(
                f"| {name} | {'COMBINED' if payload is combined['missions'][name] else 'BASIC'} | {payload['termination_reason']} | "
                f"{payload['expanded']} | {payload['generated']} | {_value(payload['generated_per_expanded'])} | "
                f"{payload['rejected']} | {payload['peak_open']} | {_value(payload['runtime_s'])} | "
                f"{_value(path_length)} | {_value(min_agl)} | {_value(payload['closest_xy_distance_m'])} / {_value(z_error)} | {_path_summary(payload)} |"
            )
    mission_b = combined["missions"]["B_relief_affected"]
    mission_e_basic = basic["missions"]["E_long_descent_9_3km"]
    mission_e_combined = combined["missions"]["E_long_descent_9_3km"]
    classification = _mission_e_classification(mission_e_basic, mission_e_combined)
    lines += [
        "",
        "## Mission B combined-path verification",
        "",
        "Ordered physical primitives:",
        "",
        "```text",
        " -> ".join(mission_b["path"]["primitives"]) if mission_b["path"] else "NO_PATH",
        "```",
        "",
        f"Combined primitive present: **{'YES' if mission_b['path'] and any('CLIMBING_' in item or 'DESCENDING_' in item for item in mission_b['path']['primitives']) else 'NO'}**.",
        "",
        "## Mission E regression check",
        "",
        f"- BASIC: generated {mission_e_basic['generated']}; peak OPEN {mission_e_basic['peak_open']}; best XY {mission_e_basic['closest_xy_distance_m']:.2f} m.",
        f"- COMBINED: generated {mission_e_combined['generated']}; peak OPEN {mission_e_combined['peak_open']}; best XY {mission_e_combined['closest_xy_distance_m']:.2f} m.",
        f"- Classification: **{classification}**.",
        "",
        "Search-level counters are intentionally separate from final reconstructed path primitive counts.",
    ]
    return "\n".join(lines) + "\n"


def _write(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("basic", "combined", "both"), default="both")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    basic = run_suite(False) if args.mode in ("basic", "both") else None
    combined = run_suite(True) if args.mode in ("combined", "both") else None
    if basic is not None:
        _write(BASIC_OUTPUT, basic)
        print(f"basic: {BASIC_OUTPUT}")
    if combined is not None:
        _write(COMBINED_OUTPUT, combined)
        print(f"combined: {COMBINED_OUTPUT}")
    if basic is not None and combined is not None:
        COMPARISON_OUTPUT.write_text(render_comparison(basic, combined), encoding="utf-8")
        print(f"comparison: {COMPARISON_OUTPUT}")


if __name__ == "__main__":
    main()
