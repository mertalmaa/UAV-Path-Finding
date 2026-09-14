"""Canonical Mission A/B baseline runner for pose-aware fixed-wing A*."""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
from pathlib import Path

from planner.aircraft_profile import load_aircraft_profile
from planner.config import DEFAULT_CONFIG
from planner.physical import PhysicalPose
from planner.pose_search import GoalPose, GoalTolerance, navigation_bearing_deg, pose_aware_astar_search
from planner.roi import load_roi
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache

ROOT = Path(__file__).resolve().parents[1]
CACHE_DIR = ROOT / "outputs" / "terrain_cache"
PROFILE_PATH = ROOT / "jsbsim" / "results" / "c172p_aircraft_profile_planner_safe_v3.json"
SOURCE_DEM_PATH = str(DEFAULT_CONFIG.working_dem_path)
FACTOR, MIN_AGL_M = 2, 100.0
# First basic-primitive baselines enter the planning ROI with explicit cruise
# altitude, rather than the old terrain-floor-at-exact-minimum-clearance start.
# This is a mission initial-condition change, not a SearchKey/cost/primitive
# tuning change.  It gives the fixed-wing vehicle a safe first trajectory.
MISSION_CRUISE_MSL = {"A_easy_open": 3500.0, "B_relief_affected": 3600.0}
# Fixed 60 m continuous primitives cannot generically land exactly on a goal.
# This is an explicit benchmark acceptance contract, not a SearchKey setting.
GOAL_TOLERANCE = GoalTolerance(xy_m=90.0, altitude_m=10.0)
CONFIG = dataclasses.replace(DEFAULT_CONFIG, xy_resolution_m=60.0, min_agl_m=MIN_AGL_M,
                             search_xy_bin_m=60.0, search_z_bin_m=5.0,
                             search_heading_bin_deg=15.0, lateral_buffer_m=0.0)


def mission_definitions(cache) -> dict[str, dict]:
    """Derive retained physical endpoints from the historical cache windows."""
    row0, row1 = 146 // FACTOR, 186 // FACTOR
    col0, col1 = 146 // FACTOR, 186 // FACTOR
    window = cache._arrays[f"max_elevation_f{FACTOR}"][row0:row1, col0:col1]

    def global_cell(cell: tuple[int, int]) -> tuple[int, int]:
        return row0 + cell[0], col0 + cell[1]

    height, width = window.shape
    corners = {"TL": (0, 0), "TR": (0, width - 1), "BL": (height - 1, 0), "BR": (height - 1, width - 1)}
    elevations = {name: float(window[cell]) for name, cell in corners.items()}
    ordered = sorted(elevations, key=elevations.get)

    def make(name: str, start_name: str, goal_name: str, criterion: str) -> dict:
        return {"criterion": criterion, "start_rc": global_cell(corners[start_name]),
                "goal_rc": global_cell(corners[goal_name]),
                "start_z": MISSION_CRUISE_MSL[name], "goal_z": MISSION_CRUISE_MSL[name],
                "altitude_contract": "explicit_roi_entry_cruise_msl"}

    return {"A_easy_open": make("A_easy_open", ordered[0], ordered[1],
                                 "retained low-relief XY endpoints; 3500 m MSL ROI entry cruise"),
            "B_relief_affected": make("B_relief_affected", ordered[0], ordered[-1],
                                        "retained relief-crossing XY endpoints; 3600 m MSL ROI entry cruise")}


def _path_metrics(result) -> dict:
    primitives = [node.incoming_primitive for node in result.nodes]
    return {"continuous_path_length_m": result.continuous_path_length_m,
            "physical_primitive_segments": len(result.trajectories),
            "turn_count": sum(p in ("LEFT_LEVEL_TURN", "RIGHT_LEVEL_TURN") for p in primitives),
            "climb_count": primitives.count("STRAIGHT_CLIMB"), "descent_count": primitives.count("STRAIGHT_DESCENT"),
            "minimum_agl_m": result.minimum_agl_m, "goal_xy_error_m": result.goal_xy_error_m,
            "goal_z_error_m": result.goal_z_error_m, "final_heading_deg": result.final_heading_deg}


def run_case(name: str, mission: dict, cache, timeout_s: float, max_expansions: int) -> dict:
    terrain = build_terrain_query_from_cache(cache, load_roi(DEFAULT_CONFIG), FACTOR)
    sx, sy = terrain.rowcol_to_xy(*mission["start_rc"])
    gx, gy = terrain.rowcol_to_xy(*mission["goal_rc"])
    heading = navigation_bearing_deg(sx, sy, gx, gy)
    result = pose_aware_astar_search(
        PhysicalPose(sx, sy, mission["start_z"], heading), GoalPose(gx, gy, mission["goal_z"]), terrain,
        load_aircraft_profile(PROFILE_PATH), goal_tolerance=GOAL_TOLERANCE, config=CONFIG,
        max_expansions=max_expansions, max_search_time_s=timeout_s)
    return {
        "name": name, "criterion": mission["criterion"], "status": result.status,
        "termination_reason": result.termination_reason, "success": result.success, "start_heading_deg": heading,
        "effective_min_agl_m": result.effective_min_agl_m, "lateral_buffer_m": result.lateral_buffer_m,
        "altitude_contract": mission["altitude_contract"], "start_z_msl_m": mission["start_z"], "goal_z_msl_m": mission["goal_z"],
        "goal_tolerance": {"xy_m": GOAL_TOLERANCE.xy_m, "altitude_m": GOAL_TOLERANCE.altitude_m},
        "runtime_s": result.runtime_s, "expanded_nodes": result.expanded_nodes,
        "generated_neighbors": result.generated_neighbors, "rejected_neighbors": result.rejected_neighbors,
        "peak_open": result.max_open_size, "peak_rss": None, "unique_search_keys": result.unique_search_keys,
        "unique_xy_bins": result.unique_xy_bins,
        "heading_bins_per_xy": {"mean": result.mean_heading_bins_per_xy, "median": result.median_heading_bins_per_xy, "max": result.max_heading_bins_per_xy},
        "z_bins_per_xy": {"mean": result.mean_z_bins_per_xy, "median": result.median_z_bins_per_xy, "max": result.max_z_bins_per_xy},
        "same_key": {"collisions": result.same_key_collision_count, "replaced_lower_g": result.same_key_replaced_lower_g,
                     "rejected_existing_better": result.same_key_rejected_existing_better,
                     "self_transitions": result.same_key_self_transition_count,
                     "self_transitions_by_primitive": result.same_key_self_transition_by_primitive},
        "reject_reasons": result.rejected_reason_counts, "primitive_counts": result.primitive_counts,
        "best_physical_xy_distance_to_goal_m": result.closest_xy_distance_to_goal_m,
        "best_physical_3d_distance_to_goal_m": result.closest_3d_distance_to_goal_m,
        "maximum_altitude_msl_m": result.maximum_altitude_msl_m, "progress_checkpoints": result.progress_checkpoints,
        "path": _path_metrics(result) if result.success else None,
        "safety": "PASS" if result.success and math.isfinite(result.minimum_agl_m) else ("NOT_APPLICABLE" if not result.success else "FAIL")}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mission", choices=("A", "B", "all"), default="A")
    parser.add_argument("--timeout", type=float, default=300.0)
    parser.add_argument("--max-expansions", type=int, default=30_000)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    cache = load_terrain_cache(CACHE_DIR, load_roi(DEFAULT_CONFIG), SOURCE_DEM_PATH)
    missions = mission_definitions(cache)
    selected = ("A_easy_open", "B_relief_affected") if args.mission == "all" else (("A_easy_open",) if args.mission == "A" else ("B_relief_affected",))
    payload = {"architecture": "pose_aware_fixed_wing_single_representative_approximate_search",
               "aircraft_profile": str(PROFILE_PATH.relative_to(ROOT)).replace("\\", "/"),
               "search_key": {"xy_m": CONFIG.search_xy_bin_m, "z_m": CONFIG.search_z_bin_m, "heading_deg": CONFIG.search_heading_bin_deg},
               "development_watchdog_s": args.timeout,
               "missions": {name: run_case(name, missions[name], cache, args.timeout, args.max_expansions) for name in selected}}
    rendered = json.dumps(payload, indent=2)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
