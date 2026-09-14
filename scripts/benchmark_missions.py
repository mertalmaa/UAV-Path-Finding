"""Current real-terrain CandidateZ + AircraftProfile benchmark runner.

Mission A is the default. Mission B is opt-in because it is a long performance
workload; Mission C is intentionally not exposed until its current contract is
defined in production documentation.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import time
from pathlib import Path

import numpy as np

from planner.aircraft_profile import load_aircraft_profile
from planner.astar import (
    _path_altitude_metrics,
    _path_min_observed_agl,
    astar_search,
    encode_candidate_altitude,
    state_to_xyz,
    validate_path_safety,
)
from planner.candidate_z import CacheBackedTerrainMetadataStore, CandidateZGenerator, MissionContext
from planner.config import DEFAULT_CONFIG
from planner.roi import load_roi
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache


ROOT = Path(__file__).resolve().parents[1]
CACHE_DIR = ROOT / "outputs" / "terrain_cache"
PROFILE_PATH = ROOT / "jsbsim" / "results" / "c172p_aircraft_profile_planner_safe_v3.json"
SOURCE_DEM_PATH = str(DEFAULT_CONFIG.working_dem_path)
FACTOR = 2
MIN_AGL_M = 100.0
CEILING_MSL = 6000.0
CONFIG = dataclasses.replace(DEFAULT_CONFIG, xy_resolution_m=60.0, min_agl_m=MIN_AGL_M)


def mission_definitions(cache) -> dict[str, dict]:
    """Derive stable benchmark endpoints from objective terrain-window rules."""
    row0, row1 = 146 // FACTOR, 186 // FACTOR
    col0, col1 = 146 // FACTOR, 186 // FACTOR
    window = cache._arrays[f"max_elevation_f{FACTOR}"][row0:row1, col0:col1]

    def floor(elevation: float) -> float:
        return math.ceil((elevation + MIN_AGL_M) / CONFIG.z_step_m) * CONFIG.z_step_m

    def global_cell(local_cell: tuple[int, int]) -> tuple[int, int]:
        return row0 + local_cell[0], col0 + local_cell[1]

    height, width = window.shape
    corners = {"TL": (0, 0), "TR": (0, width - 1), "BL": (height - 1, 0), "BR": (height - 1, width - 1)}
    corner_elevations = {name: float(window[cell]) for name, cell in corners.items()}
    ordered = sorted(corner_elevations, key=corner_elevations.get)
    low, second_low, high = ordered[0], ordered[1], ordered[-1]

    def make(start_name: str, goal_name: str, criterion: str) -> dict:
        return {
            "criterion": criterion,
            "start_rc": global_cell(corners[start_name]),
            "goal_rc": global_cell(corners[goal_name]),
            "start_z": floor(corner_elevations[start_name]),
            "goal_z": floor(corner_elevations[goal_name]),
        }

    return {
        "A_easy_open": make(low, second_low, "two lowest-elevation corners"),
        "B_relief_affected": make(low, high, "lowest-to-highest corner across relief"),
    }


def run_case(name: str, mission: dict, cache, timeout_s: float, max_expansions: int) -> dict:
    fine_roi = load_roi(DEFAULT_CONFIG)
    terrain = build_terrain_query_from_cache(cache, fine_roi, FACTOR)
    generator = CandidateZGenerator(
        CacheBackedTerrainMetadataStore(cache, FACTOR),
        MissionContext(
            start_rowcol=mission["start_rc"],
            start_z_msl=mission["start_z"],
            goal_rowcol=mission["goal_rc"],
            goal_z_msl=mission["goal_z"],
            ceiling_msl=CEILING_MSL,
            min_agl_m=MIN_AGL_M,
            z_step_m=CONFIG.z_step_m,
        ),
    )
    profile = load_aircraft_profile(PROFILE_PATH)
    start = (*mission["start_rc"], encode_candidate_altitude(mission["start_z"]))
    goal = (*mission["goal_rc"], encode_candidate_altitude(mission["goal_z"]))

    started = time.perf_counter()
    result = astar_search(
        start,
        goal,
        terrain,
        min_search_altitude_msl=min(mission["start_z"], mission["goal_z"]),
        max_search_altitude_msl=CEILING_MSL,
        config=CONFIG,
        max_expansions=max_expansions,
        max_search_time_s=timeout_s,
        candidate_z_generator=generator,
        aircraft_profile=profile,
    )
    elapsed = time.perf_counter() - started

    safety = validate_path_safety(result.path, terrain, CONFIG) if result.path else None
    minimum_agl = _path_min_observed_agl(result.path, terrain, CONFIG) if result.path else None
    altitude_metrics = _path_altitude_metrics(result.path, terrain) if result.path else None
    goal_error_xy = goal_error_z = None
    if result.path:
        gx, gy, gz = state_to_xyz(goal, terrain)
        fx, fy, fz = state_to_xyz(result.path[-1], terrain)
        goal_error_xy = math.hypot(fx - gx, fy - gy)
        goal_error_z = abs(fz - gz)

    return {
        "name": name,
        "criterion": mission["criterion"],
        "status": result.status,
        "termination_reason": result.termination_reason,
        "success": result.success,
        "safety_pass": safety,
        "min_agl_m": minimum_agl,
        "goal_error_xy_m": goal_error_xy,
        "goal_error_z_m": goal_error_z,
        "node_count": len(result.path) if result.path else 0,
        "path_cost": result.total_cost if result.path else None,
        "expanded_nodes": result.expanded_nodes,
        "generated_neighbors": result.generated_neighbors,
        "rejected_neighbors": result.rejected_neighbors,
        "search_time_s": elapsed,
        "vertical_rate_cache": {
            "lookups": result.vertical_rate_cache_lookups,
            "hits": result.vertical_rate_cache_hits,
            "misses": result.vertical_rate_cache_misses,
            "hit_rate": result.vertical_rate_cache_hit_rate,
            "aircraft_profile_queries": result.aircraft_profile_vertical_queries,
            "aircraft_profile_query_time_s": result.aircraft_profile_vertical_query_time_s,
        },
        "altitude_metrics": altitude_metrics,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mission", choices=("A", "B", "all"), default="A")
    parser.add_argument("--timeout", type=float, default=300.0, help="per-mission development watchdog in seconds")
    parser.add_argument("--max-expansions", type=int, default=30_000)
    parser.add_argument("--output", type=Path, help="optional JSON output path")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    fine_roi = load_roi(DEFAULT_CONFIG)
    cache = load_terrain_cache(CACHE_DIR, fine_roi, SOURCE_DEM_PATH)
    missions = mission_definitions(cache)
    mission_names = {"A": "A_easy_open", "B": "B_relief_affected"}
    selected = tuple(mission_names.values()) if args.mission == "all" else (mission_names[args.mission],)
    results = {name: run_case(name, missions[name], cache, args.timeout, args.max_expansions) for name in selected}
    payload = {
        "architecture": "candidate_z_with_aircraft_profile_and_multi_cell_vertical_horizon",
        "aircraft_profile": str(PROFILE_PATH.relative_to(ROOT)).replace("\\", "/"),
        "development_watchdog_s": args.timeout,
        "missions": results,
    }
    rendered = json.dumps(payload, indent=2)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
