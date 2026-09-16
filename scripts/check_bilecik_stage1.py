"""Bounded Bilecik-only checks for altitude-preserving pose search.

Run from the repository root: python -B -m scripts.check_bilecik_stage1
This checks selected existing missions, not the full 30-mission suite.
"""
import argparse
import dataclasses
import json
from pathlib import Path

from planner.config import DEFAULT_CONFIG
from planner.pose_search import GoalPose, GoalTolerance, navigation_bearing_deg, pose_aware_astar_search
from planner.physical import PhysicalPose
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from planner.terrain_following import optimize_terrain_following_altitudes
from planner.trajectory_safety import TerrainInfluenceCache
from scripts.bilecik_missions_spec import BILECIK_MISSIONS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--missions", nargs="+", default=["M01", "M03", "M08", "M11", "M18"])
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--guided", action="store_true")
    args = parser.parse_args()
    unknown = set(args.missions) - {m.id for m in BILECIK_MISSIONS}
    if unknown:
        parser.error(f"Unknown missions: {sorted(unknown)}")
    if not 0 < args.seconds < float("inf"):
        parser.error("--seconds must be finite and positive")
    config = dataclasses.replace(DEFAULT_CONFIG, min_agl_m=100.0,
                                 lateral_buffer_m=60.0, search_heuristic_weight=1.05,
                                 enable_terrain_guidance=args.guided)
    terrain = TerrainQuery(load_roi(config))
    field = TerrainInfluenceCache(terrain).field(config.lateral_buffer_m)
    records = []
    for mission in BILECIK_MISSIONS:
        if mission.id not in args.missions:
            continue
        def altitude(xy, offset):
            r, c = terrain.xy_to_rowcol(*xy)
            if not terrain.in_bounds_rowcol(r, c) or not field.valid[r, c]:
                raise ValueError(f"Invalid endpoint for {mission.id}: {xy}")
            return float(field.elevation_msl[r, c]) + max(130.0, offset)

        heading = mission.start_heading_deg
        if heading is None:
            heading = navigation_bearing_deg(*mission.start_xy, *mission.goal_xy)
        start = PhysicalPose(*mission.start_xy, altitude(mission.start_xy, mission.start_alt_offset_m), heading)
        goal = GoalPose(*mission.goal_xy, altitude(mission.goal_xy, mission.goal_alt_offset_m))
        result = pose_aware_astar_search(start, goal, terrain, config=config,
            goal_tolerance=GoalTolerance(mission.goal_tolerance_xy_m, mission.goal_tolerance_z_m),
            max_expansions=30000, max_search_time_s=args.seconds)
        profile = (optimize_terrain_following_altitudes(result.trajectories, terrain,
                   config=config, target_agl_m=120.0) if result.success else None)
        record = dict(mission=mission.id, name=mission.name,
            search_status=result.termination_reason, search_runtime_s=result.runtime_s,
            expanded_nodes=result.expanded_nodes,
            path_length_m=result.continuous_path_length_m if result.success else None,
            search_min_agl_m=result.minimum_agl_m if result.success else None,
            profile_status=profile.status if profile is not None else None,
            profile_success=profile.success if profile is not None else False,
            profile_min_agl_m=profile.minimum_agl_m if profile is not None and profile.success else None,
            rejected_reason_counts=result.rejected_reason_counts)
        records.append(record)
        print(json.dumps(record), flush=True)
    output = Path("results/test_bilecik/terrain_guided.json" if args.guided else
                  "results/test_bilecik/stage1_altitude_preservation.json")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(dict(dem=str(config.working_dem_path),
        min_agl_m=config.min_agl_m, lateral_buffer_m=config.lateral_buffer_m,
        time_limit_s=args.seconds, expansion_limit=30000,
        terrain_guidance=args.guided,
        cross_altitude_pruning=False, missions=records), indent=2, allow_nan=False), encoding="utf-8")


if __name__ == "__main__":
    main()
