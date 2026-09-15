"""End-to-end production search and lowest feasible altitude, with before/after plots.

python -B -m scripts.benchmark_low_flight
python -B -m scripts.benchmark_low_flight --suite valley --endpoint-mode agl
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import replace
import json
import math
from pathlib import Path

import numpy as np

from planner.physical import PhysicalPose, angular_distance_deg
from planner.pose_search import GoalPose, GoalTolerance, navigation_bearing_deg
from planner.roi import load_roi
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from planner.terrain_following import plan_terrain_following
from scripts.benchmark_missions import CACHE_DIR, CONFIG, FACTOR, SOURCE_DEM_PATH
from scripts.benchmark_pose_aware_af import _mission_definitions, _mission_altitudes


def behavior_missions():
    """Same endpoints as the user's B1--B6 figures, now using production code."""
    return {
        name: dict(start_rc=start, goal_rc=goal, start_z=z0, goal_z=z1,
                   tolerance=GoalTolerance(90, 10) if name.startswith("B5") else GoalTolerance(120, 30))
        for name, start, goal, z0, z1 in (
            ("B1_Mountain_Circumnavigation", (70, 30), (45, 55), 2450, 2480),
            ("B2_Left_Right_Bypass", (70, 30), (50, 50), 2450, 2450),
            ("B3_Ridge_Crossing_Chosen", (30, 10), (30, 40), 2350, 2440),
            ("B4_Ridge_Too_High_Detour", (70, 30), (45, 55), 2450, 2480),
            ("B5_S_Shaped_Corridor", (73, 73), (92, 92), 3600, 3600),
            ("B6_Narrow_Valley_Turn", (75, 5), (40, 5), 2174.8, 2020),
        )
    }


def telemetry(trajectories, terrain, target):
    points, rates, motions = [], [], Counter()
    offset = 0.0
    for trajectory in trajectories:
        for sample in trajectory.samples:
            ground = terrain.query(sample.x_m, sample.y_m).elevation
            points.append(dict(distance_m=offset + sample.horizontal_distance_along_path_m,
                               x_m=sample.x_m, y_m=sample.y_m, heading_deg=sample.heading_deg,
                               z_msl_m=sample.z_msl_m, terrain_msl_m=ground,
                               agl_m=sample.z_msl_m - ground))
        for a, b in zip(trajectory.samples, trajectory.samples[1:]):
            ds = b.horizontal_distance_along_path_m - a.horizontal_distance_along_path_m
            vz = (b.z_msl_m - a.z_msl_m) * 40.0 / ds
            rates.append(vz)
            turn = angular_distance_deg(a.heading_deg, b.heading_deg) > 1e-8
            mode = "CLIMB" if vz > 1e-7 else "DESCENT" if vz < -1e-7 else "LEVEL"
            motions[("TURN_" if turn else "STRAIGHT_") + mode] += ds
        offset += trajectory.horizontal_arc_length_m
    distance = np.array([p["distance_m"] for p in points])
    agl = np.array([p["agl_m"] for p in points])
    weights = np.diff(distance)
    return dict(points=points, mean_agl_m=float(np.sum((agl[:-1] + agl[1:]) * .5 * weights) / offset),
                minimum_centreline_agl_m=float(agl.min()),
                fraction_within_target_plus_20m=float(np.sum(weights * ((agl[:-1] + agl[1:]) * .5 <= target + 20)) / offset),
                max_climb_mps=max(0.0, max(rates)), max_descent_mps=max(0.0, -min(rates)),
                motion_distance_m=dict(motions), horizontal_length_m=offset)


def run_case(name, definition, terrain, config, target, endpoint_mode, endpoint_clearance=130.0):
    sx, sy = terrain.rowcol_to_xy(*definition["start_rc"])
    gx, gy = terrain.rowcol_to_xy(*definition["goal_rc"])
    z0, z1 = _mission_altitudes(definition)
    if endpoint_mode == "agl":
        z0 = terrain.query(sx, sy).elevation + endpoint_clearance
        z1 = terrain.query(gx, gy).elevation + endpoint_clearance
    start = PhysicalPose(sx, sy, z0, navigation_bearing_deg(sx, sy, gx, gy))
    goal = GoalPose(gx, gy, z1)
    tolerance = definition.get("tolerance", GoalTolerance(90, 10))
    result = plan_terrain_following(
        start, goal, terrain, goal_tolerance=tolerance, config=config,
        target_agl_m=target, max_expansions=30000, max_search_time_s=60,
    )
    search, profile = result.search_result, result.profile_result
    entry = dict(name=name, success=result.success, search_status=search.termination_reason,
                 profile_status=profile.status if profile else None, expanded=search.expanded_nodes,
                 runtime_s=search.runtime_s + (profile.runtime_s if profile else 0),
                 endpoint_mode=endpoint_mode, requested_start_msl_m=z0, requested_goal_msl_m=z1,
                 target_agl_m=target, hard_min_agl_m=config.min_agl_m,
                 goal_xy_error_m=search.goal_xy_error_m if search.success else None,
                 goal_z_error_m=search.goal_z_error_m if search.success else None,
                 search_primitive_counts=search.path_primitive_counts)
    if result.success:
        entry.update(before=telemetry(search.trajectories, terrain, target),
                     after=telemetry(result.trajectories, terrain, target),
                     minimum_verified_agl_m=profile.minimum_agl_m,
                     endpoints_preserved=(search.trajectories[0].start_pose == result.trajectories[0].start_pose
                                          and search.trajectories[-1].end_pose == result.trajectories[-1].end_pose))
    return entry


def plot_results(entries, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(len(entries), 1, figsize=(12, 3.2 * len(entries)), squeeze=False,
                             constrained_layout=True)
    for ax, entry in zip(axes[:, 0], entries):
        if not entry["success"]:
            ax.set_title(f"{entry['name']}: {entry['search_status']} / {entry['profile_status']}")
            continue
        before, after = entry["before"], entry["after"]
        points = after["points"]
        x = np.array([p["distance_m"] for p in points])
        ground = np.array([p["terrain_msl_m"] for p in points])
        ax.fill_between(x, ground, ground.min() - 30, color="#ad967a", alpha=.6, label="Terrain")
        ax.plot(x, ground + entry["hard_min_agl_m"], "r--", lw=1, label="Hard clearance (+100 m)")
        ax.plot(x, [p["z_msl_m"] for p in before["points"]], color="#777777", ls="--", label="Search altitude")
        ax.plot(x, [p["z_msl_m"] for p in points], color="#007f86", lw=2, label="Lowest feasible flight altitude")
        ax.set_title(f"{entry['name']} | Mean AGL: {before['mean_agl_m']:.1f} -> {after['mean_agl_m']:.1f} m"
                     f" | Verified min: {entry['minimum_verified_agl_m']:.1f} m")
        ax.set(xlabel="Ground-track distance (m)", ylabel="Altitude MSL (m)")
        ax.grid(alpha=.2)
        ax.legend(fontsize=8, ncol=2)
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", choices=("all", "behavior", "canonical", "valley"), default="all")
    parser.add_argument("--target-agl", type=float, default=100.0)
    parser.add_argument("--endpoint-mode", choices=("fixed", "agl"), default="fixed")
    parser.add_argument("--endpoint-clearance", type=float, default=130.0,
                        help="Start/goal AGL in agl mode; separate from in-flight target")
    parser.add_argument("--guided-search", action="store_true",
                        help="Use topographic Dijkstra and linear AGL search cost before profile optimization")
    args = parser.parse_args()
    if not math.isfinite(args.target_agl) or args.target_agl < 100:
        parser.error("--target-agl must be finite and >= 100 m")
    if not math.isfinite(args.endpoint_clearance) or args.endpoint_clearance < 100:
        parser.error("--endpoint-clearance must be finite and >= 100 m")
    config = replace(CONFIG, enable_combined_turns=True,
                     enable_low_altitude_cost=args.guided_search,
                     enable_terrain_guidance=args.guided_search,
                     low_altitude_cost_shape="linear", lambda_agl=.25,
                     agl_cost_scale_m=1000.0, desired_agl_m=args.target_agl)
    roi = load_roi(config)
    cache = load_terrain_cache(CACHE_DIR, roi, SOURCE_DEM_PATH)
    terrain = build_terrain_query_from_cache(cache, roi, FACTOR)
    groups = {}
    if args.suite in ("all", "behavior"):
        groups["behavior"] = behavior_missions()
    if args.suite in ("all", "canonical"):
        groups["canonical"] = _mission_definitions(cache)
    if args.suite in ("all", "valley"):
        groups["valley"] = {"Western_valley_AGL": dict(start_rc=(64, 5), goal_rc=(20, 5),
            start_z=terrain.roi.elevation[64, 5] + args.endpoint_clearance,
            goal_z=terrain.roi.elevation[20, 5] + args.endpoint_clearance,
            tolerance=GoalTolerance(120, 30))}
    out = Path("results/low_flight") / (args.endpoint_mode + ("_guided" if args.guided_search else ""))
    out.mkdir(parents=True, exist_ok=True)
    all_entries = []
    for group, missions in groups.items():
        entries = []
        for name, definition in missions.items():
            entry = run_case(name, definition, terrain, config, args.target_agl, args.endpoint_mode, args.endpoint_clearance)
            entries.append(entry)
            print(json.dumps({k: v for k, v in entry.items() if k not in ("before", "after")}), flush=True)
        (out / f"{group}.json").write_text(json.dumps(entries, indent=2, allow_nan=False), encoding="utf-8")
        plot_results(entries, out / f"{group}.png")
        all_entries.extend(entries)
    lines = ["# Low-flight production validation", "",
             f"Endpoint mode: {args.endpoint_mode}; target AGL: {args.target_agl:g} m; hard minimum: 100 m.", "",
             "Search and altitude refinement use production modules. The altitude solver finds the lowest feasible profile on the selected ground track; it does not prove a globally optimal 3D route.", "",
             "| Mission | Result | Mean AGL before / after (m) | Verified minimum (m) | Max climb / descent (m/s) |",
             "|---|---|---:|---:|---:|"]
    for e in all_entries:
        if e["success"]:
            a = e["after"]
            lines.append(f"| {e['name']} | FOUND | {e['before']['mean_agl_m']:.1f} / {a['mean_agl_m']:.1f} | {e['minimum_verified_agl_m']:.1f} | {a['max_climb_mps']:.2f} / {a['max_descent_mps']:.2f} |")
        else:
            lines.append(f"| {e['name']} | {e['search_status']} / {e['profile_status']} | - | - | - |")
    (out / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    if not all(e["success"] for e in all_entries):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
