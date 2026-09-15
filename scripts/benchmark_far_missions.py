"""Run and optionally plot ad-hoc distant pose-aware fixed-wing benchmarks.

The missions deliberately use roughly 3 km physical separation, substantially
larger than the local Mission A/B corner window.  Their explicit ROI-entry
cruise altitude is a mission initial condition, not a planner tuning knob.
For canonical controlled BASIC-vs-COMBINED A--F evidence, use
``scripts.benchmark_pose_aware_af`` instead.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

from planner.fixed_wing_envelope import FixedWingKinematicEnvelope
from planner.physical import PhysicalPose
from planner.pose_search import GoalPose, navigation_bearing_deg, pose_aware_astar_search
from planner.roi import load_roi
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from scripts.benchmark_missions import (
    CACHE_DIR, CONFIG, GOAL_TOLERANCE, SOURCE_DEM_PATH,
)


FAR_MISSIONS = {
    "C_far_south_3km": {
        "start_rc": (73, 73), "goal_rc": (123, 73), "z_msl_m": 3800.0,
        "criterion": "3.0 km southbound real-terrain crossing from the canonical origin",
    },
    "D_far_east_3km": {
        "start_rc": (73, 73), "goal_rc": (73, 123), "z_msl_m": 3800.0,
        "criterion": "3.0 km eastbound real-terrain crossing from the canonical origin",
    },
    "E_long_descent_9_3km": {
        "start_rc": (80, 5), "goal_rc": (80, 160), "z_msl_m": 4400.0,
        "goal_z_msl_m": 3900.0,
        "criterion": "9.3 km eastbound controlled 500 m descent across the ROI",
    },
    "F_turn_required_diagonal_2_3km": {
        "start_rc": (73, 73), "goal_rc": (100, 100), "z_msl_m": 3700.0,
        "criterion": "2.29 km diagonal approach requiring curved fixed-wing goal alignment",
    },
}


def _run_one(name: str, definition: dict, terrain, envelope):
    sx, sy = terrain.rowcol_to_xy(*definition["start_rc"])
    gx, gy = terrain.rowcol_to_xy(*definition["goal_rc"])
    heading = navigation_bearing_deg(sx, sy, gx, gy)
    start = PhysicalPose(sx, sy, definition["z_msl_m"], heading)
    goal = GoalPose(gx, gy, definition.get("goal_z_msl_m", definition["z_msl_m"]))
    result = pose_aware_astar_search(
        start, goal, terrain, envelope=envelope, goal_tolerance=GOAL_TOLERANCE, config=CONFIG,
        max_expansions=30_000, max_search_time_s=300.0,
    )
    separation = math.hypot(gx - sx, gy - sy)
    payload = {
        "name": name, "criterion": definition["criterion"], "start_rowcol": definition["start_rc"],
        "goal_rowcol": definition["goal_rc"], "physical_separation_m": separation,
        "altitude_contract": "explicit_roi_entry_cruise_msl", "start_z_msl_m": start.z_msl_m,
        "goal_z_msl_m": goal.z_msl_m, "start_heading_deg": start.heading_deg,
        "status": result.status, "termination_reason": result.termination_reason,
        "runtime_s": result.runtime_s, "expanded": result.expanded_nodes,
        "generated": result.generated_neighbors, "rejected": result.rejected_neighbors,
        "peak_open": result.max_open_size, "unique_search_keys": result.unique_search_keys,
        "same_key": {"collisions": result.same_key_collision_count,
                     "replaced_lower_g": result.same_key_replaced_lower_g,
                     "rejected_existing_better": result.same_key_rejected_existing_better,
                     "self_transitions": result.same_key_self_transition_count,
                     "self_transitions_by_primitive": result.same_key_self_transition_by_primitive},
        "reject_reasons": result.rejected_reason_counts,
        "search_statistics": {
            "generated_by_primitive": result.generated_by_primitive,
            "open_inserted_by_primitive": result.open_inserted_by_primitive,
            "expanded_arrivals_by_primitive": result.expanded_arrivals_by_primitive,
        },
        "best_xy_distance_to_goal_m": result.closest_xy_distance_to_goal_m,
        "best_3d_distance_to_goal_m": result.closest_3d_distance_to_goal_m,
        "maximum_altitude_msl_m": result.maximum_altitude_msl_m,
        "minimum_agl_m": result.minimum_agl_m if result.success else None,
        "path": None if not result.success else {
            "continuous_length_m": result.continuous_path_length_m,
            "segments": len(result.trajectories), "goal_xy_error_m": result.goal_xy_error_m,
            "goal_z_error_m": result.goal_z_error_m, "final_heading_deg": result.final_heading_deg,
            "primitives": list(result.path_primitives),
            "primitive_counts": result.path_primitive_counts,
        },
        "best_partial_path": None if result.success else {
            "continuous_length_m": sum(trajectory.horizontal_arc_length_m for node in result.best_nodes[1:]
                                         for trajectory in (node.incoming_trajectory,) if trajectory is not None),
            "segments": len(result.best_nodes) - 1,
            "end_xy_distance_to_goal_m": result.closest_xy_distance_to_goal_m,
            "end_3d_distance_to_goal_m": result.closest_3d_distance_to_goal_m,
            "end_pose": {"x_m": result.best_nodes[-1].end_pose.x_m, "y_m": result.best_nodes[-1].end_pose.y_m,
                         "z_msl_m": result.best_nodes[-1].end_pose.z_msl_m,
                         "heading_deg": result.best_nodes[-1].end_pose.heading_deg},
        },
    }
    return payload, result, start, goal


def _plot(results, terrain, path: Path) -> None:
    fig, axes = plt.subplots(1, len(results), figsize=(7.5 * len(results), 7), constrained_layout=True)
    if len(results) == 1:
        axes = (axes,)
    xmin, ymin, xmax, ymax = terrain.roi.bounds
    elevations = terrain.roi.elevation
    primitive_colors = {"STRAIGHT_LEVEL": "#0072B2", "LEFT_LEVEL_TURN": "#E69F00",
                        "RIGHT_LEVEL_TURN": "#CC79A7", "STRAIGHT_CLIMB": "#009E73",
                        "STRAIGHT_DESCENT": "#D55E00",
                        "CLIMBING_LEFT_TURN": "#009E73", "CLIMBING_RIGHT_TURN": "#009E73",
                        "DESCENDING_LEFT_TURN": "#D55E00", "DESCENDING_RIGHT_TURN": "#D55E00"}
    terrain_image = None
    for axis, (payload, result, start, goal) in zip(axes, results):
        terrain_image = axis.imshow(elevations, extent=(xmin, xmax, ymin, ymax), origin="upper", cmap="terrain", alpha=0.82)
        route_nodes = result.nodes[1:] if result.success else result.best_nodes[1:]
        route_trajectories = result.trajectories if result.success else tuple(
            node.incoming_trajectory for node in result.best_nodes[1:] if node.incoming_trajectory is not None
        )
        for node, trajectory in zip(route_nodes, route_trajectories):
            samples = trajectory.samples
            for first, second in zip(samples, samples[1:]):
                axis.plot((first.x_m, second.x_m), (first.y_m, second.y_m),
                          color=primitive_colors[node.incoming_primitive], linewidth=2.4, zorder=3)
        axis.scatter(start.x_m, start.y_m, marker="o", s=65, c="#00a651", edgecolors="black", label="Start", zorder=4)
        axis.scatter(goal.x_m, goal.y_m, marker="*", s=140, c="#d62728", edgecolors="black", label="Goal", zorder=4)
        arrow_length = 180.0
        heading_rad = math.radians(start.heading_deg)
        axis.quiver(start.x_m, start.y_m, arrow_length * math.sin(heading_rad), arrow_length * math.cos(heading_rad),
                    angles="xy", scale_units="xy", scale=1, color="black", width=0.004, zorder=5)
        status_label = "SUCCESS" if result.success else "BEST PARTIAL / EXPANSION LIMIT"
        axis.set_title(f"{payload['name']}\n{payload['physical_separation_m'] / 1000:.1f} km | {status_label}")
        axis.set_xlabel("UTM Easting (m)")
        axis.set_ylabel("UTM Northing (m)")
        axis.set_aspect("equal")
        axis.legend(loc="lower left")
    assert terrain_image is not None
    maneuver_handles = [Line2D([0], [0], color=color, lw=2.4, label=name.replace("_", " "))
                        for name, color in primitive_colors.items()]
    axes[-1].legend(handles=maneuver_handles, loc="lower right", fontsize=8, title="Primitive")
    fig.colorbar(terrain_image, ax=axes, fraction=0.025, pad=0.02, label="Terrain MSL (m)")
    fig.suptitle("Pose-aware fixed-wing continuous trajectories — distant benchmarks", fontsize=14)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mission", choices=("all", *FAR_MISSIONS), default="all")
    parser.add_argument("--output-json", type=Path,
                        help="optional JSON destination; canonical A--F results use benchmark_pose_aware_af")
    parser.add_argument("--output-plot", type=Path,
                        help="optional plot destination; canonical A--F results use benchmark_pose_aware_af")
    args = parser.parse_args()
    terrain = build_terrain_query_from_cache(
        load_terrain_cache(CACHE_DIR, load_roi(CONFIG), SOURCE_DEM_PATH), load_roi(CONFIG), 2
    )
    envelope = FixedWingKinematicEnvelope()
    selected = FAR_MISSIONS.items() if args.mission == "all" else ((args.mission, FAR_MISSIONS[args.mission]),)
    runs = [_run_one(name, definition, terrain, envelope) for name, definition in selected]
    payload = {
        "architecture": "pose_aware_fixed_wing_single_representative_approximate_search",
        "speed_mps": 40.0, "effective_min_agl_m": CONFIG.min_agl_m,
        "lateral_buffer_m": CONFIG.lateral_buffer_m,
        "search_key": {"xy_m": CONFIG.search_xy_bin_m, "z_m": CONFIG.search_z_bin_m,
                       "heading_deg": CONFIG.search_heading_bin_deg},
        "missions": {entry[0]["name"]: entry[0] for entry in runs},
    }
    if args.output_json is not None:
        args.output_json.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    if args.output_plot is not None:
        _plot(runs, terrain, args.output_plot)
    print(json.dumps(payload, indent=2))
    if args.output_plot is not None:
        print(f"plot: {args.output_plot}")


if __name__ == "__main__":
    main()
