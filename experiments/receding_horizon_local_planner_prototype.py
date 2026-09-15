"""Controlled 3 km / 600 m receding-horizon wrapper; prototype only.

The production physical A* is deliberately called unchanged.  This module
only chooses a moving local GoalPose, executes a prefix of an already accepted
physical path, and records the resulting evidence.
"""
from __future__ import annotations

import json
import math
import statistics
from dataclasses import dataclass

import numpy as np
from affine import Affine

from planner.aircraft_profile import load_aircraft_profile
from planner.fixed_wing_envelope import FixedWingKinematicEnvelope
from planner.physical import PhysicalPose, PhysicalTrajectory, TrajectorySample
from planner.pose_search import GoalPose, navigation_bearing_deg, pose_aware_astar_search
from planner.roi import ROIData, load_roi
from planner.terrain import TerrainQuery
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from planner.trajectory_safety import evaluate_physical_trajectory_safety
from scripts.benchmark_far_missions import FAR_MISSIONS
from scripts.benchmark_missions import CACHE_DIR, CONFIG, FACTOR, GOAL_TOLERANCE, PROFILE_PATH, ROOT, SOURCE_DEM_PATH


OUTPUT_JSON = ROOT / "results" / "receding_horizon_local_planner_prototype.json"
OUTPUT_MD = ROOT / "results" / "receding_horizon_local_planner_prototype.md"
HORIZON_M = 3000.0
EXECUTION_STEP_M = 600.0
HARD_AGL_M = 100.0
SOFT_AGL_M = 120.0
MAX_EXPANSIONS = 30_000
MAX_TIME_S = 300.0
_EPS = 1e-8


@dataclass(frozen=True)
class ExecutedPiece:
    trajectory: PhysicalTrajectory
    primitive: str
    window_index: int


def _pose_dict(pose: PhysicalPose) -> dict:
    return {"x_m": pose.x_m, "y_m": pose.y_m, "z_msl_m": pose.z_msl_m,
            "heading_deg": pose.heading_deg}


def _distance_xy(first: PhysicalPose, goal: GoalPose) -> float:
    return math.hypot(goal.x_m - first.x_m, goal.y_m - first.y_m)


def _vertical_rate(envelope: FixedWingKinematicEnvelope, altitude_msl_m: float, mode: str) -> float:
    limit = envelope.straight_vertical(altitude_msl_m, mode)
    if limit.availability != "AVAILABLE" or limit.signed_vertical_rate_mps is None:
        return 0.0
    return float(limit.signed_vertical_rate_mps)


def _reachable_altitude(current_z: float, envelope: FixedWingKinematicEnvelope, mode: str) -> float:
    """Integrate the existing altitude-local straight vertical envelope for 3 km.

    The local target remains a diagnostic request, not a new dynamics model.
    Sixty metre increments exactly match the existing straight primitive's
    horizontal advance and retain the fixed 40 m/s physical convention.
    """
    altitude = current_z
    for _ in range(int(HORIZON_M / 60.0)):
        rate = _vertical_rate(envelope, altitude, mode)
        if mode == "CLIMB" and rate <= 0.0:
            break
        if mode == "DESCENT" and rate >= 0.0:
            break
        altitude += rate * 60.0 / 40.0
    return altitude


def _local_goal(current: PhysicalPose, global_goal: GoalPose, terrain: TerrainQuery,
                envelope: FixedWingKinematicEnvelope) -> tuple[GoalPose | None, dict]:
    remaining = _distance_xy(current, global_goal)
    if remaining <= HORIZON_M + _EPS:
        return global_goal, {
            "inside_global_horizon": True, "remaining_global_xy_m": remaining,
            "local_target": {"x_m": global_goal.x_m, "y_m": global_goal.y_m,
                             "z_msl_m": global_goal.z_msl_m, "heading_deg": global_goal.heading_deg},
            "global_goal_altitude_used": True,
        }

    dx, dy = global_goal.x_m - current.x_m, global_goal.y_m - current.y_m
    local_x, local_y = current.x_m + dx * HORIZON_M / remaining, current.y_m + dy * HORIZON_M / remaining
    terrain_at_target = terrain.query(local_x, local_y)
    lower = _reachable_altitude(current.z_msl_m, envelope, "DESCENT")
    upper = _reachable_altitude(current.z_msl_m, envelope, "CLIMB")
    metadata = {
        "inside_global_horizon": False, "remaining_global_xy_m": remaining,
        "local_x_m": local_x, "local_y_m": local_y, "local_terrain_msl_m": terrain_at_target.elevation,
        "terrain_valid": terrain_at_target.valid, "terrain_reason": terrain_at_target.reason,
        "requested_soft_target_z_msl_m": None if not terrain_at_target.valid else terrain_at_target.elevation + SOFT_AGL_M,
        "reachable_min_z_msl_m": lower, "reachable_max_z_msl_m": upper,
        "global_goal_altitude_used": False,
    }
    if not terrain_at_target.valid:
        metadata["target_failure"] = "INVALID_LOCAL_TARGET_TERRAIN"
        return None, metadata
    raw = terrain_at_target.elevation + SOFT_AGL_M
    target_z = min(max(raw, lower), upper)
    hard_floor = terrain_at_target.elevation + HARD_AGL_M
    metadata["physically_clamped_target_z_msl_m"] = target_z
    metadata["hard_floor_z_msl_m"] = hard_floor
    if target_z < hard_floor - _EPS:
        metadata["target_failure"] = "HARD_AGL_NOT_VERTICALLY_REACHABLE"
        return None, metadata
    local_goal = GoalPose(local_x, local_y, target_z)
    metadata["local_target"] = {"x_m": local_x, "y_m": local_y, "z_msl_m": target_z, "heading_deg": None}
    return local_goal, metadata


def _prefix_at_or_after(trajectory: PhysicalTrajectory, distance_m: float) -> PhysicalTrajectory:
    """Return an exact generated-sample prefix, never a reconstructed pose."""
    chosen = next((sample for sample in trajectory.samples
                   if sample.horizontal_distance_along_path_m + _EPS >= distance_m), trajectory.samples[-1])
    samples = tuple(sample for sample in trajectory.samples
                    if sample.horizontal_distance_along_path_m <= chosen.horizontal_distance_along_path_m + _EPS)
    end = chosen.pose
    return PhysicalTrajectory(trajectory.start_pose, end, chosen.horizontal_distance_along_path_m, samples)


def _executed_prefix(result, window_index: int, final_window: bool) -> tuple[list[ExecutedPiece], PhysicalPose, float]:
    pieces: list[ExecutedPiece] = []
    progress = 0.0
    for primitive, trajectory in zip(result.path_primitives, result.trajectories):
        if final_window:
            selected = trajectory
        else:
            selected = _prefix_at_or_after(trajectory, EXECUTION_STEP_M - progress)
        pieces.append(ExecutedPiece(selected, primitive, window_index))
        progress += selected.horizontal_arc_length_m
        if final_window or progress + _EPS >= EXECUTION_STEP_M:
            return pieces, selected.end_pose, progress
    # A local accepted path can be shorter than 600 m because its physical
    # endpoint is already in the goal tolerance. Executing it fully remains
    # truthful and triggers another local plan, never a teleport.
    assert pieces
    return pieces, pieces[-1].trajectory.end_pose, progress


def _stitched(pieces: list[ExecutedPiece]) -> PhysicalTrajectory:
    assert pieces
    samples: list[TrajectorySample] = []
    offset = 0.0
    for piece in pieces:
        source = piece.trajectory.samples if not samples else piece.trajectory.samples[1:]
        samples.extend(TrajectorySample(sample.x_m, sample.y_m, sample.z_msl_m, sample.heading_deg,
                                        offset + sample.horizontal_distance_along_path_m)
                       for sample in source)
        offset += piece.trajectory.horizontal_arc_length_m
    return PhysicalTrajectory(pieces[0].trajectory.start_pose, pieces[-1].trajectory.end_pose, offset, tuple(samples))


def _path_agl(trajectory: PhysicalTrajectory, terrain: TerrainQuery) -> list[float]:
    values = []
    for sample in trajectory.samples:
        hit = terrain.query(sample.x_m, sample.y_m)
        if hit.valid:
            values.append(sample.z_msl_m - hit.elevation)
    return values


def _percentile(values: list[float], fraction: float) -> float:
    return float(np.percentile(values, fraction * 100.0)) if values else float("nan")


def _safety_summary(pieces: list[ExecutedPiece], terrain: TerrainQuery) -> dict:
    stitched = _stitched(pieces)
    full = evaluate_physical_trajectory_safety(
        stitched, terrain, HARD_AGL_M, CONFIG.primitive_sample_spacing_m,
        planning_bounds=terrain.roi.bounds, lateral_buffer_m=CONFIG.lateral_buffer_m,
    )
    unsafe = 0
    reasons: dict[str, int] = {}
    for piece in pieces:
        checked = evaluate_physical_trajectory_safety(
            piece.trajectory, terrain, HARD_AGL_M, CONFIG.primitive_sample_spacing_m,
            planning_bounds=terrain.roi.bounds, lateral_buffer_m=CONFIG.lateral_buffer_m,
        )
        if not checked.is_safe:
            unsafe += 1
            reasons[checked.failure_reason or "UNKNOWN"] = reasons.get(checked.failure_reason or "UNKNOWN", 0) + 1
    outside = nodata = 0
    for sample in stitched.samples:
        hit = terrain.query(sample.x_m, sample.y_m)
        outside += int(hit.reason == "out_of_bounds")
        nodata += int(hit.reason == "nodata")
    return {"full_stitched_safe": full.is_safe, "minimum_agl_m": full.min_agl_m,
            "outside_dem_samples": outside, "nodata_samples": nodata,
            "unsafe_segments": unsafe, "unsafe_reasons": reasons,
            "failure_reason": full.failure_reason, "trajectory": stitched}


def _run_receding(name: str, start: PhysicalPose, global_goal: GoalPose,
                  terrain: TerrainQuery, profile) -> dict:
    envelope = FixedWingKinematicEnvelope()
    current = start
    pieces: list[ExecutedPiece] = []
    windows = []
    failure = None
    for index in range(1, 100):
        local_goal, target = _local_goal(current, global_goal, terrain, envelope)
        entry = {"index": index, "current_pose": _pose_dict(current), **target}
        if local_goal is None:
            entry["status"] = "UNREACHABLE"
            windows.append(entry)
            failure = entry
            break
        result = pose_aware_astar_search(
            current, local_goal, terrain, profile, goal_tolerance=GOAL_TOLERANCE, config=CONFIG,
            max_expansions=MAX_EXPANSIONS, max_search_time_s=MAX_TIME_S,
        )
        entry.update({"status": result.termination_reason, "expanded": result.expanded_nodes,
                      "generated": result.generated_neighbors, "peak_open": result.max_open_size,
                      "runtime_s": result.runtime_s, "local_path_length_m": result.continuous_path_length_m if result.success else None,
                      "local_minimum_agl_m": result.minimum_agl_m if result.success else None,
                      "primitive_identities": list(result.path_primitives) if result.success else [],
                      "best_partial_pose": _pose_dict(result.best_nodes[-1].end_pose)})
        windows.append(entry)
        if not result.success:
            failure = entry
            break
        chosen, current, executed_progress = _executed_prefix(result, index, target["inside_global_horizon"])
        pieces.extend(chosen)
        entry["executed_path_progress_m"] = executed_progress
        entry["executed_end_pose"] = _pose_dict(current)
        if target["inside_global_horizon"]:
            break
    else:
        failure = {"reason": "REPLAN_GUARD"}

    completed = bool(windows and windows[-1]["status"] == "FOUND" and windows[-1]["inside_global_horizon"])
    if pieces:
        safety = _safety_summary(pieces, terrain)
        trajectory = safety.pop("trajectory")
        agl = _path_agl(trajectory, terrain)
        excess = [value - SOFT_AGL_M for value in agl]
        path = {"length_m": trajectory.horizontal_arc_length_m, "samples": len(trajectory.samples),
                "end_pose": _pose_dict(trajectory.end_pose), "agl_m": {
                    "min": min(agl), "mean": statistics.fmean(agl), "median": statistics.median(agl),
                    "p90": _percentile(agl, .9), "max": max(agl)},
                "above_soft_agl": {"within_50m_percent": 100.0 * sum(value <= 50.0 for value in excess) / len(excess),
                                    "within_100m_percent": 100.0 * sum(value <= 100.0 for value in excess) / len(excess),
                                    "above_250m_percent": 100.0 * sum(value > 250.0 for value in excess) / len(excess),
                                    "above_500m_percent": 100.0 * sum(value > 500.0 for value in excess) / len(excess)},
                "safety": safety, "xy_error_m": _distance_xy(trajectory.end_pose, global_goal),
                "z_error_m": abs(trajectory.end_pose.z_msl_m - global_goal.z_msl_m),
                "trajectory": trajectory}
    else:
        path = None
    return {"name": name, "status": "FOUND" if completed else "FAILED", "failure": failure,
            "replans": len(windows), "windows": windows, "sum_expanded": sum(item.get("expanded", 0) for item in windows),
            "sum_generated": sum(item.get("generated", 0) for item in windows),
            "max_window_expanded": max((item.get("expanded", 0) for item in windows), default=0),
            "max_peak_open": max((item.get("peak_open", 0) for item in windows), default=0),
            "total_runtime_s": sum(item.get("runtime_s", 0.0) for item in windows), "path": path}


def _mission_pose(definition: dict, terrain: TerrainQuery) -> tuple[PhysicalPose, GoalPose]:
    sx, sy = terrain.rowcol_to_xy(*definition["start_rc"])
    gx, gy = terrain.rowcol_to_xy(*definition["goal_rc"])
    start_z = definition["z_msl_m"]
    goal_z = definition.get("goal_z_msl_m", start_z)
    heading = navigation_bearing_deg(sx, sy, gx, gy)
    return PhysicalPose(sx, sy, start_z, heading), GoalPose(gx, gy, goal_z)


def _synthetic_terrain(kind: str) -> TerrainQuery:
    """Create a 9 km by 6 km flat field with a deterministic terrain feature."""
    resolution, width, height = 30.0, 300, 200
    x = (np.arange(width) + .5) * resolution
    y = (np.arange(height) + .5) * resolution
    xx, yy = np.meshgrid(x, y)
    elevation = np.full((height, width), 1000.0)
    if kind == "hill":
        elevation += 650.0 * np.exp(-((xx - 3900.0) / 550.0) ** 2 - ((yy - 3000.0) / 520.0) ** 2)
    elif kind == "edge":
        rise = np.clip((xx - 3200.0) / 1200.0, 0.0, 1.0)
        elevation += 600.0 * rise * np.exp(-((yy - 3000.0) / 600.0) ** 2)
    else:
        raise ValueError(kind)
    roi = ROIData(elevation=elevation, transform=Affine(resolution, 0.0, 0.0, 0.0, -resolution, height * resolution),
                  crs="EPSG:32636", width=width, height=height, bounds=(0.0, 0.0, width * resolution, height * resolution),
                  resolution=(resolution, resolution), nodata=None)
    return TerrainQuery(roi)


def _synthetic_case(kind: str, profile) -> dict:
    terrain = _synthetic_terrain(kind)
    start = PhysicalPose(600.0, 3000.0, 1120.0, 90.0)
    goal = GoalPose(7800.0, 3000.0, 1120.0)
    result = _run_receding(f"synthetic_{kind}", start, goal, terrain, profile)
    trajectory = result["path"]["trajectory"] if result["path"] else None
    if trajectory is None:
        return {"result": result, "classification": "FAILED", "terrain": terrain}
    deviations = [abs(sample.y_m - 3000.0) for sample in trajectory.samples]
    altitude_gain = max(sample.z_msl_m for sample in trajectory.samples) - start.z_msl_m
    if kind == "hill":
        classification = "AROUND" if max(deviations) > 250.0 else ("OVER" if altitude_gain > 200.0 else "MIXED")
        details = {"maximum_altitude_increase_m": altitude_gain, "maximum_lateral_deviation_m": max(deviations),
                   "path_length_m": result["path"]["length_m"]}
    else:
        first_relevant = next((sample for sample in trajectory.samples if sample.x_m >= 3200.0), None)
        classification = "SAFE RECOVERY" if result["status"] == "FOUND" and result["path"]["safety"]["full_stitched_safe"] else "TOO-LATE / HORIZON FAILURE"
        details = {"hill_first_relevant_x_m": 3200.0, "aircraft_pose_when_relevant": None if first_relevant is None else _pose_dict(first_relevant.pose),
                   "available_distance_from_first_replan_m": 2600.0, "outcome": classification}
    return {"result": result, "classification": classification, "details": details, "terrain": terrain}


def _plot(payload: dict, terrain: TerrainQuery, synthetic: dict) -> str:
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(3, 2, figsize=(14, 14), constrained_layout=True)
    mission = payload["mission_e"]
    route = mission["path"]["trajectory"] if mission["path"] else None
    if route is not None:
        axes[0, 0].imshow(terrain.roi.elevation, extent=terrain.roi.bounds, origin="upper", cmap="terrain")
        axes[0, 0].plot([s.x_m for s in route.samples], [s.y_m for s in route.samples], "k-", lw=2, label="executed")
        axes[0, 0].set_title("Mission E executed top-down route"); axes[0, 0].set_aspect("equal"); axes[0, 0].legend()
        path_d = [s.horizontal_distance_along_path_m for s in route.samples]
        ground = [terrain.query(s.x_m, s.y_m).elevation for s in route.samples]
        axes[0, 1].plot(path_d, ground, color="saddlebrown", label="terrain")
        axes[0, 1].plot(path_d, [v + HARD_AGL_M for v in ground], "--", color="crimson", label="terrain +100")
        axes[0, 1].plot(path_d, [v + SOFT_AGL_M for v in ground], ":", color="green", label="terrain +120")
        axes[0, 1].plot(path_d, [s.z_msl_m for s in route.samples], color="black", label="aircraft")
        axes[0, 1].set_title("Mission E altitude profile"); axes[0, 1].legend(fontsize=8)
        windows = mission["windows"]
        axes[1, 0].bar([w["index"] for w in windows], [w.get("expanded", 0) for w in windows])
        axes[1, 0].set_title("Mission E expansions per local window"); axes[1, 0].set_xlabel("window"); axes[1, 0].set_ylabel("expanded")
    for axis, label, field in ((axes[1, 1], "Synthetic hill", synthetic["hill"]), (axes[2, 0], "Horizon-edge trap", synthetic["edge"])):
        item = field["result"]; path = item["path"]["trajectory"] if item["path"] else None; local_terrain = field["terrain"]
        axis.imshow(local_terrain.roi.elevation, extent=local_terrain.roi.bounds, origin="upper", cmap="terrain")
        if path is not None:
            axis.plot([s.x_m for s in path.samples], [s.y_m for s in path.samples], "k-", lw=2)
        axis.set_title(f"{label}: {field['classification']}"); axis.set_aspect("equal")
    edge = synthetic["edge"]["result"]; edge_path = edge["path"]["trajectory"] if edge["path"] else None
    if edge_path is not None:
        t = synthetic["edge"]["terrain"]; d = [s.horizontal_distance_along_path_m for s in edge_path.samples]; ground = [t.query(s.x_m, s.y_m).elevation for s in edge_path.samples]
        axes[2, 1].plot(d, ground, color="saddlebrown", label="terrain")
        axes[2, 1].plot(d, [v + HARD_AGL_M for v in ground], "--", color="crimson", label="terrain +100")
        axes[2, 1].plot(d, [s.z_msl_m for s in edge_path.samples], color="black", label="aircraft")
        axes[2, 1].legend(fontsize=8)
    axes[2, 1].set_title("Horizon-edge altitude / terrain")
    for axis in axes.flat:
        axis.grid(alpha=.2)
    path = ROOT / "results" / "receding_horizon_local_planner_prototype.png"
    figure.savefig(path, dpi=150); plt.close(figure)
    return str(path.relative_to(ROOT)).replace("\\", "/")


def _strip_trajectory(value):
    if isinstance(value, PhysicalTrajectory):
        return None
    if isinstance(value, dict):
        return {key: _strip_trajectory(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_strip_trajectory(item) for item in value]
    return value


def _markdown(payload: dict) -> str:
    e = payload["mission_e"]
    path = e["path"]
    lines = ["# Receding-horizon 3 km local planner prototype", "", "Diagnostic/prototype only. Production A* was not changed.", "",
             "- Architecture: 3 km receding-horizon diagnostic; execute approximately 600 m from exact generated physical samples.",
             "- Core A* modified: **NO**. SearchKey/Z bin/heuristic/g/primitive/dominance/safety remain unchanged.",
             "- Distant local-target global-goal altitude used: **NO**.", "",
             "## Mission E", "", f"- Status: **{e['status']}**; replans: {e['replans']}; total expansions: {e['sum_expanded']:,}; maximum window: {e['max_window_expanded']:,}; maximum OPEN: {e['max_peak_open']:,}; runtime: {e['total_runtime_s']:.2f} s."]
    if path:
        agl, safety = path["agl_m"], path["safety"]
        lines += [f"- Final XY/Z error: {path['xy_error_m']:.2f} / {path['z_error_m']:.2f} m; length: {path['length_m']:.1f} m.",
                  f"- AGL min/mean/median/p90: {agl['min']:.1f}/{agl['mean']:.1f}/{agl['median']:.1f}/{agl['p90']:.1f} m.",
                  f"- Full safety: **{'PASS' if safety['full_stitched_safe'] and safety['unsafe_segments'] == 0 else 'FAIL'}**; unsafe segments {safety['unsafe_segments']}; outside DEM {safety['outside_dem_samples']}; NoData {safety['nodata_samples']}.",
                  f"- Low-altitude: {json.dumps(path['above_soft_agl'])}"]
    lines += ["", "## Global E baseline", "", "- Canonical global BASIC search: EXPANSION_LIMIT; expanded 30,000; generated 149,995; peak OPEN 21,356.", "",
              "## Synthetic", ""]
    for name in ("hill", "edge"):
        lines.append(f"- {name}: **{payload['synthetic'][name]['classification']}**; {json.dumps(payload['synthetic'][name].get('details', {}))}")
    lines += ["", "## C/D/F sanity", ""]
    for name, entry in payload["sanity"].items():
        lines.append(f"- {name}: {entry['status']}; safety {entry['path']['safety']['full_stitched_safe'] if entry['path'] else False}.")
    lines += ["", "## Answers", "",
              f"- Q1 Mission E reached actual global goal: **{'YES' if e['status'] == 'FOUND' else 'NO'}**.",
              f"- Q2 Maximum local search substantially smaller than global: **{'YES' if e['max_window_expanded'] < 0.75 * MAX_EXPANSIONS else 'NO'}**.",
              f"- Q3 Low-altitude behavior improved: **{'YES' if path and path['above_soft_agl']['above_500m_percent'] < 50.0 else 'NO'}**.",
              f"- Q4 Around/over choice preserved: **{'YES' if payload['synthetic']['hill']['classification'] != 'FAILED' else 'NO'}**.",
              f"- Q5 Fixed horizon shows too-late failure: **{'YES' if payload['synthetic']['edge']['classification'] == 'TOO-LATE / HORIZON FAILURE' else 'NO'}**.",
              "- Q6 Any distant local target inherits global goal altitude: **NO**.",
              "- Q7 Production-ready: **NO -- prototype evidence only.**", "", f"Plot: `{payload['plot']}`."]
    return "\n".join(lines) + "\n"


def main() -> None:
    assert not CONFIG.enable_combined_turns and not CONFIG.enable_low_altitude_cost
    roi = load_roi(CONFIG)
    terrain = build_terrain_query_from_cache(load_terrain_cache(CACHE_DIR, roi, SOURCE_DEM_PATH), roi, FACTOR)
    profile = load_aircraft_profile(PROFILE_PATH)
    e_start, e_goal = _mission_pose(FAR_MISSIONS["E_long_descent_9_3km"], terrain)
    mission_e = _run_receding("E_long_descent_9_3km", e_start, e_goal, terrain, profile)
    sanity = {}
    for name in ("C_far_south_3km", "D_far_east_3km", "F_turn_required_diagonal_2_3km"):
        start, goal = _mission_pose(FAR_MISSIONS[name], terrain)
        sanity[name] = _run_receding(name, start, goal, terrain, profile)
    synthetic = {kind: _synthetic_case(kind, profile) for kind in ("hill", "edge")}
    raw = {"architecture": "3 km receding-horizon diagnostic", "plan_horizon_m": HORIZON_M,
           "execution_step_m": EXECUTION_STEP_M, "core_astar_modified": False,
           "global_goal_altitude_used_for_distant_local_target": False,
           "mission_e": mission_e, "sanity": sanity, "synthetic": synthetic,
           "global_e_baseline": {"status": "EXPANSION_LIMIT", "expanded": 30_000, "generated": 149_995, "peak_open": 21_356},
           "production_behavior_changed": False}
    raw["plot"] = _plot(raw, terrain, synthetic)
    payload = _strip_trajectory(raw)
    OUTPUT_JSON.write_text(json.dumps(payload, indent=2, allow_nan=True) + "\n", encoding="utf-8")
    OUTPUT_MD.write_text(_markdown(raw), encoding="utf-8")
    print(f"json: {OUTPUT_JSON}")
    print(f"report: {OUTPUT_MD}")


if __name__ == "__main__":
    main()
