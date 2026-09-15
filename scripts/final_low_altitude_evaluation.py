"""Final Benchmark and Multi-Panel Vertical Flight Profile Generator.

Runs canonical Missions A-F for:
1. Baseline Weighted A* (w=1.01, pure geometric, FIFO)
2. Proposed Feasible Envelope Guidance (w=1.01 + Feasible Backward-Envelope Dominance)

Generates exact comparative metrics table and publication-quality vertical profile plots for C, D, E, F.
"""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass
import heapq
import itertools
import json
import math
from pathlib import Path
import shutil
import sys
import time
from collections import Counter, defaultdict
from typing import Any, Callable, Dict, List, Optional, Set, Tuple

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from planner.config import DEFAULT_CONFIG, PlannerConfig
from planner.fixed_wing_envelope import FixedWingKinematicEnvelope
from planner.physical import (
    FIXED_PLANAR_SPEED_MPS,
    PhysicalPose,
    PhysicalTrajectory,
    angular_distance_deg,
    build_helical_turn_trajectory,
    build_level_turn_trajectory,
    build_straight_level_trajectory,
    build_straight_vertical_trajectory,
    normalize_heading_deg,
)
from planner.pose_search import (
    GoalPose,
    GoalTolerance,
    PoseSearchNode,
    PoseSearchResult,
    SearchKey,
    _candidate_trajectories,
    _floor_bin,
    _goal_errors,
    _heuristic,
    _median,
    _safety_reason,
    _trajectory_3d_length,
    navigation_bearing_deg,
    pose_in_goal,
    search_key_for_pose,
)
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from planner.trajectory_safety import (
    TerrainInfluenceCache,
    evaluate_physical_trajectory_safety,
)
from scripts.benchmark_far_missions import FAR_MISSIONS
from scripts.benchmark_missions import (
    CACHE_DIR,
    CONFIG,
    FACTOR,
    GOAL_TOLERANCE,
    PROFILE_PATH,
    SOURCE_DEM_PATH,
    mission_definitions,
)

ARTIFACT_DIR = Path(r"C:\Users\PC_10004_YD26\.gemini\antigravity-ide\brain\703ac52a-e972-41a4-9422-6001171f41b7")


def compute_1d_backward_feasible_envelope(
    start: PhysicalPose,
    goal: GoalPose,
    terrain: TerrainQuery,
    target_agl_m: float = 120.0,
    climb_slope: float = 0.125,
    sample_spacing_m: float = 30.0,
) -> Tuple[List[float], List[float], List[float]]:
    dx = goal.x_m - start.x_m
    dy = goal.y_m - start.y_m
    total_dist = math.hypot(dx, dy)
    num_samples = max(2, int(math.ceil(total_dist / sample_spacing_m)) + 1)

    distances = [i * total_dist / (num_samples - 1) for i in range(num_samples)]
    terrain_elevs = []
    for d in distances:
        t = d / total_dist
        x = start.x_m + t * dx
        y = start.y_m + t * dy
        q = terrain.query(x, y)
        elev = q.elevation if q.valid and math.isfinite(q.elevation) else 0.0
        terrain_elevs.append(elev)

    z_floor = [elev + target_agl_m for elev in terrain_elevs]
    z_floor[-1] = max(z_floor[-1], goal.z_msl_m - 5.0)

    for i in range(len(z_floor) - 2, -1, -1):
        ds = distances[i+1] - distances[i]
        z_floor[i] = max(z_floor[i], z_floor[i+1] - climb_slope * ds)

    z_floor[0] = min(z_floor[0], start.z_msl_m)
    for i in range(len(z_floor) - 1):
        ds = distances[i+1] - distances[i]
        max_descend_to = z_floor[i] - climb_slope * ds
        z_floor[i+1] = max(z_floor[i+1], max_descend_to)

    return distances, terrain_elevs, z_floor


def run_search(
    start: PhysicalPose,
    goal: GoalPose,
    terrain: TerrainQuery,
    aircraft_profile,
    mode: str = "baseline", # "baseline" or "proposed"
    delta_g_m: float = 25.0,
    target_agl_m: float = 120.0,
    goal_tolerance: GoalTolerance = GOAL_TOLERANCE,
    config: PlannerConfig = CONFIG,
    max_expansions: int = 30000,
    max_search_time_s: float = 60.0,
) -> Tuple[PoseSearchResult, Dict[str, Any]]:
    envelope = FixedWingKinematicEnvelope()
    influence_cache = TerrainInfluenceCache(terrain)
    started = time.perf_counter()
    counter = itertools.count()
    next_node_id = itertools.count()

    g_dists, g_elevs, g_z_floor = compute_1d_backward_feasible_envelope(
        start, goal, terrain, target_agl_m, climb_slope=0.125, sample_spacing_m=30.0,
    )
    total_baseline = g_dists[-1]

    def get_feasible_floor(pose: PhysicalPose) -> float:
        dx = goal.x_m - start.x_m
        dy = goal.y_m - start.y_m
        L2 = dx*dx + dy*dy
        if L2 == 0:
            return start.z_msl_m
        t = ((pose.x_m - start.x_m)*dx + (pose.y_m - start.y_m)*dy) / L2
        d_proj = max(0.0, min(total_baseline, t * total_baseline))
        idx = int(d_proj / 30.0)
        if idx >= len(g_dists) - 1:
            z_base = g_z_floor[-1]
        else:
            alpha = (d_proj - g_dists[idx]) / (g_dists[idx+1] - g_dists[idx])
            z_base = (1 - alpha) * g_z_floor[idx] + alpha * g_z_floor[idx+1]

        q = terrain.query(pose.x_m, pose.y_m)
        loc_elev = q.elevation if q.valid and math.isfinite(q.elevation) else 0.0
        return max(z_base, loc_elev + target_agl_m)

    start_key = search_key_for_pose(start, config)
    start_node = PoseSearchNode(next(next_node_id), start_key, start, 0.0, None, None, None)
    active: Dict[SearchKey, PoseSearchNode] = {start_key: start_node}
    all_nodes: Dict[int, PoseSearchNode] = {start_node.node_id: start_node}

    xyh_representatives: Dict[Tuple[int, int, int], List[Tuple[float, float, SearchKey, int]]] = defaultdict(list)
    if mode == "proposed":
        start_xyh = (start_key.x_bin, start_key.y_bin, start_key.heading_bin)
        z_floor_start = get_feasible_floor(start)
        start_excess = max(0.0, start.z_msl_m - z_floor_start)
        xyh_representatives[start_xyh].append((0.0, start_excess, start_key, start_node.node_id))

    h_0 = _heuristic(start, goal, goal_tolerance)
    f_0 = 0.0 + 1.01 * h_0
    open_heap = [(f_0, next(counter), start_node.node_id)]

    expanded_ids = set()
    expanded_keys = set()
    expanded = generated = rejected = 0
    max_open = 1
    reject_reasons: Counter[str] = Counter()
    generated_by_primitive: Counter[str] = Counter()
    open_inserted_by_primitive: Counter[str] = Counter()
    expanded_arrivals_by_primitive: Counter[str] = Counter()
    self_by_primitive: Counter[str] = Counter()
    collisions = replaced = existing_better = self_transitions = 0
    closest_xy, closest_3d = _goal_errors(start, goal)[0], _goal_errors(start, goal)[2]
    best_node, best_node_d3 = start_node, closest_3d
    maximum_altitude = start.z_msl_m
    progress: Dict[int, Dict[str, float]] = {}
    checkpoints = {1000, 5000, 10000, 20000, 30000}
    goal_node: Optional[PoseSearchNode] = None
    status = "no_path"

    while open_heap:
        if max_search_time_s is not None and time.perf_counter() - started >= max_search_time_s:
            status = "timeout"
            break
        max_open = max(max_open, len(open_heap))
        popped = heapq.heappop(open_heap)
        node_id = popped[-1]
        node = all_nodes[node_id]

        if active.get(node.key) is not node or node_id in expanded_ids:
            continue

        expanded_ids.add(node_id)
        expanded_keys.add(node.key)
        expanded += 1

        if node.incoming_primitive is not None:
            expanded_arrivals_by_primitive[node.incoming_primitive] += 1
        xy_error, _, d3_error = _goal_errors(node.end_pose, goal)
        closest_xy, closest_3d = min(closest_xy, xy_error), min(closest_3d, d3_error)
        if d3_error < best_node_d3 - 1e-12 or (
            math.isclose(d3_error, best_node_d3, rel_tol=0.0, abs_tol=1e-12)
            and node.g_cost < best_node.g_cost
        ):
            best_node, best_node_d3 = node, d3_error
        maximum_altitude = max(maximum_altitude, node.end_pose.z_msl_m)
        if expanded in checkpoints:
            progress[expanded] = {
                "best_xy_distance_m": closest_xy,
                "best_3d_distance_m": closest_3d,
                "maximum_altitude_msl_m": maximum_altitude,
            }
        if pose_in_goal(node.end_pose, goal, goal_tolerance):
            goal_node, status = node, "success"
            break
        if max_expansions is not None and expanded >= max_expansions:
            status = "search_limit_reached"
            break

        for primitive, trajectory in _candidate_trajectories(node.end_pose, envelope, config):
            generated += 1
            generated_by_primitive[primitive] += 1
            if trajectory is None:
                rejected += 1
                reject_reasons["UNAVAILABLE_CAPABILITY"] += 1
                continue

            safety = evaluate_physical_trajectory_safety(
                trajectory, terrain, config.min_agl_m, config.primitive_sample_spacing_m,
                planning_bounds=terrain.roi.bounds, lateral_buffer_m=config.lateral_buffer_m,
                terrain_influence_cache=influence_cache,
            )
            if not safety.is_safe:
                rejected += 1
                reject_reasons[_safety_reason(safety)] += 1
                continue

            end_pose = trajectory.end_pose
            key = search_key_for_pose(end_pose, config)
            if key == node.key:
                self_transitions += 1
                self_by_primitive[primitive] += 1
                rejected += 1
                reject_reasons["SAME_KEY_SELF_TRANSITION"] += 1
                continue

            candidate_g = node.g_cost + _trajectory_3d_length(trajectory)
            existing = active.get(key)
            if existing is not None:
                collisions += 1
                if candidate_g >= existing.g_cost - 1e-12:
                    existing_better += 1
                    rejected += 1
                    reject_reasons["SAME_KEY_DOMINANCE"] += 1
                    continue
                replaced += 1

            if mode == "proposed":
                xyh = (key.x_bin, key.y_bin, key.heading_bin)
                z_fl = get_feasible_floor(end_pose)
                cand_excess = max(0.0, end_pose.z_msl_m - z_fl)

                reps = xyh_representatives[xyh]
                dominated = False
                for rep_g, rep_excess, rep_key, _ in reps:
                    if rep_excess <= cand_excess and rep_g <= candidate_g + delta_g_m:
                        dominated = True
                        break
                if dominated:
                    rejected += 1
                    reject_reasons["BACKWARD_ENVELOPE_DOMINANCE"] += 1
                    continue

                xyh_representatives[xyh] = [
                    (rg, re, rk, rn) for rg, re, rk, rn in reps
                    if not (cand_excess <= re and candidate_g <= rg + delta_g_m)
                ]
                xyh_representatives[xyh].append((candidate_g, cand_excess, key, node.node_id))

            successor = PoseSearchNode(
                next(next_node_id), key, end_pose, candidate_g, node.node_id, trajectory, primitive,
            )
            active[key] = successor
            all_nodes[successor.node_id] = successor
            open_inserted_by_primitive[primitive] += 1
            h_score = _heuristic(end_pose, goal, goal_tolerance)
            f_score = candidate_g + 1.01 * h_score
            heapq.heappush(open_heap, (f_score, next(counter), successor.node_id))

    if status == "no_path" and not open_heap:
        status = "no_path"
    runtime = time.perf_counter() - started

    def reconstruct(n: PoseSearchNode) -> Tuple[PoseSearchNode, ...]:
        rev = []
        cur: Optional[PoseSearchNode] = n
        while cur is not None:
            rev.append(cur)
            cur = all_nodes[cur.parent_node_id] if cur.parent_node_id is not None else None
        return tuple(reversed(rev))

    best_path = reconstruct(goal_node if goal_node is not None else best_node)
    path_len = 0.0
    min_agl = mean_agl = median_agl = p90_agl = float("nan")
    goal_xy_error = goal_z_error = final_heading = float("nan")
    all_agls = []
    trajectory_points = []

    if goal_node is not None:
        path = best_path
        total_cost = goal_node.g_cost
        goal_xy_error, goal_z_error, _ = _goal_errors(goal_node.end_pose, goal)
        final_heading = goal_node.end_pose.heading_deg

        cum_dist = 0.0
        for item in path[1:]:
            tr = item.incoming_trajectory
            if tr is not None:
                seg_len = _trajectory_3d_length(tr)
                path_len += seg_len
                for s in tr.samples:
                    q = terrain.query(s.x_m, s.y_m)
                    elev = q.elevation if q.valid and math.isfinite(q.elevation) else 0.0
                    agl = s.z_msl_m - elev
                    all_agls.append(agl)
                    trajectory_points.append({
                        "x_m": s.x_m, "y_m": s.y_m, "z_msl_m": s.z_msl_m,
                        "elevation_msl_m": elev, "agl_m": agl,
                    })

        if all_agls:
            all_agls_sorted = sorted(all_agls)
            min_agl = all_agls_sorted[0]
            mean_agl = sum(all_agls_sorted) / len(all_agls_sorted)
            median_agl = all_agls_sorted[len(all_agls_sorted) // 2]
            p90_idx = int(0.90 * len(all_agls_sorted))
            p90_agl = all_agls_sorted[min(p90_idx, len(all_agls_sorted) - 1)]
    else:
        path = ()
        total_cost = float("nan")

    xy_to_heading: Dict[Tuple[int, int], set] = defaultdict(set)
    xy_to_z: Dict[Tuple[int, int], set] = defaultdict(set)
    for k in expanded_keys:
        xy = (k.x_bin, k.y_bin)
        xy_to_heading[xy].add(k.heading_bin)
        xy_to_z[xy].add(k.z_bin)
    heading_counts = [len(v) for v in xy_to_heading.values()]
    z_counts = [len(v) for v in xy_to_z.values()]
    reasons = dict(sorted(reject_reasons.items()))
    termination = {"success": "FOUND", "no_path": "OPEN_EXHAUSTED", "search_limit_reached": "EXPANSION_LIMIT", "timeout": "TIMEOUT"}[status]

    res = PoseSearchResult(
        status == "success", status, termination, path, total_cost, expanded, generated, rejected, reasons,
        max_open, runtime, len(expanded_keys), len(xy_to_heading),
        sum(heading_counts) / len(heading_counts) if heading_counts else float("nan"), _median(heading_counts), max(heading_counts, default=0),
        sum(z_counts) / len(z_counts) if z_counts else float("nan"), _median(z_counts), max(z_counts, default=0),
        collisions, replaced, existing_better, self_transitions, dict(sorted(self_by_primitive.items())),
        dict(sorted(generated_by_primitive.items())), dict(sorted(open_inserted_by_primitive.items())),
        dict(sorted(expanded_arrivals_by_primitive.items())), min_agl, closest_xy, closest_3d, maximum_altitude, progress,
        config.lateral_buffer_m, config.min_agl_m, goal_xy_error, goal_z_error, final_heading,
        best_path,
    )

    metrics = {
        "status": status,
        "success": status == "success",
        "expansions": expanded,
        "generated": generated,
        "peak_open": max_open,
        "runtime_s": runtime,
        "path_length_m": path_len,
        "min_agl_m": min_agl,
        "mean_agl_m": mean_agl,
        "median_agl_m": median_agl,
        "p90_agl_m": p90_agl,
        "goal_xy_error_m": goal_xy_error,
        "goal_z_error_m": goal_z_error,
        "safety": "PASS" if status == "success" and min_agl >= config.min_agl_m - 1e-3 else ("NOT_APPLICABLE" if status != "success" else "FAIL"),
        "trajectory": trajectory_points,
    }
    return res, metrics


def plot_altitude_profiles(all_data: dict, out_png: Path):
    """Plot altitude profiles for Missions C, D, E, F."""
    fig, axes = plt.subplots(2, 2, figsize=(16, 11), constrained_layout=True)
    mission_keys = [("C", axes[0, 0]), ("D", axes[0, 1]), ("E", axes[1, 0]), ("F", axes[1, 1])]

    for m_id, ax in mission_keys:
        b_traj = all_data["baseline"][m_id]["trajectory"]
        p_traj = all_data["proposed"][m_id]["trajectory"]

        # Compute cumulative distance
        def get_series(traj):
            dists = [0.0]
            for i in range(1, len(traj)):
                p0, p1 = traj[i-1], traj[i]
                d = math.hypot(p1["x_m"] - p0["x_m"], p1["y_m"] - p0["y_m"])
                dists.append(dists[-1] + d)
            alts = [p["z_msl_m"] for p in traj]
            terrains = [p["elevation_msl_m"] for p in traj]
            agls = [p["agl_m"] for p in traj]
            return dists, alts, terrains, agls

        b_d, b_alt, b_terr, b_agl = get_series(b_traj)
        p_d, p_alt, p_terr, p_agl = get_series(p_traj)

        ax.plot(b_d, b_alt, color="#d62728", linestyle="--", linewidth=2.0, label="Baseline w=1.01 Altitude (MSL)")
        ax.plot(p_d, p_alt, color="#1f77b4", linestyle="-", linewidth=2.2, label="Proposed Feasible Altitude (MSL)")
        ax.plot(p_d, p_terr, color="#7f7f7f", linewidth=1.5, label="Terrain Elevation (MSL)")
        
        # Terrain + 100m hard floor
        p_floor = [t + 100.0 for t in p_terr]
        ax.fill_between(p_d, p_terr, p_floor, color="#ff7f0e", alpha=0.25, label="Hard Safety Floor (100m AGL)")
        ax.fill_between(p_d, 0, p_terr, color="#d3d3d3", alpha=0.5)

        ax.set_title(f"Mission {m_id}: Altitude Profile (Baseline vs Proposed)", fontsize=12, fontweight="bold")
        ax.set_xlabel("Cumulative Path Distance (m)", fontsize=10)
        ax.set_ylabel("Altitude MSL (m)", fontsize=10)
        ax.grid(True, linestyle=":", alpha=0.6)
        ax.legend(fontsize=8, loc="best")

    fig.suptitle("UAV Fixed-Wing Global A* Pathfinder: Altitude Profile Evaluation", fontsize=15, fontweight="bold")
    fig.savefig(out_png, dpi=200)
    plt.close(fig)
    print(f"Saved plot to {out_png}")


def main():
    print("=" * 95)
    print("FINAL LOW-ALTITUDE EVALUATION & BENCHMARK")
    print("=" * 95)

    cache = load_terrain_cache(CACHE_DIR, load_roi(CONFIG), SOURCE_DEM_PATH)
    terrain = build_terrain_query_from_cache(cache, load_roi(CONFIG), FACTOR)
    profile = load_aircraft_profile(PROFILE_PATH)

    missions_dict = dict(mission_definitions(cache))
    for name, definition in FAR_MISSIONS.items():
        missions_dict[name] = dict(definition)

    clean_missions = {
        "A": {
            "start_rc": missions_dict["A_easy_open"]["start_rc"],
            "goal_rc": missions_dict["A_easy_open"]["goal_rc"],
            "start_z": missions_dict["A_easy_open"]["start_z"],
            "goal_z": missions_dict["A_easy_open"]["goal_z"],
        },
        "B": {
            "start_rc": missions_dict["B_relief_affected"]["start_rc"],
            "goal_rc": missions_dict["B_relief_affected"]["goal_rc"],
            "start_z": missions_dict["B_relief_affected"]["start_z"],
            "goal_z": missions_dict["B_relief_affected"]["goal_z"],
        },
        "C": {
            "start_rc": missions_dict["C_far_south_3km"]["start_rc"],
            "goal_rc": missions_dict["C_far_south_3km"]["goal_rc"],
            "start_z": missions_dict["C_far_south_3km"]["z_msl_m"],
            "goal_z": missions_dict["C_far_south_3km"]["z_msl_m"],
        },
        "D": {
            "start_rc": missions_dict["D_far_east_3km"]["start_rc"],
            "goal_rc": missions_dict["D_far_east_3km"]["goal_rc"],
            "start_z": missions_dict["D_far_east_3km"]["z_msl_m"],
            "goal_z": missions_dict["D_far_east_3km"]["z_msl_m"],
        },
        "E": {
            "start_rc": missions_dict["E_long_descent_9_3km"]["start_rc"],
            "goal_rc": missions_dict["E_long_descent_9_3km"]["goal_rc"],
            "start_z": missions_dict["E_long_descent_9_3km"]["z_msl_m"],
            "goal_z": missions_dict["E_long_descent_9_3km"]["goal_z_msl_m"],
        },
        "F": {
            "start_rc": missions_dict["F_turn_required_diagonal_2_3km"]["start_rc"],
            "goal_rc": missions_dict["F_turn_required_diagonal_2_3km"]["goal_rc"],
            "start_z": missions_dict["F_turn_required_diagonal_2_3km"]["z_msl_m"],
            "goal_z": missions_dict["F_turn_required_diagonal_2_3km"]["z_msl_m"],
        },
    }

    all_data = {"baseline": {}, "proposed": {}}

    print("\n--- RUNNING BASELINE (Weighted A* w=1.01) ---")
    for m_id, m_data in clean_missions.items():
        sx, sy = terrain.rowcol_to_xy(*m_data["start_rc"])
        gx, gy = terrain.rowcol_to_xy(*m_data["goal_rc"])
        heading = navigation_bearing_deg(sx, sy, gx, gy)
        start = PhysicalPose(sx, sy, m_data["start_z"], heading)
        goal = GoalPose(gx, gy, m_data["goal_z"])

        res, met = run_search(start, goal, terrain, profile, mode="baseline")
        all_data["baseline"][m_id] = met
        print(
            f"  Mission {m_id} -> {met['status'].upper():<7} | "
            f"Exp: {met['expansions']:>6} | "
            f"Time: {met['runtime_s']:>5.2f}s | "
            f"Min AGL: {met['min_agl_m']:>5.1f}m | "
            f"Mean AGL: {met['mean_agl_m']:>5.1f}m | "
            f"P90 AGL: {met['p90_agl_m']:>5.1f}m | "
            f"Len: {met['path_length_m']:>7.1f}m"
        )

    print("\n--- RUNNING PROPOSED (Feasible Backward-Envelope Dominance) ---")
    for m_id, m_data in clean_missions.items():
        sx, sy = terrain.rowcol_to_xy(*m_data["start_rc"])
        gx, gy = terrain.rowcol_to_xy(*m_data["goal_rc"])
        heading = navigation_bearing_deg(sx, sy, gx, gy)
        start = PhysicalPose(sx, sy, m_data["start_z"], heading)
        goal = GoalPose(gx, gy, m_data["goal_z"])

        res, met = run_search(start, goal, terrain, profile, mode="proposed", delta_g_m=25.0)
        all_data["proposed"][m_id] = met
        print(
            f"  Mission {m_id} -> {met['status'].upper():<7} | "
            f"Exp: {met['expansions']:>6} | "
            f"Time: {met['runtime_s']:>5.2f}s | "
            f"Min AGL: {met['min_agl_m']:>5.1f}m | "
            f"Mean AGL: {met['mean_agl_m']:>5.1f}m | "
            f"P90 AGL: {met['p90_agl_m']:>5.1f}m | "
            f"Len: {met['path_length_m']:>7.1f}m"
        )

    out_png = ROOT / "results" / "low_altitude_flight_profiles.png"
    out_png.parent.mkdir(parents=True, exist_ok=True)
    plot_altitude_profiles(all_data, out_png)

    # Copy plot to artifact dir
    shutil.copy(out_png, ARTIFACT_DIR / "low_altitude_flight_profiles.png")

    out_json = ROOT / "results" / "final_low_altitude_evaluation.json"
    with open(out_json, "w") as f:
        # Save without massive trajectory points for json brevity
        clean_json = {
            "baseline": {m: {k: v for k, v in dat.items() if k != "trajectory"} for m, dat in all_data["baseline"].items()},
            "proposed": {m: {k: v for k, v in dat.items() if k != "trajectory"} for m, dat in all_data["proposed"].items()},
        }
        json.dump(clean_json, f, indent=2)
    print(f"Saved json metrics to {out_json}")


if __name__ == "__main__":
    main()
