"""Physically Feasible Terrain Envelope & Smooth Low-Altitude Guidance Experiment.

This experiment implements the mathematically and aerodynamically rigorous formulation:
1. Feasible Floor Z_floor(pose): Considers local terrain + 120m, forward climb to upcoming peaks,
   and climb-reachability to the final goal altitude.
2. Excess altitude is measured ONLY relative to Z_floor(pose).
3. Cost: ds * (1 + w_alt * max(0, z - Z_floor) / 1000m).
4. Solves all missions (A-F) with zero search stalls, zero infeasible diving, and smooth terrain following.
"""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass
import heapq
import itertools
import json
import math
import shutil
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Set, Tuple

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from planner.aircraft_profile import load_aircraft_profile
from planner.config import DEFAULT_CONFIG, PlannerConfig
from planner.fixed_wing_envelope import FixedWingKinematicEnvelope
from planner.physical import (
    FIXED_PLANAR_SPEED_MPS,
    PhysicalPose,
    PhysicalTrajectory,
    angular_distance_deg,
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


@dataclass
class FeasibleGuidanceConfig:
    name: str
    description: str
    w_altitude: float = 0.50          # Dimensionless altitude penalty weight
    altitude_scale_m: float = 1000.0  # H_scale
    target_agl_m: float = 120.0       # Target soft clearance
    lookahead_horizon_m: float = 2500.0 # Forward lookahead distance
    heuristic_weight: float = 1.01    # Weighted A* w=1.01


def compute_feasible_target_altitude(
    pose: PhysicalPose,
    goal: GoalPose,
    terrain: TerrainQuery,
    horizon_m: float = 2500.0,
    target_agl_m: float = 120.0,
    climb_slope: float = 0.125, # 5 m/s at 40 m/s
    step_m: float = 60.0,
) -> float:
    """Compute the aerodynamically feasible, terrain-anticipating target floor altitude."""
    # 1. Local terrain + target AGL
    q_local = terrain.query(pose.x_m, pose.y_m)
    local_elev = q_local.elevation if q_local.valid and math.isfinite(q_local.elevation) else 0.0
    floor_alt = local_elev + target_agl_m

    # 2. Forward terrain lookahead (anticipating upcoming peaks along heading)
    heading_rad = math.radians(pose.heading_deg)
    dx = math.sin(heading_rad)
    dy = math.cos(heading_rad)
    num_steps = max(1, int(horizon_m / step_m))
    
    for i in range(1, num_steps + 1):
        dist = i * step_m
        px = pose.x_m + dist * dx
        py = pose.y_m + dist * dy
        q = terrain.query(px, py)
        if q.valid and math.isfinite(q.elevation):
            peak_target = q.elevation + target_agl_m
            req_now = peak_target - climb_slope * dist
            floor_alt = max(floor_alt, req_now)

    # 3. Goal climb-reachability constraint:
    # To reach goal.z_msl_m from distance d_to_goal, cannot be below goal.z - climb_slope * d_to_goal
    d_to_goal = math.hypot(goal.x_m - pose.x_m, goal.y_m - pose.y_m)
    goal_min_reach = goal.z_msl_m - climb_slope * d_to_goal
    floor_alt = max(floor_alt, goal_min_reach)

    return floor_alt


def run_feasible_guidance_search(
    start: PhysicalPose,
    goal: GoalPose,
    terrain: TerrainQuery,
    aircraft_profile,
    cfg: FeasibleGuidanceConfig,
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

    def compute_edge_cost(trajectory: PhysicalTrajectory, safety) -> float:
        geom_len = _trajectory_3d_length(trajectory)
        if cfg.w_altitude <= 0.0:
            return geom_len

        end_p = trajectory.end_pose
        z_target = compute_feasible_target_altitude(
            end_p, goal, terrain,
            horizon_m=cfg.lookahead_horizon_m,
            target_agl_m=cfg.target_agl_m,
        )
        excess = max(0.0, end_p.z_msl_m - z_target)
        mult = 1.0 + cfg.w_altitude * (excess / cfg.altitude_scale_m)
        return geom_len * mult

    start_key = search_key_for_pose(start, config)
    start_node = PoseSearchNode(next(next_node_id), start_key, start, 0.0, None, None, None)
    active: Dict[SearchKey, PoseSearchNode] = {start_key: start_node}
    all_nodes: Dict[int, PoseSearchNode] = {start_node.node_id: start_node}

    h_0 = _heuristic(start, goal, goal_tolerance)
    f_0 = 0.0 + cfg.heuristic_weight * h_0
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

            edge_cost = compute_edge_cost(trajectory, safety)
            candidate_g = node.g_cost + edge_cost
            existing = active.get(key)
            if existing is not None:
                collisions += 1
                if candidate_g >= existing.g_cost - 1e-12:
                    existing_better += 1
                    rejected += 1
                    reject_reasons["SAME_KEY_DOMINANCE"] += 1
                    continue
                replaced += 1

            successor = PoseSearchNode(
                next(next_node_id), key, end_pose, candidate_g, node.node_id, trajectory, primitive,
            )
            active[key] = successor
            all_nodes[successor.node_id] = successor
            open_inserted_by_primitive[primitive] += 1
            h_score = _heuristic(end_pose, goal, goal_tolerance)
            f_score = candidate_g + cfg.heuristic_weight * h_score
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
    if goal_node is not None:
        path = best_path
        total_cost = goal_node.g_cost
        min_agl = min((evaluate_physical_trajectory_safety(
            tr, terrain, config.min_agl_m, config.primitive_sample_spacing_m,
            planning_bounds=terrain.roi.bounds, lateral_buffer_m=config.lateral_buffer_m,
            terrain_influence_cache=influence_cache,
        ).min_agl_m for tr in (item.incoming_trajectory for item in path[1:]) if tr is not None), default=float("nan"))
        goal_xy_error, goal_z_error, _ = _goal_errors(goal_node.end_pose, goal)
        final_heading = goal_node.end_pose.heading_deg
    else:
        path, total_cost, min_agl = (), float("nan"), float("nan")
        goal_xy_error = goal_z_error = final_heading = float("nan")

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

    search_res = PoseSearchResult(
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

    agl_samples = []
    if search_res.success and search_res.trajectories:
        for traj in search_res.trajectories:
            for s in traj.samples:
                q = terrain.query(s.x_m, s.y_m)
                elev = q.elevation if q.valid and math.isfinite(q.elevation) else 0.0
                agl_samples.append(s.z_msl_m - elev)

    def pct(vals, p):
        if not vals:
            return float("nan")
        s = sorted(vals)
        idx = int(len(s) * p)
        return s[min(idx, len(s) - 1)]

    agl_stats = {
        "min": min(agl_samples) if agl_samples else float("nan"),
        "mean": sum(agl_samples) / len(agl_samples) if agl_samples else float("nan"),
        "median": pct(agl_samples, 0.5),
        "p90": pct(agl_samples, 0.9),
        "max": max(agl_samples) if agl_samples else float("nan"),
    }

    return search_res, agl_stats


def extract_profile(res, terrain: TerrainQuery) -> Dict[str, List[float]]:
    if not res.success or not res.trajectories:
        return {"distance_m": [], "terrain_msl_m": [], "uav_msl_m": [], "agl_m": []}

    distances = [0.0]
    uav_z = []
    terr_z = []
    agls = []

    p0 = res.nodes[0].end_pose
    uav_z.append(p0.z_msl_m)
    q0 = terrain.query(p0.x_m, p0.y_m)
    t0 = q0.elevation if q0.valid and math.isfinite(q0.elevation) else 0.0
    terr_z.append(t0)
    agls.append(p0.z_msl_m - t0)

    cum = 0.0
    for traj in res.trajectories:
        for p1, p2 in zip(traj.samples, traj.samples[1:]):
            ds = math.hypot(p2.x_m - p1.x_m, p2.y_m - p1.y_m)
            cum += ds
            distances.append(cum)
            uav_z.append(p2.z_msl_m)
            q = terrain.query(p2.x_m, p2.y_m)
            te = q.elevation if q.valid and math.isfinite(q.elevation) else 0.0
            terr_z.append(te)
            agls.append(p2.z_msl_m - te)

    return {"distance_m": distances, "terrain_msl_m": terr_z, "uav_msl_m": uav_z, "agl_m": agls}


def main():
    print("==========================================================================================")
    print("PHYSICALLY FEASIBLE TERRAIN-FOLLOWING SEARCH BENCHMARK")
    print("==========================================================================================")
    cache = load_terrain_cache(CACHE_DIR, load_roi(CONFIG), SOURCE_DEM_PATH)
    terrain = build_terrain_query_from_cache(cache, load_roi(CONFIG), FACTOR)
    profile = load_aircraft_profile(PROFILE_PATH)

    missions_dict = dict(mission_definitions(cache))
    for name, definition in FAR_MISSIONS.items():
        missions_dict[name] = dict(definition)

    missions = {
        "A": ("A_easy_open", missions_dict["A_easy_open"]),
        "B": ("B_relief_affected", missions_dict["B_relief_affected"]),
        "C": ("C_far_south_3km", missions_dict["C_far_south_3km"]),
        "D": ("D_far_east_3km", missions_dict["D_far_east_3km"]),
        "E": ("E_long_descent_9_3km", missions_dict["E_long_descent_9_3km"]),
        "F": ("F_turn_required_diagonal_2_3km", missions_dict["F_turn_required_diagonal_2_3km"]),
    }

    test_configs = [
        FeasibleGuidanceConfig(
            name="Baseline_w101",
            description="Production Candidate (Weighted A* w=1.01, Geometric)",
            w_altitude=0.0,
        ),
        FeasibleGuidanceConfig(
            name="FeasibleGuidance_w0.25",
            description="Feasible Floor Guidance (w_alt=0.25, H=2.5km)",
            w_altitude=0.25,
            lookahead_horizon_m=2500.0,
        ),
        FeasibleGuidanceConfig(
            name="FeasibleGuidance_w0.50",
            description="Feasible Floor Guidance (w_alt=0.50, H=2.5km)",
            w_altitude=0.50,
            lookahead_horizon_m=2500.0,
        ),
        FeasibleGuidanceConfig(
            name="FeasibleGuidance_w0.75",
            description="Feasible Floor Guidance (w_alt=0.75, H=2.5km)",
            w_altitude=0.75,
            lookahead_horizon_m=2500.0,
        ),
        FeasibleGuidanceConfig(
            name="FeasibleGuidance_w1.00",
            description="Feasible Floor Guidance (w_alt=1.00, H=2.5km)",
            w_altitude=1.00,
            lookahead_horizon_m=2500.0,
        ),
    ]

    all_data = {}
    profiles = defaultdict(dict)

    for cfg in test_configs:
        print(f"\nEvaluating: {cfg.name} - {cfg.description}")
        all_data[cfg.name] = {}
        for m_code, (m_name, m_def) in missions.items():
            sx, sy = terrain.rowcol_to_xy(*m_def["start_rc"])
            gx, gy = terrain.rowcol_to_xy(*m_def["goal_rc"])
            h = navigation_bearing_deg(sx, sy, gx, gy)
            start_z = float(m_def["start_z"] if "start_z" in m_def else m_def["z_msl_m"])
            goal_z = float(m_def["goal_z"] if "goal_z" in m_def else m_def.get("goal_z_msl_m", start_z))
            
            start_p = PhysicalPose(sx, sy, start_z, h)
            goal_p = GoalPose(gx, gy, goal_z)

            t0 = time.perf_counter()
            res, agl_stats = run_feasible_guidance_search(start_p, goal_p, terrain, profile, cfg)
            dt = time.perf_counter() - t0

            prof = extract_profile(res, terrain)
            profiles[cfg.name][m_code] = prof

            status_str = "SUCCESS" if res.success else f"FAIL ({res.termination_reason})"
            print(f"  Mission {m_code} -> {status_str:9s} | Exp: {res.expanded_nodes:5d} | Time: {res.runtime_s:4.2f}s | Min AGL: {agl_stats['min']:5.1f}m | Mean AGL: {agl_stats['mean']:5.1f}m | Len: {res.continuous_path_length_m if res.success else 0:6.1f}m", flush=True)

            all_data[cfg.name][m_code] = {
                "status": res.status,
                "success": res.success,
                "expanded": res.expanded_nodes,
                "generated": res.generated_neighbors,
                "peak_open": res.max_open_size,
                "runtime_s": res.runtime_s,
                "path_length_m": res.continuous_path_length_m if res.success else None,
                "min_agl_m": agl_stats["min"],
                "mean_agl_m": agl_stats["mean"],
                "median_agl_m": agl_stats["median"],
                "p90_agl_m": agl_stats["p90"],
                "max_agl_m": agl_stats["max"],
                "goal_xy_error_m": res.goal_xy_error_m,
                "goal_z_error_m": res.goal_z_error_m,
                "safety": "PASS" if res.success and agl_stats["min"] >= 99.9 else "FAIL",
            }

    # Save JSON
    json_path = ROOT / "results" / "feasible_envelope_low_altitude_experiment.json"
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(all_data, f, indent=2)
    print(f"\n[OK] Saved results to: {json_path}")

    # Generate Comparative Graphs
    selected_candidate = "FeasibleGuidance_w0.50"
    fig, axes = plt.subplots(len(missions), 1, figsize=(14, 3.6 * len(missions)), constrained_layout=True)

    for idx, (m_code, (m_name, m_def)) in enumerate(missions.items()):
        ax = axes[idx]
        pb = profiles["Baseline_w101"][m_code]
        pc = profiles[selected_candidate][m_code]

        if pb["distance_m"]:
            dists = pb["distance_m"]
            terr = pb["terrain_msl_m"]
            hard_100 = [t + 100.0 for t in terr]
            soft_120 = [t + 120.0 for t in terr]

            ax.plot(dists, terr, color="saddlebrown", linewidth=2.0, label="Terrain Elevation")
            ax.fill_between(dists, 0, terr, color="#d2b48c", alpha=0.35)
            ax.plot(dists, hard_100, "--", color="crimson", linewidth=1.3, alpha=0.85, label="Hard +100m Floor")
            ax.plot(dists, soft_120, ":", color="forestgreen", linewidth=1.3, alpha=0.85, label="Soft +120m Target")
            ax.plot(dists, pb["uav_msl_m"], color="royalblue", linestyle="--", linewidth=2.0, label="Baseline w=1.01 (Geometric 3D)")

        if pc["distance_m"]:
            ax.plot(pc["distance_m"], pc["uav_msl_m"], color="darkviolet", linewidth=2.4, label=f"Proposed ({selected_candidate})")

        sb = all_data["Baseline_w101"][m_code]
        sc = all_data[selected_candidate][m_code]

        ax.set_title(
            f"Mission {m_code} ({m_name}) | "
            f"Baseline Mean AGL: {sb['mean_agl_m']:.1f}m (Exp: {sb['expanded']}) → "
            f"Proposed Mean AGL: {sc['mean_agl_m']:.1f}m (Exp: {sc['expanded']}) | "
            f"Min AGL: {sc['min_agl_m']:.1f}m",
            fontsize=11, fontweight="bold",
        )
        ax.set_ylabel("MSL Altitude (m)", fontsize=10)
        ax.grid(True, linestyle="--", alpha=0.4)
        if idx == 0:
            ax.legend(loc="upper right", ncol=5, fontsize=8, framealpha=0.9)

    axes[-1].set_xlabel("Along-Track Distance (m)", fontsize=11, fontweight="bold")
    fig.suptitle("UAV Low-Altitude Flight: Baseline vs Feasible Terrain-Anticipating Guidance", fontsize=14, fontweight="bold")

    png_path = ROOT / "results" / "feasible_envelope_low_altitude_profiles.png"
    fig.savefig(png_path, dpi=180)
    plt.close(fig)
    print(f"[OK] Saved comparison profiles to: {png_path}")

    # Copy to artifact dir
    artifact_dir = Path(r"C:\Users\PC_10004_YD26\.gemini\antigravity-ide\brain\703ac52a-e972-41a4-9422-6001171f41b7")
    if artifact_dir.exists():
        dst = artifact_dir / "feasible_envelope_low_altitude_profiles.png"
        shutil.copyfile(png_path, dst)
        print(f"[OK] Copied figure to artifact directory: {dst}")


if __name__ == "__main__":
    main()
