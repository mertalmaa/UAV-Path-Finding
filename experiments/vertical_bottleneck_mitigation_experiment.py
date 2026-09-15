"""Comprehensive evaluation of vertical bottleneck mitigations for fixed-wing A*.

Evaluates:
1. Baseline Production A*
2. Tie-Breaking & F-Band Ordering (h_min, g_max, f_band 10m/25m/50m)
3. Pareto Z-Dominance within (x_bin, y_bin, heading_bin)
4. Max-Z Representatives per (x_bin, y_bin, heading_bin) (K=1, 2, 3, 5)
5. Bidirectional Guidance Corridor Gating (Margin=50m, 100m, 150m, 200m)
6. Sub-optimality Bounded Weighted Heuristic (w=1.01, 1.05, 1.10)
7. Cross-validation across ALL benchmark missions (A, B, C, D, E, F)
"""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass
import heapq
import itertools
import json
import math
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
    _trajectory_edge_cost,
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
class VariantConfig:
    name: str
    category: str
    description: str
    tie_breaker: str = "fifo"       # 'fifo', 'h_min', 'g_max', 'band_h_min'
    f_band_eps: float = 0.0         # band size in meters
    heuristic_weight: float = 1.0   # w in f = g + w * h
    z_dominance: str = "none"       # 'none', 'pareto', 'max_k'
    max_k_z: int = 1000             # max Z buckets per (x, y, heading)
    guidance_mode: str = "none"     # 'none', 'corridor'
    guidance_margin_m: float = 150.0


def _build_1d_corridor(
    start: PhysicalPose,
    goal: GoalPose,
    terrain: TerrainQuery,
    min_agl_m: float = 100.0,
    sample_spacing_m: float = 60.0,
) -> Tuple[List[float], List[float], List[float], List[float]]:
    dx = goal.x_m - start.x_m
    dy = goal.y_m - start.y_m
    total_dist = math.hypot(dx, dy)
    num_samples = max(2, int(math.ceil(total_dist / sample_spacing_m)) + 1)
    
    distances = [i * total_dist / (num_samples - 1) for i in range(num_samples)]
    elevations = []
    for d in distances:
        t = d / total_dist
        x = start.x_m + t * dx
        y = start.y_m + t * dy
        q = terrain.query(x, y)
        elev = q.elevation if q.valid and math.isfinite(q.elevation) else 0.0
        elevations.append(elev)
    
    # Minimum safe terrain floor
    z_lower = [elev + min_agl_m for elev in elevations]
    z_lower[0] = max(z_lower[0], start.z_msl_m - 50.0)
    z_lower[-1] = max(z_lower[-1], goal.z_msl_m - 10.0)

    
    # Max climb/descent rate constraints (5 m/s at 40 m/s planar = 0.125 slope)
    max_slope = 5.0 / FIXED_PLANAR_SPEED_MPS
    
    # Backward climb pass (terrain avoidance requirement)
    for _ in range(5):
        for i in range(len(z_lower) - 2, -1, -1):
            ds = distances[i+1] - distances[i]
            z_lower[i] = max(z_lower[i], z_lower[i+1] - max_slope * ds)
        for i in range(len(z_lower) - 1):
            ds = distances[i+1] - distances[i]
            z_lower[i+1] = max(z_lower[i+1], z_lower[i] - max_slope * ds)

    # Upper envelope: start_z, descending towards goal
    z_upper = [start.z_msl_m + 50.0 for _ in distances]
    for i in range(len(distances)):
        dist_to_goal = total_dist - distances[i]
        max_reach_goal = goal.z_msl_m + max_slope * dist_to_goal
        z_upper[i] = min(z_upper[i], max_reach_goal + 100.0)
        z_upper[i] = max(z_upper[i], z_lower[i] + 50.0)
        
    return distances, elevations, z_lower, z_upper


def run_variant_search(
    start: PhysicalPose,
    goal: GoalPose,
    terrain: TerrainQuery,
    aircraft_profile,
    variant: VariantConfig,
    goal_tolerance: GoalTolerance = GOAL_TOLERANCE,
    config: PlannerConfig = CONFIG,
    max_expansions: int = 30000,
    max_search_time_s: float = 60.0,
) -> PoseSearchResult:
    envelope = FixedWingKinematicEnvelope()
    influence_cache = TerrainInfluenceCache(terrain)
    started = time.perf_counter()
    counter = itertools.count()
    next_node_id = itertools.count()

    # Setup Corridor Guidance
    if variant.guidance_mode == "corridor":
        g_dists, _, g_z_min, g_z_max = _build_1d_corridor(start, goal, terrain, config.min_agl_m)
        total_baseline = g_dists[-1]
        
        def check_corridor(pose: PhysicalPose) -> bool:
            dx = goal.x_m - start.x_m
            dy = goal.y_m - start.y_m
            L2 = dx*dx + dy*dy
            if L2 == 0:
                return True
            t = ((pose.x_m - start.x_m)*dx + (pose.y_m - start.y_m)*dy) / L2
            d_proj = max(0.0, min(total_baseline, t * total_baseline))
            idx = int(d_proj / 60.0)
            if idx >= len(g_dists) - 1:
                z_min, z_max = g_z_min[-1], g_z_max[-1]
            else:
                alpha = (d_proj - g_dists[idx]) / (g_dists[idx+1] - g_dists[idx])
                z_min = (1 - alpha) * g_z_min[idx] + alpha * g_z_min[idx+1]
                z_max = (1 - alpha) * g_z_max[idx] + alpha * g_z_max[idx+1]
            return (z_min - variant.guidance_margin_m) <= pose.z_msl_m <= (z_max + variant.guidance_margin_m)
    else:
        check_corridor = lambda p: True

    start_key = search_key_for_pose(start, config)
    start_node = PoseSearchNode(next(next_node_id), start_key, start, 0.0, None, None, None)
    active: Dict[SearchKey, PoseSearchNode] = {start_key: start_node}
    all_nodes: Dict[int, PoseSearchNode] = {start_node.node_id: start_node}

    # Fast Pareto Tracking per (x_bin, y_bin, heading_bin)
    # Map (x, y, heading) -> list of (g_cost, z_dist, search_key)
    pareto_frontier: Dict[Tuple[int, int, int], List[Tuple[float, float, SearchKey]]] = defaultdict(list)
    start_xyh = (start_key.x_bin, start_key.y_bin, start_key.heading_bin)
    pareto_frontier[start_xyh].append((0.0, abs(start.z_msl_m - goal.z_msl_m), start_key))

    # Queue item construction
    def make_queue_item(f: float, g: float, h: float, pose: PhysicalPose, nid: int):
        c = next(counter)
        tb = variant.tie_breaker
        if tb == "fifo":
            return (f, c, nid)
        elif tb == "h_min":
            return (f, h, c, nid)
        elif tb == "g_max":
            return (f, -g, c, nid)
        elif tb == "band_h_min":
            band = round(f / variant.f_band_eps) if variant.f_band_eps > 0 else f
            return (band, h, f, c, nid)
        return (f, c, nid)

    h_0 = _heuristic(start, goal, goal_tolerance)
    f_0 = 0.0 + variant.heuristic_weight * h_0
    open_heap = [make_queue_item(f_0, 0.0, h_0, start, start_node.node_id)]

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

            if variant.guidance_mode != "none" and not check_corridor(trajectory.end_pose):
                rejected += 1
                reject_reasons["GUIDANCE_CORRIDOR_EXCEEDED"] += 1
                continue

            safety = evaluate_physical_trajectory_safety(
                trajectory, terrain, config.min_agl_m, config.primitive_sample_spacing_m,
                planning_bounds=terrain.roi.bounds, lateral_buffer_m=config.lateral_buffer_m,
                terrain_influence_cache=influence_cache,
                record_sample_terrain=config.enable_low_altitude_cost,
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

            candidate_g = node.g_cost + _trajectory_edge_cost(trajectory, safety, config)
            existing = active.get(key)
            if existing is not None:
                collisions += 1
                if candidate_g >= existing.g_cost - 1e-12:
                    existing_better += 1
                    rejected += 1
                    reject_reasons["SAME_KEY_DOMINANCE"] += 1
                    continue
                replaced += 1

            xyh = (key.x_bin, key.y_bin, key.heading_bin)
            cand_z_dist = abs(end_pose.z_msl_m - goal.z_msl_m)

            if variant.z_dominance == "pareto":
                frontier = pareto_frontier[xyh]
                dominated = False
                for f_g, f_z_dist, _ in frontier:
                    if f_g <= candidate_g + 1e-9 and f_z_dist <= cand_z_dist + 1e-9:
                        dominated = True
                        break
                if dominated:
                    rejected += 1
                    reject_reasons["PARETO_Z_DOMINANCE"] += 1
                    continue
                # Update frontier: filter out any points dominated by candidate
                pareto_frontier[xyh] = [
                    (fg, fz, fk) for fg, fz, fk in frontier
                    if not (candidate_g <= fg + 1e-9 and cand_z_dist <= fz + 1e-9)
                ]
                pareto_frontier[xyh].append((candidate_g, cand_z_dist, key))

            elif variant.z_dominance == "max_k":
                frontier = pareto_frontier[xyh]
                if len(frontier) >= variant.max_k_z:
                    # If candidate has higher g than all existing in this XYH
                    if all(candidate_g >= fg for fg, _, _ in frontier):
                        rejected += 1
                        reject_reasons["MAX_K_Z_EXCEEDED"] += 1
                        continue
                pareto_frontier[xyh].append((candidate_g, cand_z_dist, key))

            successor = PoseSearchNode(
                next(next_node_id), key, end_pose, candidate_g, node.node_id, trajectory, primitive,
            )
            active[key] = successor
            all_nodes[successor.node_id] = successor
            open_inserted_by_primitive[primitive] += 1
            h_score = _heuristic(end_pose, goal, goal_tolerance)
            f_score = candidate_g + variant.heuristic_weight * h_score
            item = make_queue_item(f_score, candidate_g, h_score, end_pose, successor.node_id)
            heapq.heappush(open_heap, item)

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

    return PoseSearchResult(
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


def generate_plots(results_e: dict, cross_validation: dict, output_path: Path):
    """Generate publication-quality comparative figures."""
    fig, axes = plt.subplots(2, 2, figsize=(16, 12), constrained_layout=True)

    # 1. Mission E: Expansions across variants
    names = list(results_e.keys())
    expansions = [results_e[k]["expanded"] for k in names]
    successes = [results_e[k]["success"] for k in names]
    colors = ["#2ca02c" if s else "#d62728" for s in successes]
    
    short_names = [n.replace("V_", "").replace("_", "\n") for n in names]
    axes[0, 0].bar(range(len(names)), expansions, color=colors, alpha=0.85, edgecolor="black")
    axes[0, 0].set_xticks(range(len(names)))
    axes[0, 0].set_xticklabels(short_names, rotation=45, ha="right", fontsize=8)
    axes[0, 0].set_ylabel("Expanded Nodes")
    axes[0, 0].set_title("Mission E: Node Expansions (Green=Success, Red=Limit 30k)")
    axes[0, 0].grid(axis="y", linestyle="--", alpha=0.6)
    axes[0, 0].axhline(30000, color="red", linestyle=":", linewidth=1.5, label="Budget Limit (30k)")
    axes[0, 0].legend()

    # 2. Mission E: Runtime vs Z-Diversity (Mean Z-bins per XY)
    runtimes = [results_e[k]["runtime_s"] for k in names]
    z_divs = [results_e[k]["mean_z_bins_per_xy"] for k in names]
    
    for i, name in enumerate(names):
        marker = "o" if successes[i] else "X"
        c = colors[i]
        axes[0, 1].scatter(z_divs[i], runtimes[i], color=c, marker=marker, s=120, edgecolor="black", zorder=3)
        axes[0, 1].annotate(name.split("_")[1], (z_divs[i], runtimes[i]), fontsize=8,
                            xytext=(5, 5), textcoords="offset points")
    axes[0, 1].set_xlabel("Mean Z-Bins per XY Coordinate (Z-Diversity)")
    axes[0, 1].set_ylabel("Search Runtime (s)")
    axes[0, 1].set_title("Mission E: Runtime vs Vertical State Redundancy")
    axes[0, 1].grid(True, linestyle="--", alpha=0.6)

    # 3. Cross-Validation: Expansions across Missions A-F for Top Variants
    top_variants = ["Baseline", "Weighted_1.01", "Pareto_ZDom", "Guidance_Corridor_150m", "Safe_Optimum"]
    missions = ["A", "B", "C", "D", "E", "F"]
    x = range(len(missions))
    width = 0.15
    
    for idx, var_name in enumerate(top_variants):
        if var_name in cross_validation:
            var_exp = [cross_validation[var_name][m]["expanded"] for m in missions]
            axes[1, 0].bar([p + idx * width for p in x], var_exp, width=width, label=var_name)
            
    axes[1, 0].set_xticks([p + width * 2.0 for p in x])


    axes[1, 0].set_xticklabels([f"Mission {m}" for m in missions])
    axes[1, 0].set_ylabel("Expanded Nodes (Log Scale)")
    axes[1, 0].set_yscale("log")
    axes[1, 0].set_title("Cross-Validation (A-F): Expansion Count Comparison")
    axes[1, 0].legend(fontsize=8)
    axes[1, 0].grid(axis="y", linestyle="--", alpha=0.6)

    # 4. Cross-Validation: Solution Path Length (m) Comparison
    for idx, var_name in enumerate(top_variants):
        if var_name in cross_validation:
            lengths = [cross_validation[var_name][m]["path_length_m"] or 0 for m in missions]
            axes[1, 1].plot(missions, lengths, marker="o", linewidth=1.8, label=var_name)
    axes[1, 1].set_xlabel("Mission")
    axes[1, 1].set_ylabel("Path Length (m)")
    axes[1, 1].set_title("Cross-Validation (A-F): Solution Path Optimality")
    axes[1, 1].legend(fontsize=8)
    axes[1, 1].grid(True, linestyle="--", alpha=0.6)

    fig.suptitle("UAV Fixed-Wing A* Pathfinder: Vertical State Explosion Mitigation Study", fontsize=15, fontweight="bold")
    fig.savefig(output_path, dpi=200)
    plt.close(fig)
    print(f"Generated comparative graph: {output_path}")


def main():
    print("Loading terrain cache and aircraft profile...")
    cache = load_terrain_cache(CACHE_DIR, load_roi(CONFIG), SOURCE_DEM_PATH)
    terrain = build_terrain_query_from_cache(cache, load_roi(CONFIG), FACTOR)
    profile = load_aircraft_profile(PROFILE_PATH)

    # All missions
    missions_dict = dict(mission_definitions(cache))
    for name, definition in FAR_MISSIONS.items():
        missions_dict[name] = dict(definition)

    # Clean standardized mission definitions
    clean_missions = {
        "A": missions_dict["A_easy_open"],
        "B": missions_dict["B_relief_affected"],
        "C": missions_dict["C_far_south_3km"],
        "D": missions_dict["D_far_east_3km"],
        "E": missions_dict["E_long_descent_9_3km"],
        "F": missions_dict["F_turn_required_diagonal_2_3km"],
    }

    # Define Candidate Variants to rigorously test
    candidate_variants = [
        VariantConfig(name="V_Baseline", category="Baseline", description="Production baseline A* (Euclidean, FIFO)"),
        VariantConfig(name="V_TieBreak_Hmin", category="Tie-Break", description="Tie-break: Primary f, Secondary h_min", tie_breaker="h_min"),
        VariantConfig(name="V_TieBreak_Gmax", category="Tie-Break", description="Tie-break: Primary f, Secondary g_max", tie_breaker="g_max"),
        VariantConfig(name="V_FBand_Hmin_10m", category="F-Band", description="F-Band (10m) + h_min secondary", tie_breaker="band_h_min", f_band_eps=10.0),
        VariantConfig(name="V_FBand_Hmin_25m", category="F-Band", description="F-Band (25m) + h_min secondary", tie_breaker="band_h_min", f_band_eps=25.0),
        VariantConfig(name="V_FBand_Hmin_50m", category="F-Band", description="F-Band (50m) + h_min secondary", tie_breaker="band_h_min", f_band_eps=50.0),
        VariantConfig(name="V_Weighted_1.01", category="Weighted-A*", description="Sub-optimal bounded A* (w=1.01)", heuristic_weight=1.01),
        VariantConfig(name="V_Weighted_1.05", category="Weighted-A*", description="Sub-optimal bounded A* (w=1.05)", heuristic_weight=1.05),
        VariantConfig(name="V_Weighted_1.10", category="Weighted-A*", description="Sub-optimal bounded A* (w=1.10)", heuristic_weight=1.10),
        VariantConfig(name="V_Pareto_ZDom", category="Z-Dominance", description="Pareto Z-Dominance in (X,Y,Heading)", z_dominance="pareto"),
        VariantConfig(name="V_MaxK_Z_1", category="Z-Dominance", description="Max 1 Z-rep per (X,Y,Heading)", z_dominance="max_k", max_k_z=1),
        VariantConfig(name="V_MaxK_Z_3", category="Z-Dominance", description="Max 3 Z-reps per (X,Y,Heading)", z_dominance="max_k", max_k_z=3),
        VariantConfig(name="V_Guidance_50m", category="Corridor-Guidance", description="Bidirectional Corridor (margin +/-50m)", guidance_mode="corridor", guidance_margin_m=50.0),
        VariantConfig(name="V_Guidance_100m", category="Corridor-Guidance", description="Bidirectional Corridor (margin +/-100m)", guidance_mode="corridor", guidance_margin_m=100.0),
        VariantConfig(name="V_Guidance_150m", category="Corridor-Guidance", description="Bidirectional Corridor (margin +/-150m)", guidance_mode="corridor", guidance_margin_m=150.0),
        VariantConfig(name="V_Guidance_200m", category="Corridor-Guidance", description="Bidirectional Corridor (margin +/-200m)", guidance_mode="corridor", guidance_margin_m=200.0),
        VariantConfig(name="V_Combined_Optimum", category="Combined", description="Combined: F-Band (10m) + Pareto Z-Dom + Corridor (150m)",
                      tie_breaker="band_h_min", f_band_eps=10.0, z_dominance="pareto", guidance_mode="corridor", guidance_margin_m=150.0),
        VariantConfig(name="V_Safe_Optimum", category="Safe-Optimum", description="Safe Optimum: Weighted (w=1.01) + Pareto Z-Dominance (Zero Corridor Risk)",
                      heuristic_weight=1.01, z_dominance="pareto", guidance_mode="none"),
    ]

    print("==========================================================================================")
    print("PHASE 1: BENCHMARKING MISSION E (9.3 km DESCENT)")
    print("==========================================================================================")

    e_def = clean_missions["E"]
    sx, sy = terrain.rowcol_to_xy(*e_def["start_rc"])
    gx, gy = terrain.rowcol_to_xy(*e_def["goal_rc"])
    h_e = navigation_bearing_deg(sx, sy, gx, gy)
    start_e = PhysicalPose(sx, sy, e_def["z_msl_m"], h_e)
    goal_e = GoalPose(gx, gy, e_def.get("goal_z_msl_m", e_def["z_msl_m"]))

    results_e = {}
    for var in candidate_variants:
        print(f"Running Variant {var.name:25s} ({var.description})...", end="", flush=True)
        t0 = time.perf_counter()
        res = run_variant_search(start_e, goal_e, terrain, profile, var, max_expansions=30000, max_search_time_s=60.0)
        dt = time.perf_counter() - t0
        status_str = "SUCCESS" if res.success else f"FAIL ({res.termination_reason})"
        print(f" -> {status_str:12s} | Exp: {res.expanded_nodes:6d} | Time: {res.runtime_s:5.2f}s | Mean Z/XY: {res.mean_z_bins_per_xy:4.1f} | Len: {res.continuous_path_length_m if res.success else 0:7.1f}m", flush=True)
        results_e[var.name] = {
            "variant": dataclasses.asdict(var),
            "status": res.status,
            "success": res.success,
            "expanded": res.expanded_nodes,
            "generated": res.generated_neighbors,
            "rejected": res.rejected_neighbors,
            "peak_open": res.max_open_size,
            "runtime_s": res.runtime_s,
            "min_agl_m": res.minimum_agl_m,
            "path_length_m": res.continuous_path_length_m if res.success else None,
            "segments": len(res.trajectories) if res.success else 0,
            "goal_xy_error_m": res.goal_xy_error_m,
            "goal_z_error_m": res.goal_z_error_m,
            "unique_xy_bins": res.unique_xy_bins,
            "mean_z_bins_per_xy": res.mean_z_bins_per_xy,
            "max_z_bins_per_xy": res.max_z_bins_per_xy,
            "reject_reasons": res.rejected_reason_counts,
        }

    print("\n==========================================================================================")
    print("PHASE 2: CROSS-VALIDATION ON ALL MISSIONS (A, B, C, D, E, F)")
    print("==========================================================================================")

    # Select representative architectures for full benchmark
    eval_architectures = {
        "Baseline": VariantConfig(name="Baseline", category="Baseline", description="Production Baseline"),
        "Weighted_1.01": VariantConfig(name="Weighted_1.01", category="Weighted-A*", description="Weighted A* w=1.01", heuristic_weight=1.01),
        "Pareto_ZDom": VariantConfig(name="Pareto_ZDom", category="Z-Dominance", description="Pareto Z-Dominance", z_dominance="pareto"),
        "Guidance_Corridor_150m": VariantConfig(name="Guidance_Corridor_150m", category="Corridor-Guidance", description="Guidance Corridor 150m", guidance_mode="corridor", guidance_margin_m=150.0),
        "Safe_Optimum": VariantConfig(name="Safe_Optimum", category="Safe-Optimum", description="Weighted (1.01) + Pareto Z-Dom (Zero Risk)",
                                      heuristic_weight=1.01, z_dominance="pareto", guidance_mode="none"),
    }



    cross_val_results = {arch_name: {} for arch_name in eval_architectures}

    for m_code, m_def in clean_missions.items():
        sx, sy = terrain.rowcol_to_xy(*m_def["start_rc"])
        gx, gy = terrain.rowcol_to_xy(*m_def["goal_rc"])
        h = navigation_bearing_deg(sx, sy, gx, gy)
        start_z = float(m_def["start_z"] if "start_z" in m_def else m_def["z_msl_m"])
        goal_z = float(m_def["goal_z"] if "goal_z" in m_def else m_def.get("goal_z_msl_m", start_z))
        start_p = PhysicalPose(sx, sy, start_z, h)
        goal_p = GoalPose(gx, gy, goal_z)

        print(f"\nEvaluating Mission {m_code} (Sep: {math.hypot(gx-sx, gy-sy)/1000:.1f}km, Z: {start_z:.0f}m -> {goal_z:.0f}m):")
        for arch_name, var in eval_architectures.items():
            res = run_variant_search(start_p, goal_p, terrain, profile, var, max_expansions=30000, max_search_time_s=60.0)
            status_str = "SUCCESS" if res.success else f"FAIL ({res.termination_reason})"
            print(f"  {arch_name:24s} -> {status_str:10s} | Exp: {res.expanded_nodes:6d} | Time: {res.runtime_s:5.2f}s | Min AGL: {res.minimum_agl_m:5.1f}m | Len: {res.continuous_path_length_m if res.success else 0:7.1f}m", flush=True)
            cross_val_results[arch_name][m_code] = {
                "status": res.status,
                "success": res.success,
                "expanded": res.expanded_nodes,
                "generated": res.generated_neighbors,
                "rejected": res.rejected_neighbors,
                "peak_open": res.max_open_size,
                "runtime_s": res.runtime_s,
                "min_agl_m": res.minimum_agl_m,
                "path_length_m": res.continuous_path_length_m if res.success else None,
                "goal_xy_error_m": res.goal_xy_error_m,
                "goal_z_error_m": res.goal_z_error_m,
            }

    # Save outputs
    json_path = ROOT / "results" / "vertical_bottleneck_mitigation_experiment.json"
    plot_path = ROOT / "results" / "vertical_bottleneck_mitigation_graphs.png"
    
    payload = {
        "mission_e_detailed_comparison": results_e,
        "cross_validation_a_to_f": cross_val_results,
    }
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
    print(f"\n[OK] Saved results to: {json_path}")

    generate_plots(results_e, cross_val_results, plot_path)
    print(f"[OK] Saved graphs to: {plot_path}")


if __name__ == "__main__":
    main()
