"""Systematic Exploration of Low-Altitude Mechanisms for Fixed-Wing Global A*.

Evaluates pure, non-destructive mechanisms:
1. Baseline: Weighted A* w=1.01 (pure geometric, FIFO tie-breaking)
2. Dynamic Primitive Ordering: Order candidates by descent/climb preference relative to target envelope
3. F-Band Tie-Breaking: Within f-band (eps=10m, 25m, 50m), prioritize lower AGL / proximity to target envelope
4. Pareto Vertical Dominance: Dominance among vertical states based on (g_cost, |z - z_target|)
5. Combinations
"""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass
import heapq
import itertools
import json
import math
from pathlib import Path
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

from planner.aircraft_profile import load_aircraft_profile
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
class MechanismConfig:
    name: str
    description: str
    heuristic_weight: float = 1.01
    tie_breaker: str = "fifo"           # "fifo", "agl_min", "band_agl_min", "band_target_dist"
    f_band_m: float = 0.0              # Band width for f-band tie breaking (meters)
    dynamic_primitive_ordering: bool = False
    pareto_z_dominance: str = "none"    # "none", "goal_z", "target_envelope"
    target_agl_m: float = 120.0
    lookahead_m: float = 2500.0
    climb_slope: float = 0.125         # 5 m/s at 40 m/s


def compute_target_envelope_alt(
    pose: PhysicalPose,
    goal: GoalPose,
    terrain: TerrainQuery,
    target_agl_m: float = 120.0,
    lookahead_m: float = 2500.0,
    climb_slope: float = 0.125,
    step_m: float = 60.0,
) -> float:
    """Compute terrain-following target altitude with lookahead climb clearance."""
    q_loc = terrain.query(pose.x_m, pose.y_m)
    loc_elev = q_loc.elevation if q_loc.valid and math.isfinite(q_loc.elevation) else 0.0
    target_alt = loc_elev + target_agl_m

    # Lookahead along heading
    heading_rad = math.radians(pose.heading_deg)
    dx = math.sin(heading_rad)
    dy = math.cos(heading_rad)
    n_steps = max(1, int(lookahead_m / step_m))
    for i in range(1, n_steps + 1):
        d = i * step_m
        px = pose.x_m + d * dx
        py = pose.y_m + d * dy
        q = terrain.query(px, py)
        if q.valid and math.isfinite(q.elevation):
            req = (q.elevation + target_agl_m) - climb_slope * d
            target_alt = max(target_alt, req)

    # Goal reachability
    d_goal = math.hypot(goal.x_m - pose.x_m, goal.y_m - pose.y_m)
    goal_min_alt = goal.z_msl_m - climb_slope * d_goal
    target_alt = max(target_alt, goal_min_alt)
    return target_alt


def generate_candidate_trajectories(
    pose: PhysicalPose,
    envelope: FixedWingKinematicEnvelope,
    config: PlannerConfig,
    order_mode: str = "standard", # "standard", "prefer_descent", "prefer_climb"
) -> List[Tuple[str, Optional[PhysicalTrajectory]]]:
    spacing = config.primitive_sample_spacing_m
    items = []

    # Straight level
    items.append(("STRAIGHT_LEVEL", build_straight_level_trajectory(pose, 60.0, spacing).trajectory))

    # Turns
    for direction, label in (("LEFT", "LEFT_LEVEL_TURN"), ("RIGHT", "RIGHT_LEVEL_TURN")):
        limit = envelope.level_turn(pose.z_msl_m, direction)
        if limit.availability == "AVAILABLE" and limit.level_turn is not None:
            items.append((label, build_level_turn_trajectory(pose, limit.level_turn, 15.0, spacing).trajectory))
        else:
            items.append((label, None))

    # Vertical
    for mode, label in (("CLIMB", "STRAIGHT_CLIMB"), ("DESCENT", "STRAIGHT_DESCENT")):
        limit = envelope.straight_vertical(pose.z_msl_m, mode)
        if limit.availability == "AVAILABLE" and limit.signed_vertical_rate_mps is not None:
            items.append((label, build_straight_vertical_trajectory(
                pose, 60.0, limit.signed_vertical_rate_mps, spacing
            ).trajectory))
        else:
            items.append((label, None))

    if config.enable_combined_turns:
        for mode, mode_label in (("CLIMB", "CLIMBING"), ("DESCENT", "DESCENDING")):
            for direction, dir_label in (("LEFT", "LEFT"), ("RIGHT", "RIGHT")):
                label = f"{mode_label}_{dir_label}_TURN"
                limit = envelope.combined_turn(pose.z_msl_m, direction, mode)
                if limit.availability == "AVAILABLE" and limit.combined_turn is not None:
                    items.append((label, build_helical_turn_trajectory(
                        pose, limit.combined_turn, 15.0, spacing
                    ).trajectory))
                else:
                    items.append((label, None))

    if order_mode == "prefer_descent":
        descent_items = [it for it in items if "DESCENT" in it[0] or "DESCENDING" in it[0]]
        level_items = [it for it in items if "LEVEL" in it[0]]
        climb_items = [it for it in items if "CLIMB" in it[0] or "CLIMBING" in it[0]]
        return descent_items + level_items + climb_items
    elif order_mode == "prefer_climb":
        climb_items = [it for it in items if "CLIMB" in it[0] or "CLIMBING" in it[0]]
        level_items = [it for it in items if "LEVEL" in it[0]]
        descent_items = [it for it in items if "DESCENT" in it[0] or "DESCENDING" in it[0]]
        return climb_items + level_items + descent_items
    return items


def run_mechanism_search(
    start: PhysicalPose,
    goal: GoalPose,
    terrain: TerrainQuery,
    aircraft_profile,
    mech: MechanismConfig,
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

    start_key = search_key_for_pose(start, config)
    start_node = PoseSearchNode(next(next_node_id), start_key, start, 0.0, None, None, None)
    active: Dict[SearchKey, PoseSearchNode] = {start_key: start_node}
    all_nodes: Dict[int, PoseSearchNode] = {start_node.node_id: start_node}

    # Pareto frontier tracking per (x_bin, y_bin, heading_bin)
    pareto_frontier: Dict[Tuple[int, int, int], List[Tuple[float, float, SearchKey]]] = defaultdict(list)
    if mech.pareto_z_dominance != "none":
        start_xyh = (start_key.x_bin, start_key.y_bin, start_key.heading_bin)
        if mech.pareto_z_dominance == "goal_z":
            z_metric = abs(start.z_msl_m - goal.z_msl_m)
        else: # "target_envelope"
            z_targ = compute_target_envelope_alt(start, goal, terrain, mech.target_agl_m, mech.lookahead_m, mech.climb_slope)
            z_metric = abs(start.z_msl_m - z_targ)
        pareto_frontier[start_xyh].append((0.0, z_metric, start_key))

    def make_queue_item(f: float, g: float, h: float, pose: PhysicalPose, nid: int):
        c = next(counter)
        tb = mech.tie_breaker
        if tb == "fifo":
            return (f, c, nid)
        elif tb == "agl_min":
            q = terrain.query(pose.x_m, pose.y_m)
            agl = pose.z_msl_m - (q.elevation if q.valid and math.isfinite(q.elevation) else 0.0)
            return (f, agl, c, nid)
        elif tb == "band_agl_min":
            band = round(f / mech.f_band_m) if mech.f_band_m > 0 else f
            q = terrain.query(pose.x_m, pose.y_m)
            agl = pose.z_msl_m - (q.elevation if q.valid and math.isfinite(q.elevation) else 0.0)
            return (band, agl, f, c, nid)
        elif tb == "band_target_dist":
            band = round(f / mech.f_band_m) if mech.f_band_m > 0 else f
            z_target = compute_target_envelope_alt(pose, goal, terrain, mech.target_agl_m, mech.lookahead_m, mech.climb_slope)
            excess = max(0.0, pose.z_msl_m - z_target)
            return (band, excess, f, c, nid)
        return (f, c, nid)

    h_0 = _heuristic(start, goal, goal_tolerance)
    f_0 = 0.0 + mech.heuristic_weight * h_0
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

        # Primitive order mode
        if mech.dynamic_primitive_ordering:
            z_targ = compute_target_envelope_alt(node.end_pose, goal, terrain, mech.target_agl_m, mech.lookahead_m, mech.climb_slope)
            if node.end_pose.z_msl_m > z_targ + 20.0:
                p_mode = "prefer_descent"
            elif node.end_pose.z_msl_m < z_targ - 20.0:
                p_mode = "prefer_climb"
            else:
                p_mode = "standard"
        else:
            p_mode = "standard"

        for primitive, trajectory in generate_candidate_trajectories(node.end_pose, envelope, config, p_mode):
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

            if mech.pareto_z_dominance != "none":
                xyh = (key.x_bin, key.y_bin, key.heading_bin)
                if mech.pareto_z_dominance == "goal_z":
                    cand_z_metric = abs(end_pose.z_msl_m - goal.z_msl_m)
                else:
                    z_targ = compute_target_envelope_alt(end_pose, goal, terrain, mech.target_agl_m, mech.lookahead_m, mech.climb_slope)
                    cand_z_metric = abs(end_pose.z_msl_m - z_targ)

                frontier = pareto_frontier[xyh]
                dominated = False
                for f_g, f_z, _ in frontier:
                    if f_g <= candidate_g + 1e-9 and f_z <= cand_z_metric + 1e-9:
                        dominated = True
                        break
                if dominated:
                    rejected += 1
                    reject_reasons["PARETO_Z_DOMINANCE"] += 1
                    continue
                pareto_frontier[xyh] = [
                    (fg, fz, fk) for fg, fz, fk in frontier
                    if not (candidate_g <= fg + 1e-9 and cand_z_metric <= fz + 1e-9)
                ]
                pareto_frontier[xyh].append((candidate_g, cand_z_metric, key))

            successor = PoseSearchNode(
                next(next_node_id), key, end_pose, candidate_g, node.node_id, trajectory, primitive,
            )
            active[key] = successor
            all_nodes[successor.node_id] = successor
            open_inserted_by_primitive[primitive] += 1
            h_score = _heuristic(end_pose, goal, goal_tolerance)
            f_score = candidate_g + mech.heuristic_weight * h_score
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

        # Continuous trajectory metrics & AGL
        for item in path[1:]:
            tr = item.incoming_trajectory
            if tr is not None:
                path_len += _trajectory_3d_length(tr)
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


def main():
    print("=" * 95)
    print("SYSTEMATIC EXPLORATION OF LOW-ALTITUDE MECHANISMS")
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

    variants = [
        MechanismConfig(
            name="Baseline_w101",
            description="Baseline Weighted A* (w=1.01, FIFO tie-breaking)",
            tie_breaker="fifo",
        ),
        MechanismConfig(
            name="DynPrimOrder",
            description="Dynamic Primitive Ordering (prefer descent when high)",
            dynamic_primitive_ordering=True,
            tie_breaker="fifo",
        ),
        MechanismConfig(
            name="FBand10_AGLMin",
            description="F-Band (eps=10m) Tie-Breaking on Minimum AGL",
            f_band_m=10.0,
            tie_breaker="band_agl_min",
        ),
        MechanismConfig(
            name="FBand25_AGLMin",
            description="F-Band (eps=25m) Tie-Breaking on Minimum AGL",
            f_band_m=25.0,
            tie_breaker="band_agl_min",
        ),
        MechanismConfig(
            name="FBand25_TargetDist",
            description="F-Band (eps=25m) Tie-Breaking on Distance to Feasible Target",
            f_band_m=25.0,
            tie_breaker="band_target_dist",
        ),
        MechanismConfig(
            name="DynPrim_FBand25_TargetDist",
            description="Dynamic Primitive Order + F-Band (eps=25m) Target Dist",
            dynamic_primitive_ordering=True,
            f_band_m=25.0,
            tie_breaker="band_target_dist",
        ),
        MechanismConfig(
            name="Pareto_TargetDom_w101",
            description="Weighted A* w=1.01 + Target Envelope Pareto Z-Dominance",
            pareto_z_dominance="target_envelope",
            tie_breaker="fifo",
        ),
        MechanismConfig(
            name="Combined_Pareto_DynPrim_FBand25",
            description="Pareto Z-Dom + Dyn Prim Order + F-Band (25m) Target Dist",
            pareto_z_dominance="target_envelope",
            dynamic_primitive_ordering=True,
            f_band_m=25.0,
            tie_breaker="band_target_dist",
        ),
    ]

    all_results = {}

    for var in variants:
        print(f"\n{'=' * 95}")
        print(f"EVALUATING: {var.name} - {var.description}")
        print(f"{'=' * 95}")
        var_dict = {}
        for m_id, m_data in clean_missions.items():
            sx, sy = terrain.rowcol_to_xy(*m_data["start_rc"])
            gx, gy = terrain.rowcol_to_xy(*m_data["goal_rc"])
            heading = navigation_bearing_deg(sx, sy, gx, gy)
            start = PhysicalPose(sx, sy, m_data["start_z"], heading)
            goal = GoalPose(gx, gy, m_data["goal_z"])

            res, met = run_mechanism_search(
                start, goal, terrain, profile, var,
                goal_tolerance=GOAL_TOLERANCE, config=CONFIG,
                max_expansions=30000, max_search_time_s=60.0,
            )
            var_dict[m_id] = met
            status_str = f"{met['status'].upper():<7}" if met['success'] else f"FAIL ({met['status'].upper()})"
            print(
                f"  Mission {m_id} -> {status_str:<22} | "
                f"Exp: {met['expansions']:>6} | "
                f"Time: {met['runtime_s']:>5.2f}s | "
                f"Min AGL: {met['min_agl_m']:>5.1f}m | "
                f"Mean AGL: {met['mean_agl_m']:>5.1f}m | "
                f"P90 AGL: {met['p90_agl_m']:>5.1f}m | "
                f"Len: {met['path_length_m']:>7.1f}m"
            )
        all_results[var.name] = var_dict

    out_file = ROOT / "results" / "low_altitude_mechanism_exploration.json"
    out_file.parent.mkdir(parents=True, exist_ok=True)
    with open(out_file, "w") as f:
        json.dump(all_results, f, indent=2)
    print(f"\nSaved results to {out_file}")


if __name__ == "__main__":
    main()
