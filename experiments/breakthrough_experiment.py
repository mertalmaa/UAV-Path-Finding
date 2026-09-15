"""Breakthrough Experiment for Low-Altitude Terrain-Following Global A*.

Tests 4 well-founded mathematical mechanisms:
1. Baseline Weighted A* w=1.01
2. Mechanism A: Delta-G Bounded Envelope Dominance (at same XYH, if |g1 - g2| <= delta_g, prefer node closer to feasible envelope)
3. Mechanism B: Coarse-Z Target Representative Selection (SearchKey Z-resolution = 25m/50m, retaining best terrain-aligned pose)
4. Mechanism C: Feasible Envelope Cost + Consistent Heuristic (admissible envelope heuristic)
5. Mechanism D: Integrated Feasible Envelope Guidance
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
class BreakthroughConfig:
    name: str
    description: str
    heuristic_weight: float = 1.01
    mode: str = "baseline" # "baseline", "delta_g_dominance", "coarse_z", "envelope_heuristic", "envelope_cost_consistent"
    delta_g_m: float = 20.0             # Tolerance in path length to prefer lower/target altitude
    z_bin_m: float = 5.0                # Z quantization
    w_alt: float = 0.20                 # Altitude weight for cost/heuristic
    target_agl_m: float = 120.0
    lookahead_m: float = 2500.0
    climb_slope: float = 0.125          # 5 m/s at 40 m/s


def compute_feasible_target_envelope(
    pose: PhysicalPose,
    goal: GoalPose,
    terrain: TerrainQuery,
    target_agl_m: float = 120.0,
    lookahead_m: float = 2500.0,
    climb_slope: float = 0.125,
    step_m: float = 100.0,
) -> float:
    """Compute terrain-following target altitude with lookahead climb clearance and goal reachability."""
    q_loc = terrain.query(pose.x_m, pose.y_m)
    loc_elev = q_loc.elevation if q_loc.valid and math.isfinite(q_loc.elevation) else 0.0
    target_alt = loc_elev + target_agl_m

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

    d_goal = math.hypot(goal.x_m - pose.x_m, goal.y_m - pose.y_m)
    goal_min_alt = goal.z_msl_m - climb_slope * d_goal
    target_alt = max(target_alt, goal_min_alt)
    return target_alt


def run_breakthrough_search(
    start: PhysicalPose,
    goal: GoalPose,
    terrain: TerrainQuery,
    aircraft_profile,
    cfg: BreakthroughConfig,
    goal_tolerance: GoalTolerance = GOAL_TOLERANCE,
    base_config: PlannerConfig = CONFIG,
    max_expansions: int = 30000,
    max_search_time_s: float = 60.0,
) -> Tuple[PoseSearchResult, Dict[str, Any]]:
    # Create customized config with specified Z bin
    config = dataclasses.replace(base_config, search_z_bin_m=cfg.z_bin_m)
    envelope = FixedWingKinematicEnvelope()
    influence_cache = TerrainInfluenceCache(terrain)
    started = time.perf_counter()
    counter = itertools.count()
    next_node_id = itertools.count()

    start_key = search_key_for_pose(start, config)
    start_node = PoseSearchNode(next(next_node_id), start_key, start, 0.0, None, None, None)
    active: Dict[SearchKey, PoseSearchNode] = {start_key: start_node}
    all_nodes: Dict[int, PoseSearchNode] = {start_node.node_id: start_node}

    # Tracking for Delta-G Bounded Dominance per (x_bin, y_bin, heading_bin)
    # xyh -> list of (g_cost, excess_alt, search_key, node_id)
    xyh_representatives: Dict[Tuple[int, int, int], List[Tuple[float, float, SearchKey, int]]] = defaultdict(list)
    if cfg.mode == "delta_g_dominance":
        start_xyh = (start_key.x_bin, start_key.y_bin, start_key.heading_bin)
        z_targ = compute_feasible_target_envelope(start, goal, terrain, cfg.target_agl_m, cfg.lookahead_m, cfg.climb_slope)
        start_excess = max(0.0, start.z_msl_m - z_targ)
        xyh_representatives[start_xyh].append((0.0, start_excess, start_key, start_node.node_id))

    def eval_edge_cost(trajectory: PhysicalTrajectory, safety) -> float:
        geom_len = _trajectory_3d_length(trajectory)
        if cfg.mode == "envelope_cost_consistent":
            end_p = trajectory.end_pose
            z_targ = compute_feasible_target_envelope(end_p, goal, terrain, cfg.target_agl_m, cfg.lookahead_m, cfg.climb_slope)
            excess = max(0.0, end_p.z_msl_m - z_targ)
            # Gentle penalty: 1000m excess adds w_alt
            return geom_len * (1.0 + cfg.w_alt * (excess / 1000.0))
        return geom_len

    def eval_heuristic(p: PhysicalPose) -> float:
        h_geom = _heuristic(p, goal, goal_tolerance)
        if cfg.mode in ("envelope_heuristic", "envelope_cost_consistent"):
            z_targ = compute_feasible_target_envelope(p, goal, terrain, cfg.target_agl_m, cfg.lookahead_m, cfg.climb_slope)
            excess = max(0.0, p.z_msl_m - z_targ)
            # Admissible/consistent lower bound heuristic
            d_xy = math.hypot(p.x_m - goal.x_m, p.y_m - goal.y_m)
            h_alt_cost = d_xy * (cfg.w_alt * (excess / 1000.0) * 0.5) # Underestimate
            return h_geom + h_alt_cost
        return h_geom

    h_0 = eval_heuristic(start)
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

            edge_c = eval_edge_cost(trajectory, safety)
            candidate_g = node.g_cost + edge_c
            existing = active.get(key)
            if existing is not None:
                collisions += 1
                if candidate_g >= existing.g_cost - 1e-12:
                    existing_better += 1
                    rejected += 1
                    reject_reasons["SAME_KEY_DOMINANCE"] += 1
                    continue
                replaced += 1

            if cfg.mode == "delta_g_dominance":
                xyh = (key.x_bin, key.y_bin, key.heading_bin)
                z_targ = compute_feasible_target_envelope(end_pose, goal, terrain, cfg.target_agl_m, cfg.lookahead_m, cfg.climb_slope)
                cand_excess = max(0.0, end_pose.z_msl_m - z_targ)

                # Delta-G Bounded Dominance:
                # If an existing representative has comparable path length (|g - cand_g| <= delta_g)
                # and strictly lower excess altitude, candidate is dominated!
                reps = xyh_representatives[xyh]
                dominated = False
                for rep_g, rep_excess, rep_key, _ in reps:
                    # If rep is lower/closer to target and within delta_g path length:
                    if rep_excess <= cand_excess and rep_g <= candidate_g + cfg.delta_g_m:
                        dominated = True
                        break
                if dominated:
                    rejected += 1
                    reject_reasons["DELTA_G_ENVELOPE_DOMINANCE"] += 1
                    continue

                # Filter out any reps that are dominated by candidate
                xyh_representatives[xyh] = [
                    (rg, re, rk, rn) for rg, re, rk, rn in reps
                    if not (cand_excess <= re and candidate_g <= rg + cfg.delta_g_m)
                ]
                xyh_representatives[xyh].append((candidate_g, cand_excess, key, node.node_id))

            successor = PoseSearchNode(
                next(next_node_id), key, end_pose, candidate_g, node.node_id, trajectory, primitive,
            )
            active[key] = successor
            all_nodes[successor.node_id] = successor
            open_inserted_by_primitive[primitive] += 1
            h_score = eval_heuristic(end_pose)
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
    print("BREAKTHROUGH EXPERIMENT FOR LOW-ALTITUDE TERRAIN-FOLLOWING GLOBAL A*")
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
        BreakthroughConfig(
            name="Baseline_w101",
            description="Baseline Weighted A* (w=1.01, pure geometric)",
            mode="baseline",
        ),
        BreakthroughConfig(
            name="DeltaG_Dom_d10",
            description="Delta-G Bounded Envelope Dominance (delta_g=10m)",
            mode="delta_g_dominance",
            delta_g_m=10.0,
        ),
        BreakthroughConfig(
            name="DeltaG_Dom_d25",
            description="Delta-G Bounded Envelope Dominance (delta_g=25m)",
            mode="delta_g_dominance",
            delta_g_m=25.0,
        ),
        BreakthroughConfig(
            name="DeltaG_Dom_d50",
            description="Delta-G Bounded Envelope Dominance (delta_g=50m)",
            mode="delta_g_dominance",
            delta_g_m=50.0,
        ),
        BreakthroughConfig(
            name="CoarseZ_25m",
            description="Coarse Z-Quantization (Z_bin=25m)",
            mode="baseline",
            z_bin_m=25.0,
        ),
        BreakthroughConfig(
            name="CoarseZ_50m",
            description="Coarse Z-Quantization (Z_bin=50m)",
            mode="baseline",
            z_bin_m=50.0,
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

            res, met = run_breakthrough_search(
                start, goal, terrain, profile, var,
                goal_tolerance=GOAL_TOLERANCE, base_config=CONFIG,
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

    out_file = ROOT / "results" / "breakthrough_experiment.json"
    out_file.parent.mkdir(parents=True, exist_ok=True)
    with open(out_file, "w") as f:
        json.dump(all_results, f, indent=2)
    print(f"\nSaved results to {out_file}")


if __name__ == "__main__":
    main()
