"""Fixed-Wing UAV Low-Altitude Valley-Following Planner.

Implements:
1. Topographic relief and slope cost surface (Valley passability metric)
2. Geodesic Dijkstra 2D heuristic field computed over the DEM valley cost surface
3. Lookahead climb-reachability target envelope (dynamic terrain ceiling/floor)
4. Continuous 3D fixed-wing A* with DerivedC172P envelope and continuous collision caching
5. Comprehensive evaluation of Red Route (Valley Following) vs Black Route (Ridge Crossing)
6. 6-panel summary figure matching user specifications.
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

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

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
    _median,
    _safety_reason,
    _trajectory_3d_length,
    navigation_bearing_deg,
    pose_in_goal,
    search_key_for_pose,
)
from planner.roi import ROIData, load_roi
from planner.terrain import TerrainQuery
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from planner.trajectory_safety import (
    TerrainInfluenceCache,
    evaluate_physical_trajectory_safety,
)
from scripts.benchmark_missions import (
    CACHE_DIR,
    CONFIG,
    FACTOR,
    GOAL_TOLERANCE,
    PROFILE_PATH,
    SOURCE_DEM_PATH,
)

ARTIFACT_DIR = Path(r"C:\Users\PC_10004_YD26\.gemini\antigravity-ide\brain\703ac52a-e972-41a4-9422-6001171f41b7")


class TopographicValleyField:
    """Computes the continuous valley passability metric and geodesic distance field."""

    def __init__(
        self,
        roi: ROIData,
        w_ridge: float = 2.5,
        w_slope: float = 1.5,
    ):
        self.roi = roi
        self.elevation = roi.elevation.copy()
        valid_mask = np.isfinite(self.elevation)
        self.min_elev = float(np.min(self.elevation[valid_mask]))
        self.max_elev = float(np.max(self.elevation[valid_mask]))
        self.elevation[~valid_mask] = self.min_elev

        # Compute gradient (slope) using actual cell resolution
        cell_m = abs(float(roi.transform.a))
        dy, dx = np.gradient(self.elevation, cell_m)
        self.slope = np.sqrt(dx * dx + dy * dy)
        self.max_slope = float(np.percentile(self.slope, 95))

        # Normalized relief in [0, 1]
        relief_norm = (self.elevation - self.min_elev) / max(1.0, (self.max_elev - self.min_elev))
        slope_norm = np.clip(self.slope / max(0.1, self.max_slope), 0.0, 2.0)

        # Cost multiplier surface: Valleys ~ 1.0, High steep ridges ~ 4.0 - 6.0
        self.cost_surface = 1.0 + w_ridge * (relief_norm ** 2) + w_slope * (slope_norm ** 2)
        self.cell_m = cell_m

    def compute_geodesic_heuristic_grid(self, goal_rc: Tuple[int, int]) -> np.ndarray:
        """2D 8-connected Dijkstra geodesic distance on the topographic cost surface."""
        rows, cols = self.cost_surface.shape
        dist_grid = np.full((rows, cols), np.inf, dtype=np.float64)
        gr, gc = goal_rc
        gr = max(0, min(rows - 1, gr))
        gc = max(0, min(cols - 1, gc))
        dist_grid[gr, gc] = 0.0

        pq = [(0.0, gr, gc)]
        visited = set()

        cell_size = self.cell_m
        diag_size = self.cell_m * math.sqrt(2.0)
        neighbors = [
            (-1, 0, cell_size), (1, 0, cell_size), (0, -1, cell_size), (0, 1, cell_size),
            (-1, -1, diag_size), (-1, 1, diag_size), (1, -1, diag_size), (1, 1, diag_size),
        ]

        while pq:
            d, r, c = heapq.heappop(pq)
            if (r, c) in visited:
                continue
            visited.add((r, c))

            for dr, dc, step_len in neighbors:
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols:
                    mu = 0.5 * (self.cost_surface[r, c] + self.cost_surface[nr, nc])
                    new_d = d + step_len * mu
                    if new_d < dist_grid[nr, nc]:
                        dist_grid[nr, nc] = new_d
                        heapq.heappush(pq, (new_d, nr, nc))

        return dist_grid


@dataclass
class ValleyPlannerConfig:
    name: str
    description: str
    use_valley_heuristic: bool = True
    use_valley_cost: bool = True
    w_ridge: float = 2.0
    w_slope: float = 1.0
    w_alt: float = 0.30
    target_agl_m: float = 120.0
    heuristic_weight: float = 1.01
    delta_g_m: float = 25.0


def compute_lookahead_target_envelope(
    pose: PhysicalPose,
    goal: GoalPose,
    terrain: TerrainQuery,
    target_agl_m: float = 120.0,
    lookahead_m: float = 2500.0,
    climb_slope: float = 0.125,
    step_m: float = 60.0,
) -> float:
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


def run_valley_following_search(
    start: PhysicalPose,
    goal: GoalPose,
    terrain: TerrainQuery,
    aircraft_profile,
    topo_field: TopographicValleyField,
    cfg: ValleyPlannerConfig,
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

    # Precompute 2D geodesic heuristic grid if enabled
    goal_rc = terrain.xy_to_rowcol(goal.x_m, goal.y_m)
    rows, cols = topo_field.cost_surface.shape
    goal_r = max(0, min(rows - 1, goal_rc[0]))
    goal_c = max(0, min(cols - 1, goal_rc[1]))

    if cfg.use_valley_heuristic:
        h_grid = topo_field.compute_geodesic_heuristic_grid((goal_r, goal_c))
    else:
        h_grid = None

    def eval_edge_cost(trajectory: PhysicalTrajectory, safety) -> float:
        geom_len = _trajectory_3d_length(trajectory)
        if not cfg.use_valley_cost:
            return geom_len

        end_p = trajectory.end_pose
        r, c = terrain.xy_to_rowcol(end_p.x_m, end_p.y_m)
        r = max(0, min(rows - 1, r))
        c = max(0, min(cols - 1, c))
        mu_topo = topo_field.cost_surface[r, c]

        z_target = compute_lookahead_target_envelope(end_p, goal, terrain, cfg.target_agl_m)
        excess = max(0.0, end_p.z_msl_m - z_target)
        alt_factor = 1.0 + cfg.w_alt * (excess / 1000.0)
        return geom_len * mu_topo * alt_factor

    def eval_heuristic(p: PhysicalPose) -> float:
        if h_grid is not None:
            r, c = terrain.xy_to_rowcol(p.x_m, p.y_m)
            r = max(0, min(rows - 1, r))
            c = max(0, min(cols - 1, c))
            h_2d = float(h_grid[r, c])
            dz = max(0.0, abs(p.z_msl_m - goal.z_msl_m) - goal_tolerance.altitude_m)
            return math.sqrt(h_2d * h_2d + dz * dz)
        else:
            dx = p.x_m - goal.x_m
            dy = p.y_m - goal.y_m
            d_xy = math.sqrt(dx * dx + dy * dy)
            dz = max(0.0, abs(p.z_msl_m - goal.z_msl_m) - goal_tolerance.altitude_m)
            return math.sqrt(d_xy * d_xy + dz * dz)

    start_key = search_key_for_pose(start, config)
    start_node = PoseSearchNode(next(next_node_id), start_key, start, 0.0, None, None, None)
    active: Dict[SearchKey, PoseSearchNode] = {start_key: start_node}
    all_nodes: Dict[int, PoseSearchNode] = {start_node.node_id: start_node}

    xyh_representatives: Dict[Tuple[int, int, int], List[Tuple[float, float, SearchKey, int]]] = defaultdict(list)
    start_xyh = (start_key.x_bin, start_key.y_bin, start_key.heading_bin)
    z_targ_0 = compute_lookahead_target_envelope(start, goal, terrain, cfg.target_agl_m)
    xyh_representatives[start_xyh].append((0.0, max(0.0, start.z_msl_m - z_targ_0), start_key, start_node.node_id))

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

            # Bounded Envelope Dominance
            xyh = (key.x_bin, key.y_bin, key.heading_bin)
            z_targ = compute_lookahead_target_envelope(end_pose, goal, terrain, cfg.target_agl_m)
            cand_excess = max(0.0, end_pose.z_msl_m - z_targ)

            reps = xyh_representatives[xyh]
            dominated = False
            for rep_g, rep_excess, rep_key, _ in reps:
                if rep_excess <= cand_excess and rep_g <= candidate_g + cfg.delta_g_m:
                    dominated = True
                    break
            if dominated:
                rejected += 1
                reject_reasons["VALLEY_ENVELOPE_DOMINANCE"] += 1
                continue

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
    termination = {"success": "FOUND", "no_path": "OPEN_EXHAUSTED", "search_limit_reached": "EXPANSION_LIMIT", "timeout": "TIMEOUT"}.get(status, status)

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


def generate_six_panel_figure(
    roi: ROIData,
    terrain: TerrainQuery,
    black_route_res: dict,
    red_route_res: dict,
    valley_missions: dict,
    out_png: Path,
):
    """Generate publication-quality 6-panel summary figure matching User Image 2 format."""
    fig = plt.figure(figsize=(18, 16), constrained_layout=True)
    gs = GridSpec(3, 2, figure=fig)

    ax_map = fig.add_subplot(gs[0, 0])
    ax_alt = fig.add_subplot(gs[0, 1])
    ax_perf = fig.add_subplot(gs[1, 0])
    ax_agl_box = fig.add_subplot(gs[1, 1])
    ax_cross_e_map = fig.add_subplot(gs[2, 0])
    ax_cross_e_alt = fig.add_subplot(gs[2, 1])

    # 1. Top-Down Valley Comparison (Red vs Black)
    elev = roi.elevation
    im = ax_map.imshow(elev, cmap="terrain", origin="upper", extent=[0, 10, 0, 10])
    cbar = fig.colorbar(im, ax=ax_map, shrink=0.7, pad=0.02)
    cbar.set_label("Elevation MSL (m)", fontsize=9)

    def to_km(traj):
        xs = [(p["x_m"] - roi.bounds[0]) / 1000.0 for p in traj]
        ys = [(p["y_m"] - roi.bounds[1]) / 1000.0 for p in traj]
        return xs, ys

    b_xs, b_ys = to_km(black_route_res["trajectory"])
    r_xs, r_ys = to_km(red_route_res["trajectory"])

    if b_xs:
        ax_map.plot(b_xs, b_ys, color="black", linewidth=2.8, linestyle="--", label="Black Route (Ridge Crossing)")
        ax_map.scatter([b_xs[0]], [b_ys[0]], c="black", s=80, zorder=5)
        ax_map.scatter([b_xs[-1]], [b_ys[-1]], c="black", marker="*", s=120, zorder=5)

    if r_xs:
        ax_map.plot(r_xs, r_ys, color="red", linewidth=3.2, label="Red Route (Valley Following)")
        ax_map.scatter([r_xs[0]], [r_ys[0]], c="darkred", s=90, zorder=5, label="Red Start")
        ax_map.scatter([r_xs[-1]], [r_ys[-1]], c="magenta", marker="*", s=140, zorder=5, label="Red Goal")

    ax_map.set_title("Panel A: Top-Down Route Map (Black vs Red Route)", fontsize=12, fontweight="bold")
    ax_map.set_xlabel("Easting (km)", fontsize=10)
    ax_map.set_ylabel("Northing (km)", fontsize=10)
    ax_map.legend(fontsize=8, loc="upper right")
    ax_map.grid(True, linestyle=":", alpha=0.5)

    # 2. Altitude Profile (Red vs Black Route)
    def get_profile_data(traj):
        if not traj:
            return [], [], [], []
        dists = [0.0]
        for i in range(1, len(traj)):
            p0, p1 = traj[i-1], traj[i]
            d = math.hypot(p1["x_m"] - p0["x_m"], p1["y_m"] - p0["y_m"])
            dists.append(dists[-1] + d)
        alts = [p["z_msl_m"] for p in traj]
        terrs = [p["elevation_msl_m"] for p in traj]
        agls = [p["agl_m"] for p in traj]
        return dists, alts, terrs, agls

    b_d, b_alt, b_terr, b_agl = get_profile_data(black_route_res["trajectory"])
    r_d, r_alt, r_terr, r_agl = get_profile_data(red_route_res["trajectory"])

    if b_d:
        ax_alt.plot(b_d, b_alt, color="black", linestyle="--", linewidth=2.2, label="Black Aircraft Altitude (MSL)")
        ax_alt.plot(b_d, b_terr, color="#888888", linestyle=":", linewidth=1.2, label="Black Terrain")
    if r_d:
        ax_alt.plot(r_d, r_alt, color="red", linestyle="-", linewidth=2.5, label="Red Aircraft Altitude (MSL)")
        ax_alt.plot(r_d, r_terr, color="#222222", linewidth=1.5, label="Red Terrain Elevation (MSL)")
        ax_alt.fill_between(r_d, r_terr, [t + 100.0 for t in r_terr], color="orange", alpha=0.25, label="Red Hard Floor (+100m)")

    ax_alt.set_title("Panel B: Altitude Profile & Terrain Clearance", fontsize=12, fontweight="bold")
    ax_alt.set_xlabel("Cumulative Path Distance (m)", fontsize=10)
    ax_alt.set_ylabel("Altitude MSL (m)", fontsize=10)
    ax_alt.grid(True, linestyle=":", alpha=0.6)
    ax_alt.legend(fontsize=8, loc="upper left")

    # 3. Search Performance & Expansion Comparison
    categories = ["Black Route (Naive A*)", "Red Route (Valley A*)", "Mission E (Baseline)", "Mission E (Valley A*)"]
    expansions = [black_route_res["expansions"], red_route_res["expansions"], valley_missions["E_base"]["expansions"], valley_missions["E_valley"]["expansions"]]
    times = [black_route_res["runtime_s"], red_route_res["runtime_s"], valley_missions["E_base"]["runtime_s"], valley_missions["E_valley"]["runtime_s"]]

    x_pos = np.arange(len(categories))
    ax_perf.bar(x_pos - 0.2, expansions, width=0.4, color=["#d62728", "#2ca02c", "#d62728", "#2ca02c"], edgecolor="black", alpha=0.85, label="Expansions")
    ax_perf_t = ax_perf.twinx()
    ax_perf_t.plot(x_pos + 0.2, times, color="#1f77b4", marker="o", linewidth=2.0, label="Runtime (s)")

    ax_perf.set_xticks(x_pos)
    ax_perf.set_xticklabels(categories, fontsize=8, rotation=15, ha="right")
    ax_perf.set_ylabel("Expanded Nodes", fontsize=10)
    ax_perf_t.set_ylabel("Runtime (s)", fontsize=10, color="#1f77b4")
    ax_perf.set_title("Panel C: Search Expansions & Runtime Comparison", fontsize=12, fontweight="bold")
    ax_perf.grid(True, linestyle=":", alpha=0.5)

    # 4. AGL Distribution Boxplot / Metrics
    agl_data = []
    labels = []
    if b_agl:
        agl_data.append(b_agl)
        labels.append("Black Route")
    if r_agl:
        agl_data.append(r_agl)
        labels.append("Red Route")
    if valley_missions["E_base"]["trajectory"]:
        agl_data.append([p["agl_m"] for p in valley_missions["E_base"]["trajectory"]])
        labels.append("Mission E (Base)")
    if valley_missions["E_valley"]["trajectory"]:
        agl_data.append([p["agl_m"] for p in valley_missions["E_valley"]["trajectory"]])
        labels.append("Mission E (Valley)")

    if agl_data:
        box = ax_agl_box.boxplot(agl_data, tick_labels=labels, patch_artist=True)
        colors = ["#ff9999", "#99ff99", "#ffcc99", "#99ccff"]
        for patch, c in zip(box['boxes'], colors[:len(box['boxes'])]):
            patch.set_facecolor(c)
    ax_agl_box.axhline(100.0, color="red", linestyle="--", linewidth=1.5, label="Hard Floor (100m)")
    ax_agl_box.axhline(120.0, color="green", linestyle=":", linewidth=1.5, label="Target AGL (120m)")

    ax_agl_box.set_title("Panel D: Flight AGL Distribution (Terrain Following Quality)", fontsize=12, fontweight="bold")
    ax_agl_box.set_ylabel("AGL (m)", fontsize=10)
    ax_agl_box.grid(True, linestyle=":", alpha=0.5)
    ax_agl_box.legend(fontsize=8, loc="upper right")

    # 5. Mission E Top-Down View
    ax_cross_e_map.imshow(elev, cmap="terrain", origin="upper", extent=[0, 10, 0, 10])
    e_b_xs, e_b_ys = to_km(valley_missions["E_base"]["trajectory"])
    e_v_xs, e_v_ys = to_km(valley_missions["E_valley"]["trajectory"])
    if e_b_xs:
        ax_cross_e_map.plot(e_b_xs, e_b_ys, color="purple", linewidth=2.5, linestyle="--", label="Mission E Baseline")
    if e_v_xs:
        ax_cross_e_map.plot(e_v_xs, e_v_ys, color="blue", linewidth=2.8, label="Mission E Valley-Guided")
    ax_cross_e_map.set_title("Panel E: Mission E (9.3 km Cross-ROI Valley vs Ridge)", fontsize=12, fontweight="bold")
    ax_cross_e_map.set_xlabel("Easting (km)", fontsize=10)
    ax_cross_e_map.set_ylabel("Northing (km)", fontsize=10)
    ax_cross_e_map.legend(fontsize=8, loc="upper right")
    ax_cross_e_map.grid(True, linestyle=":", alpha=0.5)

    # 6. Mission E Altitude Profile
    e_b_d, e_b_alt, e_b_terr, _ = get_profile_data(valley_missions["E_base"]["trajectory"])
    e_v_d, e_v_alt, e_v_terr, _ = get_profile_data(valley_missions["E_valley"]["trajectory"])

    if e_b_d:
        ax_cross_e_alt.plot(e_b_d, e_b_alt, color="purple", linestyle="--", linewidth=2.0, label="Baseline (Stays High 4400m)")
    if e_v_d:
        ax_cross_e_alt.plot(e_v_d, e_v_alt, color="blue", linestyle="-", linewidth=2.2, label="Valley-Guided Altitude")
        ax_cross_e_alt.plot(e_v_d, e_v_terr, color="#555555", linewidth=1.5, label="Terrain Elevation (MSL)")
        ax_cross_e_alt.fill_between(e_v_d, e_v_terr, [t + 100.0 for t in e_v_terr], color="orange", alpha=0.25, label="Hard Safety Floor (+100m)")

    ax_cross_e_alt.set_title("Panel F: Mission E Vertical Flight Profile", fontsize=12, fontweight="bold")
    ax_cross_e_alt.set_xlabel("Cumulative Path Distance (m)", fontsize=10)
    ax_cross_e_alt.set_ylabel("Altitude MSL (m)", fontsize=10)
    ax_cross_e_alt.grid(True, linestyle=":", alpha=0.6)
    ax_cross_e_alt.legend(fontsize=8, loc="upper left")

    fig.suptitle("UAV Fixed-Wing Low-Altitude Valley-Following Global Path Planner: System Evaluation", fontsize=16, fontweight="bold")
    fig.savefig(out_png, dpi=200)
    plt.close(fig)
    print(f"Saved 6-panel summary figure to {out_png}")


def main():
    print("=" * 95)
    print("LOW-ALTITUDE VALLEY-FOLLOWING PLANNER INVESTIGATION & BENCHMARK")
    print("=" * 95)

    roi = load_roi(CONFIG)
    cache = load_terrain_cache(CACHE_DIR, roi, SOURCE_DEM_PATH)
    terrain = build_terrain_query_from_cache(cache, roi, FACTOR)
    profile = load_aircraft_profile(PROFILE_PATH)

    # Initialize Topographic Valley Field using terrain's downsampled grid
    topo_field = TopographicValleyField(terrain.roi, w_ridge=2.5, w_slope=1.5)

    # 1. Red Route: Natural North-South valley contour at the western foot of the massif
    red_start_rc = (140, 35)
    red_goal_rc = (15, 40)
    sx_r, sy_r = terrain.rowcol_to_xy(*red_start_rc)
    gx_r, gy_r = terrain.rowcol_to_xy(*red_goal_rc)
    q_sr = terrain.query(sx_r, sy_r)
    q_gr = terrain.query(gx_r, gy_r)
    start_z_r = q_sr.elevation + 120.0
    goal_z_r = q_gr.elevation + 120.0
    heading_r = navigation_bearing_deg(sx_r, sy_r, gx_r, gy_r)

    # 2. Black Route: Forces climbing straight into the mountain massif
    black_start_rc = (120, 20)
    black_goal_rc = (45, 100)
    sx_b, sy_b = terrain.rowcol_to_xy(*black_start_rc)
    gx_b, gy_b = terrain.rowcol_to_xy(*black_goal_rc)
    q_sb = terrain.query(sx_b, sy_b)
    q_gb = terrain.query(gx_b, gy_b)
    start_z_b = q_sb.elevation + 120.0
    goal_z_b = q_gb.elevation + 120.0
    heading_b = navigation_bearing_deg(sx_b, sy_b, gx_b, gy_b)

    # Planner configurations
    cfg_valley = ValleyPlannerConfig(name="ValleyPlanner", description="Geodesic Valley Heuristic + Topo Cost", use_valley_heuristic=True, use_valley_cost=True)
    cfg_baseline = ValleyPlannerConfig(name="BaselinePlanner", description="Euclidean A* (w=1.01)", use_valley_heuristic=False, use_valley_cost=False)

    print("\n--- RUNNING RED ROUTE (Valley Following Mission) ---")
    res_red, met_red = run_valley_following_search(
        PhysicalPose(sx_r, sy_r, start_z_r, heading_r), GoalPose(gx_r, gy_r, goal_z_r),
        terrain, profile, topo_field, cfg_valley,
    )
    print(f"  Red Route -> Status: {met_red['status'].upper()} | Exp: {met_red['expansions']} | Time: {met_red['runtime_s']:.2f}s | Min AGL: {met_red['min_agl_m']:.1f}m | Mean AGL: {met_red['mean_agl_m']:.1f}m | Len: {met_red['path_length_m']:.1f}m")

    print("\n--- RUNNING BLACK ROUTE (Ridge Crossing Mission) ---")
    res_black, met_black = run_valley_following_search(
        PhysicalPose(sx_b, sy_b, start_z_b, heading_b), GoalPose(gx_b, gy_b, goal_z_b),
        terrain, profile, topo_field, cfg_baseline,
    )
    print(f"  Black Route -> Status: {met_black['status'].upper()} | Exp: {met_black['expansions']} | Time: {met_black['runtime_s']:.2f}s | Min AGL: {met_black['min_agl_m']:.1f}m | Mean AGL: {met_black['mean_agl_m']:.1f}m | Len: {met_black['path_length_m']:.1f}m")

    # Run Mission E comparisons
    print("\n--- RUNNING MISSION E (9.3 km Cross-ROI) ---")
    sx_e, sy_e = terrain.rowcol_to_xy(80, 5)
    gx_e, gy_e = terrain.rowcol_to_xy(80, 160)
    heading_e = navigation_bearing_deg(sx_e, sy_e, gx_e, gy_e)

    _, met_e_base = run_valley_following_search(
        PhysicalPose(sx_e, sy_e, 4400.0, heading_e), GoalPose(gx_e, gy_e, 3900.0),
        terrain, profile, topo_field, cfg_baseline,
    )
    _, met_e_valley = run_valley_following_search(
        PhysicalPose(sx_e, sy_e, 4400.0, heading_e), GoalPose(gx_e, gy_e, 3900.0),
        terrain, profile, topo_field, cfg_valley,
    )
    print(f"  Mission E Baseline -> Exp: {met_e_base['expansions']} | Time: {met_e_base['runtime_s']:.2f}s | Mean AGL: {met_e_base['mean_agl_m']:.1f}m")
    print(f"  Mission E Valley   -> Exp: {met_e_valley['expansions']} | Time: {met_e_valley['runtime_s']:.2f}s | Mean AGL: {met_e_valley['mean_agl_m']:.1f}m")

    valley_missions = {
        "E_base": met_e_base,
        "E_valley": met_e_valley,
    }

    out_png = ROOT / "results" / "valley_following_evaluation_6panel.png"
    out_png.parent.mkdir(parents=True, exist_ok=True)
    generate_six_panel_figure(roi, terrain, met_black, met_red, valley_missions, out_png)

    # Copy to artifact dir
    shutil.copy(out_png, ARTIFACT_DIR / "valley_following_evaluation_6panel.png")

    out_json = ROOT / "results" / "valley_following_evaluation.json"
    with open(out_json, "w") as f:
        clean_json = {
            "red_route": {k: v for k, v in met_red.items() if k != "trajectory"},
            "black_route": {k: v for k, v in met_black.items() if k != "trajectory"},
            "mission_e_baseline": {k: v for k, v in met_e_base.items() if k != "trajectory"},
            "mission_e_valley": {k: v for k, v in met_e_valley.items() if k != "trajectory"},
        }
        json.dump(clean_json, f, indent=2)
    print(f"Saved evaluation JSON to {out_json}")


if __name__ == "__main__":
    main()
