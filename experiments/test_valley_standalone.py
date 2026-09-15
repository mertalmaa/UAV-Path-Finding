"""Quick standalone test of Low-Altitude Valley-Following Planner."""
import math
import time
import heapq
import itertools
from collections import defaultdict, Counter
import numpy as np

import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from planner.config import DEFAULT_CONFIG, PlannerConfig
from planner.roi import load_roi, ROIData
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from planner.aircraft_profile import load_aircraft_profile
from planner.fixed_wing_envelope import FixedWingKinematicEnvelope
from planner.physical import (
    PhysicalPose,
    PhysicalTrajectory,
    FIXED_PLANAR_SPEED_MPS,
)
from planner.pose_search import (
    GoalPose,
    GoalTolerance,
    PoseSearchNode,
    PoseSearchResult,
    SearchKey,
    _candidate_trajectories,
    _goal_errors,
    _trajectory_3d_length,
    navigation_bearing_deg,
    pose_in_goal,
    search_key_for_pose,
)
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

def compute_valley_grid(roi: ROIData, w_relief=2.0, w_slope=1.0):
    elev = roi.elevation.copy()
    valid = np.isfinite(elev)
    min_e, max_e = float(np.min(elev[valid])), float(np.max(elev[valid]))
    elev[~valid] = min_e
    cell_m = abs(float(roi.transform.a))
    dy, dx = np.gradient(elev, cell_m)
    slope = np.sqrt(dx*dx + dy*dy)
    max_s = float(np.percentile(slope, 95))

    r_norm = (elev - min_e) / max(1.0, max_e - min_e)
    s_norm = np.clip(slope / max(0.1, max_s), 0.0, 2.0)
    cost_surface = 1.0 + w_relief * (r_norm**2) + w_slope * (s_norm**2)
    return cost_surface, cell_m

def compute_dijkstra_heuristic(cost_surface, cell_m, goal_rc):
    rows, cols = cost_surface.shape
    dist_grid = np.full((rows, cols), np.inf, dtype=np.float64)
    gr, gc = max(0, min(rows-1, goal_rc[0])), max(0, min(cols-1, goal_rc[1]))
    dist_grid[gr, gc] = 0.0
    pq = [(0.0, gr, gc)]
    visited = set()
    diag_m = cell_m * math.sqrt(2.0)
    nbrs = [
        (-1, 0, cell_m), (1, 0, cell_m), (0, -1, cell_m), (0, 1, cell_m),
        (-1, -1, diag_m), (-1, 1, diag_m), (1, -1, diag_m), (1, 1, diag_m),
    ]
    while pq:
        d, r, c = heapq.heappop(pq)
        if (r, c) in visited:
            continue
        visited.add((r, c))
        for dr, dc, step in nbrs:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                mu = 0.5 * (cost_surface[r, c] + cost_surface[nr, nc])
                nd = d + step * mu
                if nd < dist_grid[nr, nc]:
                    dist_grid[nr, nc] = nd
                    heapq.heappush(pq, (nd, nr, nc))
    return dist_grid

def run_test(name, start_rc, goal_rc, w_h=1.01, w_alt=0.25, use_valley=True):
    roi = load_roi(CONFIG)
    cache = load_terrain_cache(CACHE_DIR, roi, SOURCE_DEM_PATH)
    terrain = build_terrain_query_from_cache(cache, roi, FACTOR)
    profile = load_aircraft_profile(PROFILE_PATH)
    envelope = FixedWingKinematicEnvelope()
    cache_infl = TerrainInfluenceCache(terrain)

    cost_surf, cell_m = compute_valley_grid(terrain.roi)
    rows, cols = cost_surf.shape

    sx, sy = terrain.rowcol_to_xy(*start_rc)
    gx, gy = terrain.rowcol_to_xy(*goal_rc)
    q_s = terrain.query(sx, sy)
    q_g = terrain.query(gx, gy)
    start_z = q_s.elevation + 120.0
    goal_z = q_g.elevation + 120.0
    heading = navigation_bearing_deg(sx, sy, gx, gy)

    start = PhysicalPose(sx, sy, start_z, heading)
    goal = GoalPose(gx, gy, goal_z)

    if use_valley:
        h_grid = compute_dijkstra_heuristic(cost_surf, cell_m, goal_rc)
    else:
        h_grid = None

    def heuristic_fn(p: PhysicalPose):
        if h_grid is not None:
            r, c = terrain.xy_to_rowcol(p.x_m, p.y_m)
            r = max(0, min(rows - 1, r))
            c = max(0, min(cols - 1, c))
            h_2d = float(h_grid[r, c])
            dz = max(0.0, abs(p.z_msl_m - goal.z_msl_m) - GOAL_TOLERANCE.altitude_m)
            return math.sqrt(h_2d*h_2d + dz*dz)
        else:
            dx = p.x_m - goal.x_m
            dy = p.y_m - goal.y_m
            dz = max(0.0, abs(p.z_msl_m - goal.z_msl_m) - GOAL_TOLERANCE.altitude_m)
            return math.sqrt(dx*dx + dy*dy + dz*dz)

    def edge_cost_fn(tr: PhysicalTrajectory):
        l3d = _trajectory_3d_length(tr)
        if not use_valley:
            return l3d
        ep = tr.end_pose
        r, c = terrain.xy_to_rowcol(ep.x_m, ep.y_m)
        r = max(0, min(rows - 1, r))
        c = max(0, min(cols - 1, c))
        mu = cost_surf[r, c]
        q = terrain.query(ep.x_m, ep.y_m)
        elev = q.elevation if q.valid and math.isfinite(q.elevation) else 0.0
        agl = ep.z_msl_m - elev
        excess_agl = max(0.0, agl - 120.0)
        return l3d * (mu + w_alt * (excess_agl / 1000.0))

    start_key = search_key_for_pose(start, CONFIG)
    start_node = PoseSearchNode(0, start_key, start, 0.0, None, None, None)
    active = {start_key: start_node}
    all_nodes = {0: start_node}
    pareto_frontier = defaultdict(list)
    start_xyh = (start_key.x_bin, start_key.y_bin, start_key.heading_bin)
    pareto_frontier[start_xyh].append((0.0, abs(start.z_msl_m - goal.z_msl_m), start_key))

    f_0 = w_h * heuristic_fn(start)
    counter = itertools.count(1)
    open_heap = [(f_0, 0, 0)]
    expanded_ids = set()
    expanded = 0
    t0 = time.perf_counter()
    goal_node = None

    while open_heap and expanded < 30000:
        _, _, node_id = heapq.heappop(open_heap)
        node = all_nodes[node_id]
        if active.get(node.key) is not node or node_id in expanded_ids:
            continue
        expanded_ids.add(node_id)
        expanded += 1

        if pose_in_goal(node.end_pose, goal, GOAL_TOLERANCE):
            goal_node = node
            break

        for prim, traj in _candidate_trajectories(node.end_pose, envelope, CONFIG):
            if traj is None:
                continue
            safety = evaluate_physical_trajectory_safety(
                traj, terrain, CONFIG.min_agl_m, CONFIG.primitive_sample_spacing_m,
                planning_bounds=terrain.roi.bounds, lateral_buffer_m=CONFIG.lateral_buffer_m,
                terrain_influence_cache=cache_infl,
            )
            if not safety.is_safe:
                continue

            end_p = traj.end_pose
            key = search_key_for_pose(end_p, CONFIG)
            if key == node.key:
                continue

            cand_g = node.g_cost + edge_cost_fn(traj)
            existing = active.get(key)
            if existing is not None and cand_g >= existing.g_cost - 1e-12:
                continue

            xyh = (key.x_bin, key.y_bin, key.heading_bin)
            cand_z_dist = abs(end_p.z_msl_m - goal.z_msl_m)

            # Pareto Z-dominance
            frontier = pareto_frontier[xyh]
            dominated = False
            for fg, fz, _ in frontier:
                if fg <= cand_g + 1e-9 and fz <= cand_z_dist + 1e-9:
                    dominated = True
                    break
            if dominated:
                continue

            pareto_frontier[xyh] = [
                (fg, fz, fk) for fg, fz, fk in frontier
                if not (cand_g <= fg + 1e-9 and cand_z_dist <= fz + 1e-9)
            ]
            pareto_frontier[xyh].append((cand_g, cand_z_dist, key))

            nid = next(counter)
            succ = PoseSearchNode(nid, key, end_p, cand_g, node.node_id, traj, prim)
            active[key] = succ
            all_nodes[nid] = succ
            f_score = cand_g + w_h * heuristic_fn(end_p)
            heapq.heappush(open_heap, (f_score, nid, nid))

    dt = time.perf_counter() - t0
    if goal_node:
        # Reconstruct path and compute metrics
        cur = goal_node
        agls = []
        path_len = 0.0
        while cur is not None:
            if cur.incoming_trajectory:
                path_len += _trajectory_3d_length(cur.incoming_trajectory)
                for s in cur.incoming_trajectory.samples:
                    elev = terrain.query(s.x_m, s.y_m).elevation
                    agls.append(s.z_msl_m - elev)
            cur = all_nodes.get(cur.parent_node_id)
        min_agl = min(agls) if agls else 0.0
        mean_agl = sum(agls)/len(agls) if agls else 0.0
        print(f"[{name}] SUCCESS! Exp: {expanded} | Time: {dt:.2f}s | Len: {path_len:.1f}m | Min AGL: {min_agl:.1f}m | Mean AGL: {mean_agl:.1f}m")
    else:
        print(f"[{name}] FAILED / NO PATH! Exp: {expanded} | Time: {dt:.2f}s")

if __name__ == "__main__":
    print("Testing Red Route (Valley Following):")
    run_test("Red Route (Valley Guided)", (140, 35), (15, 40), w_h=1.01, use_valley=True)
    run_test("Red Route (Euclidean Baseline)", (140, 35), (15, 40), w_h=1.01, use_valley=False)

    print("\nTesting Black Route (Ridge Crossing):")
    run_test("Black Route (Ridge Crossing)", (120, 20), (45, 100), w_h=1.01, use_valley=False)
