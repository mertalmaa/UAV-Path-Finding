import math
import sys
import time
import numpy as np
from pathlib import Path
from collections import Counter, defaultdict
import heapq
import itertools

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from planner.roi import load_roi
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from planner.aircraft_profile import load_aircraft_profile
from planner.fixed_wing_envelope import FixedWingKinematicEnvelope
from planner.physical import PhysicalPose, PhysicalTrajectory
from planner.pose_search import (
    GoalPose, GoalTolerance, PoseSearchNode, SearchKey,
    _candidate_trajectories, _trajectory_3d_length,
    navigation_bearing_deg, pose_in_goal, search_key_for_pose,
)
from planner.trajectory_safety import TerrainInfluenceCache, evaluate_physical_trajectory_safety
from scripts.benchmark_missions import CACHE_DIR, CONFIG, FACTOR, GOAL_TOLERANCE, PROFILE_PATH, SOURCE_DEM_PATH

def compute_fast_valley_search(start_rc, goal_rc, w_h=1.01, w_alt=0.20, use_valley=True):
    roi = load_roi(CONFIG)
    cache = load_terrain_cache(CACHE_DIR, roi, SOURCE_DEM_PATH)
    terrain = build_terrain_query_from_cache(cache, roi, FACTOR)
    profile = load_aircraft_profile(PROFILE_PATH)
    envelope = FixedWingKinematicEnvelope()
    infl_cache = TerrainInfluenceCache(terrain)

    elev_grid = terrain.roi.elevation.copy()
    valid = np.isfinite(elev_grid)
    min_e, max_e = float(np.min(elev_grid[valid])), float(np.max(elev_grid[valid]))
    elev_grid[~valid] = min_e
    cell_m = abs(float(terrain.roi.transform.a))
    rows, cols = elev_grid.shape

    # 1. Precompute topographic cost surface
    dy, dx = np.gradient(elev_grid, cell_m)
    slope = np.sqrt(dx*dx + dy*dy)
    max_s = float(np.percentile(slope, 95))
    r_norm = (elev_grid - min_e) / max(1.0, max_e - min_e)
    s_norm = np.clip(slope / max(0.1, max_s), 0.0, 2.0)
    cost_surf = 1.0 + 2.0 * (r_norm**2) + 1.0 * (s_norm**2)

    # 2. Precompute 2D Dijkstra Geodesic heuristic field
    gr, gc = max(0, min(rows-1, goal_rc[0])), max(0, min(cols-1, goal_rc[1]))
    h_grid = np.full((rows, cols), np.inf, dtype=np.float64)
    h_grid[gr, gc] = 0.0
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
                mu = 0.5 * (cost_surf[r, c] + cost_surf[nr, nc])
                nd = d + step * mu
                if nd < h_grid[nr, nc]:
                    h_grid[nr, nc] = nd
                    heapq.heappush(pq, (nd, nr, nc))

    sx, sy = terrain.rowcol_to_xy(*start_rc)
    gx, gy = terrain.rowcol_to_xy(*goal_rc)
    elev_s = terrain.query(sx, sy).elevation
    elev_g = terrain.query(gx, gy).elevation
    start_z = elev_s + 120.0
    goal_z = elev_g + 120.0
    heading = navigation_bearing_deg(sx, sy, gx, gy)

    start = PhysicalPose(sx, sy, start_z, heading)
    goal = GoalPose(gx, gy, goal_z)
    tol = GoalTolerance(xy_m=120.0, altitude_m=30.0)

    def heuristic_fn(p: PhysicalPose):
        if use_valley:
            r, c = terrain.xy_to_rowcol(p.x_m, p.y_m)
            r = max(0, min(rows - 1, r))
            c = max(0, min(cols - 1, c))
            h_2d = float(h_grid[r, c])
            dz = max(0.0, abs(p.z_msl_m - goal.z_msl_m) - tol.altitude_m)
            return math.sqrt(h_2d*h_2d + dz*dz)
        else:
            dx = p.x_m - goal.x_m
            dy = p.y_m - goal.y_m
            dz = max(0.0, abs(p.z_msl_m - goal.z_msl_m) - tol.altitude_m)
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
        elev = elev_grid[r, c]
        agl = ep.z_msl_m - elev
        excess_agl = max(0.0, agl - 120.0)
        return l3d * (mu + w_alt * (excess_agl / 1000.0))

    start_key = search_key_for_pose(start, CONFIG)
    start_node = PoseSearchNode(0, start_key, start, 0.0, None, None, None)
    active = {start_key: start_node}
    all_nodes = {0: start_node}
    pareto_frontier = defaultdict(list)
    start_xyh = (start_key.x_bin, start_key.y_bin, start_key.heading_bin)
    pareto_frontier[start_xyh].append((0.0, abs(start.z_msl_m - (elev_s + 120.0)), start_key))

    f_0 = w_h * heuristic_fn(start)
    counter = itertools.count(1)
    open_heap = [(f_0, 0, 0)]
    expanded_ids = set()
    expanded = 0
    generated = 0
    t0 = time.perf_counter()
    goal_node = None

    while open_heap and expanded < 30000:
        _, _, node_id = heapq.heappop(open_heap)
        node = all_nodes[node_id]
        if active.get(node.key) is not node or node_id in expanded_ids:
            continue
        expanded_ids.add(node_id)
        expanded += 1

        if pose_in_goal(node.end_pose, goal, tol):
            goal_node = node
            break

        for prim, traj in _candidate_trajectories(node.end_pose, envelope, CONFIG):
            generated += 1
            if traj is None:
                continue
            safety = evaluate_physical_trajectory_safety(
                traj, terrain, CONFIG.min_agl_m, CONFIG.primitive_sample_spacing_m,
                planning_bounds=terrain.roi.bounds, lateral_buffer_m=CONFIG.lateral_buffer_m,
                terrain_influence_cache=infl_cache,
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
            r_end, c_end = terrain.xy_to_rowcol(end_p.x_m, end_p.y_m)
            r_end = max(0, min(rows - 1, r_end))
            c_end = max(0, min(cols - 1, c_end))
            local_elev = elev_grid[r_end, c_end]
            cand_excess_agl = max(0.0, (end_p.z_msl_m - local_elev) - 120.0)

            # Pareto AGL Dominance
            frontier = pareto_frontier[xyh]
            dominated = False
            for fg, f_excess, _ in frontier:
                if fg <= cand_g + 1e-9 and f_excess <= cand_excess_agl + 1e-9:
                    dominated = True
                    break
            if dominated:
                continue

            pareto_frontier[xyh] = [
                (fg, fe, fk) for fg, fe, fk in frontier
                if not (cand_g <= fg + 1e-9 and cand_excess_agl <= fe + 1e-9)
            ]
            pareto_frontier[xyh].append((cand_g, cand_excess_agl, key))

            nid = next(counter)
            succ = PoseSearchNode(nid, key, end_p, cand_g, node.node_id, traj, prim)
            active[key] = succ
            all_nodes[nid] = succ
            f_score = cand_g + w_h * heuristic_fn(end_p)
            heapq.heappush(open_heap, (f_score, nid, nid))

    dt = time.perf_counter() - t0
    name = "VALLEY-GUIDED" if use_valley else "BASELINE-EUCLID"
    if goal_node:
        cur = goal_node
        agls = []
        path_len = 0.0
        prims = []
        traj_points = []
        while cur is not None:
            if cur.incoming_trajectory:
                path_len += _trajectory_3d_length(cur.incoming_trajectory)
                prims.append(cur.incoming_primitive)
                for s in cur.incoming_trajectory.samples:
                    elev = terrain.query(s.x_m, s.y_m).elevation
                    agl = s.z_msl_m - elev
                    agls.append(agl)
                    traj_points.append({
                        "x_m": s.x_m, "y_m": s.y_m, "z_msl_m": s.z_msl_m,
                        "elevation_msl_m": elev, "agl_m": agl, "heading_deg": s.heading_deg,
                    })
            cur = all_nodes.get(cur.parent_node_id)
        min_agl = min(agls) if agls else 0.0
        mean_agl = sum(agls)/len(agls) if agls else 0.0
        median_agl = float(np.median(agls))
        p90_agl = float(np.percentile(agls, 90))
        pct_100_150 = sum(1 for a in agls if 100.0 <= a <= 150.0) / len(agls) * 100.0
        pct_150_200 = sum(1 for a in agls if 150.0 < a <= 200.0) / len(agls) * 100.0
        pct_gt_250 = sum(1 for a in agls if a > 250.0) / len(agls) * 100.0
        print(f"[{name}] SUCCESS!")
        print(f"  Expansions: {expanded} | Generated: {generated} | Time: {dt:.3f}s | PathLen: {path_len:.1f}m | Primitives: {len(prims)}")
        print(f"  Min AGL: {min_agl:.1f}m | Mean AGL: {mean_agl:.1f}m | Median AGL: {median_agl:.1f}m | P90 AGL: {p90_agl:.1f}m")
        print(f"  Distribution: 100-150m: {pct_100_150:.1f}%, 150-200m: {pct_150_200:.1f}%, >250m: {pct_gt_250:.1f}%")
        return {
            "name": name, "success": True, "status": "success", "expansions": expanded, "generated": generated,
            "runtime_s": dt, "path_length_m": path_len, "primitives": list(reversed(prims)),
            "min_agl_m": min_agl, "mean_agl_m": mean_agl, "median_agl_m": median_agl, "p90_agl_m": p90_agl,
            "pct_100_150": pct_100_150, "pct_150_200": pct_150_200, "pct_gt_250": pct_gt_250,
            "start_pose": {"x_m": start.x_m, "y_m": start.y_m, "z_msl_m": start.z_msl_m, "heading_deg": start.heading_deg, "elev": elev_s, "agl": start_z - elev_s},
            "goal_pose": {"x_m": goal.x_m, "y_m": goal.y_m, "z_msl_m": goal.z_msl_m, "elev": elev_g, "target_agl": goal_z - elev_g},
            "trajectory": list(reversed(traj_points)),
        }
    else:
        print(f"[{name}] FAILED / NO PATH | Exp: {expanded} | Time: {dt:.3f}s")
        return {
            "name": name, "success": False, "status": "no_path", "expansions": expanded, "generated": generated,
            "runtime_s": dt, "path_length_m": 0.0, "primitives": [],
            "start_pose": {"x_m": start.x_m, "y_m": start.y_m, "z_msl_m": start.z_msl_m, "heading_deg": start.heading_deg, "elev": elev_s, "agl": start_z - elev_s},
            "goal_pose": {"x_m": goal.x_m, "y_m": goal.y_m, "z_msl_m": goal.z_msl_m, "elev": elev_g, "target_agl": goal_z - elev_g},
            "trajectory": [],
        }

if __name__ == "__main__":
    print("Testing Western Valley Corridor Mission: Row 75, Col 5 -> Row 20, Col 5 (3.3 km Northbound)")
    res_valley = compute_fast_valley_search((75, 5), (20, 5), w_h=1.01, w_alt=0.20, use_valley=True)
    res_base = compute_fast_valley_search((75, 5), (20, 5), w_h=1.01, w_alt=0.0, use_valley=False)
