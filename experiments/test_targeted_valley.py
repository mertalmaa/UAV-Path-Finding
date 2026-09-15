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

def test_valley_flight(start_rc, goal_rc, w_h=1.01, w_alt=0.0, use_topo=False):
    roi = load_roi(CONFIG)
    cache = load_terrain_cache(CACHE_DIR, roi, SOURCE_DEM_PATH)
    terrain = build_terrain_query_from_cache(cache, roi, FACTOR)
    profile = load_aircraft_profile(PROFILE_PATH)
    envelope = FixedWingKinematicEnvelope()
    infl_cache = TerrainInfluenceCache(terrain)

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

    elev_grid = terrain.roi.elevation.copy()
    valid = np.isfinite(elev_grid)
    min_e, max_e = float(np.min(elev_grid[valid])), float(np.max(elev_grid[valid]))
    elev_grid[~valid] = min_e
    cell_m = abs(float(terrain.roi.transform.a))
    dy, dx = np.gradient(elev_grid, cell_m)
    slope = np.sqrt(dx*dx + dy*dy)
    max_s = float(np.percentile(slope, 95))
    r_norm = (elev_grid - min_e) / max(1.0, max_e - min_e)
    s_norm = np.clip(slope / max(0.1, max_s), 0.0, 2.0)
    cost_surf = 1.0 + 2.0 * (r_norm**2) + 1.0 * (s_norm**2)

    rows, cols = cost_surf.shape
    gr, gc = max(0, min(rows-1, goal_rc[0])), max(0, min(cols-1, goal_rc[1]))
    dist_grid = np.full((rows, cols), np.inf, dtype=np.float64)
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
                mu = 0.5 * (cost_surf[r, c] + cost_surf[nr, nc])
                nd = d + step * mu
                if nd < dist_grid[nr, nc]:
                    dist_grid[nr, nc] = nd
                    heapq.heappush(pq, (nd, nr, nc))

    def heuristic_fn(p: PhysicalPose):
        if use_topo:
            r, c = terrain.xy_to_rowcol(p.x_m, p.y_m)
            r = max(0, min(rows - 1, r))
            c = max(0, min(cols - 1, c))
            h_2d = float(dist_grid[r, c])
            dz = max(0.0, abs(p.z_msl_m - goal.z_msl_m) - tol.altitude_m)
            return math.sqrt(h_2d*h_2d + dz*dz)
        else:
            dx = p.x_m - goal.x_m
            dy = p.y_m - goal.y_m
            dz = max(0.0, abs(p.z_msl_m - goal.z_msl_m) - tol.altitude_m)
            return math.sqrt(dx*dx + dy*dy + dz*dz)

    def edge_cost_fn(tr: PhysicalTrajectory):
        l3d = _trajectory_3d_length(tr)
        if not use_topo or w_alt <= 0.0:
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

        if pose_in_goal(node.end_pose, goal, tol):
            goal_node = node
            break

        for prim, traj in _candidate_trajectories(node.end_pose, envelope, CONFIG):
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
    mode_name = "VALLEY-GUIDED" if use_topo else "BASELINE-EUCLID"
    if goal_node:
        cur = goal_node
        agls = []
        path_len = 0.0
        prims = []
        while cur is not None:
            if cur.incoming_trajectory:
                path_len += _trajectory_3d_length(cur.incoming_trajectory)
                prims.append(cur.incoming_primitive)
                for s in cur.incoming_trajectory.samples:
                    elev = terrain.query(s.x_m, s.y_m).elevation
                    agls.append(s.z_msl_m - elev)
            cur = all_nodes.get(cur.parent_node_id)
        min_agl = min(agls) if agls else 0.0
        mean_agl = sum(agls)/len(agls) if agls else 0.0
        median_agl = float(np.median(agls))
        p90_agl = float(np.percentile(agls, 90))
        print(f"[{mode_name}] SUCCESS: Exp={expanded} | Time={dt:.2f}s | Len={path_len:.1f}m | Prims={len(prims)} | MinAGL={min_agl:.1f}m | MeanAGL={mean_agl:.1f}m | MedAGL={median_agl:.1f}m | P90AGL={p90_agl:.1f}m")
    else:
        print(f"[{mode_name}] FAILED / NO PATH: Exp={expanded} | Time={dt:.2f}s")

if __name__ == "__main__":
    print("Testing Western Valley Corridor Mission: Row 75, Col 5 -> Row 20, Col 5 (3.3 km Northbound)")
    test_valley_flight((75, 5), (20, 5), w_h=1.01, use_topo=False)
    test_valley_flight((75, 5), (20, 5), w_h=1.01, w_alt=0.25, use_topo=True)

    print("\nTesting Western Valley Corridor Mission: Row 85, Col 8 -> Row 20, Col 5 (3.9 km Northbound)")
    test_valley_flight((85, 8), (20, 5), w_h=1.01, use_topo=False)
    test_valley_flight((85, 8), (20, 5), w_h=1.01, w_alt=0.25, use_topo=True)
