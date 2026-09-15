import math
import sys
import time
import json
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
    _candidate_trajectories, _trajectory_3d_length, _goal_errors,
    navigation_bearing_deg, pose_in_goal, search_key_for_pose,
)
from planner.trajectory_safety import TerrainInfluenceCache, evaluate_physical_trajectory_safety
from scripts.benchmark_far_missions import FAR_MISSIONS
from scripts.benchmark_missions import (
    CACHE_DIR, CONFIG, FACTOR, GOAL_TOLERANCE, PROFILE_PATH, SOURCE_DEM_PATH,
    mission_definitions,
)

def build_cost_surface(terrain):
    elev_grid = terrain.roi.elevation.copy()
    valid = np.isfinite(elev_grid)
    min_e, max_e = float(np.min(elev_grid[valid])), float(np.max(elev_grid[valid]))
    elev_grid[~valid] = min_e
    cell_m = abs(float(terrain.roi.transform.a))

    dy, dx = np.gradient(elev_grid, cell_m)
    slope = np.sqrt(dx * dx + dy * dy)
    max_s = float(np.percentile(slope, 95))
    r_norm = (elev_grid - min_e) / max(1.0, max_e - min_e)
    s_norm = np.clip(slope / max(0.1, max_s), 0.0, 2.0)
    cost_surf = 1.0 + 1.5 * (r_norm ** 2) + 0.8 * (s_norm ** 2)
    return elev_grid, cost_surf, cell_m

def compute_dijkstra_2d(cost_surf, cell_m, goal_rc):
    rows, cols = cost_surf.shape
    gr, gc = max(0, min(rows - 1, goal_rc[0])), max(0, min(cols - 1, goal_rc[1]))
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
    return h_grid

def run_consistent_search(
    start: PhysicalPose,
    goal: GoalPose,
    terrain,
    envelope,
    infl_cache,
    elev_grid,
    cost_surf,
    cell_m,
    is_valley_mission: bool,
    goal_tolerance: GoalTolerance = GOAL_TOLERANCE,
    max_expansions: int = 15000,
    max_time_s: float = 20.0,
):
    t0 = time.perf_counter()
    rows, cols = cost_surf.shape
    goal_rc = terrain.xy_to_rowcol(goal.x_m, goal.y_m)
    
    if is_valley_mission:
        h_grid = compute_dijkstra_2d(cost_surf, cell_m, goal_rc)
    else:
        h_grid = None

    def heuristic_fn(p: PhysicalPose):
        if is_valley_mission:
            r, c = terrain.xy_to_rowcol(p.x_m, p.y_m)
            r = max(0, min(rows - 1, r))
            c = max(0, min(cols - 1, c))
            h_2d = float(h_grid[r, c])
            local_elev = elev_grid[r, c]
            d_to_goal = math.hypot(p.x_m - goal.x_m, p.y_m - goal.y_m)
            z_req = max(local_elev + 120.0, goal.z_msl_m - 0.075 * d_to_goal)
            dz = max(0.0, abs(p.z_msl_m - z_req) - goal_tolerance.altitude_m)
            return math.sqrt(h_2d * h_2d + dz * dz)
        else:
            dx = p.x_m - goal.x_m
            dy = p.y_m - goal.y_m
            dz = max(0.0, abs(p.z_msl_m - goal.z_msl_m) - goal_tolerance.altitude_m)
            return math.sqrt(dx * dx + dy * dy + dz * dz)

    def edge_cost_fn(tr: PhysicalTrajectory):
        l3d = _trajectory_3d_length(tr)
        if not is_valley_mission:
            return l3d
        ep = tr.end_pose
        r, c = terrain.xy_to_rowcol(ep.x_m, ep.y_m)
        r = max(0, min(rows - 1, r))
        c = max(0, min(cols - 1, c))
        return l3d * cost_surf[r, c]

    start_key = search_key_for_pose(start, CONFIG)
    start_node = PoseSearchNode(0, start_key, start, 0.0, None, None, None)
    active = {start_key: start_node}
    all_nodes = {0: start_node}

    pareto_frontier = defaultdict(list)
    start_xyh = (start_key.x_bin, start_key.y_bin, start_key.heading_bin)
    pareto_frontier[start_xyh].append((0.0, abs(start.z_msl_m - goal.z_msl_m), start_key))

    f_0 = 1.01 * heuristic_fn(start)
    counter = itertools.count(1)
    open_heap = [(f_0, 0, 0)]
    expanded_ids = set()
    expanded = 0
    generated = 0
    peak_open = 1
    goal_node = None
    status = "no_path"

    while open_heap and expanded < max_expansions:
        if time.perf_counter() - t0 >= max_time_s:
            status = "timeout"
            break
        peak_open = max(peak_open, len(open_heap))
        _, _, node_id = heapq.heappop(open_heap)
        node = all_nodes[node_id]
        if active.get(node.key) is not node or node_id in expanded_ids:
            continue
        expanded_ids.add(node_id)
        expanded += 1

        if pose_in_goal(node.end_pose, goal, goal_tolerance):
            goal_node = node
            status = "success"
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
            cand_z_dist = abs(end_p.z_msl_m - goal.z_msl_m)

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
            f_score = cand_g + 1.01 * heuristic_fn(end_p)
            heapq.heappush(open_heap, (f_score, nid, nid))

    dt = time.perf_counter() - t0
    path_len = 0.0
    agls = []
    safety_checks = []
    primitives_list = []

    if goal_node is not None:
        cur = goal_node
        nodes_rev = []
        while cur is not None:
            nodes_rev.append(cur)
            cur = all_nodes.get(cur.parent_node_id)
        nodes = list(reversed(nodes_rev))

        for item in nodes[1:]:
            primitives_list.append(item.incoming_primitive)
            tr = item.incoming_trajectory
            path_len += _trajectory_3d_length(tr)
            s_val = evaluate_physical_trajectory_safety(
                tr, terrain, CONFIG.min_agl_m, CONFIG.primitive_sample_spacing_m,
                planning_bounds=terrain.roi.bounds, lateral_buffer_m=CONFIG.lateral_buffer_m,
            )
            safety_checks.append(s_val)
            for s in tr.samples:
                elev = terrain.query(s.x_m, s.y_m).elevation
                agls.append(s.z_msl_m - elev)

    min_agl = min(agls) if agls else float("nan")
    mean_agl = sum(agls) / len(agls) if agls else float("nan")
    median_agl = float(np.median(agls)) if agls else float("nan")
    all_safe = all(s.is_safe for s in safety_checks) if safety_checks else False

    xy_err = math.hypot(goal_node.end_pose.x_m - goal.x_m, goal_node.end_pose.y_m - goal.y_m) if goal_node else float("nan")
    z_err = abs(goal_node.end_pose.z_msl_m - goal.z_msl_m) if goal_node else float("nan")

    return {
        "status": "FOUND" if goal_node else "FAIL",
        "expansions": expanded,
        "generated": generated,
        "peak_open": peak_open,
        "runtime_s": dt,
        "path_length_m": path_len,
        "min_agl_m": min_agl,
        "mean_agl_m": mean_agl,
        "median_agl_m": median_agl,
        "goal_xy_error_m": xy_err,
        "goal_z_error_m": z_err,
        "all_safe": all_safe,
        "primitive_count": len(primitives_list),
        "primitive_counts": dict(Counter(primitives_list)),
    }

def main():
    roi = load_roi(CONFIG)
    cache = load_terrain_cache(CACHE_DIR, roi, SOURCE_DEM_PATH)
    terrain = build_terrain_query_from_cache(cache, roi, FACTOR)
    profile = load_aircraft_profile(PROFILE_PATH)
    envelope = FixedWingKinematicEnvelope()
    infl_cache = TerrainInfluenceCache(terrain)

    elev_grid, cost_surf, cell_m = build_cost_surface(terrain)

    missions = {}

    # Canonical A - F
    missions_ab = mission_definitions(cache)
    for m_name in ["A_easy_open", "B_relief_affected"]:
        spec = missions_ab[m_name]
        sx, sy = terrain.rowcol_to_xy(*spec["start_rc"])
        gx, gy = terrain.rowcol_to_xy(*spec["goal_rc"])
        missions[f"can_{m_name}"] = {
            "start": PhysicalPose(sx, sy, spec["start_z"], navigation_bearing_deg(sx, sy, gx, gy)),
            "goal": GoalPose(gx, gy, spec["goal_z"]),
            "tol": GOAL_TOLERANCE,
            "is_valley": False,
        }

    for m_name in ["C_far_south_3km", "D_far_east_3km", "E_long_descent_9_3km", "F_turn_required_diagonal_2_3km"]:
        spec = FAR_MISSIONS[m_name]
        sx, sy = terrain.rowcol_to_xy(*spec["start_rc"])
        gx, gy = terrain.rowcol_to_xy(*spec["goal_rc"])
        missions[f"can_{m_name}"] = {
            "start": PhysicalPose(sx, sy, spec["z_msl_m"], navigation_bearing_deg(sx, sy, gx, gy)),
            "goal": GoalPose(gx, gy, spec.get("goal_z_msl_m", spec["z_msl_m"])),
            "tol": GOAL_TOLERANCE,
            "is_valley": False,
        }

    # 1. Western Valley Descent
    sx_v, sy_v = terrain.rowcol_to_xy(75, 5)
    gx_v, gy_v = terrain.rowcol_to_xy(20, 5)
    elev_sv = float(terrain.query(sx_v, sy_v).elevation)
    elev_gv = float(terrain.query(gx_v, gy_v).elevation)
    missions["1_west_valley_descent"] = {
        "start": PhysicalPose(sx_v, sy_v, elev_sv + 120.0, navigation_bearing_deg(sx_v, sy_v, gx_v, gy_v)),
        "goal": GoalPose(gx_v, gy_v, elev_gv + 140.0),
        "tol": GoalTolerance(xy_m=120.0, altitude_m=30.0),
        "is_valley": True,
    }

    # 2. Western Valley Reverse Climb
    missions["2_west_valley_climb"] = {
        "start": PhysicalPose(gx_v, gy_v, elev_gv + 120.0, navigation_bearing_deg(gx_v, gy_v, sx_v, sy_v)),
        "goal": GoalPose(sx_v, sy_v, elev_sv + 140.0),
        "tol": GoalTolerance(xy_m=120.0, altitude_m=30.0),
        "is_valley": True,
    }

    print("=" * 125)
    print(f"{'Mission Name':<35s} | {'Status':<6s} | {'Exp':<6s} | {'Time (s)':<8s} | {'Path (m)':<9s} | {'Min AGL':<8s} | {'Mean AGL':<9s} | {'Safety':<6s}")
    print("=" * 125)

    for m_name, m_spec in missions.items():
        res = run_consistent_search(
            m_spec["start"], m_spec["goal"],
            terrain, envelope, infl_cache,
            elev_grid, cost_surf, cell_m,
            is_valley_mission=m_spec["is_valley"],
            goal_tolerance=m_spec["tol"],
        )
        status_str = res["status"]
        exp_str = str(res["expansions"])
        time_str = f"{res['runtime_s']:.2f}"
        len_str = f"{res['path_length_m']:.1f}" if res["status"] == "FOUND" else "N/A"
        min_agl_str = f"{res['min_agl_m']:.1f}" if res["status"] == "FOUND" else "N/A"
        mean_agl_str = f"{res['mean_agl_m']:.1f}" if res["status"] == "FOUND" else "N/A"
        safe_str = "PASS" if res["all_safe"] else "FAIL"

        print(f"{m_name:<35s} | {status_str:<6s} | {exp_str:<6s} | {time_str:<8s} | {len_str:<9s} | {min_agl_str:<8s} | {mean_agl_str:<9s} | {safe_str:<6s}")

if __name__ == "__main__":
    main()
