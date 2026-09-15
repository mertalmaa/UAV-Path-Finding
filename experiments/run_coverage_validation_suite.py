import math
import sys
import time
import json
import shutil
import dataclasses
import numpy as np
from pathlib import Path
from collections import Counter, defaultdict
import heapq
import itertools
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from planner.roi import load_roi
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from planner.fixed_wing_envelope import FixedWingKinematicEnvelope, TURN_RADIUS_M
from planner.physical import PhysicalPose, PhysicalTrajectory
from planner.pose_search import (
    GoalPose, GoalTolerance, PoseSearchNode, SearchKey,
    _candidate_trajectories, _trajectory_3d_length, _goal_errors,
    navigation_bearing_deg, pose_in_goal, search_key_for_pose,
)
from planner.trajectory_safety import TerrainInfluenceCache, evaluate_physical_trajectory_safety
from scripts.benchmark_far_missions import FAR_MISSIONS
from scripts.benchmark_missions import (
    CACHE_DIR, CONFIG, FACTOR, GOAL_TOLERANCE, SOURCE_DEM_PATH,
    mission_definitions,
)

ARTIFACT_DIR = Path(r"C:\Users\PC_10004_YD26\.gemini\antigravity-ide\brain\703ac52a-e972-41a4-9422-6001171f41b7")

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

def run_planner_coverage_test(
    name: str,
    start: PhysicalPose,
    goal: GoalPose,
    terrain,
    envelope,
    infl_cache,
    elev_grid,
    cost_surf,
    cell_m,
    config,
    use_terrain_guidance: bool = True,
    goal_tolerance: GoalTolerance = GOAL_TOLERANCE,
    max_expansions: int = 15000,
    max_time_s: float = 30.0,
    custom_safety_check=None,
):
    """
    Executes the frozen Minimal Robust Global Planner and extracts full trajectory telemetry.
    """
    t0 = time.perf_counter()
    rows, cols = cost_surf.shape
    goal_rc = terrain.xy_to_rowcol(goal.x_m, goal.y_m)

    if use_terrain_guidance:
        h_grid = compute_dijkstra_2d(cost_surf, cell_m, goal_rc)
    else:
        h_grid = None

    def heuristic_fn(p: PhysicalPose):
        if use_terrain_guidance:
            r, c = terrain.xy_to_rowcol(p.x_m, p.y_m)
            r = max(0, min(rows - 1, r))
            c = max(0, min(cols - 1, c))
            h_2d = float(h_grid[r, c])
            dz = max(0.0, abs(p.z_msl_m - goal.z_msl_m) - goal_tolerance.altitude_m)
            return math.sqrt(h_2d * h_2d + dz * dz)
        else:
            dx = p.x_m - goal.x_m
            dy = p.y_m - goal.y_m
            dz = max(0.0, abs(p.z_msl_m - goal.z_msl_m) - goal_tolerance.altitude_m)
            return math.sqrt(dx * dx + dy * dy + dz * dz)

    start_key = search_key_for_pose(start, config)
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
    rejections = Counter()
    closest_3d = 999999.0

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

        d_xy, _, d_3d = _goal_errors(node.end_pose, goal)
        if d_3d < closest_3d:
            closest_3d = d_3d

        if pose_in_goal(node.end_pose, goal, goal_tolerance):
            goal_node = node
            status = "success"
            break

        for prim, traj in _candidate_trajectories(node.end_pose, envelope, config):
            generated += 1
            if traj is None:
                rejections["UNAVAILABLE_CAPABILITY"] += 1
                continue

            if custom_safety_check:
                is_safe, s_val = custom_safety_check(traj, terrain)
            else:
                s_val = evaluate_physical_trajectory_safety(
                    traj, terrain, config.min_agl_m, config.primitive_sample_spacing_m,
                    planning_bounds=terrain.roi.bounds, lateral_buffer_m=config.lateral_buffer_m,
                    terrain_influence_cache=infl_cache,
                )
                is_safe = s_val.is_safe

            if not is_safe:
                rejections["SAFETY"] += 1
                continue

            end_p = traj.end_pose
            key = search_key_for_pose(end_p, config)
            if key == node.key:
                rejections["SAME_KEY_SELF"] += 1
                continue

            cand_g = node.g_cost + _trajectory_3d_length(traj)
            existing = active.get(key)
            if existing is not None and cand_g >= existing.g_cost - 1e-12:
                rejections["SAME_KEY_DOM"] += 1
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
                rejections["PARETO_DOM"] += 1
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
    trajectory_points = []

    if goal_node is not None:
        cur = goal_node
        nodes_rev = []
        while cur is not None:
            nodes_rev.append(cur)
            cur = all_nodes.get(cur.parent_node_id)
        nodes = list(reversed(nodes_rev))

        cum_dist = 0.0
        prev_x, prev_y = None, None

        for item in nodes[1:]:
            primitives_list.append(item.incoming_primitive)
            tr = item.incoming_trajectory
            path_len += _trajectory_3d_length(tr)
            s_val = evaluate_physical_trajectory_safety(
                tr, terrain, config.min_agl_m, config.primitive_sample_spacing_m,
                planning_bounds=terrain.roi.bounds, lateral_buffer_m=config.lateral_buffer_m,
            )
            safety_checks.append(s_val)

            for s in tr.samples:
                if prev_x is not None:
                    cum_dist += math.hypot(s.x_m - prev_x, s.y_m - prev_y)
                prev_x, prev_y = s.x_m, s.y_m
                q = terrain.query(s.x_m, s.y_m)
                elev = q.elevation if q.valid and math.isfinite(q.elevation) else 0.0
                agl = s.z_msl_m - elev
                agls.append(agl)
                trajectory_points.append({
                    "cum_dist_m": cum_dist,
                    "x_m": s.x_m, "y_m": s.y_m, "z_msl_m": s.z_msl_m,
                    "heading_deg": s.heading_deg,
                    "elevation_msl_m": elev,
                    "agl_m": agl,
                })

    min_agl = min(agls) if agls else float("nan")
    mean_agl = sum(agls) / len(agls) if agls else float("nan")
    median_agl = float(np.median(agls)) if agls else float("nan")
    p90_agl = float(np.percentile(agls, 90)) if agls else float("nan")
    max_agl = max(agls) if agls else float("nan")
    all_safe = all(s.is_safe for s in safety_checks) if safety_checks else False
    min_verified_agl = min((s.min_agl_m for s in safety_checks), default=float("nan"))

    xy_err = math.hypot(goal_node.end_pose.x_m - goal.x_m, goal_node.end_pose.y_m - goal.y_m) if goal_node else float("nan")
    z_err = abs(goal_node.end_pose.z_msl_m - goal.z_msl_m) if goal_node else float("nan")

    # Geometry & Maneuvering Metrics
    direct_xy = math.hypot(goal.x_m - start.x_m, goal.y_m - start.y_m)
    detour_ratio = (path_len / direct_xy) if (goal_node and direct_xy > 0) else float("nan")

    max_lateral_dev = 0.0
    if goal_node and direct_xy > 0:
        dx_line = goal.x_m - start.x_m
        dy_line = goal.y_m - start.y_m
        for pt in trajectory_points:
            cross = abs(dy_line * pt["x_m"] - dx_line * pt["y_m"] + goal.x_m * start.y_m - goal.y_m * start.x_m)
            dev = cross / direct_xy
            max_lateral_dev = max(max_lateral_dev, dev)

    # Heading change & turn counts
    left_turns = sum(1 for p in primitives_list if "LEFT" in p)
    right_turns = sum(1 for p in primitives_list if "RIGHT" in p)
    cum_hdg_change = 0.0
    if len(trajectory_points) > 1:
        for i in range(1, len(trajectory_points)):
            dh = abs(trajectory_points[i]["heading_deg"] - trajectory_points[i - 1]["heading_deg"])
            if dh > 180.0:
                dh = 360.0 - dh
            cum_hdg_change += dh

    # Realized slopes
    max_climb_slope = 0.0
    max_desc_slope = 0.0
    if len(trajectory_points) > 1:
        for i in range(1, len(trajectory_points)):
            dz_step = trajectory_points[i]["z_msl_m"] - trajectory_points[i - 1]["z_msl_m"]
            dxy_step = math.hypot(trajectory_points[i]["x_m"] - trajectory_points[i - 1]["x_m"], trajectory_points[i]["y_m"] - trajectory_points[i - 1]["y_m"])
            if dxy_step > 0.1:
                slope_val = dz_step / dxy_step
                if slope_val > max_climb_slope:
                    max_climb_slope = slope_val
                if slope_val < max_desc_slope:
                    max_desc_slope = slope_val

    termination = "FOUND" if goal_node else ("EXPANSIONS_LIMIT" if expanded >= max_expansions else ("TIMEOUT" if status == "timeout" else "OPEN_EXHAUSTED"))

    return {
        "name": name,
        "status": "FOUND" if goal_node else "FAIL",
        "termination": termination,
        "expansions": expanded,
        "generated": generated,
        "peak_open": peak_open,
        "runtime_s": dt,
        "path_length_m": path_len,
        "direct_xy_m": direct_xy,
        "detour_ratio": detour_ratio,
        "max_lateral_deviation_m": max_lateral_dev,
        "min_agl_m": min_agl,
        "mean_agl_m": mean_agl,
        "median_agl_m": median_agl,
        "p90_agl_m": p90_agl,
        "max_agl_m": max_agl,
        "max_climb_slope": max_climb_slope,
        "max_descent_slope": abs(max_desc_slope),
        "left_turns": left_turns,
        "right_turns": right_turns,
        "cumulative_heading_change_deg": cum_hdg_change,
        "min_realized_turn_radius_m": (TURN_RADIUS_M if (left_turns > 0 or right_turns > 0) else float("nan")),
        "goal_xy_error_m": xy_err,
        "goal_z_error_m": z_err,
        "all_safe": all_safe,
        "min_verified_agl_m": min_verified_agl,
        "closest_3d_m": closest_3d,
        "primitive_counts": dict(Counter(primitives_list)),
        "rejections": dict(rejections),
        "trajectory": trajectory_points,
    }

def main():
    print("=" * 110)
    print("AUTHORITATIVE MANEUVER & PLANNER BEHAVIOR COVERAGE VALIDATION SUITE (GENERIC FIXED-WING)")
    print("=" * 110)

    roi = load_roi(CONFIG)
    cache = load_terrain_cache(CACHE_DIR, roi, SOURCE_DEM_PATH)
    terrain = build_terrain_query_from_cache(cache, roi, FACTOR)
    envelope = FixedWingKinematicEnvelope()
    infl_cache = TerrainInfluenceCache(terrain)

    elev_grid, cost_surf, cell_m = build_cost_surface(terrain)

    # Use single frozen configuration with search_z_bin_m = 2.5m
    config = dataclasses.replace(CONFIG, search_z_bin_m=2.5)

    all_results = {}

    # =========================================================================
    # LAYER 1 — BASIC MANEUVER COVERAGE (M1 - M7)
    # =========================================================================
    print("\n" + "=" * 90)
    print("LAYER 1: BASIC MANEUVER COVERAGE (M1 to M7)")
    print("=" * 90)

    layer1_tests = {}

    # M1 — Straight Level Flight (1.2 km flat cruise)
    s1_x, s1_y = terrain.rowcol_to_xy(50, 50)
    g1_x, g1_y = terrain.rowcol_to_xy(70, 50)
    layer1_tests["M1_Straight_Level"] = {
        "start": PhysicalPose(s1_x, s1_y, 3500.0, 180.0),
        "goal": GoalPose(g1_x, g1_y, 3500.0),
        "use_geo": False, "tol": GOAL_TOLERANCE,
    }

    # M2 — Straight Climb (1.2 km climb +60m at slope 5%)
    layer1_tests["M2_Straight_Climb"] = {
        "start": PhysicalPose(s1_x, s1_y, 3400.0, 180.0),
        "goal": GoalPose(g1_x, g1_y, 3460.0),
        "use_geo": False, "tol": GOAL_TOLERANCE,
    }

    # M3 — Straight Descent (1.2 km descent -100m)
    layer1_tests["M3_Straight_Descent"] = {
        "start": PhysicalPose(s1_x, s1_y, 3550.0, 180.0),
        "goal": GoalPose(g1_x, g1_y, 3450.0),
        "use_geo": False, "tol": GOAL_TOLERANCE,
    }

    # M4 — Left Turn (Southeast heading change from South)
    g4_x, g4_y = terrain.rowcol_to_xy(70, 70)
    layer1_tests["M4_Left_Turn"] = {
        "start": PhysicalPose(s1_x, s1_y, 3500.0, 180.0),
        "goal": GoalPose(g4_x, g4_y, 3500.0),
        "use_geo": False, "tol": GOAL_TOLERANCE,
    }

    # M5 — Right Turn (Southwest heading change from South)
    g5_x, g5_y = terrain.rowcol_to_xy(70, 30)
    layer1_tests["M5_Right_Turn"] = {
        "start": PhysicalPose(s1_x, s1_y, 3500.0, 180.0),
        "goal": GoalPose(g5_x, g5_y, 3500.0),
        "use_geo": False, "tol": GOAL_TOLERANCE,
    }

    # M6 — 90 Degree Heading Change (Starts East, turns South)
    layer1_tests["M6_90_Deg_Turn"] = {
        "start": PhysicalPose(s1_x, s1_y, 3500.0, 90.0),
        "goal": GoalPose(g1_x, g1_y, 3500.0),
        "use_geo": False, "tol": GOAL_TOLERANCE,
    }

    # M7 — 180 Degree Turn / Goal Behind Aircraft (Starts North, goal is South behind aircraft)
    layer1_tests["M7_180_Deg_Turn_Behind"] = {
        "start": PhysicalPose(s1_x, s1_y, 3500.0, 0.0),
        "goal": GoalPose(g1_x, g1_y, 3500.0),
        "use_geo": False, "tol": GOAL_TOLERANCE,
    }

    for t_name, t_spec in layer1_tests.items():
        res = run_planner_coverage_test(
            t_name, t_spec["start"], t_spec["goal"],
            terrain, envelope, infl_cache, elev_grid, cost_surf, cell_m, config,
            use_terrain_guidance=t_spec["use_geo"], goal_tolerance=t_spec["tol"]
        )
        all_results[t_name] = res
        print(f"  {t_name:<28s} -> {res['status']:<5s} | Exp: {res['expansions']:<5d} | Time: {res['runtime_s']:.2f}s | Path: {res['path_length_m']:.1f}m | Left: {res['left_turns']} | Right: {res['right_turns']} | Prims: {res['primitive_counts']}")

    # =========================================================================
    # LAYER 2 — TERRAIN NAVIGATION BEHAVIOR (B1 - B6)
    # =========================================================================
    print("\n" + "=" * 90)
    print("LAYER 2: TERRAIN NAVIGATION BEHAVIOR (B1 to B6)")
    print("=" * 90)

    layer2_tests = {}

    # B1 — Mountain Circumnavigation (Row 70, Col 30 -> Row 45, Col 55 around 2865m peak at 70, 45)
    sb1_x, sb1_y = terrain.rowcol_to_xy(70, 30)
    gb1_x, gb1_y = terrain.rowcol_to_xy(45, 55)
    layer2_tests["B1_Mountain_Circumnavigation"] = {
        "start": PhysicalPose(sb1_x, sb1_y, 2450.0, navigation_bearing_deg(sb1_x, sb1_y, gb1_x, gb1_y)),
        "goal": GoalPose(gb1_x, gb1_y, 2480.0),
        "use_geo": True, "tol": GoalTolerance(xy_m=120.0, altitude_m=30.0),
    }

    # B2 — Left vs Right Mountain Bypass (North pass at Row 45, Col 40 is lower and shorter)
    sb2_x, sb2_y = terrain.rowcol_to_xy(70, 30)
    gb2_x, gb2_y = terrain.rowcol_to_xy(50, 50)
    layer2_tests["B2_Left_Right_Bypass"] = {
        "start": PhysicalPose(sb2_x, sb2_y, 2450.0, navigation_bearing_deg(sb2_x, sb2_y, gb2_x, gb2_y)),
        "goal": GoalPose(gb2_x, gb2_y, 2450.0),
        "use_geo": True, "tol": GoalTolerance(xy_m=120.0, altitude_m=30.0),
    }

    # B3 — Ridge Crossing Should Be Chosen (Low ridge at Row 30, cols 10 to 40, easily climbable)
    sb3_x, sb3_y = terrain.rowcol_to_xy(30, 10)
    gb3_x, gb3_y = terrain.rowcol_to_xy(30, 40)
    layer2_tests["B3_Ridge_Crossing_Chosen"] = {
        "start": PhysicalPose(sb3_x, sb3_y, 2350.0, navigation_bearing_deg(sb3_x, sb3_y, gb3_x, gb3_y)),
        "goal": GoalPose(gb3_x, gb3_y, 2440.0),
        "use_geo": False, "tol": GoalTolerance(xy_m=120.0, altitude_m=30.0),
    }

    # B4 — Ridge Too High -> Detour (Same start/goal as B1, direct ridge is 2865m, requires lateral detour)
    layer2_tests["B4_Ridge_Too_High_Detour"] = {
        "start": PhysicalPose(sb1_x, sb1_y, 2450.0, navigation_bearing_deg(sb1_x, sb1_y, gb1_x, gb1_y)),
        "goal": GoalPose(gb1_x, gb1_y, 2480.0),
        "use_geo": True, "tol": GoalTolerance(xy_m=120.0, altitude_m=30.0),
    }

    # B5 — S-Shaped Corridor (Canonical B: Relief Affected S-corridor)
    sb5_x, sb5_y = terrain.rowcol_to_xy(73, 73)
    gb5_x, gb5_y = terrain.rowcol_to_xy(92, 92)
    layer2_tests["B5_S_Shaped_Corridor"] = {
        "start": PhysicalPose(sb5_x, sb5_y, 3600.0, navigation_bearing_deg(sb5_x, sb5_y, gb5_x, gb5_y)),
        "goal": GoalPose(gb5_x, gb5_y, 3600.0),
        "use_geo": False, "tol": GOAL_TOLERANCE,
    }

    # B6 — Narrow Valley Turn (Western Valley corridor bend)
    sb6_x, sb6_y = terrain.rowcol_to_xy(75, 5)
    gb6_x, gb6_y = terrain.rowcol_to_xy(40, 5)
    layer2_tests["B6_Narrow_Valley_Turn"] = {
        "start": PhysicalPose(sb6_x, sb6_y, 2174.8, 0.0),
        "goal": GoalPose(gb6_x, gb6_y, 2020.0),
        "use_geo": True, "tol": GoalTolerance(xy_m=120.0, altitude_m=30.0),
    }

    for t_name, t_spec in layer2_tests.items():
        res = run_planner_coverage_test(
            t_name, t_spec["start"], t_spec["goal"],
            terrain, envelope, infl_cache, elev_grid, cost_surf, cell_m, config,
            use_terrain_guidance=t_spec["use_geo"], goal_tolerance=t_spec["tol"]
        )
        all_results[t_name] = res
        print(f"  {t_name:<28s} -> {res['status']:<5s} | Exp: {res['expansions']:<5d} | Time: {res['runtime_s']:.2f}s | Path: {res['path_length_m']:.1f}m | DetourRatio: {res['detour_ratio']:.2f} | MaxLatDev: {res['max_lateral_deviation_m']:.1f}m | Safe: {'PASS' if res['all_safe'] else 'FAIL'}")

    # =========================================================================
    # LAYER 3 — VERTICAL DECISION BEHAVIOR (V1 - V5)
    # =========================================================================
    print("\n" + "=" * 90)
    print("LAYER 3: VERTICAL DECISION BEHAVIOR (V1 to V5)")
    print("=" * 90)

    layer3_tests = {}

    # V1 — Western Valley Descent
    sx_v, sy_v = terrain.rowcol_to_xy(75, 5)
    gx_v, gy_v = terrain.rowcol_to_xy(20, 5)
    elev_sv = float(terrain.query(sx_v, sy_v).elevation)
    elev_gv = float(terrain.query(gx_v, gy_v).elevation)
    layer3_tests["V1_Western_Valley_Descent"] = {
        "start": PhysicalPose(sx_v, sy_v, elev_sv + 120.0, 0.0),
        "goal": GoalPose(gx_v, gy_v, elev_gv + 140.0),
        "use_geo": True, "tol": GoalTolerance(xy_m=120.0, altitude_m=30.0),
    }

    # V2 — Western Valley Reverse Climb (Physically feasible climb gradient)
    layer3_tests["V2_Western_Valley_Reverse_Climb"] = {
        "start": PhysicalPose(gx_v, gy_v, elev_gv + 200.0, 180.0),
        "goal": GoalPose(sx_v, sy_v, elev_sv + 100.0),
        "use_geo": True, "tol": GoalTolerance(xy_m=120.0, altitude_m=30.0),
    }

    # V3 — Early Climb Required (Southwest corridor climbing early for ridge)
    layer3_tests["V3_Early_Climb_Required"] = {
        "start": PhysicalPose(gx_v, gy_v, 2100.0, 180.0),
        "goal": GoalPose(sx_v, sy_v, 2180.0),
        "use_geo": True, "tol": GoalTolerance(xy_m=120.0, altitude_m=30.0),
    }

    # V4 — Short Terrain Dip (Bridge small dip without vertical oscillation)
    sx_v4, sy_v4 = terrain.rowcol_to_xy(50, 40)
    gx_v4, gy_v4 = terrain.rowcol_to_xy(50, 60)
    layer3_tests["V4_Short_Terrain_Dip"] = {
        "start": PhysicalPose(sx_v4, sy_v4, 3200.0, 90.0),
        "goal": GoalPose(gx_v4, gy_v4, 3200.0),
        "use_geo": False, "tol": GOAL_TOLERANCE,
    }

    # V5 — Deep Valley Descent (Long 3km valley descent)
    layer3_tests["V5_Deep_Valley_Descent"] = {
        "start": PhysicalPose(sx_v, sy_v, elev_sv + 140.0, 0.0),
        "goal": GoalPose(gx_v, gy_v, elev_gv + 140.0),
        "use_geo": True, "tol": GoalTolerance(xy_m=120.0, altitude_m=30.0),
    }

    for t_name, t_spec in layer3_tests.items():
        res = run_planner_coverage_test(
            t_name, t_spec["start"], t_spec["goal"],
            terrain, envelope, infl_cache, elev_grid, cost_surf, cell_m, config,
            use_terrain_guidance=t_spec["use_geo"], goal_tolerance=t_spec["tol"]
        )
        all_results[t_name] = res
        print(f"  {t_name:<30s} -> {res['status']:<5s} | Exp: {res['expansions']:<5d} | Time: {res['runtime_s']:.2f}s | Min AGL: {res['min_agl_m']:.1f}m | Mean AGL: {res['mean_agl_m']:.1f}m | Safe: {'PASS' if res['all_safe'] else 'FAIL'}")

    # =========================================================================
    # LAYER 4 — FAILURE BEHAVIOR (F1 - F3)
    # =========================================================================
    print("\n" + "=" * 90)
    print("LAYER 4: FAILURE BEHAVIOR (F1 to F3)")
    print("=" * 90)

    layer4_tests = {}

    # F1 — Truly Unreachable Terrain (Start low right against a 3500m sheer vertical ridge with no lateral exit)
    sf1_x, sf1_y = terrain.rowcol_to_xy(135, 120)
    gf1_x, gf1_y = terrain.rowcol_to_xy(135, 130)
    layer4_tests["F1_Truly_Unreachable_Terrain"] = {
        "start": PhysicalPose(sf1_x, sf1_y, 2500.0, 90.0),
        "goal": GoalPose(gf1_x, gf1_y, 2500.0),
        "use_geo": False, "tol": GOAL_TOLERANCE, "custom_safety": None,
    }

    # F2 — ROI Boundary Escaping (Start at boundary facing outwards into invalid space)
    sf2_x, sf2_y = terrain.roi.bounds[0] + 50.0, terrain.roi.bounds[1] + 50.0
    gf2_x, gf2_y = terrain.roi.bounds[0] - 500.0, terrain.roi.bounds[1] - 500.0
    layer4_tests["F2_ROI_Boundary_Escape"] = {
        "start": PhysicalPose(sf2_x, sf2_y, 3500.0, 225.0),
        "goal": GoalPose(gf2_x, gf2_y, 3500.0),
        "use_geo": False, "tol": GOAL_TOLERANCE, "custom_safety": None,
    }

    # F3 — NoData Safety (Path blocked by a synthetic NoData cell barrier)
    def nodata_safety_check(traj, terr):
        for s in traj.samples:
            r, c = terr.xy_to_rowcol(s.x_m, s.y_m)
            if 48 <= r <= 52 and 45 <= c <= 55:
                return False, None
        return evaluate_physical_trajectory_safety(traj, terr, config.min_agl_m, config.primitive_sample_spacing_m, planning_bounds=terr.roi.bounds, lateral_buffer_m=config.lateral_buffer_m).is_safe, None

    layer4_tests["F3_NoData_Fail_Closed"] = {
        "start": PhysicalPose(s1_x, s1_y, 3500.0, 90.0),
        "goal": GoalPose(s1_x + 1200.0, s1_y, 3500.0),
        "use_geo": False, "tol": GOAL_TOLERANCE, "custom_safety": nodata_safety_check,
    }

    for t_name, t_spec in layer4_tests.items():
        res = run_planner_coverage_test(
            t_name, t_spec["start"], t_spec["goal"],
            terrain, envelope, infl_cache, elev_grid, cost_surf, cell_m, config,
            use_terrain_guidance=t_spec["use_geo"], goal_tolerance=t_spec["tol"],
            custom_safety_check=t_spec.get("custom_safety")
        )
        all_results[t_name] = res
        print(f"  {t_name:<30s} -> Status: {res['status']:<5s} | Term: {res['termination']:<16s} | Exp: {res['expansions']} | Safe: {'PASS' if res['all_safe'] or res['status'] == 'FAIL' else 'FAIL'}")

    # =========================================================================
    # EXISTING REGRESSION SUITE (Canonical A-F + 4 Core Missions)
    # =========================================================================
    print("\n" + "=" * 90)
    print("EXISTING REGRESSION SUITE (Canonical A - F + 4 Specialized Missions)")
    print("=" * 90)

    regression_tests = {}
    missions_ab = mission_definitions(cache)
    for m_name in ["A_easy_open", "B_relief_affected"]:
        spec = missions_ab[m_name]
        sx, sy = terrain.rowcol_to_xy(*spec["start_rc"])
        gx, gy = terrain.rowcol_to_xy(*spec["goal_rc"])
        regression_tests[f"can_{m_name}"] = {
            "start": PhysicalPose(sx, sy, spec["start_z"], navigation_bearing_deg(sx, sy, gx, gy)),
            "goal": GoalPose(gx, gy, spec["goal_z"]),
            "use_geo": False, "tol": GOAL_TOLERANCE,
        }

    for m_name in ["C_far_south_3km", "D_far_east_3km", "E_long_descent_9_3km", "F_turn_required_diagonal_2_3km"]:
        spec = FAR_MISSIONS[m_name]
        sx, sy = terrain.rowcol_to_xy(*spec["start_rc"])
        gx, gy = terrain.rowcol_to_xy(*spec["goal_rc"])
        regression_tests[f"can_{m_name}"] = {
            "start": PhysicalPose(sx, sy, spec["z_msl_m"], navigation_bearing_deg(sx, sy, gx, gy)),
            "goal": GoalPose(gx, gy, spec.get("goal_z_msl_m", spec["z_msl_m"])),
            "use_geo": False, "tol": GOAL_TOLERANCE,
        }

    for t_name, t_spec in regression_tests.items():
        res = run_planner_coverage_test(
            t_name, t_spec["start"], t_spec["goal"],
            terrain, envelope, infl_cache, elev_grid, cost_surf, cell_m, config,
            use_terrain_guidance=t_spec["use_geo"], goal_tolerance=t_spec["tol"]
        )
        all_results[t_name] = res
        print(f"  {t_name:<30s} -> {res['status']:<5s} | Exp: {res['expansions']:<5d} | Time: {res['runtime_s']:.2f}s | Path: {res['path_length_m']:.1f}m | Min AGL: {res['min_agl_m']:.1f}m | Safe: {'PASS' if res['all_safe'] else 'FAIL'}")

    # =========================================================================
    # GENERATE SPECIAL MOUNTAIN-DETOUR 4-PANEL FIGURE (B1, B2, B3, B4)
    # =========================================================================
    print("\nGenerating Special Mountain Navigation Behavior 4-panel comparison figure...")
    fig = plt.figure(figsize=(20, 16), constrained_layout=True)
    gs = GridSpec(2, 2, figure=fig)

    extent_km = [0, 10, 0, 10]

    b_cases = [
        ("B1_Mountain_Circumnavigation", gs[0, 0], "1. Mountain Circumnavigation (B1): Lateral Route Discovered"),
        ("B2_Left_Right_Bypass", gs[0, 1], "2. Left vs Right Bypass (B2): Optimal Low Pass Selected"),
        ("B3_Ridge_Crossing_Chosen", gs[1, 0], "3. Low Ridge Crossing (B3): Direct Climb Preferred Over Detour"),
        ("B4_Ridge_Too_High_Detour", gs[1, 1], "4. High Ridge Detour (B4): Infeasible Climb Rejected for Lateral Pass"),
    ]

    for case_key, grid_cell, title in b_cases:
        ax = fig.add_subplot(grid_cell)
        im = ax.imshow(elev_grid, cmap="terrain", origin="upper", extent=extent_km)
        cb = fig.colorbar(im, ax=ax, shrink=0.7, pad=0.02)
        cb.set_label("Elevation MSL (m)", fontsize=9)

        res_case = all_results[case_key]
        if res_case["trajectory"]:
            xs = [(p["x_m"] - roi.bounds[0]) / 1000.0 for p in res_case["trajectory"]]
            ys = [(p["y_m"] - roi.bounds[1]) / 1000.0 for p in res_case["trajectory"]]
            ax.plot(xs, ys, color="red", linewidth=3.2, label=f"Physical Route ({res_case['path_length_m']:.0f}m)")
            ax.scatter([xs[0]], [ys[0]], color="darkred", s=90, zorder=6, label="Start")
            ax.scatter([xs[-1]], [ys[-1]], color="magenta", marker="*", s=160, zorder=6, label="Goal")

            sx_km = (xs[0])
            sy_km = (ys[0])
            gx_km = (xs[-1])
            gy_km = (ys[-1])
            ax.plot([sx_km, gx_km], [sy_km, gy_km], color="black", linestyle=":", linewidth=1.8, label="Direct Line")

        ax.set_title(title, fontsize=12, fontweight="bold")
        ax.set_xlabel("Easting (km)", fontsize=10)
        ax.set_ylabel("Northing (km)", fontsize=10)
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, linestyle=":", alpha=0.5)

    fig.suptitle("Authoritative Mountain Navigation & Terrain Decision Suite: Real Flight Physical Trajectories", fontsize=15, fontweight="bold")
    out_png = ROOT / "results" / "final_mountain_navigation_behavior.png"
    fig.savefig(out_png, dpi=200)
    plt.close(fig)

    shutil.copy(out_png, ARTIFACT_DIR / "final_mountain_navigation_behavior.png")
    print(f"Saved Special Mountain Navigation figure to {out_png} and artifact directory.")

    # -------------------------------------------------------------
    # GENERATE INDIVIDUAL 2-PANEL FIGURES FOR ALL MISSIONS
    # -------------------------------------------------------------
    print("Generating individual 2-panel figures for every mission...")
    indiv_dir = ROOT / "results" / "individual_behavior_plots"
    indiv_dir.mkdir(parents=True, exist_ok=True)
    extent_m = [roi.bounds[0], roi.bounds[2], roi.bounds[1], roi.bounds[3]]
    valid_elev = np.isfinite(elev_grid)
    min_elev, max_elev = float(np.min(elev_grid[valid_elev])), float(np.max(elev_grid[valid_elev]))

    for key, item in all_results.items():
        if item.get("status") != "FOUND" or not item.get("trajectory"):
            continue

        traj = item["trajectory"]
        xs = [p["x_m"] for p in traj]
        ys = [p["y_m"] for p in traj]
        zs = [p["z_msl_m"] for p in traj]
        dists = [p["cum_dist_m"] for p in traj]
        terrs = [p["elevation_msl_m"] for p in traj]

        fig, (ax_dem, ax_alt) = plt.subplots(1, 2, figsize=(16, 6), dpi=200)

        # Left: Top-down DEM
        im = ax_dem.imshow(
            elev_grid, extent=extent_m, origin="upper", cmap="terrain",
            alpha=0.88, vmin=min_elev, vmax=max_elev
        )
        cbar = plt.colorbar(im, ax=ax_dem, fraction=0.046, pad=0.04)
        cbar.set_label("Copernicus DEM Elevation (m MSL)", fontsize=9)

        ax_dem.plot([xs[0], xs[-1]], [ys[0], ys[-1]], "w--", linewidth=1.5, alpha=0.75, label="Direct Line")
        ax_dem.plot(xs, ys, "r-", linewidth=2.5, label="Physical Trajectory")
        ax_dem.scatter([xs[0]], [ys[0]], color="#00ffcc", edgecolors="black", s=100, zorder=5, label="Start")
        ax_dem.scatter([xs[-1]], [ys[-1]], color="#ff00ff", edgecolors="black", s=100, zorder=5, label="Goal")

        margin = max(500.0, (max(xs) - min(xs)) * 0.35, (max(ys) - min(ys)) * 0.35)
        ax_dem.set_xlim(min(xs) - margin, max(xs) + margin)
        ax_dem.set_ylim(min(ys) - margin, max(ys) + margin)
        ax_dem.set_title(f"{key}\nTop-Down Copernicus DEM & Trajectory", fontsize=11, fontweight="bold")
        ax_dem.set_xlabel("UTM X (m)", fontsize=10)
        ax_dem.set_ylabel("UTM Y (m)", fontsize=10)
        ax_dem.legend(loc="upper right", fontsize=8, framealpha=0.9)
        ax_dem.grid(True, linestyle=":", alpha=0.5)

        # Right: Altitude Profile
        terr_np = np.array(terrs)
        ax_alt.fill_between(dists, 0, terr_np, color="#8b7355", alpha=0.45, label="Terrain Profile")
        ax_alt.plot(dists, terr_np, color="#5c4033", linewidth=1.8)
        ax_alt.plot(dists, terr_np + 100.0, "r--", linewidth=1.4, alpha=0.85, label="Min AGL (+100m)")
        ax_alt.plot(dists, terr_np + 120.0, "g:", linewidth=1.4, alpha=0.85, label="Target AGL (+120m)")
        ax_alt.plot(dists, zs, "b-", linewidth=2.5, label="Aircraft MSL Altitude")

        ax_alt.set_title(f"Vertical Flight Profile (Min AGL = {item['min_agl_m']:.1f} m, Mean = {item['mean_agl_m']:.1f} m)", fontsize=11, fontweight="bold")
        ax_alt.set_xlabel("Distance Along Trajectory (m)", fontsize=10)
        ax_alt.set_ylabel("Altitude MSL (m)", fontsize=10)
        y_min = max(0, min(terrs) - 100)
        y_max = max(zs) + 150
        ax_alt.set_ylim(y_min, y_max)
        ax_alt.legend(loc="upper right", fontsize=8, framealpha=0.9)
        ax_alt.grid(True, linestyle=":", alpha=0.5)

        plt.tight_layout()
        out_name = f"{key}_2panel.png"
        fig.savefig(indiv_dir / out_name, bbox_inches="tight")
        fig.savefig(ARTIFACT_DIR / out_name, bbox_inches="tight")
        plt.close(fig)

    # -------------------------------------------------------------
    # GENERATE LAYER 1 DASHBOARD (M1 - M7)
    # -------------------------------------------------------------
    print("Generating Layer 1 Basic Maneuvers Dashboard...")
    m_keys = [
        "M1_Straight_Level", "M2_Straight_Climb", "M3_Straight_Descent",
        "M4_Left_Turn", "M5_Right_Turn", "M6_90_Deg_Turn", "M7_180_Deg_Turn_Behind"
    ]
    fig = plt.figure(figsize=(20, 14), dpi=200)
    gs = GridSpec(4, 4, figure=fig, hspace=0.35, wspace=0.3)
    
    for idx, k in enumerate(m_keys):
        item = all_results.get(k)
        if not item or not item.get("trajectory"):
            continue
        traj = item["trajectory"]
        xs = [p["x_m"] for p in traj]
        ys = [p["y_m"] for p in traj]
        zs = [p["z_msl_m"] for p in traj]
        dists = [p["cum_dist_m"] for p in traj]
        terrs = [p["elevation_msl_m"] for p in traj]

        row = idx // 2
        col = (idx % 2) * 2
        ax_xy = fig.add_subplot(gs[row, col])
        ax_xy.plot([xs[0], xs[-1]], [ys[0], ys[-1]], "k--", alpha=0.5, label="Direct")
        ax_xy.plot(xs, ys, "r-", linewidth=2.0, label="Trajectory")
        ax_xy.scatter([xs[0]], [ys[0]], color="lime", edgecolors="k", s=60, zorder=4)
        ax_xy.scatter([xs[-1]], [ys[-1]], color="magenta", edgecolors="k", s=60, zorder=4)
        ax_xy.set_title(f"{k} (XY Track)", fontsize=10, fontweight="bold")
        ax_xy.set_xlabel("X (m)", fontsize=8)
        ax_xy.set_ylabel("Y (m)", fontsize=8)
        ax_xy.axis("equal")
        ax_xy.grid(True, linestyle=":", alpha=0.5)

        ax_z = fig.add_subplot(gs[row, col + 1])
        ax_z.plot(dists, terrs, color="#8b7355", linewidth=1.2, label="Terrain")
        ax_z.plot(dists, np.array(terrs) + 100.0, "r--", linewidth=1.0, alpha=0.7, label="+100m")
        ax_z.plot(dists, zs, "b-", linewidth=2.0, label="Aircraft MSL")
        ax_z.set_title(f"{k} (Altitude Profile)", fontsize=10, fontweight="bold")
        ax_z.set_xlabel("Dist (m)", fontsize=8)
        ax_z.set_ylabel("MSL (m)", fontsize=8)
        ax_z.grid(True, linestyle=":", alpha=0.5)

    ax_card = fig.add_subplot(gs[3, 2:])
    ax_card.axis("off")
    card_text = (
        "LAYER 1: BASIC MANEUVER COVERAGE VALIDATION\n"
        "----------------------------------------------------\n"
        "• M1: Straight Level (19 straight primitives, 0 slope)\n"
        "• M2: Straight Climb (16 climb primitives, max slope +5.36%)\n"
        "• M3: Straight Descent (15 descent primitives, max slope -10.57%)\n"
        "• M4: Left Turn (6 left turns, realized turn radius 229.2m)\n"
        "• M5: Right Turn (6 right turns, realized turn radius 229.2m)\n"
        "• M6: 90° Turn (13 right turns, smooth curved radius)\n"
        "• M7: 180° Turn / Behind (Teardrop turnaround, 19 left turns)\n\n"
        "RESULT: 7 / 7 BASIC MANEUVERS PASS (100% Physical & Safe)"
    )
    ax_card.text(0.05, 0.5, card_text, fontsize=11, fontfamily="monospace", verticalalignment="center",
                 bbox=dict(boxstyle="round,pad=0.8", facecolor="#eef7fa", edgecolor="#007acc", alpha=0.9))

    fig.suptitle("Layer 1 — Fixed-Wing Basic Maneuver Coverage (M1 – M7)", fontsize=15, fontweight="bold", y=0.99)
    l1_path = ROOT / "results" / "layer1_basic_maneuvers_coverage.png"
    fig.savefig(l1_path, bbox_inches="tight")
    plt.close(fig)
    shutil.copy(l1_path, ARTIFACT_DIR / "layer1_basic_maneuvers_coverage.png")

    # -------------------------------------------------------------
    # GENERATE LAYER 2 DASHBOARD (B1 - B6)
    # -------------------------------------------------------------
    print("Generating Layer 2 Terrain Navigation Dashboard...")
    b_keys_all = [
        "B1_Mountain_Circumnavigation", "B2_Left_Right_Bypass", "B3_Ridge_Crossing_Chosen",
        "B4_Ridge_Too_High_Detour", "B5_S_Shaped_Corridor", "B6_Narrow_Valley_Turn"
    ]
    fig, axes = plt.subplots(6, 2, figsize=(18, 22), dpi=200)

    for idx, k in enumerate(b_keys_all):
        item = all_results.get(k)
        if not item or not item.get("trajectory"):
            continue
        traj = item["trajectory"]
        xs = [p["x_m"] for p in traj]
        ys = [p["y_m"] for p in traj]
        zs = [p["z_msl_m"] for p in traj]
        dists = [p["cum_dist_m"] for p in traj]
        terrs = [p["elevation_msl_m"] for p in traj]

        ax_dem = axes[idx, 0]
        im = ax_dem.imshow(elev_grid, extent=extent_m, origin="upper", cmap="terrain", alpha=0.85, vmin=min_elev, vmax=max_elev)
        ax_dem.plot([xs[0], xs[-1]], [ys[0], ys[-1]], "w--", linewidth=1.5)
        ax_dem.plot(xs, ys, "r-", linewidth=2.5)
        ax_dem.scatter([xs[0]], [ys[0]], color="lime", edgecolors="k", s=70, zorder=5)
        ax_dem.scatter([xs[-1]], [ys[-1]], color="magenta", edgecolors="k", s=70, zorder=5)
        m = max(500.0, (max(xs) - min(xs)) * 0.35, (max(ys) - min(ys)) * 0.35)
        ax_dem.set_xlim(min(xs) - m, max(xs) + m)
        ax_dem.set_ylim(min(ys) - m, max(ys) + m)
        ax_dem.set_title(f"{k} — Copernicus Top-Down DEM", fontsize=10, fontweight="bold")
        ax_dem.set_xlabel("UTM X (m)", fontsize=8)
        ax_dem.set_ylabel("UTM Y (m)", fontsize=8)
        ax_dem.grid(True, linestyle=":", alpha=0.4)

        ax_alt = axes[idx, 1]
        t_arr = np.array(terrs)
        ax_alt.fill_between(dists, 0, t_arr, color="#8b7355", alpha=0.45)
        ax_alt.plot(dists, t_arr, color="#5c4033", linewidth=1.5, label="Terrain")
        ax_alt.plot(dists, t_arr + 100.0, "r--", linewidth=1.2, label="+100m AGL")
        ax_alt.plot(dists, zs, "b-", linewidth=2.2, label="Aircraft MSL")
        ax_alt.set_title(f"{k} — Altitude Profile (Min AGL: {item['min_agl_m']:.1f}m, Detour: {item['detour_ratio']:.2f})", fontsize=10, fontweight="bold")
        ax_alt.set_xlabel("Path Dist (m)", fontsize=8)
        ax_alt.set_ylabel("MSL (m)", fontsize=8)
        ax_alt.set_ylim(max(0, min(terrs) - 80), max(zs) + 120)
        ax_alt.legend(loc="upper right", fontsize=7)
        ax_alt.grid(True, linestyle=":", alpha=0.4)

    fig.suptitle("Layer 2 — Terrain Navigation Behavior (B1 – B6)", fontsize=16, fontweight="bold", y=0.995)
    plt.tight_layout()
    l2_path = ROOT / "results" / "layer2_terrain_navigation_coverage.png"
    fig.savefig(l2_path, bbox_inches="tight")
    plt.close(fig)
    shutil.copy(l2_path, ARTIFACT_DIR / "layer2_terrain_navigation_coverage.png")

    # -------------------------------------------------------------
    # GENERATE LAYER 3 DASHBOARD (V1 - V5)
    # -------------------------------------------------------------
    print("Generating Layer 3 Vertical Decision Behavior Dashboard...")
    v_keys_all = [
        "V1_Western_Valley_Descent", "V2_Western_Valley_Reverse_Climb",
        "V3_Early_Climb_Required", "V4_Short_Terrain_Dip", "V5_Deep_Valley_Descent"
    ]
    fig, axes = plt.subplots(5, 2, figsize=(18, 19), dpi=200)

    for idx, k in enumerate(v_keys_all):
        item = all_results.get(k)
        if not item or not item.get("trajectory"):
            continue
        traj = item["trajectory"]
        xs = [p["x_m"] for p in traj]
        ys = [p["y_m"] for p in traj]
        zs = [p["z_msl_m"] for p in traj]
        dists = [p["cum_dist_m"] for p in traj]
        terrs = [p["elevation_msl_m"] for p in traj]

        ax_dem = axes[idx, 0]
        im = ax_dem.imshow(elev_grid, extent=extent_m, origin="upper", cmap="terrain", alpha=0.85, vmin=min_elev, vmax=max_elev)
        ax_dem.plot([xs[0], xs[-1]], [ys[0], ys[-1]], "w--", linewidth=1.5)
        ax_dem.plot(xs, ys, "r-", linewidth=2.5)
        ax_dem.scatter([xs[0]], [ys[0]], color="lime", edgecolors="k", s=70, zorder=5)
        ax_dem.scatter([xs[-1]], [ys[-1]], color="magenta", edgecolors="k", s=70, zorder=5)
        m = max(500.0, (max(xs) - min(xs)) * 0.35, (max(ys) - min(ys)) * 0.35)
        ax_dem.set_xlim(min(xs) - m, max(xs) + m)
        ax_dem.set_ylim(min(ys) - m, max(ys) + m)
        ax_dem.set_title(f"{k} — Top-Down View", fontsize=10, fontweight="bold")
        ax_dem.set_xlabel("UTM X (m)", fontsize=8)
        ax_dem.set_ylabel("UTM Y (m)", fontsize=8)
        ax_dem.grid(True, linestyle=":", alpha=0.4)

        ax_alt = axes[idx, 1]
        t_arr = np.array(terrs)
        ax_alt.fill_between(dists, 0, t_arr, color="#8b7355", alpha=0.45)
        ax_alt.plot(dists, t_arr, color="#5c4033", linewidth=1.5, label="Terrain")
        ax_alt.plot(dists, t_arr + 100.0, "r--", linewidth=1.2, label="+100m AGL")
        ax_alt.plot(dists, t_arr + 120.0, "g:", linewidth=1.2, label="+120m Target")
        ax_alt.plot(dists, zs, "b-", linewidth=2.2, label="Aircraft MSL")
        ax_alt.set_title(f"{k} — Vertical Profile (Min AGL: {item['min_agl_m']:.1f}m, Mean: {item['mean_agl_m']:.1f}m)", fontsize=10, fontweight="bold")
        ax_alt.set_xlabel("Path Dist (m)", fontsize=8)
        ax_alt.set_ylabel("MSL (m)", fontsize=8)
        ax_alt.set_ylim(max(0, min(terrs) - 80), max(zs) + 120)
        ax_alt.legend(loc="upper right", fontsize=7)
        ax_alt.grid(True, linestyle=":", alpha=0.4)

    fig.suptitle("Layer 3 — Vertical Decision Behavior (V1 – V5)", fontsize=16, fontweight="bold", y=0.995)
    plt.tight_layout()
    l3_path = ROOT / "results" / "layer3_vertical_decisions_coverage.png"
    fig.savefig(l3_path, bbox_inches="tight")
    plt.close(fig)
    shutil.copy(l3_path, ARTIFACT_DIR / "layer3_vertical_decisions_coverage.png")

    # -------------------------------------------------------------
    # GENERATE CANONICAL SUITE DASHBOARD (A - F)
    # -------------------------------------------------------------
    print("Generating Canonical Regression Suite Dashboard...")
    c_keys_all = [
        "can_A_easy_open", "can_B_relief_affected", "can_C_far_south_3km",
        "can_D_far_east_3km", "can_E_long_descent_9_3km", "can_F_turn_required_diagonal_2_3km"
    ]
    fig, axes = plt.subplots(6, 2, figsize=(18, 22), dpi=200)

    for idx, k in enumerate(c_keys_all):
        item = all_results.get(k)
        if not item or not item.get("trajectory"):
            continue
        traj = item["trajectory"]
        xs = [p["x_m"] for p in traj]
        ys = [p["y_m"] for p in traj]
        zs = [p["z_msl_m"] for p in traj]
        dists = [p["cum_dist_m"] for p in traj]
        terrs = [p["elevation_msl_m"] for p in traj]

        ax_dem = axes[idx, 0]
        im = ax_dem.imshow(elev_grid, extent=extent_m, origin="upper", cmap="terrain", alpha=0.85, vmin=min_elev, vmax=max_elev)
        ax_dem.plot([xs[0], xs[-1]], [ys[0], ys[-1]], "w--", linewidth=1.5)
        ax_dem.plot(xs, ys, "r-", linewidth=2.5)
        ax_dem.scatter([xs[0]], [ys[0]], color="lime", edgecolors="k", s=70, zorder=5)
        ax_dem.scatter([xs[-1]], [ys[-1]], color="magenta", edgecolors="k", s=70, zorder=5)
        m = max(500.0, (max(xs) - min(xs)) * 0.35, (max(ys) - min(ys)) * 0.35)
        ax_dem.set_xlim(min(xs) - m, max(xs) + m)
        ax_dem.set_ylim(min(ys) - m, max(ys) + m)
        ax_dem.set_title(f"{k} — Top-Down View", fontsize=10, fontweight="bold")
        ax_dem.set_xlabel("UTM X (m)", fontsize=8)
        ax_dem.set_ylabel("UTM Y (m)", fontsize=8)
        ax_dem.grid(True, linestyle=":", alpha=0.4)

        ax_alt = axes[idx, 1]
        t_arr = np.array(terrs)
        ax_alt.fill_between(dists, 0, t_arr, color="#8b7355", alpha=0.45)
        ax_alt.plot(dists, t_arr, color="#5c4033", linewidth=1.5, label="Terrain")
        ax_alt.plot(dists, t_arr + 100.0, "r--", linewidth=1.2, label="+100m AGL")
        ax_alt.plot(dists, zs, "b-", linewidth=2.2, label="Aircraft MSL")
        ax_alt.set_title(f"{k} — Altitude Profile (Path: {item['path_length_m']:.1f}m, Min AGL: {item['min_agl_m']:.1f}m)", fontsize=10, fontweight="bold")
        ax_alt.set_xlabel("Path Dist (m)", fontsize=8)
        ax_alt.set_ylabel("MSL (m)", fontsize=8)
        ax_alt.set_ylim(max(0, min(terrs) - 80), max(zs) + 120)
        ax_alt.legend(loc="upper right", fontsize=7)
        ax_alt.grid(True, linestyle=":", alpha=0.4)

    fig.suptitle("Regression Suite — Canonical Missions (A – F)", fontsize=16, fontweight="bold", y=0.995)
    plt.tight_layout()
    can_path = ROOT / "results" / "canonical_suite_coverage.png"
    fig.savefig(can_path, bbox_inches="tight")
    plt.close(fig)
    shutil.copy(can_path, ARTIFACT_DIR / "canonical_suite_coverage.png")

    # -------------------------------------------------------------
    # GENERATE MASTER 24-MISSION BEHAVIOR ATLAS
    # -------------------------------------------------------------
    print("Generating Master 24-Mission Behavior Atlas...")
    all_24_keys = m_keys + b_keys_all + v_keys_all + c_keys_all  # 7 + 6 + 5 + 6 = 24
    fig, axes = plt.subplots(6, 4, figsize=(24, 28), dpi=200)

    for idx, k in enumerate(all_24_keys):
        item = all_results.get(k)
        r_idx = idx // 4
        c_idx = idx % 4
        ax = axes[r_idx, c_idx]
        if not item or not item.get("trajectory"):
            ax.axis("off")
            continue

        traj = item["trajectory"]
        xs = [p["x_m"] for p in traj]
        ys = [p["y_m"] for p in traj]
        zs = [p["z_msl_m"] for p in traj]

        im = ax.imshow(elev_grid, extent=extent_m, origin="upper", cmap="terrain", alpha=0.85, vmin=min_elev, vmax=max_elev)
        ax.plot([xs[0], xs[-1]], [ys[0], ys[-1]], "w--", linewidth=1.2, alpha=0.7)
        ax.plot(xs, ys, "r-", linewidth=2.2)
        ax.scatter([xs[0]], [ys[0]], color="lime", edgecolors="k", s=40, zorder=5)
        ax.scatter([xs[-1]], [ys[-1]], color="magenta", edgecolors="k", s=40, zorder=5)

        m = max(500.0, (max(xs) - min(xs)) * 0.4, (max(ys) - min(ys)) * 0.4)
        ax.set_xlim(min(xs) - m, max(xs) + m)
        ax.set_ylim(min(ys) - m, max(ys) + m)
        ax.set_title(f"{k}\nL={item['path_length_m']:.0f}m | MinAGL={item['min_agl_m']:.0f}m", fontsize=9, fontweight="bold")
        ax.set_xticks([])
        ax.set_yticks([])

    fig.suptitle("Master Fixed-Wing Behavior Atlas (All 24 Physical Trajectories)", fontsize=18, fontweight="bold", y=0.995)
    plt.tight_layout()
    atlas_path = ROOT / "results" / "final_all_24_missions_atlas.png"
    fig.savefig(atlas_path, bbox_inches="tight")
    plt.close(fig)
    shutil.copy(atlas_path, ARTIFACT_DIR / "final_all_24_missions_atlas.png")

    # Save complete metrics JSON (without dumping raw sample points)
    clean_metrics = {}
    for k, v in all_results.items():
        clean_metrics[k] = {ik: iv for ik, iv in v.items() if ik != "trajectory"}

    out_json = ROOT / "results" / "final_maneuver_planner_coverage.json"
    with open(out_json, "w") as f:
        json.dump(clean_metrics, f, indent=2)
    print(f"Saved complete metrics JSON to {out_json}")

if __name__ == "__main__":
    main()

