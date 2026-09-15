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

def run_valley_eval(start_rc, goal_rc, start_agl=130.0, goal_agl=120.0):
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

    start_z = elev_s + start_agl
    goal_z = elev_g + goal_agl
    heading = navigation_bearing_deg(sx, sy, gx, gy)

    start = PhysicalPose(sx, sy, start_z, heading)
    goal = GoalPose(gx, gy, goal_z)
    tol = GoalTolerance(xy_m=120.0, altitude_m=30.0)

    print(f"\n=======================================================")
    print(f"RUNNING MISSION: Start RC={start_rc} -> Goal RC={goal_rc}")
    print(f"Start: ({sx:.1f}, {sy:.1f}, {start_z:.1f} MSL), Elev={elev_s:.1f}m, AGL={start_agl:.1f}m, Hdg={heading:.1f}°")
    print(f"Goal:  ({gx:.1f}, {gy:.1f}, {goal_z:.1f} MSL), Elev={elev_g:.1f}m, Target AGL={goal_agl:.1f}m")
    sys.stdout.flush()

    for w_h in [1.01]:
        for mode in ["BASELINE (Euclidean)", "VALLEY_GUIDED (Lookahead / Topo)"]:
            t0 = time.perf_counter()
            use_valley = "VALLEY" in mode

            def heuristic_fn(p: PhysicalPose):
                dx = p.x_m - goal.x_m
                dy = p.y_m - goal.y_m
                dz = max(0.0, abs(p.z_msl_m - goal.z_msl_m) - tol.altitude_m)
                return math.sqrt(dx*dx + dy*dy + dz*dz)

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
            generated = 0
            rejected = 0
            reject_reasons = Counter()
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
                        rejected += 1
                        reject_reasons["UNAVAILABLE_CAPABILITY"] += 1
                        continue

                    safety = evaluate_physical_trajectory_safety(
                        traj, terrain, CONFIG.min_agl_m, CONFIG.primitive_sample_spacing_m,
                        planning_bounds=terrain.roi.bounds, lateral_buffer_m=CONFIG.lateral_buffer_m,
                        terrain_influence_cache=infl_cache,
                    )
                    if not safety.is_safe:
                        rejected += 1
                        reject_reasons["SAFETY_VIOLATION"] += 1
                        continue

                    end_p = traj.end_pose
                    key = search_key_for_pose(end_p, CONFIG)
                    if key == node.key:
                        rejected += 1
                        reject_reasons["SAME_KEY_SELF_TRANSITION"] += 1
                        continue

                    cand_g = node.g_cost + _trajectory_3d_length(traj)
                    existing = active.get(key)
                    if existing is not None and cand_g >= existing.g_cost - 1e-12:
                        rejected += 1
                        reject_reasons["SAME_KEY_DOMINANCE"] += 1
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
                        rejected += 1
                        reject_reasons["PARETO_Z_DOMINANCE"] += 1
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
                pct_100_150 = sum(1 for a in agls if 100.0 <= a <= 150.0) / len(agls) * 100.0
                pct_150_200 = sum(1 for a in agls if 150.0 < a <= 200.0) / len(agls) * 100.0
                pct_gt_250 = sum(1 for a in agls if a > 250.0) / len(agls) * 100.0
                print(f"[{mode}] SUCCESS in {dt:.3f}s | Exp: {expanded} | Gen: {generated} | Len: {path_len:.1f}m | Prims: {len(prims)}")
                print(f"  Min AGL: {min_agl:.1f}m | Mean AGL: {mean_agl:.1f}m | Median AGL: {median_agl:.1f}m | P90 AGL: {p90_agl:.1f}m")
                print(f"  Distribution: [100-150m]: {pct_100_150:.1f}% | [150-200m]: {pct_150_200:.1f}% | [>250m]: {pct_gt_250:.1f}%")
            else:
                print(f"[{mode}] FAILED in {dt:.3f}s | Exp: {expanded} | Gen: {generated} | Rejections: {dict(reject_reasons)}")
            sys.stdout.flush()

if __name__ == "__main__":
    # Test starting past the bump at Row 65 (2.7 km valley flight to Row 20)
    run_valley_eval((64, 5), (20, 5), start_agl=130.0, goal_agl=120.0)
    # Test starting at Row 75 with sufficient initial clearance (start_agl=145.0)
    run_valley_eval((75, 5), (20, 5), start_agl=145.0, goal_agl=120.0)
