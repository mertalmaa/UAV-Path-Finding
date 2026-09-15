import math
import sys
import numpy as np
import heapq
import itertools
from collections import Counter, defaultdict
from pathlib import Path

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
from scripts.benchmark_missions import CACHE_DIR, CONFIG, FACTOR, GOAL_TOLERANCE, PROFILE_PATH, SOURCE_DEM_PATH

roi = load_roi(CONFIG)
cache = load_terrain_cache(CACHE_DIR, roi, SOURCE_DEM_PATH)
terrain = build_terrain_query_from_cache(cache, roi, FACTOR)
profile = load_aircraft_profile(PROFILE_PATH)
envelope = FixedWingKinematicEnvelope()
infl_cache = TerrainInfluenceCache(terrain)

start_rc = (64, 5)
goal_rc = (20, 5)
sx, sy = terrain.rowcol_to_xy(*start_rc)
gx, gy = terrain.rowcol_to_xy(*goal_rc)
elev_s = float(terrain.query(sx, sy).elevation)
elev_g = float(terrain.query(gx, gy).elevation)
start_z = elev_s + 130.0
goal_z = elev_g + 120.0
heading = navigation_bearing_deg(sx, sy, gx, gy)

start = PhysicalPose(sx, sy, start_z, heading)
goal = GoalPose(gx, gy, goal_z)
tol = GoalTolerance(xy_m=120.0, altitude_m=30.0)

def heuristic_fn(p: PhysicalPose):
    dx = p.x_m - goal.x_m
    dy = p.y_m - goal.y_m
    dz = max(0.0, abs(p.z_msl_m - goal.z_msl_m) - tol.altitude_m)
    return math.sqrt(dx*dx + dy*dy + dz*dz)

start_key = search_key_for_pose(start, CONFIG)
start_node = PoseSearchNode(0, start_key, start, 0.0, None, None, None)
active = {start_key: start_node}
all_nodes = {0: start_node}

f_0 = 1.01 * heuristic_fn(start)
counter = itertools.count(1)
open_heap = [(f_0, 0, 0)]
expanded_ids = set()
expanded = 0

while open_heap and expanded < 35:
    _, _, node_id = heapq.heappop(open_heap)
    node = all_nodes[node_id]
    if active.get(node.key) is not node or node_id in expanded_ids:
        continue
    expanded_ids.add(node_id)
    expanded += 1

    print(f"\n--- EXPANDING NODE {expanded}: pos=({node.end_pose.x_m:.1f}, {node.end_pose.y_m:.1f}, {node.end_pose.z_msl_m:.1f}), g={node.g_cost:.1f}, key={node.key} ---")

    for prim, traj in _candidate_trajectories(node.end_pose, envelope, CONFIG):
        if traj is None:
            print(f"  {prim:20s} -> NONE")
            continue
        safety = evaluate_physical_trajectory_safety(
            traj, terrain, CONFIG.min_agl_m, CONFIG.primitive_sample_spacing_m,
            planning_bounds=terrain.roi.bounds, lateral_buffer_m=CONFIG.lateral_buffer_m,
            terrain_influence_cache=infl_cache,
        )
        if not safety.is_safe:
            print(f"  {prim:20s} -> UNSAFE (min_agl={safety.min_agl_m:.1f}m, reason={safety.failure_reason})")
            continue

        end_p = traj.end_pose
        key = search_key_for_pose(end_p, CONFIG)
        if key == node.key:
            print(f"  {prim:20s} -> SAME_KEY_SELF")
            continue

        cand_g = node.g_cost + _trajectory_3d_length(traj)
        existing = active.get(key)
        if existing is not None and cand_g >= existing.g_cost - 1e-12:
            print(f"  {prim:20s} -> SAME_KEY_DOM (existing_g={existing.g_cost:.1f}, cand_g={cand_g:.1f})")
            continue

        nid = next(counter)
        succ = PoseSearchNode(nid, key, end_p, cand_g, node.node_id, traj, prim)
        active[key] = succ
        all_nodes[nid] = succ
        f_score = cand_g + 1.01 * heuristic_fn(end_p)
        heapq.heappush(open_heap, (f_score, nid, nid))
        print(f"  {prim:20s} -> INSERTED: end_z={end_p.z_msl_m:.1f}, key={key}, f={f_score:.1f}")
