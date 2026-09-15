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

# Mission: 3.3 km Western Valley Corridor (Row 75, Col 5 -> Row 20, Col 5)
start_rc = (75, 5)
goal_rc = (20, 5)
sx, sy = terrain.rowcol_to_xy(*start_rc)
gx, gy = terrain.rowcol_to_xy(*goal_rc)
elev_s = float(terrain.query(sx, sy).elevation)
elev_g = float(terrain.query(gx, gy).elevation)
start_z = elev_s + 120.0  # 2055 + 120 = 2175m MSL
goal_z = elev_g + 140.0   # 1795 + 140 = 1935m MSL (Delta Z = 240m over 3300m = 7.27% slope, strictly feasible!)
heading = navigation_bearing_deg(sx, sy, gx, gy)

start = PhysicalPose(sx, sy, start_z, heading)
goal = GoalPose(gx, gy, goal_z)
tol = GoalTolerance(xy_m=120.0, altitude_m=30.0)

print(f"Start: ({sx:.1f}, {sy:.1f}, {start_z:.1f} MSL), Elev={elev_s:.1f}, AGL=120m, Heading={heading:.1f}")
print(f"Goal:  ({gx:.1f}, {gy:.1f}, {goal_z:.1f} MSL), Elev={elev_g:.1f}, Target AGL=140m")
print(f"Distance: 3300m, Delta Z: {start_z - goal_z:.1f}m, Required Slope: {(start_z - goal_z)/3300*100:.2f}% (Max C172 is 7.5%)")

elev_grid = terrain.roi.elevation.copy()
valid = np.isfinite(elev_grid)
min_e, max_e = float(np.min(elev_grid[valid])), float(np.max(elev_grid[valid]))
elev_grid[~valid] = min_e
cell_m = abs(float(terrain.roi.transform.a))
rows, cols = elev_grid.shape

dy, dx = np.gradient(elev_grid, cell_m)
slope = np.sqrt(dx*dx + dy*dy)
max_s = float(np.percentile(slope, 95))
r_norm = (elev_grid - min_e) / max(1.0, max_e - min_e)
s_norm = np.clip(slope / max(0.1, max_s), 0.0, 2.0)
cost_surf = 1.0 + 2.0 * (r_norm**2) + 1.0 * (s_norm**2)

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

def heuristic_fn(p: PhysicalPose):
    r, c = terrain.xy_to_rowcol(p.x_m, p.y_m)
    r = max(0, min(rows - 1, r))
    c = max(0, min(cols - 1, c))
    h_2d = float(h_grid[r, c])
    local_elev = elev_grid[r, c]
    d_to_goal = math.hypot(p.x_m - goal.x_m, p.y_m - goal.y_m)
    z_req = max(local_elev + 120.0, goal.z_msl_m - 0.075 * d_to_goal)
    dz = max(0.0, abs(p.z_msl_m - z_req) - tol.altitude_m)
    return math.sqrt(h_2d*h_2d + dz*dz)

start_key = search_key_for_pose(start, CONFIG)
start_node = PoseSearchNode(0, start_key, start, 0.0, None, None, None)
active = {start_key: start_node}
all_nodes = {0: start_node}

pareto_frontier = defaultdict(list)
start_xyh = (start_key.x_bin, start_key.y_bin, start_key.heading_bin)
r_s, c_s = terrain.xy_to_rowcol(start.x_m, start.y_m)
pareto_frontier[start_xyh].append((0.0, max(0.0, start.z_msl_m - (elev_grid[r_s, c_s] + 120.0)), start_key))

f_0 = 1.01 * heuristic_fn(start)
counter = itertools.count(1)
open_heap = [(f_0, 0, 0)]
expanded_ids = set()
expanded = 0
goal_node = None

while open_heap and expanded < 10000:
    _, _, node_id = heapq.heappop(open_heap)
    node = all_nodes[node_id]
    if active.get(node.key) is not node or node_id in expanded_ids:
        continue
    expanded_ids.add(node_id)
    expanded += 1

    if pose_in_goal(node.end_pose, goal, tol):
        goal_node = node
        print(f"GOAL REACHED at expansion {expanded}!")
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

        l3d = _trajectory_3d_length(traj)
        r_end, c_end = terrain.xy_to_rowcol(end_p.x_m, end_p.y_m)
        r_end = max(0, min(rows - 1, r_end))
        c_end = max(0, min(cols - 1, c_end))
        local_elev = elev_grid[r_end, c_end]
        cand_excess = max(0.0, (end_p.z_msl_m - local_elev) - 120.0)
        cand_g = node.g_cost + l3d * (cost_surf[r_end, c_end] + 0.20 * (cand_excess / 500.0))

        xyh = (key.x_bin, key.y_bin, key.heading_bin)
        frontier = pareto_frontier[xyh]
        dominated = False
        for fg, fe, _ in frontier:
            if fg <= cand_g + 1e-9 and fe <= cand_excess + 1e-9:
                dominated = True
                break
        if dominated:
            continue

        pareto_frontier[xyh] = [
            (fg, fe, fk) for fg, fe, fk in frontier
            if not (cand_g <= fg + 1e-9 and cand_excess <= fe + 1e-9)
        ]
        pareto_frontier[xyh].append((cand_g, cand_excess, key))

        nid = next(counter)
        succ = PoseSearchNode(nid, key, end_p, cand_g, node.node_id, traj, prim)
        active[key] = succ
        all_nodes[nid] = succ
        f_score = cand_g + 1.01 * heuristic_fn(end_p)
        heapq.heappush(open_heap, (f_score, nid, nid))

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
    print(f"SUCCESS in {expanded} expansions!")
    print(f"  Primitives ({len(prims)}): {dict(Counter(prims))}")
    print(f"  Path Length: {path_len:.1f}m | Min AGL: {min(agls):.1f}m | Mean AGL: {sum(agls)/len(agls):.1f}m | Med: {np.median(agls):.1f}m | P90: {np.percentile(agls, 90):.1f}m")
else:
    print(f"FAILED in {expanded} expansions.")
