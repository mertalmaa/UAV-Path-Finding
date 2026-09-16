import dataclasses
from pathlib import Path
import time
import heapq
import itertools

from planner.config import DEFAULT_CONFIG
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from planner.physical import PhysicalPose
from planner.fixed_wing_envelope import FixedWingKinematicEnvelope
from planner.pose_search import (
    GoalPose, GoalTolerance, _TerrainGuidance, vertical_reachability_heuristic,
    navigation_bearing_deg, PoseSearchNode, search_key_for_pose,
    evaluate_physical_trajectory_safety, _candidate_trajectories,
    _trajectory_edge_cost, pose_in_goal
)
from scripts.bilecik_missions_spec import BILECIK_MISSIONS
from planner.trajectory_safety import TerrainInfluenceCache

base_cfg = dataclasses.replace(
    DEFAULT_CONFIG,
    working_dem_path=Path('regions/bilecik/working_dem.tif'),
    roi_size_m=30_000.0,
    roi_center_lonlat=(30.30, 40.25),
    target_crs='EPSG:32636',
    min_agl_m=100.0,
    lateral_buffer_m=60.0,
    search_heuristic_weight=1.05,
    enable_combined_turns=True,
    enable_pareto_z_pruning=False,
    enable_terrain_guidance=True,
)
roi = load_roi(base_cfg)
tq = TerrainQuery(roi)
cache = TerrainInfluenceCache(tq)
field = cache.field(60.0)
envelope = FixedWingKinematicEnvelope()

def smha_star(start, goal, terrain, goal_tolerance, config, K=3, max_expansions=30000, max_search_time_s=10.0):
    influence_cache = TerrainInfluenceCache(terrain)
    guidance = _TerrainGuidance(terrain, goal, goal_tolerance, config, envelope, influence_cache)
    counter = itertools.count()
    next_node_id = itertools.count()
    start_key = search_key_for_pose(start, config)
    start_node = PoseSearchNode(next(next_node_id), start_key, start, 0.0, None, None, None)
    active = {start_key: start_node}
    all_nodes = {start_node.node_id: start_node}
    h_anchor = vertical_reachability_heuristic(start, goal, goal_tolerance, envelope)
    h_guided = guidance.estimate(start, h_anchor)
    open_0 = [(config.search_heuristic_weight * h_anchor, next(counter), start_node.node_id)]
    open_1 = [(config.search_heuristic_weight * h_guided, next(counter), start_node.node_id)]
    expanded_ids = set()
    expanded = 0
    step_count = 0
    t0 = time.perf_counter()
    while open_0 or open_1:
        if time.perf_counter() - t0 >= max_search_time_s:
            return False, 'timeout', expanded, time.perf_counter() - t0
        step_count += 1
        use_guided = (step_count % (K + 1) != 0) if open_1 else False
        target_heap = open_1 if use_guided else open_0
        if not target_heap: target_heap = open_1 if open_1 else open_0
        _, _, node_id = heapq.heappop(target_heap)
        node = all_nodes[node_id]
        if active.get(node.key) is not node or node_id in expanded_ids: continue
        expanded_ids.add(node_id)
        expanded += 1
        if pose_in_goal(node.end_pose, goal, goal_tolerance):
            return True, 'success', expanded, time.perf_counter() - t0
        if expanded >= max_expansions: return False, 'expansion_limit', expanded, time.perf_counter() - t0
        for primitive, trajectory in _candidate_trajectories(node.end_pose, envelope, config):
            if trajectory is None: continue
            safety = evaluate_physical_trajectory_safety(trajectory, terrain, config.min_agl_m, config.primitive_sample_spacing_m, planning_bounds=terrain.roi.bounds, lateral_buffer_m=config.lateral_buffer_m, terrain_influence_cache=influence_cache)
            if not safety.is_safe: continue
            key = search_key_for_pose(trajectory.end_pose, config)
            if key == node.key: continue
            candidate_g = node.g_cost + _trajectory_edge_cost(trajectory, safety, config, guidance)
            existing = active.get(key)
            if existing is not None and candidate_g >= existing.g_cost - 1e-12: continue
            succ = PoseSearchNode(next(next_node_id), key, trajectory.end_pose, candidate_g, node.node_id, trajectory, primitive)
            active[key] = succ
            all_nodes[succ.node_id] = succ
            h_geom = vertical_reachability_heuristic(trajectory.end_pose, goal, goal_tolerance, envelope)
            h_gui = guidance.estimate(trajectory.end_pose, h_geom)
            heapq.heappush(open_0, (candidate_g + config.search_heuristic_weight * h_geom, next(counter), succ.node_id))
            heapq.heappush(open_1, (candidate_g + config.search_heuristic_weight * h_gui, next(counter), succ.node_id))
    return False, 'no_path', expanded, time.perf_counter() - t0

passed = 0
failed = []
print("=== BENCHMARKING ALL 30 MISSIONS (K=3) ===", flush=True)
for idx, m in enumerate(BILECIK_MISSIONS, 1):
    sx, sy = m.start_xy
    gx, gy = m.goal_xy
    r1, c1 = tq.xy_to_rowcol(sx, sy)
    r2, c2 = tq.xy_to_rowcol(gx, gy)
    s_ground = float(field.elevation_msl[r1, c1])
    g_ground = float(field.elevation_msl[r2, c2])
    sz = s_ground + max(130.0, m.start_alt_offset_m)
    gz = g_ground + max(130.0, m.goal_alt_offset_m)
    hd = m.start_heading_deg or navigation_bearing_deg(sx, sy, gx, gy)
    start_pose = PhysicalPose(sx, sy, sz, hd)
    goal_pose = GoalPose(gx, gy, gz)
    tol = GoalTolerance(m.goal_tolerance_xy_m, m.goal_tolerance_z_m)
    s, r, exp, dt = smha_star(start_pose, goal_pose, tq, tol, base_cfg, K=3)
    if s: 
        passed += 1
        tag = "PASS"
    else: 
        failed.append(m.id)
        tag = "FAIL"
    print(f"[{idx:02d}/30] {m.id}: {tag} in {dt:.2f}s | exp={exp}", flush=True)

print(f"\nFinal Result: {passed}/30 Passed! Failed: {failed}", flush=True)
