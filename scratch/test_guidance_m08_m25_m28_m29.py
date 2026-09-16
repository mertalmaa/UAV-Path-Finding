import dataclasses
from pathlib import Path
import time
import sys

from planner.config import DEFAULT_CONFIG
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from planner.physical import PhysicalPose
from planner.pose_search import GoalPose, GoalTolerance, pose_aware_astar_search, navigation_bearing_deg
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
)
roi = load_roi(base_cfg)
tq = TerrainQuery(roi)
cache = TerrainInfluenceCache(tq)
field = cache.field(60.0)

for guided in [False, True]:
    print(f"\n================ TESTING GUIDED={guided} ================", flush=True)
    cfg = dataclasses.replace(base_cfg, enable_terrain_guidance=guided)
    for mid in ['M08', 'M25', 'M28', 'M29']:
        m = [x for x in BILECIK_MISSIONS if x.id == mid][0]
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
        
        t0 = time.perf_counter()
        res = pose_aware_astar_search(
            start_pose, goal_pose, tq, goal_tolerance=tol,
            config=cfg, max_expansions=30000, max_search_time_s=10.0
        )
        dt = time.perf_counter() - t0
        print(f"  {mid}: success={res.success} in {dt:.2f}s | reason={res.termination_reason} | expanded={res.expanded_nodes}", flush=True)
