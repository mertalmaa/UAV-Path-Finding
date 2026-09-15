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

def trace_descent_reasons():
    roi = load_roi(CONFIG)
    cache = load_terrain_cache(CACHE_DIR, roi, SOURCE_DEM_PATH)
    terrain = build_terrain_query_from_cache(cache, roi, FACTOR)
    profile = load_aircraft_profile(PROFILE_PATH)
    envelope = FixedWingKinematicEnvelope()
    infl_cache = TerrainInfluenceCache(terrain)

    start_rc = (75, 5)
    goal_rc = (20, 5)

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

    # Inspect the terrain under the straight line between start and goal
    n_pts = 30
    for alpha in np.linspace(0, 1, n_pts):
        px = sx + alpha * (gx - sx)
        py = sy + alpha * (gy - sy)
        q = terrain.query(px, py)
        r, c = terrain.xy_to_rowcol(px, py)
        print(f"Dist {alpha*3300:4.0f}m: (r={r:2d}, c={c:2d}), Terrain Elev = {q.elevation:.1f} m MSL, Required Z >= {q.elevation + 100:.1f} m")

if __name__ == "__main__":
    trace_descent_reasons()
