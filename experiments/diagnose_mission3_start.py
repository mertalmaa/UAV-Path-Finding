import math
import sys
import numpy as np
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from planner.roi import load_roi
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from planner.aircraft_profile import load_aircraft_profile
from planner.fixed_wing_envelope import FixedWingKinematicEnvelope
from planner.physical import PhysicalPose
from planner.pose_search import GoalPose, GoalTolerance, _candidate_trajectories, navigation_bearing_deg
from planner.trajectory_safety import TerrainInfluenceCache, evaluate_physical_trajectory_safety
from scripts.benchmark_missions import CACHE_DIR, CONFIG, FACTOR, PROFILE_PATH, SOURCE_DEM_PATH

roi = load_roi(CONFIG)
cache = load_terrain_cache(CACHE_DIR, roi, SOURCE_DEM_PATH)
terrain = build_terrain_query_from_cache(cache, roi, FACTOR)
profile = load_aircraft_profile(PROFILE_PATH)
envelope = FixedWingKinematicEnvelope()
infl_cache = TerrainInfluenceCache(terrain)

sx, sy = terrain.rowcol_to_xy(146, 15)
gx, gy = terrain.rowcol_to_xy(154, 60)
elev_s = float(terrain.query(sx, sy).elevation)
hdg = navigation_bearing_deg(sx, sy, gx, gy)
start = PhysicalPose(sx, sy, elev_s + 130.0, hdg)

print(f"Start Pose: ({sx:.1f}, {sy:.1f}, {start.z_msl_m:.1f} MSL), heading={hdg:.1f}, elev={elev_s:.1f}")

for prim, traj in _candidate_trajectories(start, envelope, CONFIG):
    if traj is None:
        print(f"  {prim:20s} -> NONE")
        continue
    safety = evaluate_physical_trajectory_safety(
        traj, terrain, CONFIG.min_agl_m, CONFIG.primitive_sample_spacing_m,
        planning_bounds=terrain.roi.bounds, lateral_buffer_m=CONFIG.lateral_buffer_m,
        terrain_influence_cache=infl_cache,
    )
    print(f"  {prim:20s} -> Safe: {safety.is_safe} (min_agl={safety.min_agl_m:.1f}m, reason={safety.failure_reason})")
