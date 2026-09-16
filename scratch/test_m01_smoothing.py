import numpy as np
import math
import dataclasses
from planner.config import DEFAULT_CONFIG
from planner.pose_search import GoalPose, GoalTolerance, navigation_bearing_deg, pose_aware_astar_search
from planner.physical import PhysicalPose, PhysicalTrajectory, TrajectorySample
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from planner.terrain_following import optimize_terrain_following_altitudes
from planner.trajectory_safety import TerrainInfluenceCache
from scripts.bilecik_missions_spec import BILECIK_MISSIONS

# 1. Setup config and load M01
config = dataclasses.replace(DEFAULT_CONFIG, min_agl_m=100.0, lateral_buffer_m=60.0, search_heuristic_weight=1.05, enable_terrain_guidance=True)
terrain = TerrainQuery(load_roi(config))
field = TerrainInfluenceCache(terrain).field(config.lateral_buffer_m)
mission = BILECIK_MISSIONS[0] # M01

def altitude(xy, offset):
    r, c = terrain.xy_to_rowcol(*xy)
    return float(field.elevation_msl[r, c]) + max(130.0, offset)

start = PhysicalPose(*mission.start_xy, altitude(mission.start_xy, mission.start_alt_offset_m), 0.0)
goal = GoalPose(*mission.goal_xy, altitude(mission.goal_xy, mission.goal_alt_offset_m))
result = pose_aware_astar_search(start, goal, terrain, config=config, goal_tolerance=GoalTolerance(90.0, 15.0), max_expansions=30000, max_search_time_s=10.0)
profile = optimize_terrain_following_altitudes(result.trajectories, terrain, config=config, target_agl_m=120.0)

print(f"M01 Profile Success: {profile.success}, trajectories: {len(profile.trajectories)}")

# Extract dense samples
raw_s = []
raw_x = []
raw_y = []
raw_z = []
raw_h = []
cum_s = 0.0
traj_indices = []

for t_idx, traj in enumerate(profile.trajectories):
    for s in traj.samples:
        raw_s.append(cum_s + s.horizontal_distance_along_path_m)
        raw_x.append(s.x_m)
        raw_y.append(s.y_m)
        raw_z.append(s.z_msl_m)
        raw_h.append(s.heading_deg)
        traj_indices.append(t_idx)
    cum_s += traj.horizontal_arc_length_m

raw_s = np.array(raw_s)
raw_x = np.array(raw_x)
raw_y = np.array(raw_y)
raw_z = np.array(raw_z)
raw_h = np.array(raw_h)

# Find junctions where primitive curvature changes
prims = profile.trajectories
curvatures = []
R_turn = 349.9
for t in prims:
    dpsi = (t.end_pose.heading_deg - t.start_pose.heading_deg + 180)%360 - 180
    if abs(dpsi) < 1e-3:
        curvatures.append(0.0)
    elif dpsi > 0:
        curvatures.append(+1.0 / R_turn)
    else:
        curvatures.append(-1.0 / R_turn)

print(f"Total primitives: {len(prims)}")
junctions = []
cum_dist = 0.0
for i in range(len(prims) - 1):
    cum_dist += prims[i].horizontal_arc_length_m
    k_curr = curvatures[i]
    k_next = curvatures[i+1]
    if abs(k_next - k_curr) > 1e-5:
        junctions.append((i, cum_dist, k_curr, k_next))

print(f"Detected {len(junctions)} curvature discontinuity junctions.")
for j in junctions[:5]:
    kind = "STRAIGHT->TURN" if j[2]==0 else ("TURN->STRAIGHT" if j[3]==0 else ("REVERSAL" if j[2]*j[3]<0 else "OTHER"))
    print(f"  Junction at s={j[1]:.1f}m: prim {j[0]}->{j[0]+1}, kind={kind}, k1={j[2]:+.5f}, k2={j[3]:+.5f}")
