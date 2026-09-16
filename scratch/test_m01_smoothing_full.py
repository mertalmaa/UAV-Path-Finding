import sys
from pathlib import Path
sys.path.insert(0, ".")

import math
import numpy as np
import dataclasses
from planner.config import DEFAULT_CONFIG
from planner.pose_search import GoalPose, GoalTolerance, navigation_bearing_deg, pose_aware_astar_search
from planner.physical import PhysicalPose, PhysicalTrajectory, TrajectorySample
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from planner.terrain_following import optimize_terrain_following_altitudes
from planner.trajectory_safety import TerrainInfluenceCache
from scripts.bilecik_missions_spec import BILECIK_MISSIONS

def quintic_hermite_eval(P_A, T_A, N_A, kappa_A,
                         P_B, T_B, N_B, kappa_B,
                         L_orig, u):
    """Evaluate 2D quintic Hermite spline at parameter array u in [0, 1]."""
    u = np.asarray(u)
    u2 = u * u
    u3 = u2 * u
    u4 = u3 * u
    u5 = u4 * u
    
    h0 = 1.0 - 10.0*u3 + 15.0*u4 - 6.0*u5
    h1 = u - 6.0*u3 + 8.0*u4 - 3.0*u5
    h2 = 0.5*u2 - 1.5*u3 + 1.5*u4 - 0.5*u5
    h3 = 0.5*u3 - u4 + 0.5*u5
    h4 = -4.0*u3 + 7.0*u4 - 3.0*u5
    h5 = 10.0*u3 - 15.0*u4 + 6.0*u5
    
    sigma = L_orig
    V_A = sigma * T_A
    A_A = (sigma ** 2) * kappa_A * N_A
    V_B = sigma * T_B
    A_B = (sigma ** 2) * kappa_B * N_B
    
    pos = (np.outer(h0, P_A) + np.outer(h1, V_A) + np.outer(h2, A_A) +
           np.outer(h3, A_B) + np.outer(h4, V_B) + np.outer(h5, P_B))
           
    dh0 = -30.0*u2 + 60.0*u3 - 30.0*u4
    dh1 = 1.0 - 18.0*u2 + 32.0*u3 - 15.0*u4
    dh2 = u - 4.5*u2 + 6.0*u3 - 2.5*u4
    dh3 = 1.5*u2 - 4.0*u3 + 2.5*u4
    dh4 = -12.0*u2 + 28.0*u3 - 15.0*u4
    dh5 = 30.0*u2 - 60.0*u3 + 30.0*u4
    vel = (np.outer(dh0, P_A) + np.outer(dh1, V_A) + np.outer(dh2, A_A) +
           np.outer(dh3, A_B) + np.outer(dh4, V_B) + np.outer(dh5, P_B))
           
    d2h0 = -60.0*u + 180.0*u2 - 120.0*u3
    d2h1 = -36.0*u + 96.0*u2 - 60.0*u3
    d2h2 = 1.0 - 9.0*u + 18.0*u2 - 10.0*u3
    d2h3 = 3.0*u - 12.0*u2 + 10.0*u3
    d2h4 = -24.0*u + 84.0*u2 - 60.0*u3
    d2h5 = 60.0*u - 180.0*u2 + 120.0*u3
    acc = (np.outer(d2h0, P_A) + np.outer(d2h1, V_A) + np.outer(d2h2, A_A) +
           np.outer(d2h3, A_B) + np.outer(d2h4, V_B) + np.outer(d2h5, P_B))
           
    xp = vel[:, 0]
    yp = vel[:, 1]
    xpp = acc[:, 0]
    ypp = acc[:, 1]
    speed_sq = xp**2 + yp**2
    # Navigation curvature kappa = (yp * xpp - xp * ypp) / speed_sq^1.5
    curv = (yp * xpp - xp * ypp) / (speed_sq ** 1.5)
    headings_deg = np.degrees(np.arctan2(xp, yp)) % 360.0
    
    return pos, vel, acc, curv, headings_deg

# Load M01
config = dataclasses.replace(DEFAULT_CONFIG, min_agl_m=100.0, lateral_buffer_m=60.0, search_heuristic_weight=1.05, enable_terrain_guidance=True)
terrain = TerrainQuery(load_roi(config))
field = TerrainInfluenceCache(terrain).field(config.lateral_buffer_m)
mission = BILECIK_MISSIONS[0]

def altitude(xy, offset):
    r, c = terrain.xy_to_rowcol(*xy)
    return float(field.elevation_msl[r, c]) + max(130.0, offset)

start = PhysicalPose(*mission.start_xy, altitude(mission.start_xy, mission.start_alt_offset_m), 0.0)
goal = GoalPose(*mission.goal_xy, altitude(mission.goal_xy, mission.goal_alt_offset_m))
result = pose_aware_astar_search(start, goal, terrain, config=config, goal_tolerance=GoalTolerance(90.0, 15.0), max_expansions=30000, max_search_time_s=10.0)
profile = optimize_terrain_following_altitudes(result.trajectories, terrain, config=config, target_agl_m=120.0)

# Dense resample at 2.0m
trajectories = profile.trajectories
raw_s, raw_x, raw_y, raw_z, raw_h = [], [], [], [], []
cum_s = 0.0
prim_lengths = []
for t in trajectories:
    prim_lengths.append(t.horizontal_arc_length_m)
    for s in t.samples:
        raw_s.append(cum_s + s.horizontal_distance_along_path_m)
        raw_x.append(s.x_m)
        raw_y.append(s.y_m)
        raw_z.append(s.z_msl_m)
        raw_h.append(s.heading_deg)
    cum_s += t.horizontal_arc_length_m

raw_s = np.array(raw_s)
raw_x = np.array(raw_x)
raw_y = np.array(raw_y)
raw_z = np.array(raw_z)
raw_h = np.unwrap(np.radians(raw_h))

total_dist = raw_s[-1]
sample_step = 1.0
dense_s = np.arange(0.0, total_dist + sample_step, sample_step)
dense_x = np.interp(dense_s, raw_s, raw_x)
dense_y = np.interp(dense_s, raw_s, raw_y)
dense_z = np.interp(dense_s, raw_s, raw_z)

# Find junctions
curvatures = []
R_turn = 349.9
for t in trajectories:
    dpsi = (t.end_pose.heading_deg - t.start_pose.heading_deg + 180)%360 - 180
    if abs(dpsi) < 1e-3:
        curvatures.append(0.0)
    elif dpsi > 0:
        curvatures.append(+1.0 / R_turn)
    else:
        curvatures.append(-1.0 / R_turn)

cum_lengths = np.cumsum([0.0] + prim_lengths)
smoothed_x = dense_x.copy()
smoothed_y = dense_y.copy()
smoothed_z = dense_z.copy()

max_deviation = 0.0
safe_smoothed = 0
total_junctions = 0

for i in range(len(trajectories) - 1):
    k_curr = curvatures[i]
    k_next = curvatures[i+1]
    if abs(k_next - k_curr) < 1e-5:
        continue
    total_junctions += 1
    s_j = cum_lengths[i+1]
    
    # Available length on both sides
    l_pre_avail = prim_lengths[i]
    l_post_avail = prim_lengths[i+1]
    
    # Window size based on curvature delta
    delta_k = abs(k_next - k_curr)
    target_half = 35.0 if delta_k < 0.004 else 45.0
    half_win = min(target_half, l_pre_avail * 0.45, l_post_avail * 0.45)
    
    s_A = s_j - half_win
    s_B = s_j + half_win
    
    mask = (dense_s >= s_A) & (dense_s <= s_B)
    if not np.any(mask) or np.sum(mask) < 4:
        continue
        
    P_A = np.array([np.interp(s_A, dense_s, dense_x), np.interp(s_A, dense_s, dense_y)])
    P_B = np.array([np.interp(s_B, dense_s, dense_y), np.interp(s_B, dense_s, dense_y)]) # wait typo in test script
    # let's interpolate properly
    P_A = np.array([np.interp(s_A, dense_s, dense_x), np.interp(s_A, dense_s, dense_y)])
    P_B = np.array([np.interp(s_B, dense_s, dense_x), np.interp(s_B, dense_s, dense_y)])
    
    th_A = np.interp(s_A, raw_s, raw_h)
    th_B = np.interp(s_B, raw_s, raw_h)
    
    T_A = np.array([math.sin(th_A), math.cos(th_A)])
    N_A = np.array([math.cos(th_A), -math.sin(th_A)])
    T_B = np.array([math.sin(th_B), math.cos(th_B)])
    N_B = np.array([math.cos(th_B), -math.sin(th_B)])
    
    u_vals = (dense_s[mask] - s_A) / (s_B - s_A)
    spline_pos, _, _, _, _ = quintic_hermite_eval(
        P_A, T_A, N_A, k_curr,
        P_B, T_B, N_B, k_next,
        s_B - s_A, u_vals
    )
    
    # Check deviation
    orig_pts = np.column_stack([dense_x[mask], dense_y[mask]])
    dev = np.linalg.norm(spline_pos - orig_pts, axis=1)
    cur_max_dev = np.max(dev)
    
    # Check terrain clearance
    is_safe = True
    z_interp = dense_z[mask]
    for pt, z_val in zip(spline_pos, z_interp):
        r, c = terrain.xy_to_rowcol(pt[0], pt[1])
        if terrain.in_bounds_rowcol(r, c) and field.valid[r, c]:
            gnd = float(field.elevation_msl[r, c])
            if z_val - gnd < 100.0:
                is_safe = False
                break
        else:
            is_safe = False
            break
            
    if is_safe and cur_max_dev <= 15.0:
        smoothed_x[mask] = spline_pos[:, 0]
        smoothed_y[mask] = spline_pos[:, 1]
        max_deviation = max(max_deviation, cur_max_dev)
        safe_smoothed += 1

print(f"Junctions smoothed: {safe_smoothed} / {total_junctions}")
print(f"Max corridor deviation observed: {max_deviation:.2f} meters")

# Calculate curvature and roll rate along smoothed vs original
dx = np.gradient(smoothed_x, dense_s)
dy = np.gradient(smoothed_y, dense_s)
d2x = np.gradient(dx, dense_s)
d2y = np.gradient(dy, dense_s)
speed = np.sqrt(dx**2 + dy**2)
curv_smooth = (dy * d2x - dx * d2y) / (speed**3 + 1e-12)

V = 40.0
g = 9.80665
bank_smooth = np.degrees(np.arctan((V**2) * curv_smooth / g))
dt = (dense_s[1] - dense_s[0]) / V
dphi_smooth = np.gradient(bank_smooth) / dt

print(f"Smoothed Max Bank Angle: {np.max(np.abs(bank_smooth)):.1f} deg")
print(f"Smoothed Max Roll Rate:  {np.max(np.abs(dphi_smooth)):.1f} deg/s")
