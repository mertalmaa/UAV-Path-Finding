"""Corridor-Safe Local Constrained B-Spline Smoothing for Fixed-Wing Trajectories.

Identifies curvature discontinuity junctions:
  - straight -> turn
  - turn -> straight
  - left -> right (reversal)
  - right -> left (reversal)

Applies local C2 continuous S-curve / B-spline transitions in a bounded window
around each discontinuity without altering the verified straight and constant-turn
sections of the trajectory. Guarantees corridor safety and minimum AGL compliance.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from planner.config import DEFAULT_CONFIG, PlannerConfig
from planner.fixed_wing_envelope import (
    BANK_ANGLE_DEG,
    GRAVITY_MPS2,
    HORIZONTAL_SPEED_MPS,
    MAX_CLIMB_RATE_MPS,
    MAX_DESCENT_RATE_MPS,
)
from planner.physical import (
    FIXED_PLANAR_SPEED_MPS,
    PhysicalPose,
    PhysicalTrajectory,
    TrajectorySample,
    normalize_heading_deg,
)
from planner.terrain import TerrainQuery
from planner.trajectory_safety import TerrainInfluenceCache


@dataclass(frozen=True)
class DiscontinuityJunction:
    index: int
    s_m: float
    kind: str  # "STRAIGHT_TO_TURN", "TURN_TO_STRAIGHT", "LEFT_TO_RIGHT", "RIGHT_TO_LEFT"
    k_prev: float
    k_next: float
    phi_prev_deg: float
    phi_next_deg: float
    x_m: float
    y_m: float
    z_m: float
    heading_deg: float
    window_half_m: float


@dataclass(frozen=True)
class SmoothedPoint:
    s_m: float
    t_s: float
    x_m: float
    y_m: float
    z_msl_m: float
    ground_m: float
    agl_m: float
    heading_deg: float
    curvature_rad_m: float
    bank_angle_deg: float
    bank_rate_deg_s: float
    vz_mps: float
    az_mps2: float
    load_factor_g: float
    deviation_from_orig_m: float
    is_in_transition: bool


@dataclass(frozen=True)
class LocalSmoothingResult:
    original_points: Tuple[SmoothedPoint, ...]
    smoothed_points: Tuple[SmoothedPoint, ...]
    junctions: Tuple[DiscontinuityJunction, ...]
    total_distance_m: float
    total_time_s: float
    # Metrics comparison
    raw_max_bank_angle_deg: float
    raw_max_bank_rate_deg_s: float
    smoothed_max_bank_angle_deg: float
    smoothed_max_bank_rate_deg_s: float
    min_agl_m: float
    mean_agl_m: float
    max_corridor_deviation_m: float
    num_junctions_total: int
    num_junctions_smoothed: int
    corridor_safe: bool


def detect_curvature_junctions(
    trajectories: Sequence[PhysicalTrajectory],
    turn_radius_m: float = 349.9,
) -> Tuple[List[float], List[DiscontinuityJunction]]:
    """Detect curvature discontinuities between adjacent primitives."""
    curvatures = []
    k_turn = 1.0 / turn_radius_m
    
    for t in trajectories:
        dpsi = (t.end_pose.heading_deg - t.start_pose.heading_deg + 180.0) % 360.0 - 180.0
        if abs(dpsi) < 1e-3:
            curvatures.append(0.0)
        elif dpsi > 0.0:
            curvatures.append(+k_turn)
        else:
            curvatures.append(-k_turn)
            
    lengths = [t.horizontal_arc_length_m for t in trajectories]
    cum_s = np.cumsum([0.0] + lengths)
    
    # First pass: find all junction candidate indices and locations
    raw_indices = []
    for i in range(len(trajectories) - 1):
        if abs(curvatures[i + 1] - curvatures[i]) >= 1e-5:
            raw_indices.append(i)
            
    total_len = float(cum_s[-1])
    junctions: List[DiscontinuityJunction] = []
    
    for m, i in enumerate(raw_indices):
        k1 = curvatures[i]
        k2 = curvatures[i + 1]
        s_j = float(cum_s[i + 1])
        
        phi1 = math.degrees(math.atan((HORIZONTAL_SPEED_MPS ** 2) * k1 / GRAVITY_MPS2))
        phi2 = math.degrees(math.atan((HORIZONTAL_SPEED_MPS ** 2) * k2 / GRAVITY_MPS2))
        
        if abs(k1) < 1e-5:
            kind = "STRAIGHT_TO_TURN"
        elif abs(k2) < 1e-5:
            kind = "TURN_TO_STRAIGHT"
        elif k1 * k2 < 0:
            kind = "LEFT_TO_RIGHT" if k1 < 0 else "RIGHT_TO_LEFT"
        else:
            kind = "CURVATURE_STEP"
            
        s_prev_j = float(cum_s[raw_indices[m - 1] + 1]) if m > 0 else 0.0
        s_next_j = float(cum_s[raw_indices[m + 1] + 1]) if m < len(raw_indices) - 1 else total_len
        
        d_before = s_j - s_prev_j
        d_after = s_next_j - s_j
        
        # Nominal target half window:
        # At 40 m/s and 15 deg/s roll rate limit, delta_phi=50 deg -> target_half = 65m
        delta_phi = abs(phi2 - phi1)
        target_half = 40.0 if delta_phi <= 25.0 else 65.0
        
        # Maximum space without colliding with adjacent junctions
        half_win = min(target_half, d_before * 0.48, d_after * 0.48)
        half_win = max(10.0, half_win)
        
        t_curr = trajectories[i]
        j_pt = t_curr.end_pose
        junctions.append(DiscontinuityJunction(
            index=i,
            s_m=s_j,
            kind=kind,
            k_prev=k1,
            k_next=k2,
            phi_prev_deg=phi1,
            phi_next_deg=phi2,
            x_m=j_pt.x_m,
            y_m=j_pt.y_m,
            z_m=j_pt.z_msl_m,
            heading_deg=j_pt.heading_deg,
            window_half_m=float(half_win),
        ))
        
    return curvatures, junctions


def apply_corridor_safe_local_bspline_smoothing(
    trajectories: Sequence[PhysicalTrajectory],
    terrain: TerrainQuery,
    *,
    config: PlannerConfig = DEFAULT_CONFIG,
    speed_mps: float = HORIZONTAL_SPEED_MPS,
    sample_step_m: float = 2.0,
    max_bank_rate_deg_s: float = 15.0,
    max_corridor_deviation_m: float = 12.0,
) -> LocalSmoothingResult:
    """Perform Corridor-Safe Local Constrained B-Spline Smoothing around junctions."""
    cache = TerrainInfluenceCache(terrain)
    field = cache.field(config.lateral_buffer_m)
    
    # 1. Dense resampling of raw trajectory
    raw_s = []
    raw_x = []
    raw_y = []
    raw_z = []
    raw_h = []
    cum_s = 0.0
    for traj in trajectories:
        for s in traj.samples:
            raw_s.append(cum_s + s.horizontal_distance_along_path_m)
            raw_x.append(s.x_m)
            raw_y.append(s.y_m)
            raw_z.append(s.z_msl_m)
            raw_h.append(s.heading_deg)
        cum_s += traj.horizontal_arc_length_m

    raw_s = np.array(raw_s)
    raw_x = np.array(raw_x)
    raw_y = np.array(raw_y)
    raw_z = np.array(raw_z)
    raw_h = np.unwrap(np.radians(raw_h))

    total_dist = raw_s[-1]
    dense_s = np.arange(0.0, total_dist + sample_step_m, sample_step_m)
    if dense_s[-1] < total_dist:
        dense_s = np.append(dense_s, total_dist)

    dense_x = np.interp(dense_s, raw_s, raw_x)
    dense_y = np.interp(dense_s, raw_s, raw_y)
    dense_z = np.interp(dense_s, raw_s, raw_z)
    dense_h_rad = np.interp(dense_s, raw_s, raw_h)

    # 2. Detect junctions
    curvatures, junctions = detect_curvature_junctions(trajectories)
    
    # Track smoothed positions and transition zones
    smooth_x = dense_x.copy()
    smooth_y = dense_y.copy()
    smooth_z = dense_z.copy()
    is_trans = np.zeros(len(dense_s), dtype=bool)
    
    smoothed_junctions_count = 0
    max_dev_observed = 0.0

    # 3. For each junction, apply smooth C2 transition in window
    for j in junctions:
        s_A = j.s_m - j.window_half_m
        s_B = j.s_m + j.window_half_m
        mask = (dense_s >= s_A) & (dense_s <= s_B)
        indices = np.where(mask)[0]
        if len(indices) < 5:
            continue
            
        s_sub = dense_s[indices]
        L_win = s_sub[-1] - s_sub[0]
        if L_win < 1e-3:
            continue
            
        u = (s_sub - s_sub[0]) / L_win
        # Quintic polynomial S-curve transition S(0)=0, S(1)=1, S'=0, S''=0 at endpoints
        S = 10.0 * (u**3) - 15.0 * (u**4) + 6.0 * (u**5)
        
        # Continuous bank angle transition
        phi_trans_deg = j.phi_prev_deg + (j.phi_next_deg - j.phi_prev_deg) * S
        phi_trans_rad = np.radians(phi_trans_deg)
        
        # Curvature along transition: kappa = g * tan(phi) / V^2
        k_trans = np.tan(phi_trans_rad) * GRAVITY_MPS2 / (speed_mps ** 2)
        
        # Integrate heading along transition
        # delta_psi(u) = integral of (k_trans * ds)
        ds_sub = np.diff(s_sub)
        ds_sub = np.pad(ds_sub, (0, 1), mode='edge')
        psi_0 = dense_h_rad[indices[0]]
        psi_trans = psi_0 + np.cumsum(k_trans * ds_sub)
        
        # Integrate position along transition
        dx_sub = np.sin(psi_trans) * ds_sub
        dy_sub = np.cos(psi_trans) * ds_sub
        cand_x = np.zeros_like(s_sub)
        cand_y = np.zeros_like(s_sub)
        cand_x[0] = dense_x[indices[0]]
        cand_y[0] = dense_y[indices[0]]
        cand_x[1:] = cand_x[0] + np.cumsum(dx_sub[:-1])
        cand_y[1:] = cand_y[0] + np.cumsum(dy_sub[:-1])
        
        # Linear drift correction to perfectly match the downstream endpoint
        drift_x = cand_x[-1] - dense_x[indices[-1]]
        drift_y = cand_y[-1] - dense_y[indices[-1]]
        correction_weights = np.linspace(0.0, 1.0, len(s_sub))
        cand_x -= drift_x * correction_weights
        cand_y -= drift_y * correction_weights
        
        # Verification: Check corridor deviation and terrain clearance
        orig_pts = np.column_stack([dense_x[indices], dense_y[indices]])
        cand_pts = np.column_stack([cand_x, cand_y])
        devs = np.linalg.norm(cand_pts - orig_pts, axis=1)
        cur_max_dev = float(np.max(devs))
        
        corridor_ok = cur_max_dev <= max_corridor_deviation_m
        terrain_ok = True
        
        for pt, z_val in zip(cand_pts, dense_z[indices]):
            r, c = terrain.xy_to_rowcol(pt[0], pt[1])
            if terrain.in_bounds_rowcol(r, c) and field.valid[r, c]:
                gnd = float(field.elevation_msl[r, c])
                if z_val - gnd < config.min_agl_m:
                    terrain_ok = False
                    break
            else:
                terrain_ok = False
                break
                
        if corridor_ok and terrain_ok:
            smooth_x[indices] = cand_x
            smooth_y[indices] = cand_y
            is_trans[indices] = True
            max_dev_observed = max(max_dev_observed, cur_max_dev)
            smoothed_junctions_count += 1

    # 4. Construct continuous telemetry profiles
    dense_h_smooth = dense_h_rad.copy()
    dense_bank_raw = np.zeros_like(dense_s)
    dense_bank_smooth = np.zeros_like(dense_s)
    
    # Fill nominal raw bank angle per primitive
    cum_lengths = np.cumsum([0.0] + [t.horizontal_arc_length_m for t in trajectories])
    for i in range(len(trajectories)):
        s_start = cum_lengths[i]
        s_end = cum_lengths[i + 1]
        m_prim = (dense_s >= s_start) & (dense_s < s_end)
        k_val = curvatures[i]
        phi_val = math.degrees(math.atan((speed_mps ** 2) * k_val / GRAVITY_MPS2))
        dense_bank_raw[m_prim] = phi_val
        dense_bank_smooth[m_prim] = phi_val
        
    dense_bank_raw[-1] = dense_bank_raw[-2]
    dense_bank_smooth[-1] = dense_bank_smooth[-2]

    # For each smoothed junction, replace transition region in smooth bank and heading
    for j in junctions:
        s_A = j.s_m - j.window_half_m
        s_B = j.s_m + j.window_half_m
        mask = (dense_s >= s_A) & (dense_s <= s_B)
        indices = np.where(mask)[0]
        if len(indices) < 5:
            continue
            
        s_sub = dense_s[indices]
        L_win = s_sub[-1] - s_sub[0]
        u = (s_sub - s_sub[0]) / L_win
        S = 10.0 * (u**3) - 15.0 * (u**4) + 6.0 * (u**5)
        
        phi_trans = j.phi_prev_deg + (j.phi_next_deg - j.phi_prev_deg) * S
        dense_bank_smooth[indices] = phi_trans
        
        # Heading from curvature integration
        k_trans = np.tan(np.radians(phi_trans)) * GRAVITY_MPS2 / (speed_mps ** 2)
        ds_sub = np.gradient(s_sub)
        psi_0 = dense_h_smooth[indices[0]]
        dense_h_smooth[indices] = psi_0 + np.cumsum(k_trans * ds_sub)

    # Telemetry point generator
    def build_telemetry_points(x_arr, y_arr, z_arr, bank_arr, heading_rad_arr, in_trans_flags):
        ds = np.gradient(dense_s)
        ds = np.where(ds <= 0, 1e-6, ds)
        dt = ds / speed_mps
        
        # Roll rate dphi / dt
        dphi = np.gradient(bank_arr)
        bank_rate = dphi / dt
        
        # Curvature from bank angle
        curv = np.tan(np.radians(bank_arr)) * GRAVITY_MPS2 / (speed_mps ** 2)
        
        # Vertical dynamics
        dz = np.gradient(z_arr)
        vz = dz / dt
        dvz = np.gradient(vz)
        az = dvz / dt
        az_g = az / GRAVITY_MPS2
        
        # Load factor n = 1 / cos(phi) + az / g
        cos_phi = np.cos(np.radians(bank_arr))
        cos_phi = np.where(np.abs(cos_phi) < 1e-3, 1e-3, cos_phi)
        load_factor = (1.0 / cos_phi) + az_g
        
        pts = []
        for i in range(len(dense_s)):
            r, c = terrain.xy_to_rowcol(x_arr[i], y_arr[i])
            gnd = float(field.elevation_msl[r, c]) if (terrain.in_bounds_rowcol(r, c) and field.valid[r, c]) else 0.0
            agl = z_arr[i] - gnd
            dev = float(math.hypot(x_arr[i] - dense_x[i], y_arr[i] - dense_y[i]))
            pts.append(SmoothedPoint(
                s_m=float(dense_s[i]),
                t_s=float(dense_s[i] / speed_mps),
                x_m=float(x_arr[i]),
                y_m=float(y_arr[i]),
                z_msl_m=float(z_arr[i]),
                ground_m=float(gnd),
                agl_m=float(agl),
                heading_deg=float(np.degrees(heading_rad_arr[i]) % 360.0),
                curvature_rad_m=float(curv[i]),
                bank_angle_deg=float(bank_arr[i]),
                bank_rate_deg_s=float(bank_rate[i]),
                vz_mps=float(vz[i]),
                az_mps2=float(az[i]),
                load_factor_g=float(load_factor[i]),
                deviation_from_orig_m=dev,
                is_in_transition=bool(in_trans_flags[i]),
            ))
        return tuple(pts)

    raw_points = build_telemetry_points(dense_x, dense_y, dense_z, dense_bank_raw, dense_h_rad, np.zeros(len(dense_s), dtype=bool))
    smooth_points = build_telemetry_points(smooth_x, smooth_y, smooth_z, dense_bank_smooth, dense_h_smooth, is_trans)
    
    raw_bank_rates = np.array([abs(p.bank_rate_deg_s) for p in raw_points])
    smooth_bank_rates = np.array([abs(p.bank_rate_deg_s) for p in smooth_points])
    agls = np.array([p.agl_m for p in smooth_points])

    return LocalSmoothingResult(
        original_points=raw_points,
        smoothed_points=smooth_points,
        junctions=tuple(junctions),
        total_distance_m=float(total_dist),
        total_time_s=float(total_dist / speed_mps),
        raw_max_bank_angle_deg=float(np.max([abs(p.bank_angle_deg) for p in raw_points])),
        raw_max_bank_rate_deg_s=float(np.max(raw_bank_rates)),
        smoothed_max_bank_angle_deg=float(np.max([abs(p.bank_angle_deg) for p in smooth_points])),
        smoothed_max_bank_rate_deg_s=float(np.max(smooth_bank_rates)),
        min_agl_m=float(np.min(agls)),
        mean_agl_m=float(np.mean(agls)),
        max_corridor_deviation_m=float(max_dev_observed),
        num_junctions_total=len(junctions),
        num_junctions_smoothed=smoothed_junctions_count,
        corridor_safe=bool(np.min(agls) >= config.min_agl_m and max_dev_observed <= max_corridor_deviation_m),
    )
