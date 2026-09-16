"""Comprehensive flight dynamics, physical realism, and trajectory smoothing metrics.

Calculates continuous telemetry:
- Horizontal speed: 40.0 m/s
- Vertical rate: v_z(t) in [-5.0, +5.0] m/s
- Vertical acceleration: a_z(t) with S-curve continuous transition (|a_z| <= 0.5g)
- Bank angle: phi(t) = arctan(V^2 * kappa / g) <= 25.0 deg
- Bank angle rate: dphi/dt <= 15.0 deg/s
- Load factor: n = 1 / cos(phi) + a_z / g
- Terrain clearance: min AGL >= 100.0 m with lateral buffer
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple, Optional

import numpy as np

from planner.fixed_wing_envelope import (
    BANK_ANGLE_DEG,
    GRAVITY_MPS2,
    HORIZONTAL_SPEED_MPS,
    MAX_CLIMB_RATE_MPS,
    MAX_DESCENT_RATE_MPS,
)
from planner.physical import PhysicalTrajectory, angular_distance_deg
from planner.terrain import TerrainQuery
from planner.trajectory_safety import TerrainInfluenceCache


@dataclass(frozen=True)
class TelemetryPoint:
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


@dataclass(frozen=True)
class TrajectoryFidelityMetrics:
    total_distance_m: float
    total_time_s: float
    min_agl_m: float
    mean_agl_m: float
    max_altitude_m: float
    min_altitude_m: float
    max_climb_rate_mps: float
    max_descent_rate_mps: float
    max_vertical_accel_mps2: float
    max_vertical_accel_g: float
    max_bank_angle_deg: float
    max_bank_rate_deg_s: float
    max_load_factor_g: float
    min_turn_radius_m: float
    points: Tuple[TelemetryPoint, ...]


def apply_scurve_vertical_smoothing(
    s_arr: np.ndarray, z_raw: np.ndarray, speed_mps: float = HORIZONTAL_SPEED_MPS,
    max_az_mps2: float = 2.5,
) -> np.ndarray:
    """Apply S-curve (cubic spline / continuous acceleration) smoothing to piecewise-linear altitude."""
    if len(s_arr) < 3:
        return z_raw.copy()
    
    # Compute raw vertical velocity
    ds = np.diff(s_arr)
    ds = np.where(ds <= 0, 1e-6, ds)
    vz_raw = np.diff(z_raw) / ds * speed_mps
    vz_raw = np.pad(vz_raw, (0, 1), mode='edge')
    
    # Smooth vz with a moving gaussian/hanning window sized by max_az
    # delta_t_trans = delta_vz / max_az -> window_size_s
    win_len_m = max(20.0, speed_mps * (MAX_CLIMB_RATE_MPS / max_az_mps2))
    # Sample count for window
    mean_ds = float(np.mean(ds))
    k = max(3, int(win_len_m / mean_ds))
    if k % 2 == 0:
        k += 1
    
    window = np.hanning(k)
    window /= window.sum()
    vz_smooth = np.convolve(vz_raw, window, mode='same')
    
    # Reintegrate z from smoothed vz while preserving exact endpoints
    dz_smooth = vz_smooth[:-1] / speed_mps * np.diff(s_arr)
    z_smooth = np.zeros_like(z_raw)
    z_smooth[0] = z_raw[0]
    z_smooth[1:] = z_raw[0] + np.cumsum(dz_smooth)
    
    # Linear drift correction to match exact end altitude
    drift = z_smooth[-1] - z_raw[-1]
    if abs(drift) > 1e-6:
        correction = np.linspace(0.0, drift, len(z_smooth))
        z_smooth -= correction
        
    return z_smooth


def compute_trajectory_fidelity_metrics(
    trajectories: Sequence[PhysicalTrajectory],
    terrain: TerrainQuery,
    lateral_buffer_m: float = 60.0,
    min_agl_m: float = 100.0,
    speed_mps: float = HORIZONTAL_SPEED_MPS,
    max_az_mps2: float = 2.5,
    sample_step_m: float = 2.0,
) -> TrajectoryFidelityMetrics:
    """Calculate physical flight metrics along the continuous trajectory."""
    cache = TerrainInfluenceCache(terrain)
    field = cache.field(lateral_buffer_m)
    
    # 1. Flatten all trajectory samples
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

    # Dense interpolation along horizontal arc length
    total_dist = raw_s[-1]
    dense_s = np.arange(0.0, total_dist + sample_step_m, sample_step_m)
    if dense_s[-1] < total_dist:
        dense_s = np.append(dense_s, total_dist)
        
    dense_x = np.interp(dense_s, raw_s, raw_x)
    dense_y = np.interp(dense_s, raw_s, raw_y)
    dense_z_raw = np.interp(dense_s, raw_s, raw_z)
    dense_h_rad = np.interp(dense_s, raw_s, raw_h)
    dense_h_deg = np.degrees(dense_h_rad) % 360.0
    
    # 2. Apply S-curve vertical acceleration smoothing
    dense_z = apply_scurve_vertical_smoothing(dense_s, dense_z_raw, speed_mps, max_az_mps2)
    
    # 3. Terrain sampling
    dense_ground = np.zeros_like(dense_s)
    for i, (x, y) in enumerate(zip(dense_x, dense_y)):
        r, c = terrain.xy_to_rowcol(x, y)
        if terrain.in_bounds_rowcol(r, c) and field.valid[r, c]:
            dense_ground[i] = float(field.elevation_msl[r, c])
        else:
            dense_ground[i] = 0.0
            
    dense_agl = dense_z - dense_ground

    # 4. Kinematics Derivatives
    ds = np.gradient(dense_s)
    ds = np.where(ds <= 0, 1e-6, ds)
    dt = ds / speed_mps
    
    # Heading derivative -> Raw Curvature kappa = d_psi / ds
    dh_rad = np.gradient(dense_h_rad)
    # Wrap gradient to [-pi, pi]
    dh_rad = (dh_rad + np.pi) % (2 * np.pi) - np.pi
    raw_curvature = dh_rad / ds
    
    # Raw bank angle phi = arctan(V^2 * kappa / g)
    raw_bank_angles_deg = np.degrees(np.arctan((speed_mps ** 2) * raw_curvature / GRAVITY_MPS2))
    
    # Smooth bank angle to respect roll rate limit (dphi/dt <= 15 deg/s -> win_len ~= 60m)
    roll_win_m = max(30.0, speed_mps * (25.0 / 15.0))  # ~66.7m
    k_roll = max(5, int(roll_win_m / float(np.mean(ds))))
    if k_roll % 2 == 0:
        k_roll += 1
    roll_window = np.hanning(k_roll)
    roll_window /= roll_window.sum()
    bank_angles_deg = np.convolve(raw_bank_angles_deg, roll_window, mode='same')
    bank_angles_rad = np.radians(bank_angles_deg)
    
    # Bank angle rate dphi / dt
    dphi_deg = np.gradient(bank_angles_deg)
    bank_rate_deg_s = dphi_deg / dt
    
    # Effective curvature from smoothed bank angle
    curvature = np.tan(bank_angles_rad) * GRAVITY_MPS2 / (speed_mps ** 2)
    
    # Vertical velocity vz = dz / dt
    dz = np.gradient(dense_z)
    vz_mps = dz / dt
    
    # Vertical acceleration az = dvz / dt with smoothing window
    dvz = np.gradient(vz_mps)
    raw_az = dvz / dt
    k_az = max(3, int(30.0 / float(np.mean(ds))))
    if k_az % 2 == 0:
        k_az += 1
    az_window = np.hanning(k_az)
    az_window /= az_window.sum()
    az_mps2 = np.convolve(raw_az, az_window, mode='same')
    az_g = az_mps2 / GRAVITY_MPS2
    
    # Load factor n = 1 / cos(phi) + az / g
    cos_phi = np.cos(bank_angles_rad)
    cos_phi = np.where(np.abs(cos_phi) < 1e-3, 1e-3, cos_phi)
    load_factor = (1.0 / cos_phi) + az_g
    
    # Non-zero turn radius
    abs_curv = np.abs(curvature)
    turn_radii = np.where(abs_curv > 1e-6, 1.0 / np.maximum(abs_curv, 1e-6), np.inf)

    # 5. Pack points
    points = []
    for i in range(len(dense_s)):
        points.append(TelemetryPoint(
            s_m=float(dense_s[i]),
            t_s=float(dense_s[i] / speed_mps),
            x_m=float(dense_x[i]),
            y_m=float(dense_y[i]),
            z_msl_m=float(dense_z[i]),
            ground_m=float(dense_ground[i]),
            agl_m=float(dense_agl[i]),
            heading_deg=float(dense_h_deg[i]),
            curvature_rad_m=float(curvature[i]),
            bank_angle_deg=float(bank_angles_deg[i]),
            bank_rate_deg_s=float(bank_rate_deg_s[i]),
            vz_mps=float(vz_mps[i]),
            az_mps2=float(az_mps2[i]),
            load_factor_g=float(load_factor[i]),
        ))

    return TrajectoryFidelityMetrics(
        total_distance_m=float(total_dist),
        total_time_s=float(total_dist / speed_mps),
        min_agl_m=float(np.min(dense_agl)),
        mean_agl_m=float(np.mean(dense_agl)),
        max_altitude_m=float(np.max(dense_z)),
        min_altitude_m=float(np.min(dense_z)),
        max_climb_rate_mps=float(np.max(vz_mps)),
        max_descent_rate_mps=float(np.min(vz_mps)),
        max_vertical_accel_mps2=float(np.max(np.abs(az_mps2))),
        max_vertical_accel_g=float(np.max(np.abs(az_g))),
        max_bank_angle_deg=float(np.max(np.abs(bank_angles_deg))),
        max_bank_rate_deg_s=float(np.max(np.abs(bank_rate_deg_s))),
        max_load_factor_g=float(np.max(load_factor)),
        min_turn_radius_m=float(np.min(turn_radii)),
        points=tuple(points),
    )
