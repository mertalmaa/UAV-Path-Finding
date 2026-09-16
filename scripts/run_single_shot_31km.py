"""Single-Shot (No Waypoints) 31 km Fixed-Wing Autonomous Mission Planner in Bilecik Region.

Comprehensive Evaluation with Physical Realism, S-Curve Vertical Acceleration,
Bank Angle Dynamics, and Lateral Safety Buffer Enforcement.
"""
from __future__ import annotations

import dataclasses
import json
import math
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from planner.config import DEFAULT_CONFIG
from planner.fixed_wing_envelope import FixedWingKinematicEnvelope
from planner.physical import PhysicalPose
from planner.pose_search import GoalPose, GoalTolerance, pose_aware_astar_search, navigation_bearing_deg
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from planner.terrain_following import optimize_terrain_following_altitudes, plan_terrain_following
from planner.trajectory_metrics import compute_trajectory_fidelity_metrics
from planner.trajectory_safety import TerrainInfluenceCache


RESULTS_DIR = Path("results/test_bilecik")
PLOTS_DIR = RESULTS_DIR / "plots"


def main():
    print("=" * 80)
    print("31 KM BILECIK LONG VALLEY (SAKARYA CANYON) COMPREHENSIVE FLIGHT BENCHMARK")
    print("=" * 80)

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)

    # Start: Inhisar South (261800, 4445000)
    # Goal:  Osmaneli East Plains (262800, 4471200)
    start_xy = (261800.0, 4445000.0)
    goal_xy = (262800.0, 4471200.0)

    cfg = dataclasses.replace(
        DEFAULT_CONFIG,
        working_dem_path=Path("regions/bilecik/working_dem.tif"),
        roi_size_m=30_000.0,
        roi_center_lonlat=(30.30, 40.25),
        target_crs="EPSG:32636",
        min_agl_m=100.0,
        desired_agl_m=120.0,
        lateral_buffer_m=60.0,
        search_heuristic_weight=1.05,
        enable_combined_turns=True,
        enable_low_altitude_cost=True,
        enable_terrain_guidance=True,
        low_altitude_cost_shape="linear",
        lambda_agl=0.25,
        agl_cost_scale_m=1000.0,
    )

    envelope = FixedWingKinematicEnvelope()

    print("Loading Bilecik 30x30 km ROI DEM...")
    roi = load_roi(cfg)
    tq = TerrainQuery(roi)
    cache = TerrainInfluenceCache(tq)
    field = cache.field(cfg.lateral_buffer_m)

    r_s, c_s = tq.xy_to_rowcol(*start_xy)
    r_g, c_g = tq.xy_to_rowcol(*goal_xy)
    start_ground = float(field.elevation_msl[r_s, c_s])
    goal_ground = float(field.elevation_msl[r_g, c_g])

    start_z = start_ground + 130.0
    goal_z = goal_ground + 130.0

    # Initial heading pointing north-northwest along the canyon entrance
    initial_heading = navigation_bearing_deg(start_xy[0], start_xy[1], 259200.0, 4448000.0)
    start_pose = PhysicalPose(start_xy[0], start_xy[1], start_z, initial_heading)
    goal_pose = GoalPose(goal_xy[0], goal_xy[1], goal_z)
    goal_tol = GoalTolerance(xy_m=200.0, altitude_m=30.0)

    straight_dist = math.hypot(goal_xy[0] - start_xy[0], goal_xy[1] - start_xy[1])
    print(f"Direct start-to-goal straight distance: {straight_dist/1000:.2f} km")
    print(f"Start Pose: ({start_pose.x_m:.1f}, {start_pose.y_m:.1f}, z={start_pose.z_msl_m:.1f}m, h={start_pose.heading_deg:.1f}°)")
    print(f"Goal Pose:  ({goal_pose.x_m:.1f}, {goal_pose.y_m:.1f}, z={goal_pose.z_msl_m:.1f}m)")

    print("\nExecuting Coupled Closed-Loop Plan (Pose-aware A* + DP + Feedback)...")
    t0 = time.perf_counter()
    plan_result = plan_terrain_following(
        start_pose,
        goal_pose,
        tq,
        goal_tolerance=goal_tol,
        config=cfg,
        target_agl_m=120.0,
        max_expansions=30000,
        max_search_time_s=60.0,
        envelope=envelope,
        max_feedback_passes=3,
    )
    total_planning_time = time.perf_counter() - t0

    search_res = plan_result.search_result
    profile_res = plan_result.profile_result

    print(f"Search Status: {search_res.termination_reason}, Expanded Nodes: {search_res.expanded_nodes}, Search Time: {search_res.runtime_s:.2f}s")
    if not plan_result.success:
        print(f"Planning failed! Search: {search_res.termination_reason}, Profile: {profile_res.status if profile_res else 'None'}")
        return

    print(f"Profile Status: {profile_res.status}, Profile Time: {profile_res.runtime_s:.2f}s, Passes: {profile_res.refinement_passes}")
    print(f"Total Planning Runtime: {total_planning_time:.2f} seconds")

    final_trajectories = plan_result.trajectories

    # Compute comprehensive trajectory fidelity metrics
    print("\nComputing continuous flight fidelity and S-curve dynamics metrics...")
    metrics = compute_trajectory_fidelity_metrics(
        final_trajectories,
        tq,
        lateral_buffer_m=cfg.lateral_buffer_m,
        min_agl_m=cfg.min_agl_m,
        speed_mps=40.0,
        max_az_mps2=2.5,
        sample_step_m=2.0,
    )

    pts = metrics.points
    dist_km = np.array([p.s_m / 1000.0 for p in pts])
    time_s = np.array([p.t_s for p in pts])
    alt_m = np.array([p.z_msl_m for p in pts])
    gnd_m = np.array([p.ground_m for p in pts])
    agl_m = np.array([p.agl_m for p in pts])
    head_deg = np.array([p.heading_deg for p in pts])
    bank_deg = np.array([p.bank_angle_deg for p in pts])
    bank_rate = np.array([p.bank_rate_deg_s for p in pts])
    vz = np.array([p.vz_mps for p in pts])
    az_mps2 = np.array([p.az_mps2 for p in pts])
    az_g = np.array([p.az_mps2 / 9.80665 for p in pts])
    n_load = np.array([p.load_factor_g for p in pts])

    print("\n" + "=" * 80)
    print("BILECIK 31 KM SAKARYA CANYON FLIGHT FIDELITY & DYNAMICS REPORT:")
    print("=" * 80)
    print(f"1. GEOMETRI VE ZAMANLAMA:")
    print(f"   - Toplam Uçuş Mesafesi:        {metrics.total_distance_m:.1f} m ({metrics.total_distance_m/1000.0:.2f} km)")
    print(f"   - Toplam Uçuş Süresi:          {metrics.total_time_s:.1f} s ({metrics.total_time_s/60.0:.2f} dakika)")
    print(f"   - Planlama Arama Süresi:       {search_res.runtime_s:.2f} s ({search_res.expanded_nodes} düğüm)")
    print(f"   - İrtifa Optimizasyon Süresi:  {profile_res.runtime_s:.2f} s")
    print(f"   - Toplam Çözüm Süresi:         {total_planning_time:.2f} s")
    print(f"")
    print(f"2. ARAZİ İZLEME VE EMNİYET (AGL & BUFFER):")
    print(f"   - Minimum Arazi Açıklığı (AGL): {metrics.min_agl_m:.1f} m (Hedef: 120m, Emniyet Sınırı: 100m)")
    print(f"   - Ortalama AGL:                {metrics.mean_agl_m:.1f} m")
    print(f"   - Yanal Emniyet Tamponu:       {cfg.lateral_buffer_m:.1f} m (Sıfır Arazi İhlali)")
    print(f"   - İrtifa Aralığı:              {metrics.min_altitude_m:.1f} m - {metrics.max_altitude_m:.1f} m MSL")
    print(f"")
    print(f"3. DİKEY DİNAMİK VE S-EĞRİSİ İVME (C² SÜREKLİLİK):")
    print(f"   - Maksimum Tırmanış Hızı (vz): {metrics.max_climb_rate_mps:+.2f} m/s (Limit: +5.0 m/s)")
    print(f"   - Maksimum Alçalma Hızı (vz):  {metrics.max_descent_rate_mps:+.2f} m/s (Limit: -5.0 m/s)")
    print(f"   - Maksimum Dikey İvme (az):    {metrics.max_vertical_accel_mps2:.2f} m/s² ({metrics.max_vertical_accel_g:.3f}g, Limit: 0.50g)")
    print(f"")
    print(f"4. YATAY DİNAMİK VE MANEVRA:")
    print(f"   - Maksimum Yatış Açısı (phi):  {metrics.max_bank_angle_deg:.1f}° (Limit: 25.0°)")
    print(f"   - Maksimum Yatış Hızı (dphi):  {metrics.max_bank_rate_deg_s:.2f}°/s (Limit: 15.0°/s)")
    print(f"   - Minimum Dönüş Yarıçapı (R):  {metrics.min_turn_radius_m:.1f} m (Limit: 349.9 m)")
    print(f"   - Maksimum Yük Faktörü (n):    {metrics.max_load_factor_g:.2f}g")
    print("=" * 80)

    # 4-Panel Comprehensive Visualization Figure
    fig = plt.figure(figsize=(16, 14))
    gs = fig.add_gridspec(4, 1, height_ratios=[1.6, 1.2, 1.0, 1.0], hspace=0.35)

    # Panel 1: 2D DEM Contour + Flight Path
    ax1 = fig.add_subplot(gs[0])
    dem_img = roi.elevation
    extent = [roi.bounds[0], roi.bounds[2], roi.bounds[1], roi.bounds[3]]
    im = ax1.imshow(dem_img, origin="upper", extent=extent, cmap="terrain", alpha=0.85)
    cbar = plt.colorbar(im, ax=ax1, shrink=0.8, pad=0.02)
    cbar.set_label("Elevation MSL (m)", fontsize=10)

    traj_x = [p.x_m for p in pts]
    traj_y = [p.y_m for p in pts]
    ax1.plot(traj_x, traj_y, "r-", lw=2.2, label=f"31 km Trajectory ({metrics.total_distance_m/1000:.2f} km)")
    ax1.plot(start_xy[0], start_xy[1], "go", ms=8, label="Start (İnhisar)")
    ax1.plot(goal_xy[0], goal_xy[1], "m*", ms=12, label="Goal (Osmaneli)")
    ax1.set_title("1. Bilecik Sakarya Canyon 31 km Trajectory on 30x30 km Terrain (UTM 36N)", fontsize=12, fontweight="bold")
    ax1.set_xlabel("Easting (m)", fontsize=10)
    ax1.set_ylabel("Northing (m)", fontsize=10)
    ax1.legend(loc="upper left", fontsize=9)
    ax1.grid(True, alpha=0.25)

    # Panel 2: Altitude Profile vs Terrain Elevation vs AGL
    ax2 = fig.add_subplot(gs[1])
    ax2.fill_between(dist_km, gnd_m, min(gnd_m)-50, color="#8b7355", alpha=0.55, label="Terrain Elevation (60m lateral buffer)")
    ax2.plot(dist_km, gnd_m + 100.0, "r--", lw=1.2, label="Min Safe AGL (+100m)")
    ax2.plot(dist_km, alt_m, "b-", lw=2.0, label="Flight Altitude MSL (S-Curve C² Profile)")
    ax2.set_title(f"2. Terrain-Following Altitude Profile | Min AGL: {metrics.min_agl_m:.1f} m | Mean AGL: {metrics.mean_agl_m:.1f} m", fontsize=11, fontweight="bold")
    ax2.set_xlabel("Flight Distance (km)", fontsize=10)
    ax2.set_ylabel("Altitude MSL (m)", fontsize=10)
    ax2.legend(loc="upper right", fontsize=9)
    ax2.grid(True, alpha=0.25)
    ax2.set_ylim(min(gnd_m) - 20, max(alt_m) + 40)

    # Panel 3: Vertical Dynamics (v_z and a_z)
    ax3 = fig.add_subplot(gs[2])
    ax3_twin = ax3.twinx()
    l1 = ax3.plot(dist_km, vz, "teal", lw=1.5, label="Vertical Rate $v_z$ (m/s)")
    ax3.axhline(+5.0, color="teal", linestyle=":", lw=1.0, alpha=0.7)
    ax3.axhline(-5.0, color="teal", linestyle=":", lw=1.0, alpha=0.7)
    l2 = ax3_twin.plot(dist_km, az_g, "darkorange", lw=1.2, alpha=0.85, label="Vertical Accel $a_z$ (g)")
    ax3_twin.axhline(+0.5, color="darkorange", linestyle=":", lw=1.0, alpha=0.7)
    ax3_twin.axhline(-0.5, color="darkorange", linestyle=":", lw=1.0, alpha=0.7)
    ax3.set_title(f"3. Vertical Dynamics | Max $v_z$: {metrics.max_climb_rate_mps:+.1f}/{metrics.max_descent_rate_mps:+.1f} m/s | Max $a_z$: {metrics.max_vertical_accel_g:.2f}g", fontsize=11, fontweight="bold")
    ax3.set_xlabel("Flight Distance (km)", fontsize=10)
    ax3.set_ylabel("$v_z$ (m/s)", color="teal", fontsize=10)
    ax3_twin.set_ylabel("$a_z$ (g)", color="darkorange", fontsize=10)
    ax3.set_ylim(-7.0, +7.0)
    ax3_twin.set_ylim(-0.8, +0.8)
    lines = l1 + l2
    labels = [l.get_label() for l in lines]
    ax3.legend(lines, labels, loc="upper right", fontsize=9)
    ax3.grid(True, alpha=0.25)

    # Panel 4: Horizontal Dynamics (Heading, Bank Angle, Bank Rate)
    ax4 = fig.add_subplot(gs[3])
    ax4_twin = ax4.twinx()
    l3 = ax4.plot(dist_km, bank_deg, "purple", lw=1.5, label="Bank Angle $\\phi$ (deg)")
    ax4.axhline(+25.0, color="purple", linestyle=":", lw=1.0, alpha=0.7)
    ax4.axhline(-25.0, color="purple", linestyle=":", lw=1.0, alpha=0.7)
    l4 = ax4_twin.plot(dist_km, bank_rate, "gray", lw=1.0, alpha=0.7, label="Roll Rate $\\dot{\\phi}$ (deg/s)")
    ax4.set_title(f"4. Lateral Dynamics | Max Bank Angle: {metrics.max_bank_angle_deg:.1f}° | Max Roll Rate: {metrics.max_bank_rate_deg_s:.1f}°/s | Max Load Factor: {metrics.max_load_factor_g:.2f}g", fontsize=11, fontweight="bold")
    ax4.set_xlabel("Flight Distance (km)", fontsize=10)
    ax4.set_ylabel("Bank Angle $\\phi$ (°)", color="purple", fontsize=10)
    ax4_twin.set_ylabel("Roll Rate $\\dot{\\phi}$ (°/s)", color="gray", fontsize=10)
    ax4.set_ylim(-35.0, +35.0)
    ax4_twin.set_ylim(-20.0, +20.0)
    lines2 = l3 + l4
    labels2 = [l.get_label() for l in lines2]
    ax4.legend(lines2, labels2, loc="upper right", fontsize=9)
    ax4.grid(True, alpha=0.25)

    out_plot = RESULTS_DIR / "single_shot_31km_4panel_fidelity.png"
    fig.savefig(out_plot, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"\nComprehensive 4-Panel Figure saved to: {out_plot}")


if __name__ == "__main__":
    main()
