"""Bilecik 5 Diverse Missions Local Constrained B-Spline Smoothing Benchmark & Visualization.

Runs 5 distinct missions in Bilecik terrain:
  - M01: Sakarya Riverbed North Transit (Canyon corridor & meanders)
  - M03: Vezirhan Narrow Gorge Passage (Tight canyon gorge with steep walls)
  - M08: Short Steep Valley Drop (Steep valley descent)
  - M11: Riverbed To Plateau Climb (Climb-out from canyon to high plateau)
  - M18: Saddle Pass Through Ridge (Navigating saddle between mountain ridges)

Performs Corridor-Safe Local Constrained B-Spline Smoothing around all curvature discontinuities,
extracts high-fidelity physical metrics, and generates detailed 5-panel visualization PNGs
allowing visual inspection of the smoothed curves against the raw Dubins corners.
"""
from __future__ import annotations

import dataclasses
import json
import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from planner.config import DEFAULT_CONFIG, PlannerConfig
from planner.local_trajectory_smoothing import (
    DiscontinuityJunction,
    LocalSmoothingResult,
    apply_corridor_safe_local_bspline_smoothing,
    detect_curvature_junctions,
)
from planner.physical import PhysicalPose
from planner.pose_search import GoalPose, GoalTolerance, navigation_bearing_deg, pose_aware_astar_search
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from planner.terrain_following import optimize_terrain_following_altitudes
from planner.trajectory_safety import TerrainInfluenceCache
from scripts.bilecik_missions_spec import BILECIK_MISSIONS, BilecikMissionDef

RESULTS_DIR = PROJECT_ROOT / "results" / "test_bilecik"
PLOTS_DIR = RESULTS_DIR / "plots"
PLOTS_DIR.mkdir(parents=True, exist_ok=True)


def run_mission_benchmark(
    mission_def: BilecikMissionDef,
    terrain: TerrainQuery,
    cache: TerrainInfluenceCache,
    config: PlannerConfig,
) -> Tuple[Dict, LocalSmoothingResult, Path]:
    field = cache.field(config.lateral_buffer_m)
    
    def get_alt(xy, offset):
        r, c = terrain.xy_to_rowcol(*xy)
        return float(field.elevation_msl[r, c]) + max(130.0, offset)

    heading = mission_def.start_heading_deg
    if heading is None:
        heading = navigation_bearing_deg(*mission_def.start_xy, *mission_def.goal_xy)
        
    start_pose = PhysicalPose(*mission_def.start_xy, get_alt(mission_def.start_xy, mission_def.start_alt_offset_m), heading)
    goal_pose = GoalPose(*mission_def.goal_xy, get_alt(mission_def.goal_xy, mission_def.goal_alt_offset_m))
    
    print(f"\n[{mission_def.id}] {mission_def.name} - Searching path...")
    t0 = time.perf_counter()
    search_res = pose_aware_astar_search(
        start_pose,
        goal_pose,
        terrain,
        config=config,
        goal_tolerance=GoalTolerance(mission_def.goal_tolerance_xy_m, mission_def.goal_tolerance_z_m),
        max_expansions=30000,
        max_search_time_s=15.0,
    )
    search_time = time.perf_counter() - t0
    
    if not search_res.success:
        raise RuntimeError(f"Search failed for {mission_def.id}: {search_res.termination_reason}")
        
    t1 = time.perf_counter()
    profile_res = optimize_terrain_following_altitudes(search_res.trajectories, terrain, config=config, target_agl_m=120.0)
    profile_time = time.perf_counter() - t1
    
    if not profile_res.success:
        raise RuntimeError(f"Profile failed for {mission_def.id}: {profile_res.status}")
        
    print(f"[{mission_def.id}] Path found! Length: {search_res.continuous_path_length_m:.1f} m, Nodes: {search_res.expanded_nodes}")
    print(f"[{mission_def.id}] Applying Corridor-Safe Local Constrained B-Spline Smoothing...")
    
    t2 = time.perf_counter()
    smooth_res = apply_corridor_safe_local_bspline_smoothing(
        profile_res.trajectories,
        terrain,
        config=config,
        speed_mps=40.0,
        sample_step_m=1.0,
        max_bank_rate_deg_s=15.0,
        max_corridor_deviation_m=12.0,
    )
    smooth_time = time.perf_counter() - t2
    
    print(f"[{mission_def.id}] Smoothing completed in {smooth_time*1000:.1f} ms! "
          f"Junctions: {smooth_res.num_junctions_smoothed}/{smooth_res.num_junctions_total}, "
          f"Max Dev: {smooth_res.max_corridor_deviation_m:.2f} m, "
          f"Peak Roll Rate: {smooth_res.raw_max_bank_rate_deg_s:.1f} -> {smooth_res.smoothed_max_bank_rate_deg_s:.1f} deg/s")

    # Generate Detailed Visual Inspection Figure
    plot_path = generate_mission_smoothing_plot(mission_def, terrain, smooth_res, config)
    
    summary = {
        "mission_id": mission_def.id,
        "name": mission_def.name,
        "category": mission_def.category,
        "search_runtime_s": round(search_time, 3),
        "profile_runtime_s": round(profile_time, 3),
        "smoothing_runtime_ms": round(smooth_time * 1000, 2),
        "expanded_nodes": search_res.expanded_nodes,
        "total_distance_m": round(smooth_res.total_distance_m, 1),
        "flight_time_s": round(smooth_res.total_time_s, 1),
        "min_agl_m": round(smooth_res.min_agl_m, 1),
        "mean_agl_m": round(smooth_res.mean_agl_m, 1),
        "max_corridor_deviation_m": round(smooth_res.max_corridor_deviation_m, 2),
        "num_junctions_total": smooth_res.num_junctions_total,
        "num_junctions_smoothed": smooth_res.num_junctions_smoothed,
        "raw_max_bank_angle_deg": round(smooth_res.raw_max_bank_angle_deg, 1),
        "smoothed_max_bank_angle_deg": round(smooth_res.smoothed_max_bank_angle_deg, 1),
        "raw_max_roll_rate_deg_s": round(smooth_res.raw_max_bank_rate_deg_s, 1),
        "smoothed_max_roll_rate_deg_s": round(smooth_res.smoothed_max_bank_rate_deg_s, 1),
        "corridor_safe": smooth_res.corridor_safe,
        "plot_path": str(plot_path.relative_to(PROJECT_ROOT)),
    }
    
    return summary, smooth_res, plot_path


def generate_mission_smoothing_plot(
    mission_def: BilecikMissionDef,
    terrain: TerrainQuery,
    res: LocalSmoothingResult,
    config: PlannerConfig,
) -> Path:
    fig = plt.figure(figsize=(18, 14))
    gs = GridSpec(3, 2, height_ratios=[1.2, 1.0, 1.0], width_ratios=[1.1, 1.0], hspace=0.32, wspace=0.22)
    
    raw_pts = res.original_points
    sm_pts = res.smoothed_points
    
    s_raw = np.array([p.s_m / 1000.0 for p in raw_pts])
    s_sm = np.array([p.s_m / 1000.0 for p in sm_pts])
    
    # -------------------------------------------------------------------------
    # Panel 1: Top-Down DEM Overview with Rota & Highlighted Junctions
    # -------------------------------------------------------------------------
    ax_map = fig.add_subplot(gs[0, 0])
    roi = terrain.roi
    extent = [roi.bounds[0], roi.bounds[2], roi.bounds[1], roi.bounds[3]]
    im = ax_map.imshow(roi.elevation, origin="upper", extent=extent, cmap="terrain", alpha=0.85)
    cb = plt.colorbar(im, ax=ax_map, shrink=0.75, pad=0.02)
    cb.set_label("Elevation MSL (m)", fontsize=9)
    
    sm_x = [p.x_m for p in sm_pts]
    sm_y = [p.y_m for p in sm_pts]
    ax_map.plot(sm_x, sm_y, "b-", lw=2.0, label="B-Spline Smoothed Trajectory")
    ax_map.plot(mission_def.start_xy[0], mission_def.start_xy[1], "go", ms=9, label="Start")
    ax_map.plot(mission_def.goal_xy[0], mission_def.goal_xy[1], "r*", ms=13, label="Goal")
    
    # Mark a couple of junctions
    for j in res.junctions[:6]:
        ax_map.plot(j.x_m, j.y_m, "k^", ms=5, alpha=0.7)
    if res.junctions:
        ax_map.plot(res.junctions[0].x_m, res.junctions[0].y_m, "k^", ms=6, label=f"Junctions ({len(res.junctions)} total)")
        
    ax_map.set_title(f"1. Top-Down Map: {mission_def.id} - {mission_def.name} ({res.total_distance_m/1000:.2f} km)", fontsize=11, fontweight="bold")
    ax_map.set_xlabel("Easting (m)", fontsize=9)
    ax_map.set_ylabel("Northing (m)", fontsize=9)
    ax_map.legend(loc="upper left", fontsize=8.5)
    ax_map.grid(True, alpha=0.25)
    
    # Zoom bounds to mission area with padding
    pad = 800.0
    all_x = sm_x + [mission_def.start_xy[0], mission_def.goal_xy[0]]
    all_y = sm_y + [mission_def.start_xy[1], mission_def.goal_xy[1]]
    ax_map.set_xlim(min(all_x) - pad, max(all_x) + pad)
    ax_map.set_ylim(min(all_y) - pad, max(all_y) + pad)

    # -------------------------------------------------------------------------
    # Panel 2: ZOOMED-IN CURVE INSPECTION (Raw Dubins Corner vs Smoothed B-Spline)
    # -------------------------------------------------------------------------
    ax_zoom = fig.add_subplot(gs[0, 1])
    # Find the most pronounced junction (e.g. reversal or straight-to-turn)
    target_j = None
    for j in res.junctions:
        if "REVERSAL" in j.kind or "LEFT_TO_RIGHT" in j.kind or "RIGHT_TO_LEFT" in j.kind:
            target_j = j
            break
    if target_j is None and res.junctions:
        target_j = res.junctions[0]
        
    if target_j is not None:
        j_s = target_j.s_m
        win = target_j.window_half_m * 2.2
        zoom_mask = (np.array([p.s_m for p in raw_pts]) >= j_s - win) & (np.array([p.s_m for p in raw_pts]) <= j_s + win)
        
        zx_raw = np.array([p.x_m for p in raw_pts])[zoom_mask]
        zy_raw = np.array([p.y_m for p in raw_pts])[zoom_mask]
        
        zx_sm = np.array([p.x_m for p in sm_pts])[zoom_mask]
        zy_sm = np.array([p.y_m for p in sm_pts])[zoom_mask]
        
        ax_zoom.plot(zx_raw, zy_raw, "r--", lw=2.2, label="Raw Dubins Corner (Sharp Transition)")
        ax_zoom.plot(zx_sm, zy_sm, "b-", lw=2.5, label="Corridor-Safe B-Spline (Smooth C² Curve)")
        ax_zoom.plot(target_j.x_m, target_j.y_m, "ko", ms=8, label=f"Junction Apex ({target_j.kind})")
        
        # Draw corridor boundary visualization (+- 15m)
        ax_zoom.set_title(f"2. Zoomed-in Curve Inspection @ s={target_j.s_m:.0f}m\n"
                          f"Transition Window: ±{target_j.window_half_m:.1f}m | Max Dev: {res.max_corridor_deviation_m:.2f}m", 
                          fontsize=11, fontweight="bold")
        ax_zoom.set_xlabel("Easting (m)", fontsize=9)
        ax_zoom.set_ylabel("Northing (m)", fontsize=9)
        ax_zoom.legend(loc="best", fontsize=8.5)
        ax_zoom.grid(True, alpha=0.3)
        ax_zoom.axis("equal")

    # -------------------------------------------------------------------------
    # Panel 3: Bank Angle Comparison (phi(s) Before vs After)
    # -------------------------------------------------------------------------
    ax_bank = fig.add_subplot(gs[1, 0])
    raw_bank = [p.bank_angle_deg for p in raw_pts]
    sm_bank = [p.bank_angle_deg for p in sm_pts]
    
    ax_bank.plot(s_raw, raw_bank, "r--", lw=1.2, alpha=0.7, label=f"Raw Dubins Bank (Bang-Bang, Max: {res.raw_max_bank_angle_deg:.1f}°)")
    ax_bank.plot(s_sm, sm_bank, "purple", lw=1.8, label=f"Smoothed C² Bank Profile (Max: {res.smoothed_max_bank_angle_deg:.1f}°)")
    ax_bank.axhline(+25.0, color="k", linestyle=":", lw=1.0, alpha=0.6, label="Bank Limit (±25.0°)")
    ax_bank.axhline(-25.0, color="k", linestyle=":", lw=1.0, alpha=0.6)
    
    # Highlight transition regions
    trans_mask = np.array([p.is_in_transition for p in sm_pts])
    ax_bank.fill_between(s_sm, -28.0, 28.0, where=trans_mask, color="yellow", alpha=0.15, label="B-Spline Transition Zones")
    
    ax_bank.set_title(f"3. Bank Angle Dynamics $\\phi(t)$ | Continuous C² Transition", fontsize=11, fontweight="bold")
    ax_bank.set_xlabel("Flight Distance (km)", fontsize=9)
    ax_bank.set_ylabel("Bank Angle $\\phi$ (°)", fontsize=9)
    ax_bank.set_ylim(-32.0, 32.0)
    ax_bank.legend(loc="upper right", fontsize=8.5)
    ax_bank.grid(True, alpha=0.25)

    # -------------------------------------------------------------------------
    # Panel 4: Roll Rate Comparison (dphi/dt Before vs After)
    # -------------------------------------------------------------------------
    ax_roll = fig.add_subplot(gs[1, 1])
    raw_rates = [p.bank_rate_deg_s for p in raw_pts]
    sm_rates = [p.bank_rate_deg_s for p in sm_pts]
    
    ax_roll.plot(s_raw, raw_rates, "gray", linestyle="--", lw=1.0, alpha=0.5, label=f"Raw Roll Rate (Spikes: {res.raw_max_bank_rate_deg_s:.1f}°/s)")
    ax_roll.plot(s_sm, sm_rates, "teal", lw=1.5, label=f"Smoothed Roll Rate $\\dot{{\\phi}}$ (Max: {res.smoothed_max_bank_rate_deg_s:.1f}°/s)")
    ax_roll.axhline(+15.0, color="teal", linestyle=":", lw=1.2, alpha=0.8, label="Target Limit (±15.0°/s)")
    ax_roll.axhline(-15.0, color="teal", linestyle=":", lw=1.2, alpha=0.8)
    
    ax_roll.set_title(f"4. Roll Rate $\\dot{{\\phi}}$ Reduction | Chattering Eliminated", fontsize=11, fontweight="bold")
    ax_roll.set_xlabel("Flight Distance (km)", fontsize=9)
    ax_roll.set_ylabel("Roll Rate $\\dot{\\phi}$ (°/s)", fontsize=9)
    ax_roll.set_ylim(-60.0, 60.0)
    ax_roll.legend(loc="upper right", fontsize=8.5)
    ax_roll.grid(True, alpha=0.25)

    # -------------------------------------------------------------------------
    # Panel 5: Terrain-Following Altitude Profile & Min AGL Safe Margin
    # -------------------------------------------------------------------------
    ax_alt = fig.add_subplot(gs[2, :])
    gnd = [p.ground_m for p in sm_pts]
    alt = [p.z_msl_m for p in sm_pts]
    
    ax_alt.fill_between(s_sm, gnd, min(gnd) - 40, color="#8b7355", alpha=0.55, label="Terrain Elevation MSL (60m lateral buffer)")
    ax_alt.plot(s_sm, np.array(gnd) + 100.0, "r--", lw=1.2, label="Min Safe AGL (+100.0m hard limit)")
    ax_alt.plot(s_sm, alt, "b-", lw=2.0, label=f"Flight Altitude Profile (Min AGL: {res.min_agl_m:.1f}m, Mean: {res.mean_agl_m:.1f}m)")
    
    ax_alt.set_title(f"5. Terrain-Following Altitude Profile | Guaranteed Minimum Clearance: {res.min_agl_m:.1f} m (Safety: {'VERIFIED SAFE' if res.corridor_safe else 'BREACH'})", fontsize=11, fontweight="bold")
    ax_alt.set_xlabel("Flight Distance (km)", fontsize=9)
    ax_alt.set_ylabel("Altitude MSL (m)", fontsize=9)
    ax_alt.set_ylim(min(gnd) - 20, max(alt) + 40)
    ax_alt.legend(loc="upper right", fontsize=8.5)
    ax_alt.grid(True, alpha=0.25)

    fig.suptitle(f"Bilecik Mission {mission_def.id}: {mission_def.name}\n"
                 f"Corridor-Safe Local Constrained B-Spline Smoothing Performance Benchmark",
                 fontsize=13, fontweight="bold", y=0.99)

    out_file = PLOTS_DIR / f"{mission_def.id}_local_bspline_smoothing.png"
    fig.savefig(out_file, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out_file


def main():
    print("=" * 80)
    print("BILECIK 5 DIVERSE MISSIONS: CORRIDOR-SAFE LOCAL CONSTRAINED B-SPLINE BENCHMARK")
    print("=" * 80)

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
        enable_terrain_guidance=True,
    )

    print("Loading Bilecik 30x30 km ROI DEM...")
    roi = load_roi(cfg)
    terrain = TerrainQuery(roi)
    cache = TerrainInfluenceCache(terrain)

    # Select the 5 diverse missions:
    target_ids = ["M01", "M03", "M08", "M11", "M18"]
    selected_defs = [m for m in BILECIK_MISSIONS if m.id in target_ids]

    results_summary = []
    
    for m_def in selected_defs:
        summary, smooth_res, plot_file = run_mission_benchmark(m_def, terrain, cache, cfg)
        results_summary.append(summary)

    # Save summary JSON
    out_json = RESULTS_DIR / "local_bspline_5missions_benchmark.json"
    out_json.write_text(json.dumps(results_summary, indent=2), encoding="utf-8")
    print(f"\nBenchmark results saved to: {out_json}")
    
    print("\n" + "=" * 100)
    print(f"{'ID':<5} | {'Mission Name':<30} | {'Dist (km)':<9} | {'Min AGL':<8} | {'Max Dev':<8} | {'Raw Roll Rate':<14} | {'Smoothed Roll':<14} | {'Status'}")
    print("-" * 100)
    for r in results_summary:
        status = "PASSED (SAFE)" if r["corridor_safe"] else "FAILED"
        print(f"{r['mission_id']:<5} | {r['name'][:30]:<30} | {r['total_distance_m']/1000:<9.2f} | {r['min_agl_m']:<8.1f} | {r['max_corridor_deviation_m']:<8.2f} | {r['raw_max_roll_rate_deg_s']:<14.1f} | {r['smoothed_max_roll_rate_deg_s']:<14.1f} | {status}")
    print("=" * 100)


if __name__ == "__main__":
    main()
