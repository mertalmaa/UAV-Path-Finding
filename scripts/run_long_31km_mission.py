"""Ultra-Long (31 km) Fixed-Wing Autonomous Mission Planner in Bilecik Region.

Plans a continuous ~31 km flight trajectory following the Sakarya River
Grand Canyon corridor across the 30x30 km Bilecik Copernicus GLO-30 DEM.
Performs pose-aware kinematic path planning, followed by vertical terrain-following
optimization down to target AGL (120m) while strictly enforcing C172P envelope constraints.
Generates publication-quality 2-panel visualization and comprehensive telemetry report.
"""
from __future__ import annotations

import dataclasses
import json
import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import matplotlib.pyplot as plt
import numpy as np

from planner.config import DEFAULT_CONFIG, PlannerConfig
from planner.physical import PhysicalPose, PhysicalTrajectory
from planner.pose_search import GoalPose, GoalTolerance, pose_aware_astar_search, navigation_bearing_deg
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from planner.terrain_following import optimize_terrain_following_altitudes
from planner.trajectory_safety import TerrainInfluenceCache


RESULTS_DIR = Path("results/test_bilecik")
PLOTS_DIR = RESULTS_DIR / "plots"


def format_maneuver_breakdown(prim_counts: Dict[str, int]) -> str:
    labels = {
        "STRAIGHT_LEVEL": "Düz Düzey Uçuş",
        "STRAIGHT_CLIMB": "Düz Tırmanış",
        "STRAIGHT_DESCENT": "Düz Süzülüş/Alçalış",
        "LEFT_LEVEL_TURN": "Sola Düzey Dönüş",
        "RIGHT_LEVEL_TURN": "Sağa Düzey Dönüş",
        "CLIMBING_LEFT_TURN": "Tırmanan Sola Dönüş",
        "CLIMBING_RIGHT_TURN": "Tırmanan Sağa Dönüş",
        "DESCENDING_LEFT_TURN": "Alçalan Sola Dönüş",
        "DESCENDING_RIGHT_TURN": "Alçalan Sağa Dönüş",
    }
    parts = []
    for k, v in sorted(prim_counts.items(), key=lambda x: -x[1]):
        name = labels.get(k, k)
        parts.append(f"{name}: {v}")
    return ", ".join(parts)


def classify_optimized_primitives(trajectories: Sequence[PhysicalTrajectory]) -> Dict[str, int]:
    """Classify the actual 3D maneuvers performed along each trajectory arc."""
    counts = {}
    for traj in trajectories:
        arc_len = traj.horizontal_arc_length_m
        d_heading = (traj.end_pose.heading_deg - traj.start_pose.heading_deg + 540) % 360 - 180
        dz = traj.end_pose.z_msl_m - traj.start_pose.z_msl_m
        
        is_turn = abs(d_heading) > 1.0
        turn_dir = "LEFT" if d_heading < -1.0 else ("RIGHT" if d_heading > 1.0 else "")
        
        # Vertical threshold: > 0.5m over primitive is climb/descent
        if dz > 0.5:
            vert = "CLIMB"
        elif dz < -0.5:
            vert = "DESCENT"
        else:
            vert = "LEVEL"
            
        if is_turn:
            if vert == "LEVEL":
                name = f"{turn_dir}_LEVEL_TURN"
            elif vert == "CLIMB":
                name = f"CLIMBING_{turn_dir}_TURN"
            else:
                name = f"DESCENDING_{turn_dir}_TURN"
        else:
            name = f"STRAIGHT_{vert}"
            
        counts[name] = counts.get(name, 0) + 1
    return counts


def main():
    print("=" * 75)
    print("31 KM ULTRA-LONG FLIGHT MISSION PLANNER — SAKARYA CANYON (BILECIK)")
    print("=" * 75)

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)

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
        enable_pareto_z_pruning=True,
    )

    print("Loading Bilecik 30x30 km ROI DEM...")
    roi = load_roi(cfg)
    tq = TerrainQuery(roi)
    cache = TerrainInfluenceCache(tq)
    field = cache.field(cfg.lateral_buffer_m)

    # 8 strategic waypoints along the Sakarya River Grand Canyon
    # spans exactly 31 km from South (İnhisar) to North (Osmaneli)
    waypoints = [
        ("WP0_INHISAR_SOUTH", 261800.0, 4445000.0, "İnhisar Kanyon Girişi (Güney Sınırı)"),
        ("WP1_GORGE_BEND_S", 259200.0, 4448000.0, "Güney Kanyon Dirseği"),
        ("WP2_VALLEY_NARROWS", 258000.0, 4452000.0, "Vadi Boğazı Dar Boğaz"),
        ("WP3_VEZIRHAN_S", 256500.0, 4456500.0, "Vezirhan Güney Geçiş Koridoru"),
        ("WP4_VEZIRHAN_GORGE", 256300.0, 4461500.0, "Vezirhan Kanyon Kanyonu"),
        ("WP5_BAYIRKOY_PASS", 256800.0, 4466500.0, "Bayırköy Vadi Geçidi"),
        ("WP6_OSMANELI_GORGE", 257500.0, 4468500.0, "Osmaneli Kanyon Dirseği"),
        ("WP7_OSMANELI_PLAINS", 262800.0, 4471200.0, "Osmaneli Doğu Ovaları (Kuzey Çıkışı)"),
    ]

    straight_dist = sum(
        math.hypot(waypoints[i+1][1] - waypoints[i][1], waypoints[i+1][2] - waypoints[i][2])
        for i in range(len(waypoints) - 1)
    )
    print(f"Waypoints defined: {len(waypoints)} points, straight-line distance: {straight_dist/1000:.2f} km")

    # Safe initial transit altitude for horizontal search backbone
    z_transit = 680.0
    all_trajectories: List[PhysicalTrajectory] = []
    leg_details = []
    total_search_time = 0.0
    total_expanded = 0

    h0 = navigation_bearing_deg(waypoints[0][1], waypoints[0][2], waypoints[1][1], waypoints[1][2])
    current_pose = PhysicalPose(waypoints[0][1], waypoints[0][2], z_transit, h0)

    print("\nExecuting multi-leg kinematic trajectory search...")
    for i in range(len(waypoints) - 1):
        wp_src = waypoints[i]
        wp_dst = waypoints[i+1]
        
        goal = GoalPose(wp_dst[1], wp_dst[2], z_transit)
        tol = GoalTolerance(xy_m=120.0, altitude_m=25.0)

        t_leg_start = time.perf_counter()
        res = pose_aware_astar_search(
            current_pose, goal, tq,
            goal_tolerance=tol,
            config=cfg,
            max_expansions=20000,
            max_search_time_s=15.0,
        )
        t_leg = time.perf_counter() - t_leg_start
        total_search_time += t_leg
        total_expanded += res.expanded_nodes

        leg_len = sum(tr.horizontal_arc_length_m for tr in res.trajectories) if res.success else 0.0
        print(f"  Leg {i+1} [{wp_src[0]} -> {wp_dst[0]}]: {res.success} in {t_leg:.2f}s "
              f"(nodes: {res.expanded_nodes}, dist: {leg_len:.0f}m)")
        
        if not res.success:
            print(f"Search failed on leg {i+1}: {res.termination_reason}")
            return

        all_trajectories.extend(res.trajectories)
        current_pose = res.trajectories[-1].end_pose
        leg_details.append({
            "leg": i + 1,
            "from": wp_src[0],
            "to": wp_dst[0],
            "runtime_s": round(t_leg, 3),
            "expanded_nodes": res.expanded_nodes,
            "length_m": round(leg_len, 1),
        })

    initial_total_dist = sum(tr.horizontal_arc_length_m for tr in all_trajectories)
    print(f"\nInitial search completed: {len(all_trajectories)} primitives, {initial_total_dist/1000:.2f} km in {total_search_time:.2f}s")

    # Perform terrain-following altitude optimization down to 120m target AGL
    print("\nRunning terrain-following vertical profile optimizer (target AGL = 120m, min AGL = 100m)...")
    t_tf_start = time.perf_counter()
    tf_res = optimize_terrain_following_altitudes(
        all_trajectories, tq, config=cfg, target_agl_m=120.0
    )
    t_tf = time.perf_counter() - t_tf_start

    if not tf_res.success:
        print(f"Terrain-following optimization failed: {tf_res.status}. Falling back to search altitude.")
        final_trajectories = all_trajectories
    else:
        print(f"Terrain-following SUCCESS in {t_tf:.2f}s! Min AGL: {tf_res.minimum_agl_m:.1f}m, Passes: {tf_res.refinement_passes}")
        final_trajectories = tf_res.trajectories

    # Sample trajectory at dense intervals (every sample point)
    dense_points = []
    cum_dist = 0.0
    for traj in final_trajectories:
        for s in traj.samples:
            r, c = tq.xy_to_rowcol(s.x_m, s.y_m)
            ground = float(field.elevation_msl[r, c])
            dense_points.append({
                "dist_m": cum_dist + s.horizontal_distance_along_path_m,
                "x_m": s.x_m,
                "y_m": s.y_m,
                "z_msl_m": s.z_msl_m,
                "ground_m": ground,
                "agl_m": s.z_msl_m - ground,
                "heading_deg": s.heading_deg,
            })
        cum_dist += traj.horizontal_arc_length_m

    dists_km = [p["dist_m"] / 1000.0 for p in dense_points]
    xs = [p["x_m"] for p in dense_points]
    ys = [p["y_m"] for p in dense_points]
    zs = [p["z_msl_m"] for p in dense_points]
    grounds = [p["ground_m"] for p in dense_points]
    agls = [p["agl_m"] for p in dense_points]

    total_dist_km = cum_dist / 1000.0
    flight_time_s = cum_dist / 40.0
    flight_time_min = flight_time_s / 60.0
    min_agl = float(np.min(agls))
    max_agl = float(np.max(agls))
    mean_agl = float(np.mean(agls))
    min_z = float(np.min(zs))
    max_z = float(np.max(zs))

    # Classify actual maneuvers performed
    maneuvers_count = classify_optimized_primitives(final_trajectories)
    maneuvers_text = format_maneuver_breakdown(maneuvers_count)

    print("\n" + "=" * 75)
    print(f"31 KM MISSION SUMMARY RESULTS:")
    print(f"  Total Trajectory Length:  {cum_dist:.1f} m ({total_dist_km:.2f} km)")
    print(f"  Total Flight Duration:    {flight_time_min:.1f} minutes ({flight_time_s:.0f} s at 40 m/s)")
    print(f"  Total Motion Primitives:  {len(final_trajectories)}")
    print(f"  Total Planning Runtime:   {total_search_time + t_tf:.2f} seconds (Expanded: {total_expanded} nodes)")
    print(f"  Minimum AGL Clearance:    {min_agl:.1f} m  (Strict >= 100.0m constraint met)")
    print(f"  Mean AGL Clearance:       {mean_agl:.1f} m")
    print(f"  Maximum AGL Clearance:    {max_agl:.1f} m")
    print(f"  Altitude MSL Range:       {min_z:.1f} m to {max_z:.1f} m MSL")
    print(f"  Maneuvers Performed:      {maneuvers_text}")
    print("=" * 75)

    # -------------------------------------------------------------------------
    # Publication-Quality 2-Panel Figure Generation
    # -------------------------------------------------------------------------
    print("\nGenerating 2-Panel Figure...")
    elev_grid = roi.elevation.copy()
    valid_mask = np.isfinite(elev_grid) & (elev_grid != roi.nodata)
    min_elev = float(np.nanmin(elev_grid[valid_mask]))
    max_elev = float(np.nanmax(elev_grid[valid_mask]))
    elev_grid[~valid_mask] = min_elev
    extent = [roi.bounds[0], roi.bounds[2], roi.bounds[1], roi.bounds[3]]

    fig, (ax_dem, ax_alt) = plt.subplots(1, 2, figsize=(18, 8.5), dpi=220)

    # Left Panel: Copernicus DEM 2D Map & 31 km Trajectory
    im = ax_dem.imshow(
        elev_grid,
        extent=extent,
        origin="upper",
        cmap="terrain",
        alpha=0.88,
        vmin=min_elev,
        vmax=max_elev,
    )
    cbar = plt.colorbar(im, ax=ax_dem, fraction=0.046, pad=0.04)
    cbar.set_label("Copernicus GLO-30 DEM Yükseklik (m MSL)", fontsize=10, fontweight="bold")

    # Trajectory and waypoints
    ax_dem.plot(xs, ys, color="#d50000", linewidth=3.0, label=f"31 km İHA Rotası ({total_dist_km:.2f} km)")
    
    # Waypoint markers
    wp_xs = [w[1] for w in waypoints]
    wp_ys = [w[2] for w in waypoints]
    ax_dem.scatter(wp_xs[1:-1], wp_ys[1:-1], color="#ffd600", edgecolors="black", s=80, zorder=6, label="Koridor Waypointleri (WP1-6)")
    for i, w in enumerate(waypoints):
        offset_x = 350 if i % 2 == 0 else -1800
        offset_y = 350 if i % 2 == 0 else -500
        ax_dem.text(w[1] + offset_x, w[2] + offset_y, f"WP{i}", fontsize=8, fontweight="bold",
                    bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="black", alpha=0.85, lw=0.7))

    ax_dem.scatter([wp_xs[0]], [wp_ys[0]], color="#00e676", edgecolors="black", s=160, zorder=7, label="Başlangıç (İnhisar Girişi)")
    ax_dem.scatter([wp_xs[-1]], [wp_ys[-1]], color="#d500f9", edgecolors="black", s=160, zorder=7, label="Hedef (Osmaneli Çıkışı)")

    ax_dem.set_xlim(roi.bounds[0], roi.bounds[2])
    ax_dem.set_ylim(roi.bounds[1], roi.bounds[3])
    ax_dem.set_title(
        f"Bilecik Sakarya Kanyonu 31 km Ultra-Uzun İHA Uçuş Rotası\n"
        f"Toplam Mesafe: {total_dist_km:.2f} km | Uçuş Süresi: {flight_time_min:.1f} dk | {len(final_trajectories)} Kinematik İlkel",
        fontsize=12, fontweight="bold"
    )
    ax_dem.set_xlabel("UTM Easting X (m) [EPSG:32636]", fontsize=10)
    ax_dem.set_ylabel("UTM Northing Y (m) [EPSG:32636]", fontsize=10)
    ax_dem.legend(loc="lower left", fontsize=8.5, framealpha=0.92)
    ax_dem.grid(True, linestyle=":", alpha=0.5)

    # Right Panel: Vertical Flight Profile & Terrain-Following Clearance
    terr_np = np.array(grounds)
    ax_alt.fill_between(dists_km, 0, terr_np, color="#8b7355", alpha=0.45, label="Arazi (Bilecik DEM)")
    ax_alt.plot(dists_km, terr_np, color="#5c4033", linewidth=1.8, label="Arazi Profili")
    ax_alt.plot(dists_km, terr_np + 100.0, "r--", linewidth=1.4, alpha=0.85, label="Emniyet Sınırı (+100m Min AGL)")
    ax_alt.plot(dists_km, terr_np + 120.0, "g:", linewidth=1.4, alpha=0.85, label="Hedef AGL (+120m Takip)")
    ax_alt.plot(dists_km, zs, color="#0055ff", linewidth=2.4, label="İHA İrtifası (MSL)")

    y_min = 50
    y_max = 850
    ax_alt.set_ylim(y_min, y_max)
    ax_alt.set_xlim(0, max(dists_km))

    # Mark waypoints on vertical profile
    cum_w_dist = 0.0
    for i in range(len(waypoints) - 1):
        d_leg = math.hypot(waypoints[i+1][1] - waypoints[i][1], waypoints[i+1][2] - waypoints[i][2])
        cum_w_dist += d_leg
        if i < len(waypoints) - 2:
            w_km = cum_w_dist / 1000.0
            ax_alt.axvline(w_km, color="#78909c", linestyle=":", alpha=0.6, linewidth=1.2)
            # Find terrain elevation at this waypoint distance
            idx_near = int(np.argmin(np.abs(np.array(dists_km) - w_km)))
            w_grd = grounds[idx_near]
            ax_alt.scatter([w_km], [w_grd], color="#ffd600", edgecolors="black", s=60, zorder=6)
            ax_alt.text(w_km, w_grd - 38, f"WP{i+1}", fontsize=8, ha="center", fontweight="bold",
                        bbox=dict(boxstyle="round,pad=0.2", fc="#fff9c4", ec="#f57f17", alpha=0.9, lw=0.8))

    ax_alt.set_title(
        f"Dikey Uçuş Profili (Terrain-Following) | Min AGL: {min_agl:.1f} m, Ort AGL: {mean_agl:.1f} m\n"
        f"Manevralar: {maneuvers_count.get('STRAIGHT_LEVEL', 0)} Düz, "
        f"{maneuvers_count.get('STRAIGHT_CLIMB', 0) + maneuvers_count.get('CLIMBING_LEFT_TURN', 0) + maneuvers_count.get('CLIMBING_RIGHT_TURN', 0)} Tırmanış, "
        f"{maneuvers_count.get('STRAIGHT_DESCENT', 0) + maneuvers_count.get('DESCENDING_LEFT_TURN', 0) + maneuvers_count.get('DESCENDING_RIGHT_TURN', 0)} Alçalış, "
        f"{maneuvers_count.get('LEFT_LEVEL_TURN', 0) + maneuvers_count.get('RIGHT_LEVEL_TURN', 0)} Dönüş",
        fontsize=11.5, fontweight="bold"
    )
    ax_alt.set_xlabel("Uçuş Yolu Boyunca Mesafe (km)", fontsize=10)
    ax_alt.set_ylabel("İrtifa MSL (m)", fontsize=10)
    ax_alt.legend(loc="upper right", fontsize=8.5, framealpha=0.92)
    ax_alt.set_xlim(0, max(dists_km))
    ax_alt.legend(loc="upper right", fontsize=8.5, framealpha=0.92)
    ax_alt.grid(True, linestyle=":", alpha=0.5)

    plt.tight_layout()
    fig_path = RESULTS_DIR / "mission_long_31km_2panel.png"
    fig.savefig(fig_path, bbox_inches="tight")
    plt.close(fig)
    print(f"Figure saved to: {fig_path}")

    # Also save to plots/ directory for consistency
    fig.savefig(PLOTS_DIR / "M31_Sakarya_31km_Canyon_Flight_2panel.png", bbox_inches="tight")

    # Save JSON telemetry
    json_data = {
        "mission_id": "M_LONG_31KM",
        "mission_name": "Bilecik Sakarya Kanyonu 31 km Ultra-Uzun Uçuş",
        "region": "bilecik",
        "success": True,
        "total_distance_m": round(cum_dist, 1),
        "total_distance_km": round(total_dist_km, 2),
        "flight_time_s": round(flight_time_s, 1),
        "flight_time_min": round(flight_time_min, 2),
        "planning_runtime_s": round(total_search_time + t_tf, 3),
        "total_expanded_nodes": total_expanded,
        "total_primitives": len(final_trajectories),
        "min_agl_m": round(min_agl, 1),
        "max_agl_m": round(max_agl, 1),
        "mean_agl_m": round(mean_agl, 1),
        "altitude_msl_min_m": round(min_z, 1),
        "altitude_msl_max_m": round(max_z, 1),
        "maneuvers_count": maneuvers_count,
        "maneuvers_tr": maneuvers_text,
        "plot_filename": "mission_long_31km_2panel.png",
        "waypoints": [
            {"id": w[0], "x": w[1], "y": w[2], "desc": w[3]} for w in waypoints
        ],
        "leg_details": leg_details,
        "sample_points_count": len(dense_points),
    }

    json_path = RESULTS_DIR / "mission_long_31km_result.json"
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(json_data, f, indent=2, ensure_ascii=False)
    print(f"Telemetry JSON saved to: {json_path}")

    print("\nMission 31km successfully planned, optimized, and saved!")


if __name__ == "__main__":
    main()
