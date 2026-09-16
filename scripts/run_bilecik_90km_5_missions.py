"""5 Distinct ~90 km Single-Shot Flight Path Tests on the Bilecik 90x90 km ROI.

Runs 5 direct (zero-waypoint) start->goal missions spread across five different
directions/structures inside the full 90x90 km Bilecik working DEM:
  1. M90_01: South -> North Sakarya corridor transit
  2. M90_02: South -> North eastern ridge/plateau transit
  3. M90_03: West -> East plateau/ridge crossing
  4. M90_04: Southwest -> Northeast diagonal cross-country transit
  5. M90_05: Northwest -> Southeast diagonal cross-country transit

Pipeline (identical to the project's production single-shot benchmark):
  - Pose-aware Dual-Queue A* search (planner.pose_search)
  - Terrain-following vertical profile optimizer (target 120 m AGL, >=100 m hard floor)
  - Corridor-safe local constrained B-spline smoothing
  - Per-mission 4-panel figure + a master summary atlas + a JSON metrics file
"""
from __future__ import annotations

import dataclasses
import json
import math
import sys
import time
from pathlib import Path
from typing import Dict, List

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import numpy as np

from planner.config import DEFAULT_CONFIG, PlannerConfig
from planner.fixed_wing_envelope import FixedWingKinematicEnvelope
from planner.local_trajectory_smoothing import (
    LocalSmoothingResult,
    apply_corridor_safe_local_bspline_smoothing,
)
from planner.physical import PhysicalPose
from planner.pose_search import GoalPose, GoalTolerance, navigation_bearing_deg, pose_aware_astar_search
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from planner.terrain_following import optimize_terrain_following_altitudes
from planner.trajectory_safety import TerrainInfluenceCache

RESULTS_DIR = PROJECT_ROOT / "results" / "test_bilecik"
PLOTS_DIR = RESULTS_DIR / "plots"
RESULTS_DIR.mkdir(parents=True, exist_ok=True)
PLOTS_DIR.mkdir(parents=True, exist_ok=True)


MISSIONS_90KM = [
    {
        "id": "M90_01",
        "title_tr": "Sakarya Koridoru — Güney'den Kuzeye Ana Hat",
        "structure_type": "Kanyon & Vadi Koridoru (Canyon Corridor)",
        "description_tr": "90x90 km haritanın güney ucundan kuzey ucuna, Sakarya vadisi ekseni boyunca tek seferde ~90 km geçiş.",
        "start_xy": (261000.0, 4419000.0),
        "goal_xy": (255000.0, 4499500.0),
    },
    {
        "id": "M90_02",
        "title_tr": "Doğu Sırtlar Hattı — Güney'den Kuzeye",
        "structure_type": "Yüksek Sırt & Plato Takibi (Eastern Ridge Line)",
        "description_tr": "Haritanın doğu kesiminde, güney sınırından kuzey sınırına yüksek sırtlar boyunca tek seferde ~90 km geçiş.",
        "start_xy": (300000.0, 4420000.0),
        "goal_xy": (292000.0, 4499000.0),
    },
    {
        "id": "M90_03",
        "title_tr": "Batı-Doğu Plato ve Sırt Aşımı",
        "structure_type": "Kıtasal Sırt Aşımı (West-East Ridge Crossing)",
        "description_tr": "Batı sınırından doğu sınırına, orta hattaki dağlık sırtları ve platoları tek seferde aşan ~90 km çapraz rota.",
        "start_xy": (230500.0, 4463000.0),
        "goal_xy": (309500.0, 4455000.0),
    },
    {
        "id": "M90_04",
        "title_tr": "Güneybatı-Kuzeydoğu Çapraz İntikal",
        "structure_type": "Mega Çapraz Geçiş (SW-NE Diagonal Transit)",
        "description_tr": "Güneybatı köşesinden kuzeydoğu köşesine, haritayı köşegen boyunca kateden tek seferde ~90 km intikal.",
        "start_xy": (231000.0, 4421000.0),
        "goal_xy": (289000.0, 4479000.0),
    },
    {
        "id": "M90_05",
        "title_tr": "Kuzeybatı-Güneydoğu Çapraz İntikal",
        "structure_type": "Mega Çapraz Geçiş (NW-SE Diagonal Transit)",
        "description_tr": "Kuzeybatı köşesinden güneydoğu köşesine, ters köşegen boyunca kateden tek seferde ~90 km intikal.",
        "start_xy": (233000.0, 4497000.0),
        "goal_xy": (291000.0, 4439000.0),
    },
]

START_ALT_OFFSET_M = 150.0
GOAL_ALT_OFFSET_M = 150.0


def plan_and_evaluate_single_shot(
    spec: Dict,
    terrain: TerrainQuery,
    cache: TerrainInfluenceCache,
    config: PlannerConfig,
    envelope: FixedWingKinematicEnvelope,
) -> Dict:
    mission_id = spec["id"]
    title = spec["title_tr"]
    sx, sy = spec["start_xy"]
    gx, gy = spec["goal_xy"]
    field = cache.field(config.lateral_buffer_m)

    straight_dist = math.hypot(gx - sx, gy - sy)

    r1, c1 = terrain.xy_to_rowcol(sx, sy)
    r2, c2 = terrain.xy_to_rowcol(gx, gy)
    s_ground = float(field.elevation_msl[r1, c1])
    g_ground = float(field.elevation_msl[r2, c2])

    start_z = s_ground + START_ALT_OFFSET_M
    goal_z = g_ground + GOAL_ALT_OFFSET_M
    heading = navigation_bearing_deg(sx, sy, gx, gy)

    start_pose = PhysicalPose(sx, sy, start_z, heading)
    goal_pose = GoalPose(gx, gy, goal_z)
    tol = GoalTolerance(xy_m=200.0, altitude_m=30.0)

    print(f"\n{'='*75}")
    print(f"[{mission_id}] {title} — SINGLE-SHOT (0 WAYPOINT)")
    print(f"  Topoloji: {spec['structure_type']}")
    print(f"  Baslangic: ({sx:.0f}, {sy:.0f}, {start_z:.1f}m MSL | Zemin: {s_ground:.1f}m)")
    print(f"  Hedef:     ({gx:.0f}, {gy:.0f}, {goal_z:.1f}m MSL | Zemin: {g_ground:.1f}m)")
    print(f"  Kusucusu Mesafe: {straight_dist/1000:.2f} km")
    print(f"{'='*75}", flush=True)

    print("  -> Tek Seferde (Single-Shot) Dual-Queue A* Arama Baslatiliyor...", flush=True)
    t0 = time.perf_counter()
    search_res = pose_aware_astar_search(
        start_pose,
        goal_pose,
        terrain,
        goal_tolerance=tol,
        config=config,
        envelope=envelope,
        max_expansions=80000,
        max_search_time_s=90.0,
    )
    search_time = time.perf_counter() - t0

    if not search_res.success:
        print(f"  ARAMA BASARISIZ: {search_res.termination_reason} ({search_time:.2f}s, {search_res.expanded_nodes} exp)")
        return {
            "mission_id": mission_id,
            "title_tr": title,
            "structure_type": spec["structure_type"],
            "description_tr": spec["description_tr"],
            "success": False,
            "termination_reason": search_res.termination_reason,
            "expanded_nodes": search_res.expanded_nodes,
            "search_time_s": round(search_time, 3),
            "straight_distance_km": round(straight_dist / 1000.0, 2),
        }

    path_len_km = search_res.continuous_path_length_m / 1000.0
    print(f"  -> A* Arama BASARILI ({search_time:.2f}s)! Acilan Dugum: {search_res.expanded_nodes}, Hat Uzunlugu: {path_len_km:.2f} km", flush=True)

    print("  -> Dusey Arazi Takibi Optimizasyonu (120m Hedef AGL)...", flush=True)
    t1 = time.perf_counter()
    profile_res = optimize_terrain_following_altitudes(
        search_res.trajectories, terrain, config=config, target_agl_m=120.0, envelope=envelope
    )
    profile_time = time.perf_counter() - t1

    if not profile_res.success:
        print(f"  UYARI: Dusey profil optimizasyonu basarisiz ({profile_res.status}), arama irtifasi kullaniliyor.")
        profiled_trajectories = search_res.trajectories
    else:
        print(f"  -> Dusey Profil Basarili ({profile_time:.3f}s)! Min AGL: {profile_res.minimum_agl_m:.1f}m", flush=True)
        profiled_trajectories = profile_res.trajectories

    print("  -> Koridor Guvenlikli Lokal B-Spline Yumusatma...", flush=True)
    t2 = time.perf_counter()
    smooth_res = apply_corridor_safe_local_bspline_smoothing(
        profiled_trajectories,
        terrain,
        config=config,
        speed_mps=40.0,
        sample_step_m=1.0,
        max_bank_rate_deg_s=15.0,
        max_corridor_deviation_m=12.0,
    )
    smooth_time_ms = (time.perf_counter() - t2) * 1000.0
    print(f"  -> B-Spline Tamamlandi ({smooth_time_ms:.1f}ms): {smooth_res.num_junctions_smoothed}/{smooth_res.num_junctions_total} birlesim yumusatildi.")
    print(f"     Max Roll Rate: {smooth_res.raw_max_bank_rate_deg_s:.1f} -> {smooth_res.smoothed_max_bank_rate_deg_s:.1f} deg/s, Max Sapma: {smooth_res.max_corridor_deviation_m:.2f}m", flush=True)

    plot_file = PLOTS_DIR / f"{mission_id.lower()}_fidelity_report.png"
    generate_single_shot_4panel_plot(spec, terrain, smooth_res, plot_file)

    total_dist_km = smooth_res.total_distance_m / 1000.0
    sinuosity = total_dist_km / (straight_dist / 1000.0)

    result_data = {
        "mission_id": mission_id,
        "title_tr": title,
        "structure_type": spec["structure_type"],
        "description_tr": spec["description_tr"],
        "planning_mode": "PURE_SINGLE_SHOT_ZERO_WAYPOINTS",
        "success": True,
        "expanded_nodes": search_res.expanded_nodes,
        "search_time_s": round(search_time, 3),
        "profile_time_s": round(profile_time, 3),
        "smoothing_time_ms": round(smooth_time_ms, 1),
        "total_planning_time_s": round(search_time + profile_time + smooth_time_ms / 1000.0, 3),
        "start_xy": [sx, sy],
        "goal_xy": [gx, gy],
        "start_ground_m": round(s_ground, 1),
        "goal_ground_m": round(g_ground, 1),
        "straight_distance_km": round(straight_dist / 1000.0, 2),
        "total_distance_km": round(total_dist_km, 2),
        "sinuosity": round(sinuosity, 3),
        "flight_time_min": round(smooth_res.total_time_s / 60.0, 2),
        "primitive_count": len(profiled_trajectories),
        "min_agl_m": round(smooth_res.min_agl_m, 1),
        "mean_agl_m": round(smooth_res.mean_agl_m, 1),
        "max_bank_angle_deg": round(smooth_res.smoothed_max_bank_angle_deg, 1),
        "max_roll_rate_deg_s": round(smooth_res.smoothed_max_bank_rate_deg_s, 1),
        "max_corridor_deviation_m": round(smooth_res.max_corridor_deviation_m, 2),
        "junctions_smoothed": smooth_res.num_junctions_smoothed,
        "junctions_total": smooth_res.num_junctions_total,
        "corridor_safe": smooth_res.corridor_safe,
        "plot_png": str(plot_file.relative_to(PROJECT_ROOT)),
    }
    return result_data


def generate_single_shot_4panel_plot(
    spec: Dict,
    terrain: TerrainQuery,
    smooth_res: LocalSmoothingResult,
    out_path: Path,
):
    fig = plt.figure(figsize=(18, 12), dpi=150)
    gs = GridSpec(2, 2, height_ratios=[1.1, 1.0], width_ratios=[1.1, 1.0], hspace=0.28, wspace=0.22)

    sm_pts = smooth_res.smoothed_points
    s_km = np.array([p.s_m / 1000.0 for p in sm_pts])
    x_m = np.array([p.x_m for p in sm_pts])
    y_m = np.array([p.y_m for p in sm_pts])
    z_msl = np.array([p.z_msl_m for p in sm_pts])
    ground_m = np.array([p.ground_m for p in sm_pts])
    bank_deg = np.array([p.bank_angle_deg for p in sm_pts])
    roll_rate = np.array([p.bank_rate_deg_s for p in sm_pts])

    roi = terrain.roi
    extent = [roi.bounds[0], roi.bounds[2], roi.bounds[1], roi.bounds[3]]

    ax1 = fig.add_subplot(gs[0, 0])
    im = ax1.imshow(roi.elevation, origin="upper", extent=extent, cmap="terrain", alpha=0.88)
    cb = plt.colorbar(im, ax=ax1, shrink=0.75, pad=0.02)
    cb.set_label("Elevation MSL (m)", fontsize=9)
    ax1.plot(x_m, y_m, "b-", lw=2.2, label=f"Single-Shot Path ({smooth_res.total_distance_m/1000:.2f} km)")

    sx, sy = spec["start_xy"]
    gx, gy = spec["goal_xy"]
    ax1.plot(sx, sy, "go", ms=10, label="Start")
    ax1.plot(gx, gy, "r*", ms=14, label="Goal")
    ax1.plot([sx, gx], [sy, gy], "k--", lw=1.0, alpha=0.5, label="Direct Line (No WPs)")

    ax1.set_title(f"1. Full 90x90km DEM: {spec['id']} ({spec['structure_type']})", fontsize=11, fontweight="bold")
    ax1.set_xlabel("UTM Easting (m)", fontsize=9)
    ax1.set_ylabel("UTM Northing (m)", fontsize=9)
    ax1.legend(loc="upper left", fontsize=8.5)
    ax1.grid(True, alpha=0.25)

    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(s_km, ground_m, color="saddlebrown", lw=1.5, label="Copernicus DEM Ground")
    ax2.plot(s_km, ground_m + 100.0, "r--", lw=1.0, alpha=0.7, label="Hard Safety Floor (100m AGL)")
    ax2.plot(s_km, ground_m + 120.0, "g:", lw=1.0, alpha=0.7, label="Target AGL (120m)")
    ax2.plot(s_km, z_msl, "b-", lw=2.0, label="Optimized UAV Altitude MSL")
    ax2.fill_between(s_km, ground_m, z_msl, color="skyblue", alpha=0.25)
    ax2.set_title(f"2. Altitude Profile (Min AGL: {smooth_res.min_agl_m:.1f}m | Mean: {smooth_res.mean_agl_m:.1f}m)", fontsize=11, fontweight="bold")
    ax2.set_xlabel("Flight Distance (km)", fontsize=9)
    ax2.set_ylabel("Altitude MSL (m)", fontsize=9)
    ax2.legend(loc="upper right", fontsize=8.5)
    ax2.grid(True, alpha=0.25)

    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(s_km, bank_deg, color="darkorange", lw=1.4, label="Continuous Bank Angle (deg)")
    ax3.axhline(25.0, color="r", ls="--", lw=1.0, label="Max Bank Limit (+-25 deg)")
    ax3.axhline(-25.0, color="r", ls="--", lw=1.0)
    ax3.set_title(f"3. Bank Angle Distribution (Max: {smooth_res.smoothed_max_bank_angle_deg:.1f} deg)", fontsize=11, fontweight="bold")
    ax3.set_xlabel("Flight Distance (km)", fontsize=9)
    ax3.set_ylabel("Bank Angle phi (deg)", fontsize=9)
    ax3.set_ylim(-30, 30)
    ax3.legend(loc="upper right", fontsize=8.5)
    ax3.grid(True, alpha=0.25)

    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(s_km, roll_rate, color="purple", lw=1.2, label="Local B-Spline Roll Rate (deg/s)")
    ax4.axhline(15.0, color="r", ls="--", lw=1.0, label="Roll Rate Cap (15 deg/s)")
    ax4.axhline(-15.0, color="r", ls="--", lw=1.0)
    ax4.set_title(f"4. Roll Rate Continuity (Peak: {smooth_res.smoothed_max_bank_rate_deg_s:.1f} deg/s)", fontsize=11, fontweight="bold")
    ax4.set_xlabel("Flight Distance (km)", fontsize=9)
    ax4.set_ylabel("Roll Rate dphi/dt (deg/s)", fontsize=9)
    ax4.set_ylim(-20, 20)
    ax4.legend(loc="upper right", fontsize=8.5)
    ax4.grid(True, alpha=0.25)

    fig.suptitle(f"Bilecik 90x90km — {spec['id']}: {spec['title_tr']}", fontsize=13, fontweight="bold", y=0.98)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  -> Grafik kaydedildi: {out_path.name}")


def generate_master_summary_atlas(results: List[Dict], out_path: Path):
    ok_results = [d for d in results if d.get("success")]
    fig = plt.figure(figsize=(18, 12), dpi=150)
    gs = GridSpec(2, 3, hspace=0.35, wspace=0.28)

    ids = [d["mission_id"] for d in ok_results]
    dist_km = [d["total_distance_km"] for d in ok_results]
    min_agls = [d["min_agl_m"] for d in ok_results]
    max_devs = [d["max_corridor_deviation_m"] for d in ok_results]
    search_times = [d["search_time_s"] for d in ok_results]
    roll_rates = [d["max_roll_rate_deg_s"] for d in ok_results]

    x = np.arange(len(ids))
    w = 0.45

    ax1 = fig.add_subplot(gs[0, 0])
    bars1 = ax1.bar(x, dist_km, width=w, color="steelblue")
    ax1.axhline(90.0, color="red", linestyle="--", lw=1.2, label="90 km Target")
    ax1.set_xticks(x)
    ax1.set_xticklabels(ids, fontweight="bold")
    ax1.set_ylabel("Total Distance (km)", fontweight="bold")
    ax1.set_title("1. Mission Trajectory Distances (~90 km)", fontweight="bold")
    for bar in bars1:
        y = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2.0, y + 0.5, f"{y:.1f}km", ha="center", va="bottom", fontsize=8.5, fontweight="bold")
    ax1.legend(loc="upper right", fontsize=8.5)
    ax1.grid(True, alpha=0.25)
    ax1.set_ylim(0, max(dist_km) * 1.2 if dist_km else 100)

    ax2 = fig.add_subplot(gs[0, 1])
    bars2 = ax2.bar(x, min_agls, width=w, color="cornflowerblue")
    ax2.axhline(100.0, color="red", linestyle="--", lw=1.5, label="Safety Limit (100m)")
    ax2.axhline(120.0, color="green", linestyle=":", lw=1.2, label="Target AGL (120m)")
    ax2.set_xticks(x)
    ax2.set_xticklabels(ids, fontweight="bold")
    ax2.set_ylabel("Minimum AGL (m)", fontweight="bold")
    ax2.set_title("2. Minimum AGL Clearance", fontweight="bold")
    for bar in bars2:
        y = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width()/2.0, y + 1.0, f"{y:.1f}m", ha="center", va="bottom", fontsize=8.5, fontweight="bold")
    ax2.legend(loc="lower right", fontsize=8.5)
    ax2.grid(True, alpha=0.25)
    ax2.set_ylim(80, 140)

    ax3 = fig.add_subplot(gs[0, 2])
    bars3 = ax3.bar(x, max_devs, width=w, color="mediumpurple")
    ax3.axhline(12.0, color="red", linestyle="--", lw=1.2, label="Corridor Budget (12m)")
    ax3.set_xticks(x)
    ax3.set_xticklabels(ids, fontweight="bold")
    ax3.set_ylabel("Max Lateral Deviation (m)", fontweight="bold")
    ax3.set_title("3. B-Spline Centerline Drift", fontweight="bold")
    for bar in bars3:
        y = bar.get_height()
        ax3.text(bar.get_x() + bar.get_width()/2.0, y + 0.05, f"{y:.2f}m", ha="center", va="bottom", fontsize=8.5, fontweight="bold")
    ax3.legend(loc="upper right", fontsize=8.5)
    ax3.grid(True, alpha=0.25)
    ax3.set_ylim(0, max(max_devs) * 1.4 if max_devs else 3.0)

    ax4 = fig.add_subplot(gs[1, 0])
    bars4 = ax4.bar(x, roll_rates, width=w, color="teal")
    ax4.axhline(15.0, color="red", linestyle="--", lw=1.2, label="Physical Limit (15 deg/s)")
    ax4.set_xticks(x)
    ax4.set_xticklabels(ids, fontweight="bold")
    ax4.set_ylabel("Peak Roll Rate (deg/s)", fontweight="bold")
    ax4.set_title("4. Roll Rate Continuity", fontweight="bold")
    for bar in bars4:
        y = bar.get_height()
        ax4.text(bar.get_x() + bar.get_width()/2.0, y + 0.3, f"{y:.1f} deg/s", ha="center", va="bottom", fontsize=8.5, fontweight="bold")
    ax4.legend(loc="upper right", fontsize=8.5)
    ax4.grid(True, alpha=0.25)
    ax4.set_ylim(0, max(roll_rates) * 1.3 if roll_rates else 18)

    ax5 = fig.add_subplot(gs[1, 1])
    bars5 = ax5.bar(x, search_times, width=w, color="salmon")
    ax5.set_xticks(x)
    ax5.set_xticklabels(ids, fontweight="bold")
    ax5.set_ylabel("A* Search Time (s)", fontweight="bold")
    ax5.set_title("5. Single-Shot A* Search Runtime", fontweight="bold")
    for bar in bars5:
        y = bar.get_height()
        ax5.text(bar.get_x() + bar.get_width()/2.0, y + 0.2, f"{y:.1f}s", ha="center", va="bottom", fontsize=8.5, fontweight="bold")
    ax5.grid(True, alpha=0.25)
    ax5.set_ylim(0, max(search_times) * 1.3 if search_times else 10)

    ax6 = fig.add_subplot(gs[1, 2])
    ax6.axis("off")
    table_data = [["ID", "Topoloji", "Mesafe", "Min AGL", "Sure", "Durum"]]
    for d in results:
        status = "PASS" if d.get("success") else "FAIL"
        table_data.append([
            d["mission_id"],
            d["structure_type"][:16] + ("..." if len(d["structure_type"]) > 16 else ""),
            f"{d.get('total_distance_km', 0):.1f} km" if d.get("success") else "-",
            f"{d.get('min_agl_m', 0):.1f} m" if d.get("success") else "-",
            f"{d.get('total_planning_time_s', d.get('search_time_s', 0)):.1f} s",
            status,
        ])
    t = ax6.table(cellText=table_data, loc="center", cellLoc="center")
    t.auto_set_font_size(False)
    t.set_fontsize(8.5)
    t.scale(1.15, 1.6)
    ax6.set_title("6. 5-Mission Verification Matrix", fontweight="bold")

    fig.suptitle("Bilecik 90x90 km — 5 Farkli Yon: Single-Shot A* & B-Spline Benchmark",
                 fontsize=14, fontweight="bold", y=0.98)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Master Summary Atlas kaydedildi: {out_path}")


def main():
    print("=" * 80)
    print("BILECIK 90x90 KM — 5 FARKLI YON, ~90 KM SINGLE-SHOT UCUS TESTI")
    print("=" * 80)

    cfg = dataclasses.replace(
        DEFAULT_CONFIG,
        working_dem_path=PROJECT_ROOT / "regions" / "bilecik" / "working_dem.tif",
        roi_size_m=90_000.0,
        roi_center_lonlat=(30.30, 40.25),
        target_crs="EPSG:32636",
        min_agl_m=100.0,
        lateral_buffer_m=60.0,
        search_heuristic_weight=1.05,
        enable_combined_turns=True,
        enable_pareto_z_pruning=False,
        enable_terrain_guidance=True,
        guidance_queue_ratio=3,
    )

    print("Loading Bilecik 90x90 km ROI DEM...")
    t_load = time.perf_counter()
    roi = load_roi(cfg)
    tq = TerrainQuery(roi)
    cache = TerrainInfluenceCache(tq)
    envelope = FixedWingKinematicEnvelope()
    print(f"  90x90 km DEM yuklendi ({time.perf_counter()-t_load:.2f}s): {roi.width}x{roi.height} px, bounds={roi.bounds}")

    all_results = []
    t_all_start = time.perf_counter()

    for spec in MISSIONS_90KM:
        res = plan_and_evaluate_single_shot(spec, tq, cache, cfg, envelope)
        all_results.append(res)

    total_elapsed = time.perf_counter() - t_all_start

    benchmark_json = RESULTS_DIR / "bilecik_90km_5_missions_benchmark.json"
    payload = {
        "region": "bilecik",
        "roi_size_km": 90.0,
        "total_benchmark_time_s": round(total_elapsed, 2),
        "missions_count": len(all_results),
        "success_count": sum(1 for d in all_results if d.get("success")),
        "missions": all_results,
    }
    benchmark_json.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\nSonuclar JSON dosyasina kaydedildi: {benchmark_json}")

    master_atlas_png = PLOTS_DIR / "bilecik_90km_5_missions_summary_atlas.png"
    generate_master_summary_atlas(all_results, master_atlas_png)

    print("\n" + "=" * 80)
    n_ok = sum(1 for d in all_results if d.get("success"))
    print("BENCHMARK TAMAMLANDI:")
    print(f"  Toplam Gorev: {n_ok} / {len(all_results)} BASARILI")
    ok_dist = sum(d["total_distance_km"] for d in all_results if d.get("success"))
    print(f"  Toplam Mesafe: {ok_dist:.2f} km")
    print(f"  Toplam Sure: {total_elapsed:.2f} s")
    print("=" * 80)


if __name__ == "__main__":
    main()
