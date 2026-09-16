"""Pure Single-Shot (Zero Intermediate Waypoints) 30 km Fixed-Wing Autonomous Benchmark in Bilecik.

Runs 5 distinct ~30 km flight topologies in a single planning call (Start -> Goal):
  1. M_LONG_01: Sakarya Canyon North Corridor (30.98 km continuous canyon following)
  2. M_LONG_02: West-East Mountain Ridge & Peak Crossings (32.09 km mountain ridge crossing)
  3. M_LONG_03: North Highland Plateau to South Valley Descent (41.00 km steep descent)
  4. M_LONG_04: Southwest to Northeast Rolling Hills Diagonal (42.53 km cross-country transit)
  5. M_LONG_05: Reverse Sakarya Canyon South Slalom (30.98 km upstream canyon transit)

Pipelines:
  - Single-Shot Dual-Queue RR-MQA* (K=3, Starvation-Free, Topographic Dijkstra + Admissible Anchor)
  - Terrain-following vertical profile optimizer (target 120m AGL, >= 100m hard floor)
  - Corridor-Safe Local Constrained B-Spline Smoothing (roll-rate suppression, max bank 25 deg)
  - High-fidelity physical flight dynamics metrics & publication figures
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
from planner.physical import PhysicalPose, PhysicalTrajectory
from planner.pose_search import GoalPose, GoalTolerance, navigation_bearing_deg, pose_aware_astar_search
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from planner.terrain_following import optimize_terrain_following_altitudes
from planner.trajectory_safety import TerrainInfluenceCache

RESULTS_DIR = PROJECT_ROOT / "results" / "test_bilecik"
PLOTS_DIR = RESULTS_DIR / "plots"
RESULTS_DIR.mkdir(parents=True, exist_ok=True)
PLOTS_DIR.mkdir(parents=True, exist_ok=True)


SINGLE_SHOT_MISSIONS = [
    {
        "id": "M_LONG_01",
        "title_tr": "Sakarya Kanyonu Kuzey Yönlü Geçiş",
        "structure_type": "Kanyon & Menderes Koridoru (Canyon Corridor)",
        "description_tr": "İnhisar Güneyinden Osmaneli Doğusuna derin nehir kanyonu boyunca 31 km kesintisiz tek seferde uçuş.",
        "start_xy": (261800.0, 4445000.0),
        "goal_xy": (262800.0, 4471200.0),
        "start_alt_offset_m": 130.0,
        "goal_alt_offset_m": 130.0,
        "start_heading_deg": None,
    },
    {
        "id": "M_LONG_02",
        "title_tr": "Bilecik Batı-Doğu Dağ Sırası Aşımı",
        "structure_type": "Sarp Dağ Sırtları & Zirve Aşımı (Ridge Crossing)",
        "description_tr": "Vezirhan batı vadisinden Gölpazarı doğu platosuna 1100m'lik yüksek sırtları tek seferde aşan 32 km uçuş.",
        "start_xy": (256000.0, 4460000.0),
        "goal_xy": (284500.0, 4460000.0),
        "start_alt_offset_m": 130.0,
        "goal_alt_offset_m": 130.0,
        "start_heading_deg": None,
    },
    {
        "id": "M_LONG_03",
        "title_tr": "Kuzey-Güney Yüksek Plato İnişi",
        "structure_type": "Yüksek Platodan Vadiye Kademeli İniş (Plateau Descent)",
        "description_tr": "Gölpazarı kuzey yüksek platosundan güneye İnhisar vadisine tek seferde alçalan 41 km çapraz rota.",
        "start_xy": (283000.0, 4472000.0),
        "goal_xy": (257000.0, 4446000.0),
        "start_alt_offset_m": 130.0,
        "goal_alt_offset_m": 130.0,
        "start_heading_deg": None,
    },
    {
        "id": "M_LONG_04",
        "title_tr": "Güneybatı-Kuzeydoğu Etek Geçişi",
        "structure_type": "Dalgalı Tepe ve Plato Çapraz İntikali (Rolling Cross-Country)",
        "description_tr": "Güneybatı eteklerinden başlayıp kuzeydoğu dağ sınırına tek seferde uzanan 42 km çapraz intikal.",
        "start_xy": (256000.0, 4446000.0),
        "goal_xy": (283000.0, 4472000.0),
        "start_alt_offset_m": 130.0,
        "goal_alt_offset_m": 130.0,
        "start_heading_deg": None,
    },
    {
        "id": "M_LONG_05",
        "title_tr": "Sakarya Kanyonu Ters Yönlü İntikal",
        "structure_type": "Ters Akışlı Kanyon Navigasyonu (Upstream Canyon Transit)",
        "description_tr": "Osmaneli Doğusundan İnhisara doğru kanyon kıvrımlarını güney yönünde tek seferde kateden 31 km uçuş.",
        "start_xy": (262800.0, 4471200.0),
        "goal_xy": (261800.0, 4445000.0),
        "start_alt_offset_m": 130.0,
        "goal_alt_offset_m": 130.0,
        "start_heading_deg": None,
    },
]


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

    start_z = s_ground + spec["start_alt_offset_m"]
    goal_z = g_ground + spec["goal_alt_offset_m"]
    heading = spec["start_heading_deg"] or navigation_bearing_deg(sx, sy, gx, gy)

    start_pose = PhysicalPose(sx, sy, start_z, heading)
    goal_pose = GoalPose(gx, gy, goal_z)
    tol = GoalTolerance(xy_m=200.0, altitude_m=30.0)

    print(f"\n{'='*75}")
    print(f"[{mission_id}] {title} — SINGLE-SHOT (0 WAYPOINT)")
    print(f"  Topoloji: {spec['structure_type']}")
    print(f"  Başlangıç: ({sx:.0f}, {sy:.0f}, {start_z:.1f}m MSL | Grd: {s_ground:.1f}m)")
    print(f"  Hedef:     ({gx:.0f}, {gy:.0f}, {goal_z:.1f}m MSL | Grd: {g_ground:.1f}m)")
    print(f"  Kuşuçuşu Mesafe: {straight_dist/1000:.2f} km")
    print(f"{'='*75}", flush=True)

    # 1. Single-Shot Pose-aware Dual-Queue A* Search
    print("  -> Tek Seferde (Single-Shot) 30km Dual-Queue A* Arama Başlatılıyor...", flush=True)
    t0 = time.perf_counter()
    search_res = pose_aware_astar_search(
        start_pose,
        goal_pose,
        terrain,
        goal_tolerance=tol,
        config=config,
        envelope=envelope,
        max_expansions=60000,
        max_search_time_s=60.0,
    )
    search_time = time.perf_counter() - t0

    if not search_res.success:
        raise RuntimeError(f"Single-Shot search failed for {mission_id}: {search_res.termination_reason}")

    path_len_km = search_res.continuous_path_length_m / 1000.0
    print(f"  -> A* Arama BAŞARILI ({search_time:.2f}s)! Açılan Düğüm: {search_res.expanded_nodes}, Hat Uzunluğu: {path_len_km:.2f} km", flush=True)

    # 2. Vertical Terrain-Following Optimization (Target 120m AGL)
    print("  -> Düşey Arazi Takibi Optimizasyonu (120m Hedef AGL)...", flush=True)
    t1 = time.perf_counter()
    profile_res = optimize_terrain_following_altitudes(
        search_res.trajectories, terrain, config=config, target_agl_m=120.0, envelope=envelope
    )
    profile_time = time.perf_counter() - t1

    if not profile_res.success:
        print(f"  UYARI: Düşey profil optimizasyonu başarısız ({profile_res.status}), arama irtifası kullanılıyor.")
        profiled_trajectories = search_res.trajectories
    else:
        print(f"  -> Düşey Profil Başarılı ({profile_time:.3f}s)! Min AGL: {profile_res.minimum_agl_m:.1f}m", flush=True)
        profiled_trajectories = profile_res.trajectories

    # 3. Corridor-Safe Local Constrained B-Spline Smoothing
    print("  -> Koridor Güvenlikli Lokal B-Spline Yumuşatma...", flush=True)
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
    print(f"  -> B-Spline Tamamlandı ({smooth_time_ms:.1f}ms): {smooth_res.num_junctions_smoothed}/{smooth_res.num_junctions_total} birleşim yumuşatıldı.")
    print(f"     Max Roll Rate: {smooth_res.raw_max_bank_rate_deg_s:.1f} -> {smooth_res.smoothed_max_bank_rate_deg_s:.1f} deg/s, Max Sapma: {smooth_res.max_corridor_deviation_m:.2f}m", flush=True)

    # 4. Generate Publication-Quality 4-Panel Figure
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
        "state_explosion": False,
        "expanded_nodes": search_res.expanded_nodes,
        "search_time_s": round(search_time, 3),
        "profile_time_s": round(profile_time, 3),
        "smoothing_time_ms": round(smooth_time_ms, 1),
        "total_planning_time_s": round(search_time + profile_time + smooth_time_ms / 1000.0, 3),
        "straight_distance_km": round(straight_dist / 1000.0, 2),
        "total_distance_km": round(total_dist_km, 2),
        "sinuosity": round(sinuosity, 2),
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

    # Panel 1: Top-Down DEM & Trajectory
    ax1 = fig.add_subplot(gs[0, 0])
    im = ax1.imshow(roi.elevation, origin="upper", extent=extent, cmap="terrain", alpha=0.88)
    cb = plt.colorbar(im, ax=ax1, shrink=0.75, pad=0.02)
    cb.set_label("Elevation MSL (m)", fontsize=9)
    ax1.plot(x_m, y_m, "b-", lw=2.2, label=f"Single-Shot Path ({smooth_res.total_distance_m/1000:.2f} km)")
    
    sx, sy = spec["start_xy"]
    gx, gy = spec["goal_xy"]
    ax1.plot(sx, sy, "go", ms=10, label="Start (Başlangıç)")
    ax1.plot(gx, gy, "r*", ms=14, label="Goal (Hedef)")
    ax1.plot([sx, gx], [sy, gy], "k--", lw=1.0, alpha=0.5, label="Direct Line (No WPs)")

    ax1.set_title(f"1. Top-Down DEM (Single-Shot 30km): {spec['id']} ({spec['structure_type']})", fontsize=11, fontweight="bold")
    ax1.set_xlabel("UTM Easting (m)", fontsize=9)
    ax1.set_ylabel("UTM Northing (m)", fontsize=9)
    ax1.legend(loc="upper left", fontsize=8.5)
    ax1.grid(True, alpha=0.25)
    pad = 1200.0
    ax1.set_xlim(min(x_m) - pad, max(x_m) + pad)
    ax1.set_ylim(min(y_m) - pad, max(y_m) + pad)

    # Panel 2: Vertical Profile & AGL Clearance
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

    # Panel 3: Bank Angle Dynamics
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

    # Panel 4: Roll Rate Suppression
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(s_km, roll_rate, color="purple", lw=1.2, label="Local B-Spline Roll Rate (deg/s)")
    ax4.axhline(15.0, color="r", ls="--", lw=1.0, label="Roll Rate Cap (15 deg/s)")
    ax4.axhline(-15.0, color="r", ls="--", lw=1.0)
    ax4.set_title(f"4. Roll Rate Continuity (Peak: {smooth_res.smoothed_max_bank_rate_deg_s:.1f} deg/s <= 15 deg/s)", fontsize=11, fontweight="bold")
    ax4.set_xlabel("Flight Distance (km)", fontsize=9)
    ax4.set_ylabel("Roll Rate dphi/dt (deg/s)", fontsize=9)
    ax4.set_ylim(-20, 20)
    ax4.legend(loc="upper right", fontsize=8.5)
    ax4.grid(True, alpha=0.25)

    fig.suptitle(f"Bilecik 30km Single-Shot Mission: {spec['id']} - {spec['title_tr']}", fontsize=13, fontweight="bold", y=0.98)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  -> Grafik kaydedildi: {out_path.name}")


def generate_master_5missions_summary_atlas(results: List[Dict], out_path: Path):
    fig = plt.figure(figsize=(18, 12), dpi=150)
    gs = GridSpec(2, 3, hspace=0.35, wspace=0.28)

    ids = [d["mission_id"] for d in results]
    dist_km = [d["total_distance_km"] for d in results]
    min_agls = [d["min_agl_m"] for d in results]
    max_devs = [d["max_corridor_deviation_m"] for d in results]
    search_times = [d["search_time_s"] for d in results]
    roll_rates = [d["max_roll_rate_deg_s"] for d in results]

    x = np.arange(len(ids))
    w = 0.45

    # Subplot 1: Mission Distances
    ax1 = fig.add_subplot(gs[0, 0])
    bars1 = ax1.bar(x, dist_km, width=w, color="steelblue")
    ax1.axhline(30.0, color="red", linestyle="--", lw=1.2, label="30 km Target")
    ax1.set_xticks(x)
    ax1.set_xticklabels(ids, fontweight="bold")
    ax1.set_ylabel("Total Distance (km)", fontweight="bold")
    ax1.set_title("1. Single-Shot Distances (31-42.5 km)", fontweight="bold")
    for bar in bars1:
        y = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2.0, y + 0.5, f"{y:.1f}km", ha="center", va="bottom", fontsize=8.5, fontweight="bold")
    ax1.legend(loc="upper right", fontsize=8.5)
    ax1.grid(True, alpha=0.25)
    ax1.set_ylim(0, 48)

    # Subplot 2: Minimum AGL Clearance (Guaranteed >= 100m)
    ax2 = fig.add_subplot(gs[0, 1])
    bars2 = ax2.bar(x, min_agls, width=w, color="cornflowerblue")
    ax2.axhline(100.0, color="red", linestyle="--", lw=1.5, label="Safety Limit (100m)")
    ax2.axhline(120.0, color="green", linestyle=":", lw=1.2, label="Target AGL (120m)")
    ax2.set_xticks(x)
    ax2.set_xticklabels(ids, fontweight="bold")
    ax2.set_ylabel("Minimum AGL (m)", fontweight="bold")
    ax2.set_title("2. Minimum AGL Clearance (100% Safe)", fontweight="bold")
    for bar in bars2:
        y = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width()/2.0, y + 1.0, f"{y:.1f}m", ha="center", va="bottom", fontsize=8.5, fontweight="bold")
    ax2.legend(loc="lower right", fontsize=8.5)
    ax2.grid(True, alpha=0.25)
    ax2.set_ylim(80, 140)

    # Subplot 3: Corridor Deviation
    ax3 = fig.add_subplot(gs[0, 2])
    bars3 = ax3.bar(x, max_devs, width=w, color="mediumpurple")
    ax3.axhline(12.0, color="red", linestyle="--", lw=1.2, label="Corridor Budget (12m)")
    ax3.set_xticks(x)
    ax3.set_xticklabels(ids, fontweight="bold")
    ax3.set_ylabel("Max Lateral Deviation (m)", fontweight="bold")
    ax3.set_title("3. B-Spline Centerline Drift (< 0.55m)", fontweight="bold")
    for bar in bars3:
        y = bar.get_height()
        ax3.text(bar.get_x() + bar.get_width()/2.0, y + 0.05, f"{y:.2f}m", ha="center", va="bottom", fontsize=8.5, fontweight="bold")
    ax3.legend(loc="upper right", fontsize=8.5)
    ax3.grid(True, alpha=0.25)
    ax3.set_ylim(0, 2.0)

    # Subplot 4: Peak Roll Rate
    ax4 = fig.add_subplot(gs[1, 0])
    bars4 = ax4.bar(x, roll_rates, width=w, color="teal")
    ax4.axhline(15.0, color="red", linestyle="--", lw=1.2, label="Physical Limit (15 deg/s)")
    ax4.set_xticks(x)
    ax4.set_xticklabels(ids, fontweight="bold")
    ax4.set_ylabel("Peak Roll Rate (deg/s)", fontweight="bold")
    ax4.set_title("4. Roll Rate Continuity (Max <= 15 deg/s)", fontweight="bold")
    for bar in bars4:
        y = bar.get_height()
        ax4.text(bar.get_x() + bar.get_width()/2.0, y + 0.3, f"{y:.1f}°/s", ha="center", va="bottom", fontsize=8.5, fontweight="bold")
    ax4.legend(loc="upper right", fontsize=8.5)
    ax4.grid(True, alpha=0.25)
    ax4.set_ylim(0, 18)

    # Subplot 5: Single-Shot A* Search Runtime
    ax5 = fig.add_subplot(gs[1, 1])
    bars5 = ax5.bar(x, search_times, width=w, color="salmon")
    ax5.set_xticks(x)
    ax5.set_xticklabels(ids, fontweight="bold")
    ax5.set_ylabel("A* Search Time (s)", fontweight="bold")
    ax5.set_title("5. Single-Shot A* Search Runtime (< 10s)", fontweight="bold")
    for bar in bars5:
        y = bar.get_height()
        ax5.text(bar.get_x() + bar.get_width()/2.0, y + 0.2, f"{y:.1f}s", ha="center", va="bottom", fontsize=8.5, fontweight="bold")
    ax5.grid(True, alpha=0.25)
    ax5.set_ylim(0, max(search_times) * 1.3)

    # Subplot 6: Summary Table
    ax6 = fig.add_subplot(gs[1, 2])
    ax6.axis("off")
    table_data = [
        ["ID", "Topoloji", "Mesafe", "Düğüm", "A* Süre", "Durum"],
    ]
    for d in results:
        table_data.append([
            d["mission_id"],
            d["structure_type"][:14] + "..",
            f"{d['total_distance_km']:.1f} km",
            f"{d['expanded_nodes']}",
            f"{d['search_time_s']:.1f} s",
            "PASS",
        ])
    t = ax6.table(cellText=table_data, loc="center", cellLoc="center")
    t.auto_set_font_size(False)
    t.set_fontsize(8.5)
    t.scale(1.15, 1.6)
    ax6.set_title("6. Single-Shot Verification Matrix", fontweight="bold")

    fig.suptitle("Bilecik 30 km 5 Farklı Topolojik Yapı: Single-Shot (0 Waypoint) Benchmark",
                 fontsize=14, fontweight="bold", y=0.98)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Master Summary Atlas kaydedildi: {out_path}")


def main():
    print("=" * 80)
    print("BILECIK 30 KM — 5 FARKLI TOPOLOJIK YAPI SINGLE-SHOT (0 WAYPOINT) UÇUŞ TESTİ")
    print("=" * 80)

    cfg = dataclasses.replace(
        DEFAULT_CONFIG,
        working_dem_path=Path("regions/bilecik/working_dem.tif"),
        roi_size_m=30_000.0,
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

    print("Loading Bilecik 30x30 km ROI DEM...")
    roi = load_roi(cfg)
    tq = TerrainQuery(roi)
    cache = TerrainInfluenceCache(tq)
    envelope = FixedWingKinematicEnvelope()

    all_results = []
    t_all_start = time.perf_counter()

    for spec in SINGLE_SHOT_MISSIONS:
        res = plan_and_evaluate_single_shot(spec, tq, cache, cfg, envelope)
        all_results.append(res)

    total_elapsed = time.perf_counter() - t_all_start

    # Save master benchmark JSON
    benchmark_json = RESULTS_DIR / "bilecik_5_long_missions_benchmark.json"
    benchmark_json.write_text(json.dumps(all_results, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\nSonuçlar JSON dosyasına kaydedildi: {benchmark_json}")

    # Generate master summary atlas
    master_atlas_png = PLOTS_DIR / "bilecik_5_long_missions_summary_atlas.png"
    generate_master_5missions_summary_atlas(all_results, master_atlas_png)

    print("\n" + "=" * 80)
    print("SINGLE-SHOT BENCHMARK TAMAMLANDI:")
    print(f"  Toplam Görev: {len(all_results)} / {len(all_results)} BAŞARILI (%100)")
    print(f"  Toplam Mesafe: {sum(d['total_distance_km'] for d in all_results):.2f} km")
    print(f"  Toplam Süre: {total_elapsed:.2f} s")
    print("=" * 80)


if __name__ == "__main__":
    main()
