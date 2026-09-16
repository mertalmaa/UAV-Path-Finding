"""Bilecik 90x90 km ROI üzerinde İKİ MAVİLIK arası tek görev testi.

Başlangıç: Batı Sakarya Vadisi (Osmaneli güneyi, nehir yatağı ~67m)
Bitiş:     Geyve Boğazı çıkışı / Doğu Sakarya Nehri (Adapazarı girişi, ~35m)

İki mavi alan da Sakarya Nehri'nin farklı kolları — planlayıcı aradaki
sıradağları, platoları ve çok sayıda kanyon geçişini aşmak zorunda.
"""
import dataclasses
import json
import math
import sys
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LightSource
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from planner.config import DEFAULT_CONFIG, PlannerConfig
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from planner.physical import PhysicalPose
from planner.pose_search import GoalPose, GoalTolerance, navigation_bearing_deg, pose_aware_astar_search
from planner.terrain_following import optimize_terrain_following_altitudes
from planner.trajectory_safety import TerrainInfluenceCache
from planner.local_trajectory_smoothing import apply_corridor_safe_local_bspline_smoothing

RESULTS_DIR = ROOT / "results" / "test_bilecik"
PLOTS_DIR = RESULTS_DIR / "plots"
PLOTS_DIR.mkdir(parents=True, exist_ok=True)

# ────────────────────────────────────────────────────
# GÖREV: İki Mavılık Arası  — Batı Sakarya ↔ Geyve Boğazı
# ────────────────────────────────────────────────────
# Başlangıç: Osmaneli güneyi, Sakarya nehir yatağı batı kolu
START_XY = (253500, 4483500)  # ~30.09°E, 40.46°N, yer yük. ~66m
# Bitiş: Geyve Boğazı çıkışı, Sakarya nehir yatağı doğu kolu
GOAL_XY  = (289500, 4501500)  # ~30.50°E, 40.63°N, yer yük. ~35m

MISSION_ID = "M_MAVI_MAVI"
MISSION_NAME = "Batı Sakarya Vadisi → Geyve Boğazı (İki Mavılık Arası)"


def get_elev(tq: TerrainQuery, x: float, y: float) -> float:
    r, c = tq.xy_to_rowcol(x, y)
    return tq.elevation_at_rowcol(r, c).elevation


def main():
    print("=" * 70)
    print("BİLECİK 90x90 KM — İKİ MAVİLIK ARASI ROTA TESTİ")
    print("Başlangıç: Batı Sakarya Vadisi (Osmaneli ~67m)")
    print("Hedef:     Geyve Boğazı / Doğu Sakarya (~35m)")
    print("=" * 70)

    cfg = dataclasses.replace(
        DEFAULT_CONFIG,
        working_dem_path=ROOT / "regions" / "bilecik" / "working_dem.tif",
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

    t_load = time.perf_counter()
    roi = load_roi(cfg)
    tq = TerrainQuery(roi)
    cache = TerrainInfluenceCache(tq)
    field = cache.field(60.0)
    print(f"\n  90x90 km DEM yüklendi ({time.perf_counter()-t_load:.2f}s): {roi.width}x{roi.height} px")

    sx, sy = START_XY
    gx, gy = GOAL_XY

    r1, c1 = tq.xy_to_rowcol(sx, sy)
    r2, c2 = tq.xy_to_rowcol(gx, gy)
    s_ground = float(field.elevation_msl[r1, c1])
    g_ground = float(field.elevation_msl[r2, c2])

    start_z = s_ground + 150.0   # 150m AGL start
    goal_z  = g_ground + 150.0

    heading = navigation_bearing_deg(sx, sy, gx, gy)
    start_pose = PhysicalPose(sx, sy, start_z, heading)
    goal_pose  = GoalPose(gx, gy, goal_z)
    tol = GoalTolerance(xy_m=180.0, altitude_m=200.0)

    dx = gx - sx
    dy = gy - sy
    straight_km = math.sqrt(dx**2 + dy**2) / 1000.0

    print(f"\n  Başlangıç: ({sx:.0f}, {sy:.0f})  yer={s_ground:.1f}m  başlangıç_z={start_z:.1f}m MSL")
    print(f"  Hedef:     ({gx:.0f}, {gy:.0f})  yer={g_ground:.1f}m  hedef_z={goal_z:.1f}m MSL")
    print(f"  Kuş uçuşu: {straight_km:.2f} km  |  Başlık: {heading:.1f}°")

    print(f"\n  → Dual-Queue RR-MHA* Arama Başlatılıyor...")
    t0 = time.perf_counter()
    search_res = pose_aware_astar_search(
        start_pose, goal_pose, tq, goal_tolerance=tol,
        config=cfg, max_expansions=60_000, max_search_time_s=60.0,
    )
    t_search = time.perf_counter() - t0

    if not search_res.success:
        print(f"  ✗ ARAMA BAŞARISIZ: {search_res.termination_reason} ({t_search:.2f}s, {search_res.expanded_nodes} exp)")
        return

    print(f"  ✓ ARAMA BAŞARILI ({t_search:.2f}s)  |  Expansions: {search_res.expanded_nodes}")

    # Vertical terrain-following optimization
    print(f"  → Dikey arazi takibi profili hesaplanıyor (120m hedef AGL)...")
    t0 = time.perf_counter()
    opt_res = optimize_terrain_following_altitudes(
        search_res.trajectories, tq, config=cfg, target_agl_m=120.0
    )
    t_opt = time.perf_counter() - t0
    active_trajs = opt_res.trajectories if (opt_res and opt_res.success) else search_res.trajectories
    print(f"  ✓ Dikey profil ({t_opt:.2f}s)  Min AGL: {opt_res.minimum_agl_m:.1f}m" if opt_res and opt_res.success else f"  (fallback)")

    # B-Spline smoothing
    print(f"  → Lokal B-Spline yumuşatma...")
    t0 = time.perf_counter()
    smoothed = apply_corridor_safe_local_bspline_smoothing(
        active_trajs, tq, config=cfg,
    )
    t_smooth = time.perf_counter() - t0
    if smoothed and smoothed.num_junctions_total > 0:
        print(f"  ✓ B-Spline ({t_smooth*1000:.0f}ms): {smoothed.num_junctions_smoothed}/{smoothed.num_junctions_total} birleşim yumuşatıldı")
    else:
        print(f"  (B-Spline atlandı / uygulanamadı)")
        smoothed = None

    # Build dense sample points
    dense = []
    if smoothed and smoothed.smoothed_points:
        # Use B-Spline smoothed points directly
        for pt in smoothed.smoothed_points:
            dense.append({
                "dist_m": pt.s_m,
                "x": pt.x_m, "y": pt.y_m,
                "z_msl": pt.z_msl_m, "ground": pt.ground_m,
                "agl": pt.agl_m,
            })
        route_km = smoothed.total_distance_m / 1000.0
    else:
        # Fall back to trajectory samples
        cum = 0.0
        for traj in active_trajs:
            for s in traj.samples:
                g = get_elev(tq, s.x_m, s.y_m)
                dense.append({
                    "dist_m": cum + s.horizontal_distance_along_path_m,
                    "x": s.x_m, "y": s.y_m,
                    "z_msl": s.z_msl_m, "ground": g,
                    "agl": s.z_msl_m - g,
                })
            cum += traj.horizontal_arc_length_m
        route_km = cum / 1000.0

    xs = [p["x"] for p in dense]
    ys = [p["y"] for p in dense]
    zs = [p["z_msl"] for p in dense]
    gds = [p["ground"] for p in dense]
    agls = [p["agl"] for p in dense]
    dists = [p["dist_m"] for p in dense]
    min_agl = float(np.min(agls))
    mean_agl = float(np.mean(agls))

    print(f"\n  ═══════════════════════════════════════")
    print(f"  Rota Uzunluğu:   {route_km:.2f} km")
    print(f"  Arama Süresi:    {t_search:.2f} s")
    print(f"  Expansions:      {search_res.expanded_nodes:,}")
    print(f"  Min AGL:         {min_agl:.1f} m")
    print(f"  Ort AGL:         {mean_agl:.1f} m")
    print(f"  ═══════════════════════════════════════")

    # ──────────────────────────────────────────
    # PLOTTING: 4-panel figure
    # ──────────────────────────────────────────
    elev_grid = roi.elevation.copy()
    valid_e = elev_grid[elev_grid != roi.nodata]
    e_min, e_max = float(valid_e.min()), float(valid_e.max())
    elev_grid[elev_grid == roi.nodata] = e_min
    extent = [roi.bounds[0], roi.bounds[2], roi.bounds[1], roi.bounds[3]]

    fig = plt.figure(figsize=(20, 14), dpi=200)
    fig.patch.set_facecolor("#0a0e1a")

    # Panel 1: Full 90km DEM with route
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.set_facecolor("#0a0e1a")
    im1 = ax1.imshow(elev_grid, extent=extent, origin="upper",
                      cmap="terrain", alpha=0.9, vmin=e_min, vmax=e_max)
    plt.colorbar(im1, ax=ax1, fraction=0.046, pad=0.04).set_label("Yükseklik (m MSL)", color="white", fontsize=8)
    ax1.plot([sx, gx], [sy, gy], "w--", lw=1.5, alpha=0.6, label="Kuş Uçuşu")
    ax1.plot(xs, ys, color="#00e5ff", lw=2.5, label=f"İHA Rotası ({route_km:.1f} km)", zorder=5)
    ax1.scatter([sx], [sy], color="#00ff88", edgecolors="white", s=180, zorder=8, label="Başlangıç\n(Batı Sakarya)", marker="^")
    ax1.scatter([gx], [gy], color="#ff1744", edgecolors="white", s=180, zorder=8, label="Hedef\n(Geyve Boğazı)", marker="v")
    ax1.set_title("Bilecik 90×90 km DEM — İki Mavılık Arası Rota", color="white", fontsize=11, fontweight="bold")
    ax1.set_xlabel("UTM Easting X (m)", color="white", fontsize=9)
    ax1.set_ylabel("UTM Northing Y (m)", color="white", fontsize=9)
    ax1.tick_params(colors="white")
    ax1.legend(loc="lower right", fontsize=8, framealpha=0.7, facecolor="#1a1e2e", labelcolor="white")
    ax1.grid(True, linestyle=":", alpha=0.3, color="white")

    # Panel 2: Zoomed route view
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.set_facecolor("#0a0e1a")
    margin = 4000
    im2 = ax2.imshow(elev_grid, extent=extent, origin="upper",
                      cmap="terrain", alpha=0.92, vmin=e_min, vmax=e_max)
    ax2.plot(xs, ys, color="#00e5ff", lw=3.0, zorder=5, label=f"Rota ({route_km:.1f} km)")
    ax2.scatter([sx], [sy], color="#00ff88", edgecolors="white", s=200, zorder=8, marker="^", label="Batı Sakarya Vadisi")
    ax2.scatter([gx], [gy], color="#ff1744", edgecolors="white", s=200, zorder=8, marker="v", label="Geyve Boğazı")
    ax2.set_xlim(min(xs) - margin, max(xs) + margin)
    ax2.set_ylim(min(ys) - margin, max(ys) + margin)
    ax2.set_title("Yakın Çekim — Rota Detayı", color="white", fontsize=11, fontweight="bold")
    ax2.set_xlabel("UTM Easting X (m)", color="white", fontsize=9)
    ax2.set_ylabel("UTM Northing Y (m)", color="white", fontsize=9)
    ax2.tick_params(colors="white")
    ax2.legend(loc="upper right", fontsize=8, framealpha=0.7, facecolor="#1a1e2e", labelcolor="white")
    ax2.grid(True, linestyle=":", alpha=0.3, color="white")
    plt.colorbar(im2, ax=ax2, fraction=0.046, pad=0.04).set_label("Yükseklik (m MSL)", color="white", fontsize=8)

    # Panel 3: Elevation profile with terrain fill
    ax3 = fig.add_subplot(2, 1, 2)
    ax3.set_facecolor("#0d1117")
    gd_np = np.array(gds)
    zs_np = np.array(zs)
    dists_np = np.array(dists)
    ax3.fill_between(dists_np, 0, gd_np, color="#5c3a1e", alpha=0.6, label="Arazi (Bilecik 90km DEM)")
    ax3.plot(dists_np, gd_np, color="#8b6914", lw=1.5, alpha=0.9, label="Arazi Profili")
    ax3.plot(dists_np, gd_np + 100.0, "r--", lw=1.3, alpha=0.7, label="Min Emniyet AGL (+100m)")
    ax3.plot(dists_np, gd_np + 120.0, "g:", lw=1.3, alpha=0.7, label="Hedef AGL (+120m)")
    ax3.plot(dists_np, zs_np, color="#00e5ff", lw=2.8, label=f"İHA İrtifası (MSL) — Min AGL: {min_agl:.1f}m", zorder=5)

    # AGL fill
    ax3.fill_between(dists_np, gd_np, zs_np, alpha=0.18, color="#00e5ff")

    # Mark start/end
    ax3.axvline(x=dists_np[0], color="#00ff88", lw=1.5, linestyle="--", alpha=0.7)
    ax3.axvline(x=dists_np[-1], color="#ff1744", lw=1.5, linestyle="--", alpha=0.7)
    ax3.text(dists_np[0] + 500, max(zs_np)*0.98, "Batı Sakarya\n(~67m)", color="#00ff88", fontsize=8, ha="left")
    ax3.text(dists_np[-1] - 500, max(zs_np)*0.98, "Geyve Boğazı\n(~35m)", color="#ff1744", fontsize=8, ha="right")

    ax3.set_title(
        f"Dikey Arazi Takibi Profili  |  Rota: {route_km:.1f} km  |  Arama: {t_search:.2f}s  |  {search_res.expanded_nodes:,} Expansion  |  Min AGL: {min_agl:.1f}m  |  Ort AGL: {mean_agl:.1f}m",
        color="white", fontsize=10, fontweight="bold"
    )
    ax3.set_xlabel("Yörünge Boyunca Mesafe (m)", color="white", fontsize=10)
    ax3.set_ylabel("İrtifa MSL (m)", color="white", fontsize=10)
    ax3.set_ylim(max(0, float(gd_np.min()) - 80), float(zs_np.max()) + 150)
    ax3.tick_params(colors="white")
    ax3.legend(loc="upper right", fontsize=8.5, framealpha=0.8, facecolor="#1a1e2e", labelcolor="white")
    ax3.grid(True, linestyle=":", alpha=0.3, color="white")

    fig.suptitle(
        f"BİLECİK 90×90 KM — İKİ MAVİLIK ARASI ROTA\n"
        f"Batı Sakarya Vadisi (~67m) → Geyve Boğazı (~35m) | {route_km:.1f} km | {t_search:.2f}s arama | {search_res.expanded_nodes:,} expansion",
        color="white", fontsize=13, fontweight="bold", y=1.01
    )
    plt.tight_layout()

    out_path = PLOTS_DIR / "m_mavi_mavi_two_rivers_90km.png"
    fig.savefig(out_path, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"\n  → PNG kaydedildi: {out_path}")

    # Save JSON
    result = {
        "mission_id": MISSION_ID,
        "title": MISSION_NAME,
        "start_xy": list(START_XY), "start_ground_m": round(s_ground, 1), "start_z_msl": round(start_z, 1),
        "goal_xy": list(GOAL_XY), "goal_ground_m": round(g_ground, 1), "goal_z_msl": round(goal_z, 1),
        "straight_km": round(straight_km, 2),
        "route_km": round(route_km, 2),
        "search_time_s": round(t_search, 3),
        "expanded_nodes": search_res.expanded_nodes,
        "min_agl_m": round(min_agl, 1),
        "mean_agl_m": round(mean_agl, 1),
        "success": True,
    }
    json_path = RESULTS_DIR / "m_mavi_mavi_result.json"
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print(f"  → JSON kaydedildi: {json_path}")
    print("\n  ✅ TAMAMLANDI")


if __name__ == "__main__":
    main()
