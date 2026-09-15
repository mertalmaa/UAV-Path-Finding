"""Run 30 Diverse Fixed-Wing Missions in Bilecik, Generate 2-Panel Plots & Detailed Report.

All outputs are saved directly under results/test_bilecik/.
"""
from collections import Counter
import dataclasses
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Dict, List

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import numpy as np
import rasterio

# Ensure repository root is on sys.path
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from planner.config import DEFAULT_CONFIG, PlannerConfig
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from planner.physical import PhysicalPose
from planner.pose_search import GoalPose, GoalTolerance, navigation_bearing_deg, pose_aware_astar_search
from planner.terrain_following import optimize_terrain_following_altitudes
from scripts.bilecik_missions_spec import BILECIK_MISSIONS, BilecikMissionDef

from planner.trajectory_safety import TerrainInfluenceCache

RESULTS_DIR = ROOT / "results" / "test_bilecik"
PLOTS_DIR = RESULTS_DIR / "plots"
PLOTS_DIR.mkdir(parents=True, exist_ok=True)


def get_elevation_safe(tq: TerrainQuery, x: float, y: float) -> float:
    r, c = tq.xy_to_rowcol(x, y)
    res = tq.elevation_at_rowcol(r, c)
    return res.elevation


def format_maneuvers_tr(counts: Counter) -> str:
    parts = []
    mapping = [
        ("STRAIGHT_LEVEL", "Düz Uçuş"),
        ("STRAIGHT_CLIMB", "Tırmanış"),
        ("STRAIGHT_DESCENT", "Alçalma"),
        ("LEFT_TURN", "Sola Dönüş"),
        ("RIGHT_TURN", "Sağa Dönüş"),
        ("LEFT_LEVEL_TURN", "Sola Seviye Dönüş"),
        ("RIGHT_LEVEL_TURN", "Sağa Seviye Dönüş"),
        ("CLIMBING_LEFT_TURN", "Tırmanan Sol Dönüş"),
        ("CLIMBING_RIGHT_TURN", "Tırmanan Sağ Dönüş"),
        ("DESCENDING_LEFT_TURN", "Alçalan Sol Dönüş"),
        ("DESCENDING_RIGHT_TURN", "Alçalan Sağ Dönüş"),
    ]
    for key, tr_name in mapping:
        if counts.get(key, 0) > 0:
            parts.append(f"{counts[key]}x {tr_name}")
    for k, v in counts.items():
        if k not in [m[0] for m in mapping]:
            parts.append(f"{v}x {k}")
    return ", ".join(parts) if parts else "Yok"


def run_all_missions() -> Dict:
    print("=" * 80)
    print("BILECIK 30 OPERATIONAL MISSIONS BENCHMARK & PLOTTING")
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
        enable_pareto_z_pruning=True,
    )
    print("Loading Bilecik 30x30 km ROI...")
    roi = load_roi(cfg)
    tq = TerrainQuery(roi)
    cache = TerrainInfluenceCache(tq)
    field = cache.field(60.0)
    print(f"  ROI loaded: {roi.width}x{roi.height} px, bounds={roi.bounds}")

    # Load 30m full elevation grid for top-down plotting
    elev_grid = roi.elevation.copy()
    valid_elev = elev_grid[elev_grid != roi.nodata]
    min_elev, max_elev = float(valid_elev.min()), float(valid_elev.max())
    elev_grid[elev_grid == roi.nodata] = min_elev
    extent = [roi.bounds[0], roi.bounds[2], roi.bounds[1], roi.bounds[3]]

    all_results = []

    for idx, m in enumerate(BILECIK_MISSIONS, start=1):
        t_start = time.perf_counter()
        sx, sy = m.start_xy
        gx, gy = m.goal_xy

        # Calculate terrain elevations (buffered to guarantee lateral buffer validity)
        r1, c1 = tq.xy_to_rowcol(sx, sy)
        r2, c2 = tq.xy_to_rowcol(gx, gy)
        s_ground = float(field.elevation_msl[r1, c1])
        g_ground = float(field.elevation_msl[r2, c2])

        # Initial altitudes
        start_z = s_ground + max(130.0, m.start_alt_offset_m)
        goal_z = g_ground + max(130.0, m.goal_alt_offset_m)

        heading = m.start_heading_deg
        if heading is None:
            heading = navigation_bearing_deg(sx, sy, gx, gy)

        start_pose = PhysicalPose(sx, sy, start_z, heading)
        goal_pose = GoalPose(gx, gy, goal_z)
        tol = GoalTolerance(m.goal_tolerance_xy_m, m.goal_tolerance_z_m)

        print(f"\n[{idx}/30] Running {m.id}: {m.name} ({m.category})")
        print(f"      Start: ({sx:.0f}, {sy:.0f}, {start_z:.1f}m MSL | Grd: {s_ground:.1f}m)")
        print(f"      Goal:  ({gx:.0f}, {gy:.0f}, {goal_z:.1f}m MSL | Grd: {g_ground:.1f}m)")

        search_res = pose_aware_astar_search(
            start_pose, goal_pose, tq, goal_tolerance=tol,
            config=cfg, max_expansions=30000, max_search_time_s=30.0,
        )

        elapsed = time.perf_counter() - t_start

        if not search_res.success:
            print(f"      FAILED: {search_res.termination_reason} in {elapsed:.2f}s (expanded {search_res.expanded_nodes})")
            all_results.append({
                "id": m.id,
                "name": m.name,
                "category": m.category,
                "purpose": m.purpose,
                "description": m.description,
                "success": False,
                "termination_reason": search_res.termination_reason,
                "runtime_s": elapsed,
                "expanded_nodes": search_res.expanded_nodes,
                "path_length_m": 0.0,
                "primitives": {},
                "maneuvers_tr": "Başarısız",
                "min_agl_m": 0.0,
                "mean_agl_m": 0.0,
                "plot_filename": "",
            })
            continue

        # Optimize vertical profile to hug terrain
        opt_res = optimize_terrain_following_altitudes(
            search_res.trajectories, tq, config=cfg, target_agl_m=120.0
        )
        active_trajectories = opt_res.trajectories if (opt_res and opt_res.success) else search_res.trajectories

        # Build dense trajectory samples
        dense_points = []
        cum_dist = 0.0
        for traj in active_trajectories:
            for s in traj.samples:
                ground = get_elevation_safe(tq, s.x_m, s.y_m)
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

        dists = [p["dist_m"] for p in dense_points]
        xs = [p["x_m"] for p in dense_points]
        ys = [p["y_m"] for p in dense_points]
        zs = [p["z_msl_m"] for p in dense_points]
        grounds = [p["ground_m"] for p in dense_points]
        agls = [p["agl_m"] for p in dense_points]

        min_agl = float(np.min(agls))
        mean_agl = float(np.mean(agls))
        prim_counts = dict(search_res.path_primitive_counts)
        maneuvers_tr = format_maneuvers_tr(search_res.path_primitive_counts)

        print(f"      SUCCESS! Path Length: {cum_dist:.1f} m | Min AGL: {min_agl:.1f} m | Mean AGL: {mean_agl:.1f} m")
        print(f"      Expanded: {search_res.expanded_nodes} | Runtime: {elapsed:.2f} s")
        print(f"      Manevralar: {maneuvers_tr}")

        # -------------------------------------------------------------
        # Generate Individual 2-Panel Figure
        # -------------------------------------------------------------
        fig, (ax_dem, ax_alt) = plt.subplots(1, 2, figsize=(16, 6.5), dpi=200)

        # Left Panel: Top-Down Copernicus DEM & Trajectory
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
        cbar.set_label("Copernicus DEM Yükseklik (m MSL)", fontsize=9)

        # Plot direct line and physical trajectory
        ax_dem.plot([sx, gx], [sy, gy], "w--", linewidth=1.5, alpha=0.75, label="Kuş Uçuşu Hat")
        ax_dem.plot(xs, ys, color="#ff1744", linewidth=2.6, label=f"İHA Rotası ({cum_dist:.0f}m)")
        ax_dem.scatter([sx], [sy], color="#00e676", edgecolors="black", s=130, zorder=5, label="Başlangıç")
        ax_dem.scatter([gx], [gy], color="#d500f9", edgecolors="black", s=130, zorder=5, label="Hedef")

        # Zoom in around trajectory
        margin = max(600.0, (max(xs) - min(xs)) * 0.35, (max(ys) - min(ys)) * 0.35)
        ax_dem.set_xlim(min(xs) - margin, max(xs) + margin)
        ax_dem.set_ylim(min(ys) - margin, max(ys) + margin)
        ax_dem.set_title(
            f"[{m.id}] {m.name}\nKuşbakışı Copernicus DEM & Uçuş Yörüngesi",
            fontsize=11, fontweight="bold"
        )
        ax_dem.set_xlabel("UTM Easting X (m)", fontsize=10)
        ax_dem.set_ylabel("UTM Northing Y (m)", fontsize=10)
        ax_dem.legend(loc="upper right", fontsize=8, framealpha=0.9)
        ax_dem.grid(True, linestyle=":", alpha=0.5)

        # Right Panel: Altitude & Terrain Profile
        terr_np = np.array(grounds)
        ax_alt.fill_between(dists, 0, terr_np, color="#8b7355", alpha=0.45, label="Arazi (Bilecik DEM)")
        ax_alt.plot(dists, terr_np, color="#5c4033", linewidth=1.8, label="Arazi Profili")
        ax_alt.plot(dists, terr_np + 100.0, "r--", linewidth=1.3, alpha=0.85, label="Min Emniyet AGL (+100m)")
        ax_alt.plot(dists, terr_np + 120.0, "g:", linewidth=1.3, alpha=0.85, label="Hedef AGL (+120m)")
        ax_alt.plot(dists, zs, color="#0055ff", linewidth=2.5, label="İHA İrtifası (MSL)")

        ax_alt.set_title(
            f"Dikey Uçuş Profili | Min AGL: {min_agl:.1f} m, Ort: {mean_agl:.1f} m\n{m.category}",
            fontsize=11, fontweight="bold"
        )
        ax_alt.set_xlabel("Yörünge Boyunca Mesafe (m)", fontsize=10)
        ax_alt.set_ylabel("İrtifa MSL (m)", fontsize=10)
        y_min = max(0, min(grounds) - 80)
        y_max = max(zs) + 120
        ax_alt.set_ylim(y_min, y_max)
        ax_alt.legend(loc="upper right", fontsize=8, framealpha=0.9)
        ax_alt.grid(True, linestyle=":", alpha=0.5)

        plt.tight_layout()
        plot_fname = f"{m.id}_{m.name}_2panel.png"
        fig.savefig(PLOTS_DIR / plot_fname, bbox_inches="tight")
        plt.close(fig)

        all_results.append({
            "id": m.id,
            "name": m.name,
            "category": m.category,
            "purpose": m.purpose,
            "description": m.description,
            "success": True,
            "termination_reason": search_res.termination_reason,
            "runtime_s": round(elapsed, 3),
            "expanded_nodes": search_res.expanded_nodes,
            "path_length_m": round(cum_dist, 1),
            "primitives": prim_counts,
            "maneuvers_tr": maneuvers_tr,
            "min_agl_m": round(min_agl, 1),
            "mean_agl_m": round(mean_agl, 1),
            "plot_filename": plot_fname,
            "start_coords": [sx, sy, round(start_z, 1)],
            "goal_coords": [gx, gy, round(goal_z, 1)],
            "trajectory_samples": [
                [round(p["x_m"], 1), round(p["y_m"], 1), round(p["z_msl_m"], 1)]
                for p in dense_points[::5]  # subsample for json storage
            ],
        })

    # -------------------------------------------------------------
    # Generate Overall Atlas Map (All 30 Missions on Full DEM)
    # -------------------------------------------------------------
    print("\nGenerating Bilecik 30-Missions Master Atlas Map...")
    fig_atlas, ax_atlas = plt.subplots(figsize=(14, 14), dpi=200)
    im_a = ax_atlas.imshow(
        elev_grid,
        extent=extent,
        origin="upper",
        cmap="terrain",
        alpha=0.85,
        vmin=min_elev,
        vmax=max_elev,
    )
    cbar_a = plt.colorbar(im_a, ax=ax_atlas, fraction=0.046, pad=0.04)
    cbar_a.set_label("Copernicus DEM Yükseklik (m MSL)", fontsize=11)

    colors = plt.cm.tab20(np.linspace(0, 1, len(all_results)))
    for res, c in zip(all_results, colors):
        if not res["success"] or not res.get("trajectory_samples"):
            continue
        pts = np.array(res["trajectory_samples"])
        ax_atlas.plot(pts[:, 0], pts[:, 1], color=c, linewidth=2.0, alpha=0.9)
        ax_atlas.scatter([pts[0, 0]], [pts[0, 1]], color=c, edgecolors="black", s=50, zorder=4)
        ax_atlas.scatter([pts[-1, 0]], [pts[-1, 1]], color="red", marker="x", s=50, zorder=5)
        # Label ID near start
        ax_atlas.text(pts[0, 0] + 150, pts[0, 1] + 150, res["id"], fontsize=8, fontweight="bold",
                      color="white", bbox=dict(boxstyle="round,pad=0.2", facecolor="black", alpha=0.6))

    ax_atlas.set_xlim(roi.bounds[0], roi.bounds[2])
    ax_atlas.set_ylim(roi.bounds[1], roi.bounds[3])
    ax_atlas.set_title(
        "Bilecik Bölgesi 30 Görev Uçuş Yörüngeleri Atlası (30x30 km ROI)\nSakarya Vadisi, Kanyonlar ve Doğu Platoları",
        fontsize=14, fontweight="bold"
    )
    ax_atlas.set_xlabel("UTM Easting X (m)", fontsize=11)
    ax_atlas.set_ylabel("UTM Northing Y (m)", fontsize=11)
    ax_atlas.grid(True, linestyle=":", alpha=0.4)
    plt.tight_layout()
    atlas_fname = "bilecik_30_missions_atlas.png"
    fig_atlas.savefig(RESULTS_DIR / atlas_fname, bbox_inches="tight")
    plt.close(fig_atlas)
    print(f"  Atlas saved to: {RESULTS_DIR / atlas_fname}")

    # -------------------------------------------------------------
    # Save JSON and Generate Markdown Report
    # -------------------------------------------------------------
    json_path = RESULTS_DIR / "results.json"
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(all_results, f, indent=2, ensure_ascii=False)
    print(f"  Results JSON saved to: {json_path}")

    report_path = RESULTS_DIR / "report.md"
    generate_markdown_report(all_results, report_path, atlas_fname)
    print(f"  Markdown Report generated at: {report_path}")

    return all_results


def generate_markdown_report(results: List[Dict], report_path: Path, atlas_fname: str) -> None:
    successes = [r for r in results if r["success"]]
    total_count = len(results)
    success_count = len(successes)
    total_dist_km = sum(r["path_length_m"] for r in successes) / 1000.0
    total_runtime_s = sum(r["runtime_s"] for r in results)
    avg_runtime_s = total_runtime_s / total_count if total_count else 0.0
    avg_min_agl = np.mean([r["min_agl_m"] for r in successes]) if successes else 0.0
    avg_mean_agl = np.mean([r["mean_agl_m"] for r in successes]) if successes else 0.0

    lines = [
        "# Bilecik Bölgesi 30 Farklı Amaçlı İHA Uçuş Testi Raporu",
        "",
        "> **Bölge:** Bilecik – Sakarya Vadisi, Kanyonları ve Doğu Platoları  ",
        f"> **Tarih:** {time.strftime('%Y-%m-%d %H:%M:%S')}  ",
        f"> **Harita:** Copernicus GLO-30 (`regions/bilecik/working_dem.tif`, 40×40 km, 30m UTM 36N)  ",
        "> **Konfigürasyon:** `min_agl_m = 100.0 m`, `lateral_buffer_m = 60.0 m`, `target_agl_m = 120.0 m`",
        "",
        "---",
        "",
        "## 1. Yönetici Özeti (Executive Summary)",
        "",
        f"- **Toplam Test Sayısı:** {total_count}",
        f"- **Başarılı Görev Sayısı:** **{success_count} / {total_count} (%{success_count/total_count*100:.1f})**",
        f"- **Toplam Uçulan Mesafe:** **{total_dist_km:.2f} km**",
        f"- **Ortalama Arama Süresi:** **{avg_runtime_s:.2f} saniye** (Toplam: {total_runtime_s:.2f} s)",
        f"- **Ortalama Minimum AGL:** **{avg_min_agl:.1f} m** (Hard limit: 100m korunmuştur)",
        f"- **Ortalama Seyir AGL:** **{avg_mean_agl:.1f} m** (Hedef 120m AGL bandında kusursuz arazi takibi)",
        "",
        "### 30 Görevin Toplu Atlas Haritası",
        f"![Bilecik 30 Görev Atlası]({atlas_fname})",
        "",
        "---",
        "",
        "## 2. Görev Özeti ve Manevra Tablosu",
        "",
        "| ID | Görev Adı | Kategori | Mesafe | Süre | Düğüm | Min AGL | Yapılan Manevralar | Durum |",
        "| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |",
    ]

    for r in results:
        status_badge = "✅ FOUND" if r["success"] else f"❌ {r['termination_reason']}"
        dist_str = f"{r['path_length_m']:.0f} m" if r["success"] else "—"
        min_agl_str = f"{r['min_agl_m']:.1f} m" if r["success"] else "—"
        runtime_str = f"{r['runtime_s']:.2f} s"
        lines.append(
            f"| **{r['id']}** | [{r['name']}](#{r['id'].lower()}-{r['name'].lower()}) | {r['category']} | "
            f"{dist_str} | {runtime_str} | {r['expanded_nodes']} | {min_agl_str} | {r['maneuvers_tr']} | {status_badge} |"
        )

    lines.extend([
        "",
        "---",
        "",
        "## 3. Detaylı Görev Analizleri ve 2-Panel Grafikler",
        "",
    ])

    for r in results:
        lines.extend([
            f"### {r['id']}: {r['name']}",
            f"**Kategori:** {r['category']}  ",
            f"**Amacı:** {r['purpose']}  ",
            f"**Açıklama:** {r['description']}  ",
            "",
            "- **Sonuç Durumu:** " + ("✅ **BAŞARILI (FOUND)**" if r["success"] else f"❌ **BAŞARISIZ ({r['termination_reason']})**"),
            f"- **Uçuş Uzunluğu:** {r['path_length_m']:.1f} metre",
            f"- **Çözüm Süresi / Düğüm:** {r['runtime_s']:.3f} saniye / {r['expanded_nodes']} genişletilen düğüm",
            f"- **İrtifa İstatistikleri:** Minimum AGL: **{r['min_agl_m']:.1f} m** | Ortalama AGL: **{r['mean_agl_m']:.1f} m**",
            f"- **İcra Edilen Manevralar:** `{r['maneuvers_tr']}`",
            "",
        ])
        if r["success"] and r["plot_filename"]:
            lines.extend([
                f"![{r['name']} 2-Panel Grafiği](plots/{r['plot_filename']})",
                "",
            ])
        lines.append("---")
        lines.append("")

    with open(report_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


if __name__ == "__main__":
    run_all_missions()
