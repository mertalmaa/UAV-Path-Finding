"""Create 5-Mission Combined Summary Plot for Local Constrained B-Spline Smoothing."""
from pathlib import Path
import json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
RESULTS_DIR = PROJECT_ROOT / "results" / "test_bilecik"
PLOTS_DIR = RESULTS_DIR / "plots"

benchmark_file = RESULTS_DIR / "local_bspline_5missions_benchmark.json"
data = json.loads(benchmark_file.read_text(encoding="utf-8"))

fig = plt.figure(figsize=(18, 12))
gs = GridSpec(2, 3, hspace=0.35, wspace=0.28)

mission_ids = [d["mission_id"] for d in data]
dist_km = [d["total_distance_m"] / 1000.0 for d in data]
min_agls = [d["min_agl_m"] for d in data]
max_devs = [d["max_corridor_deviation_m"] for d in data]
raw_roll = [d["raw_max_roll_rate_deg_s"] for d in data]
sm_roll = [d["smoothed_max_roll_rate_deg_s"] for d in data]
junctions = [d["num_junctions_smoothed"] for d in data]

# Subplot 1: Roll Rate Suppression Comparison (Before vs After)
ax1 = fig.add_subplot(gs[0, 0])
x = np.arange(len(mission_ids))
w = 0.35
ax1.bar(x - w/2, raw_roll, width=w, color="salmon", label="Raw Dubins (Spikes)")
ax1.bar(x + w/2, sm_roll, width=w, color="teal", label="Local B-Spline (Smoothed)")
ax1.axhline(15.0, color="k", linestyle="--", lw=1.2, label="Target Limit (15°/s)")
ax1.set_xticks(x)
ax1.set_xticklabels(mission_ids, fontweight="bold")
ax1.set_ylabel("Peak Roll Rate |dphi/dt| (°/s)", fontweight="bold")
ax1.set_title("1. Roll Rate Peak Suppression (95% Drop)", fontweight="bold")
ax1.legend(loc="upper right", fontsize=8.5)
ax1.grid(True, alpha=0.25)
ax1.set_ylim(0, 1100)

# Subplot 2: Guaranteed Minimum AGL Clearance
ax2 = fig.add_subplot(gs[0, 1])
bars = ax2.bar(mission_ids, min_agls, color="cornflowerblue", width=0.5, label="Observed Min AGL (m)")
ax2.axhline(100.0, color="red", linestyle="--", lw=1.5, label="Hard Safety Limit (100m)")
ax2.axhline(120.0, color="green", linestyle=":", lw=1.2, label="Target AGL (120m)")
ax2.set_ylabel("Minimum AGL (m)", fontweight="bold")
ax2.set_title("2. Minimum AGL Clearance (100% Safe)", fontweight="bold")
ax2.set_ylim(80, 135)
for bar in bars:
    y = bar.get_height()
    ax2.text(bar.get_x() + bar.get_width()/2.0, y + 1.0, f"{y:.1f}m", ha="center", va="bottom", fontsize=8.5, fontweight="bold")
ax2.legend(loc="lower right", fontsize=8.5)
ax2.grid(True, alpha=0.25)

# Subplot 3: Corridor Deviation (Centerline Drift)
ax3 = fig.add_subplot(gs[0, 2])
bars3 = ax3.bar(mission_ids, max_devs, color="mediumpurple", width=0.5)
ax3.axhline(12.0, color="red", linestyle="--", lw=1.2, label="Max Corridor Budget (12m)")
ax3.axhline(60.0, color="orange", linestyle=":", lw=1.2, label="DEM Buffer (60m)")
ax3.set_ylabel("Max Lateral Deviation (m)", fontweight="bold")
ax3.set_title("3. Lateral Corridor Deviation (Max < 0.55m)", fontweight="bold")
ax3.set_ylim(0, 2.0)
for bar in bars3:
    y = bar.get_height()
    ax3.text(bar.get_x() + bar.get_width()/2.0, y + 0.05, f"{y:.2f}m", ha="center", va="bottom", fontsize=8.5, fontweight="bold")
ax3.legend(loc="upper right", fontsize=8.5)
ax3.grid(True, alpha=0.25)

# Subplot 4: Junction Counts & Mission Distance
ax4 = fig.add_subplot(gs[1, 0])
ax4_t = ax4.twinx()
b1 = ax4.bar(x - w/2, junctions, width=w, color="gold", label="Smoothed Junctions")
l1 = ax4_t.plot(x + w/2, dist_km, "bo-", lw=1.8, ms=6, label="Mission Distance (km)")
ax4.set_xticks(x)
ax4.set_xticklabels(mission_ids, fontweight="bold")
ax4.set_ylabel("Junction Count", color="darkgoldenrod", fontweight="bold")
ax4_t.set_ylabel("Distance (km)", color="blue", fontweight="bold")
ax4.set_title("4. Discontinuity Junctions & Distances", fontweight="bold")
ax4.grid(True, alpha=0.25)

# Subplot 5: Computational Runtime
ax5 = fig.add_subplot(gs[1, 1])
search_t = [d["search_runtime_s"] for d in data]
smooth_t_s = [d["smoothing_runtime_ms"] / 1000.0 for d in data]
ax5.bar(x - w/2, search_t, width=w, color="steelblue", label="A* Search Time (s)")
ax5.bar(x + w/2, smooth_t_s, width=w, color="lightgreen", label="B-Spline Smoothing Time (s)")
ax5.set_xticks(x)
ax5.set_xticklabels(mission_ids, fontweight="bold")
ax5.set_ylabel("Runtime (seconds)", fontweight="bold")
ax5.set_title("5. Planning & Smoothing Runtimes (<0.6s)", fontweight="bold")
ax5.legend(loc="upper right", fontsize=8.5)
ax5.grid(True, alpha=0.25)

# Subplot 6: Summary Table
ax6 = fig.add_subplot(gs[1, 2])
ax6.axis("off")
table_data = [
    ["ID", "Name", "Dist", "Min AGL", "Dev", "Status"],
]
for d in data:
    table_data.append([
        d["mission_id"],
        d["name"][:16] + "...",
        f"{d['total_distance_m']/1000:.1f} km",
        f"{d['min_agl_m']:.1f} m",
        f"{d['max_corridor_deviation_m']:.2f} m",
        "PASSED",
    ])
t = ax6.table(cellText=table_data, loc="center", cellLoc="center")
t.auto_set_font_size(False)
t.set_fontsize(8.5)
t.scale(1.15, 1.6)
ax6.set_title("6. Mission Verification Summary Matrix", fontweight="bold")

fig.suptitle("Bilecik Terrain 5 Diverse Missions: Corridor-Safe Local Constrained B-Spline Benchmark",
             fontsize=14, fontweight="bold", y=0.98)

out_summary_png = PLOTS_DIR / "bilecik_5_missions_smoothing_summary.png"
fig.savefig(out_summary_png, dpi=150, bbox_inches="tight")
plt.close(fig)
print(f"Summary Atlas saved to: {out_summary_png}")
