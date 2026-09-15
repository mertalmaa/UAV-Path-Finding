"""Generate high-resolution vertical flight profile plots for all missions (A-F).

Plots along-track distance (m) vs MSL altitude (m) showing:
1. Terrain elevation under the continuous trajectory (saddlebrown)
2. Hard clearance limit: Terrain + 100m AGL (dashed crimson)
3. Soft target clearance: Terrain + 120m AGL (dotted darkgreen)
4. Actual UAV planned 3D trajectory (royalblue / purple)
"""
from __future__ import annotations

import json
import math
import shutil
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from planner.aircraft_profile import load_aircraft_profile
from planner.config import DEFAULT_CONFIG
from planner.fixed_wing_envelope import FixedWingKinematicEnvelope
from planner.physical import PhysicalPose
from planner.pose_search import (
    GoalPose,
    GoalTolerance,
    navigation_bearing_deg,
    pose_aware_astar_search,
)
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from experiments.vertical_bottleneck_mitigation_experiment import (
    VariantConfig,
    run_variant_search,
)
from scripts.benchmark_far_missions import FAR_MISSIONS
from scripts.benchmark_missions import (
    CACHE_DIR,
    CONFIG,
    FACTOR,
    GOAL_TOLERANCE,
    PROFILE_PATH,
    SOURCE_DEM_PATH,
    mission_definitions,
)



def extract_trajectory_vertical_profile(
    result,
    terrain: TerrainQuery,
) -> Dict[str, List[float]]:
    """Sample the continuous solution trajectory at fine resolution along path distance."""
    if not result.success or not result.trajectories:
        return {"distance_m": [], "terrain_msl_m": [], "uav_msl_m": [], "agl_m": []}

    distances = [0.0]
    uav_altitudes = []
    terrain_altitudes = []
    agls = []

    # First point
    first_pose = result.nodes[0].end_pose
    uav_altitudes.append(first_pose.z_msl_m)
    q0 = terrain.query(first_pose.x_m, first_pose.y_m)
    t0 = q0.elevation if q0.valid and math.isfinite(q0.elevation) else 0.0
    terrain_altitudes.append(t0)
    agls.append(first_pose.z_msl_m - t0)

    cum_dist = 0.0
    for traj in result.trajectories:
        samples = traj.samples
        for p1, p2 in zip(samples, samples[1:]):
            ds = math.hypot(p2.x_m - p1.x_m, p2.y_m - p1.y_m)
            cum_dist += ds
            distances.append(cum_dist)
            uav_altitudes.append(p2.z_msl_m)
            q = terrain.query(p2.x_m, p2.y_m)
            te = q.elevation if q.valid and math.isfinite(q.elevation) else 0.0
            terrain_altitudes.append(te)
            agls.append(p2.z_msl_m - te)

    return {
        "distance_m": distances,
        "terrain_msl_m": terrain_altitudes,
        "uav_msl_m": uav_altitudes,
        "agl_m": agls,
    }


def main():
    print("Loading terrain and aircraft profile...")
    cache = load_terrain_cache(CACHE_DIR, load_roi(CONFIG), SOURCE_DEM_PATH)
    terrain = build_terrain_query_from_cache(cache, load_roi(CONFIG), FACTOR)
    profile = load_aircraft_profile(PROFILE_PATH)

    missions_dict = dict(mission_definitions(cache))
    for name, definition in FAR_MISSIONS.items():
        missions_dict[name] = dict(definition)

    missions = {
        "A_easy_open": ("Mission A: Easy Open (1.1 km)", missions_dict["A_easy_open"]),
        "B_relief_affected": ("Mission B: Relief Crossing (1.6 km)", missions_dict["B_relief_affected"]),
        "C_far_south_3km": ("Mission C: Far South (3.0 km)", missions_dict["C_far_south_3km"]),
        "D_far_east_3km": ("Mission D: Far East (3.0 km)", missions_dict["D_far_east_3km"]),
        "E_long_descent_9_3km": ("Mission E: Long Descent (9.3 km, 4400m -> 3900m)", missions_dict["E_long_descent_9_3km"]),
        "F_turn_required_diagonal_2_3km": ("Mission F: Turn Required (2.3 km)", missions_dict["F_turn_required_diagonal_2_3km"]),
    }

    # Variant: Pure Weighted A* (w=1.01) (zero Pareto risk, zero corridor risk)
    var_w101 = VariantConfig(
        name="Weighted_1.01",
        category="Production_Candidate",
        description="Weighted A* w=1.01 (No Pareto, No Corridor)",
        heuristic_weight=1.01,
        z_dominance="none",
        guidance_mode="none",
    )

    profiles_data = {}

    fig, axes = plt.subplots(len(missions), 1, figsize=(13, 3.2 * len(missions)), constrained_layout=True)

    print("\nRunning missions and extracting vertical flight profiles...")
    for idx, (m_key, (title, m_def)) in enumerate(missions.items()):
        ax = axes[idx]
        sx, sy = terrain.rowcol_to_xy(*m_def["start_rc"])
        gx, gy = terrain.rowcol_to_xy(*m_def["goal_rc"])
        h = navigation_bearing_deg(sx, sy, gx, gy)
        start_z = float(m_def["start_z"] if "start_z" in m_def else m_def["z_msl_m"])
        goal_z = float(m_def["goal_z"] if "goal_z" in m_def else m_def.get("goal_z_msl_m", start_z))
        
        start_pose = PhysicalPose(sx, sy, start_z, h)
        goal_pose = GoalPose(gx, gy, goal_z)

        print(f"  Generating path for {m_key}...", end="", flush=True)
        t0 = time.perf_counter()
        res = run_variant_search(start_pose, goal_pose, terrain, profile, var_w101, max_expansions=30000)
        dt = time.perf_counter() - t0
        print(f" done in {dt:.2f}s (Success: {res.success}, Exp: {res.expanded_nodes}, Len: {res.continuous_path_length_m:.1f}m)")

        prof = extract_trajectory_vertical_profile(res, terrain)
        profiles_data[m_key] = prof

        if prof["distance_m"]:
            dists = prof["distance_m"]
            terr = prof["terrain_msl_m"]
            uav_z = prof["uav_msl_m"]
            hard_100 = [t + 100.0 for t in terr]
            soft_120 = [t + 120.0 for t in terr]

            # 1. Terrain area / curve
            ax.plot(dists, terr, color="saddlebrown", linewidth=2.0, label="Terrain")
            ax.fill_between(dists, 0, terr, color="#d2b48c", alpha=0.35)

            # 2. Hard & Soft safety clearance lines
            ax.plot(dists, hard_100, "--", color="crimson", linewidth=1.4, alpha=0.85, label="Hard +100m AGL (Safety Limit)")
            ax.plot(dists, soft_120, ":", color="forestgreen", linewidth=1.4, alpha=0.85, label="Soft +120m AGL (Target Floor)")

            # 3. Actual UAV Trajectory
            ax.plot(dists, uav_z, color="royalblue", linewidth=2.4, label="UAV Flight Path (w=1.01)")

            # Annotations
            min_agl = min(prof["agl_m"])
            ax.set_title(f"{title} | Length: {dists[-1]:.0f}m | Min AGL: {min_agl:.1f}m | Expansions: {res.expanded_nodes}", fontsize=11, fontweight="bold")
            ax.set_ylabel("MSL Altitude (m)", fontsize=10)
            ax.grid(True, linestyle="--", alpha=0.4)
            
            # Set Y limits with padding
            y_min = min(min(terr), min(uav_z)) - 100
            y_max = max(max(soft_120), max(uav_z)) + 120
            ax.set_ylim(max(0, y_min), y_max)
            ax.set_xlim(0, max(dists))

            if idx == 0:
                ax.legend(loc="upper right", ncol=4, fontsize=8, framealpha=0.9)
        else:
            ax.set_title(f"{title} - [FAILED]")

    axes[-1].set_xlabel("Along-Track Distance (m)", fontsize=11, fontweight="bold")
    fig.suptitle("UAV Fixed-Wing 3D Flight Altitude Profiles (A* w=1.01 Continuous Motion)", fontsize=14, fontweight="bold")

    out_png = ROOT / "results" / "mission_vertical_flight_profiles.png"
    fig.savefig(out_png, dpi=180)
    plt.close(fig)
    print(f"\n[OK] Saved vertical flight profile figure to: {out_png}")

    # Copy to artifact dir
    artifact_dir = Path(r"C:\Users\PC_10004_YD26\.gemini\antigravity-ide\brain\703ac52a-e972-41a4-9422-6001171f41b7")
    if artifact_dir.exists():
        dst = artifact_dir / "mission_vertical_flight_profiles.png"
        shutil.copyfile(out_png, dst)
        print(f"[OK] Copied figure to artifact directory: {dst}")


if __name__ == "__main__":
    main()
