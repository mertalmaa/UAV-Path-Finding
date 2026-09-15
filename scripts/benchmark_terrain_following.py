"""Reproducible altitude-stage comparison on explicit continuous ground tracks.

These are profile benchmarks, not A* expansion/runtime claims. Real DEM cases
are optional. Every plotted candidate comes from the returned validated path.
Run: python -B -m scripts.benchmark_terrain_following --real-terrain
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np
from affine import Affine

from planner.config import DEFAULT_CONFIG
from planner.fixed_wing_envelope import FixedWingKinematicEnvelope, MAX_CLIMB_RATE_MPS, MAX_DESCENT_RATE_MPS
from planner.physical import PhysicalPose, build_straight_level_trajectory, build_straight_vertical_trajectory
from planner.pose_search import navigation_bearing_deg
from planner.roi import ROIData, load_roi
from planner.terrain import TerrainQuery
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from planner.terrain_following import optimize_terrain_following_altitudes
from planner.trajectory_safety import evaluate_physical_trajectory_safety
from scripts.benchmark_missions import CACHE_DIR, FACTOR, SOURCE_DEM_PATH


def synthetic_terrain(ridge: bool) -> TerrainQuery:
    height, width, cell = 30, 330, 30.0
    elevation = np.full((height, width), 1000.0, dtype=np.float32)
    if ridge:
        x = (np.arange(width) + 0.5) * cell
        elevation += (160 * np.exp(-((x - 6000) / 210) ** 2))[None, :]
    return TerrainQuery(ROIData(
        elevation=elevation, transform=Affine(cell, 0, 0, 0, -cell, height * cell),
        crs="EPSG:32636", width=width, height=height,
        bounds=(0, 0, width * cell, height * cell), resolution=(cell, cell), nodata=-9999.0,
    ))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--real-terrain", action="store_true")
    args = parser.parse_args()
    envelope = FixedWingKinematicEnvelope()
    config = dataclasses.replace(DEFAULT_CONFIG, min_agl_m=100.0, lateral_buffer_m=60.0)
    cases = []
    for ridge, name in ((False, "Flat ground / fixed high endpoints"), (True, "Ridge / early climb")):
        terrain = synthetic_terrain(ridge)
        trajectory = build_straight_level_trajectory(PhysicalPose(300, 450, 1300, 90), 9000, 10).trajectory
        cases.append((name, terrain, (trajectory,)))
    if args.real_terrain:
        roi = load_roi(DEFAULT_CONFIG)
        cache = load_terrain_cache(CACHE_DIR, roi, SOURCE_DEM_PATH)
        terrain = build_terrain_query_from_cache(cache, roi, FACTOR)
        for name, first, last, start_z, end_z in (
            ("DEM south / 3 km", (73, 73), (123, 73), 3800.0, 3800.0),
            ("DEM east / 9.3 km descent", (80, 5), (80, 160), 4400.0, 3900.0),
        ):
            sx, sy = terrain.rowcol_to_xy(*first)
            gx, gy = terrain.rowcol_to_xy(*last)
            length = math.hypot(gx - sx, gy - sy)
            start = PhysicalPose(sx, sy, start_z, navigation_bearing_deg(sx, sy, gx, gy))
            if start_z == end_z:
                track = (build_straight_level_trajectory(start, length, 10).trajectory,)
            else:
                descend_length = (start_z - end_z) / MAX_DESCENT_RATE_MPS * 40.0
                level = build_straight_level_trajectory(start, length - descend_length, 10).trajectory
                descend = build_straight_vertical_trajectory(level.end_pose, descend_length, -MAX_DESCENT_RATE_MPS, 10).trajectory
                track = (level, descend)
            cases.append((name, terrain, track))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(len(cases), 1, figsize=(11, 3.3 * len(cases)), constrained_layout=True)
    metrics = []
    for axis, (name, terrain, original) in zip(axes, cases):
        result = optimize_terrain_following_altitudes(original, terrain, envelope=envelope, config=config, target_agl_m=120)
        input_safe = all(evaluate_physical_trajectory_safety(
            tr, terrain, config.min_agl_m, config.primitive_sample_spacing_m,
            lateral_buffer_m=config.lateral_buffer_m).is_safe for tr in original)
        payload = {"case": name, "input_kind": "explicit_ground_track_not_search_output",
                   "profile_status": result.status, "profile_runtime_s": result.runtime_s,
                   "input_terrain_safe": input_safe, "refinement_passes": result.refinement_passes,
                   "hard_agl_m": config.min_agl_m, "target_agl_m": 120, "lateral_buffer_m": 60}
        baseline = [s for tr in original for s in tr.samples]
        distances = []
        offset = 0.0
        for tr in original:
            distances.extend(offset + s.horizontal_distance_along_path_m for s in tr.samples)
            offset += tr.horizontal_arc_length_m
        ground = np.array([terrain.query(s.x_m, s.y_m).elevation for s in baseline])
        x = np.array(distances) / 1000
        axis.plot(x, ground, color="#7c5938", label="Centreline terrain")
        axis.plot(x, ground + 100, color="#bb8b58", linestyle=":", label="Centreline + 100 m")
        axis.plot(x, [s.z_msl_m for s in baseline], color="#555555", linestyle="--", label="Input altitude")
        if result.success:
            samples = [s for tr in result.trajectories for s in tr.samples]
            z = np.array([s.z_msl_m for s in samples])
            rates = [(b.z_msl_m - a.z_msl_m) * 40 / (b.horizontal_distance_along_path_m - a.horizontal_distance_along_path_m)
                     for tr in result.trajectories for a, b in zip(tr.samples, tr.samples[1:])]
            weights = np.diff(np.array(distances))
            # Length-weighted trapezoidal AGL; duplicate boundary stations have
            # zero weight, so primitive subdivision cannot bias the metric.
            def mean_agl(altitude):
                agl = np.asarray(altitude) - ground
                return float(np.sum((agl[:-1] + agl[1:]) * .5 * weights) / np.sum(weights))
            payload.update(minimum_buffered_agl_m=result.minimum_agl_m,
                           input_mean_centreline_agl_m=mean_agl([s.z_msl_m for s in baseline]),
                           output_mean_centreline_agl_m=mean_agl(z),
                           minimum_altitude_msl_m=float(np.min(z)),
                           max_climb_mps=max(0.0, max(rates)), max_descent_mps=max(0.0, -min(rates)),
                           endpoints_preserved=(result.trajectories[0].start_pose == original[0].start_pose
                                                and result.trajectories[-1].end_pose == original[-1].end_pose))
            axis.plot(x, z, color="#007e80", linewidth=2, label="Validated terrain-following altitude")
        axis.set(title=f"{name} — {result.status}", xlabel="Ground-track distance (km)", ylabel="Altitude MSL (m)")
        axis.grid(alpha=.2)
        axis.legend(loc="best", fontsize=8)
        metrics.append(payload)
        print(json.dumps(payload), flush=True)
    out = Path("results")
    out.mkdir(exist_ok=True)
    (out / "terrain_following_profile_benchmark.json").write_text(json.dumps(metrics, indent=2, allow_nan=False), encoding="utf-8")
    fig.savefig(out / "terrain_following_profile_benchmark.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
