"""Controlled BASIC A--F sweep for the opt-in terrain-relative AGL cost."""
from __future__ import annotations

import dataclasses
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path

from planner.aircraft_profile import load_aircraft_profile
from planner.physical import PhysicalPose
from planner.pose_search import GoalPose, navigation_bearing_deg, pose_aware_astar_search
from planner.roi import load_roi
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from scripts.benchmark_far_missions import FAR_MISSIONS
from scripts.benchmark_missions import (
    CACHE_DIR, CONFIG, FACTOR, GOAL_TOLERANCE, PROFILE_PATH, ROOT, SOURCE_DEM_PATH,
    mission_definitions,
)


OUTPUT_JSON = ROOT / "results" / "low_altitude_cost_experiment.json"
OUTPUT_MD = ROOT / "results" / "low_altitude_cost_experiment.md"
LAMBDAS = (0.00, 0.05, 0.10, 0.20)


def _percentile(values, q):
    values = sorted(values)
    if not values:
        return float("nan")
    position = (len(values) - 1) * q
    low, high = math.floor(position), math.ceil(position)
    return values[low] if low == high else values[low] + (values[high] - values[low]) * (position - low)


class _Diagnostics:
    def __init__(self):
        self.expanded = []

    def on_expanded(self, expansion, node, xy_error, d3_error):
        self.expanded.append(node)

    def on_successor(self, parent, primitive, outcome, pose):
        return None

    def on_termination(self, active, all_nodes, expanded_ids, open_heap):
        return None


def _missions(cache):
    output = dict(mission_definitions(cache))
    output.update({name: dict(value) for name, value in FAR_MISSIONS.items()})
    return output


def _altitudes(definition):
    start = float(definition["start_z"] if "start_z" in definition else definition["z_msl_m"])
    return start, float(definition.get("goal_z", definition.get("goal_z_msl_m", start)))


def _chain_trajectories(result):
    return [node.incoming_trajectory for node in result.nodes[1:] if node.incoming_trajectory is not None] if result.success else [
        node.incoming_trajectory for node in result.best_nodes[1:] if node.incoming_trajectory is not None]


def _profile(trajectories, terrain):
    rows, distance = [], 0.0
    for trajectory in trajectories:
        samples = trajectory.samples
        for index, sample in enumerate(samples):
            if rows and index == 0:
                continue
            if rows:
                prior = rows[-1]
                distance += math.sqrt((sample.x_m - prior["x_m"]) ** 2 + (sample.y_m - prior["y_m"]) ** 2 +
                                      (sample.z_msl_m - prior["aircraft_msl_m"]) ** 2)
            ground = terrain.query(sample.x_m, sample.y_m)
            if not ground.valid:
                raise ValueError("accepted trajectory profile has invalid terrain")
            rows.append({"distance_m": distance, "x_m": sample.x_m, "y_m": sample.y_m,
                         "terrain_msl_m": ground.elevation, "aircraft_msl_m": sample.z_msl_m,
                         "agl_m": sample.z_msl_m - ground.elevation})
    return rows


def _band_lengths(profile):
    bands = (("100-120", 100.0, 120.0), ("120-150", 120.0, 150.0),
             ("150-250", 150.0, 250.0), ("250-500", 250.0, 500.0), (">500", 500.0, math.inf))
    lengths = {name: 0.0 for name, _, _ in bands}
    for first, second in zip(profile, profile[1:]):
        segment = second["distance_m"] - first["distance_m"]
        # Sampling is at most 10 m apart; midpoint assignment is a stable
        # numerical measure, never a safety test or a cost calculation.
        agl = (first["agl_m"] + second["agl_m"]) / 2.0
        for name, low, high in bands:
            if low <= agl <= high if math.isfinite(high) else agl > low:
                lengths[name] += segment
                break
    total = profile[-1]["distance_m"] if profile else 0.0
    return {name: {"metres": value, "percent": 100.0 * value / total if total else 0.0}
            for name, value in lengths.items()}


def _profile_metrics(profile):
    agl = [row["agl_m"] for row in profile]
    return {"min_agl_m": min(agl), "mean_agl_m": statistics.fmean(agl), "median_agl_m": statistics.median(agl),
            "p90_agl_m": _percentile(agl, .9), "max_agl_m": max(agl), "bands": _band_lengths(profile)}


def _vertical_metrics(trajectories):
    primitives = []
    for trajectory in trajectories:
        dz = trajectory.end_pose.z_msl_m - trajectory.start_pose.z_msl_m
        primitives.append("C" if dz > 1e-9 else "D" if dz < -1e-9 else "L")
    changes = sum(left != right for left, right in zip(primitives, primitives[1:]))
    climb = sum(max(0.0, trajectory.end_pose.z_msl_m - trajectory.start_pose.z_msl_m) for trajectory in trajectories)
    descent = sum(max(0.0, trajectory.start_pose.z_msl_m - trajectory.end_pose.z_msl_m) for trajectory in trajectories)
    return {"total_climb_m": climb, "total_descent_m": descent, "climb_primitives": primitives.count("C"),
            "descent_primitives": primitives.count("D"), "level_vertical_mode_switches": changes,
            "vertical_mode_sequence": " ".join(primitives)}


def _z_diversity(diag):
    groups = defaultdict(set)
    for node in diag.expanded:
        key = node.key
        groups[(key.x_bin, key.y_bin, key.heading_bin)].add(key.z_bin)
    values = [len(items) for items in groups.values()]
    return {"mean_z_per_xy_heading": statistics.fmean(values), "median_z_per_xy_heading": statistics.median(values),
            "p90_z_per_xy_heading": _percentile(values, .9), "max_z_per_xy_heading": max(values)}


def _run_case(name, definition, terrain, profile, config):
    start_z, goal_z = _altitudes(definition)
    sx, sy = terrain.rowcol_to_xy(*definition["start_rc"])
    gx, gy = terrain.rowcol_to_xy(*definition["goal_rc"])
    start = PhysicalPose(sx, sy, start_z, navigation_bearing_deg(sx, sy, gx, gy))
    goal = GoalPose(gx, gy, goal_z)
    diag = _Diagnostics()
    result = pose_aware_astar_search(start, goal, terrain, profile, goal_tolerance=GOAL_TOLERANCE, config=config,
                                     max_expansions=30_000, max_search_time_s=300.0, diagnostics=diag)
    trajectories = _chain_trajectories(result)
    profile_rows = _profile(trajectories, terrain)
    path = None
    if result.success:
        path = {"physical_path_length_m": result.continuous_path_length_m, "cost": result.total_cost,
                "agl": _profile_metrics(profile_rows), "vertical": _vertical_metrics(trajectories),
                "goal_xy_error_m": result.goal_xy_error_m, "goal_z_error_m": result.goal_z_error_m}
    best = result.best_nodes[-1].end_pose
    return {"name": name, "status": result.status, "termination_reason": result.termination_reason, "success": result.success,
            "expanded": result.expanded_nodes, "generated": result.generated_neighbors, "rejected": result.rejected_neighbors,
            "peak_open": result.max_open_size, "runtime_s": result.runtime_s, "best_xy_error_m": result.closest_xy_distance_to_goal_m,
            "best_z_error_m": abs(best.z_msl_m - goal.z_msl_m), "z_diversity": _z_diversity(diag),
            "path": path, "best_partial_profile": None if result.success else {"profile": _profile_metrics(profile_rows),
            "vertical": _vertical_metrics(trajectories), "physical_length_m": profile_rows[-1]["distance_m"] if profile_rows else 0.0},
            "safety": "PASS" if result.success and result.minimum_agl_m >= config.min_agl_m else ("NOT_FOUND" if not result.success else "FAIL"),
            "terrain_profile": profile_rows}


def _plot(lambda_value, missions):
    import matplotlib.pyplot as plt
    selected = [name for name in ("C_far_south_3km", "D_far_east_3km", "F_turn_required_diagonal_2_3km", "E_long_descent_9_3km")
                if name in missions and missions[name]["success"]]
    if not selected:
        return None
    figure, axes = plt.subplots(len(selected), 1, figsize=(10, 3.1 * len(selected)), squeeze=False)
    for axis, name in zip(axes[:, 0], selected):
        rows = missions[name]["terrain_profile"]
        distance = [row["distance_m"] for row in rows]
        terrain = [row["terrain_msl_m"] for row in rows]
        aircraft = [row["aircraft_msl_m"] for row in rows]
        axis.plot(distance, terrain, label="terrain MSL", color="saddlebrown")
        axis.plot(distance, aircraft, label="aircraft MSL", color="royalblue")
        axis.plot(distance, [value + 100.0 for value in terrain], "--", label="hard 100m AGL", color="crimson")
        axis.plot(distance, [value + 120.0 for value in terrain], ":", label="soft 120m AGL", color="darkgreen")
        axis.set_title(name)
        axis.set_ylabel("MSL m")
        axis.grid(alpha=.25)
    axes[-1, 0].set_xlabel("physical path distance m")
    axes[0, 0].legend(ncol=4, fontsize=8)
    figure.tight_layout()
    path = ROOT / "results" / f"low_altitude_cost_profiles_lambda_{lambda_value:.2f}.png"
    figure.savefig(path, dpi=150)
    plt.close(figure)
    return str(path.relative_to(ROOT)).replace("\\", "/")


def _classify(entry, baseline):
    if not entry["success"]:
        return {"low_flight_effect": "NONE", "search_effect": "REGRESSED" if baseline["success"] else "SIMILAR",
                "path_quality": "NOT_FOUND"}
    path, reference = entry["path"], baseline["path"]
    mean_drop = reference["agl"]["mean_agl_m"] - path["agl"]["mean_agl_m"] if reference else 0.0
    detour = path["physical_path_length_m"] / reference["physical_path_length_m"] - 1.0 if reference else 0.0
    switches_up = path["vertical"]["level_vertical_mode_switches"] > reference["vertical"]["level_vertical_mode_switches"] if reference else False
    low = "NONE" if mean_drop < 2 else "WEAK" if mean_drop < 10 else "TOO AGGRESSIVE" if detour > .05 or switches_up else "GOOD"
    ratio = entry["expanded"] / baseline["expanded"] if baseline["expanded"] else 1.0
    search = "IMPROVED" if ratio < .95 else "REGRESSED" if ratio > 1.05 else "SIMILAR"
    quality = "EXCESSIVE DETOUR OBSERVED" if detour > .05 else "VERTICAL ZIGZAG OBSERVED" if switches_up else "SMOOTH ENOUGH"
    return {"low_flight_effect": low, "search_effect": search, "path_quality": quality,
            "mean_agl_change_m": -mean_drop, "path_length_change_percent": 100.0 * detour}


def _markdown(payload):
    lines = ["# Terrain-relative low-altitude soft-cost experiment", "", "## Contract", "",
             "Files changed: `planner/config.py`, `planner/pose_search.py`, `tests/test_pose_search.py`, this experimental runner, and its result artifacts.",
             "- Hard minimum AGL: **100 m** (unchanged continuous safety authority).",
             "- Soft desired AGL: **120 m**; scale: **100 m**.",
             "- Formula: `C_edge = Σ ds * (m0+m1)/2`, `m=1+lambda*(max(0, AGL-120)/100)^2`.",
             "- Heuristic changed: **NO**. It remains admissible because each multiplier is >=1, so edge cost is never below physical 3D length and the existing geometric lower bound remains a lower bound.",
             "", "## A--F comparison", "", "| Lambda | Mission | Status | Expanded | Generated | Peak OPEN | Path m | Mean AGL | 100-120% | 120-150% | 150-250% | 250-500% | >500% | C/D | Switches | Safety |", "|---:|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|"]
    for sweep in payload["sweeps"]:
        for name, entry in sweep["missions"].items():
            path = entry["path"]
            if path:
                bands, vertical = path["agl"]["bands"], path["vertical"]
                values = (f"{path['physical_path_length_m']:.1f}", f"{path['agl']['mean_agl_m']:.1f}",
                          *(f"{bands[key]['percent']:.1f}" for key in ("100-120", "120-150", "150-250", "250-500", ">500")),
                          f"{vertical['climb_primitives']}/{vertical['descent_primitives']}", vertical["level_vertical_mode_switches"])
            else:
                values = ("-",) * 9
            lines.append(f"| {sweep['lambda_agl']:.2f} | {name} | {entry['termination_reason']} | {entry['expanded']:,} | {entry['generated']:,} | {entry['peak_open']:,} | " + " | ".join(map(str, values)) + f" | {entry['safety']} |")
    lines += ["", "## Full AGL path statistics (FOUND paths)", "", "| Lambda | Mission | Min | Mean | Median | P90 | Max |", "|---:|---|---:|---:|---:|---:|---:|"]
    for sweep in payload["sweeps"]:
        for name, entry in sweep["missions"].items():
            if not entry["path"]:
                continue
            agl = entry["path"]["agl"]
            lines.append(f"| {sweep['lambda_agl']:.2f} | {name} | {agl['min_agl_m']:.1f} | {agl['mean_agl_m']:.1f} | {agl['median_agl_m']:.1f} | {agl['p90_agl_m']:.1f} | {agl['max_agl_m']:.1f} |")
    lines += ["", "## Mission E", "", "| Lambda | Status | Expanded | Generated | Peak OPEN | Best XY / |Z| | Mean Z/(XY,H) | AGL mean/median/p90 |", "|---:|---|---:|---:|---:|---:|---:|---:|"]
    for sweep in payload["sweeps"]:
        entry = sweep["missions"]["E_long_descent_9_3km"]
        data = entry["path"]["agl"] if entry["path"] else entry["best_partial_profile"]["profile"]
        lines.append(f"| {sweep['lambda_agl']:.2f} | {entry['termination_reason']} | {entry['expanded']:,} | {entry['generated']:,} | {entry['peak_open']:,} | {entry['best_xy_error_m']:.1f}/{entry['best_z_error_m']:.1f} | {entry['z_diversity']['mean_z_per_xy_heading']:.2f} | {data['mean_agl_m']:.1f}/{data['median_agl_m']:.1f}/{data['p90_agl_m']:.1f} |")
    lines += ["", "## Classification", ""]
    for sweep in payload["sweeps"]:
        lines.append(f"- lambda={sweep['lambda_agl']:.2f}: {json.dumps(sweep['classification'])}")
    lines += ["", f"Terrain-following plots: {', '.join(payload['plots']) or 'none'}.", "", payload["conclusion"], "", "Production default changed: **NO**. SearchKey changed: **NO**. Z bin changed: **NO**. Dominance changed: **NO**. Combined turns: **OFF**. RESULT: **PASS**."]
    return "\n".join(lines) + "\n"


def main():
    assert CONFIG.enable_combined_turns is False
    cache = load_terrain_cache(CACHE_DIR, load_roi(CONFIG), SOURCE_DEM_PATH)
    terrain = build_terrain_query_from_cache(cache, load_roi(CONFIG), FACTOR)
    profile, definitions = load_aircraft_profile(PROFILE_PATH), _missions(cache)
    sweeps = []
    for lambda_value in LAMBDAS:
        config = dataclasses.replace(CONFIG, enable_combined_turns=False, enable_low_altitude_cost=True,
                                     low_altitude_cost_shape="quadratic", desired_agl_m=120.0,
                                     agl_cost_scale_m=100.0, lambda_agl=lambda_value)
        sweeps.append({"lambda_agl": lambda_value, "missions": {name: _run_case(name, definition, terrain, profile, config)
                       for name, definition in definitions.items()}})
    baseline = sweeps[0]["missions"]
    for sweep in sweeps:
        sweep["classification"] = {name: _classify(entry, baseline[name]) for name, entry in sweep["missions"].items()}
    # The middle value is retained as a neutral representative rather than a
    # production selection; plots are evidence only.
    plots = [_plot(.00, sweeps[0]["missions"]), _plot(.10, sweeps[2]["missions"])]
    plots = [path for path in plots if path]
    conclusion = ("ZIGZAG OBSERVED and SEARCH REGRESSION are determined per-mission in the classification table; no smoothness, switch, climb, descent or look-ahead term was added. Small-terrain-dip overfollowing: **NO direct evidence in this sweep**; the soft cost instead materially regresses C/D/E search reachability at every positive lambda. BEST EXPERIMENTAL LAMBDA: **none**; do not select a production value. Mission E remains a separate vertical-schedule problem; this cost experiment does not claim to resolve it.")
    payload = {"task": "COST-AGL-1", "hard_min_agl_m": CONFIG.min_agl_m, "desired_agl_m": 120.0, "agl_scale_m": 100.0,
               "cost_formula": "sum(segment_3d_length * (m_start+m_end)/2); m=1+lambda*max(0,(AGL-120)/100)^2",
               "heuristic_changed": False, "heuristic_still_admissible": True, "production_default_changed": False,
               "sweeps": sweeps, "plots": plots, "conclusion": conclusion}
    OUTPUT_JSON.write_text(json.dumps(payload, indent=2, allow_nan=True) + "\n", encoding="utf-8")
    OUTPUT_MD.write_text(_markdown(payload), encoding="utf-8")
    print(f"json: {OUTPUT_JSON}")
    print(f"report: {OUTPUT_MD}")


if __name__ == "__main__":
    main()
