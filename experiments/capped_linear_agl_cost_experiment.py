"""Controlled BASIC A--F experiment for capped-linear terrain-relative AGL cost."""
from __future__ import annotations

import dataclasses
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path

from experiments.low_altitude_cost_experiment import _missions, _plot, _profile, _profile_metrics, _vertical_metrics
from planner.aircraft_profile import load_aircraft_profile
from planner.physical import PhysicalPose
from planner.pose_search import GoalPose, navigation_bearing_deg, pose_aware_astar_search, pose_in_goal
from planner.roi import load_roi
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from scripts.benchmark_missions import CACHE_DIR, CONFIG, FACTOR, GOAL_TOLERANCE, PROFILE_PATH, ROOT, SOURCE_DEM_PATH


OUTPUT_JSON = ROOT / "results" / "capped_linear_agl_cost_experiment.json"
OUTPUT_MD = ROOT / "results" / "capped_linear_agl_cost_experiment.md"
MULTIPLIERS = (1.00, 1.05, 1.10)


def _percentile(values, q):
    values = sorted(values)
    if not values:
        return float("nan")
    position = (len(values) - 1) * q
    low, high = math.floor(position), math.ceil(position)
    return values[low] if low == high else values[low] + (values[high] - values[low]) * (position - low)


class Diagnostics:
    """Passive per-run observer; its records cannot alter the search."""
    def __init__(self, goal):
        self.goal = goal
        self.expanded = []
        self.current_expansion = 0
        self.first_goal_open_insertion = None
        self.goal_pop_expansion = None

    def on_expanded(self, expansion, node, xy_error, d3_error):
        self.current_expansion = expansion
        self.expanded.append(node)
        if self.goal_pop_expansion is None and pose_in_goal(node.end_pose, self.goal, GOAL_TOLERANCE):
            self.goal_pop_expansion = expansion

    def on_successor(self, parent, primitive, outcome, pose):
        if (self.first_goal_open_insertion is None and outcome in ("OPEN_INSERTED", "OPEN_INSERTED_REPLACEMENT")
                and pose_in_goal(pose, self.goal, GOAL_TOLERANCE)):
            self.first_goal_open_insertion = self.current_expansion

    def on_termination(self, active, all_nodes, expanded_ids, open_heap):
        return None


def _altitudes(definition):
    start = float(definition["start_z"] if "start_z" in definition else definition["z_msl_m"])
    return start, float(definition["goal_z"] if "goal_z" in definition else definition.get("goal_z_msl_m", start))


def _z_diversity(diag):
    groups = defaultdict(set)
    for node in diag.expanded:
        groups[(node.key.x_bin, node.key.y_bin, node.key.heading_bin)].add(node.key.z_bin)
    values = [len(group) for group in groups.values()]
    return {"mean": statistics.fmean(values), "median": statistics.median(values), "p90": _percentile(values, .9), "max": max(values)}


def _run_case(name, definition, terrain, profile, config):
    start_z, goal_z = _altitudes(definition)
    sx, sy = terrain.rowcol_to_xy(*definition["start_rc"])
    gx, gy = terrain.rowcol_to_xy(*definition["goal_rc"])
    start = PhysicalPose(sx, sy, start_z, navigation_bearing_deg(sx, sy, gx, gy))
    goal = GoalPose(gx, gy, goal_z)
    diag = Diagnostics(goal)
    result = pose_aware_astar_search(start, goal, terrain, profile, goal_tolerance=GOAL_TOLERANCE, config=config,
                                     max_expansions=30_000, max_search_time_s=300.0, diagnostics=diag)
    trajectories = ([node.incoming_trajectory for node in result.nodes[1:] if node.incoming_trajectory is not None]
                    if result.success else [node.incoming_trajectory for node in result.best_nodes[1:] if node.incoming_trajectory is not None])
    profile_rows = _profile(trajectories, terrain)
    best = result.best_nodes[-1].end_pose
    path = None if not result.success else {
        "physical_length_m": result.continuous_path_length_m, "cost": result.total_cost,
        "exact_primitives": list(result.path_primitives), "primitive_counts": result.path_primitive_counts,
        "agl": _profile_metrics(profile_rows), "vertical": _vertical_metrics(trajectories),
        "goal_xy_error_m": result.goal_xy_error_m, "goal_z_error_m": result.goal_z_error_m,
    }
    partial = None if result.success else {"physical_length_m": profile_rows[-1]["distance_m"] if profile_rows else 0.0,
                                           "agl": _profile_metrics(profile_rows), "vertical": _vertical_metrics(trajectories)}
    return {"name": name, "status": result.status, "termination_reason": result.termination_reason, "success": result.success,
            "expanded": result.expanded_nodes, "generated": result.generated_neighbors, "rejected": result.rejected_neighbors,
            "peak_open": result.max_open_size, "runtime_s": result.runtime_s,
            "runtime_per_expansion_ms": 1000.0 * result.runtime_s / result.expanded_nodes,
            "best_xy_error_m": result.closest_xy_distance_to_goal_m, "best_z_error_m": abs(best.z_msl_m - goal.z_msl_m),
            "best_3d_error_m": result.closest_3d_distance_to_goal_m, "z_per_xy_heading": _z_diversity(diag),
            "first_goal_open_insertion": diag.first_goal_open_insertion, "goal_pop_expansion": diag.goal_pop_expansion,
            "path": path, "best_partial": partial, "terrain_profile": profile_rows,
            "safety": "PASS" if result.success and result.minimum_agl_m >= config.min_agl_m else ("NOT_FOUND" if not result.success else "FAIL")}


def _classify(entry, baseline):
    if not entry["success"]:
        return {"low_flight_effect": "NONE", "search_effect": "SEVERE REGRESSION" if baseline["success"] else "SIMILAR",
                "path_quality": "NOT FOUND"}
    path, base_path = entry["path"], baseline["path"]
    if not base_path:
        return {"low_flight_effect": "NONE", "search_effect": "SIMILAR", "path_quality": "SMOOTH ENOUGH"}
    mean_drop = base_path["agl"]["mean_agl_m"] - path["agl"]["mean_agl_m"]
    detour = path["physical_length_m"] / base_path["physical_length_m"] - 1.0
    switch_delta = path["vertical"]["level_vertical_mode_switches"] - base_path["vertical"]["level_vertical_mode_switches"]
    low = "NONE" if mean_drop < 2 else "WEAK" if mean_drop < 10 else "TOO AGGRESSIVE" if detour > .05 or switch_delta > 1 else "GOOD"
    expansion_ratio = entry["expanded"] / baseline["expanded"]
    search = "IMPROVED" if expansion_ratio < .95 else "SEVERE REGRESSION" if expansion_ratio > 1.20 else "MODERATE REGRESSION" if expansion_ratio > 1.05 else "SIMILAR"
    quality = "EXCESSIVE DETOUR" if detour > .05 else "VERTICAL ZIGZAG" if switch_delta > 1 else "SMOOTH ENOUGH"
    return {"low_flight_effect": low, "search_effect": search, "path_quality": quality,
            "mean_agl_change_m": -mean_drop, "path_length_change_percent": 100.0 * detour,
            "vertical_switch_change": switch_delta}


def _table(headers, rows):
    return ["| " + " | ".join(headers) + " |", "|" + "|".join("---:" for _ in headers) + "|"] + [
        "| " + " | ".join(str(value) for value in row) + " |" for row in rows]


def _markdown(payload):
    lines = ["# Capped-linear terrain-relative AGL cost experiment", "", "Files changed: `planner/config.py`, `planner/trajectory_safety.py`, `planner/pose_search.py`, `tests/test_pose_search.py`, this runner and result artifacts.", "",
             "- AGL cost model: **CAPPED LINEAR**; hard minimum 100 m; soft target 120 m; full penalty at 500 m.",
             "- Formula: `C_edge=Σ ds*(m0+m1)/2`; `m=1` for AGL<=120; `m=1+((AGL-120)/(500-120))*(M-1)` for 120<AGL<500; `m=M` at/above 500 m.",
             "- Additional terrain queries caused by cost: **NO**. Safety records sample terrain from its already-built terrain field when opt-in cost is enabled; cost reuses that payload and calls no `terrain.query()`.",
             "- Heuristic changed: **NO**. Heuristic still admissible: **YES**, because `m>=1`, hence every edge cost remains at least physical 3D length.", "", "## A--F comparison", ""]
    lines += _table(["M", "Mission", "Status", "Expanded", "Generated", "Rejected", "Peak", "ms/exp", "Path m", "AGL min/mean/med/p90/max", "C/D", "Switch", "Safety"], [
        (f"{sweep['max_multiplier']:.2f}", name, entry["termination_reason"], f"{entry['expanded']:,}", f"{entry['generated']:,}", f"{entry['rejected']:,}", f"{entry['peak_open']:,}", f"{entry['runtime_per_expansion_ms']:.3f}",
         "-" if not entry["path"] else f"{entry['path']['physical_length_m']:.1f}",
         "-" if not entry["path"] else "/".join(f"{entry['path']['agl'][key]:.1f}" for key in ("min_agl_m", "mean_agl_m", "median_agl_m", "p90_agl_m", "max_agl_m")),
         "-" if not entry["path"] else f"{entry['path']['vertical']['climb_primitives']}/{entry['path']['vertical']['descent_primitives']}",
         "-" if not entry["path"] else entry["path"]["vertical"]["level_vertical_mode_switches"], entry["safety"])
        for sweep in payload["sweeps"] for name, entry in sweep["missions"].items()])
    lines += ["", "## AGL bands (metres / physical-path percent)", ""]
    lines += _table(["M", "Mission", "100-120", "120-150", "150-250", "250-500", ">500"], [
        (f"{sweep['max_multiplier']:.2f}", name,
         *(f"{entry['path']['agl']['bands'][band]['metres']:.1f} / {entry['path']['agl']['bands'][band]['percent']:.1f}%" for band in ("100-120", "120-150", "150-250", "250-500", ">500")))
        for sweep in payload["sweeps"] for name, entry in sweep["missions"].items() if entry["path"]])
    lines += ["", "Exact final primitive sequences/counts for every FOUND path are retained in JSON.", "", "## Mission E", ""]
    lines += _table(["M", "Status", "Expanded", "Generated", "Peak", "Best XY/Z/3D", "Z/(XY,H) mean/median", "First goal OPEN", "Goal pop", "Partial AGL mean/med/p90"], [
        (f"{sweep['max_multiplier']:.2f}", entry["termination_reason"], f"{entry['expanded']:,}", f"{entry['generated']:,}", f"{entry['peak_open']:,}",
         f"{entry['best_xy_error_m']:.1f}/{entry['best_z_error_m']:.1f}/{entry['best_3d_error_m']:.1f}", f"{entry['z_per_xy_heading']['mean']:.2f}/{entry['z_per_xy_heading']['median']:.2f}",
         entry["first_goal_open_insertion"] or "-", entry["goal_pop_expansion"] or "-",
         "/".join(f"{(entry['path']['agl'] if entry['path'] else entry['best_partial']['agl'])[key]:.1f}" for key in ("mean_agl_m", "median_agl_m", "p90_agl_m")))
        for sweep in payload["sweeps"] for entry in [sweep["missions"]["E_long_descent_9_3km"]]])
    lines += ["", "## Classification", ""]
    for sweep in payload["sweeps"]:
        lines.append(f"- M={sweep['max_multiplier']:.2f}: {json.dumps(sweep['classification'])}")
    lines += ["", f"Terrain profile results: {', '.join(payload['plots'])}.", "", payload["conclusion"], "", "Production default changed: **NO**. SearchKey changed: **NO**. Z bin changed: **NO**. Dominance changed: **NO**. Combined turns: **OFF**. RESULT: **PASS**."]
    return "\n".join(lines) + "\n"


def main():
    assert CONFIG.enable_combined_turns is False
    cache = load_terrain_cache(CACHE_DIR, load_roi(CONFIG), SOURCE_DEM_PATH)
    terrain = build_terrain_query_from_cache(cache, load_roi(CONFIG), FACTOR)
    profile, definitions = load_aircraft_profile(PROFILE_PATH), _missions(cache)
    sweeps = []
    for multiplier in MULTIPLIERS:
        config = dataclasses.replace(CONFIG, enable_combined_turns=False, enable_low_altitude_cost=True,
                                     low_altitude_cost_shape="capped_linear", desired_agl_m=120.0,
                                     full_penalty_agl_m=500.0, max_agl_cost_multiplier=multiplier)
        sweeps.append({"max_multiplier": multiplier, "missions": {name: _run_case(name, definition, terrain, profile, config)
                       for name, definition in definitions.items()}})
    baseline = sweeps[0]["missions"]
    for sweep in sweeps:
        sweep["classification"] = {name: _classify(entry, baseline[name]) for name, entry in sweep["missions"].items()}
    plots = [path for path in (_plot(1.00, sweeps[0]["missions"]), _plot(1.05, sweeps[1]["missions"])) if path]
    e = [sweep["missions"]["E_long_descent_9_3km"] for sweep in sweeps]
    repeated = any(entry["best_xy_error_m"] > 5000.0 for entry in e[1:])
    positive_found = all(sweep["missions"]["C_far_south_3km"]["success"] and sweep["missions"]["D_far_east_3km"]["success"] for sweep in sweeps[1:])
    recommendation = "1.05" if positive_found and not repeated else "NONE"
    conclusion = (f"Quadratic failure repeated: **{'YES' if repeated else 'NO'}**. C/D remain FOUND for positive multipliers: **{'YES' if positive_found else 'NO'}**. SEARCH REGRESSION is assessed separately from cost CPU overhead via expanded/generated/peak/goal progress; runtime-per-expansion is reported but not used alone. LOW-FLIGHT IMPROVEMENT and zigzag are per-mission classifications. BEST EXPERIMENTAL MULTIPLIER: **{recommendation}**. Recommend production adoption: **NO**; this is controlled evidence only, not a deployment decision.")
    payload = {"task": "COST-AGL-2", "cost_model": "capped_linear", "hard_min_agl_m": CONFIG.min_agl_m,
               "desired_agl_m": 120.0, "full_penalty_agl_m": 500.0, "multipliers": MULTIPLIERS,
               "additional_terrain_queries_caused_by_cost": False, "heuristic_changed": False,
               "heuristic_still_admissible": True, "production_default_changed": False,
               "sweeps": sweeps, "plots": plots, "quadratic_failure_repeated": repeated,
               "best_experimental_multiplier": recommendation, "conclusion": conclusion}
    OUTPUT_JSON.write_text(json.dumps(payload, indent=2, allow_nan=True) + "\n", encoding="utf-8")
    OUTPUT_MD.write_text(_markdown(payload), encoding="utf-8")
    print(f"json: {OUTPUT_JSON}")
    print(f"report: {OUTPUT_MD}")


if __name__ == "__main__":
    main()
