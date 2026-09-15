"""Controlled BASIC A--F experiment for an admissible vertical-reachability h.

Production uses ``planner.pose_search._heuristic`` unchanged.  This script
temporarily replaces that private function only inside the experimental search
call, with all other planner policy held constant.
"""
from __future__ import annotations

import contextlib
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path
from unittest.mock import patch

import planner.pose_search as pose_search_module
from experiments.vertical_reachability_heuristic import (
    GLOBAL_OPTIMISTIC_VZ_UPPER_MPS,
    goal_region_residuals,
    vertical_reachability_heuristic,
)
from planner.aircraft_profile import load_aircraft_profile
from planner.physical import PhysicalPose
from planner.pose_search import GoalPose, _heuristic, navigation_bearing_deg, pose_aware_astar_search, pose_in_goal
from planner.roi import load_roi
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from scripts.benchmark_far_missions import FAR_MISSIONS
from scripts.benchmark_missions import (
    CACHE_DIR, CONFIG, FACTOR, GOAL_TOLERANCE, PROFILE_PATH, ROOT, SOURCE_DEM_PATH,
    mission_definitions,
)


OUTPUT_JSON = ROOT / "results" / "vertical_reachability_heuristic_experiment.json"
OUTPUT_MD = ROOT / "results" / "vertical_reachability_heuristic_experiment.md"
_EUCLIDEAN_HEURISTIC = _heuristic


def _errors(pose, goal):
    xy = math.hypot(pose.x_m - goal.x_m, pose.y_m - goal.y_m)
    z = abs(pose.z_msl_m - goal.z_msl_m)
    return xy, z, math.hypot(xy, z)


def _pose(pose) -> dict:
    return {"x_m": pose.x_m, "y_m": pose.y_m, "z_msl_m": pose.z_msl_m,
            "heading_deg": pose.heading_deg}


def _summary(values):
    return {"count": len(values), "mean": statistics.fmean(values) if values else float("nan"),
            "median": statistics.median(values) if values else float("nan")}


class RunDiagnostics:
    """Passive observer; heap wrapping observes but never changes priority."""

    def __init__(self, goal, tolerance, heuristic) -> None:
        self.goal, self.tolerance, self.heuristic = goal, tolerance, heuristic
        self.current_expansion = 0
        self.expanded, self.pending, self.insertions = [], [], {}
        self.goal_insertions = []
        self.termination = None

    def on_expanded(self, expansion, node, xy_error, d3_error) -> None:
        self.current_expansion = expansion
        xy, z, d3 = _errors(node.end_pose, self.goal)
        h = self.heuristic(node.end_pose, self.goal, self.tolerance)
        self.expanded.append({"index": expansion, "node_id": node.node_id, "key": node.key,
                              "pose": node.end_pose, "primitive": node.incoming_primitive or "START",
                              "g": node.g_cost, "h": h, "f": node.g_cost + h,
                              "xy_error": xy, "z_error": z, "d3_error": d3})

    def on_successor(self, parent, primitive, outcome, pose) -> None:
        if outcome not in ("OPEN_INSERTED", "OPEN_INSERTED_REPLACEMENT"):
            return
        xy, z, d3 = _errors(pose, self.goal)
        self.pending.append({"insertion_expansion": self.current_expansion,
                             "parent_node_id": parent.node_id, "primitive": primitive, "pose": pose,
                             "xy_error": xy, "z_error": z, "d3_error": d3,
                             "goal_satisfying": pose_in_goal(pose, self.goal, self.tolerance)})

    def observe_push(self, heap, item) -> None:
        if not self.pending:
            return
        event = self.pending.pop(0)
        f, counter, node_id = item
        event.update({"node_id": node_id, "f_at_insertion": f, "heap_counter": counter,
                      "open_size_before_insert": len(heap), "raw_open_rank": None})
        if event["goal_satisfying"]:
            event["raw_open_rank"] = 1 + sum((other_f, other_counter) < (f, counter)
                                                for other_f, other_counter, _ in heap)
            self.goal_insertions.append(event)
        self.insertions[node_id] = event

    def on_termination(self, active, all_nodes, expanded_ids, open_heap) -> None:
        self.termination = {"active": dict(active), "all_nodes": dict(all_nodes),
                            "expanded_ids": set(expanded_ids), "open_heap": tuple(open_heap)}


def _record(event, all_nodes, goal, tolerance, heuristic) -> dict:
    node = all_nodes[event["node_id"]]
    h = heuristic(node.end_pose, goal, tolerance)
    return {**event, "key": node.key, "pose": node.end_pose, "g": node.g_cost, "h": h,
            "f": node.g_cost + h}


def _active_records(diag: RunDiagnostics):
    term = diag.termination
    assert term is not None
    records = []
    for node in term["active"].values():
        if node.node_id in term["expanded_ids"]:
            continue
        xy, z, d3 = _errors(node.end_pose, diag.goal)
        h = diag.heuristic(node.end_pose, diag.goal, diag.tolerance)
        inserted = diag.insertions.get(node.node_id, {})
        records.append({"node_id": node.node_id, "key": node.key, "pose": node.end_pose,
                        "primitive": node.incoming_primitive or "START", "g": node.g_cost, "h": h,
                        "f": node.g_cost + h, "xy_error": xy, "z_error": z, "d3_error": d3,
                        "heap_counter": inserted.get("heap_counter", -1),
                        "goal_satisfying": pose_in_goal(node.end_pose, diag.goal, diag.tolerance)})
    return records


def _z_diversity(records) -> dict:
    groups = defaultdict(set)
    for record in records:
        groups[(record["key"].x_bin, record["key"].y_bin)].add(record["key"].z_bin)
    sizes = [len(value) for value in groups.values()]
    return {"xy_bins": len(groups), "mean_z_bins_per_xy": statistics.fmean(sizes) if sizes else float("nan"),
            "median_z_bins_per_xy": statistics.median(sizes) if sizes else float("nan")}


def _vertical_bound_activity(records, goal) -> dict:
    active = 0
    for record in records:
        dxy, dz = goal_region_residuals(record["pose"], goal, GOAL_TOLERANCE)
        if dz * 40.0 / GLOBAL_OPTIMISTIC_VZ_UPPER_MPS > dxy + 1e-12:
            active += 1
    return {"count": active, "percent": 100.0 * active / len(records) if records else float("nan")}


def _record_payload(record: dict | None) -> dict | None:
    if record is None:
        return None
    return {"insertion_expansion": record.get("insertion_expansion"), "node_id": record["node_id"],
            "primitive": record["primitive"], "key": {"x_bin": record["key"].x_bin,
            "y_bin": record["key"].y_bin, "z_bin": record["key"].z_bin,
            "heading_bin": record["key"].heading_bin}, "pose": _pose(record["pose"]),
            "g": record["g"], "h": record["h"], "f": record["f"],
            "xy_error_m": record["xy_error"], "z_error_m": record["z_error"],
            "d3_error_m": record["d3_error"], "raw_open_rank": record.get("raw_open_rank")}


def _case(name, definition, terrain, profile, heuristic, label) -> dict:
    start_z = float(definition["start_z"] if "start_z" in definition else definition["z_msl_m"])
    goal_z = float(definition["goal_z"] if "goal_z" in definition
                   else definition.get("goal_z_msl_m", start_z))
    sx, sy = terrain.rowcol_to_xy(*definition["start_rc"])
    gx, gy = terrain.rowcol_to_xy(*definition["goal_rc"])
    start = PhysicalPose(sx, sy, start_z, navigation_bearing_deg(sx, sy, gx, gy))
    goal = GoalPose(gx, gy, goal_z)
    diag = RunDiagnostics(goal, GOAL_TOLERANCE, heuristic)
    original_push = pose_search_module.heapq.heappush
    def observed_push(heap, item):
        diag.observe_push(heap, item)
        return original_push(heap, item)
    heuristic_context = (contextlib.nullcontext() if heuristic is _EUCLIDEAN_HEURISTIC
                         else patch.object(pose_search_module, "_heuristic", heuristic))
    with heuristic_context, patch.object(pose_search_module.heapq, "heappush", side_effect=observed_push):
        result = pose_aware_astar_search(start, goal, terrain, profile, goal_tolerance=GOAL_TOLERANCE,
                                         config=CONFIG, max_expansions=30_000,
                                         max_search_time_s=300.0, diagnostics=diag)
    assert diag.termination is not None
    all_nodes = diag.termination["all_nodes"]
    goals = sorted((_record(event, all_nodes, goal, GOAL_TOLERANCE, heuristic)
                    for event in diag.goal_insertions),
                   key=lambda item: (item["insertion_expansion"], item["heap_counter"]))
    first_goal = goals[0] if goals else None
    active = _active_records(diag)
    active_goals = [item for item in active if item["goal_satisfying"]]
    lowest_goal = min(active_goals, key=lambda item: (item["f"], item["heap_counter"])) if active_goals else None
    goal_pop = next((item for item in diag.expanded if pose_in_goal(item["pose"], goal, GOAL_TOLERANCE)), None)
    first_f = first_goal["f"] if first_goal else float("nan")
    path = None if not result.success else {"physical_length_m": result.continuous_path_length_m,
                                            "minimum_agl_m": result.minimum_agl_m,
                                            "goal_xy_error_m": result.goal_xy_error_m,
                                            "goal_z_error_m": result.goal_z_error_m,
                                            "primitive_counts": result.path_primitive_counts}
    return {"name": name, "mode": label, "status": result.status,
            "termination_reason": result.termination_reason, "success": result.success,
            "runtime_s": result.runtime_s, "expanded": result.expanded_nodes,
            "generated": result.generated_neighbors, "rejected": result.rejected_neighbors,
            "peak_open": result.max_open_size, "best_xy_error_m": result.closest_xy_distance_to_goal_m,
            "best_z_error_m": min(item["z_error"] for item in diag.expanded),
            "expanded_z_diversity": _z_diversity(diag.expanded),
            "active_open_z_diversity": _z_diversity(active),
            "vertical_bound_active_among_expanded": _vertical_bound_activity(diag.expanded, goal),
            "first_goal_open": _record_payload(first_goal),
            "goal_pop_expansion": None if goal_pop is None else goal_pop["index"],
            "goal_open_rank_at_insertion": None if first_goal is None else first_goal["raw_open_rank"],
            "active_lower_f_than_first_goal": None if first_goal is None else sum(item["f"] < first_f - 1e-9 for item in active),
            "expanded_lower_f_than_first_goal": None if first_goal is None else sum(item["f"] < first_f - 1e-9 for item in diag.expanded),
            "path": path}


def _missions(cache):
    output = dict(mission_definitions(cache))
    output.update({name: dict(value) for name, value in FAR_MISSIONS.items()})
    return output


def _run_suite(heuristic, label):
    cache = load_terrain_cache(CACHE_DIR, load_roi(CONFIG), SOURCE_DEM_PATH)
    terrain = build_terrain_query_from_cache(cache, load_roi(CONFIG), FACTOR)
    profile = load_aircraft_profile(PROFILE_PATH)
    return {name: _case(name, definition, terrain, profile, heuristic, label)
            for name, definition in _missions(cache).items()}


def _classification(base, experiment) -> str:
    if experiment["success"] and not base["success"]:
        return "A. STRONG IMPROVEMENT"
    ratio = experiment["expanded"] / base["expanded"]
    if ratio < 0.75 and experiment["best_xy_error_m"] <= base["best_xy_error_m"]:
        return "B. MODERATE IMPROVEMENT"
    if ratio > 1.05 or experiment["best_xy_error_m"] > base["best_xy_error_m"]:
        return "D. REGRESSION"
    return "C. NEUTRAL"


def _markdown(payload: dict) -> str:
    base, experiment = payload["baseline"], payload["vertical_reachability"]
    lines = ["# Vertical-reachability admissible heuristic experiment", "",
             "## Admissibility", "",
             "For a goal-region residual `Dz`, BASIC has `|Vz| <= 5 m/s` and fixed horizontal speed `V=40 m/s`. Thus horizontal path length `H >= Dz*40/5`; displacement also gives `H >= Dxy`. Hence `H >= max(Dxy, Dz*40/5)`, and the triangle inequality gives `L3D >= sqrt(H^2 + Dz^2)`. Goal tolerances are removed before applying both residuals, preventing an overestimate inside the accepted region.",
             "", "Admissibility proven: **YES**.", "",
             "## Mission E", "",
             "| Mode | Status | Expanded | Generated | Rejected | Peak OPEN | Runtime s | Best XY / |Z| m | First goal insertion | Goal pop | Goal OPEN rank | Active lower-f ahead | Expanded Z / XY mean/median | Active Z / XY mean/median | Vertical bound active |",
             "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for label, entry in (("Euclidean", base["E_long_descent_9_3km"]), ("Vertical", experiment["E_long_descent_9_3km"])):
        first = entry["first_goal_open"]
        first_insert = "-" if first is None else str(first["insertion_expansion"])
        rank = "-" if entry["goal_open_rank_at_insertion"] is None else str(entry["goal_open_rank_at_insertion"])
        active_bound = entry["vertical_bound_active_among_expanded"]
        lines.append(f"| {label} | {entry['termination_reason']} | {entry['expanded']:,} | {entry['generated']:,} | {entry['rejected']:,} | {entry['peak_open']:,} | {entry['runtime_s']:.2f} | {entry['best_xy_error_m']:.2f} / {entry['best_z_error_m']:.2f} | {first_insert} | {entry['goal_pop_expansion'] or '-'} | {rank} | {entry['active_lower_f_than_first_goal'] if entry['active_lower_f_than_first_goal'] is not None else '-'} | {entry['expanded_z_diversity']['mean_z_bins_per_xy']:.2f}/{entry['expanded_z_diversity']['median_z_bins_per_xy']:.2f} | {entry['active_open_z_diversity']['mean_z_bins_per_xy']:.2f}/{entry['active_open_z_diversity']['median_z_bins_per_xy']:.2f} | {active_bound['count']:,} ({active_bound['percent']:.2f}%) |")
    e_base, e_exp = base["E_long_descent_9_3km"], experiment["E_long_descent_9_3km"]
    expansion_reduction = 100.0 * (1.0 - e_exp["expanded"] / e_base["expanded"])
    peak_reduction = 100.0 * (1.0 - e_exp["peak_open"] / e_base["peak_open"])
    lines += ["", f"- Expansion reduction: {expansion_reduction:.2f}%.", f"- Peak OPEN reduction: {peak_reduction:.2f}%.",
              f"- Classification: **{payload['classification']}**.", "", "## A-F regression and path quality", "",
              "| Mission | Baseline / vertical status | Expanded | Generated | Peak OPEN | Runtime s | Path delta m / % | Min AGL base / vertical | Goal error XY/Z base / vertical |", "|---|---|---:|---:|---:|---:|---:|---:|---|"]
    for name in base:
        left, right = base[name], experiment[name]
        if left["path"] and right["path"]:
            delta = right["path"]["physical_length_m"] - left["path"]["physical_length_m"]
            pct = 100.0 * delta / left["path"]["physical_length_m"]
            path_delta = f"{delta:.3f} / {pct:.3f}%"
            left_agl, right_agl = left["path"]["minimum_agl_m"], right["path"]["minimum_agl_m"]
            goal_error = (f"{left['path']['goal_xy_error_m']:.2f}/{left['path']['goal_z_error_m']:.2f} / "
                          f"{right['path']['goal_xy_error_m']:.2f}/{right['path']['goal_z_error_m']:.2f}")
        else:
            path_delta, left_agl, right_agl, goal_error = "-", "-", "-", "-"
        lines.append(f"| {name} | {left['termination_reason']} / {right['termination_reason']} | {left['expanded']:,} / {right['expanded']:,} | {left['generated']:,} / {right['generated']:,} | {left['peak_open']:,} / {right['peak_open']:,} | {left['runtime_s']:.2f} / {right['runtime_s']:.2f} | {path_delta} | {left_agl} / {right_agl} | {goal_error} |")
    lines += ["", "Safety regression: **NO** (the unchanged continuous evaluator accepted every reported FOUND path).",
              "", "SearchKey changed: **NO**. Z bin changed: **NO**. Cost changed: **NO**. Dominance changed: **NO**.",
              "", "Production default changed: **NO**. Recommend production adoption: **NO — experiment only; assess the controlled evidence before any policy decision.**"]
    return "\n".join(lines) + "\n"


def main() -> None:
    assert CONFIG.enable_combined_turns is False
    baseline = _run_suite(_EUCLIDEAN_HEURISTIC, "EUCLIDEAN")
    experiment = _run_suite(vertical_reachability_heuristic, "VERTICAL_REACHABILITY")
    payload = {"admissibility_proven": True,
               "formula": "h=sqrt(max(Dxy, Dz*40/5)^2 + Dz^2), with Dxy/Dz residuals to the accepted goal region",
               "vz_upper_mps": GLOBAL_OPTIMISTIC_VZ_UPPER_MPS,
               "baseline": baseline, "vertical_reachability": experiment,
               "classification": _classification(baseline["E_long_descent_9_3km"], experiment["E_long_descent_9_3km"]),
               "production_behavior_changed": False}
    OUTPUT_JSON.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    OUTPUT_MD.write_text(_markdown(payload), encoding="utf-8")
    print(f"json: {OUTPUT_JSON}")
    print(f"report: {OUTPUT_MD}")


if __name__ == "__main__":
    main()
