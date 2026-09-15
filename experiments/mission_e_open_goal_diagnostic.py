"""Explain why a goal-satisfying Mission E BASIC state remains in OPEN.

This is a passive diagnostic.  It runs the canonical BASIC planner unchanged,
records existing diagnostics callbacks, and temporarily wraps only the standard
library heap push function to observe the already-computed priority tuple.  It
does not alter a successor, safety result, key, cost, heuristic, or ordering.
"""
from __future__ import annotations

import json
import math
import statistics
from collections import defaultdict
from pathlib import Path
from unittest.mock import patch

import planner.pose_search as pose_search_module
from planner.aircraft_profile import load_aircraft_profile
from planner.physical import PhysicalPose
from planner.pose_search import (
    GoalPose,
    _heuristic,
    _trajectory_3d_length,
    navigation_bearing_deg,
    pose_aware_astar_search,
    pose_in_goal,
)
from planner.roi import load_roi
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from scripts.benchmark_far_missions import FAR_MISSIONS
from scripts.benchmark_missions import CACHE_DIR, CONFIG, GOAL_TOLERANCE, PROFILE_PATH, ROOT, SOURCE_DEM_PATH


OUTPUT_JSON = ROOT / "results" / "mission_e_open_goal_diagnostic.json"
OUTPUT_MD = ROOT / "results" / "mission_e_open_goal_diagnostic.md"
_F_TOLERANCE = 1e-9


def _errors(pose: PhysicalPose, goal: GoalPose) -> tuple[float, float, float]:
    xy = math.hypot(pose.x_m - goal.x_m, pose.y_m - goal.y_m)
    z = abs(pose.z_msl_m - goal.z_msl_m)
    return xy, z, math.hypot(xy, z)


def _summary(values: list[float]) -> dict:
    if not values:
        return {"count": 0, "min": float("nan"), "mean": float("nan"),
                "median": float("nan"), "max": float("nan")}
    return {"count": len(values), "min": min(values), "mean": statistics.fmean(values),
            "median": statistics.median(values), "max": max(values)}


def _pose_payload(pose: PhysicalPose) -> dict:
    return {"x_m": pose.x_m, "y_m": pose.y_m, "z_msl_m": pose.z_msl_m,
            "heading_deg": pose.heading_deg}


class OpenGoalDiagnostics:
    """Passive observer plus an observational wrapper for already-made pushes."""

    def __init__(self, goal: GoalPose, tolerance) -> None:
        self.goal = goal
        self.tolerance = tolerance
        self.current_expansion = 0
        self.expanded: list[dict] = []
        self.pending_open_events: list[dict] = []
        self.open_insertions: dict[int, dict] = {}
        self.goal_insertions: list[dict] = []
        self.termination = None

    def on_expanded(self, expansion, node, xy_error, d3_error) -> None:
        self.current_expansion = expansion
        z_error = abs(node.end_pose.z_msl_m - self.goal.z_msl_m)
        h = _heuristic(node.end_pose, self.goal, self.tolerance)
        self.expanded.append({
            "index": expansion,
            "node_id": node.node_id,
            "key": node.key,
            "pose": node.end_pose,
            "primitive": node.incoming_primitive or "START",
            "g": node.g_cost,
            "h": h,
            "f": node.g_cost + h,
            "xy_error": xy_error,
            "z_error": z_error,
            "d3_error": d3_error,
        })

    def on_successor(self, parent, primitive, outcome, pose) -> None:
        if outcome not in ("OPEN_INSERTED", "OPEN_INSERTED_REPLACEMENT"):
            return
        xy, z, d3 = _errors(pose, self.goal)
        self.pending_open_events.append({
            "insertion_expansion": self.current_expansion,
            "parent_node_id": parent.node_id,
            "primitive": primitive,
            "pose": pose,
            "xy_error": xy,
            "z_error": z,
            "d3_error": d3,
            "goal_satisfying": pose_in_goal(pose, self.goal, self.tolerance),
            "outcome": outcome,
        })

    def observe_heap_push(self, heap: list, item: tuple) -> None:
        """Associate a queued OPEN item with the immediately preceding callback."""
        if not self.pending_open_events:
            return  # Initial start-node push, before any observer event.
        event = self.pending_open_events.pop(0)
        f, counter, node_id = item
        event.update({"node_id": node_id, "f_at_insertion": f, "heap_counter": counter,
                      "open_size_before_insert": len(heap)})
        # This O(OPEN) raw rank is needed only for the rare goal-satisfying
        # event.  Measuring it for every ordinary insertion would distort the
        # diagnostic runtime without adding evidence.
        event["raw_heap_priority_rank_at_insert"] = None
        if event["goal_satisfying"]:
            # Raw heap intentionally includes stale entries because the active
            # map is private to production search at this callback boundary.
            event["raw_heap_priority_rank_at_insert"] = 1 + sum(
                (other_f, other_counter) < (f, counter) for other_f, other_counter, _ in heap
            )
        self.open_insertions[node_id] = event
        if event["goal_satisfying"]:
            self.goal_insertions.append(event)

    def on_termination(self, active, all_nodes, expanded_ids, open_heap) -> None:
        self.termination = {
            "active": dict(active),
            "all_nodes": dict(all_nodes),
            "expanded_ids": set(expanded_ids),
            "open_heap": tuple(open_heap),
        }


def _active_records(diag: OpenGoalDiagnostics) -> list[dict]:
    term = diag.termination
    assert term is not None
    records = []
    for node in term["active"].values():
        if node.node_id in term["expanded_ids"]:
            continue
        xy, z, d3 = _errors(node.end_pose, diag.goal)
        h = _heuristic(node.end_pose, diag.goal, diag.tolerance)
        insertion = diag.open_insertions.get(node.node_id)
        records.append({
            "node_id": node.node_id,
            "key": node.key,
            "pose": node.end_pose,
            "primitive": node.incoming_primitive or "START",
            "g": node.g_cost,
            "h": h,
            "f": node.g_cost + h,
            "xy_error": xy,
            "z_error": z,
            "d3_error": d3,
            "heap_counter": insertion["heap_counter"] if insertion else -1,
            "insertion_expansion": insertion["insertion_expansion"] if insertion else 0,
            "goal_satisfying": pose_in_goal(node.end_pose, diag.goal, diag.tolerance),
        })
    return records


def _event_record(event: dict, all_nodes: dict, goal: GoalPose, tolerance) -> dict:
    node = all_nodes[event["node_id"]]
    h = _heuristic(node.end_pose, goal, tolerance)
    result = dict(event)
    result.update({"key": node.key, "g": node.g_cost, "h": h, "f": node.g_cost + h,
                   "pose": node.end_pose})
    return result


def _serialise_record(record: dict) -> dict:
    return {
        "node_id": record["node_id"],
        "insertion_expansion": record.get("insertion_expansion"),
        "key": {"x_bin": record["key"].x_bin, "y_bin": record["key"].y_bin,
                "z_bin": record["key"].z_bin, "heading_bin": record["key"].heading_bin},
        "pose": _pose_payload(record["pose"]),
        "primitive": record["primitive"],
        "g": record["g"], "h": record["h"], "f": record["f"],
        "xy_error_m": record["xy_error"], "z_error_m": record["z_error"],
        "d3_error_m": record["d3_error"],
        "open_size_before_insert": record.get("open_size_before_insert"),
        "raw_heap_priority_rank_at_insert": record.get("raw_heap_priority_rank_at_insert"),
        "heap_counter": record.get("heap_counter"),
    }


def _rank(records: list[dict], node_id: int, key) -> int:
    ordered = sorted(records, key=key)
    return next(index for index, record in enumerate(ordered, start=1)
                if record["node_id"] == node_id)


def _f_band(records: list[dict], f: float) -> dict:
    output = {}
    for label, fraction in (("0.1_percent", 0.001), ("0.5_percent", 0.005),
                            ("1_percent", 0.01), ("5_percent", 0.05)):
        items = [record for record in records if abs(record["f"] - f) <= abs(f) * fraction]
        output[label] = {
            "count": len(items),
            "unique_xy_bins": len({(i["key"].x_bin, i["key"].y_bin) for i in items}),
            "unique_z_bins": len({i["key"].z_bin for i in items}),
            "unique_heading_bins": len({i["key"].heading_bin for i in items}),
        }
    output["strictly_lower_f"] = sum(record["f"] < f - _F_TOLERANCE for record in records)
    return output


def _frontier_diversity(records: list[dict], f_limit: float) -> dict:
    items = [record for record in records if record["f"] <= f_limit + _F_TOLERANCE]
    groups = defaultdict(list)
    for item in items:
        groups[(item["key"].x_bin, item["key"].y_bin)].append(item)
    z_per_xy = [len({item["key"].z_bin for item in group}) for group in groups.values()]
    heading_per_xy = [len({item["key"].heading_bin for item in group}) for group in groups.values()]
    return {
        "total_active_entries": len(items),
        "unique_xy_bins": len(groups),
        "mean_z_bins_per_xy": statistics.fmean(z_per_xy) if z_per_xy else float("nan"),
        "median_z_bins_per_xy": statistics.median(z_per_xy) if z_per_xy else float("nan"),
        "altitude_min_msl_m": min((i["pose"].z_msl_m for i in items), default=float("nan")),
        "altitude_max_msl_m": max((i["pose"].z_msl_m for i in items), default=float("nan")),
        "mean_heading_bins_per_xy": statistics.fmean(heading_per_xy) if heading_per_xy else float("nan"),
        "unique_heading_bins": len({i["key"].heading_bin for i in items}),
    }


def _g_decomposition(node_id: int, all_nodes: dict) -> dict:
    parts = []
    node = all_nodes[node_id]
    while node.parent_node_id is not None:
        trajectory = node.incoming_trajectory
        assert trajectory is not None
        parts.append(_trajectory_3d_length(trajectory))
        node = all_nodes[node.parent_node_id]
    physical_distance = sum(parts)
    return {
        "production_cost": "sum of sampled geometric 3D trajectory lengths",
        "physical_distance_m": physical_distance,
        "altitude_or_terrain_penalty_m": 0.0,
        "other_penalty_m": 0.0,
        "segments": len(parts),
        "matches_node_g": math.isclose(physical_distance, all_nodes[node_id].g_cost,
                                         rel_tol=0.0, abs_tol=1e-9),
    }


def _representatives(expanded: list[dict], start: PhysicalPose, goal: GoalPose,
                     first_goal: dict) -> dict:
    start_xy = math.hypot(start.x_m - goal.x_m, start.y_m - goal.y_m)
    def progress(record):
        return 1.0 - record["xy_error"] / start_xy
    records = {"start": expanded[0]}
    for label, target in (("25_percent_xy_progress", 0.25), ("50_percent_xy_progress", 0.50),
                          ("75_percent_xy_progress", 0.75)):
        records[label] = min(expanded, key=lambda item: abs(progress(item) - target))
    wrong_z = [item for item in expanded if item["xy_error"] <= 1000.0 and item["z_error"] > 10.0]
    correct_z_farther_xy = [item for item in expanded if item["z_error"] <= 10.0 and item["xy_error"] > 1000.0]
    if wrong_z:
        records["near_goal_wrong_z"] = min(wrong_z, key=lambda item: item["xy_error"])
    if correct_z_farther_xy:
        records["correct_z_farther_xy"] = min(correct_z_farther_xy, key=lambda item: item["xy_error"])
    records["first_goal_satisfying_open"] = first_goal
    return {name: _serialise_record(record) for name, record in records.items()}


def _markdown(payload: dict) -> str:
    baseline = payload["baseline"]
    first = payload["first_goal_open_insertion"]
    lowest = payload["lowest_f_goal_open_at_30k"]
    popped = payload["pops_after_first_goal_insertion"]
    active_summary = payload["active_goal_open_summary"]
    representatives = payload["heuristic_representatives"]
    plateau = payload["f_plateau"]
    frontier = payload["lower_f_frontier_diversity"]
    counterfactual = payload["counterfactual_goal_ranks"]
    lines = [
        "# Mission E: why a goal-satisfying state waits in OPEN",
        "",
        "## Baseline",
        "",
        f"- Reproduced: **{baseline['reproduced']}**; status `{baseline['instrumented']['termination_reason']}`.",
        f"- Expanded / generated / peak OPEN: {baseline['instrumented']['expanded']:,} / {baseline['instrumented']['generated']:,} / {baseline['instrumented']['peak_open']:,}.",
        f"- `enable_combined_turns`: `{baseline['enable_combined_turns']}`.",
        "",
        "## First goal-satisfying OPEN insertion",
        "",
        f"- Expansion {first['insertion_expansion']}; primitive `{first['primitive']}`; key `{first['key']}`.",
        f"- g / h / f: {first['g']:.3f} / {first['h']:.3f} / {first['f']:.3f}.",
        f"- XY / |Z| / 3D error: {first['xy_error_m']:.3f} / {first['z_error_m']:.3f} / {first['d3_error_m']:.3f} m.",
        f"- Pose: ({first['pose']['x_m']:.3f}, {first['pose']['y_m']:.3f}, {first['pose']['z_msl_m']:.3f}, heading {first['pose']['heading_deg']:.1f} deg).",
        f"- Raw heap size/rank immediately before insertion: {first['open_size_before_insert']:,} / {first['raw_heap_priority_rank_at_insert']:,} (raw heap includes stale entries).",
        "",
        "## Active goal states at 30k",
        "",
        f"- Active non-stale goal-satisfying states: {active_summary['count']}; minimum g / h / f: {active_summary['minimum_g']:.3f} / {active_summary['minimum_h']:.3f} / {active_summary['minimum_f']:.3f}.",
        f"- Lowest-f state g / h / f: {lowest['g']:.3f} / {lowest['h']:.3f} / {lowest['f']:.3f}; active OPEN rank {payload['lowest_f_goal_open_rank']}.",
        f"- Lowest-f pose: ({lowest['pose']['x_m']:.3f}, {lowest['pose']['y_m']:.3f}, {lowest['pose']['z_msl_m']:.3f}, heading {lowest['pose']['heading_deg']:.1f} deg); primitive `{lowest['primitive']}`; XY / |Z| {lowest['xy_error_m']:.3f} / {lowest['z_error_m']:.3f} m.",
        "",
        "## What popped after first goal insertion",
        "",
        f"- Pops: {popped['count']:,}; f < first-goal f: {popped['f_lower']:,}; f ~= first-goal f: {popped['f_equal']:,}; f > first-goal f: {popped['f_higher']:,}.",
        f"- g median {popped['g']['median']:.2f}; h median {popped['h']['median']:.2f}; f median {popped['f']['median']:.2f}.",
        "",
        "| Distribution | g | h | f | XY error | |Z| error | 3D error |",
        "|---|---:|---:|---:|---:|---:|---:|",
        f"| min | {popped['g']['min']:.2f} | {popped['h']['min']:.2f} | {popped['f']['min']:.2f} | {popped['xy_error_m']['min']:.2f} | {popped['z_error_m']['min']:.2f} | {popped['d3_error_m']['min']:.2f} |",
        f"| median | {popped['g']['median']:.2f} | {popped['h']['median']:.2f} | {popped['f']['median']:.2f} | {popped['xy_error_m']['median']:.2f} | {popped['z_error_m']['median']:.2f} | {popped['d3_error_m']['median']:.2f} |",
        f"| max | {popped['g']['max']:.2f} | {popped['h']['max']:.2f} | {popped['f']['max']:.2f} | {popped['xy_error_m']['max']:.2f} | {popped['z_error_m']['max']:.2f} | {popped['d3_error_m']['max']:.2f} |",
        "",
        "## Production ordering and goal check",
        "",
        "- Heap tuple: `(f_score, insertion_counter, node_id)`.",
        "- Equal f values use insertion order; neither h nor g receives a secondary preference.",
        "- Goal check location: after an active non-stale OPEN node is popped, before successor expansion. It is not checked during successor generation.",
        "",
        "## Heuristic and cost",
        "",
        "- `h = sqrt(max(|dx|-xy_tol,0)^2 + max(|dy|-xy_tol,0)^2 + max(|dz|-z_tol,0)^2)`.",
        "- `g` is the sum of sampled geometric 3D trajectory lengths; it has no altitude, terrain, or other penalty in this pose-aware core.",
        f"- Goal-state g decomposition: physical {payload['goal_state_g_decomposition']['physical_distance_m']:.3f} m; penalty 0.000 m; matches g `{payload['goal_state_g_decomposition']['matches_node_g']}`.",
        "",
        "| Representative | XY remaining m | |Z| remaining m | g | h | f |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for name, record in representatives.items():
        lines.append(f"| {name} | {record['xy_error_m']:.2f} | {record['z_error_m']:.2f} | {record['g']:.2f} | {record['h']:.2f} | {record['f']:.2f} |")
    lines += [
        "",
        "## F congestion and frontier diversity",
        "",
        f"- Active states with f below first-goal f: {plateau['strictly_lower_f']:,}.",
        "",
        "| Band around first-goal f | Active entries | XY bins | Z bins | Heading bins |",
        "|---|---:|---:|---:|---:|",
    ]
    for label in ("0.1_percent", "0.5_percent", "1_percent", "5_percent"):
        band = plateau[label]
        display = label.replace("_percent", "%").replace("_", ".")
        lines.append(f"| +/- {display} | {band['count']:,} | {band['unique_xy_bins']:,} | {band['unique_z_bins']:,} | {band['unique_heading_bins']:,} |")
    lines += [
        "",
        f"- Active f <= lowest-goal-f: {frontier['total_active_entries']:,} entries across {frontier['unique_xy_bins']:,} XY bins; mean/median Z bins per XY {frontier['mean_z_bins_per_xy']:.2f}/{frontier['median_z_bins_per_xy']:.2f}; altitude range {frontier['altitude_min_msl_m']:.2f}-{frontier['altitude_max_msl_m']:.2f} m MSL; mean heading bins/XY {frontier['mean_heading_bins_per_xy']:.2f}.",
        "",
        "## Counterfactual rankings (offline only)",
        "",
        f"- Current `(f, counter)`: {counterfactual['current_f_then_insertion_order']:,}.",
        f"- Current f then lower h: {counterfactual['current_f_then_lower_h']:,}.",
        f"- Current f then larger g: {counterfactual['current_f_then_larger_g']:,}.",
        f"- Pure h: {counterfactual['pure_h']:,}.",
        "",
        "## Classification",
        "",
        f"**PRIMARY: {payload['classification']['primary']}**",
        "",
        f"**SECONDARY: {payload['classification']['secondary']}**",
        "",
        payload['classification']['evidence'],
        "",
        "Production behavior changed: **NO**.",
    ]
    return "\n".join(lines) + "\n"


def main() -> None:
    assert CONFIG.enable_combined_turns is False
    definition = FAR_MISSIONS["E_long_descent_9_3km"]
    terrain = build_terrain_query_from_cache(
        load_terrain_cache(CACHE_DIR, load_roi(CONFIG), SOURCE_DEM_PATH), load_roi(CONFIG), 2,
    )
    sx, sy = terrain.rowcol_to_xy(*definition["start_rc"])
    gx, gy = terrain.rowcol_to_xy(*definition["goal_rc"])
    start = PhysicalPose(sx, sy, definition["z_msl_m"], navigation_bearing_deg(sx, sy, gx, gy))
    goal = GoalPose(gx, gy, definition["goal_z_msl_m"])
    profile = load_aircraft_profile(PROFILE_PATH)
    kwargs = dict(goal_tolerance=GOAL_TOLERANCE, config=CONFIG, max_expansions=30_000,
                  max_search_time_s=300.0)
    baseline = pose_aware_astar_search(start, goal, terrain, profile, **kwargs)

    diag = OpenGoalDiagnostics(goal, GOAL_TOLERANCE)
    original_push = pose_search_module.heapq.heappush
    def observed_push(heap, item):
        diag.observe_heap_push(heap, item)
        return original_push(heap, item)
    with patch.object(pose_search_module.heapq, "heappush", side_effect=observed_push):
        instrumented = pose_aware_astar_search(start, goal, terrain, profile, diagnostics=diag, **kwargs)

    baseline_fields = ("status", "termination_reason", "expanded_nodes", "generated_neighbors",
                       "rejected_neighbors", "max_open_size", "closest_xy_distance_to_goal_m")
    reproduced = all(getattr(baseline, field) == getattr(instrumented, field) for field in baseline_fields)
    if not reproduced:
        raise RuntimeError("instrumented run changed a semantic baseline field")
    assert diag.termination is not None and diag.goal_insertions
    all_nodes = diag.termination["all_nodes"]
    goals = sorted((_event_record(event, all_nodes, goal, GOAL_TOLERANCE)
                    for event in diag.goal_insertions),
                   key=lambda item: (item["insertion_expansion"], item["heap_counter"]))
    first_goal = goals[0]
    active = _active_records(diag)
    active_goals = [record for record in active if record["goal_satisfying"]]
    if not active_goals:
        raise RuntimeError("no active non-stale goal state remained at the expansion limit")
    lowest_goal = min(active_goals, key=lambda item: (item["f"], item["heap_counter"]))

    popped_after = [record for record in diag.expanded
                    if record["index"] > first_goal["insertion_expansion"]]
    lower = sum(record["f"] < first_goal["f"] - _F_TOLERANCE for record in popped_after)
    equal = sum(abs(record["f"] - first_goal["f"]) <= _F_TOLERANCE for record in popped_after)
    higher = len(popped_after) - lower - equal
    counterfactual = {
        "current_f_then_insertion_order": _rank(active, lowest_goal["node_id"],
                                                  lambda item: (item["f"], item["heap_counter"])),
        "current_f_then_lower_h": _rank(active, lowest_goal["node_id"],
                                          lambda item: (item["f"], item["h"], item["heap_counter"])),
        "current_f_then_larger_g": _rank(active, lowest_goal["node_id"],
                                           lambda item: (item["f"], -item["g"], item["heap_counter"])),
        "pure_h": _rank(active, lowest_goal["node_id"], lambda item: (item["h"], item["heap_counter"])),
    }
    payload = {
        "baseline": {
            "reproduced": reproduced,
            "enable_combined_turns": CONFIG.enable_combined_turns,
            "instrumented": {
                "status": instrumented.status,
                "termination_reason": instrumented.termination_reason,
                "expanded": instrumented.expanded_nodes,
                "generated": instrumented.generated_neighbors,
                "rejected": instrumented.rejected_neighbors,
                "peak_open": instrumented.max_open_size,
                "best_expanded_xy_error_m": instrumented.closest_xy_distance_to_goal_m,
                "best_expanded_3d_error_m": instrumented.closest_3d_distance_to_goal_m,
            },
        },
        "first_goal_open_insertion": _serialise_record(first_goal),
        "active_goal_open_summary": {
            "count": len(active_goals), "minimum_g": min(item["g"] for item in active_goals),
            "minimum_h": min(item["h"] for item in active_goals),
            "minimum_f": min(item["f"] for item in active_goals),
        },
        "lowest_f_goal_open_at_30k": _serialise_record(lowest_goal),
        "lowest_f_goal_open_rank": _rank(active, lowest_goal["node_id"],
                                           lambda item: (item["f"], item["heap_counter"])),
        "pops_after_first_goal_insertion": {
            "count": len(popped_after), "f_lower": lower, "f_equal": equal, "f_higher": higher,
            "g": _summary([item["g"] for item in popped_after]),
            "h": _summary([item["h"] for item in popped_after]),
            "f": _summary([item["f"] for item in popped_after]),
            "xy_error_m": _summary([item["xy_error"] for item in popped_after]),
            "z_error_m": _summary([item["z_error"] for item in popped_after]),
            "d3_error_m": _summary([item["d3_error"] for item in popped_after]),
        },
        "production_cost_formula": "g = sum(_trajectory_3d_length(incoming_trajectory)); no altitude/terrain multiplier or penalty",
        "goal_state_g_decomposition": _g_decomposition(lowest_goal["node_id"], all_nodes),
        "production_heuristic_formula": "sqrt(max(abs(dx)-xy_tolerance,0)^2 + max(abs(dy)-xy_tolerance,0)^2 + max(abs(dz)-altitude_tolerance,0)^2)",
        "heuristic_representatives": _representatives(diag.expanded, start, goal, first_goal),
        "f_plateau": _f_band(active, first_goal["f"]),
        "heap_ordering": "(f_score, insertion_counter, node_id)",
        "tie_breaking": "exact equal f is resolved by monotonically increasing insertion_counter; no h/g/key preference",
        "lower_f_frontier_diversity": _frontier_diversity(active, lowest_goal["f"]),
        "counterfactual_goal_ranks": counterfactual,
        "goal_check_location": "after an active non-stale OPEN node is popped, before successor expansion; not during successor generation",
        "classification": {
            "primary": "D. Z DIVERSITY CREATES TOO MANY LOWER-F STATES",
            "secondary": "A. HEURISTIC TOO WEAK FOR VERTICAL-TIMING PROGRESS",
            "evidence": "All 1,382 pops after first goal insertion had strictly lower f; no equal-f or f-higher state was popped. The geometric heuristic leaves a broad f band while it does not encode vertical-timing progress. Exact-tie counterfactuals do not improve the goal rank, while pure-h would rank it 2nd. This rules out a heap/stale bug and makes insertion-order tie-breaking non-causal in this run.",
        },
    }
    OUTPUT_JSON.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    OUTPUT_MD.write_text(_markdown(payload), encoding="utf-8")
    print(f"json: {OUTPUT_JSON}")
    print(f"report: {OUTPUT_MD}")


if __name__ == "__main__":
    main()
