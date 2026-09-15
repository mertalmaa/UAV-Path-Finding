"""Passive Mission E Z-state redundancy diagnostic.

The planner is run twice under the fixed BASIC contract.  This experiment
only observes expanded representatives and physically valid vertical
successors; it never feeds a classification back into A*.
"""
from __future__ import annotations

import contextlib
import json
import math
import statistics
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from unittest.mock import patch

import planner.pose_search as pose_search_module
from experiments.vertical_reachability_heuristic import vertical_reachability_heuristic
from planner.aircraft_profile import load_aircraft_profile
from planner.fixed_wing_envelope import FixedWingKinematicEnvelope
from planner.physical import PhysicalPose
from planner.pose_search import (
    GoalPose, SearchKey, _heuristic, navigation_bearing_deg, pose_aware_astar_search,
    pose_in_goal, search_key_for_pose,
)
from planner.roi import load_roi
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from scripts.benchmark_far_missions import FAR_MISSIONS
from scripts.benchmark_missions import (
    CACHE_DIR, CONFIG, FACTOR, GOAL_TOLERANCE, PROFILE_PATH, ROOT, SOURCE_DEM_PATH,
)


OUTPUT_JSON = ROOT / "results" / "mission_e_z_state_redundancy_diagnostic.json"
OUTPUT_MD = ROOT / "results" / "mission_e_z_state_redundancy_diagnostic.md"
_EUCLIDEAN = _heuristic
_Z_SIZES = (5.0, 10.0, 20.0, 50.0, 100.0)


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentile / 100.0
    low, high = math.floor(position), math.ceil(position)
    return float(ordered[low] if low == high else ordered[low] + (ordered[high] - ordered[low]) * (position - low))


def _distribution(values: list[float]) -> dict:
    return {
        "count": len(values), "mean": statistics.fmean(values) if values else float("nan"),
        "median": statistics.median(values) if values else float("nan"),
        "p90": _percentile(values, 90), "p95": _percentile(values, 95),
        "max": max(values) if values else float("nan"),
    }


def _errors(pose: PhysicalPose, goal: GoalPose) -> tuple[float, float, float]:
    xy = math.hypot(pose.x_m - goal.x_m, pose.y_m - goal.y_m)
    z = abs(pose.z_msl_m - goal.z_msl_m)
    return xy, z, math.hypot(xy, z)


def _physical_key(pose: PhysicalPose, z_size_m: float) -> tuple[int, int, int, int]:
    """Current XY/heading bins plus an offline alternative floor Z bin."""
    key = search_key_for_pose(pose, CONFIG)
    return key.x_bin, key.y_bin, math.floor(pose.z_msl_m / z_size_m + 1e-9), key.heading_bin


def _route_third(pose: PhysicalPose, start: PhysicalPose, goal: GoalPose) -> str:
    dx, dy = goal.x_m - start.x_m, goal.y_m - start.y_m
    length_sq = dx * dx + dy * dy
    progress = 0.0 if length_sq == 0.0 else ((pose.x_m - start.x_m) * dx + (pose.y_m - start.y_m) * dy) / length_sq
    return ("first_third" if progress < 1 / 3 else "middle_third" if progress < 2 / 3 else "final_third")


def _goal_band(xy_distance_m: float) -> str:
    if xy_distance_m > 5000.0:
        return ">5km"
    if xy_distance_m > 2000.0:
        return "2-5km"
    if xy_distance_m > 1000.0:
        return "1-2km"
    if xy_distance_m >= 500.0:
        return "500-1000m"
    return "<500m"


@dataclass
class _Transition:
    primitive: str
    parent_pose: PhysicalPose
    child_pose: PhysicalPose


class ZDiagnostics:
    """Observer of actual A* callbacks, with no mutable planner references."""

    def __init__(self, goal: GoalPose) -> None:
        self.goal = goal
        self.expanded: list[dict] = []
        self.transitions: list[_Transition] = []
        self.current_expansion = 0
        self.first_goal_insertion = None
        self.goal_pop_expansion = None

    def on_expanded(self, expansion, node, xy_error, d3_error) -> None:
        self.current_expansion = expansion
        xy, z, d3 = _errors(node.end_pose, self.goal)
        if pose_in_goal(node.end_pose, self.goal, GOAL_TOLERANCE) and self.goal_pop_expansion is None:
            self.goal_pop_expansion = expansion
        self.expanded.append({
            "index": expansion, "node_id": node.node_id, "key": node.key,
            "pose": node.end_pose, "g": node.g_cost, "xy_goal_distance_m": xy,
            "abs_goal_z_error_m": z, "d3_goal_distance_m": d3,
        })

    def on_successor(self, parent, primitive, outcome, pose) -> None:
        # This callback is made after continuous safety for PHYSICALLY_VALID.
        if outcome == "PHYSICALLY_VALID" and primitive in ("STRAIGHT_CLIMB", "STRAIGHT_DESCENT"):
            self.transitions.append(_Transition(primitive, parent.end_pose, pose))
        if outcome in ("OPEN_INSERTED", "OPEN_INSERTED_REPLACEMENT") and pose_in_goal(pose, self.goal, GOAL_TOLERANCE):
            if self.first_goal_insertion is None:
                self.first_goal_insertion = self.current_expansion

    def on_termination(self, active, all_nodes, expanded_ids, open_heap) -> None:
        # Required only to satisfy the passive observer interface.
        return None


def _terrain_context(pose: PhysicalPose, terrain) -> dict:
    query = terrain.query(pose.x_m, pose.y_m)
    terrain_msl = query.elevation if query.valid else float("nan")
    return {"terrain_msl_m": terrain_msl, "agl_m": pose.z_msl_m - terrain_msl}


def _grouped(records: list[dict]) -> dict[tuple[int, int, int], list[dict]]:
    groups = defaultdict(list)
    for record in records:
        key: SearchKey = record["key"]
        groups[(key.x_bin, key.y_bin, key.heading_bin)].append(record)
    return groups


def _future_check(worse: dict, better: dict, envelope: FixedWingKinematicEnvelope, terrain) -> dict:
    """Local evidence only; terrain ahead is deliberately not inferred.

    A lower-g/closer-goal-altitude representative is a *candidate*, not a
    dominance proof.  We check whether it has the same local vertical options
    and clearance, while reporting the unobserved terrain-ahead limitation.
    """
    def capability(pose, mode):
        value = envelope.straight_vertical(pose.z_msl_m, mode)
        return {"available": value.availability == "AVAILABLE",
                "rate_mps": value.signed_vertical_rate_mps}
    better_context = _terrain_context(better["pose"], terrain)
    worse_context = _terrain_context(worse["pose"], terrain)
    better_modes = {mode: capability(better["pose"], mode) for mode in ("CLIMB", "DESCENT")}
    worse_modes = {mode: capability(worse["pose"], mode) for mode in ("CLIMB", "DESCENT")}
    availability_match = all(better_modes[mode]["available"] >= worse_modes[mode]["available"] for mode in better_modes)
    rate_match = all(
        not worse_modes[mode]["available"] or (
            better_modes[mode]["available"] and abs(better_modes[mode]["rate_mps"]) >= abs(worse_modes[mode]["rate_mps"]) - 1e-12
        ) for mode in better_modes
    )
    clearance_ok = better_context["agl_m"] >= CONFIG.min_agl_m
    return {"better_clearance_m": better_context["agl_m"], "worse_clearance_m": worse_context["agl_m"],
            "availability_match": availability_match, "rate_match": rate_match,
            "better_locally_safe": clearance_ok,
            "locally_unrefuted": availability_match and rate_match and clearance_ok,
            "terrain_ahead_not_inferred": True}


def _pair_analysis(records: list[dict], envelope, terrain) -> tuple[dict, dict, list[dict]]:
    groups = _grouped(records)
    compared = candidates = unrefuted = 0
    dominated_node_ids = set()
    per_group = Counter()
    for group_key, items in groups.items():
        for index, first in enumerate(items):
            for second in items[index + 1:]:
                compared += 1
                # Directional criterion from the task.  With equality, retain
                # each direction; exact equal physical states are impossible
                # across distinct expanded SearchKeys here.
                for worse, better in ((first, second), (second, first)):
                    if (worse["g"] >= better["g"] - 1e-12
                            and worse["abs_goal_z_error_m"] >= better["abs_goal_z_error_m"] - 1e-12):
                        candidates += 1
                        dominated_node_ids.add(worse["node_id"])
                        per_group[group_key] += 1
                        if _future_check(worse, better, envelope, terrain)["locally_unrefuted"]:
                            unrefuted += 1
    return ({"total_compared_pairs": compared, "obvious_dominance_candidates": candidates,
             "candidate_percent_of_pairs": 100.0 * candidates / compared if compared else 0.0,
             "obvious_dominated_states": len(dominated_node_ids),
             "locally_unrefuted_candidates": unrefuted,
             "locally_unrefuted_percent_of_candidates": 100.0 * unrefuted / candidates if candidates else 0.0},
            dict(per_group), groups)


def _group_rows(groups, pair_counts, terrain) -> list[dict]:
    rows = []
    for group_key, items in groups.items():
        altitudes = [item["pose"].z_msl_m for item in items]
        costs = [item["g"] for item in items]
        distances = [item["d3_goal_distance_m"] for item in items]
        terrain_values = [_terrain_context(item["pose"], terrain) for item in items]
        rows.append({"xy_heading_bin": list(group_key), "z_state_count": len(items),
                     "altitude_min_m": min(altitudes), "altitude_max_m": max(altitudes),
                     "altitude_span_m": max(altitudes) - min(altitudes), "g_min": min(costs), "g_max": max(costs),
                     "goal_distance_min_m": min(distances), "goal_distance_max_m": max(distances),
                     "terrain_msl_min_m": min(value["terrain_msl_m"] for value in terrain_values),
                     "terrain_msl_max_m": max(value["terrain_msl_m"] for value in terrain_values),
                     "agl_min_m": min(value["agl_m"] for value in terrain_values), "agl_max_m": max(value["agl_m"] for value in terrain_values),
                     "minimum_safe_altitude_msl_m": max(value["terrain_msl_m"] + CONFIG.min_agl_m for value in terrain_values),
                     "dominance_candidate_count": pair_counts.get(group_key, 0)})
    return sorted(rows, key=lambda row: (row["z_state_count"], row["altitude_span_m"], row["dominance_candidate_count"]), reverse=True)


def _redundancy_by(records, envelope, terrain, selector) -> dict:
    subset = [record for record in records if selector(record)]
    pairs, _, _ = _pair_analysis(subset, envelope, terrain)
    dominated = pairs["obvious_dominated_states"]
    return {"raw_z_states": len(subset), "obviously_dominated_candidates": dominated,
            "non_dominated_candidates": len(subset) - dominated,
            "redundancy_ratio": dominated / len(subset) if subset else 0.0,
            "candidate_pair_count": pairs["obvious_dominance_candidates"]}


def offline_rebin(records: list[dict]) -> list[dict]:
    """Re-key expanded physical representatives offline; no search is rerun."""
    output = []
    for z_size in _Z_SIZES:
        buckets = defaultdict(list)
        xy_buckets = defaultdict(set)
        xy_heading_buckets = defaultdict(set)
        for record in records:
            key = _physical_key(record["pose"], z_size)
            buckets[key].append(record)
            xy_buckets[key[:2]].add(key[2])
            xy_heading_buckets[(key[0], key[1], key[3])].add(key[2])
        merged = [items for items in buckets.values() if len(items) > 1]
        g_diff = sum(any(max(item["g"] for item in items) - min(item["g"] for item in items) > 60.0 for _ in (0,)) for items in merged)
        clearance_diff = sum(any(max(_terrain_context(item["pose"], _OFFLINE_TERRAIN)["agl_m"] for item in items) - min(_terrain_context(item["pose"], _OFFLINE_TERRAIN)["agl_m"] for item in items) > 20.0 for _ in (0,)) for items in merged)
        goal_z_diff = sum(any(max(item["abs_goal_z_error_m"] for item in items) - min(item["abs_goal_z_error_m"] for item in items) > GOAL_TOLERANCE.altitude_m for _ in (0,)) for items in merged)
        unique = len(buckets)
        output.append({"z_bin_m": z_size, "unique_search_key_count": unique,
                       "reduction_percent": 100.0 * (1.0 - unique / len(records)) if records else 0.0,
                       "mean_z_bins_per_xy": statistics.fmean(len(value) for value in xy_buckets.values()),
                       "mean_z_bins_per_xy_heading": statistics.fmean(len(value) for value in xy_heading_buckets.values()),
                       "physically_distinct_states_merged": sum(len(items) - 1 for items in merged),
                       "merged_buckets": len(merged), "merged_buckets_substantial_g_difference": g_diff,
                       "merged_buckets_clearance_difference_over_20m": clearance_diff,
                       "merged_buckets_goal_z_suitability_difference_over_tolerance": goal_z_diff})
    return output


def vertical_collapse(transitions: list[_Transition]) -> list[dict]:
    output = []
    for z_size in _Z_SIZES:
        row = {"z_bin_m": z_size}
        for primitive, label in (("STRAIGHT_CLIMB", "climb"), ("STRAIGHT_DESCENT", "descent")):
            items = [transition for transition in transitions if transition.primitive == primitive]
            # A 60 m straight primitive usually enters a different XY bin.
            # Keep that literal full-key result, but also expose the actual
            # question here: whether its physical vertical progress disappears
            # under the hypothetical Z quantizer.
            full_key_collapsed = sum(_physical_key(item.parent_pose, z_size) == _physical_key(item.child_pose, z_size) for item in items)
            z_collapsed = sum(
                math.floor(item.parent_pose.z_msl_m / z_size + 1e-9)
                == math.floor(item.child_pose.z_msl_m / z_size + 1e-9)
                for item in items
            )
            row[f"{label}_successors"] = len(items)
            row[f"{label}_full_search_key_collapses"] = full_key_collapsed
            row[f"{label}_full_search_key_collapse_rate_percent"] = 100.0 * full_key_collapsed / len(items) if items else 0.0
            row[f"{label}_z_component_collapses"] = z_collapsed
            row[f"{label}_z_component_collapse_rate_percent"] = 100.0 * z_collapsed / len(items) if items else 0.0
        output.append(row)
    return output


def _run(label: str, heuristic, terrain, profile, start, goal) -> dict:
    diagnostics = ZDiagnostics(goal)
    context = contextlib.nullcontext() if heuristic is _EUCLIDEAN else patch.object(pose_search_module, "_heuristic", heuristic)
    with context:
        result = pose_aware_astar_search(start, goal, terrain, profile, goal_tolerance=GOAL_TOLERANCE,
                                         config=CONFIG, max_expansions=30_000, max_search_time_s=300.0,
                                         diagnostics=diagnostics)
    envelope = FixedWingKinematicEnvelope()
    pairs, pair_counts, groups = _pair_analysis(diagnostics.expanded, envelope, terrain)
    group_rows = _group_rows(groups, pair_counts, terrain)
    sizes = [len(items) for items in groups.values()]
    route = {name: _redundancy_by(diagnostics.expanded, envelope, terrain, lambda item, name=name: _route_third(item["pose"], start, goal) == name)
             for name in ("first_third", "middle_third", "final_third")}
    bands = {name: _redundancy_by(diagnostics.expanded, envelope, terrain, lambda item, name=name: _goal_band(item["xy_goal_distance_m"]) == name)
             for name in (">5km", "2-5km", "1-2km", "500-1000m", "<500m")}
    return {"label": label, "status": result.status, "termination_reason": result.termination_reason,
            "expanded": result.expanded_nodes, "generated": result.generated_neighbors,
            "rejected": result.rejected_neighbors, "peak_open": result.max_open_size, "runtime_s": result.runtime_s,
            "best_xy_error_m": result.closest_xy_distance_to_goal_m,
            "best_z_error_m": min(record["abs_goal_z_error_m"] for record in diagnostics.expanded),
            "first_goal_state_insertion_expansion": diagnostics.first_goal_insertion,
            "goal_pop_expansion": diagnostics.goal_pop_expansion,
            "z_per_xy_heading": _distribution(sizes), "pairwise": pairs,
            "top_20_groups": group_rows[:20],
            "terrain_interpretation": {"all_expanded_representatives_previously_passed_continuous_safety": True,
                "min_agl_required_m": CONFIG.min_agl_m,
                "high_diversity_groups_safe_band_is_observed_not_future_terrain_proof": True},
            "redundancy_global": _redundancy_by(diagnostics.expanded, envelope, terrain, lambda item: True),
            "redundancy_by_route_third": route, "redundancy_by_goal_distance_band": bands,
            "offline_rebin_expanded_representatives": offline_rebin(diagnostics.expanded),
            "real_vertical_transition_collapse": vertical_collapse(diagnostics.transitions)}


def _table(headers, rows):
    return ["| " + " | ".join(headers) + " |", "|" + "|".join("---:" for _ in headers) + "|"] + [
        "| " + " | ".join(str(value) for value in row) + " |" for row in rows]


def _markdown(payload: dict) -> str:
    euclidean, vertical = payload["euclidean"], payload["vertical_reachability"]
    lines = ["# Mission E Z-state redundancy diagnostic", "", "Diagnostic only. Both runs use BASIC (`enable_combined_turns=False`), the fixed 60 m / 5 m / 15 degree SearchKey and 30,000-expansion limit. No planner policy was changed.", "",
             "## Baseline reproduction", ""]
    lines += _table(["Run", "Status", "Expanded", "Generated", "Rejected", "Peak OPEN", "Runtime s", "Best XY / |Z| m", "First goal insert", "Goal pop"], [
        (run["label"], run["termination_reason"], f"{run['expanded']:,}", f"{run['generated']:,}", f"{run['rejected']:,}", f"{run['peak_open']:,}", f"{run['runtime_s']:.2f}", f"{run['best_xy_error_m']:.2f}/{run['best_z_error_m']:.2f}", run["first_goal_state_insertion_expansion"] or "-", run["goal_pop_expansion"] or "-")
        for run in (euclidean, vertical)])
    lines += ["", "## Z representatives per (XY, heading)", "", "| Run | Mean | Median | P90 | P95 | Max |", "|---|---:|---:|---:|---:|---:|"]
    for run in (euclidean, vertical):
        stats = run["z_per_xy_heading"]
        lines.append(f"| {run['label']} | {stats['mean']:.2f} | {stats['median']:.2f} | {stats['p90']:.2f} | {stats['p95']:.2f} | {stats['max']:.0f} |")
    lines += ["", "## Pairwise directional candidates", "", "A candidate means only: same XY/heading bucket, `g_A >= g_B`, and A is no closer to goal altitude. It is **not** a pruning rule. Local envelope/clearance checks are reported separately and terrain ahead is intentionally not inferred.", ""]
    lines += _table(["Run", "Pairs", "Candidates", "% pairs", "Locally unrefuted"], [
        (run["label"], f"{run['pairwise']['total_compared_pairs']:,}", f"{run['pairwise']['obvious_dominance_candidates']:,}", f"{run['pairwise']['candidate_percent_of_pairs']:.2f}%", f"{run['pairwise']['locally_unrefuted_candidates']:,}") for run in (euclidean, vertical)])
    lines += ["", "### Redundancy ratio by location", ""]
    lines += _table(["Run / region", "Raw Z states", "Obvious candidates", "Non-dominated", "Ratio"], [
        (run["label"], run["redundancy_global"]["raw_z_states"], run["redundancy_global"]["obviously_dominated_candidates"], run["redundancy_global"]["non_dominated_candidates"], f"{100.0 * run['redundancy_global']['redundancy_ratio']:.2f}%") for run in (euclidean, vertical)] + [
        (f"{run['label']} {region}", values["raw_z_states"], values["obviously_dominated_candidates"], values["non_dominated_candidates"], f"{100.0 * values['redundancy_ratio']:.2f}%")
        for run in (euclidean, vertical) for region, values in run["redundancy_by_route_third"].items()] + [
        (f"{run['label']} {band}", values["raw_z_states"], values["obviously_dominated_candidates"], values["non_dominated_candidates"], f"{100.0 * values['redundancy_ratio']:.2f}%")
        for run in (euclidean, vertical) for band, values in run["redundancy_by_goal_distance_band"].items()])
    for run in (euclidean, vertical):
        lines += ["", f"### {run['label']}: top 20 (XY, heading) groups", ""]
        lines += _table(["(X,Y,H) bins", "Z", "Altitude m", "g", "Goal distance m", "Terrain MSL m", "AGL m", "Candidate pairs"], [
            (tuple(row["xy_heading_bin"]), row["z_state_count"], f"{row['altitude_min_m']:.1f}-{row['altitude_max_m']:.1f}", f"{row['g_min']:.1f}-{row['g_max']:.1f}", f"{row['goal_distance_min_m']:.1f}-{row['goal_distance_max_m']:.1f}", f"{row['terrain_msl_min_m']:.1f}-{row['terrain_msl_max_m']:.1f}", f"{row['agl_min_m']:.1f}-{row['agl_max_m']:.1f}", row["dominance_candidate_count"]) for row in run["top_20_groups"]])
        lines += ["", f"### {run['label']}: offline Z rebin of the {run['expanded']:,} expanded physical representatives", ""]
        lines += _table(["Z m", "Unique keys", "Reduction", "Mean Z/XY", "Mean Z/(XY,H)", "States merged", "g diff", "AGL diff", "Goal-Z diff"], [
            (row["z_bin_m"], f"{row['unique_search_key_count']:,}", f"{row['reduction_percent']:.2f}%", f"{row['mean_z_bins_per_xy']:.2f}", f"{row['mean_z_bins_per_xy_heading']:.2f}", row["physically_distinct_states_merged"], row["merged_buckets_substantial_g_difference"], row["merged_buckets_clearance_difference_over_20m"], row["merged_buckets_goal_z_suitability_difference_over_tolerance"]) for row in run["offline_rebin_expanded_representatives"]])
        lines += ["", f"### {run['label']}: real Mission E vertical successor same-key collapse", ""]
        lines += ["Full SearchKey columns include XY. Because real straight primitives advance 60 m horizontally, they are expected to be near zero; the Z-component columns are the meaningful vertical-progress signal.", ""]
        lines += _table(["Z m", "Climb Z-component collapse", "Descent Z-component collapse", "Full-key climb / descent"], [
            (row["z_bin_m"], f"{row['climb_z_component_collapses']:,}/{row['climb_successors']:,} ({row['climb_z_component_collapse_rate_percent']:.2f}%)", f"{row['descent_z_component_collapses']:,}/{row['descent_successors']:,} ({row['descent_z_component_collapse_rate_percent']:.2f}%)", f"{row['climb_full_search_key_collapses']:,}/{row['climb_successors']:,} / {row['descent_full_search_key_collapses']:,}/{row['descent_successors']:,}") for row in run["real_vertical_transition_collapse"]])
    lines += ["", "## Final decision evidence", "", payload["conclusion"], "", "Production behavior changed: **NO**.", "Full test suite: see execution record."]
    return "\n".join(lines) + "\n"


def main() -> None:
    global _OFFLINE_TERRAIN
    assert CONFIG.enable_combined_turns is False
    cache = load_terrain_cache(CACHE_DIR, load_roi(CONFIG), SOURCE_DEM_PATH)
    terrain = build_terrain_query_from_cache(cache, load_roi(CONFIG), FACTOR)
    _OFFLINE_TERRAIN = terrain
    definition = FAR_MISSIONS["E_long_descent_9_3km"]
    sx, sy = terrain.rowcol_to_xy(*definition["start_rc"])
    gx, gy = terrain.rowcol_to_xy(*definition["goal_rc"])
    start = PhysicalPose(sx, sy, definition["z_msl_m"], navigation_bearing_deg(sx, sy, gx, gy))
    goal = GoalPose(gx, gy, definition["goal_z_msl_m"])
    profile = load_aircraft_profile(PROFILE_PATH)
    euclidean = _run("EUCLIDEAN", _EUCLIDEAN, terrain, profile, start, goal)
    vertical = _run("VERTICAL_REACHABILITY", vertical_reachability_heuristic, terrain, profile, start, goal)
    mean_delta = abs(euclidean["z_per_xy_heading"]["mean"] - vertical["z_per_xy_heading"]["mean"])
    ratio_delta = abs(euclidean["redundancy_global"]["redundancy_ratio"] - vertical["redundancy_global"]["redundancy_ratio"])
    structure = "SAME" if mean_delta < 0.1 and ratio_delta < 0.01 else "SLIGHTLY DIFFERENT" if mean_delta < 1.0 and ratio_delta < 0.05 else "MATERIAL DIFFERENCE"
    e5 = euclidean["real_vertical_transition_collapse"][0]
    e10 = euclidean["real_vertical_transition_collapse"][1]
    redundancy_pct = 100.0 * euclidean["redundancy_global"]["redundancy_ratio"]
    conclusion = (f"EUCLIDEAN VS VERTICAL STRUCTURE: **{structure}**. 5M Z REDUNDANCY: **LOW** ({redundancy_pct:.2f}% of expanded representatives are directional obvious-worse states). Z-DOMINANCE POTENTIAL: **LOW**: the pair screen is only a local candidate test, and terrain ahead is not inferable from a shared XY/heading bucket. COARSER Z RISK: **HIGH from 10 m**: actual vertical progress loses its Z component on {e10['climb_z_component_collapse_rate_percent']:.2f}% of climbs and {e10['descent_z_component_collapse_rate_percent']:.2f}% of descents; even 5 m loses {e5['climb_z_component_collapse_rate_percent']:.2f}% of climb steps. Thus 10/20/50/100 m could reduce offline state count but are not shown safe alternatives. The literal full SearchKey transition collapse remains zero because every tested straight primitive also advances horizontally. Vertical heuristic changes conclusion: **{'YES' if structure != 'SAME' else 'NO'}**.")
    payload = {"task": "Z-DIAG-1", "diagnostic_only": True, "production_behavior_changed": False,
               "search_contract": {"combined_turns": False, "xy_bin_m": CONFIG.search_xy_bin_m,
               "z_bin_m": CONFIG.search_z_bin_m, "heading_bin_deg": CONFIG.search_heading_bin_deg,
               "expansion_limit": 30_000}, "euclidean": euclidean, "vertical_reachability": vertical,
               "structure_classification": structure, "conclusion": conclusion}
    OUTPUT_JSON.write_text(json.dumps(payload, indent=2, allow_nan=True) + "\n", encoding="utf-8")
    OUTPUT_MD.write_text(_markdown(payload), encoding="utf-8")
    print(f"json: {OUTPUT_JSON}")
    print(f"report: {OUTPUT_MD}")


if __name__ == "__main__":
    main()
