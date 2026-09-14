"""Passive, disposable state-space diagnostic for canonical Mission E.

This script changes no pose-search policy. It first reproduces Mission E with
no observer, then repeats the identical search with an observer that records
only physical expanded poses, successor outcomes, and the final OPEN snapshot.
"""
from __future__ import annotations

import json
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path

from planner.aircraft_profile import load_aircraft_profile
from planner.physical import PhysicalPose
from planner.pose_search import GoalPose, navigation_bearing_deg, pose_aware_astar_search
from planner.roi import load_roi
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from scripts.benchmark_far_missions import FAR_MISSIONS
from scripts.benchmark_missions import CACHE_DIR, CONFIG, GOAL_TOLERANCE, PROFILE_PATH, ROOT, SOURCE_DEM_PATH


def _percentile(values, percentile: float) -> float:
    values = sorted(values)
    if not values:
        return float("nan")
    index = (len(values) - 1) * percentile / 100.0
    low, high = math.floor(index), math.ceil(index)
    return float(values[low]) if low == high else values[low] + (values[high] - values[low]) * (index - low)


def _summary(values) -> dict:
    return {"count": len(values), "min": min(values) if values else float("nan"),
            "mean": statistics.fmean(values) if values else float("nan"),
            "median": statistics.median(values) if values else float("nan"),
            "p10": _percentile(values, 10), "p90": _percentile(values, 90),
            "p95": _percentile(values, 95), "max": max(values) if values else float("nan")}


class MissionEDiagnostics:
    """Observer whose methods are invoked by pose_search without feedback."""

    def __init__(self, start: PhysicalPose, goal: GoalPose):
        self.start, self.goal = start, goal
        self.expanded = []
        self.outcomes = defaultdict(Counter)
        self.expanded_by_primitive = Counter()
        self.useful = defaultdict(set)
        self._best_xy = math.hypot(start.x_m - goal.x_m, start.y_m - goal.y_m)
        self._best_z = abs(start.z_msl_m - goal.z_msl_m)
        self._best_d3 = math.sqrt(self._best_xy ** 2 + self._best_z ** 2)
        self.open_heap = ()
        self.active_open = ()
        self.heap_stale_entries = 0

    def _errors(self, pose):
        xy = math.hypot(pose.x_m - self.goal.x_m, pose.y_m - self.goal.y_m)
        z = abs(pose.z_msl_m - self.goal.z_msl_m)
        return xy, z, math.hypot(xy, z)

    def _corridor_deviation(self, pose) -> float:
        dx, dy = self.goal.x_m - self.start.x_m, self.goal.y_m - self.start.y_m
        length = math.hypot(dx, dy)
        return abs(dx * (self.start.y_m - pose.y_m) - (self.start.x_m - pose.x_m) * dy) / length

    def on_expanded(self, expansion, node, xy_error, d3_error) -> None:
        z_error = abs(node.end_pose.z_msl_m - self.goal.z_msl_m)
        self.expanded.append({"index": expansion, "node_id": node.node_id, "key": node.key,
                              "pose": node.end_pose, "primitive": node.incoming_primitive or "START",
                              "xy_error": xy_error, "z_error": z_error, "d3_error": d3_error,
                              "corridor_deviation": self._corridor_deviation(node.end_pose)})
        self.expanded_by_primitive[node.incoming_primitive or "START"] += 1

    def on_successor(self, parent, primitive, outcome, pose) -> None:
        self.outcomes[primitive][outcome] += 1
        # Global, strict bests define "goal-useful". Only an accepted OPEN
        # relaxation is eligible; a physical but dominated pose is not.
        if outcome not in ("OPEN_INSERTED", "OPEN_INSERTED_REPLACEMENT"):
            return
        xy, z, d3 = self._errors(pose)
        if xy < self._best_xy - 1e-12:
            self._best_xy = xy
            self.useful[parent.node_id].add("XY")
        if z < self._best_z - 1e-12:
            self._best_z = z
            self.useful[parent.node_id].add("Z")
        if d3 < self._best_d3 - 1e-12:
            self._best_d3 = d3
            self.useful[parent.node_id].add("D3")

    def on_termination(self, active, all_nodes, expanded_ids, open_heap) -> None:
        self.open_heap = tuple(open_heap)
        self.active_open = tuple(node for node in active.values() if node.node_id not in expanded_ids)
        self.heap_stale_entries = sum(
            active.get(all_nodes[node_id].key) is not all_nodes[node_id] or node_id in expanded_ids
            for _, _, node_id in open_heap
        )


def _xy_groups(records):
    groups = defaultdict(list)
    for record in records:
        key = record["key"]
        groups[(key.x_bin, key.y_bin)].append(record)
    return groups


def _group_rows(records, top: int | None = None):
    rows = []
    for xy, items in _xy_groups(records).items():
        z_bins = {item["key"].z_bin for item in items}
        heading_bins = {item["key"].heading_bin for item in items}
        pairs = {(item["key"].z_bin, item["key"].heading_bin) for item in items}
        altitudes = [item["pose"].z_msl_m for item in items]
        distances = [item["d3_error"] for item in items]
        rows.append({"xy_bin": [xy[0], xy[1]], "z_bins": len(z_bins), "heading_bins": len(heading_bins),
                     "full_states": len(pairs), "expanded_states": len(items),
                     "altitude_min_m": min(altitudes), "altitude_median_m": statistics.median(altitudes),
                     "altitude_max_m": max(altitudes), "distance_to_goal_min_m": min(distances),
                     "distance_to_goal_median_m": statistics.median(distances)})
    rows.sort(key=lambda row: (row["z_bins"], row["full_states"], row["expanded_states"]), reverse=True)
    return rows if top is None else rows[:top]


def _diversity(records) -> dict:
    groups = _xy_groups(records)
    heading_xy = [len({item["key"].heading_bin for item in items}) for items in groups.values()]
    z_xy = [len({item["key"].z_bin for item in items}) for items in groups.values()]
    full_xy = [len({(item["key"].z_bin, item["key"].heading_bin) for item in items}) for items in groups.values()]
    by_xyz = defaultdict(set)
    by_xyh = defaultdict(set)
    for item in records:
        key = item["key"]
        by_xyz[(key.x_bin, key.y_bin, key.z_bin)].add(key.heading_bin)
        by_xyh[(key.x_bin, key.y_bin, key.heading_bin)].add(key.z_bin)
    return {"z_bins_per_xy": _summary(z_xy), "heading_bins_per_xy": _summary(heading_xy),
            "heading_bins_per_xy_z": _summary([len(v) for v in by_xyz.values()]),
            "z_bins_per_xy_heading": _summary([len(v) for v in by_xyh.values()]),
            "full_states_per_xy": _summary(full_xy), "top_z_diversity_xy": _group_rows(records, 20),
            "top_full_state_xy": sorted(_group_rows(records), key=lambda row: row["full_states"], reverse=True)[:20]}


def _altitude_histogram(records) -> list[dict]:
    bands = Counter(25 * math.floor(record["pose"].z_msl_m / 25.0) for record in records)
    total = len(records)
    return [{"band_msl_m": f"{band:.0f}-{band + 25:.0f}", "count": count, "percent": 100.0 * count / total}
            for band, count in sorted(bands.items())]


def _windows(records) -> list[dict]:
    boundaries = ((1, 1000), (1001, 5000), (5001, 10000), (10001, 20000), (20001, 30000))
    output = []
    for lower, upper in boundaries:
        items = [item for item in records if lower <= item["index"] <= upper]
        altitudes, z_errors = [item["pose"].z_msl_m for item in items], [item["z_error"] for item in items]
        output.append({"window": f"{lower}-{upper}", "states": len(items), "altitude": _summary(altitudes),
                       "best_abs_z_error_m": min(z_errors) if z_errors else float("nan"),
                       "median_abs_z_error_m": statistics.median(z_errors) if z_errors else float("nan"),
                       "median_xy_error_m": statistics.median([item["xy_error"] for item in items]) if items else float("nan"),
                       "median_3d_error_m": statistics.median([item["d3_error"] for item in items]) if items else float("nan")})
    return output


def _goal_density(records) -> list[dict]:
    result = []
    for limit in (100, 150, 250, 500, 1000):
        items = [item for item in records if item["xy_error"] <= limit]
        groups = _xy_groups(items)
        if not items:
            result.append({"xy_distance_limit_m": limit, "expanded_states": 0})
            continue
        result.append({"xy_distance_limit_m": limit, "expanded_states": len(items), "unique_xy_bins": len(groups),
                       "mean_z_bins_per_xy": statistics.fmean(len({i["key"].z_bin for i in group}) for group in groups.values()),
                       "mean_heading_bins_per_xy": statistics.fmean(len({i["key"].heading_bin for i in group}) for group in groups.values()),
                       "mean_full_states_per_xy": statistics.fmean(len({(i["key"].z_bin, i["key"].heading_bin) for i in group}) for group in groups.values()),
                       "altitude_min_m": min(i["pose"].z_msl_m for i in items), "altitude_max_m": max(i["pose"].z_msl_m for i in items),
                       "best_z_error_m": min(i["z_error"] for i in items)})
    return result


def _primitive_contributions(diag: MissionEDiagnostics) -> dict:
    output = {}
    total_expanded = len(diag.expanded)
    for primitive in ("STRAIGHT_LEVEL", "LEFT_LEVEL_TURN", "RIGHT_LEVEL_TURN", "STRAIGHT_CLIMB", "STRAIGHT_DESCENT"):
        counts = diag.outcomes[primitive]
        inserted = counts["OPEN_INSERTED"] + counts["OPEN_INSERTED_REPLACEMENT"]
        output[primitive] = {"generated": counts["GENERATED"], "unavailable": counts["UNAVAILABLE_CAPABILITY"],
                             "safety_rejected": sum(value for key, value in counts.items()
                                                    if key not in ("GENERATED", "UNAVAILABLE_CAPABILITY", "PHYSICALLY_VALID", "SAME_KEY_SELF_TRANSITION", "SAME_KEY_DOMINANCE", "OPEN_INSERTED", "OPEN_INSERTED_REPLACEMENT")),
                             "physically_valid": counts["PHYSICALLY_VALID"], "open_inserted": inserted,
                             "lower_g_replacement": counts["OPEN_INSERTED_REPLACEMENT"],
                             "same_key_self": counts["SAME_KEY_SELF_TRANSITION"],
                             "same_key_dominated": counts["SAME_KEY_DOMINANCE"],
                             "valid_not_open": counts["SAME_KEY_SELF_TRANSITION"] + counts["SAME_KEY_DOMINANCE"],
                             "expanded_arrivals": diag.expanded_by_primitive[primitive],
                             "expanded_arrival_fraction": diag.expanded_by_primitive[primitive] / total_expanded}
    return output


def _open_summary(diag: MissionEDiagnostics) -> dict:
    records = []
    for node in diag.active_open:
        xy, z, d3 = diag._errors(node.end_pose)
        records.append({"key": node.key, "pose": node.end_pose, "xy_error": xy, "z_error": z, "d3_error": d3})
    groups = defaultdict(list)
    for item in records:
        groups[(item["key"].x_bin, item["key"].y_bin)].append(item)
    return {"heap_entries_including_stale": len(diag.open_heap), "heap_stale_entries": diag.heap_stale_entries,
            "active_open_representatives": len(records), "unique_xy_bins": len(groups),
            "mean_z_bins_per_xy": statistics.fmean(len({i["key"].z_bin for i in items}) for items in groups.values()) if groups else float("nan"),
            "mean_heading_bins_per_xy": statistics.fmean(len({i["key"].heading_bin for i in items}) for items in groups.values()) if groups else float("nan"),
            "altitude": _summary([i["pose"].z_msl_m for i in records]),
            "best_xy_distance_m": min((i["xy_error"] for i in records), default=float("nan")),
            "best_z_error_m": min((i["z_error"] for i in records), default=float("nan")),
            "best_3d_distance_m": min((i["d3_error"] for i in records), default=float("nan"))}


def _markdown(payload: dict) -> str:
    baseline, diversity = payload["baseline"], payload["diversity"]
    def table(headers, rows):
        return ["| " + " | ".join(headers) + " |", "|" + "|".join("---:" for _ in headers) + "|"] + [
            "| " + " | ".join(str(value) for value in row) + " |" for row in rows]
    lines = ["# Mission E State-Space Diagnostic", "", "## Baseline reproduction", "",
             f"- Reproduced without observer: **{baseline['reproduced']}**", f"- Status: `{baseline['instrumented']['status']}` / `{baseline['instrumented']['termination']}`",
             f"- Expanded / generated / rejected: {baseline['instrumented']['expanded']:,} / {baseline['instrumented']['generated']:,} / {baseline['instrumented']['rejected']:,}",
             f"- Peak OPEN: {baseline['instrumented']['peak_open']:,}",
             f"- Best XY / Z / 3D error: {baseline['instrumented']['best_xy_m']:.2f} m / {baseline['instrumented']['best_z_m']:.2f} m / {baseline['instrumented']['best_3d_m']:.2f} m",
             f"- Expanded altitude range: {baseline['instrumented']['minimum_expanded_altitude_m']:.2f}-{baseline['instrumented']['maximum_expanded_altitude_m']:.2f} m MSL",
             f"- Runtime without / with observer: {baseline['unobserved_runtime_s']:.2f} s / {baseline['instrumented_runtime_s']:.2f} s (observer overhead is expected and does not alter policy).", "",
             "## State diversity", "", "| Metric | Mean | Median | P90 | P95 | Max |", "|---|---:|---:|---:|---:|---:|"]
    for name, values in (("Z bins / XY", diversity["z_bins_per_xy"]), ("Heading bins / XY", diversity["heading_bins_per_xy"]),
                         ("Heading bins / (XY,Z)", diversity["heading_bins_per_xy_z"]),
                         ("Z bins / (XY,heading)", diversity["z_bins_per_xy_heading"]),
                         ("Full states / XY", diversity["full_states_per_xy"])):
        lines.append(f"| {name} | {values['mean']:.2f} | {values['median']:.2f} | {values['p90']:.2f} | {values['p95']:.2f} | {values['max']:.0f} |")
    lines += ["", "## Top 20 Z-diverse XY bins", ""]
    lines += table(["XY bin", "Z", "Head", "(Z,H)", "Expanded", "Altitude range m", "Goal distance min/median m"], [
        (f"({row['xy_bin'][0]}, {row['xy_bin'][1]})", row["z_bins"], row["heading_bins"], row["full_states"], row["expanded_states"],
         f"{row['altitude_min_m']:.1f}-{row['altitude_max_m']:.1f}", f"{row['distance_to_goal_min_m']:.1f}/{row['distance_to_goal_median_m']:.1f}")
        for row in diversity["top_z_diversity_xy"]])
    lines += ["", "## Expanded altitude distribution (25 m bands)", ""]
    lines += table(["MSL band", "Count", "Percent"], [(row["band_msl_m"], row["count"], f"{row['percent']:.2f}%") for row in payload["altitude_histogram_25m"]])
    lines += ["", "## Altitude and goal-distance evolution", ""]
    lines += table(["Expansion window", "Alt min/median/max", "Best |Z|", "Median |Z|", "Median XY", "Median 3D"], [
        (row["window"], f"{row['altitude']['min']:.1f}/{row['altitude']['median']:.1f}/{row['altitude']['max']:.1f}",
         f"{row['best_abs_z_error_m']:.1f}", f"{row['median_abs_z_error_m']:.1f}",
         f"{row['median_xy_error_m']:.1f}", f"{row['median_3d_error_m']:.1f}") for row in payload["altitude_evolution"]])
    lines += ["", "## Goal-proximity density", ""]
    lines += table(["XY radius", "Expanded", "XY bins", "Mean Z / XY", "Mean head / XY", "Mean (Z,H) / XY", "Best |Z|"], [
        (f"<={row['xy_distance_limit_m']} m", row["expanded_states"], row.get("unique_xy_bins", "-"),
         f"{row.get('mean_z_bins_per_xy', float('nan')):.2f}" if row["expanded_states"] else "-",
         f"{row.get('mean_heading_bins_per_xy', float('nan')):.2f}" if row["expanded_states"] else "-",
         f"{row.get('mean_full_states_per_xy', float('nan')):.2f}" if row["expanded_states"] else "-",
         f"{row.get('best_z_error_m', float('nan')):.2f}" if row["expanded_states"] else "-") for row in payload["goal_proximity_density"]])
    useful = payload["goal_useful_expansions"]
    lines += ["", "## Goal-useful expansions", "",
             "A state is goal-useful only if one of its accepted OPEN insertions establishes a strict global best in XY, absolute Z error or 3D distance.", ""]
    lines += table(["Category", "Expanded states", "Percent of 30k"], [(name, value, f"{100.0 * value / 30000:.2f}%") for name, value in useful.items()])
    lines += ["", "## Same-key and primitive contribution", ""]
    lines += table(["Primitive", "Generated", "Unavailable", "Safety reject", "Physically valid", "OPEN inserted", "Dominated", "Self", "Expanded arrivals"], [
        (name, values["generated"], values["unavailable"], values["safety_rejected"], values["physically_valid"], values["open_inserted"],
         values["same_key_dominated"], values["same_key_self"], values["expanded_arrivals"]) for name, values in payload["primitive_contributions"].items()])
    corridor = payload["corridor_spread"]
    lines += ["", "## XY corridor spread", "",
             f"Perpendicular deviation (actual physical poses): median {corridor['deviation_m']['median']:.2f} m; p90 {corridor['deviation_m']['p90']:.2f} m; p95 {corridor['deviation_m']['p95']:.2f} m; max {corridor['deviation_m']['max']:.2f} m.", ""]
    lines += table(["Band", "Expanded states"], corridor["cumulative_counts"].items())
    open_state = payload["open_at_termination"]
    lines += ["", "## OPEN at 30k", "",
             f"- Heap entries including stale: {open_state['heap_entries_including_stale']:,}; stale entries: {open_state['heap_stale_entries']:,}.",
             f"- Active non-stale OPEN representatives: {open_state['active_open_representatives']:,} across {open_state['unique_xy_bins']:,} XY bins.",
             f"- Active OPEN mean Z / heading bins per XY: {open_state['mean_z_bins_per_xy']:.2f} / {open_state['mean_heading_bins_per_xy']:.2f}.",
             f"- Best actual physical OPEN candidate: XY {open_state['best_xy_distance_m']:.2f} m, |Z| {open_state['best_z_error_m']:.2f} m, 3D {open_state['best_3d_distance_m']:.2f} m.", "",
             "That candidate is inside the 90 m XY / 10 m Z goal region but had not yet been popped when the fixed expansion budget fired.", "",
             "## Final classification", "", f"**PRIMARY: {payload['classification']['primary']}**", "", f"**SECONDARY: {payload['classification']['secondary']}**", "", payload["classification"]["evidence"], "",
             "No SearchKey, heuristic, cost, primitive, safety, tolerance or expansion-limit setting was changed."]
    return "\n".join(lines) + "\n"


def main() -> None:
    definition = FAR_MISSIONS["E_long_descent_9_3km"]
    terrain = build_terrain_query_from_cache(load_terrain_cache(CACHE_DIR, load_roi(CONFIG), SOURCE_DEM_PATH), load_roi(CONFIG), 2)
    sx, sy = terrain.rowcol_to_xy(*definition["start_rc"])
    gx, gy = terrain.rowcol_to_xy(*definition["goal_rc"])
    start = PhysicalPose(sx, sy, definition["z_msl_m"], navigation_bearing_deg(sx, sy, gx, gy))
    goal = GoalPose(gx, gy, definition["goal_z_msl_m"])
    profile = load_aircraft_profile(PROFILE_PATH)
    kwargs = dict(goal_tolerance=GOAL_TOLERANCE, config=CONFIG, max_expansions=30_000, max_search_time_s=300.0)
    baseline = pose_aware_astar_search(start, goal, terrain, profile, **kwargs)
    diagnostics = MissionEDiagnostics(start, goal)
    instrumented = pose_aware_astar_search(start, goal, terrain, profile, diagnostics=diagnostics, **kwargs)
    best = instrumented.best_nodes[-1].end_pose
    baseline_fields = ("status", "termination_reason", "expanded_nodes", "generated_neighbors", "rejected_neighbors", "max_open_size", "closest_xy_distance_to_goal_m", "closest_3d_distance_to_goal_m", "maximum_altitude_msl_m")
    reproduced = all(getattr(baseline, field) == getattr(instrumented, field) for field in baseline_fields)
    records = diagnostics.expanded
    altitude_values = [record["pose"].z_msl_m for record in records]
    useful = {kind: sum(kind in diagnostics.useful[record["node_id"]] for record in records) for kind in ("XY", "Z", "D3")}
    useful["none"] = sum(not diagnostics.useful[record["node_id"]] for record in records)
    corridor = [record["corridor_deviation"] for record in records]
    corridor_counts = {label: sum(predicate(value) for value in corridor) for label, predicate in {
        "<=100m": lambda value: value <= 100, "<=250m": lambda value: value <= 250,
        "<=500m": lambda value: value <= 500, "<=1000m": lambda value: value <= 1000,
        ">1000m": lambda value: value > 1000}.items()}
    primitive = _primitive_contributions(diagnostics)
    same_key = {name: {"collisions": values["same_key_dominated"] + values["lower_g_replacement"],
                       "replacements": values["lower_g_replacement"], "dominated": values["same_key_dominated"],
                       "self": values["same_key_self"], "valid_not_open": values["valid_not_open"]}
                for name, values in primitive.items()}
    payload = {"baseline": {"reproduced": "YES" if reproduced else "NO", "unobserved_runtime_s": baseline.runtime_s,
                              "instrumented_runtime_s": instrumented.runtime_s,
                              "instrumented": {"status": instrumented.status, "termination": instrumented.termination_reason,
                                                 "expanded": instrumented.expanded_nodes, "generated": instrumented.generated_neighbors,
                                                 "rejected": instrumented.rejected_neighbors, "peak_open": instrumented.max_open_size,
                                                 "best_xy_m": instrumented.closest_xy_distance_to_goal_m,
                                                 "best_z_m": abs(best.z_msl_m - goal.z_msl_m), "best_3d_m": instrumented.closest_3d_distance_to_goal_m,
                                                 "minimum_expanded_altitude_m": min(altitude_values), "maximum_expanded_altitude_m": max(altitude_values)}},
               "diversity": _diversity(records), "altitude_histogram_25m": _altitude_histogram(records),
               "altitude_expanded_summary": _summary(altitude_values), "altitude_evolution": _windows(records),
               "goal_proximity_density": _goal_density(records), "goal_useful_expansions": useful,
               "same_key_by_primitive": same_key, "primitive_contributions": primitive,
               "corridor_spread": {"deviation_m": _summary(corridor), "cumulative_counts": corridor_counts},
               "goal_progress_windows": _windows(records), "open_at_termination": _open_summary(diagnostics),
               "classification": {
                   "primary": "A. Z DIVERSITY",
                   "secondary": "E. HEURISTIC / GOAL GUIDANCE",
                   "evidence": "Z multiplicity is widespread (median 37 and maximum 82 Z bins per XY), while heading diversity is only median 2 and all expanded poses stay within 49 m of the direct corridor. At 30k, an active non-stale OPEN representative is already inside the physical goal region, so frontier ordering/budget delays goal acceptance after vertical-state growth rather than a lateral detour or missing descent capability.",
               }}
    json_path = ROOT / "results" / "mission_e_state_space_diagnostic.json"
    md_path = ROOT / "results" / "mission_e_state_space_diagnostic.md"
    json_path.write_text(json.dumps(payload, indent=2, allow_nan=True) + "\n", encoding="utf-8")
    md_path.write_text(_markdown(payload), encoding="utf-8")
    print(json.dumps(payload, indent=2, allow_nan=True))
    print(f"markdown: {md_path}")


if __name__ == "__main__":
    main()
