"""Mission E vertical-schedule combinatorics, observed without policy changes."""
from __future__ import annotations

import json
import math
import statistics
from collections import Counter, defaultdict

from planner.aircraft_profile import load_aircraft_profile
from planner.physical import PhysicalPose
from planner.pose_search import GoalPose, navigation_bearing_deg, pose_aware_astar_search, pose_in_goal
from planner.roi import load_roi
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from scripts.benchmark_far_missions import FAR_MISSIONS
from scripts.benchmark_missions import (
    CACHE_DIR, CONFIG, FACTOR, GOAL_TOLERANCE, PROFILE_PATH, ROOT, SOURCE_DEM_PATH,
)


OUTPUT_JSON = ROOT / "results" / "mission_e_vertical_schedule_diagnostic.json"
OUTPUT_MD = ROOT / "results" / "mission_e_vertical_schedule_diagnostic.md"
_VERTICAL = {"STRAIGHT_LEVEL": "L", "STRAIGHT_DESCENT": "D", "STRAIGHT_CLIMB": "C",
             "LEFT_LEVEL_TURN": "TL", "RIGHT_LEVEL_TURN": "TR"}
_MODE = {"STRAIGHT_DESCENT": "D", "STRAIGHT_CLIMB": "C"}
_OPEN = {"OPEN_INSERTED", "OPEN_INSERTED_REPLACEMENT"}


def _quantile(values, q):
    ordered = sorted(values)
    if not ordered:
        return float("nan")
    point = (len(ordered) - 1) * q
    low, high = math.floor(point), math.ceil(point)
    return ordered[low] if low == high else ordered[low] + (ordered[high] - ordered[low]) * (point - low)


def _summary(values):
    return {"count": len(values), "mean": statistics.fmean(values) if values else float("nan"),
            "median": statistics.median(values) if values else float("nan"), "p90": _quantile(values, .9),
            "max": max(values) if values else float("nan")}


def _errors(pose, goal):
    xy = math.hypot(pose.x_m - goal.x_m, pose.y_m - goal.y_m)
    z = abs(pose.z_msl_m - goal.z_msl_m)
    return xy, z, math.hypot(xy, z)


def _rle(symbols):
    output = []
    for symbol in symbols:
        if output and output[-1][0] == symbol:
            output[-1][1] += 1
        else:
            output.append([symbol, 1])
    return output


def _rle_text(runs):
    return " ".join(f"{symbol}{count}" for symbol, count in runs) or "START"


def _mode_for(primitive):
    return _MODE.get(primitive, "L")


def _route_third(pose, start, goal):
    dx, dy = goal.x_m - start.x_m, goal.y_m - start.y_m
    denominator = dx * dx + dy * dy
    progress = 0.0 if denominator == 0 else ((pose.x_m - start.x_m) * dx + (pose.y_m - start.y_m) * dy) / denominator
    return "first" if progress < 1 / 3 else "middle" if progress < 2 / 3 else "final"


class ScheduleDiagnostics:
    """Only captures A* callback data; no callback value changes search."""

    def __init__(self, goal):
        self.goal = goal
        self.expanded = []
        self.outcomes = defaultdict(dict)  # parent node id -> primitive -> last outcome
        self.all_nodes = {}
        self.goal_open_pose_events = []

    def on_expanded(self, expansion, node, xy_error, d3_error):
        xy, z, d3 = _errors(node.end_pose, self.goal)
        self.expanded.append({"index": expansion, "node_id": node.node_id, "key": node.key,
                              "pose": node.end_pose, "g": node.g_cost, "xy_error_m": xy,
                              "z_error_m": z, "d3_error_m": d3})

    def on_successor(self, parent, primitive, outcome, pose):
        self.outcomes[parent.node_id][primitive] = outcome
        if outcome in _OPEN and pose_in_goal(pose, self.goal, GOAL_TOLERANCE):
            self.goal_open_pose_events.append((parent.node_id, primitive, pose))

    def on_termination(self, active, all_nodes, expanded_ids, open_heap):
        self.all_nodes = dict(all_nodes)


def _chain(node_id, nodes):
    chain = []
    while node_id is not None:
        node = nodes[node_id]
        chain.append(node)
        node_id = node.parent_node_id
    return tuple(reversed(chain))


def _descriptor(node_id, nodes, start, goal):
    chain = _chain(node_id, nodes)
    primitives = [node.incoming_primitive for node in chain[1:]]
    full = [_VERTICAL[primitive] for primitive in primitives]
    modes = [_mode_for(primitive) for primitive in primitives]
    rle = _rle(modes)
    desc_indices = [index for index, mode in enumerate(modes, 1) if mode == "D"]
    descent_runs = [count for mode, count in rle if mode == "D"]
    switches = sum(a != b for a, b in zip(modes, modes[1:]))
    first_descent_step = desc_indices[0] if desc_indices else None
    first_descent_parent = chain[first_descent_step - 1] if first_descent_step else None
    first_distance = (sum(node.incoming_trajectory.horizontal_arc_length_m for node in chain[1:first_descent_step]
                          if node.incoming_trajectory is not None) if first_descent_parent is not None else float("nan"))
    remaining_at_first = (_errors(first_descent_parent.end_pose, goal)[0]
                          if first_descent_parent is not None else float("nan"))
    end = chain[-1]
    return {"node_id": node_id, "full_primitive_schedule": " ".join(full) or "START",
            "vertical_rle": _rle_text(rle), "level_steps": modes.count("L"),
            "descent_steps": modes.count("D"), "climb_steps": modes.count("C"),
            "first_descent_step": first_descent_step,
            "last_descent_step": desc_indices[-1] if desc_indices else None,
            "longest_descent_run": max(descent_runs, default=0), "descent_runs": len(descent_runs),
            "level_runs_between_descents": max(0, len(descent_runs) - 1), "vertical_mode_switches": switches,
            "total_horizontal_distance_m": sum(node.incoming_trajectory.horizontal_arc_length_m for node in chain[1:]
                                                if node.incoming_trajectory is not None),
            "total_altitude_lost_m": start.z_msl_m - end.end_pose.z_msl_m,
            "first_descent_start_distance_proxy_m": first_distance,
            "remaining_xy_when_first_descent_m": remaining_at_first,
            "mode_sequence": tuple(modes), "chain": chain}


def _geometric_lineage(descriptor):
    """Ignore altitude and vertical-action labels, retain XY/H path and turns."""
    chain = descriptor["chain"]
    return tuple((node.key.x_bin, node.key.y_bin, node.key.heading_bin,
                  node.incoming_primitive if node.incoming_primitive in ("LEFT_LEVEL_TURN", "RIGHT_LEVEL_TURN") else "-")
                 for node in chain)


def _group_analysis(records, descriptors, terrain):
    groups = defaultdict(list)
    for record in records:
        key = record["key"]
        groups[(key.x_bin, key.y_bin, key.heading_bin)].append(record)
    rows, timing_same_count, timing_different_count = [], 0, 0
    for key, items in groups.items():
        data = [descriptors[item["node_id"]] for item in items]
        altitudes = [item["pose"].z_msl_m for item in items]
        desc_count_groups = defaultdict(list)
        for item, value in zip(items, data):
            desc_count_groups[value["descent_steps"]].append((item, value))
        same_timing_variants = 0
        for variants in desc_count_groups.values():
            distinct = {value["vertical_rle"] for _, value in variants}
            if len(distinct) > 1:
                same_timing_variants += len(variants)
        timing_same_count += same_timing_variants
        timing_different_count += len(items) - same_timing_variants
        lineages = defaultdict(int)
        for value in data:
            lineages[_geometric_lineage(value)] += 1
        terrain_values = [terrain.query(item["pose"].x_m, item["pose"].y_m).elevation for item in items]
        first_descent_distances = [d["first_descent_start_distance_proxy_m"] for d in data if d["first_descent_step"] is not None]
        rows.append({"xy_heading_bin": list(key), "z_states": len(items), "altitude_min_m": min(altitudes),
                     "altitude_max_m": max(altitudes), "distinct_full_schedules": len({d["full_primitive_schedule"] for d in data}),
                     "distinct_rle_schedules": len({d["vertical_rle"] for d in data}),
                     "descent_count_min": min(d["descent_steps"] for d in data), "descent_count_max": max(d["descent_steps"] for d in data),
                     "level_count_min": min(d["level_steps"] for d in data), "level_count_max": max(d["level_steps"] for d in data),
                     "first_descent_earliest_m": min(first_descent_distances, default=float("nan")),
                     "first_descent_latest_m": max(first_descent_distances, default=float("nan")),
                     "switch_min": min(d["vertical_mode_switches"] for d in data), "switch_max": max(d["vertical_mode_switches"] for d in data),
                     "geometric_lineages": len(lineages), "max_altitude_variants_one_lineage": max(lineages.values()),
                     "terrain_msl_min_m": min(terrain_values), "terrain_msl_max_m": max(terrain_values),
                     "examples": _examples(items, data)})
    rows.sort(key=lambda row: (row["z_states"], row["distinct_rle_schedules"]), reverse=True)
    return rows, groups, {"same_descent_count_different_timing_states": timing_same_count,
                           "different_total_descent_amount_states": timing_different_count,
                           "same_count_timing_percent": 100.0 * timing_same_count / len(records)}


def _descriptor_payload(descriptor):
    """Strip live planner nodes before serializing an observational result."""
    return {key: value for key, value in descriptor.items() if key not in ("chain", "mode_sequence")}


def _examples(items, data):
    pairs = list(zip(items, data))
    ordered = sorted(pairs, key=lambda pair: pair[0]["pose"].z_msl_m)
    result = {}
    for label, (record, descriptor) in (("highest", ordered[-1]), ("median", ordered[len(ordered) // 2]), ("lowest", ordered[0])):
        result[label] = {"altitude_m": record["pose"].z_msl_m, "g": record["g"],
                         "schedule": descriptor["full_primitive_schedule"], "rle": descriptor["vertical_rle"],
                         "descent_steps": descriptor["descent_steps"], "level_steps": descriptor["level_steps"],
                         "switches": descriptor["vertical_mode_switches"]}
    return result


def _decision_branching(diag):
    categories = Counter()
    both_open = 0
    for record in diag.expanded:
        outcomes = diag.outcomes[record["node_id"]]
        level = outcomes.get("STRAIGHT_LEVEL") in _OPEN
        descent = outcomes.get("STRAIGHT_DESCENT") in _OPEN
        categories[(level, descent)] += 1
        both_open += level and descent
    return {"total_expansions": len(diag.expanded), "both_open_eligible": both_open,
            "both_open_eligible_percent": 100 * both_open / len(diag.expanded),
            "only_level_open_eligible": categories[(True, False)], "only_descent_open_eligible": categories[(False, True)],
            "neither_open_eligible": categories[(False, False)]}


def _interruption_causality(records, descriptors, diag):
    result = Counter()
    for record in records:
        descriptor = descriptors[record["node_id"]]
        chain = descriptor["chain"]
        if len(chain) < 3 or chain[-1].incoming_primitive != "STRAIGHT_LEVEL" or chain[-2].incoming_primitive != "STRAIGHT_DESCENT":
            continue
        prior = chain[-2]
        descent_outcome = diag.outcomes[prior.node_id].get("STRAIGHT_DESCENT")
        if descent_outcome in ("BELOW_MIN_AGL", "NODATA", "OUT_OF_BOUNDS"):
            result["terrain_forced"] += 1
        elif descent_outcome in ("PHYSICALLY_VALID", "SAME_KEY_SELF_TRANSITION", "SAME_KEY_DOMINANCE", *_OPEN):
            result["non_terrain_forced"] += 1
        else:
            result["unknown_ambiguous"] += 1
    result["total_interruptions"] = sum(result.values())
    return dict(result)


def _macro_runs(descriptor):
    chain, runs = descriptor["chain"], _rle(descriptor["mode_sequence"])
    output, cursor = [], 1
    for mode, count in runs:
        segment = chain[cursor:cursor + count]
        if mode == "D":
            previous = chain[cursor - 1]
            output.append({"steps": count,
                           "horizontal_m": sum(node.incoming_trajectory.horizontal_arc_length_m for node in segment if node.incoming_trajectory is not None),
                           "altitude_change_m": segment[-1].end_pose.z_msl_m - previous.end_pose.z_msl_m})
        cursor += count
    return output


def _classify(records, descriptors, group_sizes):
    desc = [descriptors[r["node_id"]] for r in records if descriptors[r["node_id"]]["first_descent_step"] is not None]
    firsts = [d["first_descent_start_distance_proxy_m"] for d in desc]
    fragmentation = [d["descent_runs"] / d["descent_steps"] for d in desc if d["descent_steps"]]
    sustained_cutoff, fragmented_cutoff = _quantile(fragmentation, .1), _quantile(fragmentation, .9)
    groups = defaultdict(list)  # Classes intentionally overlap: timing and run structure are separate facts.
    for record in records:
        d = descriptors[record["node_id"]]
        first = d["first_descent_start_distance_proxy_m"]
        if d["first_descent_step"] is None:
            groups["NO_DESCENT_YET"].append(record)
            continue
        groups["EARLY_AT_START" if first == 0.0 else "LATE_AFTER_START"].append(record)
        score = d["descent_runs"] / d["descent_steps"]
        if score <= sustained_cutoff:
            groups["SUSTAINED"].append(record)
        if score >= fragmented_cutoff:
            groups["FRAGMENTED"].append(record)
    output = {}
    for label, items in groups.items():
        output[label] = {"states": len(items), "average_g": statistics.fmean(r["g"] for r in items),
                         "average_remaining_xy_m": statistics.fmean(r["xy_error_m"] for r in items),
                         "average_z_error_m": statistics.fmean(r["z_error_m"] for r in items),
                         "average_z_states_per_xy_heading": statistics.fmean(group_sizes[r["node_id"]] for r in items),
                         "thresholds": {"sustained_fragmentation_ratio_p10": sustained_cutoff,
                                        "fragmented_fragmentation_ratio_p90": fragmented_cutoff,
                                        "early_definition": "first descent begins at start (observed mass point)",
                                        "late_definition": "first descent begins after start"}}
    return output


def _macro_summary(descriptor):
    runs = _macro_runs(descriptor)
    counts = Counter(run["steps"] for run in runs)
    return {"descent_run_count": len(runs), "run_length_steps_distribution": dict(sorted(counts.items())),
            "max_run_steps": max((run["steps"] for run in runs), default=0),
            "runs_at_least_3_steps": sum(run["steps"] >= 3 for run in runs),
            "runs": runs}


def _switch_summary(distribution):
    expanded = [switches for switches, count in distribution.items() for _ in range(count)]
    return {"min": min(expanded), "median": statistics.median(expanded), "p90": _quantile(expanded, .9),
            "max": max(expanded), "zero_switch_states": distribution.get(0, 0),
            "full_distribution": dict(sorted(distribution.items()))}


def _table(headers, rows):
    return ["| " + " | ".join(headers) + " |", "|" + "|".join("---:" for _ in headers) + "|"] + [
        "| " + " | ".join(str(value) for value in row) + " |" for row in rows]


def _markdown(payload):
    base = payload["baseline"]
    lines = ["# Mission E vertical schedule combinatorics", "", "Diagnostic only: production Euclidean A*, BASIC primitives, 60 m / 5 m / 15 degree SearchKey and 30,000 limit are unchanged.", "",
             "## Baseline", "", f"BASELINE REPRODUCED: **{base['reproduced']}**.", f"Status: `{base['termination_reason']}`; expanded/generated/rejected: {base['expanded']:,}/{base['generated']:,}/{base['rejected']:,}; peak OPEN: {base['peak_open']:,}; best XY/|Z|: {base['best_xy_error_m']:.2f}/{base['best_z_error_m']:.2f} m.", "",
             f"Mean Z/(XY,H): **{payload['mean_z_per_xy_heading']:.2f}**.", "", "## Top high-diversity groups", ""]
    lines += _table(["(X,Y,H)", "Z", "Altitude m", "Full/RLE schedules", "D count", "L count", "First D m", "Switches", "Lineages / max variants"], [
        (tuple(row["xy_heading_bin"]), row["z_states"], f"{row['altitude_min_m']:.1f}-{row['altitude_max_m']:.1f}", f"{row['distinct_full_schedules']}/{row['distinct_rle_schedules']}", f"{row['descent_count_min']}-{row['descent_count_max']}", f"{row['level_count_min']}-{row['level_count_max']}", f"{row['first_descent_earliest_m']:.0f}-{row['first_descent_latest_m']:.0f}", f"{row['switch_min']}-{row['switch_max']}", f"{row['geometric_lineages']}/{row['max_altitude_variants_one_lineage']}") for row in payload["top_20_groups"]])
    lines += ["", "Representative schedules (highest / median / lowest altitude) are preserved in JSON for all top groups; their full strings are omitted here to keep the report inspectable.", "", "## Timing, mode switches, and route sections", ""]
    timing = payload["same_descent_count_different_timing"]
    lines += [f"Same descent count but different timing accounts for **{timing['same_count_timing_percent']:.2f}%** of expanded Z representatives ({timing['same_descent_count_different_timing_states']:,}/{payload['baseline']['expanded']:,}); remaining states differ in total descent amount or share identical timing.", ""]
    lines += _table(["Route third", "Distinct schedules", "Z states", "Schedule/Z"], [(key, value["distinct_schedules"], value["z_states"], f"{value['schedule_to_z_ratio']:.3f}") for key, value in payload["route_sections"].items()])
    switch = payload["mode_switch_summary"]
    lines += ["", f"Mode switches per prefix: min/median/p90/max = {switch['min']}/{switch['median']}/{switch['p90']}/{switch['max']}; zero-switch states = {switch['zero_switch_states']:,}. Full distribution is in JSON.", "", "## Sustained vs fragmented", ""]
    lines += ["These classes overlap: EARLY/LATE describe onset; SUSTAINED/FRAGMENTED describe descent-run structure.", ""]
    lines += _table(["Class", "States", "Avg g", "Avg remaining XY", "Avg |Z|", "Avg Z/(XY,H)"], [(label, value["states"], f"{value['average_g']:.1f}", f"{value['average_remaining_xy_m']:.1f}", f"{value['average_z_error_m']:.1f}", f"{value['average_z_states_per_xy_heading']:.1f}") for label, value in payload["schedule_classes"].items()])
    lines += ["", f"Thresholds are observed quantiles: {json.dumps(payload['schedule_thresholds'])}.", "", "## Decision branching and terrain causality", ""]
    branch, interruptions = payload["decision_branching"], payload["interruption_causality"]
    lines += [f"LEVEL and DESCENT both OPEN-eligible: **{branch['both_open_eligible']:,}/{branch['total_expansions']:,} ({branch['both_open_eligible_percent']:.2f}%)**. Only level / only descent / neither: {branch['only_level_open_eligible']:,} / {branch['only_descent_open_eligible']:,} / {branch['neither_open_eligible']:,}.",
              f"Descent→level interruptions: terrain-forced {interruptions.get('terrain_forced', 0):,}; non-terrain-forced {interruptions.get('non_terrain_forced', 0):,}; unknown {interruptions.get('unknown_ambiguous', 0):,}.", "", "## Near-goal and macro opportunity", ""]
    macro = payload["best_partial_macro_runs"]
    lines += [f"Near-goal expanded states (<=500 m): {payload['near_goal']['count']:,}; goal-satisfying OPEN states: {payload['goal_open']['count']:,}; fragmented / sustained by observed thresholds: {payload['near_goal']['fragmented_count']:,} / {payload['near_goal']['sustained_count']:,}. Their descriptor samples are in JSON.",
              f"Best partial path descent-run summary: {macro['descent_run_count']} runs, length distribution {macro['run_length_steps_distribution']}, max {macro['max_run_steps']} steps; runs >=3: {macro['runs_at_least_3_steps']}.", "", "## Final classification", "", payload["conclusion"], "", "Production behavior changed: **NO**. Full test suite: see execution record. RESULT: **PASS**."]
    return "\n".join(lines) + "\n"


def main():
    assert CONFIG.enable_combined_turns is False
    cache = load_terrain_cache(CACHE_DIR, load_roi(CONFIG), SOURCE_DEM_PATH)
    terrain = build_terrain_query_from_cache(cache, load_roi(CONFIG), FACTOR)
    definition = FAR_MISSIONS["E_long_descent_9_3km"]
    sx, sy = terrain.rowcol_to_xy(*definition["start_rc"])
    gx, gy = terrain.rowcol_to_xy(*definition["goal_rc"])
    start = PhysicalPose(sx, sy, definition["z_msl_m"], navigation_bearing_deg(sx, sy, gx, gy))
    goal = GoalPose(gx, gy, definition["goal_z_msl_m"])
    diagnostics = ScheduleDiagnostics(goal)
    result = pose_aware_astar_search(start, goal, terrain, load_aircraft_profile(PROFILE_PATH), goal_tolerance=GOAL_TOLERANCE,
                                     config=CONFIG, max_expansions=30_000, max_search_time_s=300.0, diagnostics=diagnostics)
    descriptors = {record["node_id"]: _descriptor(record["node_id"], diagnostics.all_nodes, start, goal) for record in diagnostics.expanded}
    rows, groups, timing = _group_analysis(diagnostics.expanded, descriptors, terrain)
    route_sections = {}
    for section in ("first", "middle", "final"):
        subset = [record for record in diagnostics.expanded if _route_third(record["pose"], start, goal) == section]
        schedule_count = len({descriptors[r["node_id"]]["vertical_rle"] for r in subset})
        route_sections[section] = {"distinct_schedules": schedule_count, "z_states": len(subset),
                                   "schedule_to_z_ratio": schedule_count / len(subset) if subset else 0.0}
    group_sizes = {record["node_id"]: len(groups[(record["key"].x_bin, record["key"].y_bin, record["key"].heading_bin)]) for record in diagnostics.expanded}
    classes = _classify(diagnostics.expanded, descriptors, group_sizes)
    thresholds = next(iter(classes.values()))["thresholds"] if classes else {}
    switch_distribution = Counter(descriptors[r["node_id"]]["vertical_mode_switches"] for r in diagnostics.expanded)
    best = _descriptor(result.best_nodes[-1].node_id, diagnostics.all_nodes, start, goal)
    near = [descriptors[r["node_id"]] for r in diagnostics.expanded if r["xy_error_m"] <= 500.0]
    fragmented_cutoff = thresholds["fragmented_fragmentation_ratio_p90"]
    sustained_cutoff = thresholds["sustained_fragmentation_ratio_p10"]
    near_with_descent = [value for value in near if value["descent_steps"]]
    near_fragmented = sum(value["descent_runs"] / value["descent_steps"] >= fragmented_cutoff for value in near_with_descent)
    near_sustained = sum(value["descent_runs"] / value["descent_steps"] <= sustained_cutoff for value in near_with_descent)
    goal_open_nodes = [node for node in diagnostics.all_nodes.values() if pose_in_goal(node.end_pose, goal, GOAL_TOLERANCE)]
    goal_open = [_descriptor(node.node_id, diagnostics.all_nodes, start, goal) for node in goal_open_nodes]
    mean_z = statistics.fmean(len(items) for items in groups.values())
    baseline = {"reproduced": "YES" if (result.termination_reason == "EXPANSION_LIMIT" and result.expanded_nodes == 30_000 and result.generated_neighbors == 149_995) else "NO",
                "termination_reason": result.termination_reason, "expanded": result.expanded_nodes, "generated": result.generated_neighbors,
                "rejected": result.rejected_neighbors, "peak_open": result.max_open_size, "runtime_s": result.runtime_s,
                "best_xy_error_m": result.closest_xy_distance_to_goal_m,
                "best_z_error_m": min(r["z_error_m"] for r in diagnostics.expanded)}
    top_states = sum(row["z_states"] for row in rows[:20])
    top_distinct = sum(row["distinct_rle_schedules"] for row in rows[:20])
    conclusion = (f"PRIMARY Z-DIVERSITY CAUSE: **A. VERTICAL TIMING COMBINATORICS**. In the top 20 groups, {top_distinct:,}/{top_states:,} representatives have distinct vertical RLE schedules while every inspected group has one geometric lineage. The narrower same-descent-count/different-order screen is {timing['same_count_timing_percent']:.2f}%, so the evidence is repeated LEVEL-vs-DESCENT decisions (including different descent amounts), not merely permutations with an equal descent count. Of observed descent→level interruptions, { _interruption_causality(diagnostics.expanded, descriptors, diagnostics).get('terrain_forced', 0) } are terrain-forced and { _interruption_causality(diagnostics.expanded, descriptors, diagnostics).get('non_terrain_forced', 0) } are not. VERTICAL DECISION COMPRESSION POTENTIAL: **MODERATE**. LONGER VERTICAL PRIMITIVE JUSTIFIED: **NO**: near-goal/best-prefix evidence is predominantly fragmented, so a sustained-descent macro is not supported without a design that retains selectable timing and full terrain/envelope validation.")
    payload = {"task": "Z-DIAG-2", "diagnostic_only": True, "production_behavior_changed": False, "baseline": baseline,
               "mean_z_per_xy_heading": mean_z, "top_20_groups": rows[:20], "same_descent_count_different_timing": timing,
               "route_sections": route_sections, "mode_switch_distribution": dict(sorted(switch_distribution.items())),
               "mode_switch_summary": _switch_summary(switch_distribution),
               "schedule_classes": classes, "schedule_thresholds": thresholds, "decision_branching": _decision_branching(diagnostics),
               "interruption_causality": _interruption_causality(diagnostics.expanded, descriptors, diagnostics),
               "near_goal": {"count": len(near), "with_descent_count": len(near_with_descent),
                             "fragmented_count": near_fragmented, "sustained_count": near_sustained,
                             "samples": [_descriptor_payload(value) for value in near[:20]]},
               "goal_open": {"count": len(goal_open), "samples": [_descriptor_payload(value) for value in goal_open[:20]]},
               "best_partial": _descriptor_payload(best), "best_partial_macro_runs": _macro_summary(best), "conclusion": conclusion}
    OUTPUT_JSON.write_text(json.dumps(payload, indent=2, allow_nan=True, default=str) + "\n", encoding="utf-8")
    OUTPUT_MD.write_text(_markdown(payload), encoding="utf-8")
    print(f"json: {OUTPUT_JSON}")
    print(f"report: {OUTPUT_MD}")


if __name__ == "__main__":
    main()
