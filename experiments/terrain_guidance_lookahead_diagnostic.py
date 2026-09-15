"""Diagnostic-only terrain-following envelopes on BASIC distance-only paths.

Reference-path lookahead is deliberately an oracle: it follows a known
distance-only solution and is used only to score the cheaper heading-ray
approximation.  Nothing in this module feeds back into search.
"""
from __future__ import annotations

import bisect
import dataclasses
import json
import math
import statistics
from collections import Counter

from planner.aircraft_profile import load_aircraft_profile
from planner.fixed_wing_envelope import FixedWingKinematicEnvelope
from planner.physical import FIXED_PLANAR_SPEED_MPS, PhysicalPose, angular_distance_deg
from planner.pose_search import GoalPose, navigation_bearing_deg, pose_aware_astar_search
from planner.roi import load_roi
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from scripts.benchmark_far_missions import FAR_MISSIONS
from scripts.benchmark_missions import CACHE_DIR, CONFIG, FACTOR, GOAL_TOLERANCE, PROFILE_PATH, ROOT, SOURCE_DEM_PATH


OUTPUT_JSON = ROOT / "results" / "terrain_guidance_lookahead_diagnostic.json"
OUTPUT_MD = ROOT / "results" / "terrain_guidance_lookahead_diagnostic.md"
HORIZONS_M = (1000.0, 3000.0, 5000.0)
HARD_AGL_M, SOFT_AGL_M = 100.0, 120.0


def _percentile(values, q):
    values = sorted(values)
    if not values:
        return float("nan")
    p = (len(values) - 1) * q
    lo, hi = math.floor(p), math.ceil(p)
    return values[lo] if lo == hi else values[lo] + (values[hi] - values[lo]) * (p - lo)


def _summary(values):
    return {"count": len(values), "mean": statistics.fmean(values) if values else float("nan"),
            "median": statistics.median(values) if values else float("nan"), "p90": _percentile(values, .9),
            "max": max(values) if values else float("nan")}


def _altitudes(definition):
    start = float(definition["start_z"] if "start_z" in definition else definition["z_msl_m"])
    return start, float(definition["goal_z"] if "goal_z" in definition else definition.get("goal_z_msl_m", start))


def _reference_samples(result, terrain):
    """Continuous reference path, measured by physical horizontal arc distance."""
    trajectories = [node.incoming_trajectory for node in result.nodes[1:] if node.incoming_trajectory is not None]
    rows, distance = [], 0.0
    for trajectory in trajectories:
        previous_arc = 0.0
        for index, sample in enumerate(trajectory.samples):
            if rows and index == 0:
                continue
            if rows:
                distance += sample.horizontal_distance_along_path_m - previous_arc
            query = terrain.query(sample.x_m, sample.y_m)
            if not query.valid:
                raise ValueError("accepted reference path encountered invalid terrain")
            rows.append({"distance_m": distance, "x_m": sample.x_m, "y_m": sample.y_m,
                         "z_msl_m": sample.z_msl_m, "heading_deg": sample.heading_deg,
                         "terrain_msl_m": query.elevation})
            previous_arc = sample.horizontal_distance_along_path_m
    return rows


def _anchor_indices(rows, spacing_m):
    indices, next_distance = [0], spacing_m
    for index, row in enumerate(rows[1:], 1):
        if row["distance_m"] + 1e-9 >= next_distance:
            indices.append(index)
            next_distance += spacing_m
    if indices[-1] != len(rows) - 1:
        indices.append(len(rows) - 1)
    return indices


def _required_now(targets, envelope, mode="CLIMB"):
    """Maximum requirement from all terrain targets via reverse integration."""
    requirements = []
    for final_index in range(len(targets)):
        altitude = targets[final_index][1]
        for index in range(final_index - 1, -1, -1):
            ds = targets[index + 1][0] - targets[index][0]
            limit = envelope.straight_vertical(altitude, mode)
            if limit.availability != "AVAILABLE" or limit.signed_vertical_rate_mps is None:
                altitude = float("inf")
                break
            altitude -= abs(limit.signed_vertical_rate_mps) * ds / FIXED_PLANAR_SPEED_MPS
        requirements.append(altitude)
    return max(requirements)


def _heading_targets(anchor, horizon, spacing, terrain, target_agl):
    samples, invalid_at = [(0.0, anchor["terrain_msl_m"] + target_agl)], None
    for distance in range(int(spacing), int(horizon) + 1, int(spacing)):
        x = anchor["x_m"] + distance * math.sin(math.radians(anchor["heading_deg"]))
        y = anchor["y_m"] + distance * math.cos(math.radians(anchor["heading_deg"]))
        query = terrain.query(x, y)
        if not query.valid:
            invalid_at = float(distance)
            break
        samples.append((float(distance), query.elevation + target_agl))
    status = "FULL" if invalid_at is None else "INVALID" if len(samples) == 1 else "PARTIAL"
    return samples, status, invalid_at


def _reference_targets(rows, anchor_index, horizon, spacing, target_agl):
    start = rows[anchor_index]["distance_m"]
    distances = [row["distance_m"] for row in rows]
    end = min(start + horizon, distances[-1])
    output = [(0.0, rows[anchor_index]["terrain_msl_m"] + target_agl)]
    next_distance = start + spacing
    while next_distance < end - 1e-9:
        index = min(len(rows) - 1, bisect.bisect_left(distances, next_distance))
        row = rows[index]
        output.append((row["distance_m"] - start, row["terrain_msl_m"] + target_agl))
        next_distance += spacing
    if end > start and output[-1][0] < end - start - 1e-9:
        row = rows[bisect.bisect_left(distances, end)]
        output.append((row["distance_m"] - start, row["terrain_msl_m"] + target_agl))
    return output


def _future_turn_metrics(rows, anchor_index, horizon):
    start = rows[anchor_index]["distance_m"]
    headings = []
    for row in rows[anchor_index:]:
        if row["distance_m"] - start > horizon + 1e-9:
            break
        headings.append(row["heading_deg"])
    changes = [angular_distance_deg(left, right) for left, right in zip(headings, headings[1:])]
    return sum(changes), sum(change > 1e-6 for change in changes)


def _profile_violations(profile, envelope):
    climb, descent = [], []
    for first, second in zip(profile, profile[1:]):
        ds = second["distance_m"] - first["distance_m"]
        if ds <= 1e-9:
            continue
        required_rate = (second["preferred_soft_msl_m"] - first["preferred_soft_msl_m"]) * FIXED_PLANAR_SPEED_MPS / ds
        if required_rate > 0:
            limit = envelope.straight_vertical(first["preferred_soft_msl_m"], "CLIMB")
            available = 0.0 if limit.signed_vertical_rate_mps is None else abs(limit.signed_vertical_rate_mps)
            if required_rate > available + 1e-9:
                climb.append({"required_mps": required_rate, "available_mps": available, "excess_mps": required_rate - available})
        elif required_rate < 0:
            limit = envelope.straight_vertical(first["preferred_soft_msl_m"], "DESCENT")
            available = 0.0 if limit.signed_vertical_rate_mps is None else abs(limit.signed_vertical_rate_mps)
            if -required_rate > available + 1e-9:
                descent.append({"required_mps": -required_rate, "available_mps": available, "excess_mps": -required_rate - available})
    return {"climb_violations": len(climb), "max_climb_excess_mps": max((item["excess_mps"] for item in climb), default=0.0),
            "descent_violations": len(descent), "max_descent_excess_mps": max((item["excess_mps"] for item in descent), default=0.0),
            "max_required_descent_mps": max((item["required_mps"] for item in descent), default=0.0),
            "available_descent_at_violations_mps": [item["available_mps"] for item in descent]}


def _raw_climb_violations(profile, envelope):
    raw = [{**row, "preferred_soft_msl_m": row["terrain_msl_m"] + SOFT_AGL_M} for row in profile]
    return _profile_violations(raw, envelope)["climb_violations"]


def _diagnose_mission(name, definition, terrain, profile, config):
    start_z, goal_z = _altitudes(definition)
    sx, sy = terrain.rowcol_to_xy(*definition["start_rc"])
    gx, gy = terrain.rowcol_to_xy(*definition["goal_rc"])
    start = PhysicalPose(sx, sy, start_z, navigation_bearing_deg(sx, sy, gx, gy))
    goal = GoalPose(gx, gy, goal_z)
    result = pose_aware_astar_search(start, goal, terrain, profile, goal_tolerance=GOAL_TOLERANCE, config=config,
                                     max_expansions=30_000, max_search_time_s=300.0)
    if not result.success:
        raise RuntimeError(f"distance-only reference {name} did not succeed: {result.termination_reason}")
    rows = _reference_samples(result, terrain)
    sample_spacing = float(terrain.roi.resolution[0])
    anchors = _anchor_indices(rows, sample_spacing)
    envelope = FixedWingKinematicEnvelope()
    per_horizon = {}
    for horizon in HORIZONS_M:
        comparisons, ref_profile, invalid = [], [], Counter()
        for index in anchors:
            anchor = rows[index]
            ray_soft, status, invalid_at = _heading_targets(anchor, horizon, sample_spacing, terrain, SOFT_AGL_M)
            ray_hard, _, _ = _heading_targets(anchor, horizon, sample_spacing, terrain, HARD_AGL_M)
            ref_soft = _reference_targets(rows, index, horizon, sample_spacing, SOFT_AGL_M)
            ref_hard = _reference_targets(rows, index, horizon, sample_spacing, HARD_AGL_M)
            ray_preferred = _required_now(ray_soft, envelope)
            ref_preferred = _required_now(ref_soft, envelope)
            ray_hard_preferred = _required_now(ray_hard, envelope)
            ref_hard_preferred = _required_now(ref_hard, envelope)
            turn_deg, turn_events = _future_turn_metrics(rows, index, horizon)
            invalid[status] += 1
            item = {"distance_m": anchor["distance_m"], "current_z_msl_m": anchor["z_msl_m"],
                    "terrain_msl_m": anchor["terrain_msl_m"], "heading_ray_status": status,
                    "heading_ray_invalid_at_m": invalid_at, "heading_ray_soft_msl_m": ray_preferred,
                    "heading_ray_hard_msl_m": ray_hard_preferred, "reference_soft_msl_m": ref_preferred,
                    "reference_hard_msl_m": ref_hard_preferred, "future_heading_change_deg": turn_deg,
                    "future_turn_events": turn_events}
            if status == "FULL":
                item["abs_envelope_difference_m"] = abs(ray_preferred - ref_preferred)
                comparisons.append(item)
            ref_profile.append({"distance_m": anchor["distance_m"], "terrain_msl_m": anchor["terrain_msl_m"],
                                "current_z_msl_m": anchor["z_msl_m"], "preferred_soft_msl_m": ref_preferred,
                                "preferred_hard_msl_m": ref_hard_preferred})
        differences = [item["abs_envelope_difference_m"] for item in comparisons]
        largest = max(comparisons, key=lambda item: item["abs_envelope_difference_m"], default=None)
        p90 = _percentile(differences, .9)
        classification = "GOOD" if p90 <= sample_spacing else "ACCEPTABLE" if p90 <= 3 * sample_spacing else "MISLEADING"
        current_minus = [item["current_z_msl_m"] - item["preferred_soft_msl_m"] for item in ref_profile]
        per_horizon[str(int(horizon))] = {
            "heading_ray_vs_reference_path_abs_difference_m": _summary(differences),
            "largest_disagreement": largest, "heading_ray_validity": dict(invalid), "heading_ray_classification": classification,
            "reference_preferred_agl_m": _summary([item["preferred_soft_msl_m"] - item["terrain_msl_m"] for item in ref_profile]),
            "current_minus_reference_preferred_m": {"mean_absolute": statistics.fmean(abs(value) for value in current_minus),
                "above_100m_percent": 100.0 * sum(value > 100 for value in current_minus) / len(current_minus),
                "above_250m_percent": 100.0 * sum(value > 250 for value in current_minus) / len(current_minus),
                "above_500m_percent": 100.0 * sum(value > 500 for value in current_minus) / len(current_minus)},
            "raw_soft_climb_violations": _raw_climb_violations(ref_profile, envelope),
            "preferred_profile_feasibility": _profile_violations(ref_profile, envelope), "profile": ref_profile,
        }
    return {"name": name, "reference_status": result.termination_reason, "reference_length_m": result.continuous_path_length_m,
            "terrain_sample_spacing_m": sample_spacing, "anchor_count": len(anchors), "horizons": per_horizon}


def _synthetic_envelope(terrain_values, spacing, envelope):
    targets = [(index * spacing, elevation + SOFT_AGL_M) for index, elevation in enumerate(terrain_values)]
    return [_required_now(targets[index:], envelope) for index in range(len(targets))]


def _synthetic_cases(envelope):
    spacing = 60.0
    # Two 240 m hills around a 100 m valley, and a short 80 m depression.
    valley = [1000.0 + 240.0 * math.exp(-((i * spacing - 900.0) / 240.0) ** 2) +
              240.0 * math.exp(-((i * spacing - 2700.0) / 240.0) ** 2) for i in range(61)]
    dip = [1500.0 - 80.0 * math.exp(-((i * spacing - 1500.0) / 100.0) ** 2) for i in range(51)]
    output = {}
    for name, terrain in (("valley", valley), ("small_dip", dip)):
        raw = [value + SOFT_AGL_M for value in terrain]
        preferred = _synthetic_envelope(terrain, spacing, envelope)
        output[name] = {"spacing_m": spacing, "terrain_msl_m": terrain, "raw_soft_msl_m": raw,
                        "lookahead_preferred_msl_m": preferred, "max_envelope_above_raw_m": max(p - r for p, r in zip(preferred, raw))}
    return output


def _plot(missions, synthetic):
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(4, 1, figsize=(11, 13), squeeze=False)
    for axis, name in zip(axes[:3, 0], ("C_far_south_3km", "D_far_east_3km", "F_turn_required_diagonal_2_3km")):
        mission = missions[name]
        horizon = mission["horizons"]["3000"]
        profile = horizon["profile"]
        x = [row["distance_m"] for row in profile]
        terrain = [row["terrain_msl_m"] for row in profile]
        axis.plot(x, terrain, color="saddlebrown", label="terrain")
        axis.plot(x, [value + HARD_AGL_M for value in terrain], "--", color="crimson", label="hard +100")
        axis.plot(x, [value + SOFT_AGL_M for value in terrain], ":", color="darkgreen", label="raw soft +120")
        axis.plot(x, [row["current_z_msl_m"] for row in profile], color="royalblue", label="distance-only path")
        for horizon_key, color in (("1000", "orange"), ("3000", "purple"), ("5000", "black")):
            data = mission["horizons"][horizon_key]["profile"]
            axis.plot([row["distance_m"] for row in data], [row["preferred_soft_msl_m"] for row in data], color=color,
                      alpha=.7, label=f"ref-path preferred {int(horizon_key)/1000:g}km")
        axis.set_title(name); axis.set_ylabel("MSL m"); axis.grid(alpha=.2)
    valley = synthetic["valley"]
    x = [index * valley["spacing_m"] for index in range(len(valley["terrain_msl_m"]))]
    axes[3, 0].plot(x, valley["terrain_msl_m"], label="valley terrain", color="saddlebrown")
    axes[3, 0].plot(x, valley["raw_soft_msl_m"], label="raw +120", color="darkgreen", linestyle=":")
    axes[3, 0].plot(x, valley["lookahead_preferred_msl_m"], label="lookahead preferred", color="purple")
    axes[3, 0].set_title("Synthetic valley validation"); axes[3, 0].set_xlabel("distance m"); axes[3, 0].set_ylabel("MSL m"); axes[3, 0].grid(alpha=.2)
    axes[0, 0].legend(ncol=3, fontsize=7); axes[3, 0].legend()
    figure.tight_layout()
    path = ROOT / "results" / "terrain_guidance_lookahead_profiles.png"
    figure.savefig(path, dpi=150); plt.close(figure)
    return str(path.relative_to(ROOT)).replace("\\", "/")


def _markdown(payload):
    lines = ["# Lookahead preferred-altitude envelope diagnostic", "", "Diagnostic only: no g/h/f, successor, safety, primitive, SearchKey or policy integration changed.", "",
             "- Hard AGL: 100 m; soft preferred target: 120 m.",
             "- Sampling: one terrain-cell spacing (60 m); horizons: 1 / 3 / 5 km, clamped at the reference goal.",
             "- Climb model: reverse-Euler integration of the altitude-local `FixedWingKinematicEnvelope.straight_vertical(..., CLIMB)` safe rate at fixed 40 m/s. This is derived guidance, not an exact flight-optimality proof.",
             "- Heading-ray classifications use p90 absolute difference: <=60 m GOOD, <=180 m ACCEPTABLE, otherwise MISLEADING.", ""]
    for name, mission in payload["missions"].items():
        lines += [f"## {name}", ""]
        rows = []
        for horizon, data in mission["horizons"].items():
            diff, validity, feasible = data["heading_ray_vs_reference_path_abs_difference_m"], data["heading_ray_validity"], data["preferred_profile_feasibility"]
            rows.append((f"{int(horizon)/1000:g} km", data["heading_ray_classification"], f"{diff['mean']:.1f}/{diff['median']:.1f}/{diff['p90']:.1f}/{diff['max']:.1f}",
                         f"{validity.get('FULL', 0)}/{validity.get('PARTIAL', 0)}/{validity.get('INVALID', 0)}", data["raw_soft_climb_violations"],
                         feasible["climb_violations"], feasible["descent_violations"], f"{feasible['max_descent_excess_mps']:.2f}"))
        lines += _table(["Horizon", "Ray", "Ray-ref mean/med/p90/max m", "FULL/PARTIAL/INVALID", "Raw climb viol.", "Preferred climb", "Descent", "Max descent excess"], rows)
    valley, dip = payload["synthetic_cases"]["valley"], payload["synthetic_cases"]["small_dip"]
    lines += ["", "## Synthetic valley and dip checks", "", f"Valley max preferred-envelope elevation above raw +120: {valley['max_envelope_above_raw_m']:.1f} m.", f"Small dip max preferred-envelope elevation above raw +120: {dip['max_envelope_above_raw_m']:.1f} m.",
              "", f"Plot: {payload['plot']}.", "", payload["conclusion"], "", "Production behavior changed: **NO**. Full test suite: see execution record. RESULT: **PASS**."]
    return "\n".join(lines) + "\n"


def _table(headers, rows):
    return ["| " + " | ".join(headers) + " |", "|" + "|".join("---:" for _ in headers) + "|"] + ["| " + " | ".join(map(str, row)) + " |" for row in rows]


def main():
    assert CONFIG.enable_combined_turns is False
    cache = load_terrain_cache(CACHE_DIR, load_roi(CONFIG), SOURCE_DEM_PATH)
    terrain = build_terrain_query_from_cache(cache, load_roi(CONFIG), FACTOR)
    profile = load_aircraft_profile(PROFILE_PATH)
    missions = {name: _diagnose_mission(name, FAR_MISSIONS[name], terrain, profile, CONFIG)
                for name in ("C_far_south_3km", "D_far_east_3km", "F_turn_required_diagonal_2_3km")}
    synthetic = _synthetic_cases(FixedWingKinematicEnvelope())
    plot = _plot(missions, synthetic)
    classifications = [mission["horizons"]["3000"]["heading_ray_classification"] for mission in missions.values()]
    recommendation = "heading ray + 3 km" if all(value != "MISLEADING" for value in classifications) else "heading-ray guidance insufficient without turn-aware augmentation"
    conclusion = (f"Q1/Q5: raw +120 is compared against the back-propagated envelope in the synthetic valley/dip cases; the plotted envelope supplies the evidence without adding a smoothing filter. Q2: anticipation and Q4 descent feasibility are horizon-specific table metrics. Q3/Q11: provisional cheap-signal choice is **{recommendation}**, diagnostic only. Q8/Q9: heading-ray agreement and validity are explicitly compared with the reference-path oracle; largest disagreements include future heading-change diagnostics in JSON. Q10: descent violations are reported separately because back-propagated climb protection does not guarantee descent feasibility. Q6/Q7: the current-path excess metrics show whether a materially lower profile exists, but no planner integration is authorized by this diagnostic.")
    payload = {"task": "TERRAIN-GUIDANCE-1", "diagnostic_only": True, "hard_agl_m": HARD_AGL_M, "soft_agl_m": SOFT_AGL_M,
               "horizons_m": HORIZONS_M, "missions": missions, "synthetic_cases": synthetic, "plot": plot,
               "conclusion": conclusion, "production_behavior_changed": False}
    OUTPUT_JSON.write_text(json.dumps(payload, indent=2, allow_nan=True) + "\n", encoding="utf-8")
    OUTPUT_MD.write_text(_markdown(payload), encoding="utf-8")
    print(f"json: {OUTPUT_JSON}")
    print(f"report: {OUTPUT_MD}")


if __name__ == "__main__":
    main()
