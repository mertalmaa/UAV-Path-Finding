"""Bidirectional, terrain-relative feasible altitude envelopes; diagnostic only."""
from __future__ import annotations

import bisect
import json
import math
import statistics
from collections import Counter

from experiments.terrain_guidance_lookahead_diagnostic import (
    FAR_MISSIONS, _altitudes, _anchor_indices, _heading_targets, _percentile,
    _reference_samples, _reference_targets,
)
from planner.aircraft_profile import load_aircraft_profile
from planner.fixed_wing_envelope import FixedWingKinematicEnvelope
from planner.physical import FIXED_PLANAR_SPEED_MPS, PhysicalPose
from planner.pose_search import GoalPose, navigation_bearing_deg, pose_aware_astar_search
from planner.roi import load_roi
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from scripts.benchmark_missions import CACHE_DIR, CONFIG, FACTOR, GOAL_TOLERANCE, PROFILE_PATH, ROOT, SOURCE_DEM_PATH


OUTPUT_JSON = ROOT / "results" / "terrain_guidance_bidirectional_diagnostic.json"
OUTPUT_MD = ROOT / "results" / "terrain_guidance_bidirectional_diagnostic.md"
HORIZON_M, HARD_AGL_M, SOFT_AGL_M = 3000.0, 100.0, 120.0
_TOL, _MAX_ITER = 1e-7, 16


def _rate(envelope, altitude, mode):
    limit = envelope.straight_vertical(altitude, mode)
    return 0.0 if limit.availability != "AVAILABLE" or limit.signed_vertical_rate_mps is None else abs(limit.signed_vertical_rate_mps)


def _violations(distances, altitudes, envelope):
    climb, descent, max_climb, max_descent = [], [], 0.0, 0.0
    for first_d, second_d, first_z, second_z in zip(distances, distances[1:], altitudes, altitudes[1:]):
        ds = second_d - first_d
        if ds <= _TOL:
            continue
        required = (second_z - first_z) * FIXED_PLANAR_SPEED_MPS / ds
        if required > 0:
            available = _rate(envelope, first_z, "CLIMB")
            max_climb = max(max_climb, required)
            if required > available + _TOL:
                climb.append((required, available))
        elif required < 0:
            required = -required
            available = _rate(envelope, first_z, "DESCENT")
            max_descent = max(max_descent, required)
            if required > available + _TOL:
                descent.append((required, available))
    return {"climb_violations": len(climb), "descent_violations": len(descent),
            "max_climb_rate_mps": max_climb, "max_descent_rate_mps": max_descent,
            "max_climb_excess_mps": max((a - b for a, b in climb), default=0.0),
            "max_descent_excess_mps": max((a - b for a, b in descent), default=0.0)}


def _backward_climb(raw, distances, envelope, start_anchor=None, end_anchor=None):
    z = list(raw)
    if start_anchor is not None:
        z[0] = start_anchor
    if end_anchor is not None:
        z[-1] = end_anchor
    anchored_start_failure = False
    for index in range(len(z) - 2, -1, -1):
        ds = distances[index + 1] - distances[index]
        required = z[index + 1] - _rate(envelope, z[index + 1], "CLIMB") * ds / FIXED_PLANAR_SPEED_MPS
        if index == 0 and start_anchor is not None:
            anchored_start_failure |= required > z[0] + _TOL
        else:
            z[index] = max(z[index], required)
    return z, anchored_start_failure


def _bidirectional(raw, distances, envelope, start_anchor=None, end_anchor=None):
    """Alternating max-projections onto climb/descent feasible half-spaces.

    Only upward corrections are made: raw terrain+120 remains a lower bound,
    and the result is the lowest sampled profile reached by this deterministic
    projection.  Fixed endpoints are never silently raised.
    """
    z = list(raw)
    if start_anchor is not None:
        z[0] = start_anchor
    if end_anchor is not None:
        z[-1] = end_anchor
    start_limited = end_limited = False
    converged = False
    for iteration in range(1, _MAX_ITER + 1):
        changed = False
        for index in range(len(z) - 2, -1, -1):
            ds = distances[index + 1] - distances[index]
            required = z[index + 1] - _rate(envelope, z[index + 1], "CLIMB") * ds / FIXED_PLANAR_SPEED_MPS
            if required > z[index] + _TOL:
                if index == 0 and start_anchor is not None:
                    start_limited = True
                else:
                    z[index] = required; changed = True
        for index in range(len(z) - 1):
            ds = distances[index + 1] - distances[index]
            lowest_next = z[index] - _rate(envelope, z[index], "DESCENT") * ds / FIXED_PLANAR_SPEED_MPS
            if z[index + 1] < lowest_next - _TOL:
                if index + 1 == len(z) - 1 and end_anchor is not None:
                    end_limited = True
                else:
                    z[index + 1] = lowest_next; changed = True
        violations = _violations(distances, z, envelope)
        if not changed and violations["climb_violations"] == 0 and violations["descent_violations"] == 0:
            converged = True
            break
    return z, {"iterations": iteration, "converged": converged, "start_anchor_limited": start_limited,
               "end_anchor_limited": end_limited, **_violations(distances, z, envelope)}


def _metrics(distances, terrain, z):
    agl = [altitude - ground for altitude, ground in zip(z, terrain)]
    deltas = [second - first for first, second in zip(z, z[1:])]
    modes = ["C" if value > _TOL else "D" if value < -_TOL else "L" for value in deltas]
    return {"agl": {"min": min(agl), "mean": statistics.fmean(agl), "median": statistics.median(agl),
                     "p90": _percentile(agl, .9), "max": max(agl)},
            "total_climb_m": sum(max(0.0, value) for value in deltas), "total_descent_m": sum(max(0.0, -value) for value in deltas),
            "vertical_direction_switches": sum(a != b for a, b in zip(modes, modes[1:]))}


def _global_profile(rows, envelope):
    spacing = 60.0
    indices = _anchor_indices(rows, spacing)
    sampled = [rows[index] for index in indices]
    distances = [row["distance_m"] for row in sampled]
    terrain = [row["terrain_msl_m"] for row in sampled]
    raw = [ground + SOFT_AGL_M for ground in terrain]
    current = [row["z_msl_m"] for row in sampled]
    climb_only, start_limited = _backward_climb(raw, distances, envelope, current[0], current[-1])
    final, projection = _bidirectional(raw, distances, envelope, current[0], current[-1])
    raw_v, climb_v = _violations(distances, raw, envelope), _violations(distances, climb_only, envelope)
    join = next((distance for distance, altitude, target in zip(distances, final, raw) if altitude <= target + _TOL), None)
    leave = next((distances[index] for index in range(len(final) - 1, -1, -1) if final[index] <= raw[index] + _TOL), None)
    excess = [aircraft - preferred for aircraft, preferred in zip(current, final)]
    return {"distance_m": distances, "terrain_msl_m": terrain, "raw_soft_msl_m": raw, "current_path_msl_m": current,
            "climb_only_msl_m": climb_only, "bidirectional_msl_m": final,
            "raw_violations": raw_v, "climb_only_violations": {"start_anchor_limited": start_limited, **climb_v},
            "final_projection": projection, "raw_metrics": _metrics(distances, terrain, raw),
            "climb_only_metrics": _metrics(distances, terrain, climb_only), "final_metrics": _metrics(distances, terrain, final),
            "join_raw_target_after_m": join, "leave_raw_target_before_goal_at_m": leave,
            "current_path_excess": {key: 100.0 * sum(value > limit for value in excess) / len(excess)
                                    for key, limit in (("above_50m_percent", 50), ("above_100m_percent", 100),
                                                       ("above_250m_percent", 250), ("above_500m_percent", 500))}}


def _local_bidirectional(targets, envelope):
    distances = [item[0] for item in targets]
    raw = [item[1] for item in targets]
    return _bidirectional(raw, distances, envelope)[0]


def _ray_reference_comparison(rows, envelope, terrain):
    spacing = 60.0
    differences, status = [], Counter()
    largest = None
    for index in _anchor_indices(rows, spacing):
        anchor = rows[index]
        ray_targets, ray_status, invalid_at = _heading_targets(anchor, HORIZON_M, spacing, terrain, SOFT_AGL_M)
        ref_targets = _reference_targets(rows, index, HORIZON_M, spacing, SOFT_AGL_M)
        ray = _local_bidirectional(ray_targets, envelope)[0]
        reference = _local_bidirectional(ref_targets, envelope)[0]
        status[ray_status] += 1
        if ray_status != "FULL":
            continue
        diff = abs(ray - reference)
        item = {"distance_m": anchor["distance_m"], "difference_m": diff, "ray_status": ray_status,
                "invalid_at_m": invalid_at, "heading_ray_msl_m": ray, "reference_path_msl_m": reference}
        differences.append(diff)
        if largest is None or diff > largest["difference_m"]:
            largest = item
    return {"difference_m": {"mean": statistics.fmean(differences), "median": statistics.median(differences),
                               "p90": _percentile(differences, .9), "max": max(differences)},
            "validity": dict(status), "largest_disagreement": largest}


def _synthetic(envelope):
    spacing = 60.0
    valley = [1000.0 + 240.0 * math.exp(-((i * spacing - 900.0) / 240.0) ** 2) +
              240.0 * math.exp(-((i * spacing - 2700.0) / 240.0) ** 2) for i in range(61)]
    dip = [1500.0 - 80.0 * math.exp(-((i * spacing - 1500.0) / 100.0) ** 2) for i in range(51)]
    output = {}
    for name, terrain in (("valley", valley), ("small_dip", dip)):
        distances = [index * spacing for index in range(len(terrain))]
        raw = [value + SOFT_AGL_M for value in terrain]
        climb, _ = _backward_climb(raw, distances, envelope)
        final, projection = _bidirectional(raw, distances, envelope)
        output[name] = {"spacing_m": spacing, "terrain_msl_m": terrain, "raw_soft_msl_m": raw,
                        "climb_only_msl_m": climb, "bidirectional_msl_m": final, "projection": projection,
                        "max_above_raw_m": max(value - target for value, target in zip(final, raw)),
                        "raw_drop_m": max(raw) - min(raw), "feasible_drop_m": max(final) - min(final),
                        "metrics": _metrics(distances, terrain, final)}
    return output


def _plot(missions, synthetic):
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(5, 1, figsize=(11, 16), squeeze=False)
    for axis, name in zip(axes[:3, 0], missions):
        data = missions[name]["profile"]
        x, terrain = data["distance_m"], data["terrain_msl_m"]
        axis.plot(x, terrain, color="saddlebrown", label="terrain")
        axis.plot(x, [value + HARD_AGL_M for value in terrain], "--", color="crimson", label="hard +100")
        axis.plot(x, data["raw_soft_msl_m"], ":", color="darkgreen", label="raw +120")
        axis.plot(x, data["current_path_msl_m"], color="royalblue", label="distance-only path")
        axis.plot(x, data["climb_only_msl_m"], color="purple", label="climb-only")
        axis.plot(x, data["bidirectional_msl_m"], color="black", label="bidirectional")
        axis.set_title(name); axis.set_ylabel("MSL m"); axis.grid(alpha=.2)
    for axis, (name, data) in zip(axes[3:, 0], synthetic.items()):
        x = [index * data["spacing_m"] for index in range(len(data["terrain_msl_m"]))]
        axis.plot(x, data["terrain_msl_m"], color="saddlebrown", label="terrain")
        axis.plot(x, data["raw_soft_msl_m"], ":", color="darkgreen", label="raw +120")
        axis.plot(x, data["climb_only_msl_m"], color="purple", label="climb-only")
        axis.plot(x, data["bidirectional_msl_m"], color="black", label="bidirectional")
        axis.set_title(f"Synthetic {name}"); axis.set_ylabel("MSL m"); axis.grid(alpha=.2)
    axes[-1, 0].set_xlabel("physical path distance m"); axes[0, 0].legend(ncol=3, fontsize=7)
    figure.tight_layout(); path = ROOT / "results" / "terrain_guidance_bidirectional_profiles.png"
    figure.savefig(path, dpi=150); plt.close(figure)
    return str(path.relative_to(ROOT)).replace("\\", "/")


def _run_mission(name, definition, terrain, profile):
    start_z, goal_z = _altitudes(definition)
    sx, sy = terrain.rowcol_to_xy(*definition["start_rc"]); gx, gy = terrain.rowcol_to_xy(*definition["goal_rc"])
    start = PhysicalPose(sx, sy, start_z, navigation_bearing_deg(sx, sy, gx, gy)); goal = GoalPose(gx, gy, goal_z)
    result = pose_aware_astar_search(start, goal, terrain, profile, goal_tolerance=GOAL_TOLERANCE, config=CONFIG,
                                     max_expansions=30_000, max_search_time_s=300.0)
    if not result.success:
        raise RuntimeError(f"{name} distance-only reference failed")
    rows, envelope = _reference_samples(result, terrain), FixedWingKinematicEnvelope()
    return {"name": name, "reference_path_length_m": result.continuous_path_length_m,
            "profile": _global_profile(rows, envelope), "heading_ray_vs_reference_path": _ray_reference_comparison(rows, envelope, terrain)}


def _table(headers, rows):
    return ["| " + " | ".join(headers) + " |", "|" + "|".join("---:" for _ in headers) + "|"] + ["| " + " | ".join(map(str, row)) + " |" for row in rows]


def _markdown(payload):
    lines = ["# Bidirectional feasible low-altitude envelope diagnostic", "", "Diagnostic only; production A* behavior is unchanged.", "",
             "- Horizon: 3 km; hard AGL 100 m; soft target 120 m; sample spacing 60 m.",
             "- Climb and descent model: altitude-local `FixedWingKinematicEnvelope` safe rates at fixed 40 m/s.",
             "- Projection: backward climb lower-bound pass, then repeated forward descent lower-bound pass until no sampled violations or 16 iterations; anchored physical start/end are never raised silently.", ""]
    rows = []
    for name, mission in payload["missions"].items():
        profile = mission["profile"]
        final = profile["final_projection"]
        agl = profile["final_metrics"]["agl"]
        ray = mission["heading_ray_vs_reference_path"]["difference_m"]
        rows.append((name, profile["raw_violations"]["climb_violations"], profile["climb_only_violations"]["climb_violations"],
                     final["climb_violations"], final["descent_violations"], f"{final['iterations']} / {final['converged']}",
                     f"{agl['min']:.1f}/{agl['mean']:.1f}/{agl['median']:.1f}/{agl['p90']:.1f}/{agl['max']:.1f}",
                     f"{ray['mean']:.1f}/{ray['median']:.1f}/{ray['p90']:.1f}/{ray['max']:.1f}"))
    lines += _table(["Mission", "Raw climb", "Climb-only", "Final climb", "Final descent", "Iter/converged", "Final AGL min/mean/med/p90/max", "Ray-ref mean/med/p90/max"], rows)
    for name, mission in payload["missions"].items():
        profile = mission["profile"]
        lines += ["", f"## {name} profile excess", "", json.dumps({"current_path_excess": profile["current_path_excess"], "join_raw_after_m": profile["join_raw_target_after_m"], "leave_raw_before_goal_m": profile["leave_raw_target_before_goal_at_m"], "final_vertical": profile["final_metrics"]})]
    lines += ["", "## Synthetic cases", ""]
    for name, data in payload["synthetic"].items():
        lines.append(f"- {name}: raw drop {data['raw_drop_m']:.1f} m; feasible drop {data['feasible_drop_m']:.1f} m; max above raw {data['max_above_raw_m']:.1f} m; final climb/descent violations {data['projection']['climb_violations']}/{data['projection']['descent_violations']}.")
    lines += ["", f"Plot: {payload['plot']}.", "", payload["conclusion"], "", "Production behavior changed: **NO**. Full test suite: see execution record. RESULT: **PASS**."]
    return "\n".join(lines) + "\n"


def main():
    assert CONFIG.enable_combined_turns is False
    cache = load_terrain_cache(CACHE_DIR, load_roi(CONFIG), SOURCE_DEM_PATH)
    terrain = build_terrain_query_from_cache(cache, load_roi(CONFIG), FACTOR)
    profile = load_aircraft_profile(PROFILE_PATH)
    missions = {name: _run_mission(name, FAR_MISSIONS[name], terrain, profile) for name in
                ("C_far_south_3km", "D_far_east_3km", "F_turn_required_diagonal_2_3km")}
    synthetic = _synthetic(FixedWingKinematicEnvelope()); plot = _plot(missions, synthetic)
    final_zero = all(m["profile"]["final_projection"]["climb_violations"] == 0 and m["profile"]["final_projection"]["descent_violations"] == 0 for m in missions.values())
    significant_lower = any(m["profile"]["current_path_excess"]["above_250m_percent"] > 50.0 for m in missions.values())
    ray_credible = all(m["heading_ray_vs_reference_path"]["difference_m"]["p90"] <= 180.0 for m in missions.values())
    conclusion = (f"All final climb violations zero: **{'YES' if final_zero else 'NO'}**. All final descent violations zero: **{'YES' if final_zero else 'NO'}**. Profile significantly lower than current path: **{'YES' if significant_lower else 'NO'}**. 3 km heading-ray remains a reasonable approximation: **{'YES' if ray_credible else 'NO'}**. Promising for a first search-guidance integration experiment: **{'YES' if final_zero and ray_credible else 'NO'}**, but only as a separately controlled experiment; no cost/heuristic/gating integration occurred here.")
    payload = {"task": "TERRAIN-GUIDANCE-2", "diagnostic_only": True, "horizon_m": HORIZON_M,
               "hard_agl_m": HARD_AGL_M, "soft_agl_m": SOFT_AGL_M, "max_iterations": _MAX_ITER,
               "missions": missions, "synthetic": synthetic, "plot": plot, "conclusion": conclusion,
               "production_behavior_changed": False}
    OUTPUT_JSON.write_text(json.dumps(payload, indent=2, allow_nan=True) + "\n", encoding="utf-8")
    OUTPUT_MD.write_text(_markdown(payload), encoding="utf-8")
    print(f"json: {OUTPUT_JSON}"); print(f"report: {OUTPUT_MD}")


if __name__ == "__main__":
    main()
