"""Stage 38.3: mission-policy generalization and epsilon robustness.

Objective tests A-K are deterministic cost replays with no search. Search
tests use small 30 m synthetic maps and one continuous genuine ARA* schedule
per map. The real Aladaglar data is replay-only; no real search is started.
"""
import csv
import dataclasses
import math
import time
from dataclasses import dataclass

import numpy as np
from affine import Affine

from planner.astar import (
    _path_altitude_metrics,
    ara_star_search,
    compute_distance_reference,
    compute_edge_cost,
    msl_to_z_index,
    state_to_xyz,
)
from planner.coarse_astar import compute_coarse_edge_cost
from planner.config import DEFAULT_CONFIG
from planner.mission import MissionPolicy, production_mission_policy
from planner.primitives import build_primitive_set, evaluate_primitive
from planner.roi import ROIData, load_roi
from planner.terrain import TerrainQuery
from planner.transition import evaluate_transition
from scripts.validate_cost_function_ranking import build_direct_level


Point = tuple[float, float, float]
WEIGHTS = (1.0, 1.25, 1.5)
EPSILONS = (1.70, 1.50, 1.30)
NODATA = -9999.0


@dataclass(frozen=True)
class Candidate:
    scenario: str
    name: str
    points: tuple[Point, ...]
    expected_winner: bool = False


@dataclass(frozen=True)
class ObjectiveScenario:
    code: str
    candidates: tuple[Candidate, ...]
    altitude_reference_msl: float
    terrain_elevation: callable


def flat_terrain(elevation):
    return lambda _x, _y: elevation


def path_stats(points, policy):
    d_ref = math.dist(points[0], points[-1])
    distance = altitude = total_len = weighted_alt = 0.0
    mids = []
    climb = descent = 0.0
    for p1, p2 in zip(points, points[1:]):
        seg = math.dist(p1, p2)
        comp = policy.edge_components(seg, p1[2], p2[2], d_ref)
        distance += comp.distance
        altitude += comp.altitude
        total_len += seg
        mid = (p1[2] + p2[2]) / 2.0
        weighted_alt += seg * mid
        mids.append((mid, seg))
        dz = p2[2] - p1[2]
        climb += max(0.0, dz)
        descent += max(0.0, -dz)
    mids.sort()
    percentiles = []
    for target in (0.25, 0.50, 0.75):
        acc = 0.0
        value = mids[-1][0]
        for mid, weight in mids:
            acc += weight
            if acc >= target * total_len:
                value = mid
                break
        percentiles.append(value)
    low_band_limit = policy.altitude_reference_msl + 50.0
    low_band_fraction = sum(length for mid, length in mids if mid <= low_band_limit) / total_len
    return {
        "length": total_len,
        "distance": distance,
        "mean_msl": weighted_alt / total_len,
        "min_msl": min(p[2] for p in points),
        "max_msl": max(p[2] for p in points),
        "altitude": altitude,
        "total": distance + altitude,
        "climb": climb,
        "descent": descent,
        "p25": percentiles[0],
        "p50": percentiles[1],
        "p75": percentiles[2],
        "low_band_fraction": low_band_fraction,
    }


def validate_polyline_safety(points, terrain_elevation, config):
    """Safety only: transition limits plus sampled AGL; no cost/ranking."""
    min_agl = math.inf
    reasons = []
    max_angle = 0.0
    for p1, p2 in zip(points, points[1:]):
        transition = evaluate_transition(p1, p2, config)
        max_angle = max(max_angle, transition.flight_path_angle_deg)
        if not transition.valid:
            reasons.append(transition.reason)
            continue
        samples = max(1, math.ceil(transition.horizontal_distance_m / config.primitive_sample_spacing_m))
        for i in range(samples + 1):
            t = i / samples
            x = p1[0] + (p2[0] - p1[0]) * t
            y = p1[1] + (p2[1] - p1[1]) * t
            z = p1[2] + (p2[2] - p1[2]) * t
            terrain = terrain_elevation(x, y)
            if terrain is None or not math.isfinite(terrain):
                reasons.append("nodata")
                continue
            agl = z - terrain
            min_agl = min(min_agl, agl)
            if agl < config.min_agl_m:
                reasons.append("below_min_agl")
    return {"safe": not reasons, "min_agl": min_agl, "max_angle": max_angle, "reasons": sorted(set(reasons))}


def objective_scenarios():
    high500 = ((0, 0, 500), (4000, 0, 500))
    low8 = ((0, 0, 500), (700, 500, 400), (3300, 500, 400), (4000, 0, 500))
    low43 = ((0, 0, 500), (700, 1400, 400), (3300, 1400, 400), (4000, 0, 500))
    low_direct = ((0, 0, 500), (600, 0, 400), (3400, 0, 400), (4000, 0, 500))
    side90 = ((0, 0, 500), (700, 90, 400), (3300, 90, 400), (4000, 0, 500))
    return (
        ObjectiveScenario("A FLAT", (
            Candidate("A FLAT", "short", high500, True),
            Candidate("A FLAT", "same_msl_detour", ((0, 0, 500), (2000, 500, 500), (4000, 0, 500))),
        ), 400, flat_terrain(0)),
        ObjectiveScenario("B LOW SLIGHT DETOUR", (
            Candidate("B LOW SLIGHT DETOUR", "high_short", high500),
            Candidate("B LOW SLIGHT DETOUR", "low_8pct_detour", low8, True),
        ), 400, flat_terrain(0)),
        ObjectiveScenario("C LOW LARGE DETOUR", (
            Candidate("C LOW LARGE DETOUR", "high_short", high500, True),
            Candidate("C LOW LARGE DETOUR", "low_43pct_detour", low43),
        ), 400, flat_terrain(0)),
        ObjectiveScenario("D RIDGE", (
            Candidate("D RIDGE", "ridge_overflight", ((0, 0, 600), (4000, 0, 600))),
            Candidate("D RIDGE", "lower_bypass", ((0, 0, 600), (700, 500, 500), (3300, 500, 500), (4000, 0, 600)), True),
        ), 500, lambda _x, y: 380 if abs(y) < 1 else 200),
        ObjectiveScenario("E BROAD VALLEY", (
            Candidate("E BROAD VALLEY", "high_cruise", high500),
            Candidate("E BROAD VALLEY", "descend_cruise_climb", low_direct, True),
        ), 400, flat_terrain(100)),
        ObjectiveScenario("F NARROW SIDE VALLEY", (
            Candidate("F NARROW SIDE VALLEY", "center_high", high500),
            Candidate("F NARROW SIDE VALLEY", "side_low_3cells", side90, True),
        ), 400, lambda _x, y: 260 if abs(y) < 1 else 100),
        ObjectiveScenario("G TWO VALLEYS", (
            Candidate("G TWO VALLEYS", "near_high_valley", high500),
            Candidate("G TWO VALLEYS", "far_deep_valley", low8, True),
        ), 400, flat_terrain(100)),
        ObjectiveScenario("H HIGH-VALLEY-HIGH", (
            Candidate("H HIGH-VALLEY-HIGH", "stay_high", high500),
            Candidate("H HIGH-VALLEY-HIGH", "early_descent", low_direct, True),
        ), 400, flat_terrain(100)),
        ObjectiveScenario("I SAFETY OVERRIDE", (
            Candidate("I SAFETY OVERRIDE", "high_safe", high500, True),
            Candidate("I SAFETY OVERRIDE", "lowest_but_unsafe", ((0, 0, 500), (600, 0, 350), (3400, 0, 350), (4000, 0, 500))),
        ), 350, flat_terrain(200)),
        ObjectiveScenario("J LOW AGL vs LOW MSL", (
            Candidate("J LOW AGL vs LOW MSL", "terrain_follow_high_msl", ((0, 0, 600), (4000, 0, 600))),
            Candidate("J LOW AGL vs LOW MSL", "higher_agl_low_msl", ((0, 0, 600), (700, 500, 500), (3300, 500, 500), (4000, 0, 600)), True),
        ), 500, lambda _x, y: 380 if abs(y) < 1 else 100),
        ObjectiveScenario("K DIFFERENT ENDPOINTS", (
            Candidate("K DIFFERENT ENDPOINTS", "monotone", ((0, 0, 500), (4000, 0, 400)), True),
            Candidate("K DIFFERENT ENDPOINTS", "unneeded_climb", ((0, 0, 500), (700, 0, 520), (3300, 0, 520), (4000, 0, 400))),
        ), 400, flat_terrain(0)),
    )


def run_objective_suite(config):
    print("=== OBJECTIVE TESTS (NO SEARCH) ===")
    rows = []
    scenario_results = {}
    for scenario in objective_scenarios():
        candidate_rows = []
        for candidate in scenario.candidates:
            safety = validate_polyline_safety(candidate.points, scenario.terrain_elevation, config)
            by_weight = {}
            for weight in WEIGHTS:
                policy = MissionPolicy(scenario.altitude_reference_msl, 1000.0, 1.0, weight, 0.0)
                by_weight[weight] = path_stats(candidate.points, policy)
            row = {"scenario": scenario.code, "candidate": candidate.name, "safety": safety, "weights": by_weight,
                   "expected": candidate.expected_winner}
            rows.append(row)
            candidate_rows.append(row)
        safe_rows = [r for r in candidate_rows if r["safety"]["safe"]]
        actual = min(safe_rows, key=lambda r: r["weights"][1.25]["total"])
        expected = next(r for r in candidate_rows if r["expected"])
        passed = actual["candidate"] == expected["candidate"]
        scenario_results[scenario.code] = {"pass": passed, "winner": actual["candidate"], "expected": expected["candidate"]}
        print(f"{scenario.code}: expected={expected['candidate']} actual={actual['candidate']} {'PASS' if passed else 'FAIL'}")
        for row in candidate_rows:
            s = row["weights"][1.25]
            print(f"  {row['candidate']}: safe={row['safety']['safe']} len={s['length']:.1f} mean={s['mean_msl']:.1f} "
                  f"min/max={s['min_msl']:.0f}/{s['max_msl']:.0f} D={s['distance']:.6f} A={s['altitude']:.6f} "
                  f"total={s['total']:.6f} P25/50/75={s['p25']:.0f}/{s['p50']:.0f}/{s['p75']:.0f} "
                  f"low_band={100*s['low_band_fraction']:.1f}% climb/descent={s['climb']:.0f}/{s['descent']:.0f}")
    return rows, scenario_results


def make_roi(elevation):
    h, w = elevation.shape
    return ROIData(elevation.astype(np.float32), Affine(30, 0, 0, 0, -30, h * 30), "EPSG:32636", w, h,
                   (0, 0, w * 30, h * 30), (30, 30), NODATA)


def synthetic_search_specs():
    a = np.full((9, 35), 100.0)
    f = np.full((17, 65), 200.0)
    f[9:12, :] = 100.0
    g = np.full((19, 65), 260.0)
    g[6:9, :] = 180.0
    g[12:15, :] = 80.0
    g[:, :13] = 80.0
    g[:, 52:] = 80.0
    h = np.full((13, 65), 100.0)
    h[:, :13] = 200.0
    h[:, 52:] = 200.0
    return (
        ("A FLAT", a, (4, 3), (4, 31), 300.0, 300.0, 300.0, lambda q: q["mean_msl"] <= 301 and q["xy"] <= 850),
        ("F NARROW SIDE VALLEY", f, (7, 4), (7, 60), 400.0, 300.0, 500.0, lambda q: q["mean_msl"] < 375),
        ("G TWO VALLEYS", g, (7, 4), (7, 60), 400.0, 280.0, 500.0, lambda q: q["mean_msl"] < 365),
        ("H HIGH-VALLEY-HIGH", h, (6, 4), (6, 60), 400.0, 300.0, 500.0, lambda q: q["mean_msl"] < 375),
    )


def state_path_report(path, terrain, config, policy, primitives, d_ref):
    if not path:
        return None
    xyz = [state_to_xyz(s, terrain, config) for s in path]
    distance = altitude = 0.0
    for p1, p2 in zip(xyz, xyz[1:]):
        comp = policy.edge_components(math.dist(p1, p2), p1[2], p2[2], d_ref)
        distance += comp.distance
        altitude += comp.altitude
    alt = _path_altitude_metrics(path, terrain, config)
    xy = sum(math.hypot(p2[0] - p1[0], p2[1] - p1[1]) for p1, p2 in zip(xyz, xyz[1:]))
    by_delta = {(p.drow, p.dcol, round(p.dz_m / config.z_step_m)): p for p in primitives}
    min_agl, max_angle, violations = math.inf, 0.0, 0
    for s1, s2 in zip(path, path[1:]):
        prim = by_delta.get((s2[0] - s1[0], s2[1] - s1[1], s2[2] - s1[2]))
        if prim is None:
            violations += 1
            continue
        ev = evaluate_primitive(state_to_xyz(s1, terrain, config), prim, terrain, config)
        violations += int(not ev.valid)
        if ev.valid:
            min_agl = min(min_agl, ev.min_agl_m)
        max_angle = max(max_angle, math.degrees(math.atan2(abs(prim.dz_m), prim.horizontal_distance_m)))
    return {"length": alt["geometric_path_length"], "xy": xy, "min_msl": alt["minimum_aircraft_msl"],
            "mean_msl": alt["average_aircraft_msl"], "distance": distance, "altitude": altitude,
            "total": distance + altitude, "climb": alt["total_climb_m"], "descent": alt["total_descent_m"],
            "safe": violations == 0 and min_agl >= config.min_agl_m and max_angle <= config.max_climb_angle_deg,
            "min_agl": min_agl, "max_angle": max_angle, "violations": violations}


def run_search_suite(config, objective_results):
    print("\n=== SEARCH TESTS (w_altitude=1.25 fixed) ===")
    rows = []
    for name, elevation, start_rc, goal_rc, start_msl, min_msl, max_msl, preferred in synthetic_search_specs():
        terrain = TerrainQuery(make_roi(elevation))
        cfg = dataclasses.replace(config, cost_mode="normalized", altitude_reference_msl=min_msl,
                                  normalized_w_distance=1.0, normalized_w_altitude=1.25,
                                  normalized_altitude_scale_m=1000.0, goal_tolerance_xy_m=0.0,
                                  goal_tolerance_z_m=0.0)
        primitives = build_primitive_set(cfg)
        z = msl_to_z_index(start_msl, cfg)
        start, goal = (start_rc[0], start_rc[1], z), (goal_rc[0], goal_rc[1], z)
        d_ref = compute_distance_reference(start, goal, terrain, cfg)
        policy = production_mission_policy(min_msl)
        t0 = time.perf_counter()
        result = ara_star_search(start, goal, terrain, min_msl, max_msl, cfg, primitives,
                                 epsilon_schedule=EPSILONS, max_expansions_cumulative=12_000)
        wall = time.perf_counter() - t0
        previous_time = 0.0
        for phase in result.phases:
            q = state_path_report(phase.incumbent_path, terrain, cfg, policy, primitives, d_ref)
            found_preferred = bool(q and preferred(q))
            objective_pass = objective_results.get(name, {"pass": True})["pass"]
            status = ("OBJECTIVE PASS + SEARCH PASS" if objective_pass and found_preferred
                      else "OBJECTIVE PASS + SEARCH FAIL" if objective_pass else "OBJECTIVE FAIL")
            row = {"scenario": name, "epsilon": phase.epsilon, "phase": phase, "q": q,
                   "phase_time": phase.cumulative_runtime_s - previous_time, "preferred": found_preferred,
                   "status": status}
            rows.append(row)
            print(f"{name} eps={phase.epsilon:.2f}: first_improve={phase.first_incumbent_improvement_expansion} "
                  f"added/cum={phase.added_expansions}/{phase.cumulative_expansions} "
                  f"time={row['phase_time']:.4f}/{phase.cumulative_runtime_s:.4f}s "
                  f"INCONS={phase.incons_size_at_end} g_impr={phase.g_value_improvement_count} "
                  + (f"cost={q['total']:.6f} len={q['length']:.1f} mean={q['mean_msl']:.1f} min={q['min_msl']:.0f} "
                     f"D/A={q['distance']:.6f}/{q['altitude']:.6f} climb/descent={q['climb']:.0f}/{q['descent']:.0f} "
                     f"safe={q['safe']} preferred={found_preferred} {status}" if q else "NO PATH"))
            previous_time = phase.cumulative_runtime_s
        print(f"  scenario wall={wall:.3f}s total_exp={result.total_expanded} limit={result.refinement_limit_reached}")
    return rows


def consistency_test(config):
    cfg = dataclasses.replace(config, cost_mode="normalized", altitude_reference_msl=400.0,
                              normalized_w_distance=1.0, normalized_w_altitude=1.25,
                              normalized_altitude_scale_m=1000.0)
    prim = next(p for p in build_primitive_set(cfg) if p.direction == "E" and p.primitive_type == "level")
    d_ref = 1200.0
    common = production_mission_policy(400.0).edge_components(prim.horizontal_distance_m, 500.0, 500.0, d_ref)
    fine = compute_edge_cost(prim, 500.0, 0, 0, cfg, d_ref, disable_reversal_cost=True)
    coarse = compute_coarse_edge_cost(prim, 500.0, cfg, d_ref, 400.0, 1000.0, 1.0, 1.25)
    delta = max(abs(common.total - fine), abs(common.total - coarse[0]))
    print(f"\n=== COST CONSISTENCY ===\ncommon={common.total:.12f} fine={fine:.12f} coarse={coarse[0]:.12f} max_delta={delta:.3e} {'PASS' if delta < 1e-12 else 'FAIL'}")
    return delta < 1e-12


def read_state_path(path):
    with open(path, newline="") as f:
        return [(int(r["row"]), int(r["col"]), int(r["z_index"])) for r in csv.DictReader(f)]


def aladaglar_replay(config):
    print("\n=== ALADAGLAR RECORDED PATH REPLAY ONLY (NO SEARCH, w_altitude=1.25) ===")
    cfg = dataclasses.replace(config, cost_mode="normalized", altitude_reference_msl=3240.0,
                              normalized_w_distance=1.0, normalized_w_altitude=1.25,
                              normalized_altitude_scale_m=1000.0)
    terrain = TerrainQuery(load_roi(cfg))
    primitives = build_primitive_set(cfg)
    z = msl_to_z_index(3760.0, cfg)
    start, goal = (48, 276, z), (264, 276, z)
    d_ref = compute_distance_reference(start, goal, terrain, cfg)
    paths = {
        "direct_level": build_direct_level(z),
        "epsilon_1.7_fast": read_state_path("outputs/stage38_ara_first_path.csv"),
        "epsilon_1.3_low": read_state_path("outputs/stage38_1_ara_final_path.csv"),
    }
    policy = production_mission_policy(3240.0)
    for name, path in paths.items():
        q = state_path_report(path, terrain, cfg, policy, primitives, d_ref)
        print(f"{name}: cost={q['total']:.9f} D/A={q['distance']:.9f}/{q['altitude']:.9f} "
              f"len={q['length']:.1f} min/mean={q['min_msl']:.1f}/{q['mean_msl']:.1f} "
              f"climb/descent={q['climb']:.0f}/{q['descent']:.0f} safe={q['safe']}")
    print("epsilon_1.5: unavailable (Stage 38/38.1 did not persist its phase path)")


def write_csvs(objective_rows, search_rows):
    with open("outputs/stage38_3_objective_tests.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["scenario", "candidate", "length_m", "mean_msl", "min_msl", "max_msl", "distance_component",
                    "altitude_component_w1.25", "cost_w1.0", "cost_w1.25", "cost_w1.5", "safety", "expected"])
        for row in objective_rows:
            p = row["weights"][1.25]
            w.writerow([row["scenario"], row["candidate"], p["length"], p["mean_msl"], p["min_msl"], p["max_msl"],
                        p["distance"], p["altitude"], row["weights"][1.0]["total"], p["total"],
                        row["weights"][1.5]["total"], row["safety"]["safe"], row["expected"]])
    with open("outputs/stage38_3_search_tests.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["scenario", "epsilon", "first_improvement_expansion", "added_expansions", "cumulative_expansions",
                    "phase_runtime_s", "cumulative_runtime_s", "mean_msl", "path_length_m", "distance_component",
                    "altitude_component", "cost", "preferred_found", "status", "incons"])
        for row in search_rows:
            p, q = row["phase"], row["q"]
            w.writerow([row["scenario"], p.epsilon, p.first_incumbent_improvement_expansion, p.added_expansions,
                        p.cumulative_expansions, row["phase_time"], p.cumulative_runtime_s,
                        q["mean_msl"] if q else "", q["length"] if q else "", q["distance"] if q else "",
                        q["altitude"] if q else "", q["total"] if q else "", row["preferred"], row["status"],
                        p.incons_size_at_end])


def main():
    config = dataclasses.replace(DEFAULT_CONFIG, normalized_w_altitude=1.25)
    objective_rows, objective_results = run_objective_suite(config)
    print("\n=== THEORETICAL 100m LOWER-MSL BREAKEVEN (at altitude reference) ===")
    for weight in WEIGHTS:
        print(f"w_altitude={weight:.2f}: {100 * weight * 100 / 1000:.1f}% extra distance")
    search_rows = run_search_suite(config, objective_results)
    consistency_test(config)
    aladaglar_replay(config)
    write_csvs(objective_rows, search_rows)
    passed = sum(v["pass"] for v in objective_results.values())
    print(f"\nSUMMARY objective={passed}/{len(objective_results)} PASS; outputs written; production w_altitude=1.25 unchanged")


if __name__ == "__main__":
    main()
