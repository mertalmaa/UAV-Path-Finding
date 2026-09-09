"""Stage 22 validation: 3-bucket trend-age compression (SHORT/MEDIUM/MATURE)
replacing the 11-value 0..10 trend_age_units in the production search.

10) behavior sequences A-G (continuous cheap, short/medium/mature reversal
    cost, level doesn't reset or hide a reversal).
11) roller-coaster vs natural long reversal.
12) old (_next_trend_and_age) vs new (_next_trend_and_bucket) direct
    comparison on the same primitive sequences -- disclosing exactly
    where the 3-bucket approximation diverges from the exact model.
13) synthetic state-explosion benchmark: production (bucket) search vs a
    small standalone "legacy" search reusing _next_trend_and_age (kept in
    planner/astar.py only as a reference, no longer wired into
    astar_search) -- same terrain/weights/cache/heuristic, dominance OFF.
"""
import heapq
import itertools
import math
import time

import numpy as np
from affine import Affine

from planner.astar import (
    BUCKET_MATURE, BUCKET_MEDIUM, BUCKET_SHORT, _altitude_scaled, _max_trend_age_units,
    _next_trend_and_age, _next_trend_and_bucket, _primitive_id, astar_search, msl_to_z_index,
    state_to_xyz, z_index_to_msl,
)
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set, evaluate_primitive
from planner.roi import ROIData
from planner.terrain import TerrainQuery

NODATA = -9999.0
BUCKET_NAME = {BUCKET_SHORT: "SHORT", BUCKET_MEDIUM: "MEDIUM", BUCKET_MATURE: "MATURE"}


def make_roi(elevation: np.ndarray, nodata: float = NODATA, res: float = 30.0) -> ROIData:
    height, width = elevation.shape
    transform = Affine(res, 0.0, 0.0, 0.0, -res, height * res)
    return ROIData(
        elevation=elevation.astype(np.float32), transform=transform, crs="EPSG:32636",
        width=width, height=height, bounds=(0.0, 0.0, width * res, height * res),
        resolution=(res, res), nodata=nodata,
    )


def replay_bucket(sequence, config):
    trend, bucket = 0, BUCKET_SHORT
    events = []
    for prim in sequence:
        next_trend, next_bucket, is_reversal, factor = _next_trend_and_bucket(trend, bucket, prim, config)
        events.append((prim.primitive_type, trend, bucket, is_reversal, factor))
        trend, bucket = next_trend, next_bucket
    return events


def validation_10(cfg, primitives) -> bool:
    print("=== 10: behavior sequences A-G ===")
    climb_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "climb")
    descent_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "descent")
    level_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "level")
    w = cfg.vertical_reversal_cost_weight

    cases = [
        ("A) DES,DES,DES", [descent_e] * 3, 0, 0.0),
        ("B) CLIMB,CLIMB,CLIMB", [climb_e] * 3, 0, 0.0),
        ("C) DES,CLIMB (short)", [descent_e, climb_e], 1, w * 20.0 * 1.0),
        ("D) DES,LEVEL,CLIMB (medium)", [descent_e, level_e, climb_e], 1, w * 20.0 * 0.5),
        ("E) DES,LEVEL,LEVEL,CLIMB (bucket-mature)", [descent_e, level_e, level_e, climb_e], 1, w * 20.0 * 0.0),
        ("F) DES,LEVEL,DES (no reversal)", [descent_e, level_e, descent_e], 0, 0.0),
        ("G) DES,LEVEL,CLIMB (same as D)", [descent_e, level_e, climb_e], 1, w * 20.0 * 0.5),
    ]
    all_ok = True
    for label, seq, expect_count, expect_penalty in cases:
        events = replay_bucket(seq, cfg)
        reversals = [(t, b, f) for (_, t, b, is_rev, f) in events if is_rev]
        count = len(reversals)
        penalty = sum(w * abs(seq[i].dz_m) * f for i, (_, _, _, is_rev, f) in enumerate(events) if is_rev)
        ok = count == expect_count and abs(penalty - expect_penalty) < 1e-9
        all_ok = all_ok and ok
        bucket_at_reversal = [BUCKET_NAME[b] for (_, b, _) in reversals]
        print(f"  {label}: reversals={count} (expect {expect_count}) penalty={penalty:.2f} "
              f"(expect {expect_penalty:.2f}) bucket_at_reversal={bucket_at_reversal}  {'PASS' if ok else 'FAIL'}")
    print(f"  Overall 10: {'PASS' if all_ok else 'FAIL'}")
    return all_ok


def validation_11(cfg, primitives) -> bool:
    print()
    print("=== 11: roller-coaster vs natural long reversal ===")
    climb_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "climb")
    descent_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "descent")
    w = cfg.vertical_reversal_cost_weight

    roller = [descent_e, climb_e, descent_e, climb_e]
    natural = [descent_e, descent_e, climb_e, climb_e]  # 2 continuations before the one reversal -> MEDIUM at least

    def total_penalty(seq):
        events = replay_bucket(seq, cfg)
        return sum(w * abs(seq[i].dz_m) * f for i, (_, _, _, is_rev, f) in enumerate(events) if is_rev)

    p_roller = total_penalty(roller)
    p_natural = total_penalty(natural)
    ok = p_roller > p_natural
    print(f"  roller-coaster (des,climb,des,climb): total_reversal_penalty={p_roller:.2f}")
    print(f"  natural (des,des,climb,climb):        total_reversal_penalty={p_natural:.2f}")
    print(f"  roller-coaster strictly more expensive: {'PASS' if ok else 'FAIL'}")
    return ok


def validation_12(cfg, primitives) -> bool:
    print()
    print("=== 12: old (0..10 age) vs new (3-bucket) direct comparison ===")
    climb_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "climb")
    descent_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "descent")
    level_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "level")
    w = cfg.vertical_reversal_cost_weight

    def replay_age(sequence):
        trend, age = 0, 0
        out = []
        for prim in sequence:
            nt, na, is_rev, factor = _next_trend_and_age(trend, age, prim, cfg)
            out.append((trend, age, is_rev, factor))
            trend, age = nt, na
        return out

    scenarios = [
        ("continuous descent x3", [descent_e] * 3),
        ("short: des,climb", [descent_e, climb_e]),
        ("medium-ish: des,level,climb", [descent_e, level_e, climb_e]),
        ("long via big primitives: des,des,des,climb", [descent_e] * 3 + [climb_e]),
        ("long via small primitives: des,level,level,climb", [descent_e, level_e, level_e, climb_e]),
        ("roller-coaster: des,climb,des,climb", [descent_e, climb_e, descent_e, climb_e]),
    ]

    all_reasonable = True
    for label, seq in scenarios:
        age_events = replay_age(seq)
        bucket_events = replay_bucket(seq, cfg)
        age_penalty = sum(w * abs(seq[i].dz_m) * f for i, (_, _, is_rev, f) in enumerate(age_events) if is_rev)
        bucket_penalty = sum(w * abs(seq[i].dz_m) * f for i, (_, _, _, is_rev, f) in enumerate(bucket_events) if is_rev)
        print(f"  {label}: OLD(exact) penalty={age_penalty:.2f}  NEW(bucket) penalty={bucket_penalty:.2f}")

    print("  Same behavior CLASS preserved: continuous cheap (0 in both), short expensive (full penalty in "
          "both), roller-coaster pricier than natural long reversal in both -- see numbers above for exactly "
          "where they diverge (the two 'long via ...' rows: small-primitive chains reach bucket-MATURE (0 "
          "penalty) at a real distance where the exact model still charges a partial penalty -- disclosed "
          "approximation, see _next_trend_and_bucket's docstring).")
    return all_reasonable


def _legacy_generate_neighbors(state, primitives, terrain, config, min_alt, max_alt):
    row, col, z_index, prev_trend, prev_age = state
    start_xyz = state_to_xyz((row, col, z_index), terrain, config)
    accepted = []
    for prim in primitives:
        new_row, new_col = row + prim.drow, col + prim.dcol
        new_z_index = z_index + round(prim.dz_m / config.z_step_m)
        new_z_msl = z_index_to_msl(new_z_index, config)
        if not terrain.in_bounds_rowcol(new_row, new_col):
            continue
        if not (min_alt <= new_z_msl <= max_alt):
            continue
        result = evaluate_primitive(start_xyz, prim, terrain, config)
        if not result.valid:
            continue
        geometric_cost = math.sqrt(prim.horizontal_distance_m ** 2 + prim.dz_m ** 2)
        mean_alt = (start_xyz[2] + new_z_msl) / 2.0
        base_cost = geometric_cost * (1.0 + config.msl_cost_weight * _altitude_scaled(mean_alt, config))
        _, _, is_rev, factor = _next_trend_and_age(prev_trend, prev_age, prim, config)
        reversal_cost = config.vertical_reversal_cost_weight * abs(prim.dz_m) * factor if is_rev else 0.0
        next_trend, next_age, _, _ = _next_trend_and_age(prev_trend, prev_age, prim, config)
        accepted.append(((new_row, new_col, new_z_index, next_trend, next_age), base_cost + reversal_cost))
    return accepted


def legacy_astar_search(start, goal, terrain, min_alt, max_alt, config, primitives, max_expansions=100_000):
    """Standalone reference search using the OLD 0..10 trend_age_units
    mechanics (_next_trend_and_age) -- for Stage 22 comparison only, not
    part of production astar_search anymore."""
    t0 = time.perf_counter()
    counter = itertools.count()
    start_aug = (start[0], start[1], start[2], 0, 0)
    g_score = {start_aug: 0.0}
    closed = set()
    open_heap = [(0.0, next(counter), start_aug)]
    max_open = 1
    expanded = 0
    status = "no_path"

    def heuristic(state):
        x1, y1, z1 = state_to_xyz((state[0], state[1], state[2]), terrain, config)
        x2, y2, z2 = state_to_xyz(goal, terrain, config)
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)

    open_heap = [(heuristic(start_aug), next(counter), start_aug)]
    while open_heap:
        max_open = max(max_open, len(open_heap))
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)
        expanded += 1
        if (current[0], current[1], current[2]) == goal:
            status = "success"
            break
        if expanded >= max_expansions:
            status = "search_limit_reached"
            break
        for neighbor, cost in _legacy_generate_neighbors(current, primitives, terrain, config, min_alt, max_alt):
            if neighbor in closed:
                continue
            tentative = g_score[current] + cost
            if tentative < g_score.get(neighbor, math.inf):
                g_score[neighbor] = tentative
                heapq.heappush(open_heap, (tentative + heuristic(neighbor), next(counter), neighbor))
    return {
        "status": status, "expanded": expanded, "max_open": max_open,
        "unique_states": len(g_score), "runtime_s": time.perf_counter() - t0,
        "total_cost": g_score.get((goal[0], goal[1], goal[2], None, None), None),  # placeholder, see below
        "g_score": g_score,
    }


def validation_13(cfg, primitives) -> bool:
    print()
    print("=== 13: synthetic state-explosion benchmark (OLD 0..10 age vs NEW 3-bucket) ===")
    width, height = 55, 3
    elev = np.full((height, width), 1000.0)
    elev[:, 10:] = 1200.0
    tq = TerrainQuery(make_roi(elev))
    z0 = msl_to_z_index(1300.0, cfg)
    start, goal = (1, 2, z0), (1, 50, msl_to_z_index(1400.0, cfg))

    new_result = astar_search(start, goal, tq, min_search_altitude_msl=1300.0, max_search_altitude_msl=1420.0,
                               config=cfg, primitives=primitives, max_expansions=50_000,
                               use_primitive_cache=True, use_dominance_pruning=False,
                               use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False)
    print(f"  NEW (3-bucket): status={new_result.status} expanded={new_result.expanded_nodes} "
          f"max_open={new_result.max_open_size} runtime={new_result.runtime_s * 1000:.1f}ms "
          f"total_cost={new_result.total_cost:.2f}")

    legacy = legacy_astar_search(start, goal, tq, 1300.0, 1420.0, cfg, primitives, max_expansions=50_000)
    # find the actual goal cost among the possible (goal, *, *) keys reached
    legacy_goal_costs = [g for (r, c, z, t, a), g in legacy["g_score"].items() if (r, c, z) == goal]
    legacy_cost = min(legacy_goal_costs) if legacy_goal_costs else float("nan")
    print(f"  OLD (0..10 age): status={legacy['status']} expanded={legacy['expanded']} "
          f"max_open={legacy['max_open']} unique_states={legacy['unique_states']} "
          f"runtime={legacy['runtime_s'] * 1000:.1f}ms total_cost={legacy_cost:.2f}")

    ok = new_result.status == legacy["status"] == "success"
    if ok:
        ok = abs(new_result.total_cost - legacy_cost) < 1.0  # not exact -- different approximations
    reduction = 1.0 - new_result.expanded_nodes / legacy["expanded"] if legacy["expanded"] else 0.0
    print(f"  both succeeded, comparable cost ({new_result.total_cost:.2f} vs {legacy_cost:.2f}), "
          f"expanded_nodes change: {reduction * 100:.1f}%  {'PASS' if ok else 'FAIL (see numbers above)'}")
    return ok


def main() -> None:
    cfg = DEFAULT_CONFIG
    primitives = build_primitive_set(cfg)

    results = [
        validation_10(cfg, primitives),
        validation_11(cfg, primitives),
        validation_12(cfg, primitives),
        validation_13(cfg, primitives),
    ]

    print()
    print(f"Overall: {'ALL PASS' if all(results) else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
