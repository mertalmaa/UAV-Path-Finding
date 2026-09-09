"""Stage 17 validation: admissible MSL-aware lower-bound heuristic.

h(n) = D3D(n, goal) * minimum_cost_multiplier, where minimum_cost_multiplier
is a single GLOBAL constant (computed once per search, from the cheapest
MSL this search could possibly ever fly at) such that no valid edge can
ever cost less than geometric_cost * minimum_cost_multiplier. Admissible
and consistent for the same reason the old plain-Euclidean heuristic was
(triangle inequality), given non-negative msl_cost_weight,
vertical_reversal_cost_weight, and a positive msl_scale_m -- verified
below, and checked defensively at runtime (falls back to multiplier=1.0
otherwise).

9)  _min_possible_aircraft_msl unit tests (the exact spec examples).
10) numeric heuristic test: new_h >= old_h, equal when w_MSL=0 or
    scaled_msl_lb=0.
11) admissibility/consistency synthetic test across many state/primitive
    combinations: h(n) <= edge_cost(n,n') + h(n'), 0 violations expected.
    Plus OFF-vs-ON correctness on a small completable A* scenario.
13) synthetic performance benchmark (heuristic OFF vs ON).
"""
import dataclasses
import math

import numpy as np
from affine import Affine

from planner.astar import (
    _heuristic, _min_possible_aircraft_msl, _msl_lower_bound_multiplier, _terrain_min_valid_elevation,
    astar_search, compute_edge_cost, msl_to_z_index,
)
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set, evaluate_primitive
from planner.roi import ROIData
from planner.terrain import TerrainQuery

NODATA = -9999.0


def make_roi(elevation: np.ndarray, nodata: float = NODATA, res: float = 30.0) -> ROIData:
    height, width = elevation.shape
    transform = Affine(res, 0.0, 0.0, 0.0, -res, height * res)
    return ROIData(
        elevation=elevation.astype(np.float32), transform=transform, crs="EPSG:32636",
        width=width, height=height, bounds=(0.0, 0.0, width * res, height * res),
        resolution=(res, res), nodata=nodata,
    )


def validation_9(cfg) -> bool:
    print("=== 9: minimum_possible_aircraft_msl unit tests ===")
    cases = [
        ("terrain_min=1700, min_agl=200, search_min=1800", 1700.0, 1800.0, 1900.0),
        ("terrain_min=1000, min_agl=200, search_min=1500", 1000.0, 1500.0, 1500.0),
    ]
    ok = True
    for label, terrain_min, search_min, expected in cases:
        actual = _min_possible_aircraft_msl(terrain_min, search_min, cfg)
        this_ok = actual == expected
        ok = ok and this_ok
        print(f"  {label}: got {actual} (expect {expected})  {'PASS' if this_ok else 'FAIL'}")

    # NoData must not corrupt the terrain-min scan.
    elev = np.full((5, 5), 1000.0)
    elev[2, 2] = NODATA
    elev[0, 0] = 850.0  # the real minimum, among valid cells
    tq = TerrainQuery(make_roi(elev))
    terrain_min = _terrain_min_valid_elevation(tq)
    this_ok = terrain_min == 850.0
    ok = ok and this_ok
    print(f"  NoData-excluded terrain min: got {terrain_min} (expect 850.0)  {'PASS' if this_ok else 'FAIL'}")

    print(f"  Overall 9: {'PASS' if ok else 'FAIL'}")
    return ok


def validation_10(cfg) -> bool:
    print()
    print("=== 10: numeric heuristic test (new_h vs old_h) ===")
    flat = np.full((5, 20), 1000.0)
    tq = TerrainQuery(make_roi(flat))
    z0 = msl_to_z_index(1300.0, cfg)
    current = (2, 2, z0, 0, 0)
    goal = (2, 15, z0)

    old_h = _heuristic(current, goal, tq, cfg, 1.0)

    min_msl = _min_possible_aircraft_msl(_terrain_min_valid_elevation(tq), 1100.0, cfg)
    mult = _msl_lower_bound_multiplier(min_msl, cfg)
    new_h = _heuristic(current, goal, tq, cfg, mult)

    ok = new_h >= old_h - 1e-9
    print(f"  old_h={old_h:.3f}  new_h={new_h:.3f} (multiplier={mult:.4f})  new_h>=old_h: {'PASS' if ok else 'FAIL'}")

    # w_MSL=0 -> multiplier must be exactly 1.0 -> new_h == old_h
    c0 = dataclasses.replace(cfg, msl_cost_weight=0.0)
    mult0 = _msl_lower_bound_multiplier(min_msl, c0)
    h0 = _heuristic(current, goal, tq, c0, mult0)
    ok2 = mult0 == 1.0 and h0 == old_h
    print(f"  w_MSL=0: multiplier={mult0} new_h={h0:.3f} == old_h={old_h:.3f}: {'PASS' if ok2 else 'FAIL'}")

    # scaled_msl_lb=0 (min_possible_msl <= msl_reference_m) -> multiplier == 1.0 -> new_h == old_h
    mult_zero_lb = _msl_lower_bound_multiplier(0.0, cfg)  # at or below msl_reference_m=0.0
    h_zero_lb = _heuristic(current, goal, tq, cfg, mult_zero_lb)
    ok3 = mult_zero_lb == 1.0 and h_zero_lb == old_h
    print(f"  scaled_msl_lb=0: multiplier={mult_zero_lb} new_h={h_zero_lb:.3f} == old_h={old_h:.3f}: "
          f"{'PASS' if ok3 else 'FAIL'}")

    ok_all = ok and ok2 and ok3
    print(f"  Overall 10: {'PASS' if ok_all else 'FAIL'}")
    return ok_all


def validation_11(cfg, primitives) -> bool:
    print()
    print("=== 11: admissibility/consistency synthetic test + OFF/ON correctness ===")
    width, height = 60, 15
    flat = np.full((height, width), 1000.0)
    tq = TerrainQuery(make_roi(flat))
    goal = (7, 55, msl_to_z_index(1300.0, cfg))

    min_msl = _min_possible_aircraft_msl(_terrain_min_valid_elevation(tq), 1100.0, cfg)
    mult = _msl_lower_bound_multiplier(min_msl, cfg)
    print(f"  minimum_possible_aircraft_msl={min_msl}  multiplier={mult:.4f}")

    violations = 0
    checked = 0
    z0 = msl_to_z_index(1300.0, cfg)
    for row in range(3, height - 3, 3):
        for col in range(2, 50, 4):
            for trend in (-1, 0, 1):
                for age in (0, 4, 8, 10):
                    current = (row, col, z0, trend, age)
                    h_current = _heuristic(current, goal, tq, cfg, mult)
                    x, y = tq.rowcol_to_xy(row, col)
                    start_xyz = (x, y, 1300.0)
                    for prim in primitives:
                        result = evaluate_primitive(start_xyz, prim, tq, cfg)
                        if not result.valid:
                            continue
                        edge_cost = compute_edge_cost(prim, 1300.0, trend, age, cfg)
                        from planner.astar import _next_trend_and_age
                        next_trend, next_age, _, _ = _next_trend_and_age(trend, age, prim, cfg)
                        new_row, new_col = row + prim.drow, col + prim.dcol
                        new_z = z0 + round(prim.dz_m / cfg.z_step_m)
                        neighbor = (new_row, new_col, new_z, next_trend, next_age)
                        h_neighbor = _heuristic(neighbor, goal, tq, cfg, mult)
                        checked += 1
                        if h_current > edge_cost + h_neighbor + 1e-9:
                            violations += 1

    ok_consistency = violations == 0
    print(f"  checked {checked} (state, feasible primitive) combinations: {violations} consistency violations  "
          f"{'PASS' if ok_consistency else 'FAIL'}")

    # Small completable A* scenario, OFF vs ON.
    start = (7, 2, z0)
    goal_state = (7, 55, z0)
    r_off = astar_search(start, goal_state, tq, min_search_altitude_msl=1100.0, max_search_altitude_msl=1500.0,
                          config=cfg, primitives=primitives, use_msl_lower_bound_heuristic=False)
    r_on = astar_search(start, goal_state, tq, min_search_altitude_msl=1100.0, max_search_altitude_msl=1500.0,
                         config=cfg, primitives=primitives, use_msl_lower_bound_heuristic=True)
    ok_correctness = (
        r_off.status == r_on.status == "success"
        and abs(r_off.total_cost - r_on.total_cost) < 1e-9
        and r_off.path[-1] == r_on.path[-1] == goal_state
    )
    print(f"  OFF: cost={r_off.total_cost:.3f} expanded={r_off.expanded_nodes} | "
          f"ON: cost={r_on.total_cost:.3f} expanded={r_on.expanded_nodes} multiplier={r_on.heuristic_cost_multiplier:.4f}  "
          f"{'PASS' if ok_correctness else 'FAIL'}")

    ok = ok_consistency and ok_correctness
    print(f"  Overall 11: {'PASS' if ok else 'FAIL'}")
    return ok


def validation_13(cfg, primitives) -> bool:
    print()
    print("=== 13: synthetic performance benchmark (heuristic OFF vs ON) ===")
    width, height = 55, 3
    elev = np.full((height, width), 1000.0)
    elev[:, 10:] = 1200.0  # forces a climb -- same shape used in Stage 15/16 benchmarks
    tq = TerrainQuery(make_roi(elev))
    z0 = msl_to_z_index(1300.0, cfg)
    start, goal = (1, 2, z0), (1, 50, msl_to_z_index(1400.0, cfg))

    results = {}
    for label, use_h in (("OFF", False), ("ON", True)):
        r = astar_search(start, goal, tq, min_search_altitude_msl=1300.0, max_search_altitude_msl=1420.0,
                          config=cfg, primitives=primitives, max_expansions=50_000,
                          use_primitive_cache=True, use_dominance_pruning=False, use_msl_lower_bound_heuristic=use_h)
        results[label] = r
        print(f"  {label}: status={r.status} total_cost={r.total_cost:.3f} expanded={r.expanded_nodes} "
              f"max_open={r.max_open_size} runtime={r.runtime_s * 1000:.1f}ms "
              f"actual_calls={r.actual_evaluate_primitive_calls} hit_rate={r.primitive_cache_hit_rate:.3f} "
              f"multiplier={r.heuristic_cost_multiplier:.4f}")

    r_off, r_on = results["OFF"], results["ON"]
    ok = (
        r_off.status == r_on.status
        and abs(r_off.total_cost - r_on.total_cost) < 1e-9
        and r_on.expanded_nodes <= r_off.expanded_nodes
    )
    reduction = 1.0 - r_on.expanded_nodes / r_off.expanded_nodes if r_off.expanded_nodes else 0.0
    print(f"  same optimal cost, expanded_nodes reduced by {reduction * 100:.1f}%  {'PASS' if ok else 'FAIL'}")
    return ok


def main() -> None:
    cfg = DEFAULT_CONFIG
    primitives = build_primitive_set(cfg)

    results = [
        validation_9(cfg),
        validation_10(cfg),
        validation_11(cfg, primitives),
        validation_13(cfg, primitives),
    ]

    print()
    print(f"Overall: {'ALL PASS' if all(results) else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
