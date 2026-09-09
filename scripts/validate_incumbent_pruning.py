"""Stage 24: incumbent upper-bound / branch-and-bound pruning (planner.astar
astar_search(use_incumbent_pruning=...), validate_and_cost_path()) plus its
interaction with dominance pruning (Stage 23).

10) unit tests for the basic g+h>=incumbent bound.
11) invalid-initial-path test (no architectural dependency on the specific
    direct-level path).
12) incumbent-update test (search finds something better than the initial
    incumbent).
13) optimality test: incumbent OFF vs ON must find the same optimal cost.
14) synthetic 4-way test: {incumbent, dominance} x {OFF, ON}.
15) interaction analysis: does incumbent make dominance worth it?
"""
import dataclasses
import math

import numpy as np
from affine import Affine

from planner.astar import (
    astar_search, msl_to_z_index, validate_and_cost_path,
)
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set
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


# ----------------------------------------------------------------------
# 10) unit tests -- basic g+h>=incumbent bound decision, in isolation
# (no search machinery, just the arithmetic the pruning relies on).
# ----------------------------------------------------------------------

def unit_test_basic_bound() -> bool:
    print("=== 10: unit test -- basic g+h>=incumbent bound ===")
    incumbent = 100.0
    cases = [
        ("A: g=40 h=30 f=70", 40.0, 30.0, incumbent, "KEEP"),
        ("B: g=60 h=40 f=100", 60.0, 40.0, incumbent, "PRUNE"),
        ("C: g=80 h=30 f=110", 80.0, 30.0, incumbent, "PRUNE"),
        ("D: g=40 h=30 f=70, incumbent=inf", 40.0, 30.0, math.inf, "KEEP"),
    ]
    all_ok = True
    for label, g, h, inc, expected in cases:
        f = g + h
        decision = "PRUNE" if f >= inc else "KEEP"
        ok = decision == expected
        all_ok = all_ok and ok
        print(f"  {label}: f={f} incumbent={inc} -> {decision} (expect {expected})  {'PASS' if ok else 'FAIL'}")
    print(f"  Overall: {'PASS' if all_ok else 'FAIL'}")
    return all_ok


# ----------------------------------------------------------------------
# 11) invalid initial path -- validate_and_cost_path must reject it
# (unsafe edge), no architectural dependency on which path this is.
# ----------------------------------------------------------------------

def invalid_initial_path_test(cfg, primitives) -> bool:
    print()
    print("=== 11: invalid initial path test ===")
    # Narrow terrain where a "direct level" chain would clip an obstacle --
    # a wall of high terrain directly in the path's way, below AGL minimum
    # if flown level at 1300m.
    width, height = 20, 3
    elev = np.full((height, width), 1000.0)
    elev[:, 8:12] = 1150.0  # a ridge that eats the level path's AGL margin
    tq = TerrainQuery(make_roi(elev))
    z0 = msl_to_z_index(1300.0, cfg)

    # Hand-build a "direct level" path along row=1 that flies straight through
    # the ridge at a constant altitude -- physically unsafe (AGL < min_agl_m
    # over the ridge: 1300 - 1150 = 150m < 200m).
    candidate_path = [(1, c, z0) for c in range(2, 18)]

    ok, cost = validate_and_cost_path(candidate_path, primitives, tq, cfg)
    rejected_correctly = ok is False and cost == math.inf
    print(f"  candidate path valid={ok} cost={cost}  {'PASS (correctly rejected)' if rejected_correctly else 'FAIL'}")

    # Search must still proceed normally (incumbent_cost=inf) and find *some*
    # feasible path if one exists (e.g. by climbing over the ridge).
    start, goal = (1, 2, z0), (1, 17, z0)
    result = astar_search(start, goal, tq, min_search_altitude_msl=1200.0, max_search_altitude_msl=1500.0,
                           config=cfg, primitives=primitives, max_expansions=5000,
                           use_primitive_cache=True, use_dominance_pruning=False,
                           use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False,
                           use_incumbent_pruning=True,
                           initial_incumbent_cost=cost if ok else math.inf,
                           initial_incumbent_path=candidate_path if ok else None)
    search_ok = result.status == "success" and not result.initial_incumbent_available
    print(f"  search with rejected incumbent: status={result.status} "
          f"initial_incumbent_available={result.initial_incumbent_available} "
          f"final_cost={result.total_cost:.2f}  {'PASS' if search_ok else 'FAIL'}")
    return rejected_correctly and search_ok


# ----------------------------------------------------------------------
# 12) incumbent update test -- search finds something strictly better
# than a deliberately suboptimal (but valid) initial incumbent.
# ----------------------------------------------------------------------

def incumbent_update_test(cfg, primitives) -> bool:
    print()
    print("=== 12: incumbent update test ===")
    # A flat corridor where a level-only path exists (safe but pricey at
    # w_MSL>0 since it never goes low), and a cheaper path exists via
    # descending into a lower strip and cruising there.
    width, height = 40, 5
    elev = np.full((height, width), 1000.0)
    tq = TerrainQuery(make_roi(elev))
    cfg2 = dataclasses.replace(cfg, msl_cost_weight=0.6)
    z_high = msl_to_z_index(1300.0, cfg2)
    start, goal = (2, 2, z_high), (2, 35, z_high)

    initial_path = [(2, c, z_high) for c in range(2, 36)]
    ok, initial_cost = validate_and_cost_path(initial_path, primitives, tq, cfg2)
    assert ok, "initial level path must be valid on flat terrain"

    result = astar_search(start, goal, tq, min_search_altitude_msl=1000.0, max_search_altitude_msl=1320.0,
                           config=cfg2, primitives=primitives, max_expansions=20_000,
                           use_primitive_cache=True, use_dominance_pruning=False,
                           use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False,
                           use_incumbent_pruning=True,
                           initial_incumbent_cost=initial_cost, initial_incumbent_path=initial_path)

    improved = result.total_cost < initial_cost - 1e-6
    updated = result.incumbent_updates >= 1
    ok_final = result.status == "success" and improved and updated
    print(f"  initial_incumbent_cost={initial_cost:.2f}  final total_cost={result.total_cost:.2f}  "
          f"incumbent_updates={result.incumbent_updates}")
    print(f"  strictly improved: {improved}  update counted: {updated}  {'PASS' if ok_final else 'FAIL'}")
    return ok_final


# ----------------------------------------------------------------------
# 13) optimality test -- incumbent OFF vs ON must find the same optimal cost.
# ----------------------------------------------------------------------

def optimality_test(cfg, primitives) -> bool:
    print()
    print("=== 13: optimality test (incumbent OFF vs ON) ===")
    width, height = 30, 3
    elev = np.full((height, width), 1000.0)
    elev[:, 12:] = 1200.0
    tq = TerrainQuery(make_roi(elev))
    z0 = msl_to_z_index(1300.0, cfg)
    start, goal = (1, 2, z0), (1, 27, msl_to_z_index(1400.0, cfg))

    off = astar_search(start, goal, tq, min_search_altitude_msl=1200.0, max_search_altitude_msl=1450.0,
                        config=cfg, primitives=primitives, max_expansions=20_000,
                        use_primitive_cache=True, use_dominance_pruning=False,
                        use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False,
                        use_incumbent_pruning=False)

    # Deliberately weak incumbent (much worse than optimal) to exercise real pruning
    # without accidentally cutting off the optimal branch.
    weak_incumbent = off.total_cost * 3.0
    on = astar_search(start, goal, tq, min_search_altitude_msl=1200.0, max_search_altitude_msl=1450.0,
                       config=cfg, primitives=primitives, max_expansions=20_000,
                       use_primitive_cache=True, use_dominance_pruning=False,
                       use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False,
                       use_incumbent_pruning=True, initial_incumbent_cost=weak_incumbent, initial_incumbent_path=None)

    ok = off.status == on.status == "success" and abs(off.total_cost - on.total_cost) < 1e-6
    print(f"  OFF: cost={off.total_cost:.4f} expanded={off.expanded_nodes}")
    print(f"  ON (weak incumbent={weak_incumbent:.2f}): cost={on.total_cost:.4f} expanded={on.expanded_nodes} "
          f"pruned_candidates={on.incumbent_pruned_candidates}")
    print(f"  same optimal cost: {ok}  {'PASS' if ok else 'FAIL'}")
    return ok


# ----------------------------------------------------------------------
# 14/15) synthetic 4-way test + interaction analysis.
# ----------------------------------------------------------------------

def four_way_test(cfg, primitives):
    print()
    print("=== 14: synthetic 4-way test (incumbent x dominance) ===")
    width, height = 55, 3
    elev = np.full((height, width), 1000.0)
    elev[:, 10:] = 1200.0
    tq = TerrainQuery(make_roi(elev))
    # Initial incumbent: a pure level chain at 1420m -- high enough to clear
    # AGL over the elevated terrain past col 10 (1420-1200=220m >= min_agl_m
    # =200m); start/goal are placed at this same altitude so the level chain
    # is a legitimate, directly reachable complete path from start to goal.
    z0 = msl_to_z_index(1420.0, cfg)
    start, goal = (1, 2, z0), (1, 50, z0)

    initial_path = [(1, c, z0) for c in range(2, 51)]
    ok, initial_cost = validate_and_cost_path(initial_path, primitives, tq, cfg)
    if not ok:
        # fall back: still run without a usable incumbent (inf) rather than fail here
        initial_cost, initial_path = math.inf, None
    print(f"  initial incumbent (direct level path): valid={ok} cost={initial_cost:.2f}" if ok
          else "  initial incumbent: not usable on this terrain (inf)")

    combos = [
        ("A) incumbent OFF, dominance OFF", False, False),
        ("B) incumbent OFF, dominance ON", False, True),
        ("C) incumbent ON,  dominance OFF", True, False),
        ("D) incumbent ON,  dominance ON", True, True),
    ]
    results = {}
    for label, use_incumbent, use_dominance in combos:
        result = astar_search(
            start, goal, tq, min_search_altitude_msl=1300.0, max_search_altitude_msl=1420.0,
            config=cfg, primitives=primitives, max_expansions=50_000,
            use_primitive_cache=True, use_dominance_pruning=use_dominance,
            use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False,
            use_incumbent_pruning=use_incumbent,
            initial_incumbent_cost=initial_cost if use_incumbent else math.inf,
            initial_incumbent_path=initial_path if use_incumbent else None,
        )
        results[label] = result
        print(f"  {label}: status={result.status} cost={result.total_cost:.2f} "
              f"expanded={result.expanded_nodes} max_open={result.max_open_size} "
              f"runtime={result.runtime_s * 1000:.1f}ms generated={result.generated_neighbors} "
              f"incumbent_pruned={result.incumbent_pruned_candidates + result.incumbent_heap_pops_skipped} "
              f"dominance_pruned={result.dominance_pruned_candidates} cache_hit={result.primitive_cache_hit_rate:.3f}")

    costs = [r.total_cost for r in results.values() if r.status == "success"]
    same_cost = len(costs) > 0 and all(abs(c - costs[0]) < 1e-6 for c in costs)
    print(f"  all successful runs found the same optimal cost: {same_cost}")

    print()
    print("=== 15: interaction analysis (C: incumbent ON/dominance OFF vs D: both ON) ===")
    c, d = results["C) incumbent ON,  dominance OFF"], results["D) incumbent ON,  dominance ON"]
    exp_change = 1.0 - d.expanded_nodes / c.expanded_nodes if c.expanded_nodes else 0.0
    open_change = 1.0 - d.max_open_size / c.max_open_size if c.max_open_size else 0.0
    runtime_ratio = d.runtime_s / c.runtime_s if c.runtime_s else float("nan")
    print(f"  expanded change (D vs C): {exp_change * 100:.1f}%  max_open change: {open_change * 100:.1f}%  "
          f"runtime ratio (D/C): {runtime_ratio:.2f}x")
    if (exp_change > 0 or open_change > 0) and runtime_ratio <= 1.05:
        verdict = "dominance now looks net-beneficial once incumbent has already shrunk the search"
    elif runtime_ratio > 1.05:
        verdict = "dominance still costs more wall-clock than it saves, even with incumbent active"
    else:
        verdict = "no clear interaction effect on this synthetic scenario"
    print(f"  verdict: {verdict}")
    return results, same_cost


def main() -> None:
    cfg = dataclasses.replace(DEFAULT_CONFIG, msl_cost_weight=0.63)
    primitives = build_primitive_set(cfg)

    r10 = unit_test_basic_bound()
    r11 = invalid_initial_path_test(cfg, primitives)
    r12 = incumbent_update_test(cfg, primitives)
    r13 = optimality_test(cfg, primitives)
    results, same_cost = four_way_test(cfg, primitives)

    print()
    all_pass = r10 and r11 and r12 and r13 and same_cost
    print(f"Overall: {'ALL PASS' if all_pass else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
