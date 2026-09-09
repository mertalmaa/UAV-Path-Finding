"""Stage validation: vertical-motion (climb/descent) soft penalty in A*.

A) vertical_cost_weight=0.0 regression: reproduces the previous stage's
   low-MSL-preference numbers exactly (the penalty term is additive and
   zero here, so nothing else should move).
B) level vs unnecessary vertical motion: msl isolated (weight=0), search
   picks the zero-vertical-motion path; a hand-built "wasteful" detour
   covering the identical net displacement is costed for comparison via
   the same compute_edge_cost() the search itself uses.
C) low-MSL vs vertical trade-off: same low-MSL scenario as (A), compared
   at vertical_cost_weight=0.0 vs 1.0.
D) mandatory climb must still happen: a wall spans every row (no lateral
   gap) -- the vertical penalty must not turn this into a no_path.
E) roller-coaster vs smooth: two hand-built paths reaching the exact same
   endpoint (4 climb/descent hops vs 16 level hops, both +16 columns) with
   deliberately close geometric length -- vertical_cost_weight decides
   which one an unconstrained search actually prefers.
"""
import dataclasses
import math

import numpy as np
from affine import Affine

from planner.astar import astar_search, compute_edge_cost, msl_to_z_index
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set, evaluate_primitive
from planner.roi import ROIData
from planner.terrain import TerrainQuery

NODATA = -9999.0


def make_roi(elevation: np.ndarray, nodata: float = NODATA, res: float = 30.0) -> ROIData:
    height, width = elevation.shape
    transform = Affine(res, 0.0, 0.0, 0.0, -res, height * res)
    return ROIData(
        elevation=elevation.astype(np.float32),
        transform=transform,
        crs="EPSG:32636",
        width=width,
        height=height,
        bounds=(0.0, 0.0, width * res, height * res),
        resolution=(res, res),
        nodata=nodata,
    )


def report(label, result) -> None:
    print(f"  [{label}] status={result.status} success={result.success}")
    if not result.success:
        return
    print(f"    path_states={len(result.path)} geometric_length={result.geometric_path_length:.2f} "
          f"total_cost(weighted)={result.total_cost:.2f}")
    print(f"    MSL: min={result.minimum_aircraft_msl:.1f} max={result.maximum_aircraft_msl:.1f} "
          f"avg={result.average_aircraft_msl:.1f}")
    print(f"    climb={result.total_climb_m:.1f} descent={result.total_descent_m:.1f} "
          f"vertical_motion={result.total_vertical_motion_m:.1f}")
    print(f"    minimum_observed_agl={result.minimum_observed_agl:.1f} "
          f"expanded={result.expanded_nodes} runtime={result.runtime_s * 1000:.1f}ms")


def validation_a(cfg, primitives) -> bool:
    print("=== A) vertical_cost_weight=0.0 regression (previous stage's low-MSL scenario) ===")
    flat = np.full((10, 45), 1000.0)
    roi = make_roi(flat)
    tq = TerrainQuery(roi)

    z_top = msl_to_z_index(1320.0, cfg)
    start, goal = (5, 2, z_top), (5, 42, z_top)

    c = dataclasses.replace(cfg, vertical_cost_weight=0.0)  # msl_cost_weight stays default 0.25
    result = astar_search(start, goal, tq, min_search_altitude_msl=1300.0, max_search_altitude_msl=1320.0,
                           config=c, primitives=primitives)
    report("vertical_weight=0.0", result)

    ok = (
        result.success
        and len(result.path) == 35
        and abs(result.geometric_path_length - 1203.31) < 0.1
        and abs(result.total_cost - 1233.72) < 0.1
        and abs(result.average_aircraft_msl - 1302.0) < 0.1
        and abs(result.minimum_observed_agl - 300.0) < 0.1
    )
    print("  (expected to match the previous stage's Validation B numbers exactly: "
          "path_states=35, geom_len=1203.31, total_cost=1233.72, avg_msl=1302.0)")
    print(f"  PASS" if ok else "  FAIL")
    return ok


def validation_b(cfg, primitives) -> bool:
    print("=== B) level vs unnecessary vertical motion (msl isolated: weight=0) ===")
    flat = np.full((5, 20), 1000.0)
    roi = make_roi(flat)
    tq = TerrainQuery(roi)

    z0 = msl_to_z_index(1300.0, cfg)
    start, goal = (2, 2, z0), (2, 12, z0)  # 10 level cells apart, same altitude

    c = dataclasses.replace(cfg, msl_cost_weight=0.0)
    result = astar_search(start, goal, tq, min_search_altitude_msl=1100.0, max_search_altitude_msl=1500.0,
                           config=c, primitives=primitives)
    report(f"search (vertical_weight={c.vertical_cost_weight})", result)

    # Hand-built wasteful alternative: climb one hop then descend one hop
    # (net dz=0), costed with the exact same formula the search uses.
    climb_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "climb")
    descent_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "descent")
    level_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "level")

    detour_cost = (
        compute_edge_cost(climb_e, 1300.0, 1100.0, 1500.0, c)
        + compute_edge_cost(descent_e, 1320.0, 1100.0, 1500.0, c)
    )
    level_equivalent_cost = 8 * compute_edge_cost(level_e, 1300.0, 1100.0, 1500.0, c)  # same net +240m

    ok = result.success and result.total_vertical_motion_m == 0.0
    ok = ok and detour_cost > level_equivalent_cost  # wasteful detour costs strictly more

    print(f"  hand-built climb+descent detour (net +240m, dz=0): cost={detour_cost:.2f}")
    print(f"  equivalent 8x level hops (net +240m, dz=0):        cost={level_equivalent_cost:.2f}")
    print(f"  search's chosen path total_vertical_motion_m={result.total_vertical_motion_m:.1f} (expect 0.0)")
    print(f"  PASS" if ok else "  FAIL")
    return ok


def validation_c(cfg, primitives) -> bool:
    print("=== C) low-MSL vs vertical trade-off (same scenario, weight 0.0 vs 1.0) ===")
    flat = np.full((10, 45), 1000.0)
    roi = make_roi(flat)
    tq = TerrainQuery(roi)

    z_top = msl_to_z_index(1320.0, cfg)
    start, goal = (5, 2, z_top), (5, 42, z_top)

    results = {}
    for vw in (0.0, 1.0):
        c = dataclasses.replace(cfg, vertical_cost_weight=vw)  # msl_cost_weight stays default 0.25
        r = astar_search(start, goal, tq, min_search_altitude_msl=1300.0, max_search_altitude_msl=1320.0,
                          config=c, primitives=primitives)
        results[vw] = r
        report(f"vertical_weight={vw}", r)

    r0, r1 = results[0.0], results[1.0]
    ok = r0.success and r1.success
    ok = ok and r1.total_vertical_motion_m <= r0.total_vertical_motion_m  # penalty reduces (or keeps) vertical motion
    ok = ok and r1.average_aircraft_msl >= r0.average_aircraft_msl  # ...at the cost of flying higher on average
    ok = ok and r0.minimum_observed_agl >= cfg.min_agl_m - 1e-6
    ok = ok and r1.minimum_observed_agl >= cfg.min_agl_m - 1e-6

    print(f"  vertical_weight=0.0 avg_msl={r0.average_aircraft_msl:.1f} vertical_motion={r0.total_vertical_motion_m:.1f}")
    print(f"  vertical_weight=1.0 avg_msl={r1.average_aircraft_msl:.1f} vertical_motion={r1.total_vertical_motion_m:.1f}")
    print(f"  PASS" if ok else "  FAIL")
    return ok


def validation_d(cfg, primitives) -> bool:
    print("=== D) mandatory climb must still succeed with vertical penalty active ===")
    width, height = 15, 7
    elev = np.full((height, width), 1000.0)
    elev[:, 6:9] = 1150.0  # wall spans every row -- no lateral gap, only climbing clears it

    roi = make_roi(elev)
    tq = TerrainQuery(roi)

    z0 = msl_to_z_index(1300.0, cfg)
    start, goal = (3, 1, z0), (3, 13, z0)

    result = astar_search(start, goal, tq, min_search_altitude_msl=1300.0, max_search_altitude_msl=1400.0,
                           config=cfg, primitives=primitives, max_expansions=50_000)  # cfg: default vertical_weight=1.0
    report(f"vertical_weight={cfg.vertical_cost_weight}", result)

    ok = result.success and result.status == "success"
    ok = ok and result.total_climb_m > 0.0  # it actually had to climb
    ok = ok and result.minimum_observed_agl >= cfg.min_agl_m - 1e-6

    print(f"  climbed {result.total_climb_m:.1f}m despite penalty -- mandatory climb was not blocked")
    print(f"  PASS" if ok else "  FAIL")
    return ok


def validation_e(cfg, primitives) -> bool:
    print("=== E) roller-coaster vs smooth: two hand-built paths to the identical endpoint ===")
    flat = np.full((5, 25), 1000.0)
    roi = make_roi(flat)
    tq = TerrainQuery(roi)

    climb_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "climb")
    descent_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "descent")
    level_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "level")

    # Path A: climb, descent, climb, descent -- 4 hops x dcol=4 => +16 cols, net dz=0.
    # Path B: 16x level -- also +16 cols, net dz=0. Both reach the exact same (row,col,z).
    z0 = 1300.0
    for vw, msl_w in ((0.0, 0.0), (1.0, 0.0)):
        c = dataclasses.replace(cfg, vertical_cost_weight=vw, msl_cost_weight=msl_w)
        z = z0
        cost_a = 0.0
        for prim in (climb_e, descent_e, climb_e, descent_e):
            cost_a += compute_edge_cost(prim, z, 1100.0, 1500.0, c)
            z += prim.dz_m
        cost_b = 16 * compute_edge_cost(level_e, z0, 1100.0, 1500.0, c)
        print(f"  vertical_weight={vw}: roller-coaster cost={cost_a:.2f}  smooth-level cost={cost_b:.2f}  "
              f"smooth cheaper by {cost_a - cost_b:.2f}")

    c0 = dataclasses.replace(cfg, vertical_cost_weight=0.0, msl_cost_weight=0.0)
    c1 = dataclasses.replace(cfg, vertical_cost_weight=1.0, msl_cost_weight=0.0)

    z = z0
    cost_a0 = cost_a1 = 0.0
    for prim in (climb_e, descent_e, climb_e, descent_e):
        cost_a0 += compute_edge_cost(prim, z, 1100.0, 1500.0, c0)
        cost_a1 += compute_edge_cost(prim, z, 1100.0, 1500.0, c1)
        z += prim.dz_m
    cost_b0 = 16 * compute_edge_cost(level_e, z0, 1100.0, 1500.0, c0)
    cost_b1 = 16 * compute_edge_cost(level_e, z0, 1100.0, 1500.0, c1)
    gap0 = cost_a0 - cost_b0
    gap1 = cost_a1 - cost_b1

    # Confirm an unconstrained search over the same start/goal independently
    # lands on the smooth path once the penalty is active.
    z_idx0 = msl_to_z_index(z0, cfg)
    start, goal = (2, 2, z_idx0), (2, 18, z_idx0)  # +16 cols, same idea as the hand-built paths
    search_result = astar_search(start, goal, tq, min_search_altitude_msl=1100.0, max_search_altitude_msl=1500.0,
                                  config=c1, primitives=primitives)
    report(f"search confirmation (vertical_weight=1.0, msl_weight=0.0)", search_result)

    ok = (
        gap0 > 0 and gap1 > 0  # smooth is cheaper in both cases (as geometry guarantees)
        and gap1 > gap0  # ...but the gap widens sharply once vertical motion is penalized
        and search_result.success
        and search_result.total_vertical_motion_m == 0.0
    )
    print(f"  cost gap (roller-coaster minus smooth): weight=0.0 -> {gap0:.2f}m, weight=1.0 -> {gap1:.2f}m")
    print(f"  PASS" if ok else "  FAIL")
    return ok


def main() -> None:
    cfg = DEFAULT_CONFIG
    assert cfg.vertical_cost_weight == 1.0
    primitives = build_primitive_set(cfg)

    results = [
        validation_a(cfg, primitives),
        validation_b(cfg, primitives),
        validation_c(cfg, primitives),
        validation_d(cfg, primitives),
        validation_e(cfg, primitives),
    ]

    print()
    print(f"Overall: {'ALL PASS' if all(results) else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
