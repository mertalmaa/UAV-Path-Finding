"""Stage 21 validation: generic vertical-reachability MSL heuristic.

h_forward is derived from a RELAXATION of the real problem (straight-line
travel of exactly D3D length, altitude free to descend at up to
sin(max_descent_angle_deg) per unit arc length down to the global floor,
endpoint altitude unconstrained) -- admissible because truncating the
integral to D never overestimates (any real, possibly-longer-via-detour
path only adds non-negative cost on top), and consistent because for any
real edge (n, n'), prefixing n''s own optimal relaxed continuation with
that edge is itself a valid candidate for n's relaxed problem (proof in
planner/astar.py's docstring / project.md "Stage 21").

10) angle-generic unit test: 5/10/20/30 deg envelopes, monotonic ordering.
11) continuous-compatibility: works for non-primitive-aligned angles too,
    using only config.max_descent_angle_deg.
    Plus a direct admissibility/consistency sweep over many
    (state, primitive) combinations -- 0 violations expected.
12) small optimality test: heuristic OFF vs ON, same optimal cost.
13) synthetic search-explosion benchmark: Stage 17 heuristic vs Stage 21.
"""
import dataclasses
import math

import numpy as np
from affine import Affine

from planner.astar import (
    _forward_vertical_reachability_heuristic, _heuristic, _min_possible_aircraft_msl,
    _msl_lower_bound_multiplier, _next_trend_and_age, _terrain_min_valid_elevation,
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


def validation_10(cfg) -> bool:
    print("=== 10: angle-generic unit test ===")
    current_msl, D, floor = 3760.0, 6480.0, 3240.0
    values = []
    for angle in (5.0, 10.0, 20.0, 30.0):
        c = dataclasses.replace(cfg, max_descent_angle_deg=angle)
        h = _forward_vertical_reachability_heuristic(current_msl, D, floor, c)
        values.append(h)
        print(f"  max_descent_angle_deg={angle}: h_forward={h:.2f}")
    ok = all(values[i] >= values[i + 1] - 1e-9 for i in range(len(values) - 1))
    print(f"  monotonically non-increasing as angle grows (5>=10>=20>=30): {'PASS' if ok else 'FAIL'}")
    return ok


def validation_11(cfg, primitives) -> bool:
    print()
    print("=== 11: continuous-compatibility + admissibility/consistency sweep ===")
    # Non-primitive-aligned angles -- proves the formula reads only config,
    # never a primitive's own discrete angle set.
    for angle in (7.5, 13.3, 22.5, 28.9):
        c = dataclasses.replace(cfg, max_descent_angle_deg=angle)
        h = _forward_vertical_reachability_heuristic(3760.0, 6480.0, 3240.0, c)
        print(f"  max_descent_angle_deg={angle} (non-primitive-aligned): h_forward={h:.2f}  (no crash, computed)")
    print("  Primitive set is untouched by any of this -- the heuristic never reads a primitive's own angle, "
          "only config.max_descent_angle_deg. A future continuous 0..30deg system needs no change here.")

    width, height = 60, 15
    flat = np.full((height, width), 1000.0)
    tq = TerrainQuery(make_roi(flat))
    goal = (7, 55, msl_to_z_index(1300.0, cfg))

    min_msl = _min_possible_aircraft_msl(_terrain_min_valid_elevation(tq), 1100.0, cfg)
    mult = _msl_lower_bound_multiplier(min_msl, cfg)

    violations = 0
    checked = 0
    z0 = msl_to_z_index(1300.0, cfg)
    for row in range(3, height - 3, 3):
        for col in range(2, 50, 4):
            for trend in (-1, 0, 1):
                for age in (0, 4, 8, 10):
                    current = (row, col, z0, trend, age)
                    h_current = _heuristic(current, goal, tq, cfg, mult, min_msl, True)
                    x, y = tq.rowcol_to_xy(row, col)
                    start_xyz = (x, y, 1300.0)
                    for prim in primitives:
                        result = evaluate_primitive(start_xyz, prim, tq, cfg)
                        if not result.valid:
                            continue
                        edge_cost = compute_edge_cost(prim, 1300.0, trend, age, cfg)
                        next_trend, next_age, _, _ = _next_trend_and_age(trend, age, prim, cfg)
                        new_row, new_col = row + prim.drow, col + prim.dcol
                        new_z = z0 + round(prim.dz_m / cfg.z_step_m)
                        neighbor = (new_row, new_col, new_z, next_trend, next_age)
                        h_neighbor = _heuristic(neighbor, goal, tq, cfg, mult, min_msl, True)
                        checked += 1
                        if h_current > edge_cost + h_neighbor + 1e-9:
                            violations += 1

    ok = violations == 0
    print(f"  checked {checked} (state, feasible primitive) combinations: {violations} consistency violations  "
          f"{'PASS' if ok else 'FAIL'}")
    return ok


def validation_12(cfg, primitives) -> bool:
    print()
    print("=== 12: small optimality test, heuristic OFF vs ON ===")
    width, height = 60, 15
    flat = np.full((height, width), 1000.0)
    tq = TerrainQuery(make_roi(flat))
    z0 = msl_to_z_index(1300.0, cfg)
    start, goal = (7, 2, z0), (7, 55, z0)

    r_off = astar_search(start, goal, tq, min_search_altitude_msl=1100.0, max_search_altitude_msl=1500.0,
                          config=cfg, primitives=primitives, use_vertical_reachability_heuristic=False)
    r_on = astar_search(start, goal, tq, min_search_altitude_msl=1100.0, max_search_altitude_msl=1500.0,
                         config=cfg, primitives=primitives, use_vertical_reachability_heuristic=True)
    ok = (
        r_off.status == r_on.status == "success"
        and abs(r_off.total_cost - r_on.total_cost) < 1e-9
        and r_off.path[-1] == r_on.path[-1] == goal
    )
    print(f"  OFF: cost={r_off.total_cost:.3f} expanded={r_off.expanded_nodes} | "
          f"ON: cost={r_on.total_cost:.3f} expanded={r_on.expanded_nodes}  {'PASS' if ok else 'FAIL'}")
    return ok


def validation_13(cfg, primitives) -> bool:
    print()
    print("=== 13: synthetic search-explosion benchmark (Stage 17 heuristic vs Stage 21) ===")
    width, height = 90, 3
    flat = np.full((height, width), 1000.0)
    tq = TerrainQuery(make_roi(flat))
    z0 = msl_to_z_index(1400.0, cfg)
    start, goal = (1, 2, z0), (1, 85, z0)

    results = {}
    for label, use_vr in (("Stage17 (global-floor only)", False), ("Stage21 (+ vertical-reachability)", True)):
        r = astar_search(start, goal, tq, min_search_altitude_msl=1300.0, max_search_altitude_msl=1400.0,
                          config=cfg, primitives=primitives, max_expansions=100_000,
                          use_primitive_cache=True, use_dominance_pruning=False,
                          use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=use_vr)
        results[label] = r
        print(f"  {label}: status={r.status} expanded={r.expanded_nodes} max_open={r.max_open_size} "
              f"runtime={r.runtime_s * 1000:.1f}ms total_cost={r.total_cost:.2f}")

    r17, r21 = results["Stage17 (global-floor only)"], results["Stage21 (+ vertical-reachability)"]
    ok = (
        r17.status == r21.status
        and abs(r17.total_cost - r21.total_cost) < 1e-9
        and r21.expanded_nodes <= r17.expanded_nodes
    )
    reduction = 1.0 - r21.expanded_nodes / r17.expanded_nodes if r17.expanded_nodes else 0.0
    print(f"  same optimal cost, expanded_nodes reduced by {reduction * 100:.1f}%  {'PASS' if ok else 'FAIL'}")
    return ok


def main() -> None:
    cfg = DEFAULT_CONFIG
    primitives = build_primitive_set(cfg)

    results = [
        validation_10(cfg),
        validation_11(cfg, primitives),
        validation_12(cfg, primitives),
        validation_13(cfg, primitives),
    ]

    print()
    print(f"Overall: {'ALL PASS' if all(results) else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
