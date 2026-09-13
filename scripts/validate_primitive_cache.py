"""Stage 15 validation: physical primitive feasibility cache.

Caches evaluate_primitive() results by (row, col, z_index, primitive_id)
-- physical feasibility never depends on search history. Cost
(compute_edge_cost) is always recomputed fresh, never cached.

8)  cache OFF vs ON: identical path/cost/metrics, only evaluate_primitive
    call count should differ.
9)  cache-key correctness: two lookups at the same physical state -- same
    primitive is a cache MISS then HIT, and the returned edge cost is
    identical both times (cost has no history dependence to vary it).
10) invalid-edge caching: a primitive that's physically INVALID at a
    given position is cached as INVALID too -- second lookup is a HIT,
    not a re-sample of the terrain.
11) synthetic benchmark: cache ON vs OFF on a scenario with real
    repeated-state search dynamics. Expect identical expanded_nodes
    (search ordering unaffected) but no more actual evaluate_primitive
    calls with the cache on.
"""
import time

import numpy as np
from affine import Affine

from planner.astar import _generate_neighbors, astar_search, msl_to_z_index
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


def validation_8(cfg, primitives) -> bool:
    print("=== 8: cache OFF vs ON correctness (mountain -> plain -> mountain) ===")
    width, height = 55, 3
    elev = np.full((height, width), 1000.0)
    elev[:, 8:14] = 1140.0
    elev[:, 35:41] = 1140.0
    roi = make_roi(elev)
    tq = TerrainQuery(roi)
    z0 = msl_to_z_index(1300.0, cfg)
    start, goal = (1, 2, z0), (1, 50, z0)

    r_off = astar_search(start, goal, tq, min_search_altitude_msl=1300.0, max_search_altitude_msl=1360.0,
                          config=cfg, primitives=primitives, max_expansions=50_000, use_primitive_cache=False)
    r_on = astar_search(start, goal, tq, min_search_altitude_msl=1300.0, max_search_altitude_msl=1360.0,
                         config=cfg, primitives=primitives, max_expansions=50_000, use_primitive_cache=True)

    print(f"  OFF: status={r_off.status} expanded={r_off.expanded_nodes} actual_calls={r_off.actual_evaluate_primitive_calls} "
          f"runtime={r_off.runtime_s * 1000:.1f}ms")
    print(f"  ON:  status={r_on.status} expanded={r_on.expanded_nodes} actual_calls={r_on.actual_evaluate_primitive_calls} "
          f"hit_rate={r_on.primitive_cache_hit_rate:.3f} runtime={r_on.runtime_s * 1000:.1f}ms")

    ok = (
        r_off.status == r_on.status
        and r_off.path == r_on.path
        and abs(r_off.total_cost - r_on.total_cost) < 1e-9
        and abs(r_off.geometric_path_length - r_on.geometric_path_length) < 1e-9
        and abs(r_off.average_aircraft_msl - r_on.average_aircraft_msl) < 1e-9
        and abs(r_off.minimum_observed_agl - r_on.minimum_observed_agl) < 1e-9
        and r_on.actual_evaluate_primitive_calls <= r_off.actual_evaluate_primitive_calls
    )
    # Step CLEAN-1 note: with the trend/bucket history dimension removed, each physical
    # (row,col,z) is now visited at most once by exact A* in a scenario this small, so
    # zero cache reuse WITHIN a single call is an expected, correct outcome (not a bug) --
    # the assertion above is therefore <=, not <. The cache still pays off whenever a
    # state genuinely gets re-evaluated (multiple predecessors before either closes, or
    # reuse ACROSS calls via external_primitive_cache).
    print(f"  identical path/cost/metrics, never more calls with cache on: {'PASS' if ok else 'FAIL'}")
    return ok


def validation_9(cfg, primitives) -> bool:
    print()
    print("=== 9: cache-key correctness (same physical state, repeated lookup) ===")
    # height=15 keeps row=7 far enough from every edge that all 24
    # primitives (including the +-3/+-4-row diagonal and N/S climb/descent
    # ones) reach the cache stage rather than being bounds-rejected first.
    flat = np.full((15, 20), 1000.0)
    roi = make_roi(flat)
    tq = TerrainQuery(roi)

    z0 = msl_to_z_index(1300.0, cfg)
    row, col = 7, 5
    cache = {}
    stats = {"hits": 0, "misses": 0, "actual_calls": 0}

    state = (row, col, z0)

    neighbors_a, _, _, _, _, _ = _generate_neighbors(state, primitives, tq, cfg, 1100.0, 1500.0, cache, stats)
    misses_after_a, hits_after_a = stats["misses"], stats["hits"]
    print(f"  after 1st call: misses={misses_after_a} hits={hits_after_a} (expect {len(primitives)} misses, 0 hits)")

    neighbors_b, _, _, _, _, _ = _generate_neighbors(state, primitives, tq, cfg, 1100.0, 1500.0, cache, stats)
    misses_after_b, hits_after_b = stats["misses"], stats["hits"]
    print(f"  after 2nd call (same physical state): misses={misses_after_b} hits={hits_after_b - hits_after_a} new "
          f"(expect 0 new misses, {len(primitives)} new hits)")

    climb_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "climb")
    climb_e_target = (row + climb_e.drow, col + climb_e.dcol, z0 + round(climb_e.dz_m / cfg.z_step_m))
    cost_a = next(ec for ns, ec in neighbors_a if ns == climb_e_target)
    cost_b = next(ec for ns, ec in neighbors_b if ns == climb_e_target)

    print(f"  E-climb cost, 1st call (miss): {cost_a:.3f}")
    print(f"  E-climb cost, 2nd call (hit): {cost_b:.3f}")

    ok = (
        misses_after_a == len(primitives) and hits_after_a == 0
        and misses_after_b == misses_after_a  # no NEW misses on the 2nd call
        and (hits_after_b - hits_after_a) == len(primitives)  # every primitive was a hit on the 2nd call
        and cost_a == cost_b  # cost is a pure function of physical state now -- no history to vary it
    )
    print(f"  cache key shared (physical only), cost identical (no history dependence): {'PASS' if ok else 'FAIL'}")
    return ok


def validation_10(cfg, primitives) -> bool:
    print()
    print("=== 10: invalid-edge caching ===")
    width, height = 10, 3
    elev = np.full((height, width), 1000.0)
    elev[:, 3] = 1110.0  # col3 is exactly one E-level hop from col2 -- makes that specific primitive unsafe
    roi = make_roi(elev)
    tq = TerrainQuery(roi)

    z0 = msl_to_z_index(1300.0, cfg)  # AGL=190 over col3's ridge -- below min_agl_m=200
    row, col = 1, 2
    cache = {}
    stats = {"hits": 0, "misses": 0, "actual_calls": 0}

    # Open altitude bounds so every primitive (not just level ones) reaches the cache stage.
    state = (row, col, z0)

    _, rej_a, _, _, _, _ = _generate_neighbors(state, primitives, tq, cfg, 1200.0, 1400.0, cache, stats)
    calls_after_a = stats["actual_calls"]
    _, rej_b, _, _, _, _ = _generate_neighbors(state, primitives, tq, cfg, 1200.0, 1400.0, cache, stats)
    calls_after_b = stats["actual_calls"]

    print(f"  rejections, 1st call: {rej_a}")
    print(f"  rejections, 2nd call: {rej_b}")
    print(f"  actual evaluate_primitive calls after 1st: {calls_after_a}, after 2nd: {calls_after_b} "
          f"(expect no increase -- same physical position, all cached)")

    ok = (
        rej_a.get("below_min_agl", 0) > 0
        and rej_b.get("below_min_agl", 0) == rej_a.get("below_min_agl", 0)
        and calls_after_b == calls_after_a  # zero new evaluate_primitive calls on the 2nd call
    )
    print(f"  INVALID result cached and reused without re-sampling terrain: {'PASS' if ok else 'FAIL'}")
    return ok


def validation_11(cfg, primitives) -> bool:
    print()
    print("=== 11: synthetic performance benchmark (cache OFF vs ON) ===")
    width, height = 55, 3
    elev = np.full((height, width), 1000.0)
    elev[:, 10:] = 1200.0  # forces climbing -- same shape as Stage 14's terrain-slope test
    roi = make_roi(elev)
    tq = TerrainQuery(roi)
    z0 = msl_to_z_index(1300.0, cfg)
    start, goal = (1, 2, z0), (1, 50, msl_to_z_index(1400.0, cfg))

    results = {}
    for label, use_cache in (("OFF", False), ("ON", True)):
        t0 = time.perf_counter()
        r = astar_search(start, goal, tq, min_search_altitude_msl=1300.0, max_search_altitude_msl=1420.0,
                          config=cfg, primitives=primitives, max_expansions=50_000, use_primitive_cache=use_cache)
        wall = time.perf_counter() - t0
        results[label] = (r, wall)
        print(f"  {label}: status={r.status} expanded={r.expanded_nodes} generated={r.generated_neighbors} "
              f"actual_calls={r.actual_evaluate_primitive_calls} hit_rate={r.primitive_cache_hit_rate} "
              f"runtime={wall * 1000:.1f}ms")

    r_off, wall_off = results["OFF"]
    r_on, wall_on = results["ON"]
    ok = (
        r_off.status == r_on.status
        and r_off.expanded_nodes == r_on.expanded_nodes  # cache must not change search ordering
        and r_off.generated_neighbors == r_on.generated_neighbors
        and r_on.actual_evaluate_primitive_calls <= r_off.actual_evaluate_primitive_calls
    )
    speedup = wall_off / wall_on if wall_on > 0 else float("inf")
    call_reduction = 1.0 - r_on.actual_evaluate_primitive_calls / r_off.actual_evaluate_primitive_calls
    print(f"  same expanded_nodes ({r_off.expanded_nodes} == {r_on.expanded_nodes}), "
          f"{call_reduction * 100:.1f}% fewer evaluate_primitive calls, {speedup:.2f}x wall-time speedup")
    print(f"  PASS" if ok else "  FAIL")
    return ok


def main() -> None:
    cfg = DEFAULT_CONFIG
    primitives = build_primitive_set(cfg)

    results = [
        validation_8(cfg, primitives),
        validation_9(cfg, primitives),
        validation_10(cfg, primitives),
        validation_11(cfg, primitives),
    ]

    print()
    print(f"Overall: {'ALL PASS' if all(results) else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
