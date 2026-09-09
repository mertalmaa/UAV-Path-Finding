"""Stage validation: low-MSL soft cost preference in A* (planner/astar.py).

A) weight=0.0 regression: reproduces the exact pre-existing baseline A*
   behavior (geometric-only cost).
B) low-MSL preference: same start/goal, weight=0.0 vs weight=0.25, shows
   the search trading a small amount of extra distance for lower MSL.
C) AGL safety supremacy: a low-altitude shortcut is unsafe (AGL<200) over
   a ridge -- the low-MSL preference must never be allowed to pick it.
D) distance vs MSL trade-off: a direct-but-higher route vs a longer
   bypass-but-lower route, compared at both weights.
"""
import dataclasses
import math

import numpy as np
from affine import Affine

from planner.astar import astar_search, msl_to_z_index
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set
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
    print(f"    minimum_observed_agl={result.minimum_observed_agl:.1f} "
          f"expanded={result.expanded_nodes} runtime={result.runtime_s * 1000:.1f}ms")


def validation_a(cfg, primitives) -> bool:
    print("=== A) weight=0.0 regression (flat level path, same as baseline A* stage) ===")
    flat = np.full((5, 20), 1000.0)
    roi = make_roi(flat)
    tq = TerrainQuery(roi)

    z_idx = msl_to_z_index(1300.0, cfg)
    start, goal = (2, 2, z_idx), (2, 12, z_idx)

    cfg0 = dataclasses.replace(cfg, msl_cost_weight=0.0)
    result0 = astar_search(start, goal, tq, min_search_altitude_msl=1100.0, max_search_altitude_msl=1500.0,
                            config=cfg0, primitives=primitives)
    report("weight=0.0", result0)

    result_default = astar_search(start, goal, tq, min_search_altitude_msl=1100.0, max_search_altitude_msl=1500.0,
                                   config=cfg, primitives=primitives)
    report(f"weight={cfg.msl_cost_weight} (for contrast)", result_default)

    ok = (
        result0.success
        and result0.path == [(2, c, z_idx) for c in range(2, 13)]
        and abs(result0.total_cost - 300.0) < 1e-6
        and abs(result0.total_cost - result0.geometric_path_length) < 1e-9  # weight=0 -> weighted == geometric exactly
    )
    print(f"  PASS" if ok else "  FAIL")
    return ok


def validation_b(cfg, primitives) -> bool:
    print("=== B) low-MSL preference (weight=0.0 vs weight=0.25) ===")
    flat = np.full((10, 45), 1000.0)  # safe everywhere: AGL>=300 even at the bottom of the altitude range
    roi = make_roi(flat)
    tq = TerrainQuery(roi)

    z_top = msl_to_z_index(1320.0, cfg)
    start, goal = (5, 2, z_top), (5, 42, z_top)  # same (high) altitude, 40 cells apart

    results = {}
    for w in (0.0, 0.25):
        c = dataclasses.replace(cfg, msl_cost_weight=w)
        r = astar_search(start, goal, tq, min_search_altitude_msl=1300.0, max_search_altitude_msl=1320.0,
                          config=c, primitives=primitives)
        results[w] = r
        report(f"weight={w}", r)

    r0, r25 = results[0.0], results[0.25]
    ok = r0.success and r25.success
    ok = ok and r0.average_aircraft_msl >= r25.average_aircraft_msl  # weight=0.25 flies lower on average
    ok = ok and r25.geometric_path_length >= r0.geometric_path_length - 1e-6  # ...at the cost of extra distance
    ok = ok and r0.minimum_observed_agl >= cfg.min_agl_m - 1e-6
    ok = ok and r25.minimum_observed_agl >= cfg.min_agl_m - 1e-6

    print(f"  weight=0.0  avg_msl={r0.average_aircraft_msl:.1f}  geom_len={r0.geometric_path_length:.1f}")
    print(f"  weight=0.25 avg_msl={r25.average_aircraft_msl:.1f}  geom_len={r25.geometric_path_length:.1f}")
    print(f"  low-MSL preference visible (lower avg MSL, >= geometric length): "
          f"{r0.average_aircraft_msl >= r25.average_aircraft_msl and r25.geometric_path_length >= r0.geometric_path_length - 1e-6}")
    print(f"  PASS" if ok else "  FAIL")
    return ok


def validation_c(cfg, primitives) -> bool:
    print("=== C) AGL safety always wins over low-MSL preference ===")
    width, height = 45, 10
    elev = np.full((height, width), 1000.0)
    elev[:, 15:21] = 1110.0  # ridge across the whole corridor: unsafe at 1300m (AGL=190), safe at 1320m (AGL=210)
    roi = make_roi(elev)
    tq = TerrainQuery(roi)

    z_top = msl_to_z_index(1320.0, cfg)
    start, goal = (5, 2, z_top), (5, 42, z_top)

    result = astar_search(start, goal, tq, min_search_altitude_msl=1300.0, max_search_altitude_msl=1320.0,
                           config=cfg, primitives=primitives)  # cfg: default msl_cost_weight=0.25
    report(f"weight={cfg.msl_cost_weight}", result)

    ok = result.success and result.minimum_observed_agl >= cfg.min_agl_m - 1e-6

    print(f"  minimum_observed_agl={result.minimum_observed_agl:.1f} >= min_agl_m={cfg.min_agl_m}: "
          f"{result.minimum_observed_agl >= cfg.min_agl_m - 1e-6}")
    print(f"  PASS" if ok else "  FAIL")
    return ok


def validation_d(cfg, primitives) -> bool:
    print("=== D) distance vs MSL trade-off (direct-climb vs low-bypass) ===")
    width, height = 30, 9
    elev = np.full((height, width), 1000.0)
    elev[1:8, 13:18] = 1150.0  # ridge blocks rows 1-7 at cols 13-17; rows 0 and 8 stay baseline (bypass gap)
    roi = make_roi(elev)
    tq = TerrainQuery(roi)

    z_low = msl_to_z_index(1300.0, cfg)
    start, goal = (4, 2, z_low), (4, 27, z_low)

    results = {}
    for w in (0.0, 0.25):
        c = dataclasses.replace(cfg, msl_cost_weight=w)
        r = astar_search(start, goal, tq, min_search_altitude_msl=1300.0, max_search_altitude_msl=1400.0,
                          config=c, primitives=primitives, max_expansions=50_000)
        results[w] = r
        report(f"weight={w}", r)

    r0, r25 = results[0.0], results[0.25]
    ok = r0.success and r25.success
    ok = ok and r0.minimum_observed_agl >= cfg.min_agl_m - 1e-6
    ok = ok and r25.minimum_observed_agl >= cfg.min_agl_m - 1e-6

    print(f"  weight=0.0  geom_len={r0.geometric_path_length:.1f}  avg_msl={r0.average_aircraft_msl:.1f}  "
          f"max_msl={r0.maximum_aircraft_msl:.1f}  weighted_cost={r0.total_cost:.2f}")
    print(f"  weight=0.25 geom_len={r25.geometric_path_length:.1f}  avg_msl={r25.average_aircraft_msl:.1f}  "
          f"max_msl={r25.maximum_aircraft_msl:.1f}  weighted_cost={r25.total_cost:.2f}")
    print("  (this compares behavior at two weights -- 0.25 is a prototype tuning value, not a claimed-correct one)")
    print(f"  PASS" if ok else "  FAIL")
    return ok


def main() -> None:
    cfg = DEFAULT_CONFIG
    assert cfg.msl_cost_weight == 0.25
    primitives = build_primitive_set(cfg)

    results = [
        validation_a(cfg, primitives),
        validation_b(cfg, primitives),
        validation_c(cfg, primitives),
        validation_d(cfg, primitives),
    ]

    print()
    print(f"Overall: {'ALL PASS' if all(results) else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
