"""Stage validation: terrain-aware 3D A* (planner/astar.py).

A) flat terrain, level path
B) flat terrain, single climb-primitive hop
C) terrain obstacle with a safe detour (altitude locked so climbing over
   the ridge isn't an option -- forces a lateral detour)
D) terrain obstacle with no gap anywhere (altitude locked) -> true no_path
E) smoke test on the real 10x10 km Aladaglar ROI

Every SUCCESS path is re-checked edge-by-edge with the existing
evaluate_primitive() as a consistency check (not a substitute for a future
independent final validator).
"""
import math

import numpy as np
from affine import Affine

from planner.astar import astar_search, msl_to_z_index, state_to_xyz
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set, evaluate_primitive
from planner.roi import ROIData, load_roi
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


def verify_path_edges(path, primitives, terrain, config) -> bool:
    """Re-check every edge of a found path with evaluate_primitive() again."""
    by_delta = {(p.drow, p.dcol, round(p.dz_m / config.z_step_m)): p for p in primitives}
    for (r1, c1, z1), (r2, c2, z2) in zip(path, path[1:]):
        prim = by_delta.get((r2 - r1, c2 - c1, z2 - z1))
        if prim is None:
            print(f"    edge-check FAIL: no primitive matches delta {(r2 - r1, c2 - c1, z2 - z1)}")
            return False
        start_xyz = state_to_xyz((r1, c1, z1), terrain, config)
        result = evaluate_primitive(start_xyz, prim, terrain, config)
        if not result.valid:
            print(f"    edge-check FAIL: ({r1},{c1},{z1})->({r2},{c2},{z2}) reason={result.reason}")
            return False
    return True


def report_metrics(label, result) -> None:
    print(f"  [{label}] status={result.status} runtime={result.runtime_s * 1000:.2f}ms "
          f"path_states={len(result.path)} total_cost={result.total_cost:.2f} "
          f"expanded={result.expanded_nodes} generated={result.generated_neighbors} "
          f"rejected={result.rejected_neighbors} max_open={result.max_open_size}")
    if result.rejected_reason_counts:
        print(f"    rejected reasons: {result.rejected_reason_counts}")


def test_a(cfg, primitives) -> bool:
    print("=== A) flat terrain / level path ===")
    flat = np.full((5, 20), 1000.0)
    roi = make_roi(flat)
    tq = TerrainQuery(roi)

    z_idx = msl_to_z_index(1300.0, cfg)
    start = (2, 2, z_idx)
    goal = (2, 12, z_idx)

    result = astar_search(start, goal, tq, min_search_altitude_msl=1100.0, max_search_altitude_msl=1500.0,
                           config=cfg, primitives=primitives)
    report_metrics("A", result)

    ok = result.success and result.path[0] == start and result.path[-1] == goal
    ok = ok and all(s[2] == z_idx for s in result.path)  # pure level: z never changes
    ok = ok and abs(result.total_cost - 300.0) < 1e-6  # 10 cells * 30m, straight east
    ok = ok and verify_path_edges(result.path, primitives, tq, cfg)

    print(f"  PASS" if ok else "  FAIL")
    return ok


def test_b(cfg, primitives) -> bool:
    print("=== B) one-primitive climb ===")
    flat = np.full((5, 20), 1000.0)
    roi = make_roi(flat)
    tq = TerrainQuery(roi)

    climb_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "climb")
    z0 = msl_to_z_index(1300.0, cfg)
    start = (2, 2, z0)
    goal = (2 + climb_e.drow, 2 + climb_e.dcol, z0 + round(climb_e.dz_m / cfg.z_step_m))

    result = astar_search(start, goal, tq, min_search_altitude_msl=1100.0, max_search_altitude_msl=1500.0,
                           config=cfg, primitives=primitives)
    report_metrics("B", result)

    expected_cost = math.sqrt(climb_e.horizontal_distance_m ** 2 + climb_e.dz_m ** 2)
    ok = result.success and result.path[0] == start and result.path[-1] == goal
    ok = ok and len(result.path) == 2  # reached in exactly one primitive hop
    ok = ok and abs(result.total_cost - expected_cost) < 1e-6
    ok = ok and verify_path_edges(result.path, primitives, tq, cfg)

    print(f"  expected single-hop cost={expected_cost:.3f}")
    print(f"  PASS" if ok else "  FAIL")
    return ok


def test_c(cfg, primitives) -> bool:
    print("=== C) terrain obstacle with safe detour (altitude locked) ===")
    width, height = 15, 7
    elev = np.full((height, width), 1000.0)
    elev[1:6, 6:9] = 1500.0  # wall blocking rows 1-5 at cols 6-8; rows 0 and 6 stay clear (the gap)
    roi = make_roi(elev)
    tq = TerrainQuery(roi)

    z_idx = msl_to_z_index(1300.0, cfg)  # AGL=300 on baseline (1000m), AGL=-200 on the ridge (1500m)
    start = (3, 1, z_idx)
    goal = (3, 13, z_idx)

    result = astar_search(start, goal, tq, min_search_altitude_msl=1300.0, max_search_altitude_msl=1300.0,
                           config=cfg, primitives=primitives, max_expansions=20000)
    report_metrics("C", result)

    def in_wall(state):
        r, c, _ = state
        return 1 <= r <= 5 and 6 <= c <= 8

    ok = result.success and result.path[0] == start and result.path[-1] == goal
    ok = ok and not any(in_wall(s) for s in result.path)
    ok = ok and verify_path_edges(result.path, primitives, tq, cfg)

    print(f"  path avoids blocked ridge cells: {not any(in_wall(s) for s in result.path)}")
    print(f"  PASS" if ok else "  FAIL")
    return ok


def test_d(cfg, primitives) -> bool:
    print("=== D) true no-path (wall spans entire width, altitude locked) ===")
    width, height = 15, 7
    elev = np.full((height, width), 1000.0)
    elev[:, 6:9] = 1500.0  # wall blocks every row -- no gap anywhere
    roi = make_roi(elev)
    tq = TerrainQuery(roi)

    z_idx = msl_to_z_index(1300.0, cfg)
    start = (3, 1, z_idx)
    goal = (3, 13, z_idx)

    result = astar_search(start, goal, tq, min_search_altitude_msl=1300.0, max_search_altitude_msl=1300.0,
                           config=cfg, primitives=primitives, max_expansions=20000)
    report_metrics("D", result)

    ok = (not result.success) and result.status == "no_path"

    print(f"  PASS" if ok else "  FAIL")
    return ok


def test_e(cfg, primitives) -> bool:
    print("=== E) real Aladaglar ROI smoke test ===")
    roi = load_roi(cfg)
    tq = TerrainQuery(roi)

    valid_elev = roi.elevation[roi.elevation != roi.nodata] if roi.nodata is not None else roi.elevation
    max_terrain = float(valid_elev.max())
    # Prototype smoke-test altitude, NOT a real mission altitude: clears the
    # highest terrain in this ROI by min_agl_m, aligned up to the z-grid.
    safe_altitude = math.ceil((max_terrain + cfg.min_agl_m) / cfg.z_step_m) * cfg.z_step_m
    z_idx = msl_to_z_index(safe_altitude, cfg)

    start_row, start_col = 140, 140
    goal_row, goal_col = 105, 175
    start = (start_row, start_col, z_idx)
    goal = (goal_row, goal_col, z_idx)

    sx, sy = tq.rowcol_to_xy(start_row, start_col)
    gx, gy = tq.rowcol_to_xy(goal_row, goal_col)
    straight_dist = math.hypot(gx - sx, gy - sy)
    print(f"  max ROI terrain={max_terrain:.1f}m -> smoke-test altitude={safe_altitude:.1f}m msl "
          f"(prototype test altitude, not a real mission altitude)")
    print(f"  start=({start_row},{start_col}) goal=({goal_row},{goal_col}) straight_dist={straight_dist:.1f}m")

    result = astar_search(start, goal, tq, min_search_altitude_msl=safe_altitude - 100.0,
                           max_search_altitude_msl=safe_altitude + 100.0,
                           config=cfg, primitives=primitives, max_expansions=200_000)
    report_metrics("E", result)

    ok = result.success and result.path[0] == start and result.path[-1] == goal
    ok = ok and verify_path_edges(result.path, primitives, tq, cfg)

    print(f"  PASS" if ok else "  FAIL")
    return ok


def main() -> None:
    cfg = DEFAULT_CONFIG
    primitives = build_primitive_set(cfg)

    results = [
        test_a(cfg, primitives),
        test_b(cfg, primitives),
        test_c(cfg, primitives),
        test_d(cfg, primitives),
        test_e(cfg, primitives),
    ]

    print()
    print(f"Overall: {'ALL PASS' if all(results) else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
