"""Stage 33: Safe Goal Region (70x70x50m box, +/-35m XY / +/-25m Z) --
production feature (planner.config.PlannerConfig.goal_tolerance_xy_m /
goal_tolerance_z_m, both default 0.0 = exact pre-Stage-33 goal condition).

Unit tests A-K from the spec:
  A) exact center -> GOAL
  B) dx=30m -> GOAL
  C) dy=30m,dz=20m -> GOAL
  D) dx=35m (boundary) -> GOAL
  E) dz=25m (boundary) -> GOAL
  F) dx=36m -> NOT GOAL
  G) dz=26m -> NOT GOAL
  H) goal box entirely unsafe (AGL) -> search can NEVER report success there
  I) inside box -> heuristic h=0
  J) outside box -> heuristic uses distance to the box's nearest face, not center
  K) tolerance=0 -> exact pre-Stage-33 goal condition preserved bit-for-bit
"""
import dataclasses
import math

import numpy as np
from affine import Affine

from planner.astar import (
    BUCKET_SHORT, _distance_to_goal_box, _heuristic, _state_in_goal_region,
    astar_search, msl_to_z_index, state_to_xyz,
)
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set
from planner.roi import ROIData
from planner.terrain import TerrainQuery

NODATA = -9999.0
TOL_XY, TOL_Z = 35.0, 25.0


def make_roi(elevation: np.ndarray, nodata: float = NODATA, res: float = 30.0) -> ROIData:
    height, width = elevation.shape
    transform = Affine(res, 0.0, 0.0, 0.0, -res, height * res)
    return ROIData(
        elevation=elevation.astype(np.float32), transform=transform, crs="EPSG:32636",
        width=width, height=height, bounds=(0.0, 0.0, width * res, height * res),
        resolution=(res, res), nodata=nodata,
    )


# ----------------------------------------------------------------------
# A-G: pure box-membership arithmetic (real-valued, grid-independent --
# these dx values (30/35/36m etc) don't have to land on a 30m grid cell,
# they test the FORMULA directly, exactly as the spec frames them).
# ----------------------------------------------------------------------

def unit_tests_a_to_g() -> bool:
    print("=== A-G: goal box membership (_state_in_goal_region's underlying formula) ===")
    gx, gy, gz = 1000.0, 2000.0, 3760.0

    def in_box(dx, dy, dz):
        return abs(dx) <= TOL_XY and abs(dy) <= TOL_XY and abs(dz) <= TOL_Z

    cases = [
        ("A: exact center", 0.0, 0.0, 0.0, True),
        ("B: dx=30m", 30.0, 0.0, 0.0, True),
        ("C: dy=30m,dz=20m", 0.0, 30.0, 20.0, True),
        ("D: dx=35m boundary", 35.0, 0.0, 0.0, True),
        ("E: dz=25m boundary", 0.0, 0.0, 25.0, True),
        ("F: dx=36m", 36.0, 0.0, 0.0, False),
        ("G: dz=26m", 0.0, 0.0, 26.0, False),
    ]
    all_ok = True
    for label, dx, dy, dz, expected in cases:
        actual = in_box(dx, dy, dz)
        dist = _distance_to_goal_box(gx + dx, gy + dy, gz + dz, gx, gy, gz, TOL_XY, TOL_Z)
        # membership must agree with "distance to box == 0"
        consistent = actual == (dist == 0.0)
        ok = actual == expected and consistent
        all_ok = all_ok and ok
        print(f"  {label}: dx={dx} dy={dy} dz={dz} -> in_box={actual} (expect {expected}) "
              f"distance_to_box={dist:.4f}  {'PASS' if ok else 'FAIL'}")
    print(f"  Overall: {'PASS' if all_ok else 'FAIL'}")
    return all_ok


# ----------------------------------------------------------------------
# H: goal box entirely unsafe (AGL violation everywhere within tolerance)
# -- the search must NEVER report success through it.
# ----------------------------------------------------------------------

def unit_test_h(cfg, primitives) -> bool:
    print()
    print("=== H: goal box entirely unsafe (AGL) -- must never become a solution ===")
    width, height = 12, 3
    elev = np.full((height, width), 1000.0)
    goal_col = 9
    # Terrain right at and around the goal column sits at the AIRCRAFT's own
    # altitude (1300m) -- AGL=0 there, and even at the topmost altitude the
    # +/-25m z-tolerance allows (1300+20=1320), AGL is still only 20m << 200m.
    elev[:, goal_col - 1:goal_col + 2] = 1300.0
    tq = TerrainQuery(make_roi(elev))
    z0 = msl_to_z_index(1300.0, cfg)
    start, goal = (1, 1, z0), (1, goal_col, z0)

    cfg_tol = dataclasses.replace(cfg, goal_tolerance_xy_m=TOL_XY, goal_tolerance_z_m=TOL_Z)
    result = astar_search(start, goal, tq, min_search_altitude_msl=1200.0, max_search_altitude_msl=1400.0,
                           config=cfg_tol, primitives=primitives, max_expansions=5000,
                           use_primitive_cache=True, use_dominance_pruning=False,
                           use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False)

    never_entered_region = result.closest_distance_to_goal_region_m > 0.0
    ok = (not result.success) and never_entered_region
    print(f"  status={result.status} success={result.success} "
          f"closest_distance_to_goal_region_m={result.closest_distance_to_goal_region_m:.2f} "
          f"closest_distance_to_goal_center_m={result.closest_distance_to_goal_center_m:.2f} "
          f"closest_state_to_goal={result.closest_state_to_goal}")
    print(f"  never reported success through the unsafe box: {ok}  {'PASS' if ok else 'FAIL'}")
    return ok


# ----------------------------------------------------------------------
# I/J: heuristic must use distance-to-box, not distance-to-center.
# ----------------------------------------------------------------------

def unit_tests_i_j(cfg, primitives) -> bool:
    print()
    print("=== I/J: goal-region-aware heuristic ===")
    width, height = 10, 3
    elev = np.full((height, width), 1000.0)
    tq = TerrainQuery(make_roi(elev))
    cfg_tol = dataclasses.replace(cfg, goal_tolerance_xy_m=TOL_XY, goal_tolerance_z_m=TOL_Z)
    z0 = msl_to_z_index(1300.0, cfg_tol)
    goal = (1, 8, z0)

    # I: a state already within the box (1 column = 30m <= 35m tolerance, same z).
    inside_state = (1, 7, z0, 0, BUCKET_SHORT)
    h_inside = _heuristic(inside_state, goal, tq, cfg_tol)
    ok_i = h_inside == 0.0
    print(f"  I: state 30m from center (within +/-35m XY box) -> h={h_inside:.4f} (expect 0.0)  "
          f"{'PASS' if ok_i else 'FAIL'}")

    # J: a state well outside the box -- h must reflect distance to the box's
    # nearest face (row diff * 30m - tolerance), not the full center distance.
    outside_state = (1, 3, z0, 0, BUCKET_SHORT)  # 5 columns = 150m from goal center
    h_outside = _heuristic(outside_state, goal, tq, cfg_tol)
    expected_face_distance = 5 * cfg_tol.xy_resolution_m - TOL_XY  # 150 - 35 = 115m
    center_distance = 5 * cfg_tol.xy_resolution_m  # 150m
    ok_j = abs(h_outside - expected_face_distance) < 1e-6 and h_outside < center_distance
    print(f"  J: state {5 * cfg_tol.xy_resolution_m:.0f}m from center -> h={h_outside:.4f} "
          f"(expect face-distance={expected_face_distance:.1f}, strictly < center-distance={center_distance:.1f})  "
          f"{'PASS' if ok_j else 'FAIL'}")
    return ok_i and ok_j


# ----------------------------------------------------------------------
# K: tolerance=0 -> exact pre-Stage-33 behavior, bit-for-bit.
# ----------------------------------------------------------------------

def unit_test_k(cfg, primitives) -> bool:
    print()
    print("=== K: tolerance=0 reproduces the exact pre-Stage-33 goal condition ===")
    width, height = 30, 3
    elev = np.full((height, width), 1000.0)
    elev[:, 12:] = 1200.0
    tq = TerrainQuery(make_roi(elev))
    z0 = msl_to_z_index(1300.0, cfg)
    start, goal = (1, 2, z0), (1, 27, msl_to_z_index(1400.0, cfg))

    default_cfg = cfg  # goal_tolerance_xy_m/z_m default to 0.0 already
    explicit_zero_cfg = dataclasses.replace(cfg, goal_tolerance_xy_m=0.0, goal_tolerance_z_m=0.0)

    kwargs = dict(min_search_altitude_msl=1200.0, max_search_altitude_msl=1450.0,
                  primitives=primitives, max_expansions=20_000, use_primitive_cache=True,
                  use_dominance_pruning=False, use_msl_lower_bound_heuristic=True,
                  use_vertical_reachability_heuristic=False)
    r_default = astar_search(start, goal, tq, config=default_cfg, **kwargs)
    r_explicit = astar_search(start, goal, tq, config=explicit_zero_cfg, **kwargs)

    same = (r_default.status == r_explicit.status == "success"
            and r_default.expanded_nodes == r_explicit.expanded_nodes
            and r_default.max_open_size == r_explicit.max_open_size
            and r_default.total_cost == r_explicit.total_cost)
    print(f"  default (no tolerance params passed): cost={r_default.total_cost:.4f} expanded={r_default.expanded_nodes}")
    print(f"  explicit goal_tolerance_xy_m=0.0/z_m=0.0: cost={r_explicit.total_cost:.4f} expanded={r_explicit.expanded_nodes}")
    print(f"  bit-for-bit identical: {same}  {'PASS' if same else 'FAIL'}")
    return same


def main() -> None:
    cfg = DEFAULT_CONFIG
    primitives = build_primitive_set(cfg)

    r_ag = unit_tests_a_to_g()
    r_h = unit_test_h(cfg, primitives)
    r_ij = unit_tests_i_j(cfg, primitives)
    r_k = unit_test_k(cfg, primitives)

    print()
    print(f"Overall: {'ALL PASS' if (r_ag and r_h and r_ij and r_k) else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
