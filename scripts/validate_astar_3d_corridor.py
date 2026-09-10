"""Stage 37.1: small integration tests for the fine A*'s new optional
Z-guidance tube (z_guide_grid/z_guide_tolerance_m, on top of Stage 37's
XY corridor_mask). No real benchmark here -- see
scripts/benchmark_3d_corridor_fine_real.py for the one real run.

1. Z interpolation along a coarse segment is correct.
2. a fine state within z_guide +/-200m is allowed.
3. a fine state outside +/-200m is rejected (outside_z_guide_tube).
4. a state outside the XY corridor is still rejected first (xy takes
   priority, z tube never even consulted).
5. real Stage 36/37.1 grids: START/GOAL are inside the 3D tube.
6. z_guide_grid=None reproduces Stage 37's XY-only behavior exactly.
"""
import numpy as np

from planner.astar import BUCKET_SHORT, _generate_neighbors, msl_to_z_index, z_index_to_msl
from planner.config import DEFAULT_CONFIG
from planner.corridor import build_z_guide_grid, fine_grid_centers
from planner.primitives import build_primitive_set
from planner.roi import ROIData
from planner.terrain import TerrainQuery
from affine import Affine

NODATA = -9999.0
MASK_NPY = "outputs/stage36_corridor_mask.npy"
START_ROW, START_COL = 48, 276
GOAL_ROW, GOAL_COL = 264, 276
Z_TOLERANCE_M = 200.0


def make_synthetic_roi(elevation: np.ndarray, res: float = 30.0) -> ROIData:
    height, width = elevation.shape
    transform = Affine(res, 0.0, 0.0, 0.0, -res, height * res)
    return ROIData(
        elevation=elevation.astype(np.float32), transform=transform, crs="EPSG:32636",
        width=width, height=height, bounds=(0.0, 0.0, width * res, height * res),
        resolution=(res, res), nodata=NODATA,
    )


def test_1_interpolation() -> bool:
    print("=== 1: Z interpolation along a coarse segment ===")
    roi = make_synthetic_roi(np.zeros((10, 10)), res=30.0)  # 300x300m
    # A single segment from (50,50,1000) to (250,50,1200) -- horizontal, MSL rises 1000->1200.
    polyline = [(50.0, 50.0, 1000.0), (250.0, 50.0, 1200.0)]
    z_guide = build_z_guide_grid(roi, polyline)
    x, y = fine_grid_centers(roi)

    # A cell near the segment's own midpoint (x~150,y~50) should read close to the
    # midpoint MSL (1100).
    idx_mid = np.unravel_index(np.argmin(np.abs(x - 150.0) + np.abs(y - 50.0)), x.shape)
    # A cell near x~50 (the segment's start) should read close to 1000.
    idx_start = np.unravel_index(np.argmin(np.abs(x - 50.0) + np.abs(y - 50.0)), x.shape)
    ok = abs(z_guide[idx_mid] - 1100.0) <= 20.0 and abs(z_guide[idx_start] - 1000.0) <= 20.0
    print(f"  z_guide near midpoint (x~150): {z_guide[idx_mid]:.1f} (expect ~1100)")
    print(f"  z_guide near start (x~50): {z_guide[idx_start]:.1f} (expect ~1000)")
    print(f"  {'PASS' if ok else 'FAIL'}")
    return ok


def test_2_3_within_and_outside_tube(cfg, primitives) -> bool:
    print()
    print("=== 2/3: within +/-200m -> allowed, outside -> rejected ===")
    elev = np.full((10, 10), 1000.0)
    tq = TerrainQuery(make_synthetic_roi(elev))
    z0 = msl_to_z_index(1300.0, cfg)
    state = (5, 5, z0, 0, BUCKET_SHORT)
    all_true_xy = np.ones((10, 10), dtype=bool)
    # Every primitive changes altitude by at most one z_step (20m) from the current
    # 1300m, so new_z_msl always lands in [1280,1320].

    # 2) z_guide close to current altitude (1300) -- every successor's new_z_msl
    # (within [1280,1320]) is comfortably inside +/-200m -> all allowed.
    z_guide_near = np.full((10, 10), 1300.0)
    cache_a, stats_a = {}, {"hits": 0, "misses": 0, "actual_calls": 0}
    accepted_a, rej_a, gen_a, rejected_a, corridor_rej_a, z_rej_a = _generate_neighbors(
        state, primitives, tq, cfg, 1000.0, 1700.0, cache_a, stats_a, None, all_true_xy,
        z_guide_near, Z_TOLERANCE_M,
    )
    ok_2 = z_rej_a == 0 and len(accepted_a) == gen_a
    print(f"  2) z_guide=1300 (near current altitude): generated={gen_a} accepted={len(accepted_a)} "
          f"z_corridor_rejects={z_rej_a}  {'PASS' if ok_2 else 'FAIL'}")

    # 3) z_guide shifted 300m above current altitude -- even the closest possible
    # new_z_msl (1320, one climb step) is |1320-1600|=280m away, > 200m -> all rejected.
    z_guide_far = np.full((10, 10), 1600.0)
    cache_b, stats_b = {}, {"hits": 0, "misses": 0, "actual_calls": 0}
    accepted_b, rej_b, gen_b, rejected_b, corridor_rej_b, z_rej_b = _generate_neighbors(
        state, primitives, tq, cfg, 1000.0, 1700.0, cache_b, stats_b, None, all_true_xy,
        z_guide_far, Z_TOLERANCE_M,
    )
    ok_3 = z_rej_b == gen_b and len(accepted_b) == 0 and rej_b.get("outside_z_guide_tube", 0) == gen_b
    print(f"  3) z_guide=1600 (300m above current altitude): generated={gen_b} accepted={len(accepted_b)} "
          f"z_corridor_rejects={z_rej_b} rejected_reason_counts={rej_b}  {'PASS' if ok_3 else 'FAIL'}")
    return ok_2 and ok_3


def test_4_xy_takes_priority(cfg, primitives) -> bool:
    print()
    print("=== 4: XY corridor rejection happens before Z tube is even consulted ===")
    elev = np.full((10, 10), 1000.0)
    tq = TerrainQuery(make_synthetic_roi(elev))
    z0 = msl_to_z_index(1300.0, cfg)
    state = (5, 5, z0, 0, BUCKET_SHORT)

    xy_mask = np.zeros((10, 10), dtype=bool)
    xy_mask[5, 5] = True  # every successor is outside XY -- z tube should never fire
    z_guide_grid = np.full((10, 10), 1300.0)  # would ALLOW everything if XY didn't reject first

    cache, stats = {}, {"hits": 0, "misses": 0, "actual_calls": 0}
    accepted, rej_counts, gen, rejected, corridor_rej, z_rej = _generate_neighbors(
        state, primitives, tq, cfg, 1000.0, 1700.0, cache, stats, None, xy_mask,
        z_guide_grid, Z_TOLERANCE_M,
    )
    ok = len(accepted) == 0 and corridor_rej == gen and z_rej == 0
    print(f"  generated={gen} accepted={len(accepted)} xy_corridor_rejects={corridor_rej} "
          f"z_corridor_rejects={z_rej}  {'PASS' if ok else 'FAIL'}")
    return ok


def test_5_real_start_goal_inside_tube() -> bool:
    print()
    print("=== 5: real coarse path -- START/GOAL inside the 3D tube ===")
    import csv
    path_xyz = []
    with open("outputs/stage36_coarse_path_eps1.5.csv") as f:
        for row in csv.DictReader(f):
            path_xyz.append((float(row["x"]), float(row["y"]), float(row["z_msl"])))

    from planner.roi import load_roi
    fine_roi = load_roi(DEFAULT_CONFIG)
    z_guide_grid = build_z_guide_grid(fine_roi, path_xyz)
    xy_mask = np.load(MASK_NPY)

    tq = TerrainQuery(fine_roi)
    start_x, start_y = tq.rowcol_to_xy(START_ROW, START_COL)
    goal_x, goal_y = tq.rowcol_to_xy(GOAL_ROW, GOAL_COL)
    start_msl = 3760.0
    goal_msl = 3760.0

    start_z_guide = z_guide_grid[START_ROW, START_COL]
    goal_z_guide = z_guide_grid[GOAL_ROW, GOAL_COL]
    start_ok = xy_mask[START_ROW, START_COL] and abs(start_msl - start_z_guide) <= Z_TOLERANCE_M
    goal_ok = xy_mask[GOAL_ROW, GOAL_COL] and abs(goal_msl - goal_z_guide) <= Z_TOLERANCE_M

    print(f"  START: xy_in_corridor={bool(xy_mask[START_ROW, START_COL])} "
          f"z_guide={start_z_guide:.1f} fine_msl={start_msl} diff={abs(start_msl - start_z_guide):.1f} "
          f"in_tube={start_ok}")
    print(f"  GOAL:  xy_in_corridor={bool(xy_mask[GOAL_ROW, GOAL_COL])} "
          f"z_guide={goal_z_guide:.1f} fine_msl={goal_msl} diff={abs(goal_msl - goal_z_guide):.1f} "
          f"in_tube={goal_ok}")
    ok = bool(start_ok) and bool(goal_ok)
    print(f"  {'PASS' if ok else 'FAIL (would need explicit force-include -- see main benchmark script)'}")
    return ok


def test_6_disabled_reproduces_stage37(cfg, primitives) -> bool:
    print()
    print("=== 6: z_guide_grid=None reproduces Stage 37 XY-only behavior exactly ===")
    elev = np.full((10, 10), 1000.0)
    tq = TerrainQuery(make_synthetic_roi(elev))
    z0 = msl_to_z_index(1300.0, cfg)
    state = (5, 5, z0, 0, BUCKET_SHORT)
    xy_mask = np.ones((10, 10), dtype=bool)

    cache_a, stats_a = {}, {"hits": 0, "misses": 0, "actual_calls": 0}
    a = _generate_neighbors(state, primitives, tq, cfg, 1000.0, 1700.0, cache_a, stats_a, None, xy_mask)
    cache_b, stats_b = {}, {"hits": 0, "misses": 0, "actual_calls": 0}
    b = _generate_neighbors(state, primitives, tq, cfg, 1000.0, 1700.0, cache_b, stats_b, None, xy_mask, None, None)

    ok = (len(a[0]) == len(b[0]) and a[1] == b[1] and a[2] == b[2] and a[3] == b[3]
          and a[4] == b[4] and b[5] == 0)
    print(f"  no z args: accepted={len(a[0])} rejected={a[3]}")
    print(f"  z_guide_grid=None,tolerance=None: accepted={len(b[0])} rejected={b[3]} z_corridor_rejects={b[5]}")
    print(f"  {'PASS' if ok else 'FAIL'}")
    return ok


def main() -> None:
    cfg = DEFAULT_CONFIG
    primitives = build_primitive_set(cfg)
    results = [
        test_1_interpolation(),
        test_2_3_within_and_outside_tube(cfg, primitives),
        test_4_xy_takes_priority(cfg, primitives),
        test_5_real_start_goal_inside_tube(),
        test_6_disabled_reproduces_stage37(cfg, primitives),
    ]
    print()
    print(f"Overall: {'ALL PASS' if all(results) else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
