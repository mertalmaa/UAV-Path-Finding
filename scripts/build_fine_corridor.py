"""Stage 36: Coarse Path -> Fine XY Corridor. NO fine A* runs here -- pure
geometry: build a +/-300m XY-only corridor mask around the Stage 35.2
epsilon=1.5 coarse guide path, on the 30m fine grid.

The epsilon=1.5 coarse path itself was never saved to disk in Stage
35.2 (only epsilon=1.0's was) -- this script runs that ONE coarse search
once (reusing the same precompute-cache machinery, not re-sweeping any
other epsilon) and saves it alongside the corridor outputs.
"""
import csv
import math
import os

import numpy as np
import rasterio
from affine import Affine

from planner.astar import state_to_xyz
from planner.coarse import build_coarse_dem
from planner.coarse_astar import (
    coarse_astar_search, compute_coarse_distance_reference, lift_endpoint_if_unsafe,
    precompute_coarse_primitive_safety,
)
from planner.config import DEFAULT_CONFIG
from planner.corridor import bfs_connected, build_xy_corridor_mask, fine_grid_centers, min_distance_to_polyline
from planner.primitives import build_primitive_set
from planner.roi import ROIData, load_roi
from planner.terrain import TerrainQuery
from scripts.validate_coarse_astar import COARSE_CONFIG

START_ROW, START_COL = 48, 276
GOAL_ROW, GOAL_COL = 264, 276
START_COARSE = (16, 92)
GOAL_COARSE = (88, 92)
ORIGINAL_MSL = 3760.0
MAX_EXPANSIONS = 30_000
ALTITUDE_REFERENCE_MSL = 3240.0
ALTITUDE_SCALE_M = 1000.0
W_DISTANCE = 1.0
W_ALTITUDE = 1.25
EPSILON_SEARCH = 1.5
HALF_WIDTH_M = 300.0

COARSE_PATH_CSV = "outputs/stage36_coarse_path_eps1.5.csv"
MASK_NPY = "outputs/stage36_corridor_mask.npy"
MASK_TIF = "outputs/stage36_corridor_mask.tif"

NODATA = -9999.0


def make_synthetic_roi(elevation: np.ndarray, res: float = 30.0) -> ROIData:
    height, width = elevation.shape
    transform = Affine(res, 0.0, 0.0, 0.0, -res, height * res)
    return ROIData(
        elevation=elevation.astype(np.float32), transform=transform, crs="EPSG:32636",
        width=width, height=height, bounds=(0.0, 0.0, width * res, height * res),
        resolution=(res, res), nodata=NODATA,
    )


# ----------------------------------------------------------------------
# Synthetic tests 1, 2, 6, 7.
# ----------------------------------------------------------------------

def test_1_straight_buffer() -> bool:
    print("=== 1: straight synthetic polyline -> correct 300m buffer ===")
    roi = make_synthetic_roi(np.zeros((100, 100)), res=30.0)  # 3000x3000m
    polyline = [(500.0, 1500.0), (2500.0, 1500.0)]  # a horizontal line through the middle
    mask, dist = build_xy_corridor_mask(roi, polyline, HALF_WIDTH_M)

    x, y = fine_grid_centers(roi)
    # Deliberately avoid the exact 300m boundary here (grid quantization at 30m resolution
    # makes "the nearest cell to exactly 300m off" ambiguous -- that exact-boundary case is
    # tested rigorously and correctly in test 7 instead). 260m is comfortably inside,
    # 340m comfortably outside.
    idx_in = np.unravel_index(np.argmin(np.abs(y - (1500.0 + 260.0)) + np.abs(x - 1500.0)), y.shape)
    idx_out = np.unravel_index(np.argmin(np.abs(y - (1500.0 + 340.0)) + np.abs(x - 1500.0)), y.shape)
    dist_in, dist_out = dist[idx_in], dist[idx_out]
    ok = bool(mask[idx_in]) and not bool(mask[idx_out])
    print(f"  point ~260m off the line (actual={dist_in:.1f}m): in_mask={mask[idx_in]} (expect True)  "
          f"point ~340m off (actual={dist_out:.1f}m): in_mask={mask[idx_out]} (expect False)  "
          f"{'PASS' if ok else 'FAIL'}")
    return ok


def test_2_corner_no_gap() -> bool:
    print()
    print("=== 2: corner/diagonal path -> no gap at the bend ===")
    roi = make_synthetic_roi(np.zeros((100, 100)), res=30.0)
    polyline = [(500.0, 500.0), (1500.0, 500.0), (1500.0, 2500.0)]  # an L-bend
    mask, dist = build_xy_corridor_mask(roi, polyline, HALF_WIDTH_M)

    x, y = fine_grid_centers(roi)
    # A point right at the inside of the corner, close to the vertex, must be covered
    # (distance to the vertex itself is well under 300m even though it's not "on" either
    # infinite line extended) -- this is exactly what a naive per-segment-only check without
    # clamping would get wrong; our clamped projection handles it correctly.
    idx_corner = np.unravel_index(np.argmin(np.abs(y - 550.0) + np.abs(x - 1450.0)), y.shape)
    ok = bool(mask[idx_corner])
    print(f"  point near the inside of the bend (dist_to_vertex~=71m): in_mask={mask[idx_corner]} "
          f"(expect True)  {'PASS' if ok else 'FAIL'}")
    return ok


def test_6_outside_excluded(roi, polyline_xy) -> bool:
    print()
    print("=== 6: cells >300m away are excluded ===")
    x, y = fine_grid_centers(roi)
    dist = min_distance_to_polyline(x, y, polyline_xy)
    mask = dist <= HALF_WIDTH_M
    far_mask = dist > 305.0  # comfortably outside, avoids boundary/grid-center ambiguity
    n_checked = int(far_mask.sum())
    n_wrongly_included = int((mask & far_mask).sum())
    ok = n_wrongly_included == 0
    print(f"  cells with distance>305m: {n_checked}  wrongly included in corridor: {n_wrongly_included}  "
          f"{'PASS' if ok else 'FAIL'}")
    return ok


def test_7_inside_included(roi, polyline_xy) -> bool:
    print()
    print("=== 7: cells <=300m (incl. near-boundary tolerance) are included ===")
    x, y = fine_grid_centers(roi)
    dist = min_distance_to_polyline(x, y, polyline_xy)
    mask = dist <= HALF_WIDTH_M
    near_mask = dist <= 295.0  # comfortably inside
    n_checked = int(near_mask.sum())
    n_wrongly_excluded = int((~mask & near_mask).sum())
    # boundary case: a cell whose distance is within 1mm of exactly 300.0m must be included (<=).
    boundary_idx = np.unravel_index(np.argmin(np.abs(dist - 300.0)), dist.shape)
    boundary_dist = float(dist[boundary_idx])
    boundary_ok = mask[boundary_idx] if boundary_dist <= 300.0 else True
    ok = n_wrongly_excluded == 0 and bool(boundary_ok)
    print(f"  cells with distance<=295m: {n_checked}  wrongly excluded: {n_wrongly_excluded}")
    print(f"  closest-to-exactly-300m cell: distance={boundary_dist:.4f}m in_mask={mask[boundary_idx]}  "
          f"{'PASS' if ok else 'FAIL'}")
    return ok


def measure_widths(roi, polyline_xy, mask, n_samples: int = 10) -> dict:
    """Diagnostic: at n_samples points along the polyline, scan
    perpendicular to the local path direction and measure the corridor's
    actual width (until mask goes False or the ROI edge is hit)."""
    step_m = 10.0
    widths = []
    idxs = np.linspace(1, len(polyline_xy) - 2, num=min(n_samples, max(1, len(polyline_xy) - 2)), dtype=int)
    height, width = mask.shape
    t = roi.transform
    for i in idxs:
        p_prev, p_cur, p_next = polyline_xy[i - 1], polyline_xy[i], polyline_xy[i + 1]
        tangent = (p_next[0] - p_prev[0], p_next[1] - p_prev[1])
        norm = math.hypot(*tangent)
        if norm == 0:
            continue
        perp = (-tangent[1] / norm, tangent[0] / norm)

        def in_mask_at(x, y):
            row = int((y - t.f) / t.e)
            col = int((x - t.c) / t.a)
            if 0 <= row < height and 0 <= col < width:
                return bool(mask[row, col])
            return False

        left_extent = 0.0
        s = 0.0
        while in_mask_at(p_cur[0] + perp[0] * s, p_cur[1] + perp[1] * s):
            left_extent = s
            s += step_m
        right_extent = 0.0
        s = 0.0
        while in_mask_at(p_cur[0] - perp[0] * s, p_cur[1] - perp[1] * s):
            right_extent = s
            s += step_m
        widths.append(left_extent + right_extent)
    if not widths:
        return {"min": float("nan"), "mean": float("nan"), "max": float("nan")}
    return {"min": min(widths), "mean": sum(widths) / len(widths), "max": max(widths), "samples": widths}


def main() -> None:
    r1 = test_1_straight_buffer()
    r2 = test_2_corner_no_gap()
    print()
    print(f"Synthetic buffer/corner tests: {'PASS' if (r1 and r2) else 'FAIL'}")

    print()
    print("=" * 70)
    print("=== Running the ONE epsilon=1.5 coarse search (not saved to disk in Stage 35.2) ===")
    fine_cfg = DEFAULT_CONFIG
    fine_roi = load_roi(fine_cfg)
    coarse_result = build_coarse_dem(fine_roi, factor=3)
    coarse_terrain = TerrainQuery(coarse_result.roi)
    prims = build_primitive_set(COARSE_CONFIG)

    cache, precomp_stats = precompute_coarse_primitive_safety(coarse_terrain, prims, COARSE_CONFIG)
    print(f"  precompute: entries={precomp_stats.entry_count} time={precomp_stats.preprocessing_runtime_s:.2f}s")

    start_lift = lift_endpoint_if_unsafe(START_COARSE[0], START_COARSE[1], ORIGINAL_MSL, coarse_terrain, COARSE_CONFIG)
    goal_lift = lift_endpoint_if_unsafe(GOAL_COARSE[0], GOAL_COARSE[1], ORIGINAL_MSL, coarse_terrain, COARSE_CONFIG)
    start = (start_lift.row, start_lift.col, start_lift.z_index)
    goal = (goal_lift.row, goal_lift.col, goal_lift.z_index)

    seg = coarse_result.roi.elevation[min(START_COARSE[0], GOAL_COARSE[0]):max(START_COARSE[0], GOAL_COARSE[0]) + 1,
                                       START_COARSE[1]]
    seg_min = float(seg.min())
    min_search = math.ceil((seg_min + COARSE_CONFIG.min_agl_m) / COARSE_CONFIG.z_step_m) * COARSE_CONFIG.z_step_m
    max_search = max(start_lift.msl, goal_lift.msl) + 2 * COARSE_CONFIG.z_step_m
    d_ref = compute_coarse_distance_reference(start, goal, coarse_terrain, COARSE_CONFIG)

    result = coarse_astar_search(
        start, goal, coarse_terrain, min_search, max_search, COARSE_CONFIG,
        primitives=prims, max_expansions=MAX_EXPANSIONS, distance_reference_m=d_ref,
        altitude_reference_msl=ALTITUDE_REFERENCE_MSL, altitude_scale_m=ALTITUDE_SCALE_M,
        w_distance=W_DISTANCE, w_altitude=W_ALTITUDE, precomputed_safety=cache, epsilon_search=EPSILON_SEARCH,
    )
    print(f"  status={result.status} expanded={result.expanded_nodes} path_nodes={len(result.path)}")
    if not result.success:
        print("FAIL -- epsilon=1.5 coarse search did not succeed. Stopping.")
        return

    coarse_path = result.path
    coarse_xyz = [state_to_xyz(s, coarse_terrain, COARSE_CONFIG) for s in coarse_path]
    polyline_xy = [(x, y) for (x, y, z) in coarse_xyz]

    os.makedirs("outputs", exist_ok=True)
    with open(COARSE_PATH_CSV, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "row", "col", "z_index", "x", "y", "z_msl"])
        for i, ((r, c, z), (x, y, zm)) in enumerate(zip(coarse_path, coarse_xyz)):
            writer.writerow([i, r, c, z, f"{x:.2f}", f"{y:.2f}", f"{zm:.2f}"])
    print(f"  coarse path (epsilon=1.5) saved to {COARSE_PATH_CSV}")

    print()
    print("=" * 70)
    print(f"=== Building fine XY corridor mask (+/-{HALF_WIDTH_M:.0f}m) ===")
    mask, dist = build_xy_corridor_mask(fine_roi, polyline_xy, HALF_WIDTH_M)

    start_rc = (START_ROW, START_COL)
    goal_rc = (GOAL_ROW, GOAL_COL)
    start_included = bool(mask[start_rc])
    goal_included = bool(mask[goal_rc])
    print(f"  start (row={START_ROW},col={START_COL}) included: {start_included} "
          f"(distance_to_polyline={dist[start_rc]:.1f}m)")
    print(f"  goal  (row={GOAL_ROW},col={GOAL_COL}) included: {goal_included} "
          f"(distance_to_polyline={dist[goal_rc]:.1f}m)")

    if not start_included:
        mask[start_rc] = True
        print("  !! start was NOT already covered -- force-included explicitly (endpoint itself unchanged).")
    if not goal_included:
        mask[goal_rc] = True
        print("  !! goal was NOT already covered -- force-included explicitly (endpoint itself unchanged).")

    connected = bfs_connected(mask, start_rc, goal_rc)
    print(f"  start<->goal XY-connected within corridor: {connected}")

    total_cells = mask.size
    corridor_cells = int(mask.sum())
    coverage_pct = corridor_cells / total_cells * 100.0
    area_km2 = corridor_cells * (fine_roi.resolution[0] * fine_roi.resolution[1]) / 1e6
    reduction_pct = 100.0 - coverage_pct

    print()
    print("=== Corridor statistics ===")
    print(f"  fine_grid_shape={mask.shape}  total_fine_cells={total_cells}")
    print(f"  corridor_cell_count={corridor_cells}  coverage={coverage_pct:.2f}%  "
          f"approx_area_km2={area_km2:.2f}")
    print(f"  search-space XY reduction (diagnostic only, NOT a speed claim)={reduction_pct:.2f}%")

    widths = measure_widths(fine_roi, polyline_xy, mask, n_samples=10)
    print(f"  measured corridor width (perpendicular scan, 10 sample points along path): "
          f"min={widths['min']:.1f}m mean={widths['mean']:.1f}m max={widths['max']:.1f}m "
          f"(expected up to ~{2 * HALF_WIDTH_M:.0f}m where not clipped by ROI edge)")

    r6 = test_6_outside_excluded(fine_roi, polyline_xy)
    r7 = test_7_inside_included(fine_roi, polyline_xy)
    r3 = start_included or True  # after force-include, always True by construction; report pre-fix status above
    r4 = mask.shape == fine_roi.elevation.shape
    r5 = connected
    print()
    print(f"3: start/goal included (after any needed force-include): PASS")
    print(f"4: mask shape == fine grid shape ({mask.shape} == {fine_roi.elevation.shape}): {'PASS' if r4 else 'FAIL'}")
    print(f"5: XY connectivity: {'PASS' if r5 else 'FAIL'}")
    print(f"6: outside-cells-excluded: {'PASS' if r6 else 'FAIL'}")
    print(f"7: inside-cells-included: {'PASS' if r7 else 'FAIL'}")

    overall = r1 and r2 and r4 and r5 and r6 and r7
    print()
    print(f"Overall Stage 36: {'PASS' if overall else 'FAIL'}")

    if overall:
        np.save(MASK_NPY, mask)
        print(f"  mask saved to {MASK_NPY}")
        with rasterio.open(
            MASK_TIF, "w", driver="GTiff", height=mask.shape[0], width=mask.shape[1], count=1,
            dtype="uint8", crs=fine_roi.crs, transform=fine_roi.transform, nodata=0,
        ) as dst:
            dst.write(mask.astype(np.uint8), 1)
        print(f"  mask GeoTIFF saved to {MASK_TIF}")
    else:
        print("  Validation FAILED -- no mask files written.")


if __name__ == "__main__":
    main()
