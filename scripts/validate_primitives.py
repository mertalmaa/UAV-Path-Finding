"""Stage validation: motion primitive generation + along-primitive terrain/AGL.

Part A checks the fixed 8-direction x {level, climb, descent} primitive set
(planner.primitives.build_primitive_set). Part B checks evaluate_primitive()
against small synthetic terrains built entirely for this script -- including
the critical case: a ridge hidden in the middle of an otherwise-clear
primitive, which an endpoint-only check would miss.
"""
import math
import time

import numpy as np
from affine import Affine

from planner.config import DEFAULT_CONFIG
from planner.primitives import DIRECTIONS, build_primitive_set, evaluate_primitive
from planner.roi import ROIData, load_roi
from planner.terrain import TerrainQuery

FLAT_Z = 1000.0
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


def part_a(cfg, all_pass: bool) -> bool:
    print("=== Part A: primitive generation ===")
    primitives = build_primitive_set(cfg)

    by_key = {(p.direction, p.primitive_type): p for p in primitives}
    level_count = sum(1 for p in primitives if p.primitive_type == "level")
    climb_count = sum(1 for p in primitives if p.primitive_type == "climb")
    descent_count = sum(1 for p in primitives if p.primitive_type == "descent")

    print(f"Total primitives: {len(primitives)} "
          f"(level={level_count}, climb={climb_count}, descent={descent_count})")
    print()
    print(f"{'dir':<4}{'type':<9}{'drow':>6}{'dcol':>6}{'dz':>7}{'h_dist':>10}{'angle':>8}")
    for d in DIRECTIONS:
        for kind in ("level", "climb", "descent"):
            p = by_key.get((d, kind))
            if p is None:
                continue
            angle = math.degrees(math.atan2(abs(p.dz_m), p.horizontal_distance_m)) if p.horizontal_distance_m else 0.0
            print(f"{p.direction:<4}{p.primitive_type:<9}{p.drow:>6}{p.dcol:>6}{p.dz_m:>7.1f}"
                  f"{p.horizontal_distance_m:>10.3f}{angle:>8.3f}")

    print()
    checks = []

    checks.append(("1. total count == 24", len(primitives) == 24))
    checks.append(("2. exactly 8 level primitives", level_count == 8))
    checks.append(("3. no no_motion primitive (all h_dist > 0)",
                    all(p.horizontal_distance_m > 0.0 for p in primitives)))

    axial_climb = by_key.get(("N", "climb"))
    axial_angle = math.degrees(math.atan2(20.0, axial_climb.horizontal_distance_m))
    checks.append((
        "4. axial climb ~120m, +20m, <=10deg, VALID",
        abs(axial_climb.horizontal_distance_m - 120.0) < 1e-6
        and axial_climb.dz_m == 20.0
        and axial_angle <= 10.0 + 1e-9,
    ))

    diag_climb = by_key.get(("NE", "climb"))
    diag_angle = math.degrees(math.atan2(20.0, diag_climb.horizontal_distance_m))
    expected_diag = 3 * cfg.xy_resolution_m * math.sqrt(2.0)
    checks.append((
        "5. diagonal climb ~127.28m, +20m, <=10deg, VALID",
        abs(diag_climb.horizontal_distance_m - expected_diag) < 1e-6
        and diag_climb.dz_m == 20.0
        and diag_angle <= 10.0 + 1e-9,
    ))

    axial_descent = by_key.get(("N", "descent"))
    axial_d_angle = math.degrees(math.atan2(20.0, axial_descent.horizontal_distance_m))
    diag_descent = by_key.get(("NE", "descent"))
    diag_d_angle = math.degrees(math.atan2(20.0, diag_descent.horizontal_distance_m))
    checks.append((
        "6. axial+diagonal descent, ~120m/~127.28m, -20m, <=10deg, VALID",
        abs(axial_descent.horizontal_distance_m - 120.0) < 1e-6
        and axial_descent.dz_m == -20.0
        and axial_d_angle <= 10.0 + 1e-9
        and abs(diag_descent.horizontal_distance_m - expected_diag) < 1e-6
        and diag_descent.dz_m == -20.0
        and diag_d_angle <= 10.0 + 1e-9,
    ))

    for label, ok in checks:
        all_pass = all_pass and ok
        print(f"  {label:<48} {'PASS' if ok else 'FAIL'}")

    return all_pass, primitives


def part_b(cfg, primitives, all_pass: bool) -> bool:
    print()
    print("=== Part B: along-primitive terrain/AGL ===")

    by_key = {(p.direction, p.primitive_type): p for p in primitives}
    level_e = by_key[("E", "level")]
    climb_e = by_key[("E", "climb")]
    descent_e = by_key[("E", "descent")]

    def report(label, result, expect_valid, expect_reason=None):
        nonlocal all_pass
        ok = result.valid == expect_valid
        if expect_reason is not None:
            ok = ok and result.reason == expect_reason
        all_pass = all_pass and ok
        min_info = "n/a"
        if result.min_agl_sample is not None:
            s = result.min_agl_sample
            min_info = f"{result.min_agl_m:.1f}m @ sample#{s.index} (x={s.x:.1f},y={s.y:.1f})"
        fail_info = "-"
        if result.first_failure is not None:
            f = result.first_failure
            fail_info = f"sample#{f.index} reason={f.reason} (x={f.x:.1f},y={f.y:.1f},agl={f.agl_m:.1f})"
        print(f"  {label}")
        print(f"    valid={result.valid} reason={result.reason} samples={result.sample_count} "
              f"min_agl={min_info}")
        print(f"    first_failure: {fail_info}")
        print(f"    {'PASS' if ok else 'FAIL'} (expected valid={expect_valid}"
              f"{', reason=' + expect_reason if expect_reason else ''})")

    # 7-9: flat, safe terrain -- level, climb, descent all VALID
    flat_elev = np.full((3, 7), FLAT_Z)
    flat_roi = make_roi(flat_elev)
    flat_tq = TerrainQuery(flat_roi)
    start = flat_tq.rowcol_to_xy(1, 0) + (FLAT_Z + 300.0,)

    report("7. level primitive on flat safe terrain", evaluate_primitive(start, level_e, flat_tq, cfg), True, "ok")
    report("8. climb primitive on flat safe terrain", evaluate_primitive(start, climb_e, flat_tq, cfg), True, "ok")
    report("9. descent primitive on flat safe terrain", evaluate_primitive(start, descent_e, flat_tq, cfg), True, "ok")

    # Critical test: ridge hidden in the middle of the climb primitive's path.
    # Columns: 0=1000, 1=1000, 2=1200 (ridge), 3=1000, 4=1000 (path spans col 0..4)
    ridge_elev = np.full((3, 7), FLAT_Z)
    ridge_elev[1, 2] = 1200.0
    ridge_roi = make_roi(ridge_elev)
    ridge_tq = TerrainQuery(ridge_roi)
    ridge_start = ridge_tq.rowcol_to_xy(1, 0) + (1300.0,)  # AGL=300 at start (baseline terrain)
    result = evaluate_primitive(ridge_start, climb_e, ridge_tq, cfg)
    print()
    report(
        "CRITICAL: hidden ridge mid-primitive (endpoints clear, middle sample AGL<200)",
        result, False, "below_min_agl",
    )
    mid_sample_ok = (
        result.first_failure is not None
        and abs(result.first_failure.terrain_elevation_msl - 1200.0) < 1e-6
        and 0.0 < result.first_failure.t < 1.0
    )
    all_pass = all_pass and mid_sample_ok
    print(f"    ridge caught strictly between endpoints (t={result.first_failure.t:.3f}, "
          f"terrain={result.first_failure.terrain_elevation_msl:.1f}m): "
          f"{'PASS' if mid_sample_ok else 'FAIL'}")

    # 10: primitive that runs off the edge of a tiny ROI -> out_of_bounds
    edge_elev = np.full((3, 3), FLAT_Z)
    edge_roi = make_roi(edge_elev)
    edge_tq = TerrainQuery(edge_roi)
    edge_start = edge_tq.rowcol_to_xy(1, 2) + (FLAT_Z + 300.0,)  # last valid column
    print()
    report("10. primitive running off ROI edge", evaluate_primitive(edge_start, level_e, edge_tq, cfg), False, "out_of_bounds")

    # 11: primitive crossing a synthetic NoData cell mid-path
    nodata_elev = np.full((3, 7), FLAT_Z)
    nodata_elev[1, 2] = NODATA
    nodata_roi = make_roi(nodata_elev)
    nodata_tq = TerrainQuery(nodata_roi)
    nodata_start = nodata_tq.rowcol_to_xy(1, 0) + (1300.0,)
    print()
    report("11. primitive crossing synthetic NoData", evaluate_primitive(nodata_start, climb_e, nodata_tq, cfg), False, "nodata")

    return all_pass


def report_performance(cfg, primitives) -> None:
    print()
    print("=== Performance: evaluate_primitive() on the real Aladaglar ROI ===")
    roi = load_roi(cfg)
    tq = TerrainQuery(roi)
    left, bottom, right, top = roi.bounds
    cx, cy = (left + right) / 2.0, (bottom + top) / 2.0
    center_elev = tq.query(cx, cy).elevation
    start = (cx, cy, center_elev + 300.0)  # comfortably above min_agl_m

    n_rounds = 50
    t0 = time.perf_counter()
    total_samples = 0
    for _ in range(n_rounds):
        for p in primitives:
            result = evaluate_primitive(start, p, tq, cfg)
            total_samples += result.sample_count
    elapsed = time.perf_counter() - t0

    n_calls = n_rounds * len(primitives)
    print(f"{n_calls} evaluate_primitive() calls ({n_rounds} rounds x {len(primitives)} primitives), "
          f"{total_samples} total terrain samples")
    print(f"Total: {elapsed * 1000:.1f} ms | "
          f"{elapsed / n_calls * 1e6:.1f} us/primitive | "
          f"{elapsed / total_samples * 1e6:.2f} us/sample")


def main() -> None:
    cfg = DEFAULT_CONFIG
    all_pass = True
    all_pass, primitives = part_a(cfg, all_pass)
    all_pass = part_b(cfg, primitives, all_pass)

    print()
    print(f"Overall: {'ALL PASS' if all_pass else 'SOME FAILED'}")

    report_performance(cfg, primitives)


if __name__ == "__main__":
    main()
