"""Stage validation: point-wise terrain elevation queries (planner/terrain.py).

Checks: ROI center query, several interior points, ROI-edge points, an
out-of-ROI point correctly rejected, row/col<->xy round-trip consistency,
a synthetic NoData cell correctly flagged invalid, and a rough runtime
measurement for a few thousand queries.

No masking, cost, or search logic here -- point lookups only.
"""
import time

import numpy as np
from affine import Affine

from planner.config import DEFAULT_CONFIG
from planner.roi import ROIData, load_roi
from planner.terrain import TerrainQuery


def main() -> None:
    roi = load_roi(DEFAULT_CONFIG)
    tq = TerrainQuery(roi)

    left, bottom, right, top = roi.bounds
    cx, cy = (left + right) / 2.0, (bottom + top) / 2.0

    cases = [("ROI center", (cx, cy), True)]

    interior_rowcols = [(50, 50), (166, 166), (300, 300), (10, 300), (300, 10)]
    for r, c in interior_rowcols:
        x, y = tq.rowcol_to_xy(r, c)
        cases.append((f"interior r{r}c{c}", (x, y), True))

    edge_rowcols = [(0, 0), (0, roi.width - 1), (roi.height - 1, 0), (roi.height - 1, roi.width - 1)]
    for r, c in edge_rowcols:
        x, y = tq.rowcol_to_xy(r, c)
        cases.append((f"edge r{r}c{c}", (x, y), True))

    cases.append(("outside ROI", (left - 5000.0, bottom - 5000.0), False))

    print(f"{'Label':<16}{'x':>13}{'y':>13}{'row':>6}{'col':>6}{'elev':>10}  Result")
    print("-" * 82)

    all_pass = True
    for label, (x, y), expect_valid in cases:
        result = tq.query(x, y)
        ok = True
        detail = ""

        if expect_valid:
            if not result.valid:
                ok = False
                detail = f"expected valid, got {result.reason}"
            else:
                direct = float(roi.elevation[result.row, result.col])
                if direct != result.elevation:
                    ok = False
                    detail = f"elevation mismatch vs array: {result.elevation} != {direct}"
        else:
            if result.valid:
                ok = False
                detail = "expected rejection, got valid"
            elif result.reason != "out_of_bounds":
                ok = False
                detail = f"expected reason=out_of_bounds, got {result.reason}"

        all_pass = all_pass and ok
        elev_str = f"{result.elevation:.1f}" if result.valid else "n/a"
        status = "PASS" if ok else f"FAIL ({detail})"
        print(f"{label:<16}{x:>13.1f}{y:>13.1f}{result.row:>6}{result.col:>6}{elev_str:>10}  {status}")

    print()
    print("in_bounds_xy direct checks:")
    ok = tq.in_bounds_xy(cx, cy) is True
    all_pass = all_pass and ok
    print(f"  ROI center               -> {tq.in_bounds_xy(cx, cy)!s:<6} (expect True)   {'PASS' if ok else 'FAIL'}")
    ok = tq.in_bounds_xy(left - 5000.0, bottom - 5000.0) is False
    all_pass = all_pass and ok
    print(f"  outside point             -> {tq.in_bounds_xy(left - 5000.0, bottom - 5000.0)!s:<6} (expect False)  {'PASS' if ok else 'FAIL'}")

    print()
    print("Round-trip (row,col) -> (x,y) -> (row,col):")
    rt_points = [(0, 0), (100, 200), (166, 166), (roi.height - 1, roi.width - 1)]
    for r, c in rt_points:
        x, y = tq.rowcol_to_xy(r, c)
        r2, c2 = tq.xy_to_rowcol(x, y)
        ok = (r2 == r) and (c2 == c)
        all_pass = all_pass and ok
        print(f"  ({r},{c}) -> ({x:.1f},{y:.1f}) -> ({r2},{c2})  {'PASS' if ok else 'FAIL'}")

    # Real ROI currently has 0 NoData pixels, so the NoData-rejection path
    # is exercised against a tiny synthetic ROIData instead.
    print()
    print("NoData handling (synthetic 3x3 ROI, center cell = nodata):")
    synth_nodata = -9999.0
    synthetic = ROIData(
        elevation=np.array(
            [[10.0, 20.0, 30.0], [40.0, synth_nodata, 60.0], [70.0, 80.0, 90.0]],
            dtype=np.float32,
        ),
        transform=Affine(30.0, 0.0, 0.0, 0.0, -30.0, 90.0),
        crs=roi.crs,
        width=3,
        height=3,
        bounds=(0.0, 0.0, 90.0, 90.0),
        resolution=(30.0, 30.0),
        nodata=synth_nodata,
    )
    synth_result = TerrainQuery(synthetic).elevation_at_rowcol(1, 1)
    ok = (not synth_result.valid) and (synth_result.reason == "nodata")
    all_pass = all_pass and ok
    print(f"  center cell -> valid={synth_result.valid}, reason={synth_result.reason}  {'PASS' if ok else 'FAIL'}")

    print()
    print(f"Overall: {'ALL PASS' if all_pass else 'SOME FAILED'}")

    n = 5000
    points = []
    for i in range(n):
        r = i % roi.height
        c = (i * 7) % roi.width  # deterministic but not row-aligned
        points.append(tq.rowcol_to_xy(r, c))

    t0 = time.perf_counter()
    for x, y in points:
        tq.query(x, y)
    elapsed = time.perf_counter() - t0

    print()
    print(f"Runtime: {n} queries in {elapsed * 1000:.2f} ms total, "
          f"{elapsed / n * 1e6:.2f} us/query average")


if __name__ == "__main__":
    main()
