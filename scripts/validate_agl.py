"""Stage validation: AGL feasibility gate (planner/agl.py).

Confirms evaluate_agl() correctly turns (x, y, aircraft_altitude_msl) into
a VALID/INVALID result against config.min_agl_m -- a 200.0 m TEST PARAMETER,
not a real mission requirement (see planner/config.py).
"""
import math

import numpy as np
from affine import Affine

from planner.agl import evaluate_agl
from planner.config import DEFAULT_CONFIG
from planner.roi import ROIData, load_roi
from planner.terrain import TerrainQuery


def main() -> None:
    cfg = DEFAULT_CONFIG
    assert cfg.min_agl_m == 200.0, "this validation assumes the TEST min_agl_m of 200.0 m"

    roi = load_roi(cfg)
    tq = TerrainQuery(roi)

    left, bottom, right, top = roi.bounds
    cx, cy = (left + right) / 2.0, (bottom + top) / 2.0
    terrain_msl = tq.query(cx, cy).elevation

    # (label, x, y, aircraft_altitude_msl, expect_valid, expect_reason)
    cases = [
        ("terrain +300m", cx, cy, terrain_msl + 300.0, True, "ok"),
        ("terrain +210m", cx, cy, terrain_msl + 210.0, True, "ok"),
        ("terrain +200m (exact)", cx, cy, terrain_msl + 200.0, True, "ok"),
        ("terrain +199m", cx, cy, terrain_msl + 199.0, False, "below_min_agl"),
        ("aircraft below terrain", cx, cy, terrain_msl - 50.0, False, "below_min_agl"),
        ("outside ROI", left - 5000.0, bottom - 5000.0, terrain_msl + 300.0, False, "out_of_bounds"),
    ]

    print(f"{'Case':<24}{'terrain':>10}{'aircraft':>11}{'AGL':>9}{'minAGL':>8}"
          f"{'expect':>9}{'actual':>9}  Result")
    print("-" * 100)

    all_pass = True
    for label, x, y, aircraft_msl, expect_valid, expect_reason in cases:
        result = evaluate_agl(tq, x, y, aircraft_msl, cfg)
        ok = (result.valid == expect_valid) and (result.reason == expect_reason)
        all_pass = all_pass and ok

        t_str = "n/a" if math.isnan(result.terrain_elevation_msl) else f"{result.terrain_elevation_msl:.1f}"
        agl_str = "n/a" if math.isnan(result.agl_m) else f"{result.agl_m:.1f}"
        expect_str = "VALID" if expect_valid else "INVALID"
        actual_str = "VALID" if result.valid else "INVALID"
        status = "PASS" if ok else f"FAIL (reason={result.reason}, expected={expect_reason})"

        print(f"{label:<24}{t_str:>10}{aircraft_msl:>11.1f}{agl_str:>9}{cfg.min_agl_m:>8.1f}"
              f"{expect_str:>9}{actual_str:>9}  {status}")

    # Synthetic NoData case -- real ROI currently has 0 NoData pixels, so
    # the nodata path is exercised against a tiny synthetic ROIData.
    print()
    print("Synthetic NoData terrain case:")
    synth_nodata = -9999.0
    synthetic = ROIData(
        elevation=np.array(
            [[10.0, 20.0, 30.0], [40.0, synth_nodata, 60.0], [70.0, 80.0, 90.0]],
            dtype=np.float32,
        ),
        transform=Affine(30.0, 0.0, 0.0, 0.0, -30.0, 90.0),
        crs=roi.crs, width=3, height=3,
        bounds=(0.0, 0.0, 90.0, 90.0), resolution=(30.0, 30.0), nodata=synth_nodata,
    )
    synth_tq = TerrainQuery(synthetic)
    sx, sy = synth_tq.rowcol_to_xy(1, 1)  # nodata cell center
    result = evaluate_agl(synth_tq, sx, sy, 2000.0, cfg)
    ok = (not result.valid) and (result.reason == "nodata")
    all_pass = all_pass and ok
    print(f"  nodata cell -> valid={result.valid}, reason={result.reason}  {'PASS' if ok else 'FAIL'}")

    print()
    print(f"Overall: {'ALL PASS' if all_pass else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
