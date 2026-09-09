"""Stage-0 validation: can we reliably load the 10x10 km Aladaglar ROI?

Runs the checks specified for this stage (CRS, resolution, extent, grid
size, NoData presence, elevation sanity, dtype/shape), prints a PASS/FAIL
table, and reports load runtime + the ROI array's RAM footprint.

No masking, cost, or search logic here -- loading and validation only.
"""
import time
import tracemalloc

import numpy as np

from planner.config import DEFAULT_CONFIG
from planner.roi import load_roi


def main() -> None:
    cfg = DEFAULT_CONFIG

    tracemalloc.start()
    t0 = time.perf_counter()
    roi = load_roi(cfg)
    elapsed = time.perf_counter() - t0
    _, peak_mem = tracemalloc.get_traced_memory()
    tracemalloc.stop()

    checks = []

    checks.append(("CRS == EPSG:32636", roi.crs, roi.crs == "EPSG:32636"))

    res_x, res_y = roi.resolution
    res_ok = abs(res_x - 30.0) < 0.5 and abs(res_y - 30.0) < 0.5
    checks.append(("Resolution ~30x30 m", f"{res_x:.3f} x {res_y:.3f} m", res_ok))

    left, bottom, right, top = roi.bounds
    roi_w, roi_h = right - left, top - bottom
    extent_ok = abs(roi_w - cfg.roi_size_m) < 100 and abs(roi_h - cfg.roi_size_m) < 100
    checks.append(("ROI extent ~10000x10000 m", f"{roi_w:.1f} x {roi_h:.1f} m", extent_ok))

    expected_px = cfg.roi_size_m / cfg.xy_resolution_m
    size_ok = abs(roi.width - expected_px) <= 2 and abs(roi.height - expected_px) <= 2
    checks.append(("Grid size ~333x333 px", f"{roi.width} x {roi.height} px", size_ok))

    if roi.nodata is not None:
        nodata_count = int(np.sum(roi.elevation == roi.nodata))
    else:
        nodata_count = 0
    checks.append(("No NoData in ROI", f"{nodata_count} px", nodata_count == 0))

    valid = roi.elevation[roi.elevation != roi.nodata] if roi.nodata is not None else roi.elevation.ravel()
    if valid.size > 0:
        elev_min, elev_max = float(valid.min()), float(valid.max())
        # Sanity range for the Aladaglar/Toroslar high-mountain region, not a physical planning limit.
        sane_ok = 500.0 < elev_min < 4000.0 and 500.0 < elev_max < 4200.0
        elev_str = f"{elev_min:.1f} - {elev_max:.1f} m"
    else:
        sane_ok = False
        elev_str = "no valid pixels"
    checks.append(("Elevation range sane", elev_str, sane_ok))

    shape_ok = roi.elevation.ndim == 2 and roi.elevation.dtype == np.float32
    checks.append(("dtype/shape", f"{roi.elevation.dtype}, {roi.elevation.shape}", shape_ok))

    name_w, val_w = 26, 26
    print(f"{'Check':<{name_w}}{'Value':<{val_w}}Result")
    print("-" * (name_w + val_w + 6))
    all_pass = True
    for name, value, ok in checks:
        result = "PASS" if ok else "FAIL"
        all_pass = all_pass and ok
        print(f"{name:<{name_w}}{str(value):<{val_w}}{result}")
    print("-" * (name_w + val_w + 6))
    print(f"Overall: {'ALL PASS' if all_pass else 'SOME FAILED'}")

    print()
    print(f"Runtime: {elapsed * 1000:.1f} ms")
    print(f"ROI array size in RAM: {roi.elevation.nbytes / (1024 ** 2):.2f} MB "
          f"(peak traced during load: {peak_mem / (1024 ** 2):.2f} MB)")


if __name__ == "__main__":
    main()
