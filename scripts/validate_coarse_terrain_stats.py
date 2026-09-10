"""Stage 34.5: coarse terrain summary (min/mean/max/relief per 90m cell) --
construction + validation only. NO A*/planner/cost work of any kind runs
here, exactly like Stage 34.

Tests:
  A) uniform 3x3 block -> min=mean=max=block value, relief=0
  B) single center peak -> min/max/relief correct, mean correct
  C) single low valley cell -> min/max/relief correct
  D) 6x6 input, factor=3 -> all four layers 2x2
  E) a NoData fine cell in the block -> min/mean/max/relief all NoData
  F) relief == max - min for every valid cell
  G) ordering: min <= mean <= max for every valid cell
  H) real ROI: new max layer == Stage 34's own coarse MAX DEM (both the
     in-memory build_coarse_dem() result AND the already-written GeoTIFF)

Then: real-ROI statistics per layer, top-5 high-relief cells (diagnostic
only), 5 "MAX-would-hide-low-terrain" example cells, START/GOAL cell
summaries, and (only if everything PASSES) writing the three NEW GeoTIFFs
(min/mean/relief) -- the existing max.tif is read back for comparison but
NEVER rewritten in this script.
"""
import numpy as np
import rasterio
from affine import Affine

from planner.coarse import build_coarse_dem, build_coarse_terrain_stats
from planner.config import DEFAULT_CONFIG
from planner.roi import ROIData, load_roi

FACTOR = 3
NODATA = -9999.0
START_ROW, START_COL = 48, 276
GOAL_ROW, GOAL_COL = 264, 276
START_COARSE = (16, 92)
GOAL_COARSE = (88, 92)
EXISTING_MAX_TIF = "working_dem/aladaglar_roi_coarse_90m_max.tif"
OUT_MIN_TIF = "working_dem/aladaglar_roi_coarse_90m_min.tif"
OUT_MEAN_TIF = "working_dem/aladaglar_roi_coarse_90m_mean.tif"
OUT_RELIEF_TIF = "working_dem/aladaglar_roi_coarse_90m_relief.tif"


def make_synthetic_roi(elevation: np.ndarray, nodata=NODATA, res: float = 30.0) -> ROIData:
    height, width = elevation.shape
    transform = Affine(res, 0.0, 500000.0, 0.0, -res, 4200000.0)
    return ROIData(
        elevation=elevation.astype(np.float32), transform=transform, crs="EPSG:32636",
        width=width, height=height, bounds=(500000.0, 4200000.0 - height * res, 500000.0 + width * res, 4200000.0),
        resolution=(res, res), nodata=nodata,
    )


def test_a() -> bool:
    print("=== A: uniform 3x3 block ===")
    roi = make_synthetic_roi(np.full((3, 3), 3000.0))
    s = build_coarse_terrain_stats(roi, factor=3)
    ok = (float(s.min_elevation[0, 0]) == 3000.0 and float(s.mean_elevation[0, 0]) == 3000.0
          and float(s.max_elevation[0, 0]) == 3000.0 and float(s.relief[0, 0]) == 0.0)
    print(f"  min={s.min_elevation[0, 0]} mean={s.mean_elevation[0, 0]} max={s.max_elevation[0, 0]} "
          f"relief={s.relief[0, 0]}  {'PASS' if ok else 'FAIL'}")
    return ok


def test_b() -> bool:
    print()
    print("=== B: single center peak ===")
    elev = np.array([
        [3000.0, 3000.0, 3000.0],
        [3000.0, 3600.0, 3000.0],
        [3000.0, 3000.0, 3000.0],
    ])
    roi = make_synthetic_roi(elev)
    s = build_coarse_terrain_stats(roi, factor=3)
    expected_mean = elev.mean()
    ok = (float(s.min_elevation[0, 0]) == 3000.0 and float(s.max_elevation[0, 0]) == 3600.0
          and float(s.relief[0, 0]) == 600.0 and abs(float(s.mean_elevation[0, 0]) - expected_mean) < 1e-3)
    print(f"  min={s.min_elevation[0, 0]} max={s.max_elevation[0, 0]} relief={s.relief[0, 0]} "
          f"mean={s.mean_elevation[0, 0]:.4f} (expect {expected_mean:.4f})  {'PASS' if ok else 'FAIL'}")
    return ok


def test_c() -> bool:
    print()
    print("=== C: single low valley cell ===")
    elev = np.array([
        [3500.0, 3500.0, 3500.0],
        [3500.0, 2900.0, 3500.0],
        [3500.0, 3500.0, 3500.0],
    ])
    roi = make_synthetic_roi(elev)
    s = build_coarse_terrain_stats(roi, factor=3)
    ok = (float(s.min_elevation[0, 0]) == 2900.0 and float(s.max_elevation[0, 0]) == 3500.0
          and float(s.relief[0, 0]) == 600.0)
    print(f"  min={s.min_elevation[0, 0]} max={s.max_elevation[0, 0]} relief={s.relief[0, 0]}  "
          f"{'PASS' if ok else 'FAIL'}")
    return ok


def test_d() -> bool:
    print()
    print("=== D: 6x6 -> 2x2 for all four layers ===")
    elev = np.arange(36, dtype=float).reshape(6, 6)
    roi = make_synthetic_roi(elev)
    s = build_coarse_terrain_stats(roi, factor=3)
    shapes = {s.min_elevation.shape, s.mean_elevation.shape, s.max_elevation.shape, s.relief.shape}
    ok = shapes == {(2, 2)}
    print(f"  shapes: min={s.min_elevation.shape} mean={s.mean_elevation.shape} "
          f"max={s.max_elevation.shape} relief={s.relief.shape}  {'PASS' if ok else 'FAIL'}")
    return ok


def test_e() -> bool:
    print()
    print("=== E: NoData fine cell -> all four stats NoData ===")
    elev = np.array([
        [3000.0, 3010.0, 3020.0],
        [2990.0, NODATA, 3030.0],
        [3000.0, 3010.0, 3020.0],
    ])
    roi = make_synthetic_roi(elev, nodata=NODATA)
    s = build_coarse_terrain_stats(roi, factor=3)
    ok = (float(s.min_elevation[0, 0]) == NODATA and float(s.mean_elevation[0, 0]) == NODATA
          and float(s.max_elevation[0, 0]) == NODATA and float(s.relief[0, 0]) == NODATA)
    print(f"  min={s.min_elevation[0, 0]} mean={s.mean_elevation[0, 0]} max={s.max_elevation[0, 0]} "
          f"relief={s.relief[0, 0]} (all expect {NODATA})  {'PASS' if ok else 'FAIL'}")
    return ok


def test_f_g(stats) -> bool:
    print()
    print("=== F/G: relief==max-min and min<=mean<=max for every valid cell (real ROI) ===")
    valid = stats.max_elevation != stats.nodata if stats.nodata is not None else np.ones_like(stats.max_elevation, dtype=bool)
    relief_check = np.abs(stats.relief[valid].astype(np.float64)
                           - (stats.max_elevation[valid].astype(np.float64) - stats.min_elevation[valid].astype(np.float64)))
    f_violations = int(np.sum(relief_check > 1e-6))
    ordering_violations = int(np.sum(
        (stats.min_elevation[valid].astype(np.float64) > stats.mean_elevation[valid].astype(np.float64) + 1e-6)
        | (stats.mean_elevation[valid].astype(np.float64) > stats.max_elevation[valid].astype(np.float64) + 1e-6)
    ))
    print(f"  valid_cells={int(valid.sum())}  relief_formula_violations={f_violations}  "
          f"ordering_violations={ordering_violations}")
    ok = f_violations == 0 and ordering_violations == 0
    print(f"  {'PASS' if ok else 'FAIL'}")
    return ok


def test_h(fine_roi, stats) -> bool:
    print()
    print("=== H: new MAX layer == Stage 34 MAX DEM (in-memory AND saved GeoTIFF) ===")
    in_memory_max = build_coarse_dem(fine_roi, factor=FACTOR).roi.elevation
    match_in_memory = np.array_equal(stats.max_elevation, in_memory_max)

    with rasterio.open(EXISTING_MAX_TIF) as ds:
        saved_max = ds.read(1)
    match_saved = np.array_equal(stats.max_elevation, saved_max)

    print(f"  matches in-memory build_coarse_dem(): {match_in_memory}")
    print(f"  matches saved {EXISTING_MAX_TIF}: {match_saved}")
    ok = match_in_memory and match_saved
    print(f"  {'PASS' if ok else 'FAIL'}")
    return ok


def layer_report(name: str, array: np.ndarray, nodata) -> None:
    if nodata is not None:
        valid = array[array != nodata]
        nodata_count = int((array == nodata).sum())
    else:
        valid = array.ravel()
        nodata_count = 0
    print(f"  [{name}] valid_count={valid.size} nodata_count={nodata_count} "
          f"min={valid.min():.2f} max={valid.max():.2f} mean={valid.mean():.2f}")


def relief_report(relief: np.ndarray, nodata) -> np.ndarray:
    valid = relief[relief != nodata] if nodata is not None else relief.ravel()
    print(f"  [RELIEF] min={valid.min():.2f} max={valid.max():.2f} mean={valid.mean():.2f} "
          f"median={np.median(valid):.2f} p90={np.percentile(valid, 90):.2f} p95={np.percentile(valid, 95):.2f}")
    return valid


def high_relief_cells(fine_roi, stats, top_n: int = 5) -> None:
    print()
    print(f"=== Top {top_n} highest-relief coarse cells (diagnostic only) ===")
    valid = stats.max_elevation != stats.nodata if stats.nodata is not None else np.ones_like(stats.relief, dtype=bool)
    relief_masked = np.where(valid, stats.relief, -np.inf)
    flat_order = np.argsort(relief_masked.ravel())[::-1][:top_n]
    coords = [np.unravel_index(idx, relief_masked.shape) for idx in flat_order]
    for (cr, cc) in coords:
        cx, cy = rasterio.transform.xy(stats.transform, cr, cc)
        fr0, fr1 = cr * FACTOR, cr * FACTOR + FACTOR
        fc0, fc1 = cc * FACTOR, cc * FACTOR + FACTOR
        block = fine_roi.elevation[fr0:fr1, fc0:fc1]
        print(f"  coarse(row={cr},col={cc}) UTM_center=({cx:.1f},{cy:.1f})  "
              f"min={stats.min_elevation[cr, cc]:.1f} mean={stats.mean_elevation[cr, cc]:.1f} "
              f"max={stats.max_elevation[cr, cc]:.1f} relief={stats.relief[cr, cc]:.1f}")
        print(f"    fine_block=\n{block}")


def valley_evidence_cells(fine_roi, stats, n: int = 5) -> None:
    print()
    print(f"=== {n} example cells where MAX alone hides low terrain (diagnostic only, NOT a valley claim) ===")
    valid = stats.max_elevation != stats.nodata if stats.nodata is not None else np.ones_like(stats.relief, dtype=bool)
    min_f = stats.min_elevation.astype(np.float64)
    mean_f = stats.mean_elevation.astype(np.float64)
    max_f = stats.max_elevation.astype(np.float64)
    # rank by how far min sits below mean, relative to mean-vs-max spread --
    # i.e. "min is a clear outlier low point, not just gentle overall slope".
    gap_below_mean = np.where(valid, mean_f - min_f, -np.inf)
    order = np.argsort(gap_below_mean.ravel())[::-1][:n]
    coords = [np.unravel_index(idx, gap_below_mean.shape) for idx in order]
    for (cr, cc) in coords:
        fr0, fr1 = cr * FACTOR, cr * FACTOR + FACTOR
        fc0, fc1 = cc * FACTOR, cc * FACTOR + FACTOR
        block = fine_roi.elevation[fr0:fr1, fc0:fc1]
        print(f"  coarse(row={cr},col={cc})  min={stats.min_elevation[cr, cc]:.1f} << "
              f"mean={stats.mean_elevation[cr, cc]:.1f} < max={stats.max_elevation[cr, cc]:.1f}")
        print(f"    fine_block=\n{block}")


def cell_summary(label: str, stats, row: int, col: int) -> None:
    print(f"  {label} coarse(row={row},col={col}): min={stats.min_elevation[row, col]:.2f} "
          f"mean={stats.mean_elevation[row, col]:.2f} max={stats.max_elevation[row, col]:.2f} "
          f"relief={stats.relief[row, col]:.2f}")


def write_tif(path: str, array: np.ndarray, stats) -> None:
    with rasterio.open(
        path, "w", driver="GTiff",
        height=array.shape[0], width=array.shape[1], count=1,
        dtype=array.dtype, crs=stats.crs, transform=stats.transform, nodata=stats.nodata,
    ) as dst:
        dst.write(array, 1)
    print(f"  wrote {path}  dtype={array.dtype}  shape={array.shape}")


def main() -> None:
    results = [test_a(), test_b(), test_c(), test_d(), test_e()]

    cfg = DEFAULT_CONFIG
    fine_roi = load_roi(cfg)
    stats = build_coarse_terrain_stats(fine_roi, factor=FACTOR)

    results.append(test_f_g(stats))
    results.append(test_h(fine_roi, stats))

    print()
    print("=== Real ROI terrain statistics per layer ===")
    layer_report("MIN", stats.min_elevation, stats.nodata)
    layer_report("MEAN", stats.mean_elevation, stats.nodata)
    layer_report("MAX", stats.max_elevation, stats.nodata)
    relief_report(stats.relief, stats.nodata)

    high_relief_cells(fine_roi, stats, top_n=5)
    valley_evidence_cells(fine_roi, stats, n=5)

    print()
    print("=== START/GOAL coarse cell terrain summary ===")
    cell_summary("START", stats, *START_COARSE)
    cell_summary("GOAL", stats, *GOAL_COARSE)

    print()
    overall_pass = all(results)
    print(f"Overall Stage 34.5 validation: {'ALL PASS' if overall_pass else 'SOME FAILED'}")

    if overall_pass:
        write_tif(OUT_MIN_TIF, stats.min_elevation, stats)
        write_tif(OUT_MEAN_TIF, stats.mean_elevation, stats)
        write_tif(OUT_RELIEF_TIF, stats.relief, stats)
        print(f"(existing {EXISTING_MAX_TIF} was only READ for comparison in test H -- never rewritten here)")
    else:
        print("Validation FAILED -- no new GeoTIFFs written.")


if __name__ == "__main__":
    main()
