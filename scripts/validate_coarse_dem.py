"""Stage 34: Conservative 90m coarse DEM -- construction + validation only.

NO A* search of any kind runs in this script (fine or coarse, no
weighted/incumbent/corridor/cost/heuristic work) -- purely terrain
representation and its correctness, per the Stage 34 spec's explicit
scope limit.

Tests:
  A) synthetic 3x3, center peak -> peak preserved
  B) synthetic 6x6 -> 2x2 coarse, correct per-block max
  C) peak in a block's corner -> still preserved
  D) a NoData fine cell in a block -> whole coarse cell NoData
  E) affine transform: 30m x factor(3) -> 90m resolution, origin unchanged
  F) world-coordinate mapping: a fine point maps to the correct coarse cell
  G) full real ROI: every valid coarse cell == its fine 3x3 block's max
  H) fine global max preserved somewhere in the coarse raster

Then: real-ROI terrain statistics (fine vs coarse), 5 local spot checks,
START/GOAL fine->UTM->coarse mapping with horizontal error, bounds/extent
comparison, and (only if everything above PASSES) writing the coarse
GeoTIFF to working_dem/aladaglar_roi_coarse_90m_max.tif -- the existing
fine DEM file is never opened for writing, only read.
"""
import math

import numpy as np
import rasterio
from affine import Affine
from rasterio.transform import rowcol as rio_rowcol
from rasterio.transform import xy as rio_xy

from planner.coarse import build_coarse_dem
from planner.config import DEFAULT_CONFIG
from planner.roi import ROIData, load_roi

START_ROW, START_COL = 48, 276
GOAL_ROW, GOAL_COL = 264, 276
FACTOR = 3
NODATA = -9999.0
OUTPUT_PATH = "working_dem/aladaglar_roi_coarse_90m_max.tif"


def make_synthetic_roi(elevation: np.ndarray, nodata=NODATA, res: float = 30.0) -> ROIData:
    height, width = elevation.shape
    transform = Affine(res, 0.0, 500000.0, 0.0, -res, 4200000.0)  # arbitrary but realistic UTM-ish origin
    return ROIData(
        elevation=elevation.astype(np.float32), transform=transform, crs="EPSG:32636",
        width=width, height=height, bounds=(500000.0, 4200000.0 - height * res, 500000.0 + width * res, 4200000.0),
        resolution=(res, res), nodata=nodata,
    )


# ----------------------------------------------------------------------
# A-D: synthetic max-pooling correctness.
# ----------------------------------------------------------------------

def test_a() -> bool:
    print("=== A: synthetic 3x3, center peak ===")
    elev = np.array([
        [3000.0, 3010.0, 3020.0],
        [2990.0, 3650.0, 3030.0],
        [3000.0, 3010.0, 3020.0],
    ])
    roi = make_synthetic_roi(elev)
    result = build_coarse_dem(roi, factor=3)
    ok = result.roi.elevation.shape == (1, 1) and float(result.roi.elevation[0, 0]) == 3650.0
    print(f"  coarse shape={result.roi.elevation.shape} value={result.roi.elevation[0, 0]} (expect 3650.0)  "
          f"{'PASS' if ok else 'FAIL'}")
    return ok


def test_b() -> bool:
    print()
    print("=== B: synthetic 6x6 -> 2x2 coarse ===")
    elev = np.arange(36, dtype=float).reshape(6, 6)
    # block (0,0) = rows0-2,cols0-2 -> max is elev[2,2]=14; block(0,1)=cols3-5 -> elev[2,5]=17
    # block (1,0)=rows3-5,cols0-2 -> elev[5,2]=32; block(1,1) -> elev[5,5]=35
    roi = make_synthetic_roi(elev)
    result = build_coarse_dem(roi, factor=3)
    expected = np.array([[14.0, 17.0], [32.0, 35.0]])
    ok = result.roi.elevation.shape == (2, 2) and np.array_equal(result.roi.elevation, expected)
    print(f"  coarse=\n{result.roi.elevation}\n  expected=\n{expected}\n  {'PASS' if ok else 'FAIL'}")
    return ok


def test_c() -> bool:
    print()
    print("=== C: peak in a block's corner ===")
    elev = np.full((3, 3), 3000.0)
    elev[0, 0] = 3999.0  # top-left corner of the block
    roi = make_synthetic_roi(elev)
    result = build_coarse_dem(roi, factor=3)
    ok = float(result.roi.elevation[0, 0]) == 3999.0
    print(f"  coarse value={result.roi.elevation[0, 0]} (expect 3999.0)  {'PASS' if ok else 'FAIL'}")
    return ok


def test_d() -> bool:
    print()
    print("=== D: a NoData fine cell in the block -> whole coarse cell NoData ===")
    elev = np.array([
        [3000.0, 3010.0, 3020.0],
        [2990.0, NODATA, 3030.0],
        [3000.0, 3010.0, 3020.0],
    ])
    roi = make_synthetic_roi(elev, nodata=NODATA)
    result = build_coarse_dem(roi, factor=3)
    ok = float(result.roi.elevation[0, 0]) == NODATA
    print(f"  coarse value={result.roi.elevation[0, 0]} (expect {NODATA})  {'PASS' if ok else 'FAIL'}")
    return ok


# ----------------------------------------------------------------------
# E: affine transform correctness.
# ----------------------------------------------------------------------

def test_e() -> bool:
    print()
    print("=== E: affine transform (30m x factor 3 -> 90m, origin preserved) ===")
    elev = np.zeros((9, 9))
    roi = make_synthetic_roi(elev, res=30.0)
    result = build_coarse_dem(roi, factor=3)
    ct = result.roi.transform
    ok = (abs(ct.a - 90.0) < 1e-9 and abs(ct.e - (-90.0)) < 1e-9
          and ct.c == roi.transform.c and ct.f == roi.transform.f
          and ct.b == roi.transform.b and ct.d == roi.transform.d)
    print(f"  coarse pixel size: ({ct.a},{ct.e}) (expect (90.0,-90.0))")
    print(f"  coarse origin: ({ct.c},{ct.f})  fine origin: ({roi.transform.c},{roi.transform.f})  "
          f"{'PASS' if ok else 'FAIL'}")
    return ok


# ----------------------------------------------------------------------
# F: world-coordinate mapping.
# ----------------------------------------------------------------------

def test_f() -> bool:
    print()
    print("=== F: world-coordinate mapping (fine point -> correct coarse cell) ===")
    elev = np.zeros((9, 9))
    roi = make_synthetic_roi(elev, res=30.0)
    result = build_coarse_dem(roi, factor=3)

    fine_row, fine_col = 7, 4  # expect coarse block (2, 1)
    x, y = rio_xy(roi.transform, fine_row, fine_col)
    coarse_row, coarse_col = rio_rowcol(result.roi.transform, x, y)
    ok = (coarse_row, coarse_col) == (2, 1)
    print(f"  fine (row={fine_row},col={fine_col}) -> UTM ({x:.1f},{y:.1f}) -> coarse (row={coarse_row},"
          f"col={coarse_col}) (expect (2,1))  {'PASS' if ok else 'FAIL'}")
    return ok


# ----------------------------------------------------------------------
# G/H: full real ROI validation.
# ----------------------------------------------------------------------

def conservatism_check(fine_roi: ROIData, coarse_result) -> dict:
    fine = fine_roi.elevation
    coarse = coarse_result.roi.elevation
    factor = coarse_result.factor
    ch, cw = coarse.shape

    cropped = fine[: ch * factor, : cw * factor]
    blocks = cropped.reshape(ch, factor, cw, factor)
    expected_max = blocks.max(axis=(1, 3))

    if fine_roi.nodata is not None:
        valid_mask = coarse != fine_roi.nodata
    else:
        valid_mask = np.ones_like(coarse, dtype=bool)

    diffs = np.abs(coarse[valid_mask].astype(np.float64) - expected_max[valid_mask].astype(np.float64))
    violations = int(np.sum(diffs > 1e-9))
    return {
        "number_of_blocks_checked": int(valid_mask.sum()),
        "number_of_violations": violations,
        "maximum_difference": float(diffs.max()) if diffs.size else 0.0,
    }


def test_g_h(fine_roi: ROIData, coarse_result) -> bool:
    print()
    print("=== G/H: full real-ROI conservatism + global-max preservation ===")
    report = conservatism_check(fine_roi, coarse_result)
    print(f"  number_of_blocks_checked={report['number_of_blocks_checked']}  "
          f"number_of_violations={report['number_of_violations']}  "
          f"maximum_difference={report['maximum_difference']}")
    ok_g = report["number_of_violations"] == 0

    if fine_roi.nodata is not None:
        fine_valid = fine_roi.elevation[fine_roi.elevation != fine_roi.nodata]
        coarse_valid = coarse_result.roi.elevation[coarse_result.roi.elevation != fine_roi.nodata]
    else:
        fine_valid = fine_roi.elevation
        coarse_valid = coarse_result.roi.elevation
    fine_max = float(fine_valid.max())
    coarse_max = float(coarse_valid.max())
    ok_h = fine_max == coarse_max
    print(f"  fine global max={fine_max}  coarse global max={coarse_max}  preserved={ok_h}")
    print(f"  {'PASS' if (ok_g and ok_h) else 'FAIL'}")
    return ok_g and ok_h


# ----------------------------------------------------------------------
# Terrain statistics.
# ----------------------------------------------------------------------

def terrain_stats(roi: ROIData, label: str) -> dict:
    if roi.nodata is not None:
        valid = roi.elevation[roi.elevation != roi.nodata]
        nodata_count = int((roi.elevation == roi.nodata).sum())
    else:
        valid = roi.elevation.ravel()
        nodata_count = 0
    stats = {
        "shape": roi.elevation.shape, "resolution": roi.resolution, "crs": roi.crs, "bounds": roi.bounds,
        "valid_cell_count": int(valid.size), "nodata_count": nodata_count,
        "min_elevation": float(valid.min()) if valid.size else float("nan"),
        "max_elevation": float(valid.max()) if valid.size else float("nan"),
        "mean_elevation": float(valid.mean()) if valid.size else float("nan"),
    }
    print(f"  [{label}] shape={stats['shape']} res={stats['resolution']} crs={stats['crs']}")
    print(f"    bounds={tuple(round(b, 1) for b in stats['bounds'])}")
    print(f"    valid_cells={stats['valid_cell_count']} nodata_cells={stats['nodata_count']}")
    print(f"    min={stats['min_elevation']:.2f} max={stats['max_elevation']:.2f} mean={stats['mean_elevation']:.2f}")
    return stats


# ----------------------------------------------------------------------
# Local spot tests.
# ----------------------------------------------------------------------

def local_spot_tests(fine_roi: ROIData, coarse_result, coarse_cells) -> None:
    print()
    print("=== Local spot tests (5 coarse cells) ===")
    factor = coarse_result.factor
    for (cr, cc) in coarse_cells:
        fr0, fr1 = cr * factor, cr * factor + factor
        fc0, fc1 = cc * factor, cc * factor + factor
        block = fine_roi.elevation[fr0:fr1, fc0:fc1]
        block_max = float(block.max())
        coarse_val = float(coarse_result.roi.elevation[cr, cc])
        print(f"  coarse(row={cr},col={cc})  fine_rows=[{fr0}:{fr1}) fine_cols=[{fc0}:{fc1})")
        print(f"    fine_block=\n{block}")
        print(f"    fine_block_max={block_max}  coarse_elevation={coarse_val}  "
              f"match={'YES' if block_max == coarse_val else 'NO'}")


# ----------------------------------------------------------------------
# START/GOAL mapping.
# ----------------------------------------------------------------------

def map_point(label: str, fine_roi: ROIData, coarse_result, fine_row: int, fine_col: int) -> None:
    x, y = rio_xy(fine_roi.transform, fine_row, fine_col)
    coarse_row, coarse_col = rio_rowcol(coarse_result.roi.transform, x, y)
    naive_row, naive_col = fine_row // coarse_result.factor, fine_col // coarse_result.factor
    cell_x, cell_y = rio_xy(coarse_result.roi.transform, coarse_row, coarse_col)
    mapping_error = math.hypot(x - cell_x, y - cell_y)
    print(f"  {label}: fine(row={fine_row},col={fine_col}) -> UTM(x={x:.2f},y={y:.2f})")
    print(f"    coarse(row={coarse_row},col={coarse_col})  cell_center=(x={cell_x:.2f},y={cell_y:.2f})  "
          f"naive_integer_division=(row={naive_row},col={naive_col})  "
          f"agrees_with_transform_mapping={(coarse_row, coarse_col) == (naive_row, naive_col)}")
    print(f"    mapping_horizontal_error_to_cell_center_m={mapping_error:.2f}")


# ----------------------------------------------------------------------
# Bounds/extent test.
# ----------------------------------------------------------------------

def bounds_extent_test(fine_roi: ROIData, coarse_result) -> None:
    print()
    print("=== Bounds/extent comparison ===")
    fl, fb, fr_, ft = fine_roi.bounds
    cl, cb, cr_, ct = coarse_result.roi.bounds
    print(f"  fine   bounds: left={fl:.2f} bottom={fb:.2f} right={fr_:.2f} top={ft:.2f}")
    print(f"  coarse bounds: left={cl:.2f} bottom={cb:.2f} right={cr_:.2f} top={ct:.2f}")
    print(f"  origin (left,top) match: {(fl, ft) == (cl, ct)}")
    dropped_right_m = fr_ - cr_
    dropped_bottom_m = fb - cb
    print(f"  right-edge crop: {dropped_right_m:.2f}m ({coarse_result.dropped_cols} fine cols dropped)")
    print(f"  bottom-edge crop: {dropped_bottom_m:.2f}m ({coarse_result.dropped_rows} fine rows dropped)")


def main() -> None:
    results = [test_a(), test_b(), test_c(), test_d(), test_e(), test_f()]

    cfg = DEFAULT_CONFIG
    fine_roi = load_roi(cfg)
    coarse_result = build_coarse_dem(fine_roi, factor=FACTOR)

    results.append(test_g_h(fine_roi, coarse_result))

    print()
    print("=== Terrain statistics ===")
    terrain_stats(fine_roi, "FINE")
    terrain_stats(coarse_result.roi, "COARSE")
    print(f"  dropped_rows={coarse_result.dropped_rows}  dropped_cols={coarse_result.dropped_cols} "
          f"(fine shape {fine_roi.elevation.shape} -> coarse shape {coarse_result.roi.elevation.shape}, factor={FACTOR})")

    # 5 spot checks -- pick a spread including the global max peak's own coarse cell.
    fine_valid_mask = fine_roi.elevation != fine_roi.nodata if fine_roi.nodata is not None else np.ones_like(
        fine_roi.elevation, dtype=bool)
    peak_row, peak_col = np.unravel_index(np.argmax(np.where(fine_valid_mask, fine_roi.elevation, -np.inf)),
                                           fine_roi.elevation.shape)
    peak_coarse = (peak_row // FACTOR, peak_col // FACTOR)
    ch, cw = coarse_result.roi.elevation.shape
    spot_cells = [peak_coarse, (0, 0), (ch - 1, cw - 1), (ch // 2, cw // 2), (START_ROW // FACTOR, START_COL // FACTOR)]
    local_spot_tests(fine_roi, coarse_result, spot_cells)

    print()
    print("=== START/GOAL fine -> UTM -> coarse mapping ===")
    map_point("START", fine_roi, coarse_result, START_ROW, START_COL)
    map_point("GOAL", fine_roi, coarse_result, GOAL_ROW, GOAL_COL)

    bounds_extent_test(fine_roi, coarse_result)

    print()
    overall_pass = all(results)
    print(f"Overall synthetic+real validation: {'ALL PASS' if overall_pass else 'SOME FAILED'}")

    if overall_pass:
        with rasterio.open(
            OUTPUT_PATH, "w", driver="GTiff",
            height=coarse_result.roi.height, width=coarse_result.roi.width, count=1,
            dtype=coarse_result.roi.elevation.dtype, crs=coarse_result.roi.crs,
            transform=coarse_result.roi.transform,
            nodata=coarse_result.roi.nodata,
        ) as dst:
            dst.write(coarse_result.roi.elevation, 1)
        print(f"Coarse DEM written to {OUTPUT_PATH}")
        print(f"  crs={coarse_result.roi.crs}  transform={coarse_result.roi.transform}  "
              f"width={coarse_result.roi.width}  height={coarse_result.roi.height}  "
              f"nodata={coarse_result.roi.nodata}  dtype={coarse_result.roi.elevation.dtype}")
    else:
        print("Validation FAILED -- coarse DEM NOT written.")


if __name__ == "__main__":
    main()
