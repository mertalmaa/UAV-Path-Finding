"""Stage 38.1: Fine Corridor Safety Precompute -- equivalence tests for
planner.fine_precompute (the fine-grid, corridor-restricted, dense-NumPy-
array analog of planner.coarse_astar's Stage 35.1 precompute). Old
evaluator = evaluate_primitive() (per-sample terrain query); new evaluator
= fine_precomputed_primitive_validity() (O(1) dense-array lookup). Every
test requires mismatch=0 between the two. Does NOT re-run any old PASS
benchmark -- purely new tests for the new evaluator.
"""
import random

import numpy as np
from affine import Affine

from planner.config import DEFAULT_CONFIG
from planner.fine_precompute import fine_precomputed_primitive_validity, precompute_fine_corridor_primitive_safety
from planner.primitives import build_primitive_set, evaluate_primitive
from planner.roi import ROIData, load_roi
from planner.terrain import TerrainQuery

NODATA = -9999.0
XY_MASK_NPY = "outputs/stage36_corridor_mask.npy"


def make_synthetic_roi(elevation: np.ndarray, nodata=NODATA, res: float = 30.0) -> ROIData:
    height, width = elevation.shape
    transform = Affine(res, 0.0, 500000.0, 0.0, -res, 4200000.0)
    return ROIData(
        elevation=elevation.astype(np.float32), transform=transform, crs="EPSG:32636",
        width=width, height=height, bounds=(500000.0, 4200000.0 - height * res, 500000.0 + width * res, 4200000.0),
        resolution=(res, res), nodata=nodata,
    )


def _reference_validity(terrain, prim, row, col, start_msl, config):
    start_xyz = (*terrain.rowcol_to_xy(row, col), start_msl)
    result = evaluate_primitive(start_xyz, prim, terrain, config)
    return result.valid, result.reason


def _compare_all(terrain, corridor_mask, primitives, precompute, config, start_msls, label) -> int:
    mismatches = 0
    checked = 0
    for row in range(terrain.roi.height):
        for col in range(terrain.roi.width):
            if not corridor_mask[row, col]:
                continue
            for prim_idx, prim in enumerate(primitives):
                for start_msl in start_msls:
                    checked += 1
                    old_valid, old_reason = _reference_validity(terrain, prim, row, col, start_msl, config)
                    new_valid, new_reason = fine_precomputed_primitive_validity(precompute, row, col, prim_idx, start_msl)
                    if old_valid != new_valid:
                        mismatches += 1
                        if mismatches <= 5:
                            print(f"    MISMATCH row={row} col={col} prim={prim.direction}/{prim.primitive_type} "
                                  f"start_msl={start_msl}: old=({old_valid},{old_reason}) new=({new_valid},{new_reason})")
    print(f"  [{label}] checked={checked} mismatches={mismatches}")
    return mismatches


def test_1_level_climb_descent_flat() -> bool:
    print("=== 1: synthetic flat terrain, level/climb/descent all match ===")
    elev = np.full((10, 10), 1000.0)
    tq = TerrainQuery(make_synthetic_roi(elev))
    mask = np.ones((10, 10), dtype=bool)
    prims = build_primitive_set(DEFAULT_CONFIG)
    precompute = precompute_fine_corridor_primitive_safety(tq, mask, prims, DEFAULT_CONFIG)
    all_ok = True
    for kind in ("level", "climb", "descent"):
        idx_prims = [(i, p) for i, p in enumerate(prims) if p.primitive_type == kind]
        mism = 0
        for row in range(tq.roi.height):
            for col in range(tq.roi.width):
                for i, p in idx_prims:
                    for start_msl in (1080.0, 1100.0, 1120.0):
                        old_valid, old_reason = _reference_validity(tq, p, row, col, start_msl, DEFAULT_CONFIG)
                        new_valid, new_reason = fine_precomputed_primitive_validity(precompute, row, col, i, start_msl)
                        if old_valid != new_valid:
                            mism += 1
        print(f"  [{kind}] mismatches={mism}")
        all_ok = all_ok and mism == 0
    return all_ok


def test_2_ridge() -> bool:
    print()
    print("=== 2: mid-segment ridge, old==new ===")
    elev = np.full((8, 8), 1000.0)
    elev[4, 4] = 1150.0
    tq = TerrainQuery(make_synthetic_roi(elev))
    mask = np.ones((8, 8), dtype=bool)
    prims = build_primitive_set(DEFAULT_CONFIG)
    precompute = precompute_fine_corridor_primitive_safety(tq, mask, prims, DEFAULT_CONFIG)
    mismatches = _compare_all(tq, mask, prims, precompute, DEFAULT_CONFIG, [1150.0, 1250.0, 1350.0], "ridge")
    return mismatches == 0


def test_3_bounds() -> bool:
    print()
    print("=== 3: bounds crossing (edge of a small grid), old==new ===")
    elev = np.full((5, 5), 1000.0)
    tq = TerrainQuery(make_synthetic_roi(elev))
    mask = np.ones((5, 5), dtype=bool)
    prims = build_primitive_set(DEFAULT_CONFIG)
    precompute = precompute_fine_corridor_primitive_safety(tq, mask, prims, DEFAULT_CONFIG)
    mismatches = _compare_all(tq, mask, prims, precompute, DEFAULT_CONFIG, [1150.0, 1250.0], "bounds")
    return mismatches == 0


def test_4_nodata() -> bool:
    print()
    print("=== 4: NoData crossing, old==new ===")
    elev = np.full((8, 8), 1000.0)
    elev[4, 4] = NODATA
    tq = TerrainQuery(make_synthetic_roi(elev, nodata=NODATA))
    mask = np.ones((8, 8), dtype=bool)
    prims = build_primitive_set(DEFAULT_CONFIG)
    precompute = precompute_fine_corridor_primitive_safety(tq, mask, prims, DEFAULT_CONFIG)
    mismatches = _compare_all(tq, mask, prims, precompute, DEFAULT_CONFIG, [1150.0, 1250.0], "nodata")
    return mismatches == 0


def test_5_start_msl_levels() -> bool:
    print()
    print("=== 5: many different start MSL levels, old/new validity identical ===")
    elev = np.random.RandomState(7).uniform(3000.0, 3400.0, size=(10, 10))
    tq = TerrainQuery(make_synthetic_roi(elev))
    mask = np.ones((10, 10), dtype=bool)
    prims = build_primitive_set(DEFAULT_CONFIG)
    precompute = precompute_fine_corridor_primitive_safety(tq, mask, prims, DEFAULT_CONFIG)
    start_msls = [3200.0 + 20.0 * i for i in range(30)]  # 3200..3780 step 20 (z_step_m)
    mismatches = _compare_all(tq, mask, prims, precompute, DEFAULT_CONFIG, start_msls, "multi-level")
    return mismatches == 0


def test_6_outside_corridor_marked_invalid() -> bool:
    print()
    print("=== 6: cells OUTSIDE the corridor mask are never computed, always invalid ===")
    elev = np.full((10, 10), 1000.0)
    tq = TerrainQuery(make_synthetic_roi(elev))
    mask = np.zeros((10, 10), dtype=bool)
    mask[3:7, 3:7] = True  # only a small interior block is "in corridor"
    prims = build_primitive_set(DEFAULT_CONFIG)
    precompute = precompute_fine_corridor_primitive_safety(tq, mask, prims, DEFAULT_CONFIG)
    ok = True
    for row in range(10):
        for col in range(10):
            if mask[row, col]:
                continue
            for i in range(len(prims)):
                valid, reason = fine_precomputed_primitive_validity(precompute, row, col, i, 1200.0)
                if valid or reason != "outside_corridor_not_computed":
                    ok = False
                    print(f"    UNEXPECTED row={row} col={col} prim_idx={i}: valid={valid} reason={reason}")
    print(f"  outside-corridor-always-invalid: {'PASS' if ok else 'FAIL'}")
    print(f"  corridor_cell_count={precompute.corridor_cell_count} (expected 16)")
    return ok and precompute.corridor_cell_count == 16


def test_7_real_corridor_random(fine_roi, corridor_mask) -> bool:
    print()
    print("=== 7: real fine ROI + Stage 36 corridor mask, random real corridor samples ===")
    tq = TerrainQuery(fine_roi)
    prims = build_primitive_set(DEFAULT_CONFIG)
    precompute = precompute_fine_corridor_primitive_safety(tq, corridor_mask, prims, DEFAULT_CONFIG)

    rows, cols = np.nonzero(corridor_mask)
    rng = random.Random(38)
    mismatches = 0
    n_checks = 4000
    for _ in range(n_checks):
        k = rng.randrange(len(rows))
        row, col = int(rows[k]), int(cols[k])
        prim_idx = rng.randrange(len(prims))
        prim = prims[prim_idx]
        start_msl = rng.choice([3300.0 + 20.0 * i for i in range(30)])
        old_valid, old_reason = _reference_validity(tq, prim, row, col, start_msl, DEFAULT_CONFIG)
        new_valid, new_reason = fine_precomputed_primitive_validity(precompute, row, col, prim_idx, start_msl)
        if old_valid != new_valid:
            mismatches += 1
            if mismatches <= 5:
                print(f"    MISMATCH row={row} col={col} prim={prim.direction}/{prim.primitive_type} "
                      f"start_msl={start_msl}: old=({old_valid},{old_reason}) new=({new_valid},{new_reason})")
    print(f"  random real-corridor checks={n_checks} mismatches={mismatches}")
    print(f"  entry_count={precompute.entry_count}  corridor_cell_count={precompute.corridor_cell_count}  "
          f"preprocessing_time={precompute.preprocessing_runtime_s:.3f}s  "
          f"approx_memory_MB={precompute.approx_memory_mb:.2f}  "
          f"static_invalid_count={precompute.static_invalid_count}")
    return mismatches == 0


def main() -> None:
    results = [
        test_1_level_climb_descent_flat(), test_2_ridge(), test_3_bounds(), test_4_nodata(),
        test_5_start_msl_levels(), test_6_outside_corridor_marked_invalid(),
    ]

    fine_roi = load_roi(DEFAULT_CONFIG)
    corridor_mask = np.load(XY_MASK_NPY)
    results.append(test_7_real_corridor_random(fine_roi, corridor_mask))

    print()
    all_pass = all(results)
    print(f"Equivalence tests: {'ALL PASS (0 mismatches everywhere)' if all_pass else 'SOME FAILED -- mismatch found'}")


if __name__ == "__main__":
    main()
