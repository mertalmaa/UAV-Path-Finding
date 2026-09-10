"""Stage 37: small integration tests for the fine A*'s new optional
corridor_mask parameter (planner.astar._generate_neighbors/astar_search).
No real benchmark here -- see scripts/benchmark_corridor_fine_real.py for
the one real 6.48km run.

1. corridor_mask=None -> identical to the pre-Stage-37 neighbor behavior
   (an all-True mask must reproduce it exactly, since "everywhere is
   allowed" and "no corridor check at all" mean the same thing).
2. a successor cell outside the corridor is rejected (outside_corridor),
   before ever reaching evaluate_primitive.
3. a successor cell inside the corridor gets normal evaluation (accepted
   if otherwise safe).
4. real Stage 36 mask: START/GOAL fine cells are inside the corridor.
"""
import numpy as np
from affine import Affine

from planner.astar import BUCKET_SHORT, _generate_neighbors, msl_to_z_index
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set
from planner.roi import ROIData
from planner.terrain import TerrainQuery

NODATA = -9999.0
MASK_NPY = "outputs/stage36_corridor_mask.npy"
START_ROW, START_COL = 48, 276
GOAL_ROW, GOAL_COL = 264, 276


def make_synthetic_roi(elevation: np.ndarray, res: float = 30.0) -> ROIData:
    height, width = elevation.shape
    transform = Affine(res, 0.0, 0.0, 0.0, -res, height * res)
    return ROIData(
        elevation=elevation.astype(np.float32), transform=transform, crs="EPSG:32636",
        width=width, height=height, bounds=(0.0, 0.0, width * res, height * res),
        resolution=(res, res), nodata=NODATA,
    )


def test_1_none_equals_all_true(cfg, primitives) -> bool:
    print("=== 1: corridor_mask=None == an all-True mask (identical behavior) ===")
    elev = np.full((10, 10), 1000.0)
    tq = TerrainQuery(make_synthetic_roi(elev))
    cache, stats = {}, {"hits": 0, "misses": 0, "actual_calls": 0}
    z0 = msl_to_z_index(1300.0, cfg)
    state = (5, 5, z0, 0, BUCKET_SHORT)

    n_none, rej_none, gen_none, rejc_none, corridor_none, zcorr_none = _generate_neighbors(
        state, primitives, tq, cfg, 1100.0, 1500.0, cache, stats, None, None,
    )
    all_true_mask = np.ones((10, 10), dtype=bool)
    cache2, stats2 = {}, {"hits": 0, "misses": 0, "actual_calls": 0}
    n_true, rej_true, gen_true, rejc_true, corridor_true, zcorr_true = _generate_neighbors(
        state, primitives, tq, cfg, 1100.0, 1500.0, cache2, stats2, None, all_true_mask,
    )
    ok = (n_none == n_true and rej_none == rej_true and gen_none == gen_true
          and rejc_none == rejc_true and corridor_none == corridor_true == 0)
    print(f"  None: accepted={len(n_none)} rejected={rejc_none} corridor_rejects={corridor_none}")
    print(f"  all-True: accepted={len(n_true)} rejected={rejc_true} corridor_rejects={corridor_true}")
    print(f"  {'PASS' if ok else 'FAIL'}")
    return ok


def test_2_outside_rejected(cfg, primitives) -> bool:
    print()
    print("=== 2: successor outside corridor -> rejected (outside_corridor) ===")
    elev = np.full((10, 10), 1000.0)
    tq = TerrainQuery(make_synthetic_roi(elev))
    cache, stats = {}, {"hits": 0, "misses": 0, "actual_calls": 0}
    z0 = msl_to_z_index(1300.0, cfg)
    state = (5, 5, z0, 0, BUCKET_SHORT)

    mask = np.zeros((10, 10), dtype=bool)
    mask[5, 5] = True  # only the current cell itself is "in corridor" -- every successor is outside

    accepted, rej_counts, gen, rejected, corridor_rej, zcorr_rej = _generate_neighbors(
        state, primitives, tq, cfg, 1100.0, 1500.0, cache, stats, None, mask,
    )
    ok = len(accepted) == 0 and corridor_rej == gen and rej_counts.get("outside_corridor", 0) == gen
    print(f"  generated={gen} accepted={len(accepted)} corridor_rejects={corridor_rej} "
          f"rejected_reason_counts={rej_counts}")
    print(f"  {'PASS' if ok else 'FAIL'}")
    return ok


def test_3_inside_normal_eval(cfg, primitives) -> bool:
    print()
    print("=== 3: successor inside corridor -> normal evaluation ===")
    elev = np.full((10, 10), 1000.0)
    tq = TerrainQuery(make_synthetic_roi(elev))
    cache, stats = {}, {"hits": 0, "misses": 0, "actual_calls": 0}
    z0 = msl_to_z_index(1300.0, cfg)
    state = (5, 5, z0, 0, BUCKET_SHORT)

    mask = np.ones((10, 10), dtype=bool)  # everything in-corridor -- flat safe terrain, all should pass normally
    accepted, rej_counts, gen, rejected, corridor_rej, zcorr_rej = _generate_neighbors(
        state, primitives, tq, cfg, 1100.0, 1500.0, cache, stats, None, mask,
    )
    ok = corridor_rej == 0 and len(accepted) > 0 and stats["actual_calls"] > 0
    print(f"  generated={gen} accepted={len(accepted)} corridor_rejects={corridor_rej} "
          f"actual_evaluate_primitive_calls={stats['actual_calls']}")
    print(f"  {'PASS' if ok else 'FAIL'}")
    return ok


def test_4_real_start_goal_inside() -> bool:
    print()
    print("=== 4: real Stage 36 corridor mask -- START/GOAL inside ===")
    mask = np.load(MASK_NPY)
    start_in = bool(mask[START_ROW, START_COL])
    goal_in = bool(mask[GOAL_ROW, GOAL_COL])
    ok = start_in and goal_in
    print(f"  mask.shape={mask.shape}  START in corridor: {start_in}  GOAL in corridor: {goal_in}  "
          f"{'PASS' if ok else 'FAIL'}")
    return ok


def main() -> None:
    cfg = DEFAULT_CONFIG
    primitives = build_primitive_set(cfg)
    results = [
        test_1_none_equals_all_true(cfg, primitives),
        test_2_outside_rejected(cfg, primitives),
        test_3_inside_normal_eval(cfg, primitives),
        test_4_real_start_goal_inside(),
    ]
    print()
    print(f"Overall: {'ALL PASS' if all(results) else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
