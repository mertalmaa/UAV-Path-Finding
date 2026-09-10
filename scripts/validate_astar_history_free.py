"""Stage 37.2: small integration tests for astar_search's new diagnostic-
only freeze_history mode (trend/bucket frozen, reversal cost forced to
0.0). NOT a production default -- False (the default) reproduces the
pre-Stage-37.2 search exactly, confirmed by the full regression suite
already re-run for Stage 24/25/37/37.1 after this change. No real
benchmark here -- see scripts/benchmark_history_free_fine_real.py.
"""
import dataclasses

import numpy as np
from affine import Affine

from planner.astar import astar_search, msl_to_z_index
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set
from planner.roi import ROIData
from planner.terrain import TerrainQuery

NODATA = -9999.0


def make_synthetic_roi(elevation: np.ndarray, nodata=NODATA, res: float = 30.0) -> ROIData:
    height, width = elevation.shape
    transform = Affine(res, 0.0, 0.0, 0.0, -res, height * res)
    return ROIData(
        elevation=elevation.astype(np.float32), transform=transform, crs="EPSG:32636",
        width=width, height=height, bounds=(0.0, 0.0, width * res, height * res),
        resolution=(res, res), nodata=nodata,
    )


def test_1_state_is_xyz(cfg, primitives) -> bool:
    print("=== 1: state effectively (row,col,z_index) -- unique_full_states == unique_expanded_xyz ===")
    elev = np.full((15, 20), 1000.0)
    elev[:, 10:] = 1200.0
    tq = TerrainQuery(make_synthetic_roi(elev))
    z0 = msl_to_z_index(1300.0, cfg)
    start, goal = (7, 2, z0), (7, 18, z0)

    result = astar_search(start, goal, tq, 1100.0, 1500.0, config=cfg, primitives=primitives,
                           max_expansions=5000, use_primitive_cache=True, use_dominance_pruning=False,
                           use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False,
                           freeze_history=True)
    ok = result.unique_full_states == result.unique_expanded_xyz and result.avg_history_states_per_xyz == 1.0
    print(f"  unique_full_states={result.unique_full_states}  unique_expanded_xyz={result.unique_expanded_xyz}  "
          f"avg_history_states_per_xyz={result.avg_history_states_per_xyz}  {'PASS' if ok else 'FAIL'}")
    return ok


def test_2_no_xyz_duplication(cfg, primitives) -> bool:
    print()
    print("=== 2: same XYZ never revisited under a different (frozen) history ===")
    # A terrain shaped so reaching a given cell via "climb then level" vs "level then
    # climb" would normally carry different (trend,bucket) -- with history frozen,
    # only ONE augmented copy of any given (row,col,z_index) can ever exist.
    elev = np.full((10, 30), 1000.0)
    tq = TerrainQuery(make_synthetic_roi(elev))
    z0 = msl_to_z_index(1300.0, cfg)
    start, goal = (5, 2, z0), (5, 26, z0)

    result_frozen = astar_search(start, goal, tq, 1200.0, 1500.0, config=cfg, primitives=primitives,
                                  max_expansions=5000, use_primitive_cache=True, use_dominance_pruning=False,
                                  use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False,
                                  freeze_history=True)
    result_normal = astar_search(start, goal, tq, 1200.0, 1500.0, config=cfg, primitives=primitives,
                                  max_expansions=5000, use_primitive_cache=True, use_dominance_pruning=False,
                                  use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False,
                                  freeze_history=False)
    ok = (result_frozen.unique_full_states == result_frozen.unique_expanded_xyz
          and result_normal.avg_history_states_per_xyz >= 1.0)
    print(f"  frozen: unique_full_states={result_frozen.unique_full_states} "
          f"unique_expanded_xyz={result_frozen.unique_expanded_xyz} (expect equal)")
    print(f"  normal: unique_full_states={result_normal.unique_full_states} "
          f"unique_expanded_xyz={result_normal.unique_expanded_xyz} "
          f"avg_history_states_per_xyz={result_normal.avg_history_states_per_xyz:.3f}")
    print(f"  {'PASS' if ok else 'FAIL'}")
    return ok


def test_3_safety_unchanged(cfg, primitives) -> bool:
    print()
    print("=== 3: safety behavior (AGL/NoData) unaffected by freeze_history ===")
    elev = np.full((6, 6), 1000.0)
    elev[3, 3] = NODATA
    tq = TerrainQuery(make_synthetic_roi(elev, nodata=NODATA))
    z0 = msl_to_z_index(1200.0, cfg)
    start, goal = (3, 0, z0), (3, 5, z0)

    r_frozen = astar_search(start, goal, tq, 1100.0, 1400.0, config=cfg, primitives=primitives,
                             max_expansions=2000, use_primitive_cache=True, use_dominance_pruning=False,
                             use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False,
                             freeze_history=True)
    r_normal = astar_search(start, goal, tq, 1100.0, 1400.0, config=cfg, primitives=primitives,
                             max_expansions=2000, use_primitive_cache=True, use_dominance_pruning=False,
                             use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False,
                             freeze_history=False)
    ok = (r_frozen.rejected_reason_counts.get("nodata", 0) > 0
          and r_normal.rejected_reason_counts.get("nodata", 0) > 0
          and r_frozen.status == r_normal.status)
    print(f"  frozen: status={r_frozen.status} nodata_rejects={r_frozen.rejected_reason_counts.get('nodata', 0)}")
    print(f"  normal: status={r_normal.status} nodata_rejects={r_normal.rejected_reason_counts.get('nodata', 0)}")
    print(f"  {'PASS' if ok else 'FAIL'}")
    return ok


def test_4_default_off_unchanged(cfg, primitives) -> bool:
    print()
    print("=== 4: freeze_history=False (default) reproduces normal planner behavior ===")
    elev = np.full((10, 10), 1000.0)
    elev[:, 5:] = 1200.0
    tq = TerrainQuery(make_synthetic_roi(elev))
    z0 = msl_to_z_index(1300.0, cfg)
    start, goal = (5, 1, z0), (5, 8, z0)

    a = astar_search(start, goal, tq, 1200.0, 1500.0, config=cfg, primitives=primitives, max_expansions=20_000,
                      use_primitive_cache=True, use_dominance_pruning=False, use_msl_lower_bound_heuristic=True,
                      use_vertical_reachability_heuristic=False)
    b = astar_search(start, goal, tq, 1200.0, 1500.0, config=cfg, primitives=primitives, max_expansions=20_000,
                      use_primitive_cache=True, use_dominance_pruning=False, use_msl_lower_bound_heuristic=True,
                      use_vertical_reachability_heuristic=False, freeze_history=False)
    import math
    same_cost = a.total_cost == b.total_cost or (math.isnan(a.total_cost) and math.isnan(b.total_cost))
    ok = a.expanded_nodes == b.expanded_nodes and same_cost and a.status == b.status
    print(f"  no freeze_history arg: expanded={a.expanded_nodes} cost={a.total_cost}")
    print(f"  freeze_history=False:  expanded={b.expanded_nodes} cost={b.total_cost}")
    print(f"  {'PASS' if ok else 'FAIL'}")
    return ok


def main() -> None:
    cfg = dataclasses.replace(DEFAULT_CONFIG, msl_cost_weight=0.63)
    primitives = build_primitive_set(cfg)
    results = [
        test_1_state_is_xyz(cfg, primitives),
        test_2_no_xyz_duplication(cfg, primitives),
        test_3_safety_unchanged(cfg, primitives),
        test_4_default_off_unchanged(cfg, primitives),
    ]
    print()
    print(f"Overall: {'ALL PASS' if all(results) else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
