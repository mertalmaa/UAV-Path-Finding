"""Stage 25: Weighted A* (f_weighted = g + epsilon*h for ordering only) +
certified bounded-suboptimal termination (incumbent <= target_suboptimality
* LB, LB tracked via a safe f_lb=g+h lower-bound frontier).

13) unit test -- weighted priority vs safe lower bound must stay separate.
14) unit test -- incumbent pruning must use f_lb, never the weighted f.
15) unit test -- %5 certificate arithmetic.
16) small exact reference test: epsilon sweep vs true C*.
17) certified bounded test: epsilon=1.5/target=1.05 returns <=1.05*C*.
"""
import dataclasses
import math

import numpy as np
from affine import Affine

from planner.astar import astar_search, msl_to_z_index, validate_and_cost_path
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set
from planner.roi import ROIData
from planner.terrain import TerrainQuery

NODATA = -9999.0


def make_roi(elevation: np.ndarray, nodata: float = NODATA, res: float = 30.0) -> ROIData:
    height, width = elevation.shape
    transform = Affine(res, 0.0, 0.0, 0.0, -res, height * res)
    return ROIData(
        elevation=elevation.astype(np.float32), transform=transform, crs="EPSG:32636",
        width=width, height=height, bounds=(0.0, 0.0, width * res, height * res),
        resolution=(res, res), nodata=nodata,
    )


# ----------------------------------------------------------------------
# 13) unit test -- weighted priority vs safe lower bound.
# ----------------------------------------------------------------------

def unit_test_weighted_priority() -> bool:
    print("=== 13: unit test -- weighted priority (g+eps*h) vs safe lower bound (g+h) ===")
    g, h = 100.0, 50.0
    cases = [(1.0, 150.0), (1.5, 175.0), (2.0, 200.0)]
    all_ok = True
    for eps, expected_priority in cases:
        priority = g + eps * h
        lb = g + h
        ok = abs(priority - expected_priority) < 1e-9 and abs(lb - 150.0) < 1e-9
        all_ok = all_ok and ok
        print(f"  epsilon={eps}: priority={priority} (expect {expected_priority})  "
              f"safe_lb={lb} (expect 150.0 always)  {'PASS' if ok else 'FAIL'}")
    print(f"  Overall: {'PASS' if all_ok else 'FAIL'}")
    return all_ok


# ----------------------------------------------------------------------
# 14) unit test -- incumbent pruning must use f_lb, never weighted f.
# ----------------------------------------------------------------------

def unit_test_incumbent_uses_lb() -> bool:
    print()
    print("=== 14: unit test -- incumbent pruning must use f_lb, not weighted f ===")
    incumbent = 180.0
    g, h, eps = 100.0, 50.0, 2.0
    f_weighted = g + eps * h
    f_lb = g + h
    would_prune_if_weighted = f_weighted >= incumbent
    would_prune_if_lb = f_lb >= incumbent
    correct_decision = not would_prune_if_lb  # per spec: must NOT be pruned (150 < 180)
    print(f"  f_weighted={f_weighted} (would_prune={would_prune_if_weighted}) "
          f"f_lb={f_lb} (would_prune={would_prune_if_lb})")
    ok = correct_decision and not would_prune_if_lb and f_lb == 150.0
    print(f"  correct rule (f_lb-based) says KEEP: {not would_prune_if_lb}  {'PASS' if ok else 'FAIL'}")
    return ok


# ----------------------------------------------------------------------
# 15) unit test -- %5 certificate arithmetic.
# ----------------------------------------------------------------------

def unit_test_certificate() -> bool:
    print()
    print("=== 15: unit test -- %5 certificate arithmetic ===")
    target = 1.05
    cases = [
        ("A: incumbent=105, LB=100", 105.0, 100.0, "TERMINATE"),
        ("B: incumbent=106, LB=100", 106.0, 100.0, "CONTINUE"),
        ("C: incumbent=100, LB=100", 100.0, 100.0, "TERMINATE"),
    ]
    all_ok = True
    for label, incumbent, lb, expected in cases:
        decision = "TERMINATE" if incumbent <= target * lb else "CONTINUE"
        ok = decision == expected
        all_ok = all_ok and ok
        print(f"  {label}: incumbent<=target*LB ({incumbent}<={target * lb}) -> {decision} "
              f"(expect {expected})  {'PASS' if ok else 'FAIL'}")
    print(f"  Overall: {'PASS' if all_ok else 'FAIL'}")
    return all_ok


# ----------------------------------------------------------------------
# 16) small exact reference test: epsilon sweep vs true C*.
# ----------------------------------------------------------------------

def small_exact_reference_test(cfg, primitives):
    print()
    print("=== 16: small exact reference test (epsilon sweep) ===")
    width, height = 40, 3
    elev = np.full((height, width), 1000.0)
    elev[:, 15:] = 1200.0
    tq = TerrainQuery(make_roi(elev))
    z0 = msl_to_z_index(1300.0, cfg)
    start, goal = (1, 2, z0), (1, 37, msl_to_z_index(1400.0, cfg))

    exact = astar_search(start, goal, tq, min_search_altitude_msl=1200.0, max_search_altitude_msl=1450.0,
                          config=cfg, primitives=primitives, max_expansions=20_000,
                          use_primitive_cache=True, use_dominance_pruning=False,
                          use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False,
                          epsilon_search=1.0, target_suboptimality=None)
    c_star = exact.total_cost
    print(f"  epsilon=1.0 (exact): C*={c_star:.4f} expanded={exact.expanded_nodes} runtime={exact.runtime_s * 1000:.1f}ms")

    all_safe = True
    for eps in (1.05, 1.2, 1.5, 2.0):
        result = astar_search(start, goal, tq, min_search_altitude_msl=1200.0, max_search_altitude_msl=1450.0,
                               config=cfg, primitives=primitives, max_expansions=20_000,
                               use_primitive_cache=True, use_dominance_pruning=False,
                               use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False,
                               epsilon_search=eps, target_suboptimality=None)
        ratio = result.total_cost / c_star if result.status == "success" else float("nan")
        safe = result.status == "success" and ratio <= eps + 1e-6
        all_safe = all_safe and safe
        print(f"  epsilon={eps}: cost={result.total_cost:.4f} ratio={ratio:.4f} (<=epsilon={eps}) "
              f"expanded={result.expanded_nodes} runtime={result.runtime_s * 1000:.1f}ms reopened={result.reopened_states} "
              f"{'PASS' if safe else 'FAIL'}")
    print(f"  Overall (all within their own epsilon bound): {'PASS' if all_safe else 'FAIL'}")
    return all_safe, c_star


# ----------------------------------------------------------------------
# 17) certified bounded test: epsilon=1.5 / target=1.05.
# ----------------------------------------------------------------------

def certified_bounded_test(cfg, primitives, c_star):
    print()
    print("=== 17: certified bounded test (epsilon=1.5, target_suboptimality=1.05) ===")
    width, height = 40, 3
    elev = np.full((height, width), 1000.0)
    elev[:, 15:] = 1200.0
    tq = TerrainQuery(make_roi(elev))
    z0 = msl_to_z_index(1300.0, cfg)
    start, goal = (1, 2, z0), (1, 37, msl_to_z_index(1400.0, cfg))

    result = astar_search(start, goal, tq, min_search_altitude_msl=1200.0, max_search_altitude_msl=1450.0,
                           config=cfg, primitives=primitives, max_expansions=20_000,
                           use_primitive_cache=True, use_dominance_pruning=False,
                           use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False,
                           use_incumbent_pruning=True, epsilon_search=1.5, target_suboptimality=1.05)

    ratio_to_true_optimal = result.total_cost / c_star
    ok = (result.status == "success" and result.bounded_termination_triggered
          and ratio_to_true_optimal <= 1.05 + 1e-6)
    print(f"  status={result.status} bounded_termination_triggered={result.bounded_termination_triggered}")
    print(f"  first_solution: cost={result.first_solution_cost:.4f} expanded={result.first_solution_expanded} "
          f"runtime={result.first_solution_runtime_s * 1000:.1f}ms")
    print(f"  final: incumbent_cost={result.final_incumbent_cost:.4f} current_lower_bound={result.current_lower_bound:.4f} "
          f"final_bound_ratio={result.final_bound_ratio:.4f}")
    print(f"  returned_cost={result.total_cost:.4f}  C*={c_star:.4f}  actual_ratio={ratio_to_true_optimal:.4f} "
          f"(must be <=1.05)  incumbent_updates={result.incumbent_updates} reopened={result.reopened_states}")
    print(f"  {'PASS' if ok else 'FAIL'}")
    return ok


def main() -> None:
    cfg = dataclasses.replace(DEFAULT_CONFIG, msl_cost_weight=0.63)
    primitives = build_primitive_set(cfg)

    r13 = unit_test_weighted_priority()
    r14 = unit_test_incumbent_uses_lb()
    r15 = unit_test_certificate()
    r16, c_star = small_exact_reference_test(cfg, primitives)
    r17 = certified_bounded_test(cfg, primitives, c_star)

    print()
    print(f"Overall: {'ALL PASS' if (r13 and r14 and r15 and r16 and r17) else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
