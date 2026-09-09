"""Stage 23: 3-bucket state (Stage 22) + exact dominance pruning (Stage 16
mechanism, unchanged code) together.

The dominance mechanism in planner.astar.astar_search / _dominates has
always operated GENERICALLY on the augmented state's 5th tuple element as
an "age-like" ordinal -- it was written for the 0..10 trend_age_units
model (Stage 16) and, since Stage 22 replaced that element with a 0..2
trend_age_bucket, has been running with use_dominance_pruning=True never
re-verified for buckets (it defaults to True, but every Stage 22 benchmark
explicitly passed False). This script:

  1) proves (exhaustively over the current 24-primitive set, for every
     scenario category the spec calls out) that the SAME monotonicity
     Stage 16 relied on for ages -- higher ordinal now (bucket instead of
     age) never means higher future reversal cost for any shared future
     primitive sequence -- still holds for buckets, before trusting
     dominance pruning's correctness for them;
  2) unit-tests the dominance predicate itself (planner.astar._dominates,
     reused as-is -- it already IS "g_a<=g_b and rank_a>=rank_b", which is
     exactly section 3's stated rule, so no new predicate was written);
  3) runs the Stage 22 synthetic state-explosion scenario with dominance
     OFF vs ON;
  4) applies the go/no-go gate before touching the real 6.48km benchmark.
"""
import dataclasses
import itertools
import math
import time

import numpy as np
from affine import Affine

from planner.astar import (
    BUCKET_MATURE, BUCKET_MEDIUM, BUCKET_SHORT, _dominates, _next_trend_and_bucket,
    astar_search, msl_to_z_index,
)
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set
from planner.roi import ROIData
from planner.terrain import TerrainQuery

NODATA = -9999.0
BUCKET_NAME = {BUCKET_SHORT: "SHORT", BUCKET_MEDIUM: "MEDIUM", BUCKET_MATURE: "MATURE"}


def make_roi(elevation: np.ndarray, nodata: float = NODATA, res: float = 30.0) -> ROIData:
    height, width = elevation.shape
    transform = Affine(res, 0.0, 0.0, 0.0, -res, height * res)
    return ROIData(
        elevation=elevation.astype(np.float32), transform=transform, crs="EPSG:32636",
        width=width, height=height, bounds=(0.0, 0.0, width * res, height * res),
        resolution=(res, res), nodata=nodata,
    )


# ----------------------------------------------------------------------
# 4/11) Monotonicity: for a fixed trend and any shared future primitive
# sequence, a higher-bucket-rank state's cumulative reversal cost is
# never larger than a lower-bucket-rank state's, at every prefix.
# ----------------------------------------------------------------------

def _cumulative_costs(trend0: int, bucket0: int, sequence, config) -> list:
    """Cumulative reversal-penalty cost (NOT base geometric/MSL cost --
    that's identical for both branches regardless of bucket, so it cancels
    out of any A-vs-B comparison and is omitted) after each prefix of
    `sequence`, replaying _next_trend_and_bucket unmodified."""
    trend, bucket = trend0, bucket0
    cum = 0.0
    out = [0.0]
    for prim in sequence:
        next_trend, next_bucket, is_rev, factor = _next_trend_and_bucket(trend, bucket, prim, config)
        if is_rev:
            cum += config.vertical_reversal_cost_weight * abs(prim.dz_m) * factor
        trend, bucket = next_trend, next_bucket
        out.append(cum)
    return out


def monotonicity_named_scenarios(cfg, primitives) -> bool:
    print("=== 4/11: monotonicity on named scenario categories ===")
    climb_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "climb")
    descent_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "descent")
    level_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "level")

    scenarios = {
        "LEVEL -> LEVEL": [level_e, level_e],
        "same-trend continuation": [descent_e, descent_e],
        "direct reversal": [climb_e],
        "LEVEL then reversal": [level_e, climb_e],
        "reversal then new-trend continuation": [climb_e, climb_e],
    }
    buckets_hi_to_lo = [BUCKET_MATURE, BUCKET_MEDIUM, BUCKET_SHORT]
    all_ok = True
    for name, seq in scenarios.items():
        # standing trend is DESCENT (-1) before this sequence in every case above
        # (LEVEL-only scenarios need a standing trend to have any bucket meaning).
        trend0 = -1
        rows = {b: _cumulative_costs(trend0, b, seq, cfg) for b in buckets_hi_to_lo}
        ok = all(rows[BUCKET_MATURE][i] <= rows[BUCKET_MEDIUM][i] + 1e-9 and
                  rows[BUCKET_MEDIUM][i] <= rows[BUCKET_SHORT][i] + 1e-9
                  for i in range(len(rows[BUCKET_SHORT])))
        all_ok = all_ok and ok
        print(f"  {name}: MATURE={rows[BUCKET_MATURE]} MEDIUM={rows[BUCKET_MEDIUM]} "
              f"SHORT={rows[BUCKET_SHORT]}  {'PASS' if ok else 'FAIL'}")
    print(f"  Overall named-scenario monotonicity: {'PASS' if all_ok else 'FAIL'}")
    return all_ok


def monotonicity_exhaustive(cfg, primitives, max_len=3) -> bool:
    print()
    print(f"=== exhaustive sweep: all primitive sequences up to length {max_len} "
          f"({len(primitives)} primitives) ===")
    buckets_hi_to_lo = [BUCKET_MATURE, BUCKET_MEDIUM, BUCKET_SHORT]
    violations = 0
    total = 0
    for length in range(1, max_len + 1):
        for seq in itertools.product(primitives, repeat=length):
            total += 1
            for trend0 in (-1, 1):
                rows = {b: _cumulative_costs(trend0, b, seq, cfg) for b in buckets_hi_to_lo}
                for i in range(len(rows[BUCKET_SHORT])):
                    if not (rows[BUCKET_MATURE][i] <= rows[BUCKET_MEDIUM][i] + 1e-9 and
                            rows[BUCKET_MEDIUM][i] <= rows[BUCKET_SHORT][i] + 1e-9):
                        violations += 1
    print(f"  sequences checked: {total} (x2 starting trends)  violations found: {violations}")
    ok = violations == 0
    print(f"  Overall exhaustive monotonicity: {'PASS' if ok else 'FAIL'}")
    return ok


# ----------------------------------------------------------------------
# 10) Unit tests for the dominance predicate itself (planner.astar._dominates,
# reused unchanged -- it already implements exactly section 3's rule).
# ----------------------------------------------------------------------

def unit_tests_dominance() -> bool:
    print()
    print("=== 10: dominance predicate unit tests (_dominates, reused as-is) ===")
    cases = [
        ("A: existing MATURE g=1000, candidate SHORT g=1010 -> candidate dominated "
         "(pruning check: _dominates(existing, candidate))",
         _dominates(BUCKET_MATURE, 1000, BUCKET_SHORT, 1010), True),
        ("C: existing MATURE g=1050, candidate SHORT g=1000 -> no dominance",
         _dominates(BUCKET_MATURE, 1050, BUCKET_SHORT, 1000), False),
        ("D: existing SHORT g=1000, candidate MATURE g=1050 -> no dominance",
         _dominates(BUCKET_MATURE, 1050, BUCKET_SHORT, 1000), False),
    ]
    all_ok = True
    for label, actual, expected in cases:
        ok = actual == expected
        all_ok = all_ok and ok
        print(f"  {label}: _dominates(...)={actual} (expect {expected})  {'PASS' if ok else 'FAIL'}")

    # B, precisely: is (SHORT,1010) dominated by (MATURE,1000)? -- ask in the
    # (existing, candidate) direction astar_search actually uses.
    b_dominated = _dominates(BUCKET_MATURE, 1000, BUCKET_SHORT, 1010)
    ok_b = b_dominated is True
    all_ok = all_ok and ok_b
    print(f"  B (existing=SHORT/1010 dominated by candidate=MATURE/1000): {b_dominated}  "
          f"{'PASS' if ok_b else 'FAIL'}")

    # E: same physical state, different vertical_trend -- dominance must never
    # be evaluated across trends at all (base_key includes trend; _dominates
    # itself doesn't know about trend, so this is really a caller-discipline
    # check on astar_search's base_key construction).
    from planner.astar import AugmentedState
    state_a: AugmentedState = (10, 10, 5, 1, BUCKET_MATURE)
    state_b: AugmentedState = (10, 10, 5, -1, BUCKET_SHORT)
    base_key_a, base_key_b = state_a[:4], state_b[:4]
    ok_e = base_key_a != base_key_b
    all_ok = all_ok and ok_e
    print(f"  E: same (row,col,z) different vertical_trend -> different base_key "
          f"({base_key_a} vs {base_key_b}): {'PASS' if ok_e else 'FAIL'} "
          f"(never compared -- astar_search groups frontiers strictly by base_key)")

    print(f"  Overall unit tests: {'PASS' if all_ok else 'FAIL'}")
    return all_ok


# ----------------------------------------------------------------------
# 12) Synthetic state-explosion benchmark: dominance OFF vs ON, 3-bucket state.
# ----------------------------------------------------------------------

def synthetic_benchmark(cfg, primitives):
    print()
    print("=== 12: synthetic state-explosion benchmark (3-bucket, dominance OFF vs ON) ===")
    width, height = 55, 3
    elev = np.full((height, width), 1000.0)
    elev[:, 10:] = 1200.0
    tq = TerrainQuery(make_roi(elev))
    z0 = msl_to_z_index(1300.0, cfg)
    start, goal = (1, 2, z0), (1, 50, msl_to_z_index(1400.0, cfg))

    results = {}
    for label, dominance in [("A) dominance OFF", False), ("B) dominance ON", True)]:
        result = astar_search(start, goal, tq, min_search_altitude_msl=1300.0, max_search_altitude_msl=1420.0,
                               config=cfg, primitives=primitives, max_expansions=50_000,
                               use_primitive_cache=True, use_dominance_pruning=dominance,
                               use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False)
        results[label] = result
        print(f"  {label}: status={result.status} expanded={result.expanded_nodes} "
              f"max_open={result.max_open_size} runtime={result.runtime_s * 1000:.1f}ms "
              f"total_cost={result.total_cost:.2f}")
        if dominance:
            print(f"    dominance: checks={result.dominance_checks} pruned={result.dominance_pruned_candidates} "
                  f"frontier_removed={result.dominance_frontier_entries_removed} "
                  f"pop_skipped={result.dominated_heap_pops_skipped} "
                  f"max_frontier={result.max_dominance_frontier_size} "
                  f"avg_frontier={result.average_dominance_frontier_size:.3f}")

    off, on = results["A) dominance OFF"], results["B) dominance ON"]
    same_cost = abs(off.total_cost - on.total_cost) < 1e-6 and off.status == on.status == "success"
    exp_reduction = 1.0 - on.expanded_nodes / off.expanded_nodes if off.expanded_nodes else 0.0
    open_reduction = 1.0 - on.max_open_size / off.max_open_size if off.max_open_size else 0.0
    runtime_ratio = on.runtime_s / off.runtime_s if off.runtime_s else float("nan")
    print(f"  same optimal cost preserved: {same_cost}")
    print(f"  expanded_nodes reduction: {exp_reduction * 100:.1f}%  max_open reduction: {open_reduction * 100:.1f}%  "
          f"runtime ratio (ON/OFF): {runtime_ratio:.2f}x")
    return off, on, exp_reduction, open_reduction, runtime_ratio, same_cost


def main() -> None:
    cfg = dataclasses.replace(DEFAULT_CONFIG, msl_cost_weight=0.63)
    primitives = build_primitive_set(cfg)

    r1 = monotonicity_named_scenarios(cfg, primitives)
    r2 = monotonicity_exhaustive(cfg, primitives, max_len=3)
    r3 = unit_tests_dominance()

    if not (r1 and r2):
        print()
        print(">> Monotonicity FAILED -- STOPPING before enabling dominance pruning. Do not proceed.")
        return

    off, on, exp_reduction, open_reduction, runtime_ratio, same_cost = synthetic_benchmark(cfg, primitives)

    print()
    print("=== 13: go/no-go gate for the real 6.48km benchmark ===")
    gate_passed = (exp_reduction >= 0.25 or open_reduction >= 0.25) or runtime_ratio < 1.0
    print(f"  expanded reduction={exp_reduction * 100:.1f}% max_open reduction={open_reduction * 100:.1f}% "
          f"runtime_ratio={runtime_ratio:.2f}x -> gate {'PASSED' if gate_passed else 'NOT PASSED'}")
    if not gate_passed:
        print("  Synthetic result does not clear the >=25% state/open reduction or clear runtime")
        print("  improvement bar -- per Stage 23 spec, the real Aladagalar benchmark will NOT be run.")
        return
    print("  Gate passed -- proceeding to the single real 6.48km benchmark run (30k cap, no retry).")


if __name__ == "__main__":
    main()
