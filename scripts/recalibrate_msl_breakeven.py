"""Stage 14, items 15-18: re-run Stage 13's real-Aladaglar cost decomposition
with the NEW spacing-sensitive reversal cost instead of the old flat
R=vertical_reversal_cost_weight*20 charge. Same scenario, same candidate
template (LEVEL^A DESCENT^k LEVEL^B CLIMB^k LEVEL^C, k=0..4), same
enumeration -- only the R term changes, via the existing
_path_vertical_reversal_metrics()/_next_trend_and_age() machinery (no
reimplementation). No new weight-tuning decision, no config change.

Then: at most 2 real A* runs (baseline weights, and one balanced
analytical candidate) with full performance reporting, per the explicit
"don't full-sweep the real ROI, state-space grew again" instruction.
"""
import dataclasses
import math
import time

from planner.astar import (
    _altitude_scaled, _path_altitude_metrics, _path_vertical_reversal_metrics,
    astar_search, compute_edge_cost, msl_to_z_index, z_index_to_msl,
)
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set, evaluate_primitive
from planner.roi import load_roi
from planner.terrain import TerrainQuery

ROW = 80
COL_START = 90
COL_GOAL = 128
CRUISE_MSL = 2840.0
W_REPRESENTATIVE = 0.025
W_SWEEP = (0.0, 0.025, 0.05, 0.10)


def compute_M(sequence, start_altitude_msl, config):
    alt = start_altitude_msl
    M = 0.0
    for prim in sequence:
        end_alt = alt + prim.dz_m
        geometric_cost = math.sqrt(prim.horizontal_distance_m ** 2 + prim.dz_m ** 2)
        mean_alt = (alt + end_alt) / 2.0
        M += geometric_cost * _altitude_scaled(mean_alt, config)
        alt = end_alt
    return M


def production_total(sequence, start_altitude_msl, config):
    alt = start_altitude_msl
    trend, age = 0, 0
    total = 0.0
    from planner.astar import _next_trend_and_age
    for prim in sequence:
        total += compute_edge_cost(prim, alt, trend, age, config)
        trend, age, _, _ = _next_trend_and_age(trend, age, prim, config)
        alt += prim.dz_m
    return total


def evaluate_candidate(start, sequence, terrain, primitives, config):
    row, col, z_index = start
    physical_path = [(row, col, z_index)]
    min_agl = math.inf

    for prim in sequence:
        x, y = terrain.rowcol_to_xy(row, col)
        start_xyz = (x, y, z_index_to_msl(z_index, config))
        result = evaluate_primitive(start_xyz, prim, terrain, config)
        if not result.valid:
            return None
        min_agl = min(min_agl, result.min_agl_m)
        row += prim.drow
        col += prim.dcol
        z_index += round(prim.dz_m / config.z_step_m)
        physical_path.append((row, col, z_index))

    alt_metrics = _path_altitude_metrics(physical_path, terrain, config)
    rev_metrics = _path_vertical_reversal_metrics(physical_path, primitives, config)

    start_msl = z_index_to_msl(start[2], config)
    G = alt_metrics["geometric_path_length"]
    R = rev_metrics["total_reversal_penalty"]
    M = compute_M(sequence, start_msl, config)

    for w in (0.0, W_REPRESENTATIVE, 0.10):
        decomposed = G + w * M + R
        actual = production_total(sequence, start_msl, dataclasses.replace(config, msl_cost_weight=w))
        assert abs(decomposed - actual) < 1e-6, f"decomposition mismatch at w={w}: {decomposed} vs {actual}"

    return {
        "path": physical_path, "G": G, "M": M, "R": R,
        "reversal_count": rev_metrics["total_vertical_reversal_count"],
        "penalized_reversal_count": rev_metrics["penalized_reversal_count"],
        "avg_reversal_spacing_m": rev_metrics["average_reversal_spacing_m"],
        "geometric_length": G,
        "avg_msl": alt_metrics["average_aircraft_msl"],
        "min_msl": alt_metrics["minimum_aircraft_msl"],
        "max_msl": alt_metrics["maximum_aircraft_msl"],
        "min_agl": min_agl,
    }


def enumerate_k(k, level_e, descent_e, climb_e, start, terrain, primitives, config):
    total_cols = COL_GOAL - COL_START
    remaining = total_cols - 8 * k
    tried = 0
    invalid = 0
    best = None
    best_key = math.inf

    placements = [(remaining, 0, 0)] if k == 0 else \
        [(a, b, remaining - a - b) for a in range(remaining + 1) for b in range(remaining - a + 1)]

    for a, b, c in placements:
        seq = [level_e] * a + [descent_e] * k + [level_e] * b + [climb_e] * k + [level_e] * c
        tried += 1
        result = evaluate_candidate(start, seq, terrain, primitives, config)
        if result is None:
            invalid += 1
            continue
        cost_at_rep = result["G"] + W_REPRESENTATIVE * result["M"] + result["R"]
        if cost_at_rep < best_key:
            best_key = cost_at_rep
            best = {**result, "a": a, "b": b, "c": c, "k": k}

    return tried, invalid, best


def main() -> None:
    cfg = DEFAULT_CONFIG
    primitives = build_primitive_set(cfg)
    level_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "level")
    descent_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "descent")
    climb_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "climb")

    roi = load_roi(cfg)
    tq = TerrainQuery(roi)
    z0 = round(CRUISE_MSL / cfg.z_step_m)
    start = (ROW, COL_START, z0)

    print(f"=== Re-decomposition with spacing-sensitive reversal cost "
          f"(reversal_relax_distance_m={cfg.reversal_relax_distance_m}) ===")
    print(f"Same scenario as Stage 13: row={ROW}, col {COL_START}->{COL_GOAL}, cruise={CRUISE_MSL:.0f}m")
    print()

    t0 = time.perf_counter()

    print("=== k=0 baseline: fully level at 2840m ===")
    flat_result = evaluate_candidate(start, [level_e] * (COL_GOAL - COL_START), tq, primitives, cfg)
    assert flat_result is not None
    print(f"  G_flat={flat_result['G']:.3f}  M_flat={flat_result['M']:.3f}  R_flat={flat_result['R']:.3f}")
    print()

    per_k_best = {}
    for k in (1, 2, 3, 4):
        tried, invalid, best = enumerate_k(k, level_e, descent_e, climb_e, start, tq, primitives, cfg)
        per_k_best[k] = best
        print(f"=== k={k} ({k * 20}m descent) ===")
        print(f"  candidates tried={tried}  infeasible={invalid}  feasible={tried - invalid}")
        if best is None:
            print("  NO FEASIBLE CANDIDATE.")
            print()
            continue
        print(f"  best placement: A={best['a']} B={best['b']} C={best['c']}")
        print(f"  min_MSL={best['min_msl']:.1f}  avg_MSL={best['avg_msl']:.1f}  min_AGL={best['min_agl']:.1f}")
        print(f"  reversal_count={best['reversal_count']} penalized={best['penalized_reversal_count']} "
              f"avg_spacing={best['avg_reversal_spacing_m']}")
        print(f"  G={best['G']:.3f}  M={best['M']:.3f}  R={best['R']:.3f}  "
              f"(Stage 13 had R=20.000 flat, regardless of spacing)")
        print()

    elapsed = time.perf_counter() - t0
    print(f"Enumeration runtime: {elapsed:.2f}s")
    print()

    print("=== New break-even w_MSL (spacing-sensitive R) ===")
    print(f"  flat: G={flat_result['G']:.3f} R={flat_result['R']:.3f} M={flat_result['M']:.3f}")
    breakevens = {}
    for k, best in per_k_best.items():
        if best is None:
            continue
        denom = flat_result["M"] - best["M"]
        num = (best["G"] + best["R"]) - (flat_result["G"] + flat_result["R"])
        if denom <= 0:
            print(f"  k={k}: NOT MSL-advantageous (denom={denom:.3f})")
            breakevens[k] = None
        else:
            w_be = num / denom
            breakevens[k] = w_be
            print(f"  k={k}: extra(G+R)={num:.3f}  M_gap={denom:.3f}  break-even w_MSL = {w_be:.4f}  "
                  f"(Stage 13 old value was ~1.5-2.1)")
    print()

    print(f"=== TOTAL(w) comparison, w in {W_SWEEP} ===")
    header = f"{'w_MSL':>8}{'flat':>12}" + "".join(f"{'k=' + str(k):>12}" for k in (1, 2, 3, 4))
    print(header)
    for w in W_SWEEP:
        row_vals = [f"{flat_result['G'] + w * flat_result['M'] + flat_result['R']:>12.2f}"]
        for k in (1, 2, 3, 4):
            best = per_k_best[k]
            row_vals.append(f"{'infeasible':>12}" if best is None else
                             f"{best['G'] + w * best['M'] + best['R']:>12.2f}")
        print(f"{w:>8}" + "".join(row_vals))
    print()

    # --- Analytical candidates (report only, no config change) ---
    print("=== Analytical prototype candidates (report only, config default NOT changed) ===")
    valid_be = [w for w in breakevens.values() if w is not None]
    if valid_be:
        lo, hi = min(valid_be), max(valid_be)
        print(f"  break-even range across feasible k: [{lo:.4f}, {hi:.4f}]")
        print(f"  conservative candidate: w_MSL ~ {hi * 1.2:.3f} (clears every feasible k's break-even with margin)")
        print(f"  balanced candidate:     w_MSL ~ {(lo + hi) / 2 * 1.1:.3f}")
        print(f"  aggressive candidate:   w_MSL ~ {lo * 1.05:.3f} (only just clears the easiest k)")
    else:
        print("  no k is MSL-advantageous at any positive weight -- no candidate to propose from this scenario alone")
    print()

    # --- Limited real A* comparison: baseline vs at most one balanced candidate ---
    print("=== Real A* runs: baseline weights vs 1 balanced candidate (max 2 runs) ===")
    min_search = math.ceil((float(roi.elevation[ROW, COL_START:COL_GOAL + 1].min()) + cfg.min_agl_m) / cfg.z_step_m) * cfg.z_step_m
    max_search = CRUISE_MSL + 20.0
    goal = (ROW, COL_GOAL, z0)

    t0 = time.perf_counter()
    r_baseline = astar_search(start, goal, tq, min_search_altitude_msl=min_search, max_search_altitude_msl=max_search,
                               config=cfg, primitives=primitives, max_expansions=30_000)
    dt_baseline = time.perf_counter() - t0
    print(f"  [baseline msl={cfg.msl_cost_weight}] status={r_baseline.status} wall={dt_baseline:.1f}s "
          f"expanded={r_baseline.expanded_nodes} max_open={r_baseline.max_open_size} "
          f"avg_MSL={r_baseline.average_aircraft_msl:.1f} reversals={r_baseline.total_vertical_reversal_count}")

    if dt_baseline > 60.0:
        print("  baseline run already took >60s -- skipping the second (balanced-candidate) run per instructions.")
    else:
        w_balanced = ((min(valid_be) + max(valid_be)) / 2 * 1.1) if valid_be else cfg.msl_cost_weight
        c_balanced = dataclasses.replace(cfg, msl_cost_weight=w_balanced)
        t0 = time.perf_counter()
        r_balanced = astar_search(start, goal, tq, min_search_altitude_msl=min_search, max_search_altitude_msl=max_search,
                                   config=c_balanced, primitives=primitives, max_expansions=30_000)
        dt_balanced = time.perf_counter() - t0
        print(f"  [balanced msl={w_balanced:.3f}] status={r_balanced.status} wall={dt_balanced:.1f}s "
              f"expanded={r_balanced.expanded_nodes} max_open={r_balanced.max_open_size} "
              f"avg_MSL={r_balanced.average_aircraft_msl:.1f} reversals={r_balanced.total_vertical_reversal_count}")

    print()
    print("=== State-space growth note ===")
    print("  Stage 12 state: (row,col,z,trend) -- 3 trend values per physical cell.")
    print("  Stage 14 state: (row,col,z,trend,trend_age_units) -- up to 3 x 11 = 33 combinations per physical cell.")
    print("  Not optimized in this stage per instructions; physical-primitive-feasibility caching noted as the "
          "candidate for a dedicated future performance stage.")


if __name__ == "__main__":
    main()
