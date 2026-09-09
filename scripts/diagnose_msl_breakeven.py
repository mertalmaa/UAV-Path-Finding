"""Stage 13: cost decomposition / break-even diagnostic for the real
Aladaglar scenario (same scenario as Stages 9-12: row=80, col 90->128,
2840m MSL cruise). Pure diagnostic -- no planner behavior change, no
weight tuning decision, no new/changed cost formula.

Enumerates smooth east-bound LEVEL^A DESCENT^k LEVEL^B CLIMB^k LEVEL^C
candidates (all descents before all climbs, no roller-coaster), validates
every edge with the existing evaluate_primitive(), and decomposes the
existing production edge cost into:

    G = sum(geometric_cost)                    -- pure distance
    M = sum(geometric_cost * altitude_scaled)   -- MSL "coefficient"
    R = sum(reversal_cost)                      -- reversal penalty

so that TOTAL(w_MSL) = G + w_MSL*M + R reproduces compute_edge_cost()'s
actual sum exactly (checked below), and the break-even w_MSL where a
descend-cruise-climb candidate becomes cheaper than staying flat can be
computed directly instead of guessed at.
"""
import dataclasses
import math
import time

from planner.astar import (
    _altitude_scaled, _is_reversal, _next_vertical_trend, _vertical_mode,
    _path_altitude_metrics, _path_vertical_reversal_metrics,
    compute_edge_cost, z_index_to_msl,
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
    """The one genuinely new piece of arithmetic here: sum(geometric_cost *
    altitude_scaled). Reuses _altitude_scaled (the actual production
    normalization), doesn't reimplement it."""
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
    """Authoritative cost: literally sums compute_edge_cost() -- the exact
    function A* uses -- over the candidate. Used to cross-check G+w*M+R."""
    alt = start_altitude_msl
    trend = 0
    total = 0.0
    for prim in sequence:
        total += compute_edge_cost(prim, alt, trend, config)
        trend = _next_vertical_trend(trend, _vertical_mode(prim))
        alt += prim.dz_m
    return total


def evaluate_candidate(start, sequence, terrain, primitives, config):
    """Walks the sequence, checking every edge with evaluate_primitive()
    (the existing safety authority). Returns None if any edge is invalid,
    else a dict of decomposed cost + descriptive metrics."""
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

    # Consistency check: decomposition must equal the actual production cost.
    for w in (0.0, W_REPRESENTATIVE, 0.10):
        decomposed = G + w * M + R
        actual = production_total(sequence, start_msl, dataclasses.replace(config, msl_cost_weight=w))
        assert abs(decomposed - actual) < 1e-6, f"decomposition mismatch at w={w}: {decomposed} vs {actual}"

    return {
        "path": physical_path, "G": G, "M": M, "R": R,
        "reversal_count": rev_metrics["vertical_reversal_count"],
        "geometric_length": G,
        "avg_msl": alt_metrics["average_aircraft_msl"],
        "min_msl": alt_metrics["minimum_aircraft_msl"],
        "max_msl": alt_metrics["maximum_aircraft_msl"],
        "total_climb": alt_metrics["total_climb_m"],
        "total_descent": alt_metrics["total_descent_m"],
        "min_agl": min_agl,
    }


def enumerate_k(k, level_e, descent_e, climb_e, start, terrain, primitives, config):
    total_cols = COL_GOAL - COL_START
    remaining = total_cols - 8 * k
    tried = 0
    invalid = 0
    best = None
    best_key = math.inf

    if k == 0:
        placements = [(remaining, 0, 0)]
    else:
        placements = [(a, b, remaining - a - b) for a in range(remaining + 1) for b in range(remaining - a + 1)]

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

    # --- Terrain profile summary ---
    seg = roi.elevation[ROW, COL_START:COL_GOAL + 1]
    seg_min, seg_max = float(seg.min()), float(seg.max())
    min_col = COL_START + int(seg.argmin())
    max_col = COL_START + int(seg.argmax())
    print("=== Terrain profile: row=80, col 90..128 ===")
    print(f"  start terrain (col{COL_START}) = {float(roi.elevation[ROW, COL_START]):.1f}m")
    print(f"  goal terrain  (col{COL_GOAL}) = {float(roi.elevation[ROW, COL_GOAL]):.1f}m")
    print(f"  min terrain = {seg_min:.1f}m at col{min_col}   max terrain = {seg_max:.1f}m at col{max_col}")
    print(f"  valley depth (max-min) = {seg_max - seg_min:.1f}m")
    print(f"  cruise = {CRUISE_MSL:.0f}m msl (cruise AGL over max terrain = {CRUISE_MSL - seg_max:.1f}m)")
    print()

    t0 = time.perf_counter()

    # --- k=0 baseline ---
    print("=== k=0 baseline: fully level at 2840m ===")
    flat_result = evaluate_candidate(start, [level_e] * (COL_GOAL - COL_START), tq, primitives, cfg)
    assert flat_result is not None, "flat baseline unexpectedly infeasible"
    print(f"  G_flat={flat_result['G']:.3f}  M_flat={flat_result['M']:.3f}  R_flat={flat_result['R']:.3f}")
    print(f"  geometric_length={flat_result['geometric_length']:.1f}  avg_MSL={flat_result['avg_msl']:.1f}  "
          f"min_AGL={flat_result['min_agl']:.1f}")
    print()

    # --- k=1..4 enumeration ---
    per_k_best = {}
    for k in (1, 2, 3, 4):
        tried, invalid, best = enumerate_k(k, level_e, descent_e, climb_e, start, tq, primitives, cfg)
        per_k_best[k] = best
        print(f"=== k={k} ({k * 20}m descent) ===")
        print(f"  candidates tried={tried}  infeasible={invalid}  feasible={tried - invalid}")
        if best is None:
            print("  NO FEASIBLE CANDIDATE for this k.")
            print()
            continue
        print(f"  best placement: A={best['a']} (level before descent) B={best['b']} (low cruise) "
              f"C={best['c']} (level after climb)")
        print(f"  min_MSL={best['min_msl']:.1f}  avg_MSL={best['avg_msl']:.1f}  max_MSL={best['max_msl']:.1f}")
        print(f"  geometric_length={best['geometric_length']:.3f}  reversal_count={best['reversal_count']}  "
              f"min_AGL={best['min_agl']:.1f}")
        print(f"  G={best['G']:.3f}  M={best['M']:.3f}  R={best['R']:.3f}  "
              f"TOTAL@w={W_REPRESENTATIVE}: {best['G'] + W_REPRESENTATIVE * best['M'] + best['R']:.3f}")
        print()

    elapsed = time.perf_counter() - t0
    print(f"Enumeration runtime: {elapsed:.2f}s")
    print()

    # --- Break-even w_MSL per k ---
    print("=== Break-even w_MSL (low candidate becomes cheaper than flat) ===")
    print(f"  flat: G={flat_result['G']:.3f} R={flat_result['R']:.3f} M={flat_result['M']:.3f}")
    breakevens = {}
    for k, best in per_k_best.items():
        if best is None:
            print(f"  k={k}: no feasible candidate -- break-even undefined")
            continue
        denom = flat_result["M"] - best["M"]
        num = (best["G"] + best["R"]) - (flat_result["G"] + flat_result["R"])
        if denom <= 0:
            print(f"  k={k}: M_low={best['M']:.3f} >= M_flat={flat_result['M']:.3f} -- "
                  f"NOT advantageous for MSL at any positive weight (denominator={denom:.3f})")
            breakevens[k] = None
        else:
            w_be = num / denom
            breakevens[k] = w_be
            print(f"  k={k}: extra_geometric+reversal={num:.3f}  M_gap={denom:.3f}  "
                  f"break-even w_MSL = {w_be:.4f}")
    print()

    # --- Numerical comparison across current/candidate weights ---
    print(f"=== TOTAL(w) comparison: flat vs each k's best candidate, w in {W_SWEEP} ===")
    header = f"{'w_MSL':>8}{'flat':>12}" + "".join(f"{'k=' + str(k):>12}" for k in (1, 2, 3, 4))
    print(header)
    for w in W_SWEEP:
        row_vals = [f"{flat_result['G'] + w * flat_result['M'] + flat_result['R']:>12.2f}"]
        for k in (1, 2, 3, 4):
            best = per_k_best[k]
            if best is None:
                row_vals.append(f"{'infeasible':>12}")
            else:
                row_vals.append(f"{best['G'] + w * best['M'] + best['R']:>12.2f}")
        print(f"{w:>8}" + "".join(row_vals))
    print()

    # --- Root-cause classification ---
    print("=== Diagnostic summary ===")
    any_feasible = any(v is not None for v in per_k_best.values())
    print(f"  Any feasible descend-cruise-climb candidate found (k=1..4): {any_feasible}")
    for k, w_be in breakevens.items():
        cur = "below current 0.025" if (w_be is not None and w_be > W_REPRESENTATIVE) else \
              ("ALREADY below current 0.025 -- should already be descending!" if w_be is not None else "N/A")
        print(f"  k={k}: break_even_w = {w_be if w_be is not None else 'N/A (MSL-disadvantaged)'}  ({cur})")


if __name__ == "__main__":
    main()
