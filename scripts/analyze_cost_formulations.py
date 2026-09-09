"""Stage 30: Cost Formulation Redesign / Common-Baseline Analysis.

Pure diagnostic. NO astar_search(), NO production code change
(planner/astar.py, planner/config.py untouched). Reuses the EXACT same
seven Stage 28/29 candidate paths (A-G, geometry unchanged) and re-costs
them under three altitude-term formulations:

    F0 (CURRENT production): edge_cost = geom*(1 + w_MSL*excess0/scale)
       + reversal, excess0 = max(0, mean_MSL - msl_reference_m=0)
    F1 (shifted linear):     same shape, but excess = max(0, mean_MSL -
       H_FLOOR) -- a single mission-wide constant, not state/row/col
       dependent, not a terrain-following reference.
    F2 (shifted quadratic):  edge_cost = geom*(1 + w_alt*(excess/scale)^2)
       + reversal -- same H_FLOOR excess as F1, squared.

Mission objective is still "prefer low ABSOLUTE MSL" throughout -- excess
is always (aircraft MSL - a fixed global constant), never (aircraft MSL -
local terrain). AGL stays a pure hard-safety gate (evaluate_primitive /
evaluate_agl are never touched here).

H_FLOOR reuses planner.astar._terrain_min_valid_elevation /
_min_possible_aircraft_msl UNCHANGED (Stage 17's own global-floor logic)
with this benchmark's own min_search_altitude_msl=3240 (Stage 26/29's
value) -- not a new formula, not row/col/state dependent, fixed once
before any candidate is costed.
"""
import dataclasses
import math

from planner.astar import (
    _min_possible_aircraft_msl, _path_altitude_metrics, _path_vertical_reversal_metrics,
    _terrain_min_valid_elevation, msl_to_z_index, state_to_xyz, validate_and_cost_path,
)
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from scripts.calibrate_low_msl_behavior import decompose_cost
from scripts.calibrate_msl_weight_sensitivity import build_deeper_longer_detour
from scripts.validate_cost_function_ranking import (
    START_COL, START_ROW,
    build_direct_level, build_early_deep_valley, build_late_descent,
    build_long_detour, build_roller_coaster, get_stage26_eps110_path,
)

AIRCRAFT_MSL = 3760.0
MIN_SEARCH_ALTITUDE_MSL = 3240.0  # Stage 26/29's own value for this exact benchmark
MSL_SCALE_M = 1000.0  # unchanged from production config.msl_scale_m
W_ALT_SWEEP = [0.25, 0.5, 0.75, 1.0, 1.5, 2.0]
GAP_THRESHOLD = 5.0


def edge_geom_and_meanalt(path, primitives, terrain, config):
    """Per-edge (geometric_cost, mean_altitude_msl) list -- shared by every
    formulation below, so G is byte-for-byte identical across F0/F1/F2
    (only the altitude term differs)."""
    by_delta = {(p.drow, p.dcol, round(p.dz_m / config.z_step_m)): p for p in primitives}
    xyz = [state_to_xyz(s, terrain, config) for s in path]
    out = []
    for i, ((r1, c1, z1), (r2, c2, z2)) in enumerate(zip(path, path[1:])):
        prim = by_delta[(r2 - r1, c2 - c1, z2 - z1)]
        geometric_cost = math.sqrt(prim.horizontal_distance_m ** 2 + prim.dz_m ** 2)
        mean_alt = (xyz[i][2] + xyz[i + 1][2]) / 2.0
        out.append((geometric_cost, mean_alt))
    return out


def altitude_component(edges, reference_msl, scale_m, power):
    """M = sum(geom_edge * (max(0, mean_alt - reference)/scale)^power).
    power=1 -> F0/F1 (linear excess), power=2 -> F2 (quadratic excess).
    Always >= 0 per edge (max(0,...) before any power), so the total stays
    non-negative and additive -- no negative reward, ever."""
    total = 0.0
    for geometric_cost, mean_alt in edges:
        excess = max(0.0, mean_alt - reference_msl)
        normalized = excess / scale_m
        total += geometric_cost * (normalized ** power)
    return total


def total_cost_for(G, M, R, w_alt):
    return G + w_alt * M + R


def main() -> None:
    cfg = DEFAULT_CONFIG
    primitives = build_primitive_set(cfg)
    roi = load_roi(cfg)
    tq = TerrainQuery(roi)
    z0 = msl_to_z_index(AIRCRAFT_MSL, cfg)
    start, goal = (START_ROW, START_COL, z0), (264, START_COL, z0)

    # --- H_FLOOR: reuse Stage 17's own global-floor logic, unchanged ---
    terrain_min_valid = _terrain_min_valid_elevation(tq)
    h_floor = _min_possible_aircraft_msl(terrain_min_valid, MIN_SEARCH_ALTITUDE_MSL, cfg)
    print(f"=== H_FLOOR ===")
    print(f"  terrain_min_valid (whole ROI) = {terrain_min_valid:.2f}m")
    print(f"  min_search_altitude_msl (this benchmark, Stage 26/29) = {MIN_SEARCH_ALTITUDE_MSL}m")
    print(f"  H_FLOOR = max(min_search, terrain_min_valid + min_agl_m) = {h_floor:.2f}m")
    print(f"  (fixed once, state/row/col-independent, reused for every candidate/weight below)\n")

    # --- Build candidates (Stage 28/29 geometry, unchanged) ---
    seg = roi.elevation[START_ROW:264 + 1, START_COL]
    seg_min = float(seg.min())
    min_search_b = math.ceil((seg_min + cfg.min_agl_m) / cfg.z_step_m) * cfg.z_step_m
    max_search_b = AIRCRAFT_MSL + 20.0
    cfg_b = dataclasses.replace(cfg, msl_cost_weight=0.63)
    print("=== Retrieving candidate B (Stage 26 path, unchanged) ===")
    b_path = get_stage26_eps110_path(cfg_b, primitives, tq, start, goal, min_search_b, max_search_b)
    print()

    candidates = {
        "A_DIRECT_LEVEL": build_direct_level(z0),
        "B_SHALLOW_VALLEY": b_path,
        "C_EARLY_DEEP_VALLEY": build_early_deep_valley(z0),
        "D_LATE_DESCENT": build_late_descent(z0),
        "E_ROLLER_COASTER": build_roller_coaster(z0),
        "F_LONG_DETOUR": build_long_detour(z0),
        "G_DEEPER_LONGER_DETOUR": build_deeper_longer_detour(z0),
    }
    order = list(candidates.keys())

    # --- G (geometric) and R (reversal) -- identical for every formulation, computed once ---
    GR = {}
    edges_per_candidate = {}
    for label in order:
        path = candidates[label]
        ok, _ = validate_and_cost_path(path, primitives, tq, cfg)
        assert ok, f"{label} unexpectedly invalid"
        alt_metrics = _path_altitude_metrics(path, tq, cfg)
        rev_metrics = _path_vertical_reversal_metrics(path, primitives, cfg)
        GR[label] = (alt_metrics["geometric_path_length"], rev_metrics["total_reversal_penalty"])
        edges_per_candidate[label] = edge_geom_and_meanalt(path, primitives, tq, cfg)

    # --- Sanity check: F0 (reference=0, power=1, w=0.63) must reproduce Stage 29's decompose_cost() exactly ---
    print("=== Sanity check: F0 @ w=0.63 reproduces Stage 28/29 production decomposition ===")
    for label in order:
        G, R = GR[label]
        M_f0 = altitude_component(edges_per_candidate[label], reference_msl=0.0, scale_m=MSL_SCALE_M, power=1)
        prod_decomp = decompose_cost(candidates[label], primitives, tq, dataclasses.replace(cfg, msl_cost_weight=0.63))
        match_G = abs(G - prod_decomp["G"]) < 1e-6
        match_M = abs(M_f0 - prod_decomp["M"]) < 1e-6
        match_R = abs(R - prod_decomp["R"]) < 1e-6
        print(f"  {label}: G_match={match_G} M_match={match_M} R_match={match_R}")
    print()

    # --- Sweep F0 / F1 / F2 ---
    formulations = {
        "F0_CURRENT": dict(reference=0.0, power=1),
        "F1_SHIFTED_LINEAR": dict(reference=h_floor, power=1),
        "F2_SHIFTED_QUADRATIC": dict(reference=h_floor, power=2),
    }

    results = {}  # results[formulation][w][label] = {"G":..,"M":..,"R":..,"total":..}
    for fname, params in formulations.items():
        results[fname] = {}
        w_values = [0.63] if fname == "F0_CURRENT" else sorted(set(W_ALT_SWEEP + [0.63]))
        for w in w_values:
            results[fname][w] = {}
            for label in order:
                G, R = GR[label]
                M = altitude_component(edges_per_candidate[label], reference_msl=params["reference"],
                                        scale_m=MSL_SCALE_M, power=params["power"])
                total = total_cost_for(G, M, R, w)
                results[fname][w][label] = {"G": G, "M": M, "R": R, "total": total}

    def gaps_for(fname, w):
        deep = results[fname][w]["C_EARLY_DEEP_VALLEY"]["total"]
        out = {}
        for label in order:
            out[label] = (results[fname][w][label]["total"] / deep - 1) * 100.0
        return out, deep

    # --- Report each formulation's w_alt sweep ---
    for fname in ["F1_SHIFTED_LINEAR", "F2_SHIFTED_QUADRATIC"]:
        print(f"=== {fname} sweep (reference={formulations[fname]['reference']:.1f}, power={formulations[fname]['power']}) ===")
        header = (f"{'w_alt':>6} {'deep_cost':>10} {'shallow%':>9} {'level%':>8} {'late%':>7} "
                  f"{'roller%':>8} {'detour%':>8} {'deeper%':>8} {'shallow>5':>10} {'level>5':>8} {'G<C?':>6}")
        print(header)
        for w in W_ALT_SWEEP:
            gaps, deep = gaps_for(fname, w)
            shallow, level, late, roller, detour, deeper = (
                gaps["B_SHALLOW_VALLEY"], gaps["A_DIRECT_LEVEL"], gaps["D_LATE_DESCENT"],
                gaps["E_ROLLER_COASTER"], gaps["F_LONG_DETOUR"], gaps["G_DEEPER_LONGER_DETOUR"],
            )
            print(f"{w:>6.2f} {deep:>10.2f} {shallow:>8.2f}% {level:>7.2f}% {late:>6.2f}% "
                  f"{roller:>7.2f}% {detour:>7.2f}% {deeper:>7.2f}% {str(shallow>5):>10} "
                  f"{str(level>5):>8} {str(deeper<0):>6}")
        print()

    # --- Common-baseline analysis: F0 vs F1 at matched w (0.63 as reference point) ---
    print("=== Common-baseline analysis: F0 (w=0.63) vs F1 (w=0.63) ===")
    for fname, w in [("F0_CURRENT", 0.63), ("F1_SHIFTED_LINEAR", 0.63)]:
        r = results[fname][w]
        deep_t, shallow_t, level_t = r["C_EARLY_DEEP_VALLEY"]["total"], r["B_SHALLOW_VALLEY"]["total"], r["A_DIRECT_LEVEL"]["total"]
        deep_M, shallow_M, level_M = r["C_EARLY_DEEP_VALLEY"]["M"], r["B_SHALLOW_VALLEY"]["M"], r["A_DIRECT_LEVEL"]["M"]
        print(f"  {fname} (w={w}): Deep={deep_t:.2f} Shallow={shallow_t:.2f} Level={level_t:.2f}")
        print(f"    Deep-vs-Shallow: abs_diff={shallow_t-deep_t:.2f}  rel_gap={(shallow_t/deep_t-1)*100:.2f}%")
        print(f"    Deep-vs-Level:   abs_diff={level_t-deep_t:.2f}  rel_gap={(level_t/deep_t-1)*100:.2f}%")
        print(f"    M_raw: Deep={deep_M:.2f} Shallow={shallow_M:.2f} Level={level_M:.2f}  "
              f"(this is the 'common baseline' magnitude -- smaller M_raw(Deep) means less of total cost is shared/baseline)")
    print()

    # --- Asymptotic ceiling analysis for F1 (still linear -> same closed form as Stage 29) ---
    print("=== F1 asymptotic ceiling (w_alt -> infinity), closed form like Stage 29 ===")
    w_probe = 0.63  # any single w suffices to extract the two linear coefficients (G, M) per candidate
    r = results["F1_SHIFTED_LINEAR"][w_probe]
    G_C, M_C = r["C_EARLY_DEEP_VALLEY"]["G"], r["C_EARLY_DEEP_VALLEY"]["M"]
    for label in ["A_DIRECT_LEVEL", "B_SHALLOW_VALLEY", "D_LATE_DESCENT", "F_LONG_DETOUR", "G_DEEPER_LONGER_DETOUR"]:
        G_x, M_x = r[label]["G"], r[label]["M"]
        ceiling = (M_x - M_C) / M_C * 100.0
        print(f"  {label}: asymptotic gap ceiling (w->inf) = {ceiling:.2f}%  "
              f"(F0's was: see Stage 29 -- Level=5.63%, Shallow=4.41%, DeeperDetour=4.38%)")
    print()

    # --- Early vs Late (C vs D) across formulations ---
    print("=== Early (C) vs Late (D) across formulations ===")
    for fname in results:
        w_values = list(results[fname].keys())
        for w in w_values:
            c, d = results[fname][w]["C_EARLY_DEEP_VALLEY"], results[fname][w]["D_LATE_DESCENT"]
            gap = (d["total"] / c["total"] - 1) * 100.0
            print(f"  {fname} w={w}: C={c['total']:.2f} D={d['total']:.2f} gap={gap:.2f}%  "
                  f"(same G/depth/min_MSL as always -- only ordering differs)")
    print()

    # --- Roller-coaster check across formulations ---
    print("=== Roller-coaster (E) vs Deep (C) and vs Level (A) across formulations ===")
    for fname in results:
        for w in results[fname]:
            e, c, a = results[fname][w]["E_ROLLER_COASTER"], results[fname][w]["C_EARLY_DEEP_VALLEY"], results[fname][w]["A_DIRECT_LEVEL"]
            print(f"  {fname} w={w}: E={e['total']:.2f}  worse_than_C={e['total']>c['total']}  "
                  f"worse_than_A={e['total']>a['total']}  gap_vs_C={(e['total']/c['total']-1)*100:.2f}%")
    print()

    # --- Degenerate-detour check: does G ever undercut C under F1/F2? ---
    print("=== Degenerate-detour check: G_DEEPER_LONGER_DETOUR vs C across all formulations/weights ===")
    for fname in ["F1_SHIFTED_LINEAR", "F2_SHIFTED_QUADRATIC"]:
        for w in W_ALT_SWEEP:
            g, c = results[fname][w]["G_DEEPER_LONGER_DETOUR"], results[fname][w]["C_EARLY_DEEP_VALLEY"]
            cheaper = g["total"] < c["total"]
            print(f"  {fname} w={w}: G={g['total']:.2f} C={c['total']:.2f} G_cheaper_than_C={cheaper}")
    print()

    # --- MAIN RESULT TABLE: best candidate weight per formulation ---
    print("=== MAIN TABLE ===")
    print(f"{'Formulation':>22} {'Reference':>10} {'Power':>6} {'w_alt':>6} {'Deep':>9} {'Shallow%':>9} "
          f"{'Level%':>8} {'Late%':>7} {'Roller%':>8} {'Detour%':>8} {'Deeper%':>8} {'Sh>5':>5} {'Lv>5':>5} "
          f"{'DegDetour':>10} {'A*-compat':>10}")
    # F0 baseline row
    r0 = results["F0_CURRENT"][0.63]
    deep0 = r0["C_EARLY_DEEP_VALLEY"]["total"]
    g0 = {l: (r0[l]["total"] / deep0 - 1) * 100 for l in order}
    print(f"{'F0_CURRENT':>22} {'0':>10} {'1':>6} {'0.63':>6} {deep0:>9.2f} {g0['B_SHALLOW_VALLEY']:>8.2f}% "
          f"{g0['A_DIRECT_LEVEL']:>7.2f}% {g0['D_LATE_DESCENT']:>6.2f}% {g0['E_ROLLER_COASTER']:>7.2f}% "
          f"{g0['F_LONG_DETOUR']:>7.2f}% {g0['G_DEEPER_LONGER_DETOUR']:>7.2f}% "
          f"{str(g0['B_SHALLOW_VALLEY']>5):>5} {str(g0['A_DIRECT_LEVEL']>5):>5} {'No':>10} {'Yes(proven)':>10}")
    for fname in ["F1_SHIFTED_LINEAR", "F2_SHIFTED_QUADRATIC"]:
        ref = formulations[fname]["reference"]
        pw = formulations[fname]["power"]
        for w in W_ALT_SWEEP:
            gaps, deep = gaps_for(fname, w)
            deg = gaps["G_DEEPER_LONGER_DETOUR"] < 0
            print(f"{fname:>22} {ref:>10.1f} {pw:>6} {w:>6.2f} {deep:>9.2f} {gaps['B_SHALLOW_VALLEY']:>8.2f}% "
                  f"{gaps['A_DIRECT_LEVEL']:>7.2f}% {gaps['D_LATE_DESCENT']:>6.2f}% {gaps['E_ROLLER_COASTER']:>7.2f}% "
                  f"{gaps['F_LONG_DETOUR']:>7.2f}% {gaps['G_DEEPER_LONGER_DETOUR']:>7.2f}% "
                  f"{str(gaps['B_SHALLOW_VALLEY']>5):>5} {str(gaps['A_DIRECT_LEVEL']>5):>5} "
                  f"{str(deg):>10} {'analysis-only':>10}")
    print()


if __name__ == "__main__":
    main()
