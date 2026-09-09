"""Stage 29: Cost Weight Sensitivity / w_MSL Calibration.

Pure cost-recalibration diagnostic. NO astar_search() call for any NEW
search (candidate B is retrieved via Stage 28's exact one-time replay of
the already-completed Stage 26 eps=1.10 result -- reusing
validate_cost_function_ranking.get_stage26_eps110_path unchanged, not a
new search). NO candidate geometry is regenerated per weight -- the same
six Stage 28 paths (A-F), built by the same functions, are re-costed
across a w_MSL sweep with the production cost function UNCHANGED
(w_distance=1.0 fixed, reversal_weight/msl_reference_m/msl_scale_m
untouched -- only config.msl_cost_weight varies).

Adds ONE new candidate, G) DEEPER_BUT_LONGER_DETOUR, built once (fixed
geometry, independent of weight) by actually scanning the real terrain for
a column offering a lower "required safe MSL" than the direct corridor's
3400m floor within the same dwell row-band, then constructing a real,
production-validated detour to and from it. See build_deeper_longer_detour
docstring for exactly what the terrain allows and why the detour looks the
way it does (the real terrain only offers 20m of extra depth within a
reasonable lateral distance -- not invented, measured).
"""
import dataclasses
import math

from planner.astar import (
    _path_altitude_metrics, _path_vertical_reversal_metrics, msl_to_z_index, validate_and_cost_path,
)
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from scripts.benchmark_long_valley import verify_path_safety
from scripts.calibrate_low_msl_behavior import compute_vertical_profile_metrics, decompose_cost
from scripts.validate_cost_function_ranking import (
    START_COL, START_ROW,
    build_direct_level, build_early_deep_valley, build_late_descent,
    build_long_detour, build_roller_coaster, get_stage26_eps110_path,
)

AIRCRAFT_MSL = 3760.0
W_MSL_BASELINE = 0.63
W_MSL_SWEEP = [0.30, 0.40, 0.50, 0.63, 0.80, 1.00, 1.25]
GAP_THRESHOLD = 1.05  # target_suboptimality from the weighted-A* stages -- NOT re-run here, only referenced


def build_deeper_longer_detour(z0):
    """Real-terrain-measured, NOT invented. A scan of required-safe-MSL
    (ceil((max_terrain_in_window + min_agl_m)/z_step)*z_step) over the same
    kind of wide dwell-window used for candidate C, across columns 220-296,
    found the lowest reachable value is 3380m (one 20m z-step below C's
    3400m), centered around column ~264-267 -- roughly 12 columns
    (~360m laterally) west of the direct corridor (column 276). No column
    in the scanned range offers materially more depth than this within
    the ROI. This candidate detours there, drops the one extra available
    step, dwells briefly, and returns -- honestly representing what this
    terrain actually allows, not a constructed extreme.

    Same start_buffer_rows(4) + 18-step descent to 3400m as
    build_early_deep_valley (identical prefix, so any G-vs-C cost
    difference in that shared prefix is exactly zero -- the detour is
    entirely inside what would otherwise be C's 68-row dwell budget).
    """
    row, col, z = START_ROW, START_COL, z0
    path = [(row, col, z)]
    for _ in range(4):  # same mandatory near-start safety buffer as candidate C
        row += 1
        path.append((row, col, z))
    for _ in range(18):  # descend to 3400m, identical to candidate C
        row += 4
        z -= 1
        path.append((row, col, z))
    # -- detour begins here (row=124, col=276, z=170/3400m) --
    for _ in range(12):  # SW level: laterally toward column ~264
        row += 1
        col -= 1
        path.append((row, col, z))
    row += 4  # one more descent step -- the only extra depth this terrain offers
    z -= 1
    path.append((row, col, z))
    for _ in range(20):  # dwell briefly at the deeper point
        row += 1
        path.append((row, col, z))
    row += 4  # climb back the one extra step
    z += 1
    path.append((row, col, z))
    for _ in range(12):  # SE level: back to column 276
        row += 1
        col += 1
        path.append((row, col, z))
    for _ in range(16):  # remainder of the original 68-row dwell budget, at 3400m
        row += 1
        path.append((row, col, z))
    for _ in range(18):  # climb back to goal MSL, identical to candidate C
        row += 4
        z += 1
        path.append((row, col, z))
    assert row == 264 and col == START_COL and z == z0, (row, col, z)
    return path


def cost_candidate_at_weight(label, path, primitives_by_w, tq, cfg_base, w_msl):
    cfg = dataclasses.replace(cfg_base, msl_cost_weight=w_msl)
    primitives = primitives_by_w  # primitive SET itself doesn't depend on msl_cost_weight
    ok, total_cost = validate_and_cost_path(path, primitives, tq, cfg)
    if not ok:
        return None
    decomp = decompose_cost(path, primitives, tq, cfg)
    recombined = decomp["G"] + w_msl * decomp["M"] + decomp["R"]
    return {
        "label": label, "w_msl": w_msl, "total_cost": total_cost,
        "G": decomp["G"], "M_raw": decomp["M"], "wM": w_msl * decomp["M"], "R": decomp["R"],
        "match": abs(total_cost - recombined) < 1e-6,
    }


def main() -> None:
    cfg_base = DEFAULT_CONFIG
    primitives = build_primitive_set(cfg_base)  # primitive geometry independent of msl_cost_weight

    roi = load_roi(cfg_base)
    tq = TerrainQuery(roi)

    z0 = msl_to_z_index(AIRCRAFT_MSL, cfg_base)
    start, goal = (START_ROW, START_COL, z0), (264, START_COL, z0)

    seg = roi.elevation[START_ROW:264 + 1, START_COL]
    seg_min = float(seg.min())
    min_search = math.ceil((seg_min + cfg_base.min_agl_m) / cfg_base.z_step_m) * cfg_base.z_step_m
    max_search = AIRCRAFT_MSL + 20.0

    print("=== Building fixed candidate geometries (Stage 28, unchanged) ===")
    cfg_for_b_retrieval = dataclasses.replace(cfg_base, msl_cost_weight=W_MSL_BASELINE)
    b_path = get_stage26_eps110_path(cfg_for_b_retrieval, primitives, tq, start, goal, min_search, max_search)
    print()

    candidates = {
        "A_DIRECT_LEVEL": build_direct_level(z0),
        "B_SHALLOW_VALLEY": b_path,
        "C_EARLY_DEEP_VALLEY": build_early_deep_valley(z0),
        "D_LATE_DESCENT": build_late_descent(z0),
        "E_ROLLER_COASTER": build_roller_coaster(z0),
        "F_LONG_DETOUR": build_long_detour(z0),
    }

    print("=== Building candidate G (deeper-but-longer detour) ===")
    g_path = build_deeper_longer_detour(z0)
    ok_g, _ = validate_and_cost_path(
        g_path, primitives, tq, dataclasses.replace(cfg_base, msl_cost_weight=W_MSL_BASELINE))
    if ok_g:
        candidates["G_DEEPER_LONGER_DETOUR"] = g_path
        profile_g = compute_vertical_profile_metrics(g_path, primitives, tq, cfg_base)
        print(f"  VALID. min_MSL={profile_g['min_path_msl']:.0f}m (candidate C's min_MSL=3400m -- "
              f"{3400 - profile_g['min_path_msl']:.0f}m deeper) geom_len={profile_g['total_horizontal_distance_m']:.1f}m")
    else:
        print("  INVALID -- construction failed safety check, NOT included in sweep (not fabricated).")
    print()

    # --- Sweep: cost every candidate at every w_MSL, geometry held fixed ---
    order = ["A_DIRECT_LEVEL", "B_SHALLOW_VALLEY", "C_EARLY_DEEP_VALLEY", "D_LATE_DESCENT",
             "E_ROLLER_COASTER", "F_LONG_DETOUR", "G_DEEPER_LONGER_DETOUR"]
    order = [o for o in order if o in candidates]

    sweep_results = {label: {} for label in order}
    for w in W_MSL_SWEEP:
        for label in order:
            r = cost_candidate_at_weight(label, candidates[label], primitives, tq, cfg_base, w)
            sweep_results[label][w] = r

    # --- Sanity check: G and R invariant across the sweep for each candidate ---
    print("=== Sanity check: G (geometric) and R (reversal) must be IDENTICAL across the whole w_MSL sweep ===")
    all_invariant = True
    for label in order:
        g_vals = {round(sweep_results[label][w]["G"], 6) for w in W_MSL_SWEEP}
        r_vals = {round(sweep_results[label][w]["R"], 6) for w in W_MSL_SWEEP}
        invariant = len(g_vals) == 1 and len(r_vals) == 1
        all_invariant &= invariant
        match_all = all(sweep_results[label][w]["match"] for w in W_MSL_SWEEP)
        print(f"  {label}: G invariant={len(g_vals)==1} (G={next(iter(g_vals)):.2f})  "
              f"R invariant={len(r_vals)==1} (R={next(iter(r_vals)):.2f})  "
              f"production==G+w*M+R for all weights: {match_all}")
    print(f"  ALL CANDIDATES INVARIANT: {all_invariant}\n")

    # --- Main table ---
    print("=== MAIN TABLE: w_MSL sweep, gaps vs C (EARLY_DEEP_VALLEY) ===")
    header = (f"{'w_MSL':>7} {'deep_cost':>10} {'shallow_gap%':>13} {'level_gap%':>11} {'late_gap%':>10} "
              f"{'roller_gap%':>12} {'detour_gap%':>12}")
    if "G_DEEPER_LONGER_DETOUR" in candidates:
        header += f" {'deeper_gap%':>12}"
    header += f" {'shallow>5%':>11} {'level>5%':>9} {'late>5%':>8}"
    print(header)

    main_rows = []
    for w in W_MSL_SWEEP:
        deep_cost = sweep_results["C_EARLY_DEEP_VALLEY"][w]["total_cost"]

        def gap(label):
            return (sweep_results[label][w]["total_cost"] / deep_cost - 1) * 100.0

        shallow_gap = gap("B_SHALLOW_VALLEY")
        level_gap = gap("A_DIRECT_LEVEL")
        late_gap = gap("D_LATE_DESCENT")
        roller_gap = gap("E_ROLLER_COASTER")
        detour_gap = gap("F_LONG_DETOUR")
        deeper_gap = gap("G_DEEPER_LONGER_DETOUR") if "G_DEEPER_LONGER_DETOUR" in candidates else None

        shallow_over5 = shallow_gap > 5.0
        level_over5 = level_gap > 5.0
        late_over5 = late_gap > 5.0

        row_str = (f"{w:>7.2f} {deep_cost:>10.2f} {shallow_gap:>12.2f}% {level_gap:>10.2f}% {late_gap:>9.2f}% "
                   f"{roller_gap:>11.2f}% {detour_gap:>11.2f}%")
        if deeper_gap is not None:
            row_str += f" {deeper_gap:>11.2f}%"
        row_str += f" {str(shallow_over5):>11} {str(level_over5):>9} {str(late_over5):>8}"
        marker = "   <== BASELINE" if abs(w - W_MSL_BASELINE) < 1e-9 else ""
        print(row_str + marker)

        main_rows.append({
            "w_msl": w, "deep_cost": deep_cost, "shallow_gap": shallow_gap, "level_gap": level_gap,
            "late_gap": late_gap, "roller_gap": roller_gap, "detour_gap": detour_gap,
            "deeper_gap": deeper_gap, "shallow_over5": shallow_over5, "level_over5": level_over5,
            "late_over5": late_over5,
        })
    print()

    # --- Common baseline / absolute-gap analysis ---
    print("=== EK ANALIZ: absolute gap decomposition (Deep vs Level, Deep vs Shallow) ===")
    for w in W_MSL_SWEEP:
        c = sweep_results["C_EARLY_DEEP_VALLEY"][w]
        a = sweep_results["A_DIRECT_LEVEL"][w]
        b = sweep_results["B_SHALLOW_VALLEY"][w]
        d_level_abs = a["total_cost"] - c["total_cost"]
        d_level_G = a["G"] - c["G"]
        d_level_wM = a["wM"] - c["wM"]
        d_shallow_abs = b["total_cost"] - c["total_cost"]
        d_shallow_G = b["G"] - c["G"]
        d_shallow_wM = b["wM"] - c["wM"]
        common_base_level = min(a["total_cost"], c["total_cost"])
        common_base_shallow = min(b["total_cost"], c["total_cost"])
        print(f"  w_MSL={w:.2f}:")
        print(f"    Level-Deep: abs_diff={d_level_abs:.2f}  (from G: {d_level_G:.2f}, from w*M: {d_level_wM:.2f})  "
              f"common_baseline~{common_base_level:.2f} ({common_base_level/max(a['total_cost'],c['total_cost'])*100:.1f}% of larger)")
        print(f"    Shallow-Deep: abs_diff={d_shallow_abs:.2f}  (from G: {d_shallow_G:.2f}, from w*M: {d_shallow_wM:.2f})  "
              f"common_baseline~{common_base_shallow:.2f} ({common_base_shallow/max(b['total_cost'],c['total_cost'])*100:.1f}% of larger)")
    print()

    # --- Degenerate-behavior watch: does G (deeper detour) ever become cheaper than C? ---
    if "G_DEEPER_LONGER_DETOUR" in candidates:
        print("=== DEEPER_LONGER_DETOUR (G) vs C across weights -- watching for a cost-crossover ===")
        crossover_w = None
        for w in W_MSL_SWEEP:
            g_cost = sweep_results["G_DEEPER_LONGER_DETOUR"][w]["total_cost"]
            c_cost = sweep_results["C_EARLY_DEEP_VALLEY"][w]["total_cost"]
            cheaper = g_cost < c_cost
            if cheaper and crossover_w is None:
                crossover_w = w
            print(f"  w_MSL={w:.2f}: G={g_cost:.2f}  C={c_cost:.2f}  G_cheaper_than_C={cheaper}")
        print(f"  Crossover weight (G becomes cheaper than C): {crossover_w if crossover_w else 'NONE in tested range'}")
        print()

    # --- Roller-coaster / late-descent degeneracy watch across the sweep ---
    print("=== Roller-coaster and late-descent separation across the sweep ===")
    for w in W_MSL_SWEEP:
        e = sweep_results["E_ROLLER_COASTER"][w]
        c = sweep_results["C_EARLY_DEEP_VALLEY"][w]
        d = sweep_results["D_LATE_DESCENT"][w]
        a = sweep_results["A_DIRECT_LEVEL"][w]
        print(f"  w_MSL={w:.2f}: roller_vs_deep_gap={(e['total_cost']/c['total_cost']-1)*100:.2f}%  "
              f"roller_still_worse_than_level={e['total_cost'] > a['total_cost']}  "
              f"late_vs_early_gap={(d['total_cost']/c['total_cost']-1)*100:.2f}%")
    print()

    # --- Pick a recommended weight: smallest w in the sweep where BOTH level and shallow clear 5%, if any ---
    recommended = None
    for row in main_rows:
        if row["level_over5"] and row["shallow_over5"]:
            recommended = row["w_msl"]
            break
    print(f"=== Recommended-weight scan result: {'w_MSL=' + str(recommended) if recommended else 'NONE of the tested weights clear the 5% band on BOTH level and shallow'} ===\n")

    # --- Second table: candidate cost @0.63 vs @recommended (or @1.25 if none clears 5%) ---
    compare_w = recommended if recommended else W_MSL_SWEEP[-1]
    print(f"=== SECOND TABLE: cost @0.63 (baseline) vs @{compare_w} ({'recommended' if recommended else 'highest tested, no weight cleared 5% band'}) ===")
    header2 = f"{'route':>22} {'cost@0.63':>11} {'cost@' + str(compare_w):>11} {'gap@0.63%':>11} {'gap@' + str(compare_w) + '%':>12}"
    print(header2)
    deep_063 = sweep_results["C_EARLY_DEEP_VALLEY"][W_MSL_BASELINE]["total_cost"]
    deep_cmp = sweep_results["C_EARLY_DEEP_VALLEY"][compare_w]["total_cost"]
    for label in order:
        c063 = sweep_results[label][W_MSL_BASELINE]["total_cost"]
        ccmp = sweep_results[label][compare_w]["total_cost"]
        gap063 = (c063 / deep_063 - 1) * 100.0
        gapcmp = (ccmp / deep_cmp - 1) * 100.0
        print(f"{label:>22} {c063:>11.2f} {ccmp:>11.2f} {gap063:>10.2f}% {gapcmp:>11.2f}%")
    print()


if __name__ == "__main__":
    main()
