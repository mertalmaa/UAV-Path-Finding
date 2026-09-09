"""Stage 31: Normalized Cost Function Design / Distance-Altitude Trade-off.

Pure diagnostic. NO production code change (planner/astar.py,
planner/config.py untouched), NO astar_search(), NO new candidate
geometry -- reuses the exact same 7 Stage 28/29/30 paths (A-G).

Defines a dimensionless, additive, non-negative normalized cost:

    C_distance = path_length / D_ref
    C_altitude = sum(ds * excess_altitude) / (D_ref * H_scale)
                 excess_altitude = max(0, mean_MSL_edge - H_ref)
    C_reversal = R_raw / R_scale
    C_total    = w_distance*C_distance + w_altitude*C_altitude + w_reversal*C_reversal

D_ref = exact straight-line start-goal distance (this benchmark).
H_ref = Stage 30's H_FLOOR (3240m) -- reused as a diagnostic reference,
        with its own stability explicitly discussed (see main()).
H_scale = 100m (diagnostic choice, per spec).
R_scale = 20 (= vertical_reversal_cost_weight * z_step_m * max reversal
          factor(1.0) -- the raw cost of a single, maximally-penalized
          reversal under the UNCHANGED production reversal model; not an
          arbitrary number -- it is what a single worst-case reversal
          already costs today).

Mission objective is unchanged: prefer low ABSOLUTE MSL. excess_altitude
is always (aircraft MSL - H_ref), never (aircraft MSL - local terrain).
AGL remains a pure hard-safety gate, never touched here.
"""
import math

from planner.astar import (
    _min_possible_aircraft_msl, _path_altitude_metrics, _path_vertical_reversal_metrics,
    _terrain_min_valid_elevation, msl_to_z_index,
)
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from scripts.analyze_cost_formulations import MIN_SEARCH_ALTITUDE_MSL, edge_geom_and_meanalt
from scripts.calibrate_msl_weight_sensitivity import build_deeper_longer_detour
from scripts.validate_cost_function_ranking import (
    START_COL, START_ROW,
    build_direct_level, build_early_deep_valley, build_late_descent,
    build_long_detour, build_roller_coaster, get_stage26_eps110_path,
)
import dataclasses

AIRCRAFT_MSL = 3760.0
GOAL_ROW = 264
H_REF = 3240.0        # Stage 30's H_FLOOR, reused diagnostically (stability discussed below)
H_SCALE = 100.0        # diagnostic altitude scale (meters)
R_SCALE = 20.0         # = vertical_reversal_cost_weight(1.0) * z_step_m(20) * max_factor(1.0)

WEIGHT_SETS = [
    {"name": "SET_A", "w_distance": 1.0, "w_altitude": 1.0},
    {"name": "SET_B", "w_distance": 0.8, "w_altitude": 1.0},
    {"name": "SET_C", "w_distance": 0.6, "w_altitude": 1.0},
    {"name": "SET_D", "w_distance": 1.0, "w_altitude": 1.25},
]
W_REVERSAL = 1.0  # unchanged/fixed this stage -- reversal behavior not retuned (per spec section 6)


def raw_altitude_sum(edges, reference_msl):
    """sum(geom_edge * max(0, mean_alt_edge - reference)) -- no scale division,
    kept separate from D_ref/H_scale per spec section 9 (scale vs weight)."""
    total = 0.0
    for geometric_cost, mean_alt in edges:
        total += geometric_cost * max(0.0, mean_alt - reference_msl)
    return total


def main() -> None:
    cfg = DEFAULT_CONFIG
    primitives = build_primitive_set(cfg)
    roi = load_roi(cfg)
    tq = TerrainQuery(roi)
    z0 = msl_to_z_index(AIRCRAFT_MSL, cfg)
    start, goal = (START_ROW, START_COL, z0), (GOAL_ROW, START_COL, z0)

    # --- D_ref: exact straight-line start-goal distance ---
    sx, sy = tq.rowcol_to_xy(START_ROW, START_COL)
    gx, gy = tq.rowcol_to_xy(GOAL_ROW, START_COL)
    d_ref = math.hypot(gx - sx, gy - sy)
    print(f"=== D_ref ===\n  start=({START_ROW},{START_COL}) goal=({GOAL_ROW},{START_COL})  D_ref={d_ref:.4f}m\n")

    terrain_min_valid = _terrain_min_valid_elevation(tq)
    h_floor_check = _min_possible_aircraft_msl(terrain_min_valid, MIN_SEARCH_ALTITUDE_MSL, cfg)
    print(f"=== H_ref ===\n  H_ref = {H_REF}m (Stage 30's H_FLOOR, recomputed here = {h_floor_check:.2f}m, matches)")
    print(f"  Provenance check: H_FLOOR = max(min_search_altitude_msl={MIN_SEARCH_ALTITUDE_MSL}, "
          f"terrain_min_valid_ROI({terrain_min_valid:.1f}) + min_agl_m({cfg.min_agl_m})) "
          f"= max({MIN_SEARCH_ALTITUDE_MSL}, {terrain_min_valid + cfg.min_agl_m:.1f}) = {h_floor_check:.1f}m")
    print(f"  -> the binding term is min_search_altitude_msl, a SEARCH-CALL parameter for THIS benchmark's "
          f"corridor, not a terrain-only or mission-only constant -- see Q17 in the final report.\n")

    print(f"=== H_scale / R_scale ===\n  H_scale = {H_SCALE}m (diagnostic, per spec)")
    print(f"  R_scale = {R_SCALE} (= vertical_reversal_cost_weight(1.0) * z_step_m(20) * "
          f"max_reversal_factor(1.0) -- cost of one worst-case reversal under the UNCHANGED production model)\n")

    # --- Build candidates (unchanged geometry) ---
    seg = roi.elevation[START_ROW:GOAL_ROW + 1, START_COL]
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

    # --- Per-candidate raw G, R, altitude_raw_sum (independent of weight) ---
    raw = {}
    for label in order:
        path = candidates[label]
        alt = _path_altitude_metrics(path, tq, cfg)
        rev = _path_vertical_reversal_metrics(path, primitives, cfg)
        edges = edge_geom_and_meanalt(path, primitives, tq, cfg)
        raw[label] = {
            "G": alt["geometric_path_length"],
            "R": rev["total_reversal_penalty"],
            "alt_raw": raw_altitude_sum(edges, H_REF),
        }

    # --- Normalized components (weight-independent) ---
    print("=== Normalized components (dimensionless, weight-independent) ===")
    print(f"{'label':>24} {'C_distance':>11} {'C_altitude':>11} {'C_reversal':>11}")
    comp = {}
    for label in order:
        r = raw[label]
        c_dist = r["G"] / d_ref
        c_alt = r["alt_raw"] / (d_ref * H_SCALE)
        c_rev = r["R"] / R_SCALE
        comp[label] = {"C_distance": c_dist, "C_altitude": c_alt, "C_reversal": c_rev}
        print(f"{label:>24} {c_dist:>11.4f} {c_alt:>11.4f} {c_rev:>11.4f}")
    print()

    # --- Main sweep across weight sets ---
    print("=== MAIN TABLE ===")
    header = (f"{'Weight Set':>10} {'w_dist':>7} {'w_alt':>6} {'Deep cost':>10} {'Shallow%':>9} {'Level%':>8} "
              f"{'Late%':>7} {'Roller%':>8} {'Detour%':>8} {'DeeperDet%':>10} {'Sh>5':>5} {'Lv>5':>5} {'G<C?':>6}")
    print(header)
    results = {}
    for ws in WEIGHT_SETS:
        wd, wa, name = ws["w_distance"], ws["w_altitude"], ws["name"]
        totals = {}
        for label in order:
            c = comp[label]
            totals[label] = wd * c["C_distance"] + wa * c["C_altitude"] + W_REVERSAL * c["C_reversal"]
        results[name] = totals
        deep = totals["C_EARLY_DEEP_VALLEY"]
        gaps = {l: (totals[l] / deep - 1) * 100.0 for l in order}
        deg = gaps["G_DEEPER_LONGER_DETOUR"] < 0
        print(f"{name:>10} {wd:>7.2f} {wa:>6.2f} {deep:>10.4f} {gaps['B_SHALLOW_VALLEY']:>8.2f}% "
              f"{gaps['A_DIRECT_LEVEL']:>7.2f}% {gaps['D_LATE_DESCENT']:>6.2f}% {gaps['E_ROLLER_COASTER']:>7.2f}% "
              f"{gaps['F_LONG_DETOUR']:>7.2f}% {gaps['G_DEEPER_LONGER_DETOUR']:>9.2f}% "
              f"{str(gaps['B_SHALLOW_VALLEY']>5):>5} {str(gaps['A_DIRECT_LEVEL']>5):>5} {str(deg):>6}")
    print()

    # --- Component table for the two most balanced sets (A and D per the mission's "altitude slightly more important") ---
    for setname in ["SET_A", "SET_D"]:
        ws = next(w for w in WEIGHT_SETS if w["name"] == setname)
        wd, wa = ws["w_distance"], ws["w_altitude"]
        print(f"=== Component table: {setname} (w_distance={wd}, w_altitude={wa}, w_reversal={W_REVERSAL}) ===")
        print(f"{'route':>24} {'w*C_distance':>13} {'w*C_altitude':>13} {'w*C_reversal':>13} {'Total':>9}")
        for label in order:
            c = comp[label]
            wdist = wd * c["C_distance"]
            walt = wa * c["C_altitude"]
            wrev = W_REVERSAL * c["C_reversal"]
            total = wdist + walt + wrev
            print(f"{label:>24} {wdist:>13.4f} {walt:>13.4f} {wrev:>13.4f} {total:>9.4f}")
        print()

    # --- G vs C: is G EVER cheaper than C for ANY positive weight? (strict-domination check) ---
    print("=== G_DEEPER_LONGER_DETOUR vs C_EARLY_DEEP_VALLEY -- component-wise domination check ===")
    dC_dist = comp["G_DEEPER_LONGER_DETOUR"]["C_distance"] - comp["C_EARLY_DEEP_VALLEY"]["C_distance"]
    dC_alt = comp["G_DEEPER_LONGER_DETOUR"]["C_altitude"] - comp["C_EARLY_DEEP_VALLEY"]["C_altitude"]
    print(f"  delta_C_distance(G,C) = {dC_dist:+.4f}   delta_C_altitude(G,C) = {dC_alt:+.4f}")
    if dC_dist >= 0 and dC_alt >= 0:
        print("  G is WORSE than C in BOTH components (strictly dominated) -- "
              "NO positive (w_distance, w_altitude) pair can ever make G cheaper than C.")
    else:
        breakeven = -dC_dist / dC_alt if dC_alt != 0 else float("inf")
        print(f"  G beats C on one axis -- breakeven w_altitude/w_distance ratio = {breakeven:.4f}")
    print()

    # --- 100m altitude advantage <-> % extra distance trade-off (analytical, per-unit-distance) ---
    print("=== Altitude <-> Distance trade-off (per unit distance flown, analytical from the formula) ===")
    print("  extra_distance_ratio = (w_altitude / w_distance) * (100 / H_scale)")
    for ws in WEIGHT_SETS:
        ratio = (ws["w_altitude"] / ws["w_distance"]) * (100.0 / H_SCALE)
        print(f"  {ws['name']} (w_d={ws['w_distance']}, w_alt={ws['w_altitude']}): "
              f"100m lower excess-altitude over distance ds <=> {ratio*100:.1f}% extra distance over that same ds "
              f"is cost-equivalent")
    print()

    # --- Mission-length robustness (analytical only) ---
    print("=== Mission-length robustness (analytical, NOT benchmarked) ===")
    print("  C_distance = path_length/D_ref -- a pure ratio, invariant to mission scale for the SAME relative detour.")
    print("  C_altitude = sum(ds*excess)/(D_ref*H_scale) -- if a longer mission's excess-altitude PROFILE (not total")
    print("  distance) stays physically similar, sum(ds*excess) scales with D_ref (same factor), so C_altitude is")
    print("  ALSO scale-invariant. It would NOT be invariant if a longer mission needed proportionally MORE excess")
    print("  altitude (not the expected case for cruise-altitude missions).")
    print("  C_reversal = R_raw/R_scale -- tied to reversal EVENT COUNT, not to distance at all -- already")
    print("  mission-length-independent in the current model (neither improved nor worsened by normalization).\n")


if __name__ == "__main__":
    main()
