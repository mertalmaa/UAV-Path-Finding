"""Stage 28: Cost Function Validation / Route Ranking Test.

Pure diagnostic. NO search, NO tuning, NO formula/weight change. Six candidate
routes over the SAME real 6.48km Aladaglar corridor (start row=48,col=276 ->
goal row=264,col=276, aircraft_msl=3760, w_MSL=0.63 for this run only) are
built directly as (row,col,z_index) waypoint sequences, expressible under the
existing 24-primitive set, and evaluated with the exact production safety +
cost authorities:

    validate_and_cost_path()  -- per-edge evaluate_primitive() safety +
                                  compute_edge_cost() via the real
                                  (vertical_trend, trend_age_bucket) state
                                  machine (_next_trend_and_bucket) -- never a
                                  separate/approximate formula.
    decompose_cost()          -- G/M/R breakdown using the SAME
                                  _altitude_scaled / _path_vertical_reversal_metrics
                                  helpers the search itself uses.

The ONE exception to "no astar_search()" is candidate B (SHALLOW VALLEY):
per the spec's own preference, this re-runs Stage 26's exact epsilon=1.10
call (identical parameters, identical code path) purely to retrieve the
concrete path array -- that array was never persisted to disk, only its
summary metrics were printed. This is not new search exploration; nothing
about search performance, epsilon, or tuning is touched.
"""
import dataclasses
import math

from planner.astar import (
    _path_vertical_reversal_metrics, astar_search, msl_to_z_index,
    validate_and_cost_path,
)
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from scripts.benchmark_long_valley import last_climb_start_distance, verify_path_safety
from scripts.calibrate_low_msl_behavior import compute_vertical_profile_metrics, decompose_cost

START_ROW, START_COL = 48, 276
GOAL_ROW, GOAL_COL = 264, 276
AIRCRAFT_MSL = 3760.0
W_MSL = 0.63


def build_direct_level(z0):
    return [(r, START_COL, z0) for r in range(START_ROW, GOAL_ROW + 1)]


def build_early_deep_valley(z0, depth_steps=18, dwell_rows=68, start_buffer_rows=4):
    """start_buffer_rows: a small mandatory level-flight buffer right after
    START before descent may begin -- NOT a design choice, a HARD safety
    requirement. The terrain peak at row=50 (~3556.8m, 2 rows south of
    START) demands MSL>=3756.8 for 200m AGL; descending immediately at
    row=48 interpolates the aircraft down to ~3750m by row=50, clipping
    that peak (AGL~193.2m < 200m -- confirmed INVALID via
    validate_and_cost_path before this buffer was added). 4 rows (120m) of
    level flight is the empirically smallest safe buffer found (0 rows:
    INVALID, 4 rows: VALID) -- taken from actual terrain, not assumed."""
    row, z = START_ROW, z0
    path = [(row, START_COL, z)]
    for _ in range(start_buffer_rows):
        row += 1
        path.append((row, START_COL, z))
    for _ in range(depth_steps):
        row += 4
        z -= 1
        path.append((row, START_COL, z))
    for _ in range(dwell_rows):
        row += 1
        path.append((row, START_COL, z))
    for _ in range(depth_steps):
        row += 4
        z += 1
        path.append((row, START_COL, z))
    assert row == GOAL_ROW and z == z0, (row, z)
    return path


def build_late_descent(z0, depth_steps=18, level_high_rows=72, dwell_rows=0):
    row, z = START_ROW, z0
    path = [(row, START_COL, z)]
    for _ in range(level_high_rows):
        row += 1
        path.append((row, START_COL, z))
    for _ in range(depth_steps):
        row += 4
        z -= 1
        path.append((row, START_COL, z))
    for _ in range(dwell_rows):
        row += 1
        path.append((row, START_COL, z))
    for _ in range(depth_steps):
        row += 4
        z += 1
        path.append((row, START_COL, z))
    assert row == GOAL_ROW and z == z0, (row, z)
    return path


def build_roller_coaster(z0, buffer_rows=72, pairs=18):
    row, z = START_ROW, z0
    path = [(row, START_COL, z)]
    for _ in range(buffer_rows):
        row += 1
        path.append((row, START_COL, z))
    for _ in range(pairs):
        row += 4
        z -= 1
        path.append((row, START_COL, z))
        row += 4
        z += 1
        path.append((row, START_COL, z))
    assert row == GOAL_ROW and z == z0, (row, z)
    return path


def build_long_detour(z0, depth_steps=18, dwell_pairs=34, start_buffer_rows=4):
    """Same start_buffer_rows safety requirement as build_early_deep_valley --
    see its docstring. dwell_pairs reduced from 36 to 34 (68 rows, matching
    C's dwell budget) to keep total rows at 216 after adding the buffer."""
    row, col, z = START_ROW, START_COL, z0
    path = [(row, col, z)]
    for _ in range(start_buffer_rows):
        row += 1
        path.append((row, col, z))
    for _ in range(depth_steps):
        row += 4
        z -= 1
        path.append((row, col, z))
    for _ in range(dwell_pairs):
        row += 1
        col += 1
        path.append((row, col, z))
        row += 1
        col -= 1
        path.append((row, col, z))
    for _ in range(depth_steps):
        row += 4
        z += 1
        path.append((row, col, z))
    assert row == GOAL_ROW and col == START_COL and z == z0, (row, col, z)
    return path


def get_stage26_eps110_path(cfg, primitives, tq, start, goal, min_search, max_search):
    """One-time retrieval of Stage 26's already-completed epsilon=1.10 run
    (identical call shape to scripts/benchmark_weighted_astar_epsilon_sweep.py
    run_one) -- its concrete path was never saved, only printed metrics were.
    Not a new search / not new tuning."""
    direct_level_path = build_direct_level(msl_to_z_index(AIRCRAFT_MSL, cfg))
    ok, incumbent_cost = validate_and_cost_path(direct_level_path, primitives, tq, cfg)
    assert ok

    result = astar_search(
        start, goal, tq, min_search_altitude_msl=min_search, max_search_altitude_msl=max_search,
        config=cfg, primitives=primitives, max_expansions=30_000,
        use_primitive_cache=True, use_dominance_pruning=False,
        use_msl_lower_bound_heuristic=True, use_vertical_reachability_heuristic=False,
        use_incumbent_pruning=True, initial_incumbent_cost=incumbent_cost,
        initial_incumbent_path=direct_level_path,
        epsilon_search=1.10, target_suboptimality=1.05,
    )
    first_found = result.first_solution_cost < math.inf
    report_path = result.path if (result.success and result.path) else (
        result.incumbent_path if first_found else []
    )
    print(f"  [Stage26 eps=1.10 retrieval] status={result.status} first_solution_found={first_found} "
          f"final_incumbent_cost={result.final_incumbent_cost:.2f} path_len={len(report_path)}")
    return report_path


def evaluate_candidate(label, path, primitives, tq, cfg):
    ok, prod_cost = validate_and_cost_path(path, primitives, tq, cfg)
    if not ok:
        print(f"\n### {label}: INVALID -- rejected by validate_and_cost_path() (edge safety or no matching primitive) ###")
        return None

    decomp = decompose_cost(path, primitives, tq, cfg)
    recombined = decomp["G"] + cfg.msl_cost_weight * decomp["M"] + decomp["R"]
    profile = compute_vertical_profile_metrics(path, primitives, tq, cfg)
    rev = _path_vertical_reversal_metrics(path, primitives, cfg)
    safety = verify_path_safety(path, primitives, tq, cfg)
    last_climb = last_climb_start_distance(path, primitives, cfg)

    row = {
        "label": label,
        "valid": True,
        "total_cost": prod_cost,
        "recombined_cost": recombined,
        "match": abs(prod_cost - recombined) < 1e-6,
        "G": decomp["G"], "M": decomp["M"], "R": decomp["R"],
        "geom_len": profile["total_horizontal_distance_m"] if profile else float("nan"),
        "avg_msl": None,
        "min_msl": profile["min_path_msl"] if profile else float("nan"),
        "max_msl": None,
        "total_climb": None,
        "total_descent": None,
        "reversals": rev["total_vertical_reversal_count"],
        "reversal_penalty": rev["total_reversal_penalty"],
        "low_dwell_ratio": profile["low_msl_dwell_ratio"] if profile else float("nan"),
        "low_dwell_dist": profile["low_msl_dwell_distance_m"] if profile else float("nan"),
        "first_descent_m": profile["first_descent_distance_m"] if profile else None,
        "last_climb_start_m": last_climb,
        "min_agl": safety.get("min_agl", float("nan")),
        "max_angle_deg": safety.get("max_angle_deg", float("nan")),
        "safety_ok": safety["ok"],
    }

    # total climb/descent + avg/max MSL via a lightweight pass (reuse existing helper)
    from planner.astar import _path_altitude_metrics
    alt = _path_altitude_metrics(path, tq, cfg)
    row["avg_msl"] = alt["average_aircraft_msl"]
    row["max_msl"] = alt["maximum_aircraft_msl"]
    row["total_climb"] = alt["total_climb_m"]
    row["total_descent"] = alt["total_descent_m"]

    print(f"\n### {label} ###")
    print(f"  SAFETY: {'PASS' if safety['ok'] else 'FAIL -- ' + safety.get('reason','')} "
          f"(min_AGL={row['min_agl']:.1f}m, max_flight_path_angle={row['max_angle_deg']:.2f} deg)")
    print(f"  TOTAL COST (production) = {prod_cost:.2f}   |  G + w*M + R = {recombined:.2f}  "
          f"(match={row['match']})")
    print(f"  decomposition: G(geometric)={decomp['G']:.2f}  M(MSL, raw)={decomp['M']:.4f}  "
          f"w*M={cfg.msl_cost_weight*decomp['M']:.2f}  R(reversal)={decomp['R']:.2f}")
    print(f"  geometric_path_length={row['geom_len']:.1f}m  avg_MSL={row['avg_msl']:.1f}  "
          f"min_MSL={row['min_msl']:.1f}  max_MSL={row['max_msl']:.1f}")
    print(f"  total_climb={row['total_climb']:.1f}m  total_descent={row['total_descent']:.1f}m  "
          f"vertical_reversals={row['reversals']}  reversal_penalty={row['reversal_penalty']:.2f}")
    print(f"  low_MSL_dwell_distance={row['low_dwell_dist']:.1f}m  ratio={row['low_dwell_ratio']:.3f}  "
          f"first_descent_at={row['first_descent_m']}m  last_climb_start_at={row['last_climb_start_m']}m")

    return row


def main() -> None:
    cfg = dataclasses.replace(DEFAULT_CONFIG, msl_cost_weight=W_MSL)
    primitives = build_primitive_set(cfg)

    roi = load_roi(cfg)
    tq = TerrainQuery(roi)

    z0 = msl_to_z_index(AIRCRAFT_MSL, cfg)
    start, goal = (START_ROW, START_COL, z0), (GOAL_ROW, GOAL_COL, z0)

    seg = roi.elevation[START_ROW:GOAL_ROW + 1, START_COL]
    seg_min = float(seg.min())
    min_search = math.ceil((seg_min + cfg.min_agl_m) / cfg.z_step_m) * cfg.z_step_m
    max_search = AIRCRAFT_MSL + 20.0

    print(f"w_MSL={W_MSL} (this diagnostic run only, config default unchanged)")
    print(f"start=({START_ROW},{START_COL}) goal=({GOAL_ROW},{GOAL_COL}) aircraft_msl={AIRCRAFT_MSL}")
    print(f"search bounds used only for candidate B's retrieval: [{min_search},{max_search}]")
    print()

    candidates = {}

    candidates["A_DIRECT_LEVEL"] = build_direct_level(z0)
    candidates["C_EARLY_DEEP_VALLEY"] = build_early_deep_valley(z0)
    candidates["D_LATE_DESCENT"] = build_late_descent(z0)
    candidates["E_ROLLER_COASTER"] = build_roller_coaster(z0)
    candidates["F_LONG_DETOUR"] = build_long_detour(z0)

    print("=== Retrieving candidate B (Stage 26 epsilon=1.10 solution path) ===")
    b_path = get_stage26_eps110_path(cfg, primitives, tq, start, goal, min_search, max_search)
    if b_path:
        candidates["B_SHALLOW_VALLEY"] = b_path
    print()

    results = {}
    order = ["A_DIRECT_LEVEL", "B_SHALLOW_VALLEY", "C_EARLY_DEEP_VALLEY", "D_LATE_DESCENT",
             "E_ROLLER_COASTER", "F_LONG_DETOUR"]
    for label in order:
        if label not in candidates:
            print(f"\n### {label}: NOT AVAILABLE (construction/retrieval failed) ###")
            continue
        r = evaluate_candidate(label, candidates[label], primitives, tq, cfg)
        if r:
            results[label] = r

    print("\n\n=== SUMMARY TABLE ===")
    cols = ["label", "total_cost", "G", "M", "R", "geom_len", "min_msl", "avg_msl",
            "climb", "descent", "reversals", "rev_pen", "min_agl", "max_ang", "dwell_ratio"]
    widths = {"label": 20, "total_cost": 11, "G": 9, "M": 8, "R": 8, "geom_len": 9, "min_msl": 8,
              "avg_msl": 8, "climb": 7, "descent": 8, "reversals": 6, "rev_pen": 8, "min_agl": 8,
              "max_ang": 8, "dwell_ratio": 11}
    print("".join(f"{c:>{widths[c]}}" for c in cols))
    for label, r in results.items():
        vals = {
            "label": label, "total_cost": f"{r['total_cost']:.2f}", "G": f"{r['G']:.1f}",
            "M": f"{r['M']:.3f}", "R": f"{r['R']:.2f}", "geom_len": f"{r['geom_len']:.1f}",
            "min_msl": f"{r['min_msl']:.1f}", "avg_msl": f"{r['avg_msl']:.1f}",
            "climb": f"{r['total_climb']:.1f}", "descent": f"{r['total_descent']:.1f}",
            "reversals": r["reversals"], "rev_pen": f"{r['reversal_penalty']:.2f}",
            "min_agl": f"{r['min_agl']:.1f}", "max_ang": f"{r['max_angle_deg']:.2f}",
            "dwell_ratio": f"{r['low_dwell_ratio']:.3f}",
        }
        print("".join(f"{str(vals[c]):>{widths[c]}}" for c in cols))

    if "A_DIRECT_LEVEL" in results and "C_EARLY_DEEP_VALLEY" in results and "B_SHALLOW_VALLEY" in results:
        a, b, c = results["A_DIRECT_LEVEL"], results["B_SHALLOW_VALLEY"], results["C_EARLY_DEEP_VALLEY"]
        print("\n=== A vs B vs C ===")
        print(f"  A (direct level)      total_cost={a['total_cost']:.2f}")
        print(f"  B (shallow valley)    total_cost={b['total_cost']:.2f}  vs A: {(1 - b['total_cost']/a['total_cost'])*100:+.2f}%")
        print(f"  C (early deep valley) total_cost={c['total_cost']:.2f}  vs A: {(1 - c['total_cost']/a['total_cost'])*100:+.2f}%  vs B: {(1 - c['total_cost']/b['total_cost'])*100:+.2f}%")

    if "C_EARLY_DEEP_VALLEY" in results and "D_LATE_DESCENT" in results:
        c, d = results["C_EARLY_DEEP_VALLEY"], results["D_LATE_DESCENT"]
        print("\n=== C (early) vs D (late) ===")
        print(f"  C total_cost={c['total_cost']:.2f}  G={c['G']:.1f} M={c['M']:.3f} R={c['R']:.2f}")
        print(f"  D total_cost={d['total_cost']:.2f}  G={d['G']:.1f} M={d['M']:.3f} R={d['R']:.2f}")
        print(f"  same G: {abs(c['G']-d['G'])<1e-6}  same total_climb/descent: "
              f"{abs(c['total_climb']-d['total_climb'])<1e-6 and abs(c['total_descent']-d['total_descent'])<1e-6}  "
              f"same min_MSL: {abs(c['min_msl']-d['min_msl'])<1e-6}")
        print(f"  cost difference (D-C) = {d['total_cost']-c['total_cost']:.2f}  "
              f"({(d['total_cost']/c['total_cost']-1)*100:+.2f}%)")

    if "C_EARLY_DEEP_VALLEY" in results and "E_ROLLER_COASTER" in results:
        c, e = results["C_EARLY_DEEP_VALLEY"], results["E_ROLLER_COASTER"]
        print("\n=== C (smooth) vs E (roller-coaster) ===")
        print(f"  C: total_climb={c['total_climb']:.1f} total_descent={c['total_descent']:.1f} "
              f"reversals={c['reversals']} reversal_penalty={c['reversal_penalty']:.2f} total_cost={c['total_cost']:.2f}")
        print(f"  E: total_climb={e['total_climb']:.1f} total_descent={e['total_descent']:.1f} "
              f"reversals={e['reversals']} reversal_penalty={e['reversal_penalty']:.2f} total_cost={e['total_cost']:.2f}")

    if "C_EARLY_DEEP_VALLEY" in results and "F_LONG_DETOUR" in results:
        c, f = results["C_EARLY_DEEP_VALLEY"], results["F_LONG_DETOUR"]
        print("\n=== C (compact) vs F (long detour) ===")
        print(f"  C: geom_len={c['geom_len']:.1f} G={c['G']:.1f} M={c['M']:.3f} total_cost={c['total_cost']:.2f}")
        print(f"  F: geom_len={f['geom_len']:.1f} G={f['G']:.1f} M={f['M']:.3f} total_cost={f['total_cost']:.2f}")
        extra_g = f['G'] - c['G']
        print(f"  extra geometric distance cost (delta_G) = {extra_g:.2f}   "
              f"delta_M*w = {(f['M']-c['M'])*W_MSL:.2f}   delta_total = {f['total_cost']-c['total_cost']:.2f}")


if __name__ == "__main__":
    main()
