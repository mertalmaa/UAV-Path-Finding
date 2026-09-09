"""Stage 9: cost tuning / sensitivity analysis for msl_cost_weight and
vertical_cost_weight. No new cost terms, no formula changes -- this only
runs the existing A* + cost model across a parameter grid and reports
physically-interpretable metrics (NOT total_weighted_cost, which isn't
comparable across different weights -- see module docstring notes below
each table).

Scenario A: low-MSL trade-off (same corridor as the low-MSL stage) -- one
    single-step dive is available or not.
Scenario B: vertical-motion trade-off -- a wider altitude band (5 z-steps)
    over a longer corridor, so "how much to dive" is a real gradient, not
    a yes/no choice.
Real ROI: same idea on the real 10x10 km Aladaglar DEM, in a spot with a
    genuine ~165 m valley the low-MSL preference can actually exploit.
    Only 4 representative weight combinations -- the real DEM makes the
    full 16-grid too slow for a prototype tuning pass (see note below).
Section 9: search-altitude normalization sensitivity -- same physical
    scenario, same weights, only min/max_search_altitude_msl changed.

This produces prototype tuning *candidates*, not a claimed-correct
physical weight -- see the printed analysis at the end.
"""
import csv
import dataclasses
import math
import time

import numpy as np
from affine import Affine

from planner.astar import astar_search, msl_to_z_index
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set
from planner.roi import ROIData, load_roi
from planner.terrain import TerrainQuery

NODATA = -9999.0
MSL_WEIGHTS = (0.0, 0.25, 0.5, 1.0)
VERTICAL_WEIGHTS = (0.0, 0.5, 1.0, 2.0)
CSV_PATH = "tuning_results.csv"

FIELDNAMES = [
    "scenario", "msl_cost_weight", "vertical_cost_weight", "status", "runtime_ms",
    "expanded_nodes", "path_state_count", "geometric_path_length_m", "total_weighted_cost",
    "average_msl_m", "minimum_msl_m", "maximum_msl_m", "total_climb_m", "total_descent_m",
    "total_vertical_motion_m", "minimum_observed_agl_m",
]


def make_roi(elevation: np.ndarray, nodata: float = NODATA, res: float = 30.0) -> ROIData:
    height, width = elevation.shape
    transform = Affine(res, 0.0, 0.0, 0.0, -res, height * res)
    return ROIData(
        elevation=elevation.astype(np.float32), transform=transform, crs="EPSG:32636",
        width=width, height=height, bounds=(0.0, 0.0, width * res, height * res),
        resolution=(res, res), nodata=nodata,
    )


def run_one(scenario, w_msl, w_vertical, start, goal, tq, min_alt, max_alt, primitives, base_cfg, max_expansions=None):
    cfg = dataclasses.replace(base_cfg, msl_cost_weight=w_msl, vertical_cost_weight=w_vertical)
    t0 = time.perf_counter()
    r = astar_search(start, goal, tq, min_search_altitude_msl=min_alt, max_search_altitude_msl=max_alt,
                      config=cfg, primitives=primitives, max_expansions=max_expansions)
    runtime_ms = (time.perf_counter() - t0) * 1000.0
    row = {
        "scenario": scenario, "msl_cost_weight": w_msl, "vertical_cost_weight": w_vertical,
        "status": r.status, "runtime_ms": round(runtime_ms, 2),
        "expanded_nodes": r.expanded_nodes, "path_state_count": len(r.path),
        "geometric_path_length_m": round(r.geometric_path_length, 2) if r.success else "",
        "total_weighted_cost": round(r.total_cost, 2) if r.success else "",
        "average_msl_m": round(r.average_aircraft_msl, 2) if r.success else "",
        "minimum_msl_m": round(r.minimum_aircraft_msl, 2) if r.success else "",
        "maximum_msl_m": round(r.maximum_aircraft_msl, 2) if r.success else "",
        "total_climb_m": round(r.total_climb_m, 2) if r.success else "",
        "total_descent_m": round(r.total_descent_m, 2) if r.success else "",
        "total_vertical_motion_m": round(r.total_vertical_motion_m, 2) if r.success else "",
        "minimum_observed_agl_m": round(r.minimum_observed_agl, 2) if r.success else "",
    }
    return row


def print_table(rows) -> None:
    cols = ["msl_cost_weight", "vertical_cost_weight", "status", "geometric_path_length_m",
            "average_msl_m", "maximum_msl_m", "total_climb_m", "total_descent_m",
            "total_vertical_motion_m", "minimum_observed_agl_m", "expanded_nodes", "runtime_ms"]
    widths = {"msl_cost_weight": 6, "vertical_cost_weight": 6, "status": 9, "geometric_path_length_m": 10,
              "average_msl_m": 9, "maximum_msl_m": 9, "total_climb_m": 7, "total_descent_m": 8,
              "total_vertical_motion_m": 9, "minimum_observed_agl_m": 8, "expanded_nodes": 9, "runtime_ms": 10}
    print("".join(f"{c:>{widths[c]}}" for c in cols))
    for row in rows:
        print("".join(f"{str(row[c]):>{widths[c]}}" for c in cols))


def scenario_a(cfg, primitives, all_rows) -> bool:
    print("=== Scenario A: low-MSL trade-off (narrow 1-step dive corridor) ===")
    flat = np.full((10, 45), 1000.0)
    roi = make_roi(flat)
    tq = TerrainQuery(roi)
    z0 = msl_to_z_index(1320.0, cfg)
    start, goal = (5, 2, z0), (5, 42, z0)

    rows = []
    ok = True
    for w_msl in MSL_WEIGHTS:
        for w_v in VERTICAL_WEIGHTS:
            row = run_one("A_low_msl", w_msl, w_v, start, goal, tq, 1300.0, 1320.0, primitives, cfg)
            rows.append(row)
            all_rows.append(row)
            if row["status"] == "success":
                ok = ok and row["minimum_observed_agl_m"] >= cfg.min_agl_m - 1e-6
            else:
                ok = False
    print_table(rows)
    print(f"  all AGL safe (>= {cfg.min_agl_m}m): {ok}")
    return ok


def scenario_b(cfg, primitives, all_rows) -> bool:
    print()
    print("=== Scenario B: vertical-motion trade-off (5-step dive gradient) ===")
    flat = np.full((3, 55), 1000.0)
    roi = make_roi(flat)
    tq = TerrainQuery(roi)
    z0 = msl_to_z_index(1400.0, cfg)
    start, goal = (1, 2, z0), (1, 52, z0)

    rows = []
    ok = True
    for w_msl in MSL_WEIGHTS:
        for w_v in VERTICAL_WEIGHTS:
            row = run_one("B_vertical", w_msl, w_v, start, goal, tq, 1300.0, 1400.0, primitives, cfg,
                           max_expansions=100_000)
            rows.append(row)
            all_rows.append(row)
            if row["status"] == "success":
                ok = ok and row["minimum_observed_agl_m"] >= cfg.min_agl_m - 1e-6
            else:
                ok = False
    print_table(rows)
    print(f"  all AGL safe (>= {cfg.min_agl_m}m): {ok}")
    return ok


def real_roi_scenario(cfg, primitives, all_rows) -> bool:
    print()
    print("=== Real Aladaglar ROI: 4 representative combinations ===")
    print("  PROTOTYPE test scenario -- start/goal/altitudes below are derived from this")
    print("  ROI's own local terrain + min_agl_m, NOT a real mission altitude.")

    roi = load_roi(cfg)
    tq = TerrainQuery(roi)

    row_idx, c_start, c_goal = 80, 90, 128
    seg = roi.elevation[row_idx, c_start:c_goal + 1]
    seg_min, seg_max = float(seg.min()), float(seg.max())
    cruise_msl = math.ceil((seg_max + cfg.min_agl_m) / cfg.z_step_m) * cfg.z_step_m + 20.0  # +1 step margin
    min_search = math.ceil((seg_min + cfg.min_agl_m) / cfg.z_step_m) * cfg.z_step_m
    max_search = cruise_msl + 20.0

    z0 = msl_to_z_index(cruise_msl, cfg)
    start, goal = (row_idx, c_start, z0), (row_idx, c_goal, z0)
    print(f"  segment terrain: min={seg_min:.1f}m max={seg_max:.1f}m (a real ~{seg_max - seg_min:.0f}m valley)")
    print(f"  start=(row={row_idx},col={c_start}) goal=(row={row_idx},col={c_goal}) "
          f"distance={(c_goal - c_start) * cfg.xy_resolution_m:.0f}m")
    print(f"  cruise_msl={cruise_msl:.0f}m, search bounds=[{min_search:.0f},{max_search:.0f}]m "
          f"(all z-grid aligned, {cfg.z_step_m}m step)")

    combos = [(0.0, 0.0), (0.25, 1.0), (0.5, 1.0), (1.0, 2.0)]
    rows = []
    ok = True
    for w_msl, w_v in combos:
        row = run_one("C_real_roi", w_msl, w_v, start, goal, tq, min_search, max_search, primitives, cfg,
                       max_expansions=50_000)
        rows.append(row)
        all_rows.append(row)
        if row["status"] == "success":
            ok = ok and row["minimum_observed_agl_m"] >= cfg.min_agl_m - 1e-6
        else:
            ok = False
    print_table(rows)
    print(f"  all AGL safe (>= {cfg.min_agl_m}m): {ok}")
    return ok


def normalization_sensitivity(cfg, primitives) -> bool:
    print()
    print("=== Section 9: search-altitude normalization sensitivity (narrow vs wide) ===")
    print("  Same physical start/goal/terrain/weights -- only min/max_search_altitude_msl differ.")
    flat = np.full((3, 55), 1000.0)
    roi = make_roi(flat)
    tq = TerrainQuery(roi)
    z0 = msl_to_z_index(1400.0, cfg)
    start, goal = (1, 2, z0), (1, 52, z0)
    c = dataclasses.replace(cfg, msl_cost_weight=0.25, vertical_cost_weight=1.0)

    results = {}
    for label, lo, hi in (("narrow [1300,1400]", 1300.0, 1400.0), ("wide [1300,2400]", 1300.0, 2400.0)):
        t0 = time.perf_counter()
        r = astar_search(start, goal, tq, min_search_altitude_msl=lo, max_search_altitude_msl=hi,
                          config=c, primitives=primitives, max_expansions=100_000)
        dt = (time.perf_counter() - t0) * 1000.0
        results[label] = r
        print(f"  {label}: status={r.status} avg_msl={r.average_aircraft_msl:.1f} "
              f"min_msl={r.minimum_aircraft_msl:.1f} vertical_motion={r.total_vertical_motion_m:.1f} "
              f"expanded={r.expanded_nodes} runtime={dt:.1f}ms")

    narrow, wide = results["narrow [1300,1400]"], results["wide [1300,2400]"]
    ok = narrow.success and wide.success
    ok = ok and narrow.total_vertical_motion_m > wide.total_vertical_motion_m  # wide range dilutes the incentive
    ok = ok and narrow.average_aircraft_msl < wide.average_aircraft_msl
    print(f"  same physical altitude difference, weaker MSL-penalty effect under the wide range: {ok}")
    return ok


def main() -> None:
    cfg = DEFAULT_CONFIG
    primitives = build_primitive_set(cfg)
    all_rows = []

    ok_a = scenario_a(cfg, primitives, all_rows)
    ok_b = scenario_b(cfg, primitives, all_rows)
    ok_c = real_roi_scenario(cfg, primitives, all_rows)
    ok_d = normalization_sensitivity(cfg, primitives)

    with open(CSV_PATH, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(all_rows)
    print()
    print(f"Wrote {CSV_PATH} ({len(all_rows)} rows)")

    print()
    print(f"Overall: {'ALL PASS' if (ok_a and ok_b and ok_c and ok_d) else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
