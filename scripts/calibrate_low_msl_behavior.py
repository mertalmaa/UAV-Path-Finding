"""Stage 18: w_MSL calibration + vertical path profile analysis.

Pure analysis -- no architecture change. Current system used exactly as-is:
primitive cache ON, MSL-aware admissible heuristic ON, dominance pruning
OFF (isolate this stage's effect), spacing-sensitive reversal model
unchanged, min AGL / 10 deg limits unchanged, cost formula unchanged.

Tests w_MSL in {0.25 (baseline), 0.32 (conservative), 0.63 (balanced),
1.01 (aggressive)} -- these are Stage 13's analytical break-even-derived
candidates, re-labelled per this stage's own naming.

For every resulting path: standard metrics (already in SearchResult) +
new vertical-profile metrics (first_descent_distance_m,
deepest_point_distance_from_start_m, low_msl_dwell_distance_m/_ratio,
distance_below_start_minus_20m) + a G/M/R cost decomposition computed
from the SAME production functions the search itself uses (never a
separate approximate formula).
"""
import dataclasses
import math
import time

import numpy as np
from affine import Affine

from planner.astar import (
    _altitude_scaled, _path_altitude_metrics, _path_vertical_reversal_metrics,
    astar_search, msl_to_z_index, state_to_xyz,
)
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set
from planner.roi import ROIData, load_roi
from planner.terrain import TerrainQuery

NODATA = -9999.0
W_LABELS = [(0.25, "baseline"), (0.32, "conservative"), (0.63, "balanced"), (1.01, "aggressive")]


def make_roi(elevation: np.ndarray, nodata: float = NODATA, res: float = 30.0) -> ROIData:
    height, width = elevation.shape
    transform = Affine(res, 0.0, 0.0, 0.0, -res, height * res)
    return ROIData(
        elevation=elevation.astype(np.float32), transform=transform, crs="EPSG:32636",
        width=width, height=height, bounds=(0.0, 0.0, width * res, height * res),
        resolution=(res, res), nodata=nodata,
    )


def _distance_below_threshold(edges, threshold):
    total = 0.0
    for a1, a2, horiz in edges:
        if a1 <= threshold and a2 <= threshold:
            total += horiz
        elif a1 <= threshold < a2:
            total += horiz * ((threshold - a1) / (a2 - a1))
        elif a2 <= threshold < a1:
            total += horiz * ((a1 - threshold) / (a1 - a2))
    return total


def compute_vertical_profile_metrics(path, primitives, terrain, config):
    """Walks each edge using the SAME primitive.horizontal_distance_m the
    search itself costs edges with (no separate approximate geometry)."""
    if len(path) < 2:
        return None

    by_delta = {(p.drow, p.dcol, round(p.dz_m / config.z_step_m)): p for p in primitives}
    xyz = [state_to_xyz(s, terrain, config) for s in path]
    altitudes = [p[2] for p in xyz]
    start_msl = altitudes[0]
    min_path_msl = min(altitudes)  # exact: altitude is piecewise-linear per edge, so extrema are at waypoints

    cum_dist = [0.0]
    edges = []  # (start_alt, end_alt, horizontal_distance, primitive_type)
    for i, ((r1, c1, z1), (r2, c2, z2)) in enumerate(zip(path, path[1:])):
        prim = by_delta.get((r2 - r1, c2 - c1, z2 - z1))
        horiz = prim.horizontal_distance_m
        cum_dist.append(cum_dist[-1] + horiz)
        edges.append((altitudes[i], altitudes[i + 1], horiz, prim.primitive_type))

    total_horizontal = cum_dist[-1]

    first_descent_distance = None
    for i, (a1, a2, horiz, ptype) in enumerate(edges):
        if ptype == "descent":
            first_descent_distance = cum_dist[i]
            break

    deepest_point_distance = cum_dist[altitudes.index(min_path_msl)]

    low_dwell = _distance_below_threshold([(a1, a2, h) for a1, a2, h, _ in edges], min_path_msl + 20.0)
    below_start = _distance_below_threshold([(a1, a2, h) for a1, a2, h, _ in edges], start_msl - 20.0)

    return {
        "first_descent_distance_m": first_descent_distance,
        "deepest_point_distance_from_start_m": deepest_point_distance,
        "low_msl_dwell_distance_m": low_dwell,
        "low_msl_dwell_ratio": low_dwell / total_horizontal if total_horizontal > 0 else float("nan"),
        "distance_below_start_minus_20m": below_start,
        "min_path_msl": min_path_msl,
        "total_horizontal_distance_m": total_horizontal,
    }


def decompose_cost(path, primitives, terrain, config):
    """G/M/R using the exact production functions -- G and R are already
    computed by the search's own helpers; M is the one piece not already
    exposed as a standalone total, computed here with _altitude_scaled
    (the same function compute_edge_cost calls), not a separate formula."""
    alt_metrics = _path_altitude_metrics(path, terrain, config)
    rev_metrics = _path_vertical_reversal_metrics(path, primitives, config)

    by_delta = {(p.drow, p.dcol, round(p.dz_m / config.z_step_m)): p for p in primitives}
    xyz = [state_to_xyz(s, terrain, config) for s in path]
    M = 0.0
    for i, ((r1, c1, z1), (r2, c2, z2)) in enumerate(zip(path, path[1:])):
        prim = by_delta.get((r2 - r1, c2 - c1, z2 - z1))
        geometric_cost = math.sqrt(prim.horizontal_distance_m ** 2 + prim.dz_m ** 2)
        mean_alt = (xyz[i][2] + xyz[i + 1][2]) / 2.0
        M += geometric_cost * _altitude_scaled(mean_alt, config)

    return {"G": alt_metrics["geometric_path_length"], "M": M, "R": rev_metrics["total_reversal_penalty"]}


def ascii_profile(profile_metrics, total_horizontal) -> str:
    if profile_metrics is None or profile_metrics["first_descent_distance_m"] is None:
        return "START ------------------------------------------- GOAL   (no descent at all)"
    frac_descent = profile_metrics["first_descent_distance_m"] / total_horizontal if total_horizontal else 0.0
    dwell_ratio = profile_metrics["low_msl_dwell_ratio"]
    width = 45
    d = max(1, min(width - 2, round(frac_descent * width)))
    low_span = max(1, min(width - d - 1, round(dwell_ratio * width)))
    return "START " + "-" * d + "\\" + "_" * low_span + "/" + "-" * max(0, width - d - low_span - 1) + " GOAL"


def report(label, w_msl, result, profile_metrics, decomp) -> dict:
    row = {
        "w_MSL": w_msl, "label": label, "status": result.status,
        "runtime_s": round(result.runtime_s, 2), "expanded": result.expanded_nodes,
        "max_open": result.max_open_size,
        "geom_len": round(result.geometric_path_length, 2) if result.success else None,
        "total_cost": round(result.total_cost, 2) if result.success else None,
        "avg_msl": round(result.average_aircraft_msl, 2) if result.success else None,
        "min_msl": round(result.minimum_aircraft_msl, 2) if result.success else None,
        "max_msl": round(result.maximum_aircraft_msl, 2) if result.success else None,
        "min_agl": round(result.minimum_observed_agl, 2) if result.success else None,
        "climb": round(result.total_climb_m, 2) if result.success else None,
        "descent": round(result.total_descent_m, 2) if result.success else None,
        "reversals": result.total_vertical_reversal_count,
        "reversal_penalty": round(result.total_reversal_penalty, 3) if result.success else None,
        "multiplier": round(result.heuristic_cost_multiplier, 4),
        "G": round(decomp["G"], 2) if decomp else None,
        "M": round(decomp["M"], 2) if decomp else None,
        "R": round(decomp["R"], 2) if decomp else None,
        "first_descent_m": round(profile_metrics["first_descent_distance_m"], 1)
                            if profile_metrics and profile_metrics["first_descent_distance_m"] is not None else None,
        "deepest_at_m": round(profile_metrics["deepest_point_distance_from_start_m"], 1) if profile_metrics else None,
        "low_dwell_ratio": round(profile_metrics["low_msl_dwell_ratio"], 3) if profile_metrics else None,
        "below_start20_m": round(profile_metrics["distance_below_start_minus_20m"], 1) if profile_metrics else None,
    }
    print(f"  w_MSL={w_msl} ({label}): status={result.status} runtime={result.runtime_s:.2f}s "
          f"expanded={result.expanded_nodes} max_open={result.max_open_size}")
    if result.success:
        print(f"    geom_len={row['geom_len']} total_cost={row['total_cost']} avg_MSL={row['avg_msl']} "
              f"min_MSL={row['min_msl']} max_MSL={row['max_msl']} min_AGL={row['min_agl']}")
        print(f"    climb={row['climb']} descent={row['descent']} reversals={row['reversals']} "
              f"reversal_penalty={row['reversal_penalty']}")
        print(f"    G={row['G']} M={row['M']} R={row['R']}  (G+w*M+R = "
              f"{row['G'] + w_msl * row['M'] + row['R']:.2f} vs total_cost={row['total_cost']})")
        print(f"    first_descent_m={row['first_descent_m']} deepest_at_m={row['deepest_at_m']} "
              f"low_dwell_ratio={row['low_dwell_ratio']} below_start-20m_m={row['below_start20_m']}")
    if result.runtime_s > 15.0 or result.status == "search_limit_reached":
        print(f"    !! FLAG: runtime={result.runtime_s:.1f}s status={result.status} -- see note in summary")
    return row


def synthetic_scenario(cfg, primitives) -> list:
    print("=== Synthetic controlled profile scenario ===")
    print("  Same start/goal altitude, wide z-range (5 steps) over a 2100m corridor -- "
          "multiple valid vertical profiles are physically available.")
    width, height = 70, 3
    flat = np.full((height, width), 1000.0)  # flat & safe everywhere across the whole altitude band
    tq = TerrainQuery(make_roi(flat))
    z0 = msl_to_z_index(1400.0, cfg)
    start, goal = (1, 2, z0), (1, 65, z0)

    rows = []
    for w_msl, label in W_LABELS:
        c = dataclasses.replace(cfg, msl_cost_weight=w_msl)
        result = astar_search(start, goal, tq, min_search_altitude_msl=1300.0, max_search_altitude_msl=1400.0,
                               config=c, primitives=primitives, max_expansions=100_000,
                               use_primitive_cache=True, use_dominance_pruning=False,
                               use_msl_lower_bound_heuristic=True)
        profile = compute_vertical_profile_metrics(result.path, primitives, tq, c) if result.success else None
        decomp = decompose_cost(result.path, primitives, tq, c) if result.success else None
        row = report(label, w_msl, result, profile, decomp)
        rows.append(row)
        if result.success:
            print(f"    profile: {ascii_profile(profile, profile['total_horizontal_distance_m'])}")
        print()
    return rows


def real_aladaglar_scenario(cfg, primitives) -> list:
    print("=== Real Aladaglar ROI (same scenario as Stage 13/17) ===")
    roi = load_roi(cfg)
    tq = TerrainQuery(roi)
    ROW, COL_START, COL_GOAL, CRUISE_MSL = 80, 90, 128, 2840.0
    seg = roi.elevation[ROW, COL_START:COL_GOAL + 1]
    min_search = math.ceil((float(seg.min()) + cfg.min_agl_m) / cfg.z_step_m) * cfg.z_step_m
    max_search = CRUISE_MSL + 20.0
    z0 = round(CRUISE_MSL / cfg.z_step_m)
    start, goal = (ROW, COL_START, z0), (ROW, COL_GOAL, z0)
    print(f"  start=({ROW},{COL_START}) goal=({ROW},{COL_GOAL}) cruise={CRUISE_MSL}m "
          f"bounds=[{min_search},{max_search}]  (PROTOTYPE scenario, not a real mission)")

    rows = []
    for w_msl, label in W_LABELS:
        c = dataclasses.replace(cfg, msl_cost_weight=w_msl)
        t0 = time.perf_counter()
        result = astar_search(start, goal, tq, min_search_altitude_msl=min_search, max_search_altitude_msl=max_search,
                               config=c, primitives=primitives, max_expansions=30_000,
                               use_primitive_cache=True, use_dominance_pruning=False,
                               use_msl_lower_bound_heuristic=True)
        wall = time.perf_counter() - t0
        profile = compute_vertical_profile_metrics(result.path, primitives, tq, c) if result.success else None
        decomp = decompose_cost(result.path, primitives, tq, c) if result.success else None
        row = report(label, w_msl, result, profile, decomp)
        row["wall_s"] = round(wall, 2)
        rows.append(row)
        if result.success:
            print(f"    profile: {ascii_profile(profile, profile['total_horizontal_distance_m'])}")
        print()
    return rows


def print_table(rows, title) -> None:
    print(f"=== {title} ===")
    cols = ["w_MSL", "label", "status", "runtime_s", "expanded", "geom_len", "total_cost", "avg_msl",
            "min_msl", "first_descent_m", "low_dwell_ratio", "climb", "descent", "reversals", "min_agl"]
    widths = {"w_MSL": 7, "label": 13, "status": 18, "runtime_s": 9, "expanded": 9, "geom_len": 9,
              "total_cost": 11, "avg_msl": 8, "min_msl": 8, "first_descent_m": 14, "low_dwell_ratio": 15,
              "climb": 7, "descent": 8, "reversals": 10, "min_agl": 8}
    print("".join(f"{c:>{widths[c]}}" for c in cols))
    for row in rows:
        print("".join(f"{str(row.get(c, '')):>{widths[c]}}" for c in cols))
    print()


def main() -> None:
    cfg = DEFAULT_CONFIG
    primitives = build_primitive_set(cfg)

    synthetic_rows = synthetic_scenario(cfg, primitives)
    print_table(synthetic_rows, "Synthetic summary table")

    real_rows = real_aladaglar_scenario(cfg, primitives)
    print_table(real_rows, "Real Aladaglar summary table")


if __name__ == "__main__":
    main()
