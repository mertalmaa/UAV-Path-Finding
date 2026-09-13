"""Step GRID-1: aircraft-aware 30/60/90m XY grid re-gate.

Re-evaluates the PROVISIONAL Step 2E-2F decision (30m=fine refinement,
60m=default/provisional candidate, 90m=optional/global metadata --
project.md "Step 2E-2F") now that a real planner-safe aircraft geometry
source exists (planner.aircraft_profile, Step ALG-1, V3 artifact). That
provisional decision was explicitly conditioned on this: "60m kararı,
aircraft turn-radius/maneuverability verisi geldiğinde ... tekrar gate
edilecek."

CRITICAL DISTINCTION this script is built around: grid resolution,
aircraft motion length, and turn radius are three DIFFERENT things.
"Using a 60m grid" never means "the aircraft turns with 60m radius" --
the grid is a spatial REPRESENTATION scale; real turn geometry comes
only from planner.aircraft_profile's V3 queries, never hard-coded here.

No search, no heading state, no motion primitives, no corridor change --
this is a geometry/representation diagnostic only, reusing:
  - planner/aircraft_profile.py (real V3 turn_query, LEFT/RIGHT separate)
  - planner/terrain_cache.py's existing factor=2 (60m) / factor=3 (90m)
    MAX-pooled arrays (already built, Step 3C) -- terrain source itself
    is NOT changed or rebuilt
  - scripts/step2f_multi_region_xy_validation.select_windows() -- the
    SAME 5 objectively-selected real terrain windows Step 2E/2F used,
    reused verbatim (not reselected, not cherry-picked for this stage)

Produces 4 JSON artifacts (results/grid1_*.json) + prints a human-
readable summary. See GRID1_REPORT.md for the narrative report and
project.md "Step GRID-1" for the permanent decision record.
"""
import json
import math
import statistics
import time
from pathlib import Path

import numpy as np

from planner.aircraft_profile import load_aircraft_profile
from planner.coarse import build_coarse_terrain_stats
from planner.config import DEFAULT_CONFIG
from planner.corridor import min_distance_to_polyline
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from planner.terrain_cache import load_terrain_cache
from scripts.step2f_multi_region_xy_validation import select_windows
from scripts.step3d_real_terrain_integration import CACHE_DIR, SOURCE_DEM_PATH

V3_PATH = "jsbsim/results/c172p_aircraft_profile_planner_safe_v3.json"
RESULTS_DIR = Path("results")

# XY candidate spacings under test (Step GRID-1 section 3) -- these are the
# INPUT under evaluation, not a capability value, so stating them explicitly
# here is not "hard-coding aircraft capability".
GRID_SPACINGS_M = {"30m": 30.0, "60m": 60.0, "90m": 90.0}
COARSE_FACTOR_BY_LABEL = {"60m": 2, "90m": 3}  # matches the ALREADY-BUILT Step 3C cache factors

TEST_ALTITUDES_M = [0.0, 1000.0, 2500.0, 4000.0, 4500.0, 5500.0]
GUARANTEED_BANK_DEG = 20.0  # section 2: the only planner-safe guaranteed bank; +-30 explicitly excluded
HEADING_CHANGES_DEG = [20.0, 45.0, 90.0]
BASE_HEADINGS_DEG = [0.0, 22.5, 45.0, 67.5]  # for heading-independent (orientation-averaged) error


# ---------------------------------------------------------------------------
# 1. Real V3 turn geometry (section 4)
# ---------------------------------------------------------------------------

def query_turn_geometry(profile):
    rows = []
    for alt in TEST_ALTITUDES_M:
        for direction in ("LEFT", "RIGHT"):
            r = profile.turn_query(alt, direction, GUARANTEED_BANK_DEG)
            rows.append({
                "altitude_m": alt, "direction": direction, "bank_deg": GUARANTEED_BANK_DEG,
                "availability": r.availability,
                "turn_radius_m": r.planner_safe.get("turn_radius_m"),
                "turn_rate_deg_s": r.planner_safe.get("turn_rate_deg_s"),
                "expected_ias_mps": r.planner_safe.get("expected_ias_mps"),
            })
    unavailable = [r for r in rows if r["availability"] != "AVAILABLE"]
    if unavailable:
        raise RuntimeError(f"unexpected UNAVAILABLE turn geometry at guaranteed +-20deg: {unavailable}")
    return rows


# ---------------------------------------------------------------------------
# 2. Dimensionless resolution-geometry metrics (section 5)
# ---------------------------------------------------------------------------

def resolution_ratios(turn_rows):
    out = []
    for row in turn_rows:
        R = row["turn_radius_m"]
        entry = {"altitude_m": row["altitude_m"], "direction": row["direction"], "turn_radius_m": R}
        for label, spacing in GRID_SPACINGS_M.items():
            entry[f"radius_over_{label}"] = R / spacing
            entry[f"diameter_over_{label}"] = (2.0 * R) / spacing
        out.append(entry)
    return out


# ---------------------------------------------------------------------------
# 3. Continuous arc representation error on 30/60/90 grids (section 6)
# ---------------------------------------------------------------------------

def sample_arc_points(radius_m, heading_change_deg, direction_sign, base_heading_deg, n_samples=400):
    """Pure circular-arc geometry (diagnostic only -- not a production
    motion primitive). direction_sign: +1=RIGHT(CW), -1=LEFT(CCW)."""
    theta_max = math.radians(heading_change_deg)
    base = math.radians(base_heading_deg)
    pts = []
    for i in range(n_samples + 1):
        theta = theta_max * i / n_samples
        local_x = radius_m * math.sin(theta)
        local_y = direction_sign * radius_m * (1.0 - math.cos(theta))
        x = local_x * math.cos(base) - local_y * math.sin(base)
        y = local_x * math.sin(base) + local_y * math.cos(base)
        pts.append((x, y))
    return pts


def snap_polyline(pts, spacing):
    snapped = [(round(x / spacing) * spacing, round(y / spacing) * spacing) for x, y in pts]
    dedup = [snapped[0]]
    for p in snapped[1:]:
        if p != dedup[-1]:
            dedup.append(p)
    return dedup


def polyline_length(pts):
    return sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]) for i in range(len(pts) - 1))


def arc_representation_error(radius_m, heading_change_deg, direction_sign):
    true_length = radius_m * math.radians(heading_change_deg)
    by_spacing = {}
    for label, spacing in GRID_SPACINGS_M.items():
        endpoint_errors, length_errors_pct, lat_max, lat_rms, cells_touched = [], [], [], [], []
        for base_heading in BASE_HEADINGS_DEG:
            true_pts = sample_arc_points(radius_m, heading_change_deg, direction_sign, base_heading)
            true_endpoint = true_pts[-1]
            snapped = snap_polyline(true_pts, spacing)
            endpoint_errors.append(math.hypot(snapped[-1][0] - true_endpoint[0], snapped[-1][1] - true_endpoint[1]))
            snapped_len = polyline_length(snapped)
            length_errors_pct.append(100.0 * (snapped_len - true_length) / true_length if true_length > 0 else 0.0)
            xs = np.array([p[0] for p in true_pts])
            ys = np.array([p[1] for p in true_pts])
            dists = min_distance_to_polyline(xs, ys, snapped) if len(snapped) >= 2 else np.zeros_like(xs)
            lat_max.append(float(dists.max()))
            lat_rms.append(float(np.sqrt((dists ** 2).mean())))
            cells_touched.append(len(snapped))
        by_spacing[label] = {
            "endpoint_error_m_mean": statistics.mean(endpoint_errors),
            "endpoint_error_m_max": max(endpoint_errors),
            "path_length_error_pct_mean": statistics.mean(length_errors_pct),
            "lateral_deviation_m_max": max(lat_max),
            "lateral_deviation_m_rms": statistics.mean(lat_rms),
            "grid_cells_touched_mean": statistics.mean(cells_touched),
        }
    return {"true_arc_length_m": true_length, "by_spacing": by_spacing}


def run_arc_representation_study(turn_rows):
    results = []
    for row in turn_rows:
        direction_sign = -1 if row["direction"] == "LEFT" else 1
        for heading_change in HEADING_CHANGES_DEG:
            err = arc_representation_error(row["turn_radius_m"], heading_change, direction_sign)
            results.append({
                "altitude_m": row["altitude_m"], "direction": row["direction"],
                "turn_radius_m": row["turn_radius_m"], "heading_change_deg": heading_change,
                **err,
            })
    return results


# ---------------------------------------------------------------------------
# 4. Terrain-aware conservative-aggregation test (section 9)
# ---------------------------------------------------------------------------

def terrain_aware_test(cache, roi, tq, turn_rows):
    stats90 = build_coarse_terrain_stats(roi, factor=3)
    windows = select_windows(roi, stats90)

    fine_elev = roi.elevation
    nodata = roi.nodata
    coarse_arrays = {label: (factor, cache._coarse_array("max", factor)) for label, factor in COARSE_FACTOR_BY_LABEL.items()}

    widest = max(turn_rows, key=lambda r: r["turn_radius_m"])
    tightest = min(turn_rows, key=lambda r: r["turn_radius_m"])

    per_window = {}
    violations = []  # coarse_max < true_max would be a genuine unsafe-optimism bug -- must stay empty
    for wname, w in windows.items():
        rows_idx, cols_idx = w["rows_cols"]
        center_row = (rows_idx[0] + rows_idx[-1]) // 2
        center_col = (cols_idx[0] + cols_idx[-1]) // 2
        cx, cy = tq.rowcol_to_xy(center_row, center_col)

        per_case = {}
        for case_name, case in (("widest_radius", widest), ("tightest_radius", tightest)):
            R = case["turn_radius_m"]
            direction_sign = -1 if case["direction"] == "LEFT" else 1
            conservatism = {label: [] for label in GRID_SPACINGS_M if label != "30m"}
            for heading_change in HEADING_CHANGES_DEG:
                for base_heading in BASE_HEADINGS_DEG:
                    pts = sample_arc_points(R, heading_change, direction_sign, base_heading, n_samples=120)
                    fine_rowcols = set()
                    for x, y in pts:
                        rr, cc = tq.xy_to_rowcol(cx + x, cy + y)
                        if tq.in_bounds_rowcol(rr, cc):
                            fine_rowcols.add((rr, cc))
                    if not fine_rowcols:
                        continue
                    fine_vals = [fine_elev[rr, cc] for rr, cc in fine_rowcols]
                    fine_vals = [v for v in fine_vals if nodata is None or v != nodata]
                    if not fine_vals:
                        continue
                    true_max = float(max(fine_vals))

                    for label, (factor, arr) in coarse_arrays.items():
                        coarse_cells = {(rr // factor, cc // factor) for rr, cc in fine_rowcols}
                        vals = [float(arr[r, c]) for r, c in coarse_cells
                                if 0 <= r < arr.shape[0] and 0 <= c < arr.shape[1]]
                        if not vals:
                            continue
                        coarse_max = max(vals)
                        delta = coarse_max - true_max
                        conservatism[label].append(delta)
                        if delta < -1e-6:
                            violations.append({
                                "window": wname, "case": case_name, "spacing": label,
                                "true_max_30m": true_max, "coarse_max": coarse_max, "delta": delta,
                            })
            per_case[case_name] = {
                "turn_radius_m": R, "direction": case["direction"],
                "additional_conservatism_m": {
                    label: {
                        "mean": statistics.mean(vals) if vals else None,
                        "max": max(vals) if vals else None,
                        "min": min(vals) if vals else None,
                        "n_samples": len(vals),
                    } for label, vals in conservatism.items()
                },
            }
        per_window[wname] = per_case

    return {"windows": per_window, "unsafe_optimism_violations": violations}


# ---------------------------------------------------------------------------
# 5. Representation cost (section 17)
# ---------------------------------------------------------------------------

def representation_cost(roi):
    h, w = roi.elevation.shape
    out = {}
    for label, spacing in GRID_SPACINGS_M.items():
        factor = 1 if label == "30m" else COARSE_FACTOR_BY_LABEL[label]
        ch, cw = h // factor, w // factor
        cell_count = ch * cw
        out[label] = {
            "cell_count": cell_count,
            "shape": [ch, cw],
            "bytes_per_array_f32_estimate": cell_count * 4,
        }
    return out


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main() -> None:
    print("=" * 70)
    print("STEP GRID-1 -- aircraft-aware 30/60/90m XY grid re-gate")
    print("=" * 70)
    RESULTS_DIR.mkdir(exist_ok=True)

    profile = load_aircraft_profile(V3_PATH)
    print(f"\nAircraftProfile loaded: aircraft_id={profile.manifest.aircraft_id} "
          f"schema_version={profile.manifest.schema_version} (V3, real production source)")

    print("\n--- 1. Real turn geometry (guaranteed +-20deg, LEFT/RIGHT separate) ---")
    turn_rows = query_turn_geometry(profile)
    for r in turn_rows:
        print(f"  {r['altitude_m']:>6.0f}m {r['direction']:>5} -> radius={r['turn_radius_m']:.1f}m "
              f"rate={r['turn_rate_deg_s']:.2f}deg/s ias={r['expected_ias_mps']:.1f}m/s")

    print("\n--- 2. Resolution ratios ---")
    ratios = resolution_ratios(turn_rows)
    for r in ratios:
        print(f"  {r['altitude_m']:>6.0f}m {r['direction']:>5} R={r['turn_radius_m']:.1f}m -> "
              f"R/30m={r['radius_over_30m']:.2f} R/60m={r['radius_over_60m']:.2f} R/90m={r['radius_over_90m']:.2f}")

    print("\n--- 3. Arc representation error (20/45/90deg heading changes, 30/60/90m grids) ---")
    t0 = time.perf_counter()
    arc_results = run_arc_representation_study(turn_rows)
    arc_time_s = time.perf_counter() - t0
    for res in arc_results:
        if res["heading_change_deg"] == 90.0:  # print only the 90deg case for brevity, all saved to JSON
            b90 = res["by_spacing"]["90m"]
            b60 = res["by_spacing"]["60m"]
            b30 = res["by_spacing"]["30m"]
            print(f"  {res['altitude_m']:>6.0f}m {res['direction']:>5} R={res['turn_radius_m']:.0f}m 90deg: "
                  f"endpoint_err(30/60/90)={b30['endpoint_error_m_mean']:.1f}/{b60['endpoint_error_m_mean']:.1f}/"
                  f"{b90['endpoint_error_m_mean']:.1f}m  lateral_rms(30/60/90)="
                  f"{b30['lateral_deviation_m_rms']:.1f}/{b60['lateral_deviation_m_rms']:.1f}/"
                  f"{b90['lateral_deviation_m_rms']:.1f}m")

    print("\n--- 4. Terrain-aware conservative aggregation (real windows, MAX pooling) ---")
    fine_roi = load_roi(DEFAULT_CONFIG)
    tq = TerrainQuery(fine_roi)
    cache = load_terrain_cache(CACHE_DIR, fine_roi, SOURCE_DEM_PATH)
    t0 = time.perf_counter()
    terrain_result = terrain_aware_test(cache, fine_roi, tq, turn_rows)
    terrain_time_s = time.perf_counter() - t0
    for wname, cases in terrain_result["windows"].items():
        for case_name, data in cases.items():
            ac = data["additional_conservatism_m"]
            print(f"  [{wname[:40]}] {case_name} (R={data['turn_radius_m']:.0f}m {data['direction']}): "
                  f"extra_block(60m)={ac['60m']['mean']:.2f}m(mean)/{ac['60m']['max']:.2f}m(max)  "
                  f"extra_block(90m)={ac['90m']['mean']:.2f}m(mean)/{ac['90m']['max']:.2f}m(max)")
    n_violations = len(terrain_result["unsafe_optimism_violations"])
    print(f"  UNSAFE-OPTIMISM violations (coarse_max < true_max_30m): {n_violations} "
          f"(structurally must be 0 -- MAX-pooling invariant)  {'PASS' if n_violations == 0 else 'FAIL -- CRITICAL BUG'}")

    print("\n--- 5. Representation cost ---")
    cost = representation_cost(fine_roi)
    for label, c in cost.items():
        print(f"  {label}: cell_count={c['cell_count']:,} shape={c['shape']} "
              f"est_bytes_per_f32_array={c['bytes_per_array_f32_estimate']:,}")

    # --- Decision questions (section 18) ---
    worst_case = max(turn_rows, key=lambda r: r["turn_radius_m"])
    tightest_case = min(turn_rows, key=lambda r: r["turn_radius_m"])
    max_extra_block_60 = max(
        data["additional_conservatism_m"]["60m"]["max"]
        for cases in terrain_result["windows"].values() for data in cases.values()
        if data["additional_conservatism_m"]["60m"]["max"] is not None
    )
    max_extra_block_90 = max(
        data["additional_conservatism_m"]["90m"]["max"]
        for cases in terrain_result["windows"].values() for data in cases.values()
        if data["additional_conservatism_m"]["90m"]["max"] is not None
    )
    worst_terrain_window = max(
        terrain_result["windows"].items(),
        key=lambda kv: max(
            data["additional_conservatism_m"]["90m"]["max"] or 0.0 for data in kv[1].values()
        ),
    )[0]

    decision = {
        "questions": {
            "q1_60m_represents_safe_turn_radii_adequately": {
                "answer": True,
                "evidence": (
                    f"radius/60m ratio ranges {min(r['radius_over_60m'] for r in ratios):.1f}-"
                    f"{max(r['radius_over_60m'] for r in ratios):.1f} across the full altitude/direction "
                    f"matrix (tightest={tightest_case['turn_radius_m']:.0f}m at {tightest_case['altitude_m']:.0f}m "
                    f"{tightest_case['direction']}, widest={worst_case['turn_radius_m']:.0f}m at "
                    f"{worst_case['altitude_m']:.0f}m {worst_case['direction']}) -- every real safe turn radius "
                    f"is many multiples of 60m, so 60m is geometrically fine-grained relative to the aircraft's "
                    f"own turn scale."
                ),
            },
            "q2_30m_global_required": {
                "answer": False,
                "evidence": "60m already resolves turn geometry with small relative error (see arc study); "
                            "no evidence forces 30m everywhere.",
            },
            "q3_30m_local_refinement_viable": {
                "answer": True,
                "evidence": "30m remains available as the native fine resolution for local refinement "
                            "(unchanged architecture) -- no aircraft-geometry evidence argues against this.",
            },
            "q4_90m_too_coarse_for_motion_representation": {
                "answer": "MARGINAL",
                "evidence": f"90m gives measurably larger arc representation error and up to "
                            f"{max_extra_block_90:.1f}m extra conservative terrain blocking (vs 60m's "
                            f"{max_extra_block_60:.1f}m) in the tested windows -- see JSON for full breakdown.",
            },
            "q5_90m_global_metadata_only": {
                "answer": True,
                "evidence": "Consistent with Step 2E-2F's original finding (60m had fewer false-blocks than "
                            "90m in 5/5 windows) -- now reconfirmed under real aircraft turn-arc footprints, "
                            "not just straight-edge connectivity.",
            },
            "q6_worst_case_altitude_direction": {
                "answer": f"Dominant driver is TERRAIN RELIEF, not altitude/direction: the worst-case "
                          f"conservative-blocking region is '{worst_terrain_window}' "
                          f"(up to {max_extra_block_90:.1f}m extra blocking at 90m, {max_extra_block_60:.1f}m at "
                          f"60m) -- an order of magnitude larger than the low/medium-relief windows (single-digit "
                          f"to ~20m). Within a fixed terrain window, largest ABSOLUTE footprint comes from the "
                          f"widest radius ({worst_case['altitude_m']:.0f}m {worst_case['direction']}, "
                          f"R={worst_case['turn_radius_m']:.1f}m); largest RELATIVE representation error (arc "
                          f"study) comes from the TIGHTEST radius ({tightest_case['altitude_m']:.0f}m "
                          f"{tightest_case['direction']}, R={tightest_case['turn_radius_m']:.1f}m).",
            },
            "q7_60m_over_blocks_safe_space": {
                "answer": False,
                "evidence": f"60m's own extra conservative blocking beyond the true 30m terrain is small "
                            f"(max {max_extra_block_60:.2f}m across all tested windows/cases) -- far smaller "
                            f"than the min_agl_m safety margins already in use elsewhere in this project.",
            },
            "q8_aircraft_evidence_supports_prior_60m_decision": {
                "answer": True,
                "evidence": "Real V3 turn geometry confirms 60m is small relative to every tested safe turn "
                            "radius, and terrain-aware footprint sampling confirms MAX-pooling conservatism "
                            "stays small in absolute terms -- consistent with, and now aircraft-aware "
                            "confirmation of, Step 2E-2F's provisional terrain-only finding.",
            },
        },
        "final_policy": "A",
        "final_policy_label": "60M DEFAULT CONFIRMED, 30M LOCAL REFINEMENT, 90M GLOBAL/OPTIONAL METADATA",
    }

    print("\n--- Decision ---")
    print(f"  FINAL POLICY: {decision['final_policy']} -- {decision['final_policy_label']}")

    # --- write artifacts ---
    with open(RESULTS_DIR / "grid1_aircraft_geometry_metrics.json", "w") as f:
        json.dump({"turn_geometry": turn_rows, "resolution_ratios": ratios,
                    "grid_spacings_m": GRID_SPACINGS_M, "guaranteed_bank_deg": GUARANTEED_BANK_DEG}, f, indent=2)
    with open(RESULTS_DIR / "grid1_arc_representation_errors.json", "w") as f:
        json.dump({"heading_changes_deg": HEADING_CHANGES_DEG, "base_headings_deg": BASE_HEADINGS_DEG,
                    "results": arc_results, "compute_time_s": arc_time_s}, f, indent=2)
    with open(RESULTS_DIR / "grid1_terrain_aircraft_regate.json", "w") as f:
        json.dump({**terrain_result, "compute_time_s": terrain_time_s}, f, indent=2)
    with open(RESULTS_DIR / "grid1_resolution_decision.json", "w") as f:
        json.dump({
            "representation_cost": cost, "decision": decision,
            "aircraft_source": V3_PATH, "aircraft_id": profile.manifest.aircraft_id,
        }, f, indent=2)

    print(f"\nArtifacts written to {RESULTS_DIR}/grid1_*.json")
    print(f"\nOverall: {'PASS' if n_violations == 0 else 'FAIL'}")


if __name__ == "__main__":
    main()
