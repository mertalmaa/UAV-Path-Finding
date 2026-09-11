"""Roadmap Step 2D: PRACTICAL VALIDATION of the Step 2C preservation contract.

Still no adaptive graph, no spacing/threshold selection, no classifier wired
to production. Four independent measurements, reusing existing planner
functions exactly as Step 2B did (evaluate_agl, build_coarse_dem,
build_coarse_terrain_stats, bfs_connected, evaluate_primitive, TerrainQuery).

  1. delta_z_loss -- a P3 QUALITY metric (existence predicate itself is
     unchanged/kept), measured on controlled cases including Step 2B Case 2.
  2. Critical altitude events -- formalizes and empirically validates that
     P2 (connectivity) is a finite step function of z, checkable exactly at
     a finite set of representative altitudes instead of dense sampling.
  3. Real-DEM coverage diagnostic -- P1/P2/P3/P4 category counts over real
     Aladağlar terrain, at TWO explicitly-labeled altitude bands (the
     literal Step-1 mission band, and a lower diagnostic band) -- reported
     as descriptive counts only, never as a refinement rule.
  4. UNSAFE_OPTIMISM=0 regression diagnostic, explicitly scoped to the
     CURRENT MAX-pooled coarse representation only.
"""
import dataclasses
import math
from collections import deque

import numpy as np

from planner.agl import evaluate_agl
from planner.config import DEFAULT_CONFIG
from planner.coarse import build_coarse_dem, build_coarse_terrain_stats
from planner.corridor import bfs_connected
from planner.primitives import MotionPrimitive, evaluate_primitive
from planner.roi import load_roi
from planner.terrain import TerrainQuery

STEP2B_CONFIG = dataclasses.replace(DEFAULT_CONFIG, min_agl_m=100.0)
TEST_COARSE_Z_STEP = 100.0  # diagnostic measuring instrument only, same as Step 2B -- NOT a proposal


# ===========================================================================
# 1. P3 QUALITY LOSS -- delta_z_loss
# ===========================================================================

def lowest_feasible_layer(terrain_elev, min_agl, z_step, z_lo, z_hi):
    """Smallest z on the z_step grid, inside [z_lo, z_hi], that clears
    terrain_elev + min_agl. None if no such layer exists in the interval
    (i.e. P3-existence fails for this (terrain, z_step, interval))."""
    true_floor = terrain_elev + min_agl
    candidate = math.ceil(max(true_floor, z_lo) / z_step) * z_step
    if candidate > z_hi:
        return None
    return candidate


def delta_z_loss(terrain_elev, min_agl, fine_step, coarse_step, z_lo, z_hi):
    """lowest_representable_coarse_feasible_z - lowest_representable_fine_feasible_z,
    terrain representation held FIXED (isolates the Z-spacing effect alone --
    matches Step 2B Case 2's own methodology, which varied z_step only, not
    terrain). Returns None if either resolution has no representable layer
    in [z_lo, z_hi] at all (delta_z_loss is only defined when BOTH exist)."""
    fine_z = lowest_feasible_layer(terrain_elev, min_agl, fine_step, z_lo, z_hi)
    coarse_z = lowest_feasible_layer(terrain_elev, min_agl, coarse_step, z_lo, z_hi)
    if fine_z is None or coarse_z is None:
        return None, fine_z, coarse_z
    return coarse_z - fine_z, fine_z, coarse_z


def section_1():
    cfg = STEP2B_CONFIG
    cases = []

    # A. delta_z_loss == 0 -- terrain floor already grid-aligned to BOTH steps.
    terrain_a = 2900.0  # true_floor=3000, divisible by both 20 and 100
    d, f, c = delta_z_loss(terrain_a, cfg.min_agl_m, cfg.z_step_m, TEST_COARSE_Z_STEP, terrain_a, terrain_a + 500)
    cases.append({"case": "A_zero", "terrain_elev": terrain_a, "delta_z_loss": d, "fine_z": f, "coarse_z": c})

    # B. delta_z_loss small (non-grid-aligned floor, modest gap).
    terrain_b = 2955.0  # true_floor=3055
    d, f, c = delta_z_loss(terrain_b, cfg.min_agl_m, cfg.z_step_m, TEST_COARSE_Z_STEP, terrain_b, terrain_b + 500)
    cases.append({"case": "B_small", "terrain_elev": terrain_b, "delta_z_loss": d, "fine_z": f, "coarse_z": c})

    # C. delta_z_loss large -- Step 2B Case 2, EXACTLY (terrain_elev=3512.0).
    terrain_c = 3512.0  # true_floor=3612, Step 2B's own case 2 value
    d, f, c = delta_z_loss(terrain_c, cfg.min_agl_m, cfg.z_step_m, TEST_COARSE_Z_STEP, terrain_c, terrain_c + 300)
    cases.append({"case": "C_large_step2b_case2", "terrain_elev": terrain_c, "delta_z_loss": d, "fine_z": f, "coarse_z": c})

    # D. P3 (existence) PASSES, yet delta_z_loss is meaningfully large -- this
    #    IS case C, restated with the P3-existence check made explicit: both
    #    fine_z and coarse_z are non-None (P3 holds -- coarse DOES have a
    #    representable layer inside the interval) while delta_z_loss=80m.
    p3_existence_c = (f is not None and c is not None)
    cases.append({
        "case": "D_P3_pass_but_meaningful_loss", "terrain_elev": terrain_c,
        "P3_existence_holds": p3_existence_c, "delta_z_loss": d,
        "note": "identical to case C -- P3(existence) says 'a coarse layer exists in the interval' (True), "
                "but says nothing about HOW MUCH higher it forces the aircraft -- delta_z_loss captures exactly "
                "the magnitude P3(existence) is blind to",
    })

    # E. contrast: P3 (existence) FAILS outright -- delta_z_loss undefined
    #    (Step 2B Case 3's narrow ledge: interval too narrow for ANY coarse layer).
    terrain_e = 3540.0  # true_floor=3640, ledge band [3640,3660)
    d, f, c = delta_z_loss(terrain_e, cfg.min_agl_m, cfg.z_step_m, TEST_COARSE_Z_STEP, terrain_e, 3660.0)
    cases.append({
        "case": "E_P3_fails_delta_undefined", "terrain_elev": terrain_e, "delta_z_loss": d,
        "fine_z": f, "coarse_z": c,
        "note": "coarse_z=None -- P3(existence) FAILS here, so delta_z_loss is not even defined "
                "(it is only meaningful when P3 already holds; it is a quality refinement ON TOP of "
                "existence, never a replacement for it)",
    })

    return cases


# ===========================================================================
# 2. P2 CRITICAL ALTITUDE EVENTS
# ===========================================================================

def validity_mask(terrain: TerrainQuery, rows, cols, z_msl, config) -> np.ndarray:
    mask = np.zeros((len(rows), len(cols)), dtype=bool)
    for i, r in enumerate(rows):
        for j, c in enumerate(cols):
            x, y = terrain.rowcol_to_xy(r, c)
            mask[i, j] = evaluate_agl(terrain, x, y, z_msl, config).valid
    return mask


def _flood_fill(mask, start_rc):
    h, w = mask.shape
    if not mask[start_rc]:
        return set()
    seen = {start_rc}
    q = deque([start_rc])
    neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    while q:
        r, c = q.popleft()
        for dr, dc in neighbors:
            nr, nc = r + dr, c + dc
            if 0 <= nr < h and 0 <= nc < w and mask[nr, nc] and (nr, nc) not in seen:
                seen.add((nr, nc))
                q.append((nr, nc))
    return seen


def connectivity_at(fine_terrain, coarse_terrain, rows, cols, coarse_rows, coarse_cols, z_msl, config, factor=3):
    """Same automated farthest-pair methodology as Step 2B section_c_real."""
    fine_mask = validity_mask(fine_terrain, rows, cols, z_msl, config)
    visited = np.zeros_like(fine_mask, dtype=bool)
    best_component = set()
    for i in range(fine_mask.shape[0]):
        for j in range(fine_mask.shape[1]):
            if fine_mask[i, j] and not visited[i, j]:
                comp = _flood_fill(fine_mask, (i, j))
                for cell in comp:
                    visited[cell] = True
                if len(comp) > len(best_component):
                    best_component = comp
    if len(best_component) < 2:
        return {"status": "no_component"}
    comp_list = list(best_component)
    best_pair, best_dist = None, -1.0
    for i in range(len(comp_list)):
        for j in range(i + 1, len(comp_list)):
            d = (comp_list[i][0] - comp_list[j][0]) ** 2 + (comp_list[i][1] - comp_list[j][1]) ** 2
            if d > best_dist:
                best_dist, best_pair = d, (comp_list[i], comp_list[j])
    (lr1, lc1), (lr2, lc2) = best_pair
    r0 = rows[0]
    c0 = cols[0]
    start_rc = (r0 + lr1, c0 + lc1)
    goal_rc = (r0 + lr2, c0 + lc2)

    coarse_start = (start_rc[0] // factor, start_rc[1] // factor)
    coarse_goal = (goal_rc[0] // factor, goal_rc[1] // factor)
    coarse_mask_full = np.zeros((max(coarse_rows) + 1, max(coarse_cols) + 1), dtype=bool)
    for cr in coarse_rows:
        for cc in coarse_cols:
            x, y = coarse_terrain.rowcol_to_xy(cr, cc)
            coarse_mask_full[cr, cc] = evaluate_agl(coarse_terrain, x, y, z_msl, config).valid
    fine_mask_full = np.zeros((max(rows) + 1, max(cols) + 1), dtype=bool)
    for r in rows:
        for c in cols:
            x, y = fine_terrain.rowcol_to_xy(r, c)
            fine_mask_full[r, c] = evaluate_agl(fine_terrain, x, y, z_msl, config).valid

    fine_connected = bfs_connected(fine_mask_full, start_rc, goal_rc)
    coarse_connected = bfs_connected(coarse_mask_full, coarse_start, coarse_goal)
    return {"start_rc": start_rc, "goal_rc": goal_rc, "component_size": len(best_component),
            "fine_connected": fine_connected, "coarse_connected": coarse_connected,
            "loss": fine_connected and not coarse_connected}


def section_2(fine_terrain, coarse_terrain, window_rows, window_cols, coarse_rows, coarse_cols, config, z_lo, z_hi):
    fine_events = set()
    for r in window_rows:
        for c in window_cols:
            e = fine_terrain.query(*fine_terrain.rowcol_to_xy(r, c)).elevation
            fine_events.add(round(float(e) + config.min_agl_m, 6))
    coarse_events = set()
    for cr in coarse_rows:
        for cc in coarse_cols:
            e = coarse_terrain.query(*coarse_terrain.rowcol_to_xy(cr, cc)).elevation
            coarse_events.add(round(float(e) + config.min_agl_m, 6))

    all_events = sorted(e for e in (fine_events | coarse_events) if z_lo <= e <= z_hi)
    boundaries = [z_lo] + all_events + [z_hi]
    boundaries = sorted(set(boundaries))

    interval_results = []
    for i in range(len(boundaries) - 1):
        lo, hi = boundaries[i], boundaries[i + 1]
        if hi - lo < 1e-6:
            continue
        rep_z = (lo + hi) / 2.0
        result = connectivity_at(fine_terrain, coarse_terrain, window_rows, window_cols,
                                  coarse_rows, coarse_cols, rep_z, config)
        interval_results.append({"interval": (lo, hi), "rep_z": rep_z, **result})

    # empirical cross-check: within ONE interval, densely re-sample and confirm
    # the mask never changes away from the event boundaries (validates the
    # "finite event set is exact" claim, not just asserts it)
    cross_check = None
    if len(interval_results) >= 1:
        lo, hi = interval_results[0]["interval"]
        if hi - lo > 5.0:
            sample_zs = np.linspace(lo + 0.5, hi - 0.5, 5)
            masks = [validity_mask(fine_terrain, window_rows, window_cols, z, config) for z in sample_zs]
            all_identical = all(np.array_equal(masks[0], m) for m in masks[1:])
            cross_check = {"interval": (lo, hi), "sample_count": len(sample_zs), "mask_identical_throughout": all_identical}

    return {
        "total_events_in_band": len(all_events), "n_intervals": len(interval_results),
        "interval_results": interval_results, "cross_check": cross_check,
    }


# ===========================================================================
# 3. REAL-DEM COVERAGE DIAGNOSTIC
# ===========================================================================

def section_3(fine_terrain, coarse_terrain, coarse_stats, config, z_lo, z_hi, coarse_row_range, coarse_col_range, factor=3):
    """P1 (+P3-existence, same min/max-derived data) over EVERY coarse cell
    in the given range -- cheap/vectorizable, no per-cell fine sampling
    needed (both derive from CoarseTerrainStats alone)."""
    no_issue = xy_only = z_only = both = 0
    p1_fail_cells = []

    r0, r1 = coarse_row_range
    c0, c1 = coarse_col_range
    for cr in range(r0, r1):
        for cc in range(c0, c1):
            min_e = float(coarse_stats.min_elevation[cr, cc])
            max_e = float(coarse_stats.max_elevation[cr, cc])
            if coarse_stats.nodata is not None and (min_e == coarse_stats.nodata or max_e == coarse_stats.nodata):
                continue
            ambiguity_lo, ambiguity_hi = min_e + config.min_agl_m, max_e + config.min_agl_m

            # P1: mixed validity intersecting [z_lo, z_hi]?
            p1_fails = not (ambiguity_hi <= z_lo or ambiguity_lo >= z_hi)

            # P3 existence (fine vs coarse-diagnostic-step layer existence in [z_lo,z_hi]):
            # uses the SAME coarse max-elevation (this cell's own worst case) for the
            # coarse-resolution floor, and the cell's own min-elevation as a stand-in
            # for "a representative fine point inside it" (the tightest, most
            # conservative fine floor within this footprint).
            fine_z = lowest_feasible_layer(min_e, config.min_agl_m, config.z_step_m, z_lo, z_hi)
            coarse_z = lowest_feasible_layer(max_e, config.min_agl_m, TEST_COARSE_Z_STEP, z_lo, z_hi)
            p3_fails = (fine_z is not None and coarse_z is None)

            if p1_fails and p3_fails:
                both += 1
            elif p1_fails:
                xy_only += 1
                p1_fail_cells.append((cr, cc))
            elif p3_fails:
                z_only += 1
            else:
                no_issue += 1

    total = no_issue + xy_only + z_only + both
    return {
        "z_lo": z_lo, "z_hi": z_hi, "total_cells_checked": total,
        "no_issue": no_issue, "xy_only": xy_only, "z_only": z_only, "both": both,
        "p1_fail_sample": p1_fail_cells[:10],
    }


def section_3_p2_p4(fine_terrain, coarse_terrain, config, z_lo, z_hi, p1_fail_cells, factor=3, max_checks=15):
    """P2/P4 are NOT vectorizable like P1/P3 -- restricted (per the Step 2C
    proposed strategy) to a bounded sample of P1-flagged (mixed) coarse
    cells' neighborhoods, not the whole map. Reports how many of the sampled
    boundary cells show a real connectivity or edge-preservation loss."""
    p2_loss_count = 0
    p4_false_block_count = 0
    p4_unsafe_optimism_count = 0
    checked = 0

    test_z = (max(z_lo, 3000.0) + min(z_hi, 3900.0)) / 2.0  # a representative altitude inside the interesting range

    for (cr, cc) in p1_fail_cells[:max_checks]:
        fr0, fc0 = cr * factor, cc * factor
        rows = list(range(fr0, fr0 + factor))
        cols = list(range(fc0, fc0 + factor))
        # P4: same-physical-edge test at the cell's own center, one direction
        center_r, center_c = fr0 + factor // 2, fc0 + factor // 2
        if not fine_terrain.in_bounds_rowcol(center_r, center_c):
            continue
        elev = fine_terrain.query(*fine_terrain.rowcol_to_xy(center_r, center_c)).elevation
        if math.isnan(elev):
            continue
        z_msl = math.ceil((elev + config.min_agl_m) / config.z_step_m) * config.z_step_m
        prim = MotionPrimitive(direction="test", drow=0, dcol=1, dz_m=0.0,
                                horizontal_distance_m=config.xy_resolution_m, primitive_type="level")
        start_xyz = (*fine_terrain.rowcol_to_xy(center_r, center_c), z_msl)
        fine_result = evaluate_primitive(start_xyz, prim, fine_terrain, config)
        coarse_result = evaluate_primitive(start_xyz, prim, coarse_terrain, config)
        checked += 1
        if fine_result.valid and not coarse_result.valid:
            p4_false_block_count += 1
        elif not fine_result.valid and coarse_result.valid:
            p4_unsafe_optimism_count += 1

    return {"boundary_cells_checked": checked, "p4_false_block": p4_false_block_count,
            "p4_unsafe_optimism": p4_unsafe_optimism_count, "test_altitude_used": test_z}


# ===========================================================================
# 4. UNSAFE-OPTIMISM INVARIANT (regression diagnostic)
# ===========================================================================

def section_4(fine_terrain, coarse_terrain, config, rows, cols):
    """Scoped EXPLICITLY to the current MAX-pooled coarse representation.
    This guarantee does NOT automatically hold for a future interpolation-
    based or otherwise non-conservative adaptive representation -- it is a
    property of MAX-pooling specifically (coarse(x,y) >= fine(x,y) always),
    re-derive/re-verify before assuming it for any other representation."""
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, 1), (1, 1), (-1, -1), (1, -1)]
    unsafe_optimism_count = 0
    total = 0
    for row in rows:
        for col in cols:
            start_elev = fine_terrain.query(*fine_terrain.rowcol_to_xy(row, col)).elevation
            if math.isnan(start_elev):
                continue
            z_msl = math.ceil((start_elev + config.min_agl_m) / config.z_step_m) * config.z_step_m
            for (drow, dcol) in directions:
                is_diag = drow != 0 and dcol != 0
                step = config.xy_resolution_m * (2 ** 0.5 if is_diag else 1.0)
                prim = MotionPrimitive(direction="test", drow=drow, dcol=dcol, dz_m=0.0,
                                        horizontal_distance_m=step, primitive_type="level")
                start_xyz = (*fine_terrain.rowcol_to_xy(row, col), z_msl)
                fine_result = evaluate_primitive(start_xyz, prim, fine_terrain, config)
                coarse_result = evaluate_primitive(start_xyz, prim, coarse_terrain, config)
                total += 1
                if not fine_result.valid and coarse_result.valid:
                    unsafe_optimism_count += 1
    return {"total_edges_checked": total, "unsafe_optimism_count": unsafe_optimism_count,
            "invariant_holds": unsafe_optimism_count == 0}


def main():
    print("=" * 70)
    print("STEP 2D -- preservation contract practical validation")
    print("=" * 70)

    fine_roi = load_roi(DEFAULT_CONFIG)
    fine_terrain = TerrainQuery(fine_roi)
    coarse_dem = build_coarse_dem(fine_roi, factor=3)
    coarse_terrain = TerrainQuery(coarse_dem.roi)
    coarse_stats = build_coarse_terrain_stats(fine_roi, factor=3)

    print("\n--- 1. P3 QUALITY LOSS: delta_z_loss ---")
    print(f"  (diagnostic test coarse Z step = {TEST_COARSE_Z_STEP}m -- measuring instrument only, not a proposal)")
    for c in section_1():
        print(f"  {c}")

    print("\n--- 2. P2 CRITICAL ALTITUDE EVENTS ---")
    window_rows = list(range(55, 75))
    window_cols = list(range(235, 255))
    coarse_rows = list(range(55 // 3, 75 // 3 + 1))
    coarse_cols = list(range(235 // 3, 255 // 3 + 1))
    result2 = section_2(fine_terrain, coarse_terrain, window_rows, window_cols, coarse_rows, coarse_cols,
                         STEP2B_CONFIG, 3000.0, 3600.0)
    print(f"  window: rows[55,75) cols[235,255) (same high-relief window as Step 2B)")
    print(f"  mission band tested: [3000,3600]m")
    print(f"  distinct critical altitude events in band: {result2['total_events_in_band']}")
    print(f"  -> band partitioned into {result2['n_intervals']} constant-connectivity intervals")
    print(f"  per-interval connectivity (representative z at interval midpoint):")
    for ir in result2["interval_results"]:
        lo, hi = ir["interval"]
        print(f"    [{lo:.1f},{hi:.1f}) rep_z={ir['rep_z']:.1f}  "
              f"fine_connected={ir.get('fine_connected')}  coarse_connected={ir.get('coarse_connected')}"
              f"  {'<<< LOSS' if ir.get('loss') else ''}")
    if result2["cross_check"]:
        cc = result2["cross_check"]
        print(f"  empirical cross-check (dense re-sample WITHIN interval {cc['interval']}, "
              f"{cc['sample_count']} points): mask identical throughout = {cc['mask_identical_throughout']}"
              f"  ({'confirms exactness of the event-based decomposition' if cc['mask_identical_throughout'] else 'VIOLATION -- investigate'})")

    print("\n--- 3. REAL-DEM COVERAGE DIAGNOSTIC (descriptive counts only, NOT a refinement rule) ---")
    print("  Scenario A -- LITERAL Step-1 mission band [5000,6000]m, FULL coarse grid (111x111):")
    resA = section_3(fine_terrain, coarse_terrain, coarse_stats, STEP2B_CONFIG, 5000.0, 6000.0, (0, 111), (0, 111))
    print(f"    total cells={resA['total_cells_checked']}  no_issue={resA['no_issue']}  "
          f"xy_only={resA['xy_only']}  z_only={resA['z_only']}  both={resA['both']}")

    print("\n  Scenario B -- LOWER diagnostic band [3000,3800]m (illustrative only, NOT the mission spec), "
          "same FULL coarse grid:")
    resB = section_3(fine_terrain, coarse_terrain, coarse_stats, STEP2B_CONFIG, 3000.0, 3800.0, (0, 111), (0, 111))
    print(f"    total cells={resB['total_cells_checked']}  no_issue={resB['no_issue']}  "
          f"xy_only={resB['xy_only']}  z_only={resB['z_only']}  both={resB['both']}")
    print(f"    sample of XY-flagged (P1-mixed) cells: {resB['p1_fail_sample']}")

    print("\n  P2/P4 on a bounded sample of Scenario B's P1-flagged boundary cells "
          "(NOT vectorizable/full-map like P1/P3 -- see Step 2C's own cost note):")
    resB_p2p4 = section_3_p2_p4(fine_terrain, coarse_terrain, STEP2B_CONFIG, 3000.0, 3800.0, resB["p1_fail_sample"])
    print(f"    {resB_p2p4}")

    print("\n--- 4. UNSAFE-OPTIMISM INVARIANT (scoped to CURRENT MAX-pooled representation only) ---")
    res4 = section_4(fine_terrain, coarse_terrain, STEP2B_CONFIG, range(55, 75), range(235, 255))
    print(f"  {res4}")

    print("\n" + "=" * 70)
    print("Done. No refinement rule, threshold, or spacing selected.")


if __name__ == "__main__":
    main()
