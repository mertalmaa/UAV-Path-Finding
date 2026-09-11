"""Roadmap Step 2B: controlled information-loss DIAGNOSTICS only.

Does NOT wire anything into an adaptive graph, does NOT produce a
refinement map, does NOT pick a threshold/spacing/classifier. Every
measurement below reuses EXISTING planner functions verbatim
(evaluate_agl, evaluate_primitive, evaluate_transition, TerrainQuery,
build_coarse_dem, build_coarse_terrain_stats, bfs_connected) -- nothing
under planner/ is edited or reimplemented.

Four independent measurements, matching the four requested sections:
  A. XY representation loss   -- SAME physical edge, fine terrain vs coarse
     terrain (never fine-primitive-geometry vs coarse-primitive-geometry).
  B. Node validity ambiguity  -- verify the [min_elev+min_agl, max_elev+min_agl)
     band (derived from EXISTING CoarseTerrainStats) against ground-truth
     fine classification.
  C. Connectivity loss        -- fine vs coarse validity-mask connectivity
     (bfs_connected, reused unmodified), synthetic + real.
  D. Z representation loss    -- fixed XY, only Z spacing varies, synthetic
     controlled cases (a diagnostic test spacing is used ONLY to make the
     phenomenon measurable -- this is NOT a proposed production value).
"""
import dataclasses
import math
from typing import List, Tuple

import numpy as np
from affine import Affine

from planner.agl import evaluate_agl
from planner.config import DEFAULT_CONFIG
from planner.coarse import build_coarse_dem, build_coarse_terrain_stats
from planner.corridor import bfs_connected
from planner.primitives import MotionPrimitive, evaluate_primitive
from planner.roi import ROIData, load_roi
from planner.terrain import TerrainQuery

STEP2B_CONFIG = dataclasses.replace(DEFAULT_CONFIG, min_agl_m=100.0)
assert STEP2B_CONFIG.z_step_m == DEFAULT_CONFIG.z_step_m  # not touched, per instructions

NODATA = -9999.0


def make_synthetic_roi(elevation: np.ndarray, res: float = 30.0, nodata=NODATA) -> ROIData:
    height, width = elevation.shape
    transform = Affine(res, 0.0, 500000.0, 0.0, -res, 4200000.0)
    return ROIData(
        elevation=elevation.astype(np.float32), transform=transform, crs="EPSG:32636",
        width=width, height=height, bounds=(500000.0, 4200000.0 - height * res, 500000.0 + width * res, 4200000.0),
        resolution=(res, res), nodata=nodata,
    )


# ===========================================================================
# A. XY REPRESENTATION LOSS -- same physical edge, fine terrain vs coarse
#    terrain. Primitive geometry (horizontal_distance_m, dz_m) is IDENTICAL
#    for both calls -- only the `terrain` argument to evaluate_primitive()
#    differs. This isolates terrain-representation change from primitive-
#    geometry change (the confound the instructions explicitly warned about).
# ===========================================================================

def section_a(fine_terrain: TerrainQuery, coarse_terrain: TerrainQuery, config, roi_rows, roi_cols):
    preserved, false_block, unsafe_optimism = [], [], []
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, 1), (1, 1), (-1, -1), (1, -1)]

    for row in roi_rows:
        for col in roi_cols:
            start_elev = fine_terrain.query(*fine_terrain.rowcol_to_xy(row, col)).elevation
            if math.isnan(start_elev):
                continue
            # candidate altitude: right at the fine-local clearance boundary
            # (grid-aligned, rounded UP) -- the altitude most likely to expose
            # a representation difference, neither trivially always-valid nor
            # always-invalid.
            z_msl = math.ceil((start_elev + config.min_agl_m) / config.z_step_m) * config.z_step_m

            for (drow, dcol) in directions:
                is_diag = drow != 0 and dcol != 0
                step = config.xy_resolution_m * (2 ** 0.5 if is_diag else 1.0)
                # SAME physical primitive object reused for both terrain evaluations.
                prim = MotionPrimitive(direction="test", drow=drow, dcol=dcol, dz_m=0.0,
                                        horizontal_distance_m=step, primitive_type="level")
                start_xyz = (*fine_terrain.rowcol_to_xy(row, col), z_msl)

                fine_result = evaluate_primitive(start_xyz, prim, fine_terrain, config)
                coarse_result = evaluate_primitive(start_xyz, prim, coarse_terrain, config)

                record = {
                    "row": row, "col": col, "drow": drow, "dcol": dcol, "z_msl": z_msl,
                    "start_elev": float(start_elev),
                    "fine_valid": fine_result.valid, "fine_reason": fine_result.reason,
                    "coarse_valid": coarse_result.valid, "coarse_reason": coarse_result.reason,
                }
                if fine_result.valid == coarse_result.valid:
                    preserved.append(record)
                elif fine_result.valid and not coarse_result.valid:
                    false_block.append(record)
                else:  # fine infeasible, coarse feasible
                    unsafe_optimism.append(record)

    return preserved, false_block, unsafe_optimism


# ===========================================================================
# B. NODE VALIDITY AMBIGUITY -- verify [min_elev+min_agl, max_elev+min_agl)
#    (from EXISTING CoarseTerrainStats) against ground-truth fine classification.
# ===========================================================================

def section_b(fine_terrain: TerrainQuery, coarse_stats, config, coarse_rows, coarse_cols, test_layers, factor=3):
    mismatches = []
    checked = 0
    predicted_counts = {"all_valid": 0, "all_invalid": 0, "ambiguous": 0}
    actual_counts = {"all_valid": 0, "all_invalid": 0, "ambiguous": 0}

    for cr in coarse_rows:
        for cc in coarse_cols:
            min_e = float(coarse_stats.min_elevation[cr, cc])
            max_e = float(coarse_stats.max_elevation[cr, cc])
            if coarse_stats.nodata is not None and (min_e == coarse_stats.nodata or max_e == coarse_stats.nodata):
                continue
            for z in test_layers:
                threshold = z - config.min_agl_m
                if max_e <= threshold:
                    predicted = "all_valid"
                elif min_e > threshold:
                    predicted = "all_invalid"
                else:
                    predicted = "ambiguous"
                predicted_counts[predicted] += 1

                # ground truth: every fine sub-cell inside this coarse footprint
                fine_valids = []
                for i in range(factor):
                    for j in range(factor):
                        fr, fc = cr * factor + i, cc * factor + j
                        if not fine_terrain.in_bounds_rowcol(fr, fc):
                            continue
                        x, y = fine_terrain.rowcol_to_xy(fr, fc)
                        result = evaluate_agl(fine_terrain, x, y, z, config)
                        if result.valid:
                            fine_valids.append(True)
                        elif result.reason == "below_min_agl":
                            fine_valids.append(False)
                        # NoData/out_of_bounds fine cells excluded from ground truth
                if not fine_valids:
                    continue
                if all(fine_valids):
                    actual = "all_valid"
                elif not any(fine_valids):
                    actual = "all_invalid"
                else:
                    actual = "ambiguous"
                actual_counts[actual] += 1
                checked += 1
                if actual != predicted:
                    mismatches.append((cr, cc, z, predicted, actual, min_e, max_e))

    return checked, predicted_counts, actual_counts, mismatches


# ===========================================================================
# C. CONNECTIVITY LOSS -- fine vs coarse validity-mask connectivity via the
#    EXISTING bfs_connected(), reused unmodified.
# ===========================================================================

def validity_mask(terrain: TerrainQuery, z_msl: float, config) -> np.ndarray:
    h, w = terrain.roi.height, terrain.roi.width
    mask = np.zeros((h, w), dtype=bool)
    for r in range(h):
        for c in range(w):
            x, y = terrain.rowcol_to_xy(r, c)
            result = evaluate_agl(terrain, x, y, z_msl, config)
            mask[r, c] = result.valid
    return mask


def connectivity_case(name, fine_elev, factor, z_msl, config, start_rc, goal_rc):
    fine_roi = make_synthetic_roi(fine_elev)
    fine_terrain = TerrainQuery(fine_roi)
    coarse = build_coarse_dem(fine_roi, factor=factor)
    coarse_terrain = TerrainQuery(coarse.roi)

    fine_mask = validity_mask(fine_terrain, z_msl, config)
    coarse_mask = validity_mask(coarse_terrain, z_msl, config)

    coarse_start = (start_rc[0] // factor, start_rc[1] // factor)
    coarse_goal = (goal_rc[0] // factor, goal_rc[1] // factor)

    fine_connected = bfs_connected(fine_mask, start_rc, goal_rc)
    coarse_connected = bfs_connected(coarse_mask, coarse_start, coarse_goal)

    return {
        "name": name, "fine_connected": fine_connected, "coarse_connected": coarse_connected,
        "loss": fine_connected and not coarse_connected,
        "fine_mask": fine_mask, "coarse_mask": coarse_mask,
    }


def section_c(config):
    """Two LOW basins (left cols 0-5, right cols 9-14) separated by a HIGH
    ridge (cols 6-8). Start/goal sit inside the basins (always valid,
    independent of the ridge/pass), so bfs_connected's own precondition
    (mask[start] and mask[goal] both True) is satisfied by construction --
    connectivity then depends ENTIRELY on whether some row-band of the ridge
    stays passable. h=12 rows so a 3-row pass (wide) fits inside exactly one
    coarse row-block at factor=3, and a 1-row pass (narrow) sits inside a
    coarse row-block whose other two rows stay ridge-high."""
    results = []
    h, w = 12, 15
    z_msl = 3000.0 + config.min_agl_m + 10.0
    start_rc, goal_rc = (4, 1), (4, 13)

    base = np.full((h, w), 4000.0)
    base[:, 0:6] = 3000.0   # left basin
    base[:, 9:15] = 3000.0  # right basin

    wide = base.copy()
    wide[3:6, 6:9] = 3000.0  # 3-row pass == one whole coarse row-block (rows 3,4,5)
    results.append(connectivity_case("wide_pass", wide, 3, z_msl, config, start_rc, goal_rc))

    narrow = base.copy()
    narrow[4, 6:9] = 3000.0  # 1-row pass -- rows 3 and 5 (same coarse row-block) stay ridge-high
    results.append(connectivity_case("narrow_pass", narrow, 3, z_msl, config, start_rc, goal_rc))

    blocked = base.copy()  # no pass at all
    results.append(connectivity_case("blocked_ridge", blocked, 3, z_msl, config, start_rc, goal_rc))

    return results


def _flood_fill(mask: np.ndarray, start_rc):
    """Plain 8-connected flood fill returning the set of cells in start_rc's
    component -- used only to pick a genuinely-connected real far-apart pair
    to hand to bfs_connected (reused unmodified) below, not a replacement
    for it."""
    from collections import deque
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


def section_c_real(fine_terrain: TerrainQuery, coarse_terrain: TerrainQuery, config, z_msl,
                    row_range, col_range, factor=3):
    """Automated search (not hand-picked coordinates): build the fine mask
    over the window, take its LARGEST connected component (guaranteeing a
    genuinely fine-connected pair exists), pick the two cells in it that are
    farthest apart, then ask whether their coarse cells are STILL connected
    via bfs_connected (reused unmodified) on the coarse mask. This is the
    real-DEM analogue of section_c's synthetic construction: start/goal are
    chosen to be valid by construction, so any disagreement is purely a
    fine-vs-coarse resolution effect, not an invalid-endpoint artifact."""
    r0, r1 = row_range
    c0, c1 = col_range
    fine_mask_full = validity_mask(fine_terrain, z_msl, config)
    fine_mask = fine_mask_full[r0:r1, c0:c1]

    visited = np.zeros_like(fine_mask, dtype=bool)
    best_component = set()
    for r in range(fine_mask.shape[0]):
        for c in range(fine_mask.shape[1]):
            if fine_mask[r, c] and not visited[r, c]:
                comp = _flood_fill(fine_mask, (r, c))
                for cell in comp:
                    visited[cell] = True
                if len(comp) > len(best_component):
                    best_component = comp

    if len(best_component) < 2:
        return {"z_msl": z_msl, "status": "no_component_large_enough"}

    comp_list = list(best_component)
    # farthest pair within the component, by simple max-distance scan (component
    # sizes here are small enough -- window-sized -- for an O(n^2) scan)
    best_pair, best_dist = None, -1.0
    for i in range(len(comp_list)):
        for j in range(i + 1, len(comp_list)):
            d = (comp_list[i][0] - comp_list[j][0]) ** 2 + (comp_list[i][1] - comp_list[j][1]) ** 2
            if d > best_dist:
                best_dist, best_pair = d, (comp_list[i], comp_list[j])

    (lr1, lc1), (lr2, lc2) = best_pair
    start_rc = (r0 + lr1, c0 + lc1)
    goal_rc = (r0 + lr2, c0 + lc2)

    coarse_start = (start_rc[0] // factor, start_rc[1] // factor)
    coarse_goal = (goal_rc[0] // factor, goal_rc[1] // factor)
    coarse_mask = validity_mask(coarse_terrain, z_msl, config)

    fine_connected = bfs_connected(fine_mask_full, start_rc, goal_rc)  # sanity check, should be True by construction
    coarse_connected = bfs_connected(coarse_mask, coarse_start, coarse_goal)

    return {
        "z_msl": z_msl, "start_rc": start_rc, "goal_rc": goal_rc,
        "component_size": len(best_component), "fine_connected": fine_connected,
        "coarse_connected": coarse_connected, "loss": fine_connected and not coarse_connected,
    }


# ===========================================================================
# D. Z REPRESENTATION LOSS -- fixed XY, only Z spacing varies. A diagnostic
#    test spacing (TEST_COARSE_Z_STEP) is used ONLY to make the phenomenon
#    measurable -- this is NOT a proposed production value (per instructions,
#    no spacing is being "chosen" here, just used as a measuring instrument).
# ===========================================================================

TEST_COARSE_Z_STEP = 100.0  # diagnostic instrument only


def layers_covering(terrain_elev, min_agl, z_step, ceiling):
    floor = math.ceil((terrain_elev + min_agl) / z_step) * z_step
    layers = []
    z = floor
    while z <= ceiling:
        layers.append(z)
        z += z_step
    return layers


def section_d():
    findings = []

    # 1. WIDE feasible interval -- flat terrain, feasible band is huge (well
    #    above min_agl up to the planning ceiling). Both fine (20m) and
    #    coarse (100m, diagnostic) test steps should place at least one
    #    layer inside it -- no loss expected.
    terrain_elev = 3000.0
    ceiling = 6000.0
    fine_layers = layers_covering(terrain_elev, STEP2B_CONFIG.min_agl_m, STEP2B_CONFIG.z_step_m, ceiling)
    coarse_layers = layers_covering(terrain_elev, STEP2B_CONFIG.min_agl_m, TEST_COARSE_Z_STEP, ceiling)
    findings.append({
        "case": "1_wide_feasible_interval", "terrain_elev": terrain_elev,
        "fine_first_layer": fine_layers[0], "coarse_first_layer": coarse_layers[0],
        "gap_m": coarse_layers[0] - fine_layers[0],
        "note": "both resolutions land close to the true floor; wide interval absorbs the coarser step easily",
    })

    # 2. NARROW usable margin just above the clearance floor -- does the
    #    coarser Z step force a materially higher altitude than necessary,
    #    or (worse) is there a usable low-altitude option only fine Z reaches?
    terrain_elev2 = 3512.0  # deliberately NOT grid-aligned to either step
    fine_layers2 = layers_covering(terrain_elev2, STEP2B_CONFIG.min_agl_m, STEP2B_CONFIG.z_step_m, terrain_elev2 + 300)
    coarse_layers2 = layers_covering(terrain_elev2, STEP2B_CONFIG.min_agl_m, TEST_COARSE_Z_STEP, terrain_elev2 + 300)
    findings.append({
        "case": "2_narrow_margin_above_floor", "terrain_elev": terrain_elev2,
        "true_floor": terrain_elev2 + STEP2B_CONFIG.min_agl_m,
        "fine_first_layer": fine_layers2[0], "coarse_first_layer": coarse_layers2[0],
        "gap_m": coarse_layers2[0] - fine_layers2[0],
        "fine_layers_below_coarse_first": [l for l in fine_layers2 if l < coarse_layers2[0]],
        "note": "fine layers strictly between the true floor and the coarse grid's first available layer are "
                "usable altitudes with NO coarse counterpart at all -- a genuinely lost option, not just imprecision",
    })

    # 3. Vertical climb/descent connection -- fixed XY (a single 90m-long
    #    level-horizontal-distance primitive, matching one coarse XY cell),
    #    testing whether a SMALL step (fine z_step) reaches a narrow safe
    #    ledge that a LARGE step (coarse z_step, same horizontal distance --
    #    XY geometry held fixed per instructions) jumps straight over.
    #    Synthetic terrain: flat except a narrow "ledge" band [3640,3660)
    #    that is the ONLY safe altitude band below 3700 due to an overhang-
    #    like obstruction represented here as a second (higher) local floor
    #    immediately above the ledge (modelling "a usable altitude window
    #    only reachable via a small step").
    ledge_terrain = 3540.0  # true ground floor -> true_floor = 3540+100 = 3640
    true_floor = ledge_terrain + STEP2B_CONFIG.min_agl_m  # 3640
    # the ledge is only 20m tall (3640-3660) before the model terrain effectively
    # requires the NEXT usable band far higher (3660+300=3960, simulating that
    # only a small step can land inside the narrow window at all)
    fine_step_lands_in_ledge = any(3640.0 <= l < 3660.0 for l in
                                    layers_covering(ledge_terrain, STEP2B_CONFIG.min_agl_m, STEP2B_CONFIG.z_step_m, 4000.0))
    coarse_step_lands_in_ledge = any(3640.0 <= l < 3660.0 for l in
                                      layers_covering(ledge_terrain, STEP2B_CONFIG.min_agl_m, TEST_COARSE_Z_STEP, 4000.0))
    findings.append({
        "case": "3_narrow_ledge_climb_connection", "true_floor": true_floor, "ledge_band": (3640.0, 3660.0),
        "fine_reaches_ledge": fine_step_lands_in_ledge, "coarse_reaches_ledge": coarse_step_lands_in_ledge,
        "note": "fine z_step=20 can place a node inside the 20m-wide ledge band; the 100m diagnostic coarse "
                "step cannot land inside a band narrower than itself at all -- this specific vertical option "
                "structurally disappears at the coarser step, independent of XY (same fixed horizontal geometry)",
    })

    # 4. Ambiguity interval narrow, but NO meaningful option lost -- the
    #    counter-example: relief is tiny (ambiguity band narrower than
    #    z_step) but the mission's actual operating band is far above it,
    #    so nothing is actually lost by using coarse Z there.
    flat_min, flat_max = 3550.0, 3562.0  # relief=12m, band width=12m < z_step=20m -> "ambiguous by width" alone
    ambiguity_band = (flat_min + STEP2B_CONFIG.min_agl_m, flat_max + STEP2B_CONFIG.min_agl_m)
    mission_operating_band = (5000.0, 6000.0)
    overlaps_mission = not (ambiguity_band[1] <= mission_operating_band[0] or ambiguity_band[0] >= mission_operating_band[1])
    findings.append({
        "case": "4_narrow_band_no_real_loss", "relief_m": flat_max - flat_min,
        "ambiguity_band": ambiguity_band, "mission_operating_band": mission_operating_band,
        "band_narrower_than_z_step": (ambiguity_band[1] - ambiguity_band[0]) < STEP2B_CONFIG.z_step_m,
        "overlaps_mission_operating_band": overlaps_mission,
        "note": "ambiguity-width alone flags this cell for refinement, but the mission never operates anywhere "
                "near this altitude band -- confirms band-width alone is NOT sufficient, context (which z the "
                "mission actually uses) matters too",
    })

    return findings


def main():
    print("=" * 70)
    print("STEP 2B -- controlled information-loss diagnostics (no adaptive graph, no thresholds)")
    print("=" * 70)

    # ---- shared real-DEM setup ----
    fine_roi = load_roi(DEFAULT_CONFIG)
    fine_terrain = TerrainQuery(fine_roi)
    coarse_dem = build_coarse_dem(fine_roi, factor=3)
    coarse_terrain = TerrainQuery(coarse_dem.roi)
    coarse_stats = build_coarse_terrain_stats(fine_roi, factor=3)

    # real high-relief window (found by scanning CoarseTerrainStats.relief for
    # the highest values -- see conversation; coarse(21,81) relief=282m is the
    # single highest-relief coarse cell in the whole ROI).
    window_r0, window_r1 = 55, 75
    window_c0, window_c1 = 235, 255

    print("\n--- A. XY REPRESENTATION LOSS (real DEM, same physical edge) ---")
    roi_rows = range(window_r0, window_r1)
    roi_cols = range(window_c0, window_c1)
    preserved, false_block, unsafe_optimism = section_a(fine_terrain, coarse_terrain, STEP2B_CONFIG, roi_rows, roi_cols)
    total = len(preserved) + len(false_block) + len(unsafe_optimism)
    print(f"  window: rows[{window_r0},{window_r1}) cols[{window_c0},{window_c1}) "
          f"(chosen around the single highest-relief coarse cell in the whole ROI, coarse(row=21,col=81), relief=282m)")
    print(f"  total edges tested: {total}")
    print(f"  PRESERVED: {len(preserved)} ({100*len(preserved)/total:.1f}%)")
    print(f"  FALSE-BLOCK: {len(false_block)} ({100*len(false_block)/total:.1f}%)")
    print(f"  UNSAFE-OPTIMISM: {len(unsafe_optimism)} ({100*len(unsafe_optimism)/total:.1f}%)"
          + ("  <<< UNEXPECTED, investigate" if unsafe_optimism else "  (expected: 0, matches MAX-pooling guarantee)"))
    if false_block:
        print("  representative FALSE-BLOCK examples (fine safe, coarse blocks it):")
        for r in false_block[:5]:
            print(f"    row={r['row']} col={r['col']} dir=({r['drow']},{r['dcol']}) z={r['z_msl']:.0f}m "
                  f"start_elev={r['start_elev']:.1f}m  fine={r['fine_valid']}  coarse={r['coarse_valid']}({r['coarse_reason']})")
    if unsafe_optimism:
        print("  UNSAFE-OPTIMISM examples (investigate as potential bug):")
        for r in unsafe_optimism[:5]:
            print(f"    {r}")

    print("\n--- B. NODE VALIDITY AMBIGUITY (real DEM, band vs ground truth) ---")
    coarse_r0, coarse_r1 = window_r0 // 3, window_r1 // 3 + 1
    coarse_c0, coarse_c1 = window_c0 // 3, window_c1 // 3 + 1
    test_layers = [3000.0, 3100.0, 3200.0, 3300.0, 3400.0, 3500.0, 3600.0, 3700.0, 3800.0, 5000.0, 6000.0]
    checked, predicted_counts, actual_counts, mismatches = section_b(
        fine_terrain, coarse_stats, STEP2B_CONFIG, range(coarse_r0, coarse_r1), range(coarse_c0, coarse_c1), test_layers,
    )
    print(f"  coarse cells x layers checked: {checked}")
    print(f"  predicted (from min/max band): {predicted_counts}")
    print(f"  actual (ground-truth fine scan): {actual_counts}")
    print(f"  mismatch count: {len(mismatches)}" + ("  <<< investigate" if mismatches else "  (exact match)"))
    if mismatches:
        for m in mismatches[:5]:
            print(f"    coarse(row={m[0]},col={m[1]}) z={m[2]} predicted={m[3]} actual={m[4]} min={m[5]:.1f} max={m[6]:.1f}")

    print("\n--- C. CONNECTIVITY LOSS (synthetic controlled cases) ---")
    for case in section_c(STEP2B_CONFIG):
        print(f"  {case['name']}: fine_connected={case['fine_connected']}  coarse_connected={case['coarse_connected']}"
              f"  {'<<< LOSS (fine connected, coarse disconnected)' if case['loss'] else ''}")

    print("\n--- C. CONNECTIVITY LOSS (real DEM, automated farthest-pair search) ---")
    print(f"  window: rows[{window_r0},{window_r1}) cols[{window_c0},{window_c1}) (same high-relief window as A)")
    for z_msl in (3050.0, 3150.0, 3250.0, 3350.0, 3450.0, 3550.0):
        result = section_c_real(fine_terrain, coarse_terrain, STEP2B_CONFIG, z_msl,
                                 (window_r0, window_r1), (window_c0, window_c1))
        if result.get("status") == "no_component_large_enough":
            print(f"  z={z_msl:.0f}m  (no fine-valid component >=2 cells in this window at this altitude)")
            continue
        print(f"  z={z_msl:.0f}m  start={result['start_rc']} goal={result['goal_rc']} "
              f"component_size={result['component_size']}  fine_connected={result['fine_connected']}  "
              f"coarse_connected={result['coarse_connected']}"
              f"  {'<<< LOSS' if result['loss'] else ''}")

    print("\n--- D. Z REPRESENTATION LOSS (synthetic controlled cases, XY fixed) ---")
    print(f"  (diagnostic test coarse Z step = {TEST_COARSE_Z_STEP}m -- measuring instrument only, not a proposal)")
    for f in section_d():
        print(f"  {f['case']}: {f}")

    print("\n" + "=" * 70)
    print("Done. No refinement rule, threshold, or spacing selected.")


if __name__ == "__main__":
    main()
