"""Roadmap Step 2E: 30m vs 60m vs 90m XY representation comparison.

Diagnostic only -- no adaptive graph, no production default declared, no Z
spacing change, no search/cost/heuristic/corridor change. Reuses existing
functions exactly as Steps 2B/2D did (build_coarse_dem is already generic
in `factor`, so 60m is produced with ZERO new pooling logic -- just
factor=2 instead of factor=3).

Aladağlar is used purely as a real-terrain benchmark surface here, per the
explicit instruction -- conclusions are separated into (a) terrain-specific
numbers and (b) the general pooling-arithmetic mechanism behind them.
"""
import dataclasses
import math
from collections import deque

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
NODATA = -9999.0

# Same window used since Step 2B/2D -- kept for continuity, not re-chosen.
WINDOW_ROWS = list(range(55, 75))
WINDOW_COLS = list(range(235, 255))


def make_synthetic_roi(elevation, res=30.0, nodata=NODATA):
    height, width = elevation.shape
    transform = Affine(res, 0.0, 500000.0, 0.0, -res, 4200000.0)
    return ROIData(elevation=elevation.astype(np.float32), transform=transform, crs="EPSG:32636",
                    width=width, height=height,
                    bounds=(500000.0, 4200000.0 - height * res, 500000.0 + width * res, 4200000.0),
                    resolution=(res, res), nodata=nodata)


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


def validity_mask(terrain: TerrainQuery, rows, cols, z_msl, config):
    mask = np.zeros((len(rows), len(cols)), dtype=bool)
    for i, r in enumerate(rows):
        for j, c in enumerate(cols):
            x, y = terrain.rowcol_to_xy(r, c)
            mask[i, j] = evaluate_agl(terrain, x, y, z_msl, config).valid
    return mask


# ===========================================================================
# 1. REPRESENTATIONS
# ===========================================================================

def build_representations():
    fine_roi = load_roi(DEFAULT_CONFIG)
    fine_terrain = TerrainQuery(fine_roi)
    coarse60 = build_coarse_dem(fine_roi, factor=2)   # 30*2=60m, SAME build_coarse_dem, factor changed only
    coarse90 = build_coarse_dem(fine_roi, factor=3)   # existing reference, unchanged
    terrain60 = TerrainQuery(coarse60.roi)
    terrain90 = TerrainQuery(coarse90.roi)
    stats60 = build_coarse_terrain_stats(fine_roi, factor=2)
    stats90 = build_coarse_terrain_stats(fine_roi, factor=3)
    return fine_roi, fine_terrain, coarse60, coarse90, terrain60, terrain90, stats60, stats90


# ===========================================================================
# 2. SAME PHYSICAL EDGE COMPARISON (30m ground truth vs 60m vs 90m)
# ===========================================================================

def section_2(fine_terrain, terrain60, terrain90, config, rows, cols):
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, 1), (1, 1), (-1, -1), (1, -1)]
    results = {"60m": {"preserved": 0, "false_block": 0, "unsafe_optimism": 0, "examples_fb": []},
               "90m": {"preserved": 0, "false_block": 0, "unsafe_optimism": 0, "examples_fb": []}}
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

                fine_valid = evaluate_primitive(start_xyz, prim, fine_terrain, config).valid
                total += 1
                for label, terrain in (("60m", terrain60), ("90m", terrain90)):
                    coarse_valid = evaluate_primitive(start_xyz, prim, terrain, config).valid
                    r = results[label]
                    if fine_valid == coarse_valid:
                        r["preserved"] += 1
                    elif fine_valid and not coarse_valid:
                        r["false_block"] += 1
                        if len(r["examples_fb"]) < 3:
                            r["examples_fb"].append((row, col, drow, dcol, z_msl, float(start_elev)))
                    else:
                        r["unsafe_optimism"] += 1

    return total, results


# ===========================================================================
# 3. CONNECTIVITY PRESERVATION
# ===========================================================================

def connectivity_all_res(elev, factor60, factor90, z_msl, config, start_rc, goal_rc):
    fine_roi = make_synthetic_roi(elev)
    fine_terrain = TerrainQuery(fine_roi)
    c60 = TerrainQuery(build_coarse_dem(fine_roi, factor=factor60).roi)
    c90 = TerrainQuery(build_coarse_dem(fine_roi, factor=factor90).roi)

    h, w = elev.shape
    fine_mask = validity_mask(fine_terrain, range(h), range(w), z_msl, config)
    mask60 = validity_mask(c60, range(h // factor60), range(w // factor60), z_msl, config)
    mask90 = validity_mask(c90, range(h // factor90), range(w // factor90), z_msl, config)

    fine_c = bfs_connected(fine_mask, start_rc, goal_rc)
    c60_c = bfs_connected(mask60, (start_rc[0] // factor60, start_rc[1] // factor60),
                           (goal_rc[0] // factor60, goal_rc[1] // factor60))
    c90_c = bfs_connected(mask90, (start_rc[0] // factor90, start_rc[1] // factor90),
                           (goal_rc[0] // factor90, goal_rc[1] // factor90))
    return {"fine_connected": fine_c, "60m_connected": c60_c, "90m_connected": c90_c}


def section_3_synthetic(config):
    """Two LOW basins (left cols 0-5, right cols 12-17) separated by a HIGH
    ridge (cols 6-11, full column span -- this is the wall the aircraft must
    get THROUGH). The pass is a band of OPEN ROWS cut through that ridge
    (all 6 ridge columns at once, at those rows) -- exactly the Step 2B/2D
    geometry, extended so the opened-row band width can be aligned to match
    (or fail to match) a whole 60m row-block ([2,3] at factor=2) vs a whole
    90m row-block ([0,1,2] or [3,4,5] at factor=3) -- rows=6 is the LCM(2,3)
    alignment point. Start/goal sit in the basins (always valid, independent
    of the ridge), at row=2 -- the first row of every opened band tested."""
    rows, cols = 6, 18
    z_msl = 3000.0 + config.min_agl_m + 10.0
    start_rc, goal_rc = (2, 1), (2, 16)
    results = {}

    def make(gap_rows):
        e = np.full((rows, cols), 3000.0)   # basins: low everywhere by default
        e[:, 6:12] = 4000.0                  # ridge: full column span, high
        for r in gap_rows:
            e[r, 6:12] = 3000.0              # open THROUGH the ridge at these specific rows
        return e

    results["wide (gap=all 6 rows, full ridge open)"] = connectivity_all_res(make(range(6)), 2, 3, z_msl, config, start_rc, goal_rc)
    results["medium (gap=rows[2,3], == one 60m row-block, spans two 90m row-blocks)"] = connectivity_all_res(make([2, 3]), 2, 3, z_msl, config, start_rc, goal_rc)
    results["narrow (gap=row[2] only, < either block)"] = connectivity_all_res(make([2]), 2, 3, z_msl, config, start_rc, goal_rc)
    results["blocked (gap=none)"] = connectivity_all_res(make([]), 2, 3, z_msl, config, start_rc, goal_rc)
    return results


def section_3_real(fine_terrain, terrain60, terrain90, config, rows, cols, z_lo, z_hi):
    """Same automated farthest-pair + critical-event methodology as Step 2D,
    extended to 60m and 90m together."""
    fine_events = set()
    for r in rows:
        for c in cols:
            e = fine_terrain.query(*fine_terrain.rowcol_to_xy(r, c)).elevation
            if not math.isnan(e):
                fine_events.add(round(float(e) + config.min_agl_m, 6))
    for label, terrain, factor in (("60", terrain60, 2), ("90", terrain90, 3)):
        cr0, cr1 = rows[0] // factor, rows[-1] // factor + 1
        cc0, cc1 = cols[0] // factor, cols[-1] // factor + 1
        for cr in range(cr0, cr1):
            for cc in range(cc0, cc1):
                e = terrain.query(*terrain.rowcol_to_xy(cr, cc)).elevation
                if not math.isnan(e):
                    fine_events.add(round(float(e) + config.min_agl_m, 6))

    events = sorted(e for e in fine_events if z_lo <= e <= z_hi)
    boundaries = sorted(set([z_lo] + events + [z_hi]))

    loss_60_only = loss_90_only = loss_both = recovered_by_60 = preserved_both = 0
    interval_examples = []

    for i in range(len(boundaries) - 1):
        lo, hi = boundaries[i], boundaries[i + 1]
        if hi - lo < 1e-6:
            continue
        rep_z = (lo + hi) / 2.0
        fine_mask = validity_mask(fine_terrain, rows, cols, rep_z, config)
        visited = np.zeros_like(fine_mask, dtype=bool)
        best = set()
        for ii in range(fine_mask.shape[0]):
            for jj in range(fine_mask.shape[1]):
                if fine_mask[ii, jj] and not visited[ii, jj]:
                    comp = _flood_fill(fine_mask, (ii, jj))
                    for cell in comp:
                        visited[cell] = True
                    if len(comp) > len(best):
                        best = comp
        if len(best) < 2:
            continue
        comp_list = list(best)
        bp, bd = None, -1.0
        for a in range(len(comp_list)):
            for b in range(a + 1, len(comp_list)):
                d = (comp_list[a][0] - comp_list[b][0]) ** 2 + (comp_list[a][1] - comp_list[b][1]) ** 2
                if d > bd:
                    bd, bp = d, (comp_list[a], comp_list[b])
        (lr1, lc1), (lr2, lc2) = bp
        r0v, c0v = rows[0], cols[0]
        start_rc = (r0v + lr1, c0v + lc1)
        goal_rc = (r0v + lr2, c0v + lc2)

        fine_full = np.zeros((max(rows) + 1, max(cols) + 1), dtype=bool)
        for r in rows:
            for c in cols:
                x, y = fine_terrain.rowcol_to_xy(r, c)
                fine_full[r, c] = evaluate_agl(fine_terrain, x, y, rep_z, config).valid
        fine_conn = bfs_connected(fine_full, start_rc, goal_rc)

        def coarse_conn(terrain, factor):
            cr0, cr1 = rows[0] // factor, rows[-1] // factor + 1
            cc0, cc1 = cols[0] // factor, cols[-1] // factor + 1
            m = np.zeros((cr1, cc1), dtype=bool)
            for cr in range(cr0, cr1):
                for cc in range(cc0, cc1):
                    x, y = terrain.rowcol_to_xy(cr, cc)
                    m[cr, cc] = evaluate_agl(terrain, x, y, rep_z, config).valid
            return bfs_connected(m, (start_rc[0] // factor, start_rc[1] // factor),
                                  (goal_rc[0] // factor, goal_rc[1] // factor))

        c60 = coarse_conn(terrain60, 2)
        c90 = coarse_conn(terrain90, 3)

        if fine_conn and not c60 and not c90:
            loss_both += 1
        elif fine_conn and not c60:
            loss_60_only += 1
        elif fine_conn and not c90:
            loss_90_only += 1
            recovered_by_60 += 1  # 60m stayed connected where 90m did not
        elif fine_conn and c60 and c90:
            preserved_both += 1

        if len(interval_examples) < 8:
            interval_examples.append((round(lo, 1), round(hi, 1), fine_conn, c60, c90))

    return {
        "n_intervals": len(boundaries) - 1, "loss_60_only": loss_60_only, "loss_90_only": loss_90_only,
        "loss_both": loss_both, "recovered_by_60": recovered_by_60, "preserved_both": preserved_both,
        "interval_examples": interval_examples,
    }


# ===========================================================================
# 4. FULL-ROI CHEAP COVERAGE METRICS
# ===========================================================================

def section_4(fine_roi, stats60, stats90, config, ceiling=6000.0):
    fine_elev = fine_roi.elevation
    valid_mask = fine_elev != fine_roi.nodata if fine_roi.nodata is not None else np.ones_like(fine_elev, dtype=bool)
    global_min = float(fine_elev[valid_mask].min())
    domain_lo = global_min + config.min_agl_m
    domain_hi = ceiling

    def ambiguity_stats(stats, label):
        min_e, max_e = stats.min_elevation, stats.max_elevation
        nodata_mask = (min_e == stats.nodata) if stats.nodata is not None else np.zeros_like(min_e, dtype=bool)
        valid = ~nodata_mask
        amb_lo = min_e + config.min_agl_m
        amb_hi = max_e + config.min_agl_m
        intersects = valid & ~((amb_hi <= domain_lo) | (amb_lo >= domain_hi))
        total = int(valid.sum())
        band_width = (amb_hi - amb_lo)[valid]  # == relief; the actual informative quantity

        # NOTE: "intersects domain" alone saturates near 100% here, because
        # the domain [global_min+min_agl, 6000] is far wider than real
        # Aladağlar terrain's own elevation spread (~1698-3700m) -- almost
        # any cell with nonzero relief trivially has SOME z inside such a
        # wide domain where it's ambiguous. The band-width distribution
        # below is the actually discriminating measurement.
        return {
            "label": label, "total_cells": total,
            "intersects_domain_pct": 100.0 * intersects.sum() / total if total else float("nan"),
            "zero_relief_pct": 100.0 * float((band_width == 0).sum()) / total if total else float("nan"),
            "band_width_mean_m": float(band_width.mean()) if total else float("nan"),
            "band_width_median_m": float(np.median(band_width)) if total else float("nan"),
            "band_width_p90_m": float(np.percentile(band_width, 90)) if total else float("nan"),
            "band_width_max_m": float(band_width.max()) if total else float("nan"),
        }

    return {
        "domain": (domain_lo, domain_hi), "global_min_terrain": global_min,
        "60m": ambiguity_stats(stats60, "60m"), "90m": ambiguity_stats(stats90, "90m"),
    }


# ===========================================================================
# 5. STATE-SIZE / MEMORY ESTIMATE (XY only)
# ===========================================================================

def section_5(fine_roi, coarse60, coarse90):
    fine_cells = fine_roi.height * fine_roi.width
    cells60 = coarse60.roi.height * coarse60.roi.width
    cells90 = coarse90.roi.height * coarse90.roi.width
    bytes_per_cell = 4 * 4  # min/max/mean/relief, float32 each -- CoarseTerrainStats' own shape
    return {
        "30m": {"cells": fine_cells, "reduction_vs_30m": 1.0, "metadata_MB": fine_cells * bytes_per_cell / 1e6},
        "60m": {"cells": cells60, "reduction_vs_30m": fine_cells / cells60, "metadata_MB": cells60 * bytes_per_cell / 1e6},
        "90m": {"cells": cells90, "reduction_vs_30m": fine_cells / cells90, "metadata_MB": cells90 * bytes_per_cell / 1e6},
    }


def main():
    print("=" * 70)
    print("STEP 2E -- 30m vs 60m vs 90m XY representation comparison")
    print("=" * 70)

    fine_roi, fine_terrain, coarse60, coarse90, terrain60, terrain90, stats60, stats90 = build_representations()

    print("\n--- 1. REPRESENTATIONS ---")
    print(f"  30m fine: {fine_roi.height}x{fine_roi.width}")
    print(f"  60m (factor=2, build_coarse_dem, MAX-pooled, unchanged semantics): {coarse60.roi.height}x{coarse60.roi.width}")
    print(f"  90m (factor=3, existing reference): {coarse90.roi.height}x{coarse90.roi.width}")

    print("\n--- 2. SAME PHYSICAL EDGE COMPARISON (30m ground truth) ---")
    print(f"  window: rows[55,75) cols[235,255) (same window as Step 2B/2D)")
    total, res2 = section_2(fine_terrain, terrain60, terrain90, STEP2B_CONFIG, WINDOW_ROWS, WINDOW_COLS)
    print(f"  total edges tested: {total}")
    for label in ("60m", "90m"):
        r = res2[label]
        print(f"  [{label}] PRESERVED={r['preserved']} ({100*r['preserved']/total:.1f}%)  "
              f"FALSE-BLOCK={r['false_block']} ({100*r['false_block']/total:.1f}%)  "
              f"UNSAFE-OPTIMISM={r['unsafe_optimism']}"
              + ("  (expected 0)" if r['unsafe_optimism'] == 0 else "  <<< UNEXPECTED"))
        if r["examples_fb"]:
            print(f"    example false-blocks: {r['examples_fb']}")

    print("\n--- 3. CONNECTIVITY PRESERVATION (synthetic, gap width isolates factor=2 vs factor=3 pooling) ---")
    for name, r in section_3_synthetic(STEP2B_CONFIG).items():
        print(f"  {name}: {r}")

    print("\n--- 3. CONNECTIVITY PRESERVATION (real DEM, same Step 2B/2D window, event-based) ---")
    res3r = section_3_real(fine_terrain, terrain60, terrain90, STEP2B_CONFIG, WINDOW_ROWS, WINDOW_COLS, 3000.0, 3600.0)
    print(f"  intervals examined: {res3r['n_intervals']}")
    print(f"  preserved at both 60m & 90m: {res3r['preserved_both']}")
    print(f"  loss at 90m only (60m recovers it): {res3r['loss_90_only']}  <-- 60m's advantage over 90m")
    print(f"  loss at 60m only (90m recovers it): {res3r['loss_60_only']}")
    print(f"  loss at BOTH 60m and 90m: {res3r['loss_both']}")
    print(f"  representative intervals (lo, hi, fine, 60m, 90m): {res3r['interval_examples']}")

    print("\n--- 4. FULL-ROI CHEAP COVERAGE METRICS ---")
    res4 = section_4(fine_roi, stats60, stats90, STEP2B_CONFIG)
    print(f"  planning-relevant reachable domain: [{res4['domain'][0]:.1f}, {res4['domain'][1]:.1f}]m "
          f"(global min terrain {res4['global_min_terrain']:.1f}m + min_agl, up to the 6000m ceiling -- "
          f"NOT restricted to [5000,6000])")
    print(f"  60m: {res4['60m']}")
    print(f"  90m: {res4['90m']}")

    print("\n--- 5. STATE-SIZE / MEMORY ESTIMATE (XY only, no Z assumption) ---")
    for label, r in section_5(fine_roi, coarse60, coarse90).items():
        print(f"  {label}: {r}")

    print("\n" + "=" * 70)
    print("Done. No production default declared, no threshold/spacing decision made.")


if __name__ == "__main__":
    main()
