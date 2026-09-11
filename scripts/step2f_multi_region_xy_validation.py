"""Roadmap Step 2F: MULTI-REGION XY VALIDATION + FINAL XY DECISION.

Reuses Step 2E's own functions (imported, not duplicated) applied to five
OBJECTIVELY selected windows instead of one. No adaptive graph, no Z
resolution choice, no search, no corridor change. planner/*.py untouched.
"""
import math
from collections import deque

import numpy as np

from planner.agl import evaluate_agl
from planner.coarse import build_coarse_terrain_stats
from planner.corridor import bfs_connected
from scripts.step2e_xy_resolution_comparison import (
    STEP2B_CONFIG, build_representations, section_2, section_3_real, validity_mask, _flood_fill,
)

WINDOW_HALF = 10  # matches the 20x20 window size used since Step 2B/2D/2E


# ===========================================================================
# 1. REPRESENTATIVE WINDOWS -- objective, measurable selection criteria
# ===========================================================================

def select_windows(fine_roi, stats90, margin=3):
    relief = stats90.relief
    min_e, max_e = stats90.min_elevation, stats90.max_elevation
    h, w = relief.shape
    nodata = stats90.nodata
    valid = (min_e != nodata) if nodata is not None else np.ones_like(min_e, dtype=bool)

    flat_relief = relief[valid]
    p10, p50 = np.percentile(flat_relief, 10), np.percentile(flat_relief, 50)

    def closest_to(target):
        best_cell, best_d = None, math.inf
        for r in range(margin, h - margin):
            for c in range(margin, w - margin):
                if not valid[r, c]:
                    continue
                d = abs(relief[r, c] - target)
                if d < best_d:
                    best_d, best_cell = d, (r, c)
        return best_cell

    def strict_local_extremum(field, want_min):
        best_cell, best_score = None, -1.0
        for r in range(margin, h - margin):
            for c in range(margin, w - margin):
                if not valid[r, c]:
                    continue
                neigh = [field[r + dr, c + dc] for dr in (-1, 0, 1) for dc in (-1, 0, 1)
                         if not (dr == 0 and dc == 0) and valid[r + dr, c + dc]]
                if len(neigh) != 8:
                    continue
                if want_min and field[r, c] < min(neigh):
                    score = min(neigh) - field[r, c]
                elif not want_min and field[r, c] > max(neigh):
                    score = field[r, c] - max(neigh)
                else:
                    continue
                if score > best_score:
                    best_score, best_cell = score, (r, c)
        return best_cell, best_score

    low_cell = closest_to(p10)
    med_cell = closest_to(p50)
    valley_cell, valley_score = strict_local_extremum(min_e, want_min=True)
    ridge_cell, ridge_score = strict_local_extremum(max_e, want_min=False)

    def to_window(coarse_rc, factor=3):
        cr, cc = coarse_rc
        center_r, center_c = cr * factor + factor // 2, cc * factor + factor // 2
        r0, c0 = center_r - WINDOW_HALF, center_c - WINDOW_HALF
        r1, c1 = center_r + WINDOW_HALF, center_c + WINDOW_HALF
        r0, c0 = max(0, r0), max(0, c0)
        r1, c1 = min(fine_roi.height, r1), min(fine_roi.width, c1)
        return list(range(r0, r1)), list(range(c0, c1))

    windows = {
        "low-relief (p10, relief={:.1f}m)".format(relief[low_cell]): {
            "coarse_cell": low_cell, "criterion": "relief closest to full-map 10th percentile",
            "rows_cols": to_window(low_cell),
        },
        "medium-relief (p50, relief={:.1f}m)".format(relief[med_cell]): {
            "coarse_cell": med_cell, "criterion": "relief closest to full-map median",
            "rows_cols": to_window(med_cell),
        },
        "high-relief (REGRESSION reference, relief={:.1f}m, unchanged since Step 2B)".format(relief[(21, 81)]): {
            "coarse_cell": (21, 81), "criterion": "single highest-relief coarse cell (Step 2B's own window, kept for continuity)",
            "rows_cols": (list(range(55, 75)), list(range(235, 255))),
        },
        "valley-like (local min_elevation dip={:.1f}m below all 8 neighbors)".format(valley_score): {
            "coarse_cell": valley_cell, "criterion": "strict local minimum of coarse min_elevation vs its 8 neighbors",
            "rows_cols": to_window(valley_cell),
        },
        "ridge-like (local max_elevation rise={:.1f}m above all 8 neighbors)".format(ridge_score): {
            "coarse_cell": ridge_cell, "criterion": "strict local maximum of coarse max_elevation vs its 8 neighbors",
            "rows_cols": to_window(ridge_cell),
        },
    }
    return windows


# ===========================================================================
# 3. INVESTIGATE loss_60_only (Step 2E's 4 intervals: 60m lost, 90m preserved)
# ===========================================================================

def investigate_loss_60_only(fine_terrain, terrain60, terrain90, config, rows, cols, z_lo, z_hi):
    fine_events = set()
    for r in rows:
        for c in cols:
            e = fine_terrain.query(*fine_terrain.rowcol_to_xy(r, c)).elevation
            if not math.isnan(e):
                fine_events.add(round(float(e) + config.min_agl_m, 6))
    for terrain, factor in ((terrain60, 2), (terrain90, 3)):
        cr0, cr1 = rows[0] // factor, rows[-1] // factor + 1
        cc0, cc1 = cols[0] // factor, cols[-1] // factor + 1
        for cr in range(cr0, cr1):
            for cc in range(cc0, cc1):
                e = terrain.query(*terrain.rowcol_to_xy(cr, cc)).elevation
                if not math.isnan(e):
                    fine_events.add(round(float(e) + config.min_agl_m, 6))
    events = sorted(e for e in fine_events if z_lo <= e <= z_hi)
    boundaries = sorted(set([z_lo] + events + [z_hi]))

    found = []
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

        def coarse_conn_and_mask(terrain, factor):
            cr0, cr1 = rows[0] // factor, rows[-1] // factor + 1
            cc0, cc1 = cols[0] // factor, cols[-1] // factor + 1
            m = np.zeros((cr1, cc1), dtype=bool)
            for cr in range(cr0, cr1):
                for cc in range(cc0, cc1):
                    x, y = terrain.rowcol_to_xy(cr, cc)
                    m[cr, cc] = evaluate_agl(terrain, x, y, rep_z, config).valid
            conn = bfs_connected(m, (start_rc[0] // factor, start_rc[1] // factor),
                                  (goal_rc[0] // factor, goal_rc[1] // factor))
            return conn, m

        c60, mask60 = coarse_conn_and_mask(terrain60, 2)
        c90, mask90 = coarse_conn_and_mask(terrain90, 3)

        if fine_conn and not c60 and c90:
            start_coarse60 = (start_rc[0] // 2, start_rc[1] // 2)
            goal_coarse60 = (goal_rc[0] // 2, goal_rc[1] // 2)
            found.append({
                "interval": (round(lo, 2), round(hi, 2)), "rep_z": round(rep_z, 2),
                "start_rc": start_rc, "goal_rc": goal_rc,
                "start_coarse60_valid": bool(mask60[start_coarse60]) if start_coarse60[0] < mask60.shape[0] and start_coarse60[1] < mask60.shape[1] else None,
                "goal_coarse60_valid": bool(mask60[goal_coarse60]) if goal_coarse60[0] < mask60.shape[0] and goal_coarse60[1] < mask60.shape[1] else None,
            })
    return found


def main():
    print("=" * 70)
    print("STEP 2F -- multi-region XY validation + final XY decision")
    print("=" * 70)

    fine_roi, fine_terrain, coarse60, coarse90, terrain60, terrain90, stats60, stats90 = build_representations()

    print("\n--- 1. REPRESENTATIVE WINDOWS (objective selection, not cherry-picked to outcome) ---")
    windows = select_windows(fine_roi, stats90)
    for name, w in windows.items():
        print(f"  {name}")
        print(f"    criterion: {w['criterion']}")
        print(f"    coarse cell: {w['coarse_cell']}  fine window: rows{w['rows_cols'][0][0], w['rows_cols'][0][-1]+1} "
              f"cols{w['rows_cols'][1][0], w['rows_cols'][1][-1]+1}")

    print("\n--- 2. 30/60/90 COMPARISON PER WINDOW ---")
    aggregate = []
    for name, w in windows.items():
        rows, cols = w["rows_cols"]
        total, res2 = section_2(fine_terrain, terrain60, terrain90, STEP2B_CONFIG, rows, cols)
        res3 = section_3_real(fine_terrain, terrain60, terrain90, STEP2B_CONFIG, rows, cols, 3000.0, 3800.0)
        print(f"\n  [{name}]")
        for label in ("60m", "90m"):
            r = res2[label]
            print(f"    edges: [{label}] PRESERVED={r['preserved']}/{total} ({100*r['preserved']/total:.1f}%)  "
                  f"FALSE-BLOCK={r['false_block']} ({100*r['false_block']/total:.1f}%)  "
                  f"UNSAFE-OPTIMISM={r['unsafe_optimism']}")
        print(f"    connectivity (band=[3000,3800]m, {res3['n_intervals']} intervals): "
              f"preserved_both={res3['preserved_both']}  loss_90_only(60m recovers)={res3['loss_90_only']}  "
              f"loss_60_only={res3['loss_60_only']}  loss_both={res3['loss_both']}")
        aggregate.append({"window": name, "total_edges": total, "res2": res2, "res3": res3})

    print("\n--- 3. INVESTIGATE loss_60_only (regression window, aligned with Step 2E) ---")
    reg_rows, reg_cols = windows["high-relief (REGRESSION reference, relief={:.1f}m, unchanged since Step 2B)".format(
        stats90.relief[(21, 81)])]["rows_cols"]
    loss60_details = investigate_loss_60_only(fine_terrain, terrain60, terrain90, STEP2B_CONFIG,
                                               reg_rows, reg_cols, 3000.0, 3600.0)
    print(f"  found {len(loss60_details)} interval(s) where fine=True, 60m=False, 90m=True:")
    for d in loss60_details:
        print(f"    {d}")

    print("\n--- 4. GENERALIZATION (aggregate across all windows) ---")
    windows_where_60m_fewer_fb = 0
    total_windows = len(aggregate)
    total_60_fb_rate, total_90_fb_rate = [], []
    total_recovered, total_lost_by_60_alone = 0, 0
    for a in aggregate:
        fb60 = a["res2"]["60m"]["false_block"]
        fb90 = a["res2"]["90m"]["false_block"]
        total_60_fb_rate.append(fb60 / a["total_edges"])
        total_90_fb_rate.append(fb90 / a["total_edges"])
        if fb60 < fb90:
            windows_where_60m_fewer_fb += 1
        total_recovered += a["res3"]["loss_90_only"]
        total_lost_by_60_alone += a["res3"]["loss_60_only"]

    print(f"  windows where 60m had FEWER false-blocks than 90m: {windows_where_60m_fewer_fb}/{total_windows}")
    print(f"  mean false-block rate: 60m={100*np.mean(total_60_fb_rate):.1f}%  90m={100*np.mean(total_90_fb_rate):.1f}%")
    print(f"  total connectivity intervals recovered by 60m (90m lost, 60m didn't) across all windows: {total_recovered}")
    print(f"  total connectivity intervals lost by 60m ALONE (90m preserved) across all windows: {total_lost_by_60_alone}")

    unsafe_optimism_total = sum(a["res2"]["60m"]["unsafe_optimism"] + a["res2"]["90m"]["unsafe_optimism"] for a in aggregate)
    print(f"  MAX-pooling safety invariant (UNSAFE-OPTIMISM) across ALL windows, both resolutions: "
          f"{unsafe_optimism_total} (expected 0)")

    print("\n" + "=" * 70)
    print("Done.")


if __name__ == "__main__":
    main()
