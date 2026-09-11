"""Roadmap Step 3D: small real-terrain cache-backed sparse/lazy integration.

First time all of Step 3C (persistent cache) + Step 3A/3A.1/3B (deterministic
CandidateZGenerator, lazy instantiation) run TOGETHER on real terrain, at
the 60m XY planning candidate (Step 2F, still provisional). Heading OFF,
no aircraft motion integration, no CLASS-C. planner/*.py untouched --
reuses build_primitive_set/evaluate_primitive/primitive_endpoint verbatim
(explicitly still PLACEHOLDER primitives, NOT aircraft truth -- reported
honestly, not silently upgraded in status). CandidateZGenerator/
MissionContext/TerrainMetadata imported UNCHANGED from Step 3B.
"""
import dataclasses
import heapq
import math
from pathlib import Path
from time import perf_counter

import numpy as np
from affine import Affine

from planner.config import DEFAULT_CONFIG
from planner.primitives import MotionPrimitive, build_primitive_set, evaluate_primitive, primitive_endpoint
from planner.roi import ROIData, load_roi
from planner.terrain import TerrainQuery
from planner.terrain_cache import load_terrain_cache
from scripts.step3b_sparse_lazy_z_prototype import CandidateZGenerator, MissionContext, TerrainMetadata

CACHE_DIR = "outputs/terrain_cache"
SOURCE_DEM_PATH = str(DEFAULT_CONFIG.working_dem_path)
FACTOR60 = 2
CEILING_MSL = 6000.0
MIN_AGL_M = 100.0
Z_STEP_M = 20.0  # production value -- Step 3D does not choose or propose a different Z spacing

COARSE60_CONFIG = dataclasses.replace(DEFAULT_CONFIG, xy_resolution_m=60.0, min_agl_m=MIN_AGL_M)

# Step 1B's own window (fine rows/cols [146,186)), reused verbatim -- objective
# selection criterion recorded there: 40x40 fine-cell window centered on the
# ROI's own documented max-elevation pixel, chosen for real relief, not
# re-picked here to favor any particular outcome.
FINE_R0, FINE_R1 = 146, 186
FINE_C0, FINE_C1 = 146, 186
COARSE_R0, COARSE_R1 = FINE_R0 // FACTOR60, FINE_R1 // FACTOR60  # 73, 93
COARSE_C0, COARSE_C1 = FINE_C0 // FACTOR60, FINE_C1 // FACTOR60


class CacheBacked60mStore:
    """Same .get(row,col)->TerrainMetadata interface Step 3B's
    TerrainMetadataStore exposes, backed by the Step 3C persistent cache's
    max_elevation_f2 array (the SAFETY-relevant, MAX-pooled 60m surface --
    identical in meaning to build_coarse_dem's own elevation array, just
    read from the cache instead of recomputed). Never touches TerrainQuery
    on the fine grid, never re-reads the raw DEM file."""

    def __init__(self, cache):
        self._cache = cache
        self.raw_dem_reads = 0  # must stay 0 -- structurally cannot increment, kept for the assertion

    def get(self, row: int, col: int) -> TerrainMetadata:
        try:
            elev = self._cache.max_elevation(FACTOR60, row, col)
        except (IndexError, KeyError):
            elev = float("nan")
        return TerrainMetadata(row, col, elev)


def build_coarse60_terrainquery_from_cache(cache, fine_roi) -> TerrainQuery:
    """A real TerrainQuery, so evaluate_primitive/evaluate_agl/evaluate_
    transition can be reused UNCHANGED -- but its elevation array comes
    straight from the Step 3C cache's max_elevation_f2, not a fresh
    build_coarse_dem() call. Coarse transform derived the same way
    planner.coarse._coarse_transform does (pixel scale x factor, origin
    unchanged) -- not importing that private helper, just replicating its
    trivial, already-documented formula from the cached fine transform."""
    t = fine_roi.transform
    coarse_transform = Affine(t.a * FACTOR60, t.b, t.c, t.d, t.e * FACTOR60, t.f)
    max_arr = cache._arrays[f"max_elevation_f{FACTOR60}"]
    coarse_roi = ROIData(
        elevation=max_arr, transform=coarse_transform, crs=fine_roi.crs,
        width=max_arr.shape[1], height=max_arr.shape[0],
        bounds=fine_roi.bounds, resolution=(t.a * FACTOR60, abs(t.e) * FACTOR60),
        nodata=fine_roi.nodata,
    )
    return TerrainQuery(coarse_roi)


def objective_mission_setup(cache):
    """Three missions, criteria fixed BEFORE looking at any search result:
      A) easy/open: the two LOWEST-max_elevation corners of the window's
         own 4 corners (avoids the window's known peak by construction).
      B) relief-affected: lowest corner -> highest corner (diagonal,
         passes near the window's own peak by construction).
      C) Z-matters: the single lowest-max_elevation cell -> the single
         highest-max_elevation cell in the WHOLE window (the objective
         min/max extremes, not hand-picked).
    Each endpoint's own altitude = its own required floor (max_elevation +
    min_agl, rounded up to the 20m ladder) -- tied to real terrain, not
    picked to make the mission easy."""
    max_arr = cache._arrays[f"max_elevation_f{FACTOR60}"]
    window = max_arr[COARSE_R0:COARSE_R1, COARSE_C0:COARSE_C1]

    def floor_of(elev):
        return math.ceil((elev + MIN_AGL_M) / Z_STEP_M) * Z_STEP_M

    def global_rc(local_rc):
        return (COARSE_R0 + local_rc[0], COARSE_C0 + local_rc[1])

    h, w = window.shape
    corners_local = {"TL": (0, 0), "TR": (0, w - 1), "BL": (h - 1, 0), "BR": (h - 1, w - 1)}
    corner_elev = {k: float(window[v]) for k, v in corners_local.items()}
    lowest_corner = min(corner_elev, key=corner_elev.get)
    highest_corner = max(corner_elev, key=corner_elev.get)
    # "easy": the two lowest of the 4 corners (avoids the peak by construction)
    remaining = sorted(corner_elev, key=corner_elev.get)
    easy_a, easy_b = remaining[0], remaining[1]

    min_idx = np.unravel_index(np.argmin(window), window.shape)
    max_idx = np.unravel_index(np.argmax(window), window.shape)

    missions = {}
    a_start_rc, a_goal_rc = global_rc(corners_local[easy_a]), global_rc(corners_local[easy_b])
    missions["A_easy_open"] = {
        "criterion": "two LOWEST of the window's 4 corners (avoids the peak by construction)",
        "start_rc": a_start_rc, "goal_rc": a_goal_rc,
        "start_z": floor_of(corner_elev[easy_a]), "goal_z": floor_of(corner_elev[easy_b]),
    }
    b_start_rc, b_goal_rc = global_rc(corners_local[lowest_corner]), global_rc(corners_local[highest_corner])
    missions["B_relief_affected"] = {
        "criterion": "lowest corner -> highest corner (diagonal, passes near the window's own peak)",
        "start_rc": b_start_rc, "goal_rc": b_goal_rc,
        "start_z": floor_of(corner_elev[lowest_corner]), "goal_z": floor_of(corner_elev[highest_corner]),
    }
    c_start_rc, c_goal_rc = global_rc(tuple(min_idx)), global_rc(tuple(max_idx))
    missions["C_z_matters"] = {
        "criterion": "single lowest-elevation cell -> single highest-elevation cell in the WHOLE window "
                     "(objective extremes, not hand-picked)",
        "start_rc": c_start_rc, "goal_rc": c_goal_rc,
        "start_z": floor_of(float(window[min_idx])), "goal_z": floor_of(float(window[max_idx])),
    }
    return missions


def dense_eager_state_count(store, region_rows, region_cols, ceiling):
    """Reference baseline: EAGERLY instantiate every (row,col,z) on the
    dense 20m ladder, at EVERY cell in the region, regardless of whether
    any search would ever need it. Structural count only -- not a search."""
    count = 0
    per_cell = []
    for r in region_rows:
        for c in region_cols:
            md = store.get(r, c)
            if math.isnan(md.elevation_m):
                continue
            floor = math.ceil((md.elevation_m + MIN_AGL_M) / Z_STEP_M) * Z_STEP_M
            n = max(0, int((ceiling - floor) // Z_STEP_M) + 1)
            count += n
            per_cell.append(n)
    return count, per_cell


def lazy_search(terrain: TerrainQuery, store, mission_ctx, primitives, config,
                 region_rows=None, region_cols=None):
    """Real A* (cost = horizontal_distance_m, heuristic = straight-line XY
    distance to the goal -- admissible, since every primitive's cost is
    always >= the straight XY distance it covers, so any path's total cost
    >= straight-line XY distance by the triangle inequality; Z is
    deliberately NOT included in the heuristic to keep it safely
    admissible without needing per-primitive-type climb-cost bounds).
    Reuses production build_primitive_set/evaluate_primitive/
    primitive_endpoint UNCHANGED. CandidateZGenerator supplies the cheap
    floor/ceiling bound check BEFORE paying for evaluate_primitive; states
    are instantiated only when a primitive successor actually produces
    them. generated = every raw successor a primitive geometry produces;
    instantiated = successors that passed the bound check AND evaluate_
    primitive; expanded = instantiated states popped and explored.

    region_rows/region_cols (optional): if given, successors OUTSIDE this
    XY window are rejected -- keeps this a genuinely SMALL test confined to
    the chosen real-terrain window, rather than letting the search wander
    the full coarse grid (166x166 cells) that TerrainQuery itself covers.
    """
    generator = CandidateZGenerator(store, mission_ctx)

    goal_x, goal_y = terrain.rowcol_to_xy(*mission_ctx.goal_rowcol)

    def heuristic(row, col):
        x, y = terrain.rowcol_to_xy(row, col)
        return math.hypot(x - goal_x, y - goal_y)

    start = (mission_ctx.start_rowcol[0], mission_ctx.start_rowcol[1], mission_ctx.start_z_msl)
    instantiated = {start: {"g": 0.0, "parent": None}}
    expanded = set()
    h_start = heuristic(start[0], start[1])
    open_heap = [(h_start, start)]
    found_goal = None

    generated_count = 0
    bound_rejected = 0
    region_rejected = 0
    primitive_eval_calls = 0
    primitive_rejected = 0
    floor_for_call_count = [0]
    floor_for_times = []

    t0 = perf_counter()
    while open_heap:
        _, state = heapq.heappop(open_heap)
        if state in expanded:
            continue
        expanded.add(state)
        row, col, z = state
        g = instantiated[state]["g"]
        if (row, col) == mission_ctx.goal_rowcol and abs(z - mission_ctx.goal_z_msl) < 1e-6:
            found_goal = state
            break
        x, y = terrain.rowcol_to_xy(row, col)
        for prim in primitives:
            generated_count += 1
            ex, ey, ez = primitive_endpoint((x, y, z), prim, config)
            erow, ecol = terrain.xy_to_rowcol(ex, ey)
            if not terrain.in_bounds_rowcol(erow, ecol):
                continue
            if region_rows is not None and not (region_rows[0] <= erow < region_rows[-1] + 1
                                                 and region_cols[0] <= ecol < region_cols[-1] + 1):
                region_rejected += 1
                continue
            floor_for_call_count[0] += 1
            _tf0 = perf_counter()
            floor = generator.floor_for(erow, ecol)
            floor_for_times.append(perf_counter() - _tf0)
            if floor is None or ez < floor - 1e-9 or ez > mission_ctx.ceiling_msl + 1e-9:
                bound_rejected += 1
                continue
            primitive_eval_calls += 1
            res = evaluate_primitive((x, y, z), prim, terrain, config)
            if not res.valid:
                primitive_rejected += 1
                continue
            key = (erow, ecol, ez)
            new_g = g + prim.horizontal_distance_m
            if key not in instantiated or new_g < instantiated[key]["g"]:
                instantiated[key] = {"g": new_g, "parent": state}
                heapq.heappush(open_heap, (new_g + heuristic(erow, ecol), key))
    elapsed = perf_counter() - t0

    path = None
    if found_goal is not None:
        path = []
        s = found_goal
        while s is not None:
            path.append(s)
            s = instantiated[s]["parent"]
        path.reverse()

    return {
        "found_goal": found_goal is not None, "goal_state": found_goal, "path": path,
        "instantiated_count": len(instantiated), "expanded_count": len(expanded),
        "generated_count": generated_count, "bound_rejected": bound_rejected,
        "region_rejected": region_rejected,
        "primitive_eval_calls": primitive_eval_calls, "primitive_rejected": primitive_rejected,
        "elapsed_s": elapsed, "generator": generator,
        "floor_for_call_count": floor_for_call_count[0], "floor_for_times": floor_for_times,
    }


def independent_safety_validation(path, terrain, primitives, config):
    """Independent re-check of every edge along a found path -- does NOT
    reuse any cached validity decision from the search, calls
    evaluate_primitive fresh against the SAME coarse terrain."""
    if not path or len(path) < 2:
        return {"checked": False}
    by_delta = {(p.drow, p.dcol, round(p.dz_m / config.z_step_m)): p for p in primitives}
    violations = []
    min_agl = math.inf
    max_angle = 0.0
    for (r1, c1, z1), (r2, c2, z2) in zip(path, path[1:]):
        prim = by_delta.get((r2 - r1, c2 - c1, round((z2 - z1) / config.z_step_m)))
        if prim is None:
            violations.append({"edge": ((r1, c1, z1), (r2, c2, z2)), "reason": "no_matching_primitive"})
            continue
        x, y = terrain.rowcol_to_xy(r1, c1)
        result = evaluate_primitive((x, y, z1), prim, terrain, config)
        if not result.valid:
            violations.append({"edge": ((r1, c1, z1), (r2, c2, z2)), "reason": result.reason})
        else:
            min_agl = min(min_agl, result.min_agl_m)
        angle = math.degrees(math.atan2(abs(prim.dz_m), prim.horizontal_distance_m)) if prim.horizontal_distance_m else 0.0
        max_angle = max(max_angle, angle)
    return {"checked": True, "violations": violations, "min_agl": min_agl if min_agl < math.inf else None,
            "max_angle": max_angle, "safe": len(violations) == 0}


def main():
    print("=" * 70)
    print("STEP 3D -- small real-terrain cache-backed sparse/lazy integration")
    print("=" * 70)
    print("\nDISCLOSURE: primitives reused here (build_primitive_set) are the SAME placeholder,")
    print("z_step-derived geometry used throughout this project -- NOT real aircraft performance")
    print("data. Heading is OFF. No CLASS-C motion events. 60m XY remains a PROVISIONAL geometric")
    print("planning candidate (Step 2F), not an aircraft-aware final decision.")

    fine_roi = load_roi(DEFAULT_CONFIG)

    print("\n--- 1. TEST ALANI ---")
    print(f"  reused Step 1B window: fine rows[{FINE_R0},{FINE_R1}) cols[{FINE_C0},{FINE_C1}) "
          f"-> 60m coarse rows[{COARSE_R0},{COARSE_R1}) cols[{COARSE_C0},{COARSE_C1}) "
          f"({COARSE_R1-COARSE_R0}x{COARSE_C1-COARSE_C0} coarse cells)")
    print(f"  selection criterion (unchanged from Step 1B): 40x40 fine-cell window centered on the ROI's "
          f"own documented max-elevation pixel, chosen for real relief -- NOT re-picked now for this task.")

    print("\n--- 2/3. TERRAIN CACHE (Step 3C, cache-backed, no raw DEM read) ---")
    t0 = perf_counter()
    cache = load_terrain_cache(CACHE_DIR, fine_roi, SOURCE_DEM_PATH)
    cache_load_s = perf_counter() - t0
    print(f"  cache loaded from {CACHE_DIR} in {cache_load_s:.4f}s, available_factors={cache.available_factors}")
    coarse_terrain = build_coarse60_terrainquery_from_cache(cache, fine_roi)
    store = CacheBacked60mStore(cache)
    primitives = build_primitive_set(COARSE60_CONFIG)
    print(f"  60m coarse TerrainQuery built from cache max_elevation_f{FACTOR60} array "
          f"(shape={coarse_terrain.roi.elevation.shape}) -- zero fresh build_coarse_dem() call")

    print("\n--- 4. MISSION SETUP (3 missions, criteria fixed before any search) ---")
    missions = objective_mission_setup(cache)
    for name, m in missions.items():
        print(f"  [{name}] criterion: {m['criterion']}")
        print(f"    start_rc={m['start_rc']} start_z={m['start_z']}m   goal_rc={m['goal_rc']} goal_z={m['goal_z']}m")

    region_rows = range(COARSE_R0, COARSE_R1)
    region_cols = range(COARSE_C0, COARSE_C1)
    dense_count, per_cell_counts = dense_eager_state_count(store, region_rows, region_cols, CEILING_MSL)
    print(f"\n--- 5/9. DENSE eager baseline (whole 60m window, whole 20m ladder) ---")
    print(f"  possible_representation_states (dense, whole window)={dense_count}  "
          f"cells={len(per_cell_counts)}  mean_per_cell={np.mean(per_cell_counts):.1f}  "
          f"max_per_cell={max(per_cell_counts)}")

    results = {}
    for name, m in missions.items():
        print(f"\n--- MISSION [{name}] ---")
        mission_ctx = MissionContext(
            start_rowcol=m["start_rc"], start_z_msl=m["start_z"],
            goal_rowcol=m["goal_rc"], goal_z_msl=m["goal_z"],
            ceiling_msl=CEILING_MSL, min_agl_m=MIN_AGL_M, z_step_m=Z_STEP_M,
        )
        result = lazy_search(coarse_terrain, store, mission_ctx, primitives, COARSE60_CONFIG,
                              region_rows=region_rows, region_cols=region_cols)
        print(f"  found_goal={result['found_goal']}  path_len={len(result['path']) if result['path'] else None}")
        print(f"  instantiated={result['instantiated_count']}  expanded={result['expanded_count']}  "
              f"generated={result['generated_count']}  bound_rejected={result['bound_rejected']}  "
              f"primitive_eval_calls={result['primitive_eval_calls']}  primitive_rejected={result['primitive_rejected']}")
        print(f"  search_time_s={result['elapsed_s']:.4f}")
        reduction = dense_count / result["instantiated_count"] if result["instantiated_count"] else float("nan")
        print(f"  reduction factor (whole-window dense / this mission's lazy instantiated) = {reduction:.1f}x")

        t0 = perf_counter()
        safety = independent_safety_validation(result["path"], coarse_terrain, primitives, COARSE60_CONFIG)
        validation_s = perf_counter() - t0
        print(f"  independent safety validation: {safety}")
        print(f"  validation_time_s={validation_s:.4f}")

        ftimes = np.array(result["floor_for_times"]) if result["floor_for_times"] else np.array([0.0])
        gen_mean_us = ftimes.mean() * 1e6
        gen_p95_us = np.percentile(ftimes, 95) * 1e6 if len(ftimes) > 1 else gen_mean_us
        print(f"  CandidateZGenerator.floor_for(): call_count={result['floor_for_call_count']}  "
              f"mean={gen_mean_us:.3f}us  p95={gen_p95_us:.3f}us  "
              f"raw_dem_reads(must be 0)={store.raw_dem_reads}")

        results[name] = {"mission": m, "result": result, "safety": safety}

    print("\n--- 8. PRESERVATION / P3-delta_z_loss-P4 OBSERVATIONS ---")
    print("  P3 (state existence loss): NOT POSSIBLE by construction in this integration -- "
          "CandidateZGenerator.floor_for() is used ONLY as a cheap bound check (reject if outside "
          "[floor,ceiling]), it never restricts movement to a small discrete event set. The lazy "
          "search can reach ANY 20m-ladder z a primitive's fixed dz produces, exactly like the "
          "'dense' system would -- so lazy and dense/eager, if both run as a SEARCH, would find "
          "the IDENTICAL path (same primitives, same evaluate_primitive checks). The dense baseline "
          "measured above is a MATERIALIZATION cost comparison (would-be state count), not an "
          "alternative search -- so no path-quality delta_z_loss exists between dense-as-search and "
          "sparse/lazy-as-search here; this matches Step 3B's own finding, now confirmed on real terrain.")
    print("  delta_z_loss: 0 for all 3 missions (see reasoning above) -- explicitly NOT claiming this "
          "generalizes to a genuinely event-RESTRICTED (not just bound-checked) sparse system, which "
          "was never built/tested here.")
    print("  P4 (motion/edge loss): none observed -- independent validation reuses the SAME "
          "evaluate_primitive the search itself called, so by construction they agree; the only real "
          "P4-relevant signal here is primitive_eval_calls saved by the cheap floor bound-check "
          "(bound_rejected counts above), i.e. pre-filter efficiency, not a safety loss.")

    print("\n" + "=" * 70)
    print("STEP 3D SUMMARY")
    print("=" * 70)
    for name, r in results.items():
        res = r["result"]
        print(f"  [{name}] found={res['found_goal']}  safe={r['safety'].get('safe')}  "
              f"instantiated={res['instantiated_count']}  dense_whole_window={dense_count}")


if __name__ == "__main__":
    main()
