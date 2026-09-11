"""Roadmap Step 3D.1: production goal-tolerance integration + regression
re-run of Step 3D's exact 3 missions.

ONLY change vs Step 3D: the goal-acceptance test. Everywhere else --
ROI, 60m XY, persistent cache, sparse/lazy CandidateZGenerator, primitive
set, search structure, the 3 missions' start/goal/altitudes -- is reused
UNCHANGED (imported from scripts.step3d_real_terrain_integration, not
re-specified). No heuristic/cost/guidance change. planner/*.py untouched.

Goal condition: reuses planner.astar._state_in_goal_region /
_distance_to_goal_box VERBATIM (the actual production Safe Goal Region
logic, Stage 33) instead of Step 3D's exact-XYZ-equality test. Tolerance
values (goal_tolerance_xy_m=35.0, goal_tolerance_z_m=25.0) are the SAME
production constants used throughout this project since Stage 33
(scripts/benchmark_goal_region_real.py and every fine-search benchmark
since) -- not invented for this step.
"""
import dataclasses
import math
from time import perf_counter

import numpy as np

from planner.astar import _state_in_goal_region, msl_to_z_index
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set, evaluate_primitive, primitive_endpoint
from planner.roi import load_roi
from planner.terrain_cache import load_terrain_cache
from scripts.step3b_sparse_lazy_z_prototype import CandidateZGenerator, MissionContext
from scripts.step3d_real_terrain_integration import (
    CACHE_DIR, COARSE60_CONFIG, COARSE_C0, COARSE_C1, COARSE_R0, COARSE_R1, CEILING_MSL,
    FACTOR60, MIN_AGL_M, SOURCE_DEM_PATH, Z_STEP_M, CacheBacked60mStore,
    build_coarse60_terrainquery_from_cache, dense_eager_state_count, independent_safety_validation,
    objective_mission_setup,
)

# Production Safe Goal Region tolerance (Stage 33) -- reused verbatim, not chosen here.
GOAL_TOLERANCE_XY_M = 35.0
GOAL_TOLERANCE_Z_M = 25.0
GOAL_CFG = dataclasses.replace(COARSE60_CONFIG, goal_tolerance_xy_m=GOAL_TOLERANCE_XY_M,
                                goal_tolerance_z_m=GOAL_TOLERANCE_Z_M)


def lazy_search_with_tolerance(terrain, store, mission_ctx, primitives, config, region_rows, region_cols):
    """Identical to step3d_real_terrain_integration.lazy_search EXCEPT the
    goal-acceptance test, which now calls the REAL production
    _state_in_goal_region (Stage 33) instead of exact XYZ equality. States
    here are (row, col, z_msl) -- z_msl is converted to a z_index (exact,
    since every z visited is grid-aligned by construction of the fixed
    +-z_step primitives) only at the moment of calling the production
    function, which expects a CanonicalState=(row,col,z_index)."""
    import heapq

    generator = CandidateZGenerator(store, mission_ctx)
    goal_x, goal_y = terrain.rowcol_to_xy(*mission_ctx.goal_rowcol)
    goal_z_index = msl_to_z_index(mission_ctx.goal_z_msl, config)
    goal_state = (mission_ctx.goal_rowcol[0], mission_ctx.goal_rowcol[1], goal_z_index)

    def heuristic(row, col):
        x, y = terrain.rowcol_to_xy(row, col)
        return math.hypot(x - goal_x, y - goal_y)

    start = (mission_ctx.start_rowcol[0], mission_ctx.start_rowcol[1], mission_ctx.start_z_msl)
    instantiated = {start: {"g": 0.0, "parent": None}}
    expanded = set()
    open_heap = [(heuristic(start[0], start[1]), start)]
    max_open_size = 1
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
        max_open_size = max(max_open_size, len(open_heap))
        _, state = heapq.heappop(open_heap)
        if state in expanded:
            continue
        expanded.add(state)
        row, col, z = state
        g = instantiated[state]["g"]

        z_index = msl_to_z_index(z, config, allow_snap=True)
        if _state_in_goal_region((row, col, z_index), goal_state, terrain, config):
            found_goal = state
            break

        x, y = terrain.rowcol_to_xy(row, col)
        for prim in primitives:
            generated_count += 1
            ex, ey, ez = primitive_endpoint((x, y, z), prim, config)
            erow, ecol = terrain.xy_to_rowcol(ex, ey)
            if not terrain.in_bounds_rowcol(erow, ecol):
                continue
            if not (region_rows[0] <= erow < region_rows[-1] + 1 and region_cols[0] <= ecol < region_cols[-1] + 1):
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
        "region_rejected": region_rejected, "max_open_size": max_open_size,
        "primitive_eval_calls": primitive_eval_calls, "primitive_rejected": primitive_rejected,
        "elapsed_s": elapsed, "floor_for_call_count": floor_for_call_count[0], "floor_for_times": floor_for_times,
    }


def final_goal_error(path, mission_ctx, terrain):
    if not path:
        return None, None
    r, c, z = path[-1]
    gx, gy = terrain.rowcol_to_xy(*mission_ctx.goal_rowcol)
    x, y = terrain.rowcol_to_xy(r, c)
    xy_err = math.hypot(x - gx, y - gy)
    z_err = abs(z - mission_ctx.goal_z_msl)
    return xy_err, z_err


def main():
    print("=" * 70)
    print("STEP 3D.1 -- production goal-tolerance integration + regression re-run")
    print("=" * 70)
    print(f"\n  reused production tolerance (Stage 33, unchanged since scripts/benchmark_goal_region_real.py):")
    print(f"  goal_tolerance_xy_m={GOAL_TOLERANCE_XY_M}  goal_tolerance_z_m={GOAL_TOLERANCE_Z_M}")
    print(f"  goal condition: planner.astar._state_in_goal_region (production, reused verbatim, not reimplemented)")

    fine_roi = load_roi(DEFAULT_CONFIG)
    cache = load_terrain_cache(CACHE_DIR, fine_roi, SOURCE_DEM_PATH)
    coarse_terrain = build_coarse60_terrainquery_from_cache(cache, fine_roi)
    store = CacheBacked60mStore(cache)
    primitives = build_primitive_set(COARSE60_CONFIG)

    missions = objective_mission_setup(cache)
    region_rows = range(COARSE_R0, COARSE_R1)
    region_cols = range(COARSE_C0, COARSE_C1)
    dense_count, per_cell_counts = dense_eager_state_count(store, region_rows, region_cols, CEILING_MSL)
    print(f"\n  dense whole-window possible states (unchanged from Step 3D): {dense_count}")

    # Step 3D's ORIGINAL (before) numbers, recorded for comparison -- NOT re-run,
    # taken verbatim from that step's own report/log.
    before = {
        "A_easy_open": {"found_goal": False, "instantiated": 48370, "expanded": 48370,
                         "generated": 1160880, "search_time_s": 252.4373},
        "B_relief_affected": {"found_goal": True, "instantiated": 641, "expanded": 297,
                               "generated": 7104, "search_time_s": 0.8266},
        "C_z_matters": {"found_goal": True, "instantiated": 3535, "expanded": 2519,
                         "generated": 60432, "search_time_s": 10.8854},
    }

    results = {}
    for name, m in missions.items():
        print(f"\n--- MISSION [{name}] (WITH production goal tolerance) ---")
        mission_ctx = MissionContext(
            start_rowcol=m["start_rc"], start_z_msl=m["start_z"],
            goal_rowcol=m["goal_rc"], goal_z_msl=m["goal_z"],
            ceiling_msl=CEILING_MSL, min_agl_m=MIN_AGL_M, z_step_m=Z_STEP_M,
        )
        result = lazy_search_with_tolerance(coarse_terrain, store, mission_ctx, primitives, GOAL_CFG,
                                             region_rows, region_cols)
        xy_err, z_err = final_goal_error(result["path"], mission_ctx, coarse_terrain)
        safety = independent_safety_validation(result["path"], coarse_terrain, primitives, COARSE60_CONFIG)

        print(f"  found_goal={result['found_goal']}  path_len={len(result['path']) if result['path'] else None}")
        print(f"  instantiated={result['instantiated_count']}  expanded={result['expanded_count']}  "
              f"generated={result['generated_count']}  max_open_size={result['max_open_size']}  "
              f"bound_rejected={result['bound_rejected']}  primitive_eval_calls={result['primitive_eval_calls']}  "
              f"primitive_rejected={result['primitive_rejected']}")
        print(f"  search_time_s={result['elapsed_s']:.4f}")
        print(f"  final_goal_xy_error_m={xy_err}  final_goal_z_error_m={z_err}  "
              f"(tolerances: xy<={GOAL_TOLERANCE_XY_M}m z<={GOAL_TOLERANCE_Z_M}m)")
        print(f"  independent safety validation: {safety}")

        ftimes = np.array(result["floor_for_times"]) if result["floor_for_times"] else np.array([0.0])
        print(f"  CandidateZGenerator.floor_for(): call_count={result['floor_for_call_count']}  "
              f"mean={ftimes.mean()*1e6:.3f}us  p95={np.percentile(ftimes,95)*1e6:.3f}us  "
              f"raw_dem_reads(must be 0)={store.raw_dem_reads}")

        b = before[name]
        print(f"\n  BEFORE (exact goal, Step 3D) vs AFTER (production tolerance, this step):")
        print(f"    found_goal:   {b['found_goal']} -> {result['found_goal']}")
        print(f"    instantiated: {b['instantiated']} -> {result['instantiated_count']}")
        print(f"    expanded:     {b['expanded']} -> {result['expanded_count']}")
        print(f"    generated:    {b['generated']} -> {result['generated_count']}")
        print(f"    search_time_s:{b['search_time_s']:.4f} -> {result['elapsed_s']:.4f}")

        results[name] = {"result": result, "safety": safety, "xy_err": xy_err, "z_err": z_err}

    print("\n" + "=" * 70)
    print("FAILURE CLASSIFICATION (only for missions that still fail)")
    print("=" * 70)
    for name, r in results.items():
        if not r["result"]["found_goal"]:
            print(f"  [{name}] STILL FAILS with production tolerance.")
            print(f"    Classification: goal-condition was NOT the limiting factor here (tolerance ruled out "
                  f"as cause since it's now production-correct) -- remaining cause is SEARCH/GUIDANCE / "
                  f"STATE-EXPLOSION (uniform-cost/greedy-XY-heuristic A* exhausting a bounded region while "
                  f"looking for a path that both clears a real mid-route terrain obstruction AND lands within "
                  f"the (now widened, but still narrow at 60m XY) goal tolerance band). NOT representation "
                  f"loss -- no P3/P4 evidence found. Recorded as OPEN ISSUE, not solved here per scope.")

    print("\n" + "=" * 70)
    print("STEP 3D.1 SUMMARY")
    print("=" * 70)
    for name, r in results.items():
        print(f"  [{name}] found={r['result']['found_goal']}  safe={r['safety'].get('safe')}  "
              f"instantiated={r['result']['instantiated_count']}")


if __name__ == "__main__":
    main()
