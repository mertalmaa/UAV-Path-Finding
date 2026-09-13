"""Roadmap Step 3E: production representation integration + regression.

Runs the SAME 3 real-terrain missions (A/B/C -- Step 3D's own objective
mission selection, `objective_mission_setup`, reused UNCHANGED) through the
REAL production planner.astar.astar_search() -- not the bespoke
lazy_search()/lazy_search_with_tolerance() test-harness functions Step
3D/3D.1 wrote for themselves -- under two arms differing ONLY in whether a
candidate_z_generator is supplied (Step CLEAN-1 removed the
representation_mode config flag; presence/absence of the generator IS the
mode now):

  "legacy" (baseline, unaffected by this step): terrain built FRESH via
    planner.coarse.build_coarse_dem (no persistent cache), candidate_z_
    generator=None -- no floor prefilter runs.

  "sparse_lazy" (new, this step): terrain built from the Step 3C persistent
    cache via planner.terrain_cache.build_terrain_query_from_cache, PLUS a
    planner.candidate_z.CandidateZGenerator (backed by
    CacheBackedTerrainMetadataStore) supplied to astar_search() as the new
    per-cell floor prefilter.

Both arms use the IDENTICAL production search otherwise: same cost_mode
(default "legacy", untouched), same heuristic flags (all defaults --
MSL-lower-bound + vertical-reachability heuristics ON, exactly as any other
production call), same primitive set, same start/goal/altitudes, same
max_expansions cap. The ONLY thing that differs between the two arms is the
representation layer (terrain source + the new floor prefilter) -- no
search heuristic/guidance/cost change is made anywhere in this file or in
planner/astar.py for this step.

Mission A is Step 3D/3D.1's own, already-diagnosed SEARCH/GUIDANCE open
issue (see project.md) -- it is run here for completeness/comparison only,
NOT as a pass/fail gate for Step 3E, and no attempt is made to fix it.
"""
import dataclasses
import time

from planner.astar import (
    _path_altitude_metrics, _path_min_observed_agl,
    astar_search, msl_to_z_index, validate_path_safety,
)
from planner.candidate_z import CacheBackedTerrainMetadataStore, CandidateZGenerator, MissionContext
from planner.coarse import build_coarse_dem
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from scripts.step3d_real_terrain_integration import (
    CACHE_DIR, CEILING_MSL, FACTOR60, MIN_AGL_M, SOURCE_DEM_PATH, objective_mission_setup,
)

COARSE60_PRODUCTION_CONFIG = dataclasses.replace(DEFAULT_CONFIG, xy_resolution_m=60.0, min_agl_m=MIN_AGL_M)
MAX_EXPANSIONS = 30_000


def run_one(name, m, terrain, config, primitives, candidate_z_generator=None):
    z0 = msl_to_z_index(m["start_z"], config)
    z1 = msl_to_z_index(m["goal_z"], config)
    start = (m["start_rc"][0], m["start_rc"][1], z0)
    goal = (m["goal_rc"][0], m["goal_rc"][1], z1)
    min_search = min(m["start_z"], m["goal_z"])
    max_search = CEILING_MSL

    t0 = time.perf_counter()
    result = astar_search(
        start, goal, terrain, min_search_altitude_msl=min_search, max_search_altitude_msl=max_search,
        config=config, primitives=primitives, max_expansions=MAX_EXPANSIONS,
        candidate_z_generator=candidate_z_generator,
    )
    wall = time.perf_counter() - t0

    safe = None
    min_agl = None
    max_angle = None
    if result.success and result.path:
        safe = validate_path_safety(result.path, primitives, terrain, config)
        min_agl = _path_min_observed_agl(result.path, primitives, terrain, config)
        alt = _path_altitude_metrics(result.path, terrain, config)
    else:
        alt = None

    goal_err_xy = goal_err_z = None
    if result.path:
        from planner.astar import state_to_xyz
        gx, gy, gz = state_to_xyz(goal, terrain, config)
        fx, fy, fz = state_to_xyz(result.path[-1], terrain, config)
        goal_err_xy = ((fx - gx) ** 2 + (fy - gy) ** 2) ** 0.5
        goal_err_z = abs(fz - gz)

    return {
        "name": name, "wall_s": wall, "status": result.status, "success": result.success,
        "path_len": len(result.path) if result.path else None,
        "expanded": result.expanded_nodes, "generated": result.generated_neighbors,
        "max_open": result.max_open_size,
        "cache_hit_rate": result.primitive_cache_hit_rate,
        "actual_evaluate_primitive_calls": result.actual_evaluate_primitive_calls,
        "below_floor_rejects": result.rejected_reason_counts.get("below_terrain_floor_sparse", 0),
        "total_cost": result.total_cost, "runtime_s": result.runtime_s,
        "safe": safe, "min_agl": min_agl, "max_angle": max_angle,
        "goal_err_xy": goal_err_xy, "goal_err_z": goal_err_z,
    }


def main():
    print("=" * 70)
    print("STEP 3E -- production representation_mode integration + regression")
    print("=" * 70)
    print("\nDISCLOSURE: same placeholder z_step-derived primitives as every earlier step (NOT real")
    print("aircraft performance data). Heading OFF. CLASS-C motion events remain unresolved/provisional.")
    print("60m XY remains a PROVISIONAL geometric planning candidate. No search heuristic/cost/guidance")
    print("change is made in this step -- ONLY the representation layer (terrain source + floor prefilter).")

    t0 = time.perf_counter()
    fine_roi = load_roi(DEFAULT_CONFIG)
    load_roi_s = time.perf_counter() - t0
    print(f"\n--- shared step: load_roi() = {load_roi_s:.4f}s (identical, done once, in BOTH arms below) ---")

    print("\n--- LEGACY terrain setup (fresh build_coarse_dem, no cache) ---")
    t0 = time.perf_counter()
    legacy_terrain = TerrainQuery(build_coarse_dem(fine_roi, FACTOR60).roi)
    legacy_setup_s = time.perf_counter() - t0
    print(f"  build_coarse_dem(factor={FACTOR60}) = {legacy_setup_s:.4f}s  shape={legacy_terrain.roi.elevation.shape}")

    print("\n--- SPARSE_LAZY terrain setup (Step 3C persistent cache) ---")
    t0 = time.perf_counter()
    cache = load_terrain_cache(CACHE_DIR, fine_roi, SOURCE_DEM_PATH)
    sparse_terrain = build_terrain_query_from_cache(cache, fine_roi, FACTOR60)
    sparse_setup_s = time.perf_counter() - t0
    print(f"  load_terrain_cache()+build_terrain_query_from_cache() = {sparse_setup_s:.4f}s  "
          f"shape={sparse_terrain.roi.elevation.shape}")
    import numpy as np
    same_array = np.array_equal(legacy_terrain.roi.elevation, sparse_terrain.roi.elevation)
    print(f"  legacy vs sparse_lazy terrain arrays bit-identical: {same_array}")

    primitives = build_primitive_set(COARSE60_PRODUCTION_CONFIG)
    missions = objective_mission_setup(cache)
    for name, m in missions.items():
        print(f"\n[{name}] criterion: {m['criterion']}")
        print(f"  start_rc={m['start_rc']} start_z={m['start_z']}m  goal_rc={m['goal_rc']} goal_z={m['goal_z']}m")

    results = {"legacy": {}, "sparse_lazy": {}}
    for name, m in missions.items():
        print(f"\n{'='*70}\nMISSION [{name}]\n{'='*70}")

        r_legacy = run_one(name, m, legacy_terrain, COARSE60_PRODUCTION_CONFIG, primitives, candidate_z_generator=None)
        print(f"  LEGACY:      status={r_legacy['status']:>20}  success={r_legacy['success']}  "
              f"expanded={r_legacy['expanded']}  max_open={r_legacy['max_open']}  "
              f"runtime={r_legacy['runtime_s']:.3f}s  cache_hit={r_legacy['cache_hit_rate']:.3f}")
        if r_legacy["success"]:
            print(f"               path_len={r_legacy['path_len']}  cost={r_legacy['total_cost']:.2f}  "
                  f"safe={r_legacy['safe']}  min_agl={r_legacy['min_agl']:.1f}  "
                  f"goal_err_xy={r_legacy['goal_err_xy']:.2f}m  goal_err_z={r_legacy['goal_err_z']:.2f}m")

        mission_ctx = MissionContext(
            start_rowcol=m["start_rc"], start_z_msl=m["start_z"],
            goal_rowcol=m["goal_rc"], goal_z_msl=m["goal_z"],
            ceiling_msl=CEILING_MSL, min_agl_m=MIN_AGL_M, z_step_m=COARSE60_PRODUCTION_CONFIG.z_step_m,
        )
        store = CacheBackedTerrainMetadataStore(cache, FACTOR60)
        generator = CandidateZGenerator(store, mission_ctx)
        r_sparse = run_one(name, m, sparse_terrain, COARSE60_PRODUCTION_CONFIG, primitives, candidate_z_generator=generator)
        print(f"  SPARSE_LAZY: status={r_sparse['status']:>20}  success={r_sparse['success']}  "
              f"expanded={r_sparse['expanded']}  max_open={r_sparse['max_open']}  "
              f"runtime={r_sparse['runtime_s']:.3f}s  cache_hit={r_sparse['cache_hit_rate']:.3f}  "
              f"raw_dem_reads={store.raw_dem_reads}  below_floor_rejects={r_sparse['below_floor_rejects']}")
        if r_sparse["success"]:
            print(f"               path_len={r_sparse['path_len']}  cost={r_sparse['total_cost']:.2f}  "
                  f"safe={r_sparse['safe']}  min_agl={r_sparse['min_agl']:.1f}  "
                  f"goal_err_xy={r_sparse['goal_err_xy']:.2f}m  goal_err_z={r_sparse['goal_err_z']:.2f}m")

        same_path = r_legacy["path_len"] == r_sparse["path_len"] and r_legacy["success"] == r_sparse["success"]
        same_cost = (r_legacy["total_cost"] == r_sparse["total_cost"]) if (r_legacy["success"] and r_sparse["success"]) else None
        print(f"  COMPARISON: same success={r_legacy['success'] == r_sparse['success']}  "
              f"same path_len={same_path}  same total_cost={same_cost}  "
              f"expanded delta={r_sparse['expanded'] - r_legacy['expanded']}  "
              f"evaluate_primitive_calls delta={r_sparse['actual_evaluate_primitive_calls'] - r_legacy['actual_evaluate_primitive_calls']}")

        results["legacy"][name] = r_legacy
        results["sparse_lazy"][name] = r_sparse

    print(f"\n{'='*70}\nSTEP 3E SUMMARY\n{'='*70}")
    print(f"  load_roi_s={load_roi_s:.4f}  legacy_terrain_setup_s={legacy_setup_s:.4f}  "
          f"sparse_terrain_setup_s={sparse_setup_s:.4f}  terrain_arrays_identical={same_array}")
    for name in missions:
        rl, rs = results["legacy"][name], results["sparse_lazy"][name]
        print(f"  [{name}] legacy: success={rl['success']} expanded={rl['expanded']} runtime={rl['runtime_s']:.2f}s | "
              f"sparse_lazy: success={rs['success']} expanded={rs['expanded']} runtime={rs['runtime_s']:.2f}s "
              f"raw_dem_reads=0 below_floor_rejects={rs['below_floor_rejects']}")


if __name__ == "__main__":
    main()
