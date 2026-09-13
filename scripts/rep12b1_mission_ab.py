"""Step REP-1.2B.1: Mission A then B real-terrain run with the multi-cell
vertical-motion horizon bridge (aircraft_profile wired into astar_search()).
Mission C is deliberately NOT run here. Mirrors scripts/rep12b_mission_ab_
smoke.py exactly, adding only the aircraft_profile argument -- so any
change in outcome is attributable to REP-1.2B.1's addition alone.
"""
import dataclasses
import json
import time

from planner.aircraft_profile import load_aircraft_profile
from planner.astar import (
    _path_altitude_metrics,
    _path_min_observed_agl,
    astar_search,
    encode_candidate_altitude,
    state_to_xyz,
    validate_path_safety,
)
from planner.candidate_z import CacheBackedTerrainMetadataStore, CandidateZGenerator, MissionContext
from planner.config import DEFAULT_CONFIG
from planner.roi import load_roi
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from scripts.step3d_real_terrain_integration import (
    CACHE_DIR, CEILING_MSL, FACTOR60, MIN_AGL_M, SOURCE_DEM_PATH, objective_mission_setup,
)

OUTPUT_PATH = "results/rep12b1_mission_ab.json"
AIRCRAFT_PROFILE_PATH = "jsbsim/results/c172p_aircraft_profile_planner_safe_v3.json"
MISSION_CONFIG = dataclasses.replace(DEFAULT_CONFIG, xy_resolution_m=60.0, min_agl_m=MIN_AGL_M)


def run_mission_case(name, mission, cache, config, aircraft_profile, max_expansions=30_000, max_search_time_s=None) -> dict:
    terrain = build_terrain_query_from_cache(cache, load_roi(DEFAULT_CONFIG), FACTOR60)
    store = CacheBackedTerrainMetadataStore(cache, FACTOR60)
    mission_ctx = MissionContext(
        start_rowcol=mission["start_rc"], start_z_msl=mission["start_z"],
        goal_rowcol=mission["goal_rc"], goal_z_msl=mission["goal_z"],
        ceiling_msl=CEILING_MSL, min_agl_m=MIN_AGL_M, z_step_m=config.z_step_m,
    )
    generator = CandidateZGenerator(store, mission_ctx)

    start = (mission["start_rc"][0], mission["start_rc"][1], encode_candidate_altitude(mission["start_z"]))
    goal = (mission["goal_rc"][0], mission["goal_rc"][1], encode_candidate_altitude(mission["goal_z"]))
    min_search = min(mission["start_z"], mission["goal_z"])
    max_search = CEILING_MSL

    t0 = time.perf_counter()
    result = astar_search(
        start, goal, terrain, min_search_altitude_msl=min_search, max_search_altitude_msl=max_search,
        config=config, max_expansions=max_expansions, max_search_time_s=max_search_time_s,
        candidate_z_generator=generator, aircraft_profile=aircraft_profile,
    )
    search_time_s = time.perf_counter() - t0

    safe = validate_path_safety(result.path, [], terrain, config, generator) if result.path else None
    min_agl = _path_min_observed_agl(result.path, [], terrain, config, generator) if result.path else None
    alt = _path_altitude_metrics(result.path, terrain, config, generator) if result.path else None

    goal_error_z_m = None
    goal_error_xy_m = None
    if result.path:
        gx, gy, gz = state_to_xyz(goal, terrain, config, generator)
        fx, fy, fz = state_to_xyz(result.path[-1], terrain, config, generator)
        goal_error_xy_m = ((fx - gx) ** 2 + (fy - gy) ** 2) ** 0.5
        goal_error_z_m = abs(fz - gz)

    return {
        "name": name,
        "status": result.status,
        "termination_reason": result.termination_reason,
        "success": result.success,
        "safety_pass": safe,
        "min_agl_m": min_agl,
        "goal_error_xy_m": goal_error_xy_m,
        "goal_error_z_m": goal_error_z_m,
        "node_count": len(result.path) if result.path else 0,
        "path_cost": result.total_cost if result.path else None,
        "expanded_nodes": result.expanded_nodes,
        "generated_neighbors": result.generated_neighbors,
        "rejected_neighbors": result.rejected_neighbors,
        "rejected_reason_counts": result.rejected_reason_counts,
        "search_time_s": search_time_s,
        "altitude_metrics": alt,
        "avg_candidates_per_expansion": (
            result.generated_neighbors / result.expanded_nodes if result.expanded_nodes else None
        ),
    }


def main() -> None:
    print("=" * 70)
    print("STEP REP-1.2B.1 -- Mission A then B (multi-cell vertical-motion horizon bridge)")
    print("=" * 70)

    aircraft_profile = load_aircraft_profile(AIRCRAFT_PROFILE_PATH)
    fine_roi = load_roi(DEFAULT_CONFIG)
    cache = load_terrain_cache(CACHE_DIR, fine_roi, SOURCE_DEM_PATH)
    missions = objective_mission_setup(cache)

    results = {}

    print("\n--- Mission A_easy_open ---")
    r = run_mission_case("A_easy_open", missions["A_easy_open"], cache, MISSION_CONFIG, aircraft_profile,
                          max_expansions=30_000, max_search_time_s=300.0)
    results["A_easy_open"] = r
    print(f"  status={r['status']} termination={r['termination_reason']} "
          f"safety={'PASS' if r['safety_pass'] else ('N/A' if r['safety_pass'] is None else 'FAIL')} "
          f"min_agl={r['min_agl_m']} goal_err_xy={r['goal_error_xy_m']} goal_err_z={r['goal_error_z_m']}")
    print(f"  nodes={r['node_count']} cost={r['path_cost']} expanded={r['expanded_nodes']} "
          f"generated={r['generated_neighbors']} rejected={r['rejected_neighbors']} "
          f"avg_candidates/expansion={r['avg_candidates_per_expansion']}")
    print(f"  search_time_s={r['search_time_s']:.3f} rejected_reasons={r['rejected_reason_counts']}")

    if r["success"] and r["safety_pass"]:
        print("\n--- Mission A PASS+safe -- proceeding to Mission B ---")
        r2 = run_mission_case("B_relief_affected", missions["B_relief_affected"], cache, MISSION_CONFIG, aircraft_profile,
                               max_expansions=30_000, max_search_time_s=300.0)
        results["B_relief_affected"] = r2
        print(f"  status={r2['status']} termination={r2['termination_reason']} "
              f"safety={'PASS' if r2['safety_pass'] else ('N/A' if r2['safety_pass'] is None else 'FAIL')} "
              f"min_agl={r2['min_agl_m']} goal_err_xy={r2['goal_error_xy_m']} goal_err_z={r2['goal_error_z_m']}")
        print(f"  nodes={r2['node_count']} cost={r2['path_cost']} expanded={r2['expanded_nodes']} "
              f"generated={r2['generated_neighbors']} rejected={r2['rejected_neighbors']} "
              f"avg_candidates/expansion={r2['avg_candidates_per_expansion']}")
        print(f"  search_time_s={r2['search_time_s']:.3f} rejected_reasons={r2['rejected_reason_counts']}")
    else:
        print("\n--- Mission A did not succeed/safe -- Mission B NOT run (per stage instructions) ---")

    with open(OUTPUT_PATH, "w") as f:
        json.dump(results, f, indent=2, default=str)
    print(f"\nWrote {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
