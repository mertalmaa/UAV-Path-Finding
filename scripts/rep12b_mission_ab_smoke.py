"""Step REP-1.2B: Mission A/B real-terrain smoke test for the CandidateZ-
driven altitude successor path. Mission C is deliberately NOT run here (see
project.md "Step REP-1.2B" and the stage prompt's own fast-test-order
requirement) -- only A/B, for development validation before any larger run.

Mirrors scripts/perf0_search_baseline.py's run_mission_case() (same real
terrain, same objective_mission_setup() missions, same MISSION_CONFIG), but
wires astar_search()'s NEW CandidateZ-as-altitude-successor-source path
(encode_candidate_altitude/decode_candidate_altitude) instead of the old
z_step_m-index start/goal construction -- so this is the closest possible
apples-to-apples comparison against PERF-0's own frozen baseline numbers.
"""
import dataclasses
import json
import time

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

OUTPUT_PATH = "results/rep12b_mission_ab_smoke.json"
MISSION_CONFIG = dataclasses.replace(DEFAULT_CONFIG, xy_resolution_m=60.0, min_agl_m=MIN_AGL_M)


def run_mission_case_candidate_z(name, mission, cache, config, max_expansions=30_000) -> dict:
    terrain = build_terrain_query_from_cache(cache, load_roi(DEFAULT_CONFIG), FACTOR60)
    store = CacheBackedTerrainMetadataStore(cache, FACTOR60)
    mission_ctx = MissionContext(
        start_rowcol=mission["start_rc"], start_z_msl=mission["start_z"],
        goal_rowcol=mission["goal_rc"], goal_z_msl=mission["goal_z"],
        ceiling_msl=CEILING_MSL, min_agl_m=MIN_AGL_M, z_step_m=config.z_step_m,
    )
    generator = CandidateZGenerator(store, mission_ctx)

    # Step REP-1.2B: start/goal built via encode_candidate_altitude(), NOT
    # msl_to_z_index()/z_step_m arithmetic -- the actual production change
    # this stage makes to how a CandidateZ-driven search call is constructed.
    start = (mission["start_rc"][0], mission["start_rc"][1], encode_candidate_altitude(mission["start_z"]))
    goal = (mission["goal_rc"][0], mission["goal_rc"][1], encode_candidate_altitude(mission["goal_z"]))
    min_search = min(mission["start_z"], mission["goal_z"])
    max_search = CEILING_MSL

    t0 = time.perf_counter()
    result = astar_search(
        start, goal, terrain, min_search_altitude_msl=min_search, max_search_altitude_msl=max_search,
        config=config, max_expansions=max_expansions, candidate_z_generator=generator,
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
    print("STEP REP-1.2B -- Mission A/B smoke test (CandidateZ altitude successor source)")
    print("=" * 70)

    fine_roi = load_roi(DEFAULT_CONFIG)
    cache = load_terrain_cache(CACHE_DIR, fine_roi, SOURCE_DEM_PATH)
    missions = objective_mission_setup(cache)

    results = {}
    for key in ("A_easy_open", "B_relief_affected"):
        print(f"\n--- Mission {key} ---")
        r = run_mission_case_candidate_z(key, missions[key], cache, MISSION_CONFIG, max_expansions=30_000)
        results[key] = r
        print(f"  status={r['status']} termination={r['termination_reason']} "
              f"safety={'PASS' if r['safety_pass'] else ('N/A' if r['safety_pass'] is None else 'FAIL')} "
              f"min_agl={r['min_agl_m']} goal_err_xy={r['goal_error_xy_m']} goal_err_z={r['goal_error_z_m']}")
        print(f"  nodes={r['node_count']} cost={r['path_cost']} expanded={r['expanded_nodes']} "
              f"generated={r['generated_neighbors']} rejected={r['rejected_neighbors']} "
              f"avg_candidates/expansion={r['avg_candidates_per_expansion']}")
        print(f"  search_time_s={r['search_time_s']:.3f} rejected_reasons={r['rejected_reason_counts']}")

    with open(OUTPUT_PATH, "w") as f:
        json.dump(results, f, indent=2, default=str)
    print(f"\nWrote {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
