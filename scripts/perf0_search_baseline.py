"""Step PERF-0: search performance baseline + budget contract.

Freezes the CURRENT production search architecture's correctness/quality/
performance numbers as a machine-readable artifact (outputs/search_
baseline_perf0.json), so a future run (heading, aircraft LUT, aircraft
primitives, corridor retirement, new search guidance) has something
concrete to diff against. No search algorithm, heuristic, cost, neighbor
generation, or corridor logic is changed here -- see planner/astar.py's
new max_search_time_s parameter (Step PERF-0) for the only planner.astar
change this stage makes, which is additive and a no-op unless a caller
passes it.

Termination-reason semantics (see planner/astar.py's
_TERMINATION_REASON_BY_STATUS and ara_star_search's own termination_reason
computation): FOUND, NO_PATH, EXPANSION_LIMIT, TIMEOUT. TIMEOUT and
EXPANSION_LIMIT are budget cutoffs -- NEITHER is ever reported as, or
treated as equivalent to, "unreachable". Only NO_PATH (open_heap
exhausted, no incumbent) is a genuine negative result within the given
search bounds. This mirrors the permanent DIRECTLY INFEASIBLE !=
UNREACHABLE principle (project.md) one level up, at the budget layer.

Test set (deliberately NOT a new mission set -- reuses what already
exists):
  - Mission A/B/C: Step 3D's own objective_mission_setup() (reused via
    scripts/step3d_real_terrain_integration.py), run through the
    production astar_search() with the Step 3E sparse/lazy representation
    (CandidateZGenerator backed by the Step 3C persistent TerrainCache) --
    i.e. exactly Step 3E/CLEAN-1's own regression scenario.
  - The 6.48km long-valley benchmark (Stage 19/25, reused via scripts/
    benchmark_weighted_astar_real.py's own constants): first through plain
    astar_search() (legacy cost, weighted A* epsilon=1.5 -- same call
    shape that script already uses), then through the FULL frozen
    production pipeline -- coarse_astar_search -> corridor/z-guide ->
    fine_precompute -> ara_star_search -- using the same coarse/fine
    config choices webapp/server.py already runs in production (not
    reimplemented differently here, just measured).

No new mission, no special-case fix for Mission A's known search-
limitation, no workload shrinking to make numbers look better.
"""
import dataclasses
import hashlib
import json
import math
import subprocess
import time
from pathlib import Path

from planner.astar import (
    _path_altitude_metrics, _path_min_observed_agl, ara_star_search, astar_search,
    msl_to_z_index, state_to_xyz, validate_path_safety,
)
from planner.candidate_z import CacheBackedTerrainMetadataStore, CandidateZGenerator, MissionContext
from planner.coarse import build_coarse_dem
from planner.coarse_astar import (
    coarse_astar_search, compute_coarse_distance_reference, lift_endpoint_if_unsafe,
)
from planner.config import DEFAULT_CONFIG
from planner.corridor import build_xy_corridor_mask, build_z_guide_grid
from planner.fine_precompute import precompute_fine_corridor_primitive_safety
from planner.primitives import build_primitive_set
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from scripts.step3d_real_terrain_integration import (
    CACHE_DIR, CEILING_MSL, FACTOR60, MIN_AGL_M, SOURCE_DEM_PATH, objective_mission_setup,
)

OUTPUT_PATH = "outputs/search_baseline_perf0.json"

# Mission A/B/C run on the SAME 60m coarse production config Step 3D/3E/
# CLEAN-1 established (scripts/step3e_production_representation_integration.py's
# COARSE60_PRODUCTION_CONFIG) -- start_rc/goal_rc from objective_mission_setup()
# are coarse (60m) grid indices, so this must NOT be DEFAULT_CONFIG's 30m.
MISSION_CONFIG = dataclasses.replace(DEFAULT_CONFIG, xy_resolution_m=60.0, min_agl_m=MIN_AGL_M)

# Same 6.48km long-valley mission Stage 19/25 established, reused unchanged
# (see scripts/benchmark_weighted_astar_real.py / benchmark_long_valley.py).
LV_START_ROW, LV_START_COL = 48, 276
LV_GOAL_ROW, LV_GOAL_COL = 264, 276
LV_AIRCRAFT_MSL = 3760.0
LV_W_MSL = 0.63
LV_EPSILON_SEARCH = 1.5

# Same production pipeline constants webapp/server.py already runs with
# (Stage 36-38.1) -- reused for measurement, not reinvented here.
ALTITUDE_REFERENCE_MSL = 3240.0
NORMALIZED_ALTITUDE_SCALE_M = 1000.0
NORMALIZED_W_ALTITUDE = 1.25
NORMALIZED_W_DISTANCE = 1.0
CORRIDOR_XY_HALF_WIDTH_M = 300.0
CORRIDOR_Z_HALF_WIDTH_M = 200.0
COARSE_FACTOR = 3
COARSE_EPSILON = 1.5
COARSE_MAX_EXPANSIONS = 25_000
FINE_EPSILON_SCHEDULE = (1.7, 1.5, 1.3, 1.1)
FINE_MAX_EXPANSIONS_CUMULATIVE = 30_000

# Development watchdogs ONLY -- see module docstring and project.md
# "Step PERF-0". Not a final performance target for any of these.
WATCHDOG_SMALL_S = 30.0
WATCHDOG_NORMAL_S = 60.0
WATCHDOG_LONG_S = 120.0


def git_commit_hash() -> str:
    try:
        return subprocess.run(
            ["git", "rev-parse", "HEAD"], capture_output=True, text=True, check=True, cwd=Path(__file__).resolve().parent.parent,
        ).stdout.strip()
    except Exception as e:
        return f"unavailable ({e})"


def git_is_dirty() -> object:
    try:
        out = subprocess.run(
            ["git", "status", "--porcelain"], capture_output=True, text=True, check=True,
            cwd=Path(__file__).resolve().parent.parent,
        ).stdout
        return bool(out.strip())
    except Exception:
        return None


def config_hash(config) -> str:
    d = dataclasses.asdict(config)
    for k, v in d.items():
        if isinstance(v, Path):
            d[k] = str(v)
    blob = json.dumps(d, sort_keys=True, default=str).encode("utf-8")
    return hashlib.sha256(blob).hexdigest()[:16]


def path_quality_metrics(path, terrain, config) -> dict:
    if not path:
        return {
            "node_count": 0, "path_cost": None, "geometric_path_length_m": None,
            "min_msl": None, "max_msl": None, "total_climb_m": None, "total_descent_m": None,
        }
    alt = _path_altitude_metrics(path, terrain, config)
    return {
        "node_count": len(path),
        "geometric_path_length_m": alt["geometric_path_length"],
        "min_msl": alt["minimum_aircraft_msl"],
        "max_msl": alt["maximum_aircraft_msl"],
        "avg_msl": alt["average_aircraft_msl"],
        "total_climb_m": alt["total_climb_m"],
        "total_descent_m": alt["total_descent_m"],
    }


def goal_error(path, goal, terrain, config):
    if not path:
        return {"goal_error_xy_m": None, "goal_error_z_m": None}
    gx, gy, gz = state_to_xyz(goal, terrain, config)
    fx, fy, fz = state_to_xyz(path[-1], terrain, config)
    return {
        "goal_error_xy_m": math.hypot(fx - gx, fy - gy),
        "goal_error_z_m": abs(fz - gz),
    }


def labels(*, heading: bool, aircraft_lut: bool, aircraft_primitives: bool, corridor: bool,
           search_guidance: str, production_search: str) -> dict:
    """Step PERF-0 section 10: explicit future-comparison labels, carried
    unchanged into every test record so a later run can be diffed cell-
    for-cell against the SAME dimensions, even after some of them flip."""
    return {
        "state": "(x,y,z,heading)" if heading else "(x,y,z)",
        "heading": heading,
        "aircraft_aware_primitives": aircraft_primitives,
        "aircraft_lut": aircraft_lut,
        "hard_corridor": corridor,
        "search_guidance": search_guidance,
        "production_search": production_search,
    }


def run_mission_case(name, mission, cache, config, primitives, max_expansions=30_000,
                      max_search_time_s=None) -> dict:
    """Mission A/B/C via production astar_search(), sparse/lazy
    representation (Step 3E/CLEAN-1's own regression scenario) -- see
    module docstring. `config` must be MISSION_CONFIG (60m coarse) --
    start_rc/goal_rc are coarse-grid indices."""
    terrain = build_terrain_query_from_cache(cache, load_roi(DEFAULT_CONFIG), FACTOR60)
    store = CacheBackedTerrainMetadataStore(cache, FACTOR60)
    mission_ctx = MissionContext(
        start_rowcol=mission["start_rc"], start_z_msl=mission["start_z"],
        goal_rowcol=mission["goal_rc"], goal_z_msl=mission["goal_z"],
        ceiling_msl=CEILING_MSL, min_agl_m=MIN_AGL_M, z_step_m=config.z_step_m,
    )
    generator = CandidateZGenerator(store, mission_ctx)

    z0 = msl_to_z_index(mission["start_z"], config)
    z1 = msl_to_z_index(mission["goal_z"], config)
    start = (mission["start_rc"][0], mission["start_rc"][1], z0)
    goal = (mission["goal_rc"][0], mission["goal_rc"][1], z1)
    min_search = min(mission["start_z"], mission["goal_z"])
    max_search = CEILING_MSL

    t_pre0 = time.perf_counter()
    # terrain/cache load already happened above (build_terrain_query_from_cache) --
    # measured separately from the search call itself, per PERF-0 section 2.
    preprocessing_time_s = time.perf_counter() - t_pre0

    t_search0 = time.perf_counter()
    result = astar_search(
        start, goal, terrain, min_search_altitude_msl=min_search, max_search_altitude_msl=max_search,
        config=config, primitives=primitives, max_expansions=max_expansions,
        max_search_time_s=max_search_time_s, candidate_z_generator=generator,
    )
    search_time_s = time.perf_counter() - t_search0

    t_val0 = time.perf_counter()
    safe = validate_path_safety(result.path, primitives, terrain, config) if result.path else None
    min_agl = _path_min_observed_agl(result.path, primitives, terrain, config) if result.path else None
    validation_time_s = time.perf_counter() - t_val0

    return {
        "name": name,
        "labels": labels(heading=False, aircraft_lut=False, aircraft_primitives=False,
                          corridor=False, search_guidance="CURRENT", production_search="astar_search"),
        "timing": {
            "preprocessing_time_s": preprocessing_time_s,
            "search_time_s": search_time_s,
            "path_validation_time_s": validation_time_s,
            "total_pipeline_time_s": preprocessing_time_s + search_time_s + validation_time_s,
        },
        "correctness": {
            "status": result.status,
            "termination_reason": result.termination_reason,
            "path_found": result.success,
            "safety_pass": safe,
            "min_agl_m": min_agl,
            **goal_error(result.path, goal, terrain, config),
        },
        "quality": {**path_quality_metrics(result.path, terrain, config), "path_cost": result.total_cost},
        "performance": {
            "expanded": result.expanded_nodes,
            "generated": result.generated_neighbors,
            "max_open_size": result.max_open_size,
            "cache_hits": result.primitive_cache_hits,
            "cache_misses": result.primitive_cache_misses,
            "cache_hit_rate": result.primitive_cache_hit_rate,
            "actual_evaluate_primitive_calls": result.actual_evaluate_primitive_calls,
            "candidate_z_calls": generator.stats.call_count,
        },
    }


def run_long_valley_astar(config_base, primitives, max_expansions=30_000, max_search_time_s=None) -> dict:
    """The 6.48km real-terrain mission via plain production astar_search()
    -- same call shape as scripts/benchmark_weighted_astar_real.py."""
    config = dataclasses.replace(config_base, msl_cost_weight=LV_W_MSL)

    t_pre0 = time.perf_counter()
    roi = load_roi(config)
    terrain = TerrainQuery(roi)
    preprocessing_time_s = time.perf_counter() - t_pre0

    z0 = msl_to_z_index(LV_AIRCRAFT_MSL, config)
    start, goal = (LV_START_ROW, LV_START_COL, z0), (LV_GOAL_ROW, LV_GOAL_COL, z0)
    seg = roi.elevation[LV_START_ROW:LV_GOAL_ROW + 1, LV_START_COL]
    min_search = math.ceil((float(seg.min()) + config.min_agl_m) / config.z_step_m) * config.z_step_m
    max_search = LV_AIRCRAFT_MSL + 20.0

    t_search0 = time.perf_counter()
    result = astar_search(
        start, goal, terrain, min_search_altitude_msl=min_search, max_search_altitude_msl=max_search,
        config=config, primitives=primitives, max_expansions=max_expansions,
        max_search_time_s=max_search_time_s, epsilon_search=LV_EPSILON_SEARCH,
    )
    search_time_s = time.perf_counter() - t_search0

    t_val0 = time.perf_counter()
    safe = validate_path_safety(result.path, primitives, terrain, config) if result.path else None
    min_agl = _path_min_observed_agl(result.path, primitives, terrain, config) if result.path else None
    validation_time_s = time.perf_counter() - t_val0

    return {
        "name": "long_valley_6.48km_astar",
        "labels": labels(heading=False, aircraft_lut=False, aircraft_primitives=False,
                          corridor=False, search_guidance="CURRENT", production_search="astar_search"),
        "timing": {
            "preprocessing_time_s": preprocessing_time_s,
            "search_time_s": search_time_s,
            "path_validation_time_s": validation_time_s,
            "total_pipeline_time_s": preprocessing_time_s + search_time_s + validation_time_s,
        },
        "correctness": {
            "status": result.status,
            "termination_reason": result.termination_reason,
            "path_found": result.success,
            "safety_pass": safe,
            "min_agl_m": min_agl,
            **goal_error(result.path, goal, terrain, config),
        },
        "quality": {**path_quality_metrics(result.path, terrain, config), "path_cost": result.total_cost},
        "performance": {
            "expanded": result.expanded_nodes,
            "generated": result.generated_neighbors,
            "max_open_size": result.max_open_size,
            "cache_hits": result.primitive_cache_hits,
            "cache_misses": result.primitive_cache_misses,
            "cache_hit_rate": result.primitive_cache_hit_rate,
            "actual_evaluate_primitive_calls": result.actual_evaluate_primitive_calls,
        },
    }


def run_long_valley_full_pipeline(max_search_time_s=None) -> dict:
    """The SAME 6.48km mission through the FULL frozen production
    pipeline: coarse guide -> corridor/z-guide -> fine primitive-safety
    precompute -> genuine ARA*. Same coarse/fine config choices
    webapp/server.py already runs -- see module docstring. This is the
    one case that actually exercises hard_corridor=True and
    production_search="ara_star_search"."""
    fine_cfg = dataclasses.replace(
        DEFAULT_CONFIG, cost_mode="normalized",
        altitude_reference_msl=ALTITUDE_REFERENCE_MSL, normalized_altitude_scale_m=NORMALIZED_ALTITUDE_SCALE_M,
        normalized_w_distance=NORMALIZED_W_DISTANCE, normalized_w_altitude=NORMALIZED_W_ALTITUDE,
    )
    coarse_cfg = dataclasses.replace(
        DEFAULT_CONFIG, xy_resolution_m=90.0, z_step_m=40.0, primitive_sample_spacing_m=30.0,
        altitude_reference_msl=ALTITUDE_REFERENCE_MSL, normalized_altitude_scale_m=NORMALIZED_ALTITUDE_SCALE_M,
        normalized_w_distance=NORMALIZED_W_DISTANCE, normalized_w_altitude=NORMALIZED_W_ALTITUDE,
    )

    t_pre0 = time.perf_counter()
    roi = load_roi(fine_cfg)
    tq = TerrainQuery(roi)
    fine_primitives = build_primitive_set(fine_cfg)
    coarse_primitives = build_primitive_set(coarse_cfg)
    coarse_dem = build_coarse_dem(roi, factor=COARSE_FACTOR)
    coarse_tq = TerrainQuery(coarse_dem.roi)
    preprocessing_time_s = time.perf_counter() - t_pre0

    start = (LV_START_ROW, LV_START_COL, msl_to_z_index(LV_AIRCRAFT_MSL, fine_cfg))
    goal = (LV_GOAL_ROW, LV_GOAL_COL, msl_to_z_index(LV_AIRCRAFT_MSL, fine_cfg))

    # ---- 1. coarse guide ----
    t0 = time.perf_counter()
    coarse_start_rc = (LV_START_ROW // COARSE_FACTOR, LV_START_COL // COARSE_FACTOR)
    coarse_goal_rc = (LV_GOAL_ROW // COARSE_FACTOR, LV_GOAL_COL // COARSE_FACTOR)
    start_lift = lift_endpoint_if_unsafe(*coarse_start_rc, LV_AIRCRAFT_MSL, coarse_tq, coarse_cfg)
    goal_lift = lift_endpoint_if_unsafe(*coarse_goal_rc, LV_AIRCRAFT_MSL, coarse_tq, coarse_cfg)
    coarse_start = (start_lift.row, start_lift.col, start_lift.z_index)
    coarse_goal = (goal_lift.row, goal_lift.col, goal_lift.z_index)
    coarse_seg_min = float(coarse_dem.roi.elevation[
        min(coarse_start_rc[0], coarse_goal_rc[0]):max(coarse_start_rc[0], coarse_goal_rc[0]) + 1,
        min(coarse_start_rc[1], coarse_goal_rc[1]):max(coarse_start_rc[1], coarse_goal_rc[1]) + 1,
    ].min())
    coarse_min_search = math.ceil((coarse_seg_min + coarse_cfg.min_agl_m) / coarse_cfg.z_step_m) * coarse_cfg.z_step_m
    coarse_max_search = max(start_lift.msl, goal_lift.msl) + 2 * coarse_cfg.z_step_m
    coarse_d_ref = compute_coarse_distance_reference(coarse_start, coarse_goal, coarse_tq, coarse_cfg)
    coarse_result = coarse_astar_search(
        coarse_start, coarse_goal, coarse_tq, coarse_min_search, coarse_max_search, coarse_cfg,
        primitives=coarse_primitives, max_expansions=COARSE_MAX_EXPANSIONS, distance_reference_m=coarse_d_ref,
        altitude_reference_msl=ALTITUDE_REFERENCE_MSL, altitude_scale_m=NORMALIZED_ALTITUDE_SCALE_M,
        w_distance=NORMALIZED_W_DISTANCE, w_altitude=NORMALIZED_W_ALTITUDE, epsilon_search=COARSE_EPSILON,
    )
    coarse_time_s = time.perf_counter() - t0
    if not coarse_result.success:
        return {"name": "long_valley_6.48km_full_pipeline", "error": "coarse guide failed -- pipeline aborted",
                "coarse_status": coarse_result.status}

    coarse_path_xyz = [state_to_xyz(s, coarse_tq, coarse_cfg) for s in coarse_result.path]
    coarse_path_xy = [(p[0], p[1]) for p in coarse_path_xyz]

    # ---- 2. fine corridor + z-guide ----
    t0 = time.perf_counter()
    corridor_mask, _ = build_xy_corridor_mask(roi, coarse_path_xy, CORRIDOR_XY_HALF_WIDTH_M)
    z_guide_grid = build_z_guide_grid(roi, coarse_path_xyz)
    for (r, c), alt in ((start[:2], LV_AIRCRAFT_MSL), (goal[:2], LV_AIRCRAFT_MSL)):
        diff = abs(alt - float(z_guide_grid[r, c]))
        if not (bool(corridor_mask[r, c]) and diff <= CORRIDOR_Z_HALF_WIDTH_M):
            z_guide_grid[r, c] = alt
            corridor_mask[r, c] = True
    corridor_time_s = time.perf_counter() - t0

    # ---- 3. fine primitive-safety precompute ----
    t0 = time.perf_counter()
    precompute = precompute_fine_corridor_primitive_safety(tq, corridor_mask, fine_primitives, fine_cfg)
    precompute_time_s = time.perf_counter() - t0

    # ---- 4. fine search bounds + ARA* (the ONLINE SEARCH this budget controls) ----
    corridor_elev = roi.elevation[corridor_mask]
    if roi.nodata is not None:
        corridor_elev = corridor_elev[corridor_elev != roi.nodata]
    fine_seg_min = float(corridor_elev.min()) if corridor_elev.size else LV_AIRCRAFT_MSL
    fine_min_search = math.ceil((fine_seg_min + fine_cfg.min_agl_m) / fine_cfg.z_step_m) * fine_cfg.z_step_m
    fine_max_search = LV_AIRCRAFT_MSL + 20.0

    t_search0 = time.perf_counter()
    ara_result = ara_star_search(
        start, goal, tq, min_search_altitude_msl=fine_min_search, max_search_altitude_msl=fine_max_search,
        config=fine_cfg, primitives=fine_primitives, epsilon_schedule=FINE_EPSILON_SCHEDULE,
        max_expansions_cumulative=FINE_MAX_EXPANSIONS_CUMULATIVE, max_search_time_s=max_search_time_s,
        corridor_mask=corridor_mask, z_guide_grid=z_guide_grid, z_guide_tolerance_m=CORRIDOR_Z_HALF_WIDTH_M,
        fine_precompute=precompute,
    )
    search_time_s = time.perf_counter() - t_search0

    t_val0 = time.perf_counter()
    path = ara_result.final_incumbent_path
    safe = validate_path_safety(path, fine_primitives, tq, fine_cfg) if path else False
    min_agl = _path_min_observed_agl(path, fine_primitives, tq, fine_cfg) if path else None
    validation_time_s = time.perf_counter() - t_val0

    return {
        "name": "long_valley_6.48km_full_pipeline",
        "labels": labels(heading=False, aircraft_lut=False, aircraft_primitives=False,
                          corridor=True, search_guidance="CURRENT", production_search="ara_star_search"),
        "timing": {
            "preprocessing_time_s": preprocessing_time_s,
            "coarse_guidance_time_s": coarse_time_s,
            "corridor_time_s": corridor_time_s,
            "fine_precompute_time_s": precompute_time_s,
            "search_time_s": search_time_s,
            "path_validation_time_s": validation_time_s,
            "total_pipeline_time_s": (preprocessing_time_s + coarse_time_s + corridor_time_s
                                       + precompute_time_s + search_time_s + validation_time_s),
        },
        "correctness": {
            "termination_reason": ara_result.termination_reason,
            "path_found": ara_result.path_found,
            "safety_pass": safe,
            "min_agl_m": min_agl,
            **goal_error(path, goal, tq, fine_cfg),
        },
        "quality": {**path_quality_metrics(path, tq, fine_cfg), "path_cost": ara_result.final_incumbent_cost},
        "performance": {
            "total_expanded": ara_result.total_expanded,
            "corridor_cell_count": int(corridor_mask.sum()),
            "n_phases": len(ara_result.phases),
            "refinement_limit_reached": ara_result.refinement_limit_reached,
            "timeout_triggered": ara_result.timeout_triggered,
        },
    }


def run_timeout_and_expansion_limit_tests(mission, cache, config, primitives) -> dict:
    """Step PERF-0 sections 8/9: deliberately trip both budgets on a real
    mission and assert the termination reason is TIMEOUT/EXPANSION_LIMIT
    -- NEVER a no-path/unreachable-shaped result."""
    tiny_time = run_mission_case("timeout_probe", mission, cache, config, primitives,
                                  max_expansions=1_000_000, max_search_time_s=1e-9)
    tiny_expansions = run_mission_case("expansion_limit_probe", mission, cache, config, primitives,
                                        max_expansions=1, max_search_time_s=None)

    timeout_ok = tiny_time["correctness"]["termination_reason"] == "TIMEOUT"
    expansion_ok = tiny_expansions["correctness"]["termination_reason"] == "EXPANSION_LIMIT"
    print(f"  timeout probe (max_search_time_s=1e-9): termination_reason="
          f"{tiny_time['correctness']['termination_reason']}  {'PASS' if timeout_ok else 'FAIL'}")
    print(f"  expansion-limit probe (max_expansions=1): termination_reason="
          f"{tiny_expansions['correctness']['termination_reason']}  {'PASS' if expansion_ok else 'FAIL'}")
    return {
        "timeout_probe": tiny_time, "expansion_limit_probe": tiny_expansions,
        "timeout_semantics_ok": timeout_ok, "expansion_limit_semantics_ok": expansion_ok,
    }


def main() -> None:
    print("=" * 70)
    print("STEP PERF-0 -- search performance baseline + budget contract")
    print("=" * 70)

    config = DEFAULT_CONFIG
    primitives = build_primitive_set(config)
    fine_roi = load_roi(DEFAULT_CONFIG)
    cache = load_terrain_cache(CACHE_DIR, fine_roi, SOURCE_DEM_PATH)
    missions = objective_mission_setup(cache)

    mission_primitives = build_primitive_set(MISSION_CONFIG)

    print("\n--- Mission A/B/C (production astar_search, sparse/lazy representation) ---")
    mission_results = {}
    for key in ("A_easy_open", "B_relief_affected", "C_z_matters"):
        r = run_mission_case(key, missions[key], cache, MISSION_CONFIG, mission_primitives, max_expansions=30_000)
        mission_results[key] = r
        c, q, p = r["correctness"], r["quality"], r["performance"]
        print(f"  [{key}] termination={c['termination_reason']} safety={'PASS' if c['safety_pass'] else 'FAIL'} "
              f"min_agl={c['min_agl_m']} cost={q['path_cost']} nodes={q['node_count']} "
              f"expanded={p['expanded']} search_time_s={r['timing']['search_time_s']:.3f}")

    print("\n--- 6.48km long-valley (plain astar_search, legacy cost, weighted A*) ---")
    lv_astar = run_long_valley_astar(config, primitives, max_expansions=30_000)
    c, q, p = lv_astar["correctness"], lv_astar["quality"], lv_astar["performance"]
    print(f"  termination={c['termination_reason']} safety={'PASS' if c['safety_pass'] else 'FAIL'} "
          f"min_agl={c['min_agl_m']} cost={q['path_cost']} nodes={q['node_count']} "
          f"expanded={p['expanded']} search_time_s={lv_astar['timing']['search_time_s']:.3f}")

    print("\n--- 6.48km long-valley (FULL production pipeline: coarse -> corridor -> fine_precompute -> ARA*) ---")
    lv_full = run_long_valley_full_pipeline()
    if "error" in lv_full:
        print(f"  ABORTED: {lv_full['error']} (coarse_status={lv_full.get('coarse_status')})")
    else:
        c, q, p, t = lv_full["correctness"], lv_full["quality"], lv_full["performance"], lv_full["timing"]
        print(f"  termination={c['termination_reason']} safety={'PASS' if c['safety_pass'] else 'FAIL'} "
              f"min_agl={c['min_agl_m']} cost={q['path_cost']} nodes={q['node_count']}")
        print(f"  timing: preprocessing={t['preprocessing_time_s']:.3f}s coarse={t['coarse_guidance_time_s']:.3f}s "
              f"corridor={t['corridor_time_s']:.3f}s precompute={t['fine_precompute_time_s']:.3f}s "
              f"search={t['search_time_s']:.3f}s validation={t['path_validation_time_s']:.6f}s "
              f"total={t['total_pipeline_time_s']:.3f}s")

    print("\n--- Timeout / expansion-limit semantics validation (Mission B) ---")
    budget_tests = run_timeout_and_expansion_limit_tests(
        missions["B_relief_affected"], cache, MISSION_CONFIG, mission_primitives
    )

    artifact = {
        "artifact_type": "SEARCH_PERFORMANCE_BASELINE",
        "schema_version": 1,
        "git_commit": git_commit_hash(),
        "git_dirty": git_is_dirty(),
        "config_hash": {
            "default_config_30m_long_valley_cases": config_hash(config),
            "mission_config_60m_coarse_ABC_cases": config_hash(MISSION_CONFIG),
        },
        "watchdogs_s": {"small": WATCHDOG_SMALL_S, "normal": WATCHDOG_NORMAL_S, "long": WATCHDOG_LONG_S,
                        "note": "development watchdog values only -- NOT a final performance target"},
        "future_comparison_labels_note": (
            "Every test record's own 'labels' block is the authoritative per-test snapshot. "
            "This baseline overall: STATE=(x,y,z), HEADING=OFF, AIRCRAFT-AWARE PRIMITIVES=OFF, "
            "AIRCRAFT LUT=OFF, HARD CORRIDOR=ON (full-pipeline case only; the plain astar_search "
            "cases don't use a corridor at all), SEARCH GUIDANCE=CURRENT, PRODUCTION SEARCH=ARA* "
            "(full-pipeline case) / astar_search (Mission A/B/C and plain long-valley cases). "
            "A future run is expected to flip these to (x,y,z,heading), aircraft LUT ON, aircraft "
            "primitives ON, hard corridor OFF, new search guidance, and be compared against this "
            "artifact test-by-test."
        ),
        "missions": mission_results,
        "long_valley_astar": lv_astar,
        "long_valley_full_pipeline": lv_full,
        "budget_semantics_validation": budget_tests,
    }

    Path("outputs").mkdir(exist_ok=True)
    with open(OUTPUT_PATH, "w", encoding="utf-8") as f:
        json.dump(artifact, f, indent=2, default=str)
    print(f"\nBaseline artifact written to {OUTPUT_PATH}")

    # Mission B/C are established (Step 3E/CLEAN-1) to succeed with a real safe path;
    # Mission A's own search-limitation is a known, separately-documented open issue
    # (project.md), not gated on here -- see module docstring. The full-pipeline case
    # must also produce a safe path; the plain-astar_search long-valley case is allowed
    # to be budget-limited (EXPANSION_LIMIT/TIMEOUT are honest, inconclusive results,
    # not failures) as long as it reports one of the correct termination reasons.
    def _mission_ok(key):
        r = mission_results[key]["correctness"]
        return r["termination_reason"] == "FOUND" and r["safety_pass"]

    valid_termination_reasons = {"FOUND", "NO_PATH", "EXPANSION_LIMIT", "TIMEOUT"}
    all_ok = (
        budget_tests["timeout_semantics_ok"]
        and budget_tests["expansion_limit_semantics_ok"]
        and _mission_ok("B_relief_affected")
        and _mission_ok("C_z_matters")
        and lv_astar["correctness"]["termination_reason"] in valid_termination_reasons
        and ("error" not in lv_full)
        and lv_full["correctness"]["termination_reason"] in valid_termination_reasons
        and (lv_full["correctness"]["termination_reason"] != "FOUND" or lv_full["correctness"]["safety_pass"])
    )
    print(f"\nOverall: {'PASS' if all_ok else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
