"""Step REP-1: production sparse CandidateZ integration audit + closure.

Produces the results/rep1_*.json artifacts + REP1_REPORT.md's supporting
data. Central finding (project.md "Step REP-1"): CandidateZGenerator.
generate()'s literal output (a 2-4 element boundary-event set -- floor,
ceiling, start/goal) is too sparse to serve as the successor Z-candidate
set for primitive-arithmetic-based stepping (every interior ladder rung
strictly between floor and ceiling is never in that tuple). The regular
z_step_m lattice therefore remains structurally necessary as the search
STATE representation; what genuinely closes this stage is a NEW, unified
`CandidateZGenerator.is_representable(row, col, z_msl)` predicate
(planner/candidate_z.py) that becomes the real representability
authority in planner/astar.py's `_generate_neighbors` (Step REP-1),
replacing the narrower floor-only check -- see planner/astar.py's
`candidate_z_generator` docstring and CANDIDATE_Z_AUDIT below.

No new Z-step is chosen (no 10/20/40 comparison). No search algorithm
change beyond the representability-gate swap. No heading, no aircraft
primitives, no corridor/guidance change.
"""
import json
import math
import random
from pathlib import Path

from planner.candidate_z import CacheBackedTerrainMetadataStore, CandidateZGenerator, MissionContext
from planner.config import DEFAULT_CONFIG
from planner.roi import load_roi
from planner.terrain_cache import load_terrain_cache
from scripts.step3d_real_terrain_integration import CACHE_DIR, CEILING_MSL, FACTOR60, MIN_AGL_M, SOURCE_DEM_PATH

RESULTS_DIR = Path("results")

# Same real window Step 3D/3E/GRID-1/CLASS-C already used -- not reselected.
WINDOW_ROW0, WINDOW_ROW1 = 73, 93
WINDOW_COL0, WINDOW_COL1 = 73, 93


# ---------------------------------------------------------------------------
# 1. z_step/z_index usage audit (section 3) -- structural facts, confirmed
#    by reading planner/astar.py, planner/coarse_astar.py, planner/
#    candidate_z.py, planner/primitives.py, webapp/server.py (not
#    re-derived at runtime).
# ---------------------------------------------------------------------------

ZSTEP_USAGE_AUDIT = {
    "classification": {
        "A_legacy_dense_regular_representation": [
            {"site": "planner/astar.py: msl_to_z_index / z_index_to_msl / state_to_xyz / path_to_xyz",
             "role": "Defines CanonicalState=(row,col,z_index) and the z_index<->msl mapping "
                      "(z_index * config.z_step_m) -- THE production search state space."},
            {"site": "planner/astar.py: _generate_neighbors",
             "role": "new_z_index = z_index + round(dz_m/z_step_m); new_z_msl = new_z_index*z_step_m -- "
                      "the actual dense-lattice successor-altitude arithmetic, used by both astar_search "
                      "and ara_star_search."},
            {"site": "planner/astar.py: validate_path_safety / validate_and_cost_path / astar_search setup",
             "role": "by_delta = {(drow,dcol,round(dz_m/z_step_m)): primitive} -- primitive lookup keyed "
                      "on the dense z_step_m grid delta."},
            {"site": "planner/coarse_astar.py: lift_endpoint_if_unsafe / _generate_coarse_neighbors / "
                      "coarse_astar_search",
             "role": "Identical dense-lattice pattern on the COARSE grid. Does not import "
                      "CandidateZGenerator at all -- the coarse search has no sparse-prefilter option, "
                      "before or after this stage."},
            {"site": "webapp/server.py: COARSE_CFG/FINE_CFG, _min_safe_altitude, /api/plan",
             "role": "Builds dense z_index states via msl_to_z_index and calls coarse_astar_search/"
                      "astar_search/ara_star_search WITHOUT a candidate_z_generator argument at all -- "
                      "confirmed: 'CandidateZGenerator' does not appear anywhere in webapp/server.py. "
                      "The live interactive tool never used the Step 3E sparse prefilter, before or "
                      "after this stage -- disclosed as a follow-up, not fixed here (see Blockers)."},
        ],
        "B_still_required_utility": [
            {"site": "planner/primitives.py: build_primitive_set", "role": "dz = sign * config.z_step_m "
              "fixes each climb/descent primitive's vertical delta to one grid step -- source of the "
              "delta primitives use, not itself a state enumeration."},
            {"site": "planner/candidate_z.py: CandidateZGenerator.floor_for", "role": "Per-cell terrain+"
              "min_agl floor (ceil((elevation+min_agl)/z_step)*z_step) -- genuine, still required, now "
              "used internally by the new is_representable() (Step REP-1) as its CLASS-A lower bound."},
            {"site": "planner/astar.py: state_to_xyz / path_to_xyz", "role": "Pure display/geometry "
              "conversion from canonical state to real-world xyz for terrain lookups/output -- not "
              "itself a state-space definition."},
        ],
        "C_test_historical_only": [
            "scripts/step3b_sparse_lazy_z_prototype.py (defines the original CandidateZGenerator/"
            "MissionContext/MotionContext prototype, later moved verbatim to planner/candidate_z.py; "
            "still the only caller of .generate() and constructor of MotionContext())",
            "scripts/step3c_persistent_terrain_cache.py (imports CandidateZGenerator from step3b, not "
            "planner.candidate_z; calls .generate())",
            "All scripts/validate_*.py, scripts/benchmark_*.py, scripts/calibrate_*.py, scripts/"
            "prototype_stage38_5_*.py, scripts/step3d*.py, scripts/step3e*.py, scripts/classc_analysis.py, "
            "scripts/perf0_search_baseline.py -- none are production (planner/, webapp/) code.",
        ],
        "D_unknown": [],
    },
    "q1_floor_for_only_call_sites_in_production": (
        "planner/astar.py:_generate_neighbors (line ~781, now via is_representable()), called only from "
        "astar_search and ara_star_search. planner/candidate_z.py's own generate() also calls floor_for() "
        "internally (sibling method, not a separate production call site). planner/coarse_astar.py and "
        "webapp/server.py contain ZERO references to CandidateZGenerator/floor_for -- confirmed by repo-wide "
        "grep."
    ),
    "q2_generate_called_outside_step3b": (
        "Yes, one other place: scripts/step3c_persistent_terrain_cache.py -- but it imports "
        "CandidateZGenerator from scripts.step3b_sparse_lazy_z_prototype, not planner.candidate_z. No "
        "production file (planner/, webapp/) calls .generate() anywhere, before or after this stage."
    ),
    "q3_motion_context_constructed_outside_step3b": "No. Exactly one construction site in the whole repo: "
        "scripts/step3b_sparse_lazy_z_prototype.py.",
    "q4_dz_m_always_exactly_z_step_m": (
        "Confirmed exactly. planner/primitives.py: dz = sign * config.z_step_m for climb(+1)/descent(-1); "
        "level primitives get dz_m=0.0. No other code path assigns dz_m. Every climb primitive has "
        "dz_m=+z_step_m, every descent has dz_m=-z_step_m, regardless of direction/config."
    ),
    "q5_other_production_files": (
        "None beyond planner/astar.py, planner/candidate_z.py, planner/coarse_astar.py, planner/config.py, "
        "planner/primitives.py, webapp/server.py. planner/fine_precompute.py and planner/vertical_motion.py "
        "mention z_step_m/z_index only in docstrings/comments, zero executable references."
    ),
}


# ---------------------------------------------------------------------------
# 2. CandidateZ.generate() audit (section 4)
# ---------------------------------------------------------------------------

GENERATE_AUDIT = {
    "candidate_families": {
        "CLASS_A_static_terrain": "floor_for(row,col) -- the lowest z_step_m-ladder altitude clearing "
            "terrain+min_agl, if any fits under the mission ceiling. Deterministic, source = real terrain "
            "elevation (live DEM or persistent TerrainCache, caller's choice).",
        "CLASS_A_B_boundary": "mission.ceiling_msl -- always present alongside the floor event.",
        "CLASS_B_mission": "start_z_msl at start_rowcol, goal_z_msl at goal_rowcol -- exact mission "
            "altitudes, deterministic, source = MissionContext (caller-supplied).",
        "CLASS_C_motion": "motion_context.provisional_events(row,col) -- ALWAYS an empty frozenset "
            "(Step 3A.1 status, unchanged since). Only consulted if motion_context is not None, which no "
            "production or Step REP-1 code path ever supplies.",
    },
    "deterministic": True,
    "altitude_units": "meters MSL (float)",
    "ordering": "tuple(sorted(candidates)) -- ascending MSL",
    "deduplication": "Python set() before sorting -- exact-float dedup only (no tolerance merging)",
    "bounds": "Implicitly bounded by floor_for's own ceiling_msl cap; no explicit lower bound beyond "
              "the terrain floor itself",
    "terrain_floor_interaction": "floor_for() is CLASS A's sole source -- ceil((elevation+min_agl)/"
                                  "z_step_m)*z_step_m, always exactly on the z_step_m ladder",
    "mission_event_interaction": "CLASS B reads MissionContext.start_z_msl/goal_z_msl directly, unmodified "
                                  "-- these are NOT snapped to the ladder by generate() itself",
    "production_usage_verdict": "generate() itself remains UNUSED in production after Step REP-1 -- its "
                                  "OWN return value (a 2-4 element set) is architecturally unsuited to serve "
                                  "as a successor-candidate membership test for primitive-arithmetic-based "
                                  "stepping (see module docstring). What Step REP-1 actually wires into "
                                  "production is a NEW method, is_representable(), built from the SAME "
                                  "existing, correct infrastructure (floor_for, MissionContext) -- not a new "
                                  "candidate system, and not generate() verbatim.",
}


def real_representability_evidence(cache):
    """Section 16's synthetic tests A-H, against REAL terrain (same window
    Step 3D/3E/GRID-1/CLASS-C already used)."""
    store = CacheBackedTerrainMetadataStore(cache, FACTOR60)
    off_lattice_start = 3251.37  # deliberately NOT a multiple of 20
    off_lattice_goal = 3583.91
    mission = MissionContext(
        start_rowcol=(WINDOW_ROW0, WINDOW_COL0), start_z_msl=off_lattice_start,
        goal_rowcol=(WINDOW_ROW1 - 1, WINDOW_COL1 - 1), goal_z_msl=off_lattice_goal,
        ceiling_msl=CEILING_MSL, min_agl_m=MIN_AGL_M, z_step_m=DEFAULT_CONFIG.z_step_m,
    )
    gen = CandidateZGenerator(store, mission)

    tests = []

    # A) terrain-derived candidate -> representable
    r, c = WINDOW_ROW0 + 5, WINDOW_COL0 + 5
    floor = gen.floor_for(r, c)
    ok_a = floor is not None and gen.is_representable(r, c, floor)
    tests.append({"case": "A_terrain_derived_candidate_representable", "row": r, "col": c, "floor": floor,
                   "pass": ok_a})

    # B) exact off-lattice start altitude -> representable
    ok_b = gen.is_representable(*mission.start_rowcol, mission.start_z_msl)
    tests.append({"case": "B_off_lattice_start_representable", "altitude_m": mission.start_z_msl, "pass": ok_b})

    # C) exact off-lattice goal altitude -> representable
    ok_c = gen.is_representable(*mission.goal_rowcol, mission.goal_z_msl)
    tests.append({"case": "C_off_lattice_goal_representable", "altitude_m": mission.goal_z_msl, "pass": ok_c})

    # D) arbitrary altitude with no source -> NOT representable (off-ladder, not a mission event)
    ok_d = not gen.is_representable(r, c, floor + 7.3 if floor else 3000.0)
    tests.append({"case": "D_arbitrary_unsourced_altitude_not_representable", "pass": ok_d})

    # E) same inputs -> same result, repeated calls
    results_e = [gen.is_representable(r, c, floor) for _ in range(5)]
    ok_e = len(set(results_e)) == 1
    tests.append({"case": "E_same_inputs_same_result", "pass": ok_e})

    # F) different query order -> same result set (no order-dependent internal state)
    cells = [(rr, cc) for rr in range(WINDOW_ROW0, WINDOW_ROW0 + 10) for cc in range(WINDOW_COL0, WINDOW_COL0 + 10)]
    forward = {(rr, cc): gen.is_representable(rr, cc, gen.floor_for(rr, cc) or 0.0) for rr, cc in cells}
    shuffled = list(cells)
    random.Random(42).shuffle(shuffled)
    gen2 = CandidateZGenerator(CacheBackedTerrainMetadataStore(cache, FACTOR60), mission)
    reverse = {(rr, cc): gen2.is_representable(rr, cc, gen2.floor_for(rr, cc) or 0.0) for rr, cc in shuffled}
    ok_f = forward == reverse
    tests.append({"case": "F_different_expansion_order_same_candidate_set", "n_cells": len(cells), "pass": ok_f})

    # G) not-instantiated candidate is still representable (no search ever ran here at all)
    r2, c2 = WINDOW_ROW0 + 8, WINDOW_COL0 + 8
    floor2 = gen.floor_for(r2, c2)
    ok_g = floor2 is not None and gen.is_representable(r2, c2, floor2)
    tests.append({"case": "G_not_instantiated_still_representable", "row": r2, "col": c2, "pass": ok_g})

    # H) below terrain floor -> NOT representable
    ok_h = floor is not None and not gen.is_representable(r, c, floor - DEFAULT_CONFIG.z_step_m)
    tests.append({"case": "H_below_terrain_floor_not_representable", "pass": ok_h})

    return tests, mission


def main() -> None:
    print("=" * 70)
    print("STEP REP-1 -- production sparse CandidateZ integration + legacy cleanup audit")
    print("=" * 70)
    RESULTS_DIR.mkdir(exist_ok=True)

    print("\n--- 1. z_step/z_index usage audit ---")
    for site in ZSTEP_USAGE_AUDIT["classification"]["A_legacy_dense_regular_representation"]:
        print(f"  [A] {site['site']}")
    for site in ZSTEP_USAGE_AUDIT["classification"]["B_still_required_utility"]:
        print(f"  [B] {site['site']}")
    print(f"  [C] {len(ZSTEP_USAGE_AUDIT['classification']['C_test_historical_only'])} test/historical-only groups")

    print("\n--- 2. CandidateZ.generate() audit ---")
    print(f"  production_usage_verdict: {GENERATE_AUDIT['production_usage_verdict'][:120]}...")

    print("\n--- 3. Representability tests (A-H, real terrain) ---")
    fine_roi = load_roi(DEFAULT_CONFIG)
    cache = load_terrain_cache(CACHE_DIR, fine_roi, SOURCE_DEM_PATH)
    rep_tests, mission_used = real_representability_evidence(cache)
    for t in rep_tests:
        print(f"  {t['case']}: {'PASS' if t['pass'] else 'FAIL'}")
    all_rep_pass = all(t["pass"] for t in rep_tests)

    with open(RESULTS_DIR / "rep1_zstep_usage_audit.json", "w") as f:
        json.dump(ZSTEP_USAGE_AUDIT, f, indent=2)
    with open(RESULTS_DIR / "rep1_before_after_candidatez.json", "w") as f:
        json.dump(GENERATE_AUDIT, f, indent=2)
    with open(RESULTS_DIR / "rep1_mission_altitude_tests.json", "w") as f:
        json.dump({"tests": rep_tests, "all_pass": all_rep_pass,
                    "mission": {"start_rowcol": mission_used.start_rowcol, "start_z_msl": mission_used.start_z_msl,
                                "goal_rowcol": mission_used.goal_rowcol, "goal_z_msl": mission_used.goal_z_msl}},
                  f, indent=2)

    print(f"\nArtifacts written to {RESULTS_DIR}/rep1_*.json (partial -- see scripts/rep1_regression.py "
          f"for the remaining artifacts)")
    print(f"\nOverall (this script's own tests): {'PASS' if all_rep_pass else 'FAIL'}")


if __name__ == "__main__":
    main()
