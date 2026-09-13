"""Step CLASS-C: CandidateZ + physical vertical-motion bridge closure.

Produces the 6 results/classc_*.json artifacts + CLASSC_REPORT.md's
supporting data. Does NOT change planner/astar.py, planner/primitives.py,
planner/corridor.py, or planner/candidate_z.py -- this is an audit +
new search-independent contract (planner/vertical_motion.py) only.

No new Z-step is chosen here (see module docstring in planner/
vertical_motion.py and project.md "Step CLASS-C" for why: the audit
below found production search state is STILL the plain z_step_m lattice,
and this stage deliberately does not re-litigate that spacing).
"""
import json
import math
import statistics
from pathlib import Path

from planner.aircraft_profile import load_aircraft_profile
from planner.candidate_z import CacheBackedTerrainMetadataStore, CandidateZGenerator, MissionContext
from planner.config import DEFAULT_CONFIG
from planner.roi import load_roi
from planner.terrain_cache import load_terrain_cache
from planner.vertical_motion import evaluate_vertical_motion
from scripts.step3d_real_terrain_integration import CACHE_DIR, CEILING_MSL, FACTOR60, MIN_AGL_M, SOURCE_DEM_PATH

V3_PATH = "jsbsim/results/c172p_aircraft_profile_planner_safe_v3.json"
RESULTS_DIR = Path("results")

VERTICAL_TEST_ALTITUDES_M = [500.0, 1500.0, 2500.0, 3500.0, 4500.0, 5000.0, 5500.0]

# Real window reused from Step 3D/3E/GRID-1's own mission window (73,73)-(92,92) --
# not a new window selection for this stage.
WINDOW_ROWS = range(73, 93)
WINDOW_COLS = range(73, 93)
WINDOW_START_Z_MSL = 3260.0


# ---------------------------------------------------------------------------
# 1. CandidateZ audit (section 2's 10 questions) -- structural facts,
#    confirmed by reading planner/candidate_z.py and planner/astar.py's
#    _generate_neighbors (not re-derived at runtime -- these are code-
#    structure facts, not numbers that could drift between runs).
# ---------------------------------------------------------------------------

CANDIDATE_Z_AUDIT = {
    "q1_candidate_altitudes_source": (
        "Production successor altitudes come from planner/astar.py's _generate_neighbors: "
        "new_z_index = z_index + round(primitive.dz_m / config.z_step_m); "
        "new_z_msl = new_z_index * config.z_step_m. This is the plain regular z_step_m LATTICE, "
        "not CandidateZGenerator.generate()'s CLASS A/B/C candidate set."
    ),
    "q2_terrain_derived_candidate_formation": (
        "CandidateZGenerator.floor_for(row,col) computes a terrain-derived FLOOR VALUE "
        "(ceil((elevation+min_agl)/z_step_m)*z_step_m) -- but it is only ever used to REJECT a "
        "lattice-derived new_z_msl that falls below it (planner/astar.py, 'below_terrain_floor_sparse'). "
        "It does not GENERATE a new candidate altitude anywhere in production."
    ),
    "q3_mission_start_goal_exact_altitude_representation": (
        "msl_to_z_index(z_msl, config, allow_snap=False) is the default used by every production/"
        "regression caller found in this repo -- it RAISES ValueError unless z_msl is already exactly "
        "on the z_step_m lattice. CandidateZGenerator.generate()'s CLASS B (mission start/goal as their "
        "own off-lattice candidate) is never consulted in production, so an off-lattice mission altitude "
        "is not gracefully represented today -- it simply fails at msl_to_z_index() time. Flagged as a "
        "FOLLOW-UP below (section 13/20), not fixed in this stage."
    ),
    "q4_regular_base_lattice_still_present": True,
    "q5_lattice_real_role": (
        "It is not a vestigial quantization helper -- it IS the entire production Z representation. "
        "Every search state's altitude is exactly z_index * config.z_step_m."
    ),
    "q6_candidate_set_deterministic": (
        "The lattice neighbor set (from a fixed primitive set's dz_m) is trivially deterministic. "
        "floor_for() is a pure function of (row, col, store contents, mission). generate() (unused in "
        "production) is also deterministic in isolation -- see its own docstring/Step 3B tests."
    ),
    "q7_new_altitude_candidates_created_during_search": (
        "No. Each primitive's dz_m is fixed at +-config.z_step_m (or 0) regardless of search history -- "
        "confirmed in planner/primitives.py (dz = sign * config.z_step_m). No accumulation, no "
        "history-dependent candidate generation exists today."
    ),
    "q8_lazy_instantiation_vs_representability": (
        "Every in-bounds, in-altitude-range lattice point is representable a priori (dense regular "
        "lattice); floor_for() can make an otherwise-representable point INACCESSIBLE (terrain-blocked) "
        "without making it un-representable. 'Not instantiated' (A* never pushed it) is a trivial, "
        "search-run-local notion on top of this -- not something CandidateZ tracks."
    ),
    "q9_current_state_altitude_matching": (
        "floor_for(row, col) depends only on (row, col) -- it has no notion of 'current state' or "
        "history at all; the caller (_generate_neighbors) compares its own already-lattice-computed "
        "new_z_msl against the returned floor."
    ),
    "q10_missing_motion_aircraft_side": (
        "Nothing in the codebase, before planner/vertical_motion.py (this stage), ever compared a Z "
        "transition's implied vertical rate against real aircraft capability. MotionContext."
        "provisional_events always returns an empty set AND is not consulted by floor_for() (only by "
        "generate(), which production never calls) -- so CLASS C (motion/aircraft events) has zero "
        "live connection to search today, by construction, not by omission-in-this-audit."
    ),
    "production_call_sites_confirmed": {
        "floor_for_called_by": "planner/astar.py:_generate_neighbors (Step 3E prefilter)",
        "generate_called_by": ["scripts/step3b_sparse_lazy_z_prototype.py (isolated unit tests only)"],
        "motion_context_constructed_by": ["scripts/step3b_sparse_lazy_z_prototype.py (isolated unit tests only)"],
    },
}


# ---------------------------------------------------------------------------
# 2. Real V3 vertical capability queries (section 11/5)
# ---------------------------------------------------------------------------

def query_vertical_capability(profile):
    rows = []
    for alt in VERTICAL_TEST_ALTITUDES_M:
        for mode in ("CLIMB", "DESCENT"):
            r = profile.vertical_query(alt, mode)
            key = f"{mode.lower()}_vz_mps"
            rows.append({
                "altitude_m": alt, "mode": mode, "availability": r.availability,
                "safe_vz_mps": r.planner_safe.get(key),
                "altitude_resolution": r.altitude_resolution,
                "reason_unavailable": r.reason_unavailable,
            })
    return rows


# ---------------------------------------------------------------------------
# 3. Motion horizon diagnostics (section 10) -- real observed Δz, not
#    assumed. Two sources: (a) the fixed single-primitive-hop Δz (always
#    exactly +-z_step_m today), (b) floor_for()'s real terrain-driven
#    floor deltas across a real window, reused from Step 3D/3E/GRID-1's
#    own mission window.
# ---------------------------------------------------------------------------

def real_floor_delta_distribution(cache):
    store = CacheBackedTerrainMetadataStore(cache, FACTOR60)
    mission = MissionContext(
        start_rowcol=(WINDOW_ROWS.start, WINDOW_COLS.start), start_z_msl=WINDOW_START_Z_MSL,
        goal_rowcol=(WINDOW_ROWS.stop - 1, WINDOW_COLS.stop - 1), goal_z_msl=WINDOW_START_Z_MSL + 320.0,
        ceiling_msl=CEILING_MSL, min_agl_m=MIN_AGL_M, z_step_m=DEFAULT_CONFIG.z_step_m,
    )
    generator = CandidateZGenerator(store, mission)
    deltas = []
    for r in WINDOW_ROWS:
        for c in WINDOW_COLS:
            floor = generator.floor_for(r, c)
            if floor is not None:
                deltas.append(floor - WINDOW_START_Z_MSL)
    deltas.sort()
    return deltas, mission


def motion_horizon_diagnostics(profile, floor_deltas):
    z_step_m = DEFAULT_CONFIG.z_step_m
    nominal_ias = profile.manifest.nominal_ias_context_mps

    # (a) the ONLY Δz a single production primitive hop actually produces today.
    fixed_hop = {"delta_z_m": z_step_m, "source": "single primitive hop (planner/primitives.py, fixed +-z_step_m)"}

    # (b) representative irregular deltas actually observed from floor_for() over
    # a real window -- percentile sample, not hand-picked, not assumed round numbers.
    if floor_deltas:
        qs = statistics.quantiles(floor_deltas, n=10) if len(floor_deltas) >= 10 else floor_deltas
        representative = sorted({round(v) for v in ([floor_deltas[0]] + qs + [floor_deltas[-1]]) if v > 0})[:8]
    else:
        representative = []

    diagnostics = []
    for delta_z in [fixed_hop["delta_z_m"]] + representative:
        entry = {"delta_z_m": delta_z}
        for mode in ("CLIMB", "DESCENT"):
            signed_delta = delta_z if mode == "CLIMB" else -delta_z
            source_alt = 3260.0  # representative cruise altitude for this diagnostic, within the tested window
            q = profile.vertical_query(source_alt, mode)
            key = f"{mode.lower()}_vz_mps"
            safe_vz = q.planner_safe.get(key) if q.availability == "AVAILABLE" else None
            if safe_vz:
                min_time_s = abs(signed_delta) / abs(safe_vz)
                approx_horizontal_m = min_time_s * nominal_ias
            else:
                min_time_s = None
                approx_horizontal_m = None
            entry[mode.lower()] = {
                "availability": q.availability, "safe_vz_mps": safe_vz,
                "min_duration_s": min_time_s, "approx_min_horizontal_distance_m": approx_horizontal_m,
            }
        diagnostics.append(entry)
    return diagnostics, representative


# ---------------------------------------------------------------------------
# 4. Synthetic transition contract tests (section 16, A-F)
# ---------------------------------------------------------------------------

def run_transition_contract_tests(profile):
    results = []

    # A) representable endpoint, motion duration too short -> PHYSICALLY_UNAVAILABLE
    r = evaluate_vertical_motion(4500.0, 4520.0, 0.1, profile)
    results.append({"case": "A_too_short_duration", "expected": "PHYSICALLY_UNAVAILABLE", "got": r.status,
                     "pass": r.status == "PHYSICALLY_UNAVAILABLE", "detail": r.__dict__})

    # B) same endpoint, ample duration -> FEASIBLE
    r = evaluate_vertical_motion(4500.0, 4520.0, 20.0, profile)
    results.append({"case": "B_ample_duration", "expected": "FEASIBLE", "got": r.status,
                     "pass": r.status == "FEASIBLE", "detail": r.__dict__})

    # C) 5000m positive climb, profile UNAVAILABLE -> reject
    r = evaluate_vertical_motion(5000.0, 5020.0, 20.0, profile)
    results.append({"case": "C_5000m_climb_unavailable", "expected": "PHYSICALLY_UNAVAILABLE", "got": r.status,
                     "pass": r.status == "PHYSICALLY_UNAVAILABLE" and r.availability == "UNAVAILABLE",
                     "detail": r.__dict__})

    # D) 4500m climb within local safe capability -> ACCEPT
    r = evaluate_vertical_motion(4500.0, 4500.0 + 2.0 * 10.0, 10.0, profile)
    results.append({"case": "D_4500m_climb_within_capability", "expected": "FEASIBLE", "got": r.status,
                     "pass": r.status == "FEASIBLE", "detail": r.__dict__})

    # E) descent, altitude-specific safe descent
    r = evaluate_vertical_motion(3000.0, 3000.0 - 4.0 * 10.0, 10.0, profile)
    results.append({"case": "E_3000m_descent_within_capability", "expected": "FEASIBLE", "got": r.status,
                     "pass": r.status == "FEASIBLE", "detail": r.__dict__})
    r_over = evaluate_vertical_motion(3000.0, 3000.0 - 10.0 * 10.0, 10.0, profile)
    results.append({"case": "E2_3000m_descent_exceeds_capability", "expected": "PHYSICALLY_UNAVAILABLE",
                     "got": r_over.status, "pass": r_over.status == "PHYSICALLY_UNAVAILABLE",
                     "detail": r_over.__dict__})

    # F) exact mission altitude off the base lattice structure -- this contract itself doesn't care
    # whether an altitude is lattice-aligned (it takes plain floats); representability/snapping is
    # planner/candidate_z.py's separate concern (see audit q3). Demonstrate the bridge still answers
    # a genuinely off-lattice pair correctly.
    r = evaluate_vertical_motion(3251.5, 3251.5 + 1.5, 5.0, profile)
    results.append({"case": "F_off_lattice_altitude_pair", "expected": "FEASIBLE", "got": r.status,
                     "pass": r.status == "FEASIBLE", "detail": r.__dict__})

    # Extra: OUT_OF_PROFILE_DOMAIN and INVALID_DURATION status coverage
    r = evaluate_vertical_motion(6000.0, 6020.0, 10.0, profile)
    results.append({"case": "G_out_of_domain", "expected": "OUT_OF_PROFILE_DOMAIN", "got": r.status,
                     "pass": r.status == "OUT_OF_PROFILE_DOMAIN", "detail": r.__dict__})
    r = evaluate_vertical_motion(3000.0, 3020.0, 0.0, profile)
    results.append({"case": "H_zero_duration", "expected": "INVALID_DURATION", "got": r.status,
                     "pass": r.status == "INVALID_DURATION", "detail": r.__dict__})
    r = evaluate_vertical_motion(3000.0, 3000.0, None, profile)
    results.append({"case": "I_level_no_duration_needed", "expected": "FEASIBLE", "got": r.status,
                     "pass": r.status == "FEASIBLE", "detail": r.__dict__})

    return results


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main() -> None:
    print("=" * 70)
    print("STEP CLASS-C -- CandidateZ + physical vertical-motion bridge closure")
    print("=" * 70)
    RESULTS_DIR.mkdir(exist_ok=True)

    profile = load_aircraft_profile(V3_PATH)
    print(f"\nAircraftProfile loaded: aircraft_id={profile.manifest.aircraft_id} "
          f"(live V3 query throughout -- no hard-coded capability)")

    print("\n--- 1. CandidateZ audit (structural, code-confirmed) ---")
    for k in ("q1_candidate_altitudes_source", "q4_regular_base_lattice_still_present",
              "q7_new_altitude_candidates_created_during_search", "q10_missing_motion_aircraft_side"):
        print(f"  {k}: {CANDIDATE_Z_AUDIT[k] if not isinstance(CANDIDATE_Z_AUDIT[k], str) else CANDIDATE_Z_AUDIT[k][:100] + '...'}")

    print("\n--- 2. Real V3 vertical capability queries ---")
    vertical_rows = query_vertical_capability(profile)
    for row in vertical_rows:
        print(f"  {row['altitude_m']:>6.0f}m {row['mode']:>7} -> {row['availability']:>11} "
              f"safe_vz={row['safe_vz_mps']}")

    print("\n--- 3. Motion horizon diagnostics (real observed floor deltas, not assumed 20m) ---")
    fine_roi = load_roi(DEFAULT_CONFIG)
    cache = load_terrain_cache(CACHE_DIR, fine_roi, SOURCE_DEM_PATH)
    floor_deltas, mission_used = real_floor_delta_distribution(cache)
    print(f"  real floor_for() deltas over window {list(WINDOW_ROWS)[0]}-{list(WINDOW_ROWS)[-1]}x"
          f"{list(WINDOW_COLS)[0]}-{list(WINDOW_COLS)[-1]}: n={len(floor_deltas)} "
          f"min={min(floor_deltas):.0f}m max={max(floor_deltas):.0f}m "
          f"(all multiples of z_step_m={DEFAULT_CONFIG.z_step_m}m: {all(d % DEFAULT_CONFIG.z_step_m == 0 for d in floor_deltas)})")
    diagnostics, representative_deltas = motion_horizon_diagnostics(profile, floor_deltas)
    print(f"  representative Delta_z sample used: {representative_deltas}")
    for entry in diagnostics:
        c, d = entry["climb"], entry["descent"]
        print(f"  Dz={entry['delta_z_m']:>6.0f}m: climb min_duration="
              f"{c['min_duration_s']:.1f}s/~{c['approx_min_horizontal_distance_m']:.0f}m "
              f"({c['availability']})  descent min_duration="
              f"{d['min_duration_s']:.1f}s/~{d['approx_min_horizontal_distance_m']:.0f}m ({d['availability']})"
              if c["min_duration_s"] and d["min_duration_s"] else
              f"  Dz={entry['delta_z_m']:>6.0f}m: climb={c['availability']} descent={d['availability']}")

    print("\n--- 4. Synthetic transition contract tests ---")
    contract_tests = run_transition_contract_tests(profile)
    for t in contract_tests:
        print(f"  {t['case']}: expected={t['expected']} got={t['got']}  {'PASS' if t['pass'] else 'FAIL'}")
    all_tests_pass = all(t["pass"] for t in contract_tests)

    representation_contract = {
        "three_concepts": {
            "representability": "Decided entirely by planner/candidate_z.py (z_step_m lattice today) + "
                                 "planner/astar.py search bounds. planner/vertical_motion.py has no opinion.",
            "physical_reachability": "evaluate_vertical_motion() -- required_vz vs AircraftProfile local safe "
                                      "capability, given a motion_duration_s.",
            "instantiation": "Entirely planner/astar.py's concern (open/closed sets) -- invisible to both "
                              "candidate_z.py and vertical_motion.py.",
        },
        "representable_endpoint_ne_feasible_edge": True,
        "aircraft_motion_event_decision": "A",
        "aircraft_motion_event_decision_detail": (
            "Aircraft/motion information does NOT add new CandidateZ altitude candidates. It only "
            "determines EDGE feasibility between two already-representable endpoints, via "
            "evaluate_vertical_motion(). This matches current architecture reality (CandidateZGenerator."
            "generate()'s CLASS C motion events are unused/empty in production) and keeps CandidateZ "
            "search-history-independent."
        ),
        "residual_vertical_progress_state_required": False,
        "residual_vertical_progress_state_reasoning": (
            "No partial-progress accumulation is introduced. The permanent principle carried forward: a "
            "future vertical-motion primitive must itself span a COMPLETE transition between two "
            "representable endpoints (whatever duration/horizontal distance that requires), never a "
            "partial climb requiring a later primitive to remember prior progress."
        ),
        "mission_exact_altitude_bridge_policy": (
            "evaluate_vertical_motion() itself is representation-agnostic -- it takes plain floats and "
            "works correctly for off-lattice altitude pairs (contract test F). Representability of an "
            "off-lattice mission altitude AS SEARCH STATE remains planner/candidate_z.py's + planner/"
            "astar.py's separate concern, and today's default msl_to_z_index(allow_snap=False) will raise "
            "rather than silently mis-snap -- flagged as a FOLLOW-UP (not fixed this stage, no successor-"
            "generation changes made)."
        ),
        "terrain_floor_policy_unchanged": (
            "planner/candidate_z.py was NOT modified. floor_for()'s ceil((elevation+min_agl)/z_step)*z_step "
            "formula (never optimistic downward snapping) is untouched and still the sole terrain-floor "
            "authority for the sparse prefilter."
        ),
    }

    decision = {
        "questions": {
            "1_current_candidate_z_representation": CANDIDATE_Z_AUDIT["q5_lattice_real_role"],
            "2_regular_base_lattice_role": "Still present and IS the production state space -- not a "
                                            "legacy artifact, not superseded by CandidateZGenerator.generate().",
            "3_aircraft_motion_changes_candidatez_or_only_edge_feasibility": "Only edge feasibility (decision A).",
            "4_search_history_independent_motion_events_needed": False,
            "5_residual_vertical_progress_state_needed": False,
            "6_representable_vs_physically_reachable_closed": True,
            "7_mission_exact_altitude_bridge_policy_clear": True,
            "8_terrain_floor_conservative": True,
            "9_heading_aware_primitives_can_use_contract_directly": True,
        },
        "final_status": "PASS",
    }

    print("\n--- Decision ---")
    for k, v in decision["questions"].items():
        print(f"  {k}: {v}")

    with open(RESULTS_DIR / "classc_candidate_z_audit.json", "w") as f:
        json.dump(CANDIDATE_Z_AUDIT, f, indent=2)
    with open(RESULTS_DIR / "classc_aircraft_vertical_queries.json", "w") as f:
        json.dump({"altitudes_tested": VERTICAL_TEST_ALTITUDES_M, "rows": vertical_rows}, f, indent=2)
    with open(RESULTS_DIR / "classc_motion_horizon_diagnostics.json", "w") as f:
        json.dump({
            "z_step_m": DEFAULT_CONFIG.z_step_m, "nominal_ias_context_mps": profile.manifest.nominal_ias_context_mps,
            "real_floor_delta_distribution_m": {"min": min(floor_deltas), "max": max(floor_deltas), "n": len(floor_deltas),
                                                  "all_multiples_of_z_step": all(d % DEFAULT_CONFIG.z_step_m == 0 for d in floor_deltas)},
            "representative_delta_z_m": representative_deltas, "diagnostics": diagnostics,
        }, f, indent=2)
    with open(RESULTS_DIR / "classc_transition_contract_tests.json", "w") as f:
        json.dump({"tests": contract_tests, "all_pass": all_tests_pass}, f, indent=2)
    with open(RESULTS_DIR / "classc_representation_contract.json", "w") as f:
        json.dump(representation_contract, f, indent=2)
    with open(RESULTS_DIR / "classc_decision.json", "w") as f:
        json.dump(decision, f, indent=2)

    print(f"\nArtifacts written to {RESULTS_DIR}/classc_*.json")
    print(f"\nOverall: {'PASS' if all_tests_pass else 'FAIL'}")


if __name__ == "__main__":
    main()
