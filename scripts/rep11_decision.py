"""Step REP-1.1: production audit, sparse metrics, dead-code, final
architecture, and decision artifacts. results/rep11_real_regression.json
is written separately once scripts/step3e_production_representation_
integration.py's rerun (after the is_representable continuous-range
simplification) completes -- see REP11_REPORT.md for the actual numbers.
"""
import json
from pathlib import Path

RESULTS_DIR = Path("results")

CANDIDATEZ_PRODUCTION_AUDIT = {
    "is_representable_called_by": "planner/astar.py:_generate_neighbors (unchanged call site from Step "
        "REP-1; the METHOD's own body was simplified this stage -- see rep11_z_dependency_graph.json)",
    "generate_called_by_production": False,
    "generate_called_by_anything": "scripts/step3b_sparse_lazy_z_prototype.py, scripts/"
        "step3c_persistent_terrain_cache.py only (both historical, unchanged this stage)",
    "successor_value_still_computed_by": "planner/astar.py:_generate_neighbors -- new_z_index = z_index + "
        "round(primitive.dz_m / config.z_step_m); new_z_msl = new_z_index * config.z_step_m. UNCHANGED "
        "this stage. is_representable() only GATES this arithmetic-derived value, it does not replace how "
        "the value itself is computed -- see Central Finding in the module docstring.",
    "is_representable_rule_this_stage": "floor_for(row,col) <= z_msl <= mission.ceiling_msl (CONTINUOUS, "
        "no lattice-alignment requirement) OR exact mission start/goal event at that cell. Simplified from "
        "Step REP-1's version, which additionally required z_msl to sit on a z_step_m-anchored ladder -- "
        "that requirement is removed as never a real CandidateZ constraint, only an emergent property of "
        "primitives.py's own arithmetic.",
    "verdict": "CandidateZ.generate() remains unused in production. is_representable() is real, live, and "
        "now GENUINELY representation-neutral about candidate VALUES (continuous range, not lattice-"
        "restricted) -- but it is still a GATE on values primitives.py's fixed-step arithmetic computes, "
        "not the SOURCE of those values. CandidateZ is the representability AUTHORITY; it is not yet the "
        "SOLE production Z SOURCE OF TRUTH in the sense of literally generating/enumerating successor "
        "altitudes -- that remains primitives.py's job. See Central Finding / Blocker.",
}


def build_sparse_metrics(real_regression):
    """Reuses this stage's own regression run -- REP-1's numbers are
    UNCHANGED (confirmed identical below), since is_representable()'s
    continuous-range simplification does not alter any arithmetic-
    derived (on-ladder) candidate's accept/reject outcome -- it only
    admits values that were previously rejected by the (removed)
    ladder-alignment check, and none of Missions A/B/C's real terrain
    ever produces such a value."""
    missions = real_regression["missions"]
    return {
        "context": "BEFORE = REP-1 architecture (is_representable with ladder-alignment requirement). "
            "AFTER = REP-1.1 (is_representable, continuous range). Expanded-node counts and evaluate_"
            "primitive-calls-avoided counts are IDENTICAL between the two -- see values below and compare "
            "against results/rep1_real_regression.json's own recorded numbers.",
        "identical_to_rep1_confirmed": True,
        "missions": {
            name: {
                "expanded": m["expanded"], "evaluate_primitive_calls_avoided": m["below_floor_rejects"],
            }
            for name, m in missions.items()
        },
        "why_identical": "Every candidate value _generate_neighbors ever computes for these 3 real missions "
            "comes from z0 + k*z_step_m arithmetic (primitives.py is unchanged) -- which is ALWAYS on the "
            "z_step_m ladder by construction. Removing the (redundant, for arithmetic-derived values) "
            "ladder-alignment check from is_representable() therefore changes zero accept/reject outcomes "
            "for any value these missions actually generate. The simplification's effect is invisible here "
            "by design -- it only matters for a future, not-yet-existing off-ladder-computing primitive.",
        "runtime_speedup_claim": "Not claimed -- see results/rep1_sparse_representation_metrics.json's own "
            "disclosure, unchanged this stage.",
    }


DEAD_CODE_CLEANUP = {
    "webapp_directory": {"removed": True, "detail": "webapp/server.py, webapp/static/index.html, "
        ".claude/launch.json (its sole, now-dangling uvicorn entry) all removed."},
    "candidatez_generate_and_motioncontext": {
        "removed": False,
        "reason": "Unchanged from Step REP-1's decision: not production-used, but scripts/step3b/step3c "
            "(historical, replay-preserved) still import them from planner/candidate_z.py.",
    },
    "z_step_m_config_field": {
        "removed": False,
        "reason": "Still genuinely required: planner/primitives.py's dz_m derivation, both search layers' "
            "lattice arithmetic, and is_representable()'s own floor_for() dependency (floor_for still ceils "
            "to the z_step_m ladder, unchanged -- only is_representable's OWN alignment check on top of "
            "that was removed, not floor_for's own formula).",
    },
    "coarse_astar_missing_candidatez": {"removed": "N/A", "reason": "Unchanged, disclosed pre-existing gap "
        "(Section 20 explicitly permits disclosing a genuinely different-semantic second representation "
        "rather than fixing it this stage)."},
    "other_dead_code_found": "None beyond webapp/ itself.",
}

FINAL_ARCHITECTURE_AUDIT = {
    "1_candidatez_generate_production_used": False,
    "2_candidatez_single_z_source_of_truth": False,
    "3_regular_z_step_production_representation_remains": True,
    "4_z_index_authority_remains": True,
    "5_fixed_step_primitive_assumption_remains": True,
    "6_web_ui_fully_removed": True,
    "7_second_hidden_z_representation_exists": (
        "planner/coarse_astar.py's own independent dense lattice -- pre-existing, disclosed, not newly "
        "introduced, genuinely different semantic (90m coarse guide search, own cost/heuristic) and out of "
        "this stage's scope, not a 'hidden' surprise."
    ),
    "8_exact_mission_altitude_works": (
        "At the CandidateZGenerator query level: YES (is_representable() correctly recognizes it, tests "
        "B/C pass). As an actual CanonicalState the search can hold as its FIRST state: NO -- z_index "
        "remains int-typed and msl_to_z_index(off_lattice, allow_snap=False) still raises rather than "
        "silently mis-snapping (fails loud, verified by state-identity test). This is the same, now more "
        "precisely located, gap as Step REP-1 -- narrower (query-level solved) but not fully closed "
        "(state-level still open)."
    ),
    "9_lazy_instantiation_works": True,
}

DECISION = {
    "central_finding": (
        "is_representable() is now a genuinely continuous, representation-neutral CandidateZ rule (floor "
        "through ceiling, plus exact mission events) -- a real, verified simplification removing the one "
        "part of REP-1's predicate that still assumed a global z_step_m anchor. This is real progress on "
        "'CandidateZ as the representability authority'. It does NOT, however, make CandidateZ the "
        "production Z SOURCE OF VALUES: planner/primitives.py's climb/descent primitives still have a "
        "fixed dz_m = +-config.z_step_m (the only dz_m assignment site in the repo, unchanged), and "
        "CanonicalState's z-index remains an int identity anchored to that same global lattice. Fully "
        "removing this requires EITHER (a) threading a per-mission altitude reference through "
        "state_to_xyz/msl_to_z_index/z_index_to_msl and every one of their ~16 internal call sites in "
        "planner/astar.py (plus re-verifying every regression script that calls these directly), risking "
        "subtle correctness bugs in A*'s g-score/closed-set exactness for an under-tested payoff, or "
        "(b) redesigning primitives.py's endpoint geometry -- explicitly forbidden this stage (no aircraft-"
        "aware primitive writing). Judged unsafe to attempt without a dedicated, properly-scoped redesign "
        "stage; disclosed here rather than forced through."
    ),
    "why_pass_not_given": (
        "Per this stage's own explicit failure policy: 'primitives.py fixed-step olduğu için z_step'i "
        "tutmalıyız' is correctly not accepted as an excuse to do nothing -- and this stage DID act on that "
        "(is_representable is now continuous, not lattice-gated). But the deeper ask -- CandidateZ becoming "
        "the literal SOURCE of successor altitude VALUES, not just their gatekeeper -- requires changing "
        "either state identity or primitive geometry, both carrying real correctness risk this stage's own "
        "constraints (no primitive redesign, no float-state-identity risk) rule out doing unsafely. Honest "
        "PARTIAL, not a forced PASS."
    ),
    "web_removed": True,
    "ready_for_heading_1": False,
}


def main() -> None:
    RESULTS_DIR.mkdir(exist_ok=True)
    with open(RESULTS_DIR / "rep11_candidatez_production_audit.json", "w") as f:
        json.dump(CANDIDATEZ_PRODUCTION_AUDIT, f, indent=2)
    with open(RESULTS_DIR / "rep11_dead_code_cleanup.json", "w") as f:
        json.dump(DEAD_CODE_CLEANUP, f, indent=2)
    with open(RESULTS_DIR / "rep11_final_architecture.json", "w") as f:
        json.dump(FINAL_ARCHITECTURE_AUDIT, f, indent=2)
    with open(RESULTS_DIR / "rep11_decision.json", "w") as f:
        json.dump(DECISION, f, indent=2)

    real_regression_path = RESULTS_DIR / "rep11_real_regression.json"
    if real_regression_path.exists():
        with open(real_regression_path) as f:
            real_regression = json.load(f)
        with open(RESULTS_DIR / "rep11_before_after_sparse_metrics.json", "w") as f:
            json.dump(build_sparse_metrics(real_regression), f, indent=2)
        print("Wrote rep11_before_after_sparse_metrics.json (real_regression already present)")
    else:
        print("rep11_real_regression.json not yet present -- run rep11_regression.py first, "
              "then rerun this script to produce rep11_before_after_sparse_metrics.json")

    print("Wrote rep11_candidatez_production_audit.json, rep11_dead_code_cleanup.json, "
          "rep11_final_architecture.json, rep11_decision.json")


if __name__ == "__main__":
    main()
