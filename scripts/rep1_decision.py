"""Step REP-1: dead-code decision + final decision artifacts.

Split from scripts/rep1_analysis.py only because the regression evidence
(results/rep1_real_regression.json, produced by a separate rerun of
scripts/step3e_production_representation_integration.py after the
is_representable() wiring) is folded in by hand once that rerun
completes -- see REP1_REPORT.md for the actual numbers.
"""
import json
from pathlib import Path

RESULTS_DIR = Path("results")

DEAD_CODE_CLEANUP = {
    "candidatez_generate_and_motioncontext": {
        "production_used": False,
        "removed": False,
        "reason": (
            "CandidateZGenerator.generate()/MotionContext/GeneratorStats are not called by any "
            "production (planner/, webapp/) code path, before or after Step REP-1 -- confirmed. However "
            "scripts/step3b_sparse_lazy_z_prototype.py and scripts/step3c_persistent_terrain_cache.py "
            "(both historical, already-passed validation scripts) import these names directly FROM "
            "planner/candidate_z.py (moved there verbatim in Step 3E) and would break (ImportError/"
            "AttributeError) if removed. Per the standing project rule that historical validation scripts "
            "stay RUNNABLE, not just their recorded results preserved, these are kept -- this is not "
            "'belki lazım olur' hoarding: it is a named, tested, historically load-bearing API whose "
            "production role is now clarified (not the live successor-generation path) rather than deleted "
            "out from under scripts that still import it."
        ),
    },
    "z_step_m_config_field": {
        "production_used": True,
        "removed": False,
        "reason": (
            "Genuinely required, multiple independent roles: (1) planner/primitives.py derives every "
            "climb/descent primitive's dz_m directly from it; (2) planner/astar.py's CanonicalState z_index "
            "IS z_index*z_step_m -- the production search state space; (3) planner/coarse_astar.py's "
            "separate coarse lattice also derives from it; (4) CandidateZGenerator.is_representable() "
            "(new, Step REP-1) uses it as the ladder-alignment spacing for its CLASS-A check. Not 'only for "
            "an obsolete lattice' -- removing it would break primitive generation, both search layers, and "
            "the new representability predicate. No rename performed either (Section 8's 'don't rename just "
            "to preserve legacy' applies in reverse here: the name still accurately describes its role)."
        ),
    },
    "webapp_server_missing_candidatez": {
        "production_used": False,
        "removed": "N/A -- not code to remove, a gap to disclose",
        "reason": (
            "webapp/server.py (the one live interactive production tool) never constructed a "
            "CandidateZGenerator or passed one to astar_search/ara_star_search/coarse_astar_search, before "
            "OR after Step REP-1. Wiring the (now more capable) is_representable() prefilter into the live "
            "webapp would be a real, additional efficiency win (proven safety-neutral by Step 3E/CLEAN-1's "
            "own regression: floor_for()-based prefiltering never changes which path is found) -- but doing "
            "so safely requires building/attaching a persistent-cache-backed store per request and was not "
            "exercised by this stage's own regression harness. Flagged as a FOLLOW-UP, not implemented here "
            "to avoid an under-tested change to the one live interactive tool."
        ),
    },
    "coarse_astar_missing_candidatez": {
        "production_used": False,
        "removed": "N/A -- pre-existing scope boundary",
        "reason": (
            "planner/coarse_astar.py has never imported CandidateZGenerator at all (not just unused -- "
            "structurally absent). Extending the coarse search with the same prefilter is a parallel, "
            "plausible future improvement but touches a second search implementation this stage's explicit "
            "scope (Section 21: 'A*, ARA*, ... coarse search ... mantığı değişmeyecek') did not authorize "
            "touching. Disclosed, not implemented."
        ),
    },
    "other_dead_code_found": "None. No stale imports, no orphaned helper functions, no legacy config "
                              "options beyond z_step_m (which is required, see above) were found during "
                              "this audit.",
}

DECISION = {
    "central_finding": (
        "CandidateZGenerator.generate()'s literal return value (a 2-4 element boundary-EVENT set: floor, "
        "ceiling, start/goal) is architecturally unsuited to serve as the successor Z-candidate SET for "
        "primitive-arithmetic-based stepping, because every interior z_step_m-ladder rung strictly between "
        "floor and ceiling -- which is most of what real multi-hop paths actually traverse -- is never one "
        "of those events. The regular z_step_m lattice therefore remains a GENUINELY NECESSARY search-state "
        "representation, not an obsolete one to be removed. What COULD and WAS closed: a new, unified "
        "CandidateZGenerator.is_representable(row, col, z_msl) predicate (built from the SAME existing "
        "floor_for()/MissionContext infrastructure, not a new candidate system) now IS the real production "
        "representability authority in planner/astar.py's _generate_neighbors, replacing the narrower "
        "floor-only check, and correctly recognizes an off-lattice mission start/goal altitude as "
        "representable at its own specific cell -- closing the 'exact mission altitude forced onto the "
        "wrong altitude' gap CLASS-C left open, for the one case (the mission's own start/goal state) where "
        "it is actually reachable under the current int-typed CanonicalState.z_index."
    ),
    "blocker": {
        "why_required": "planner/primitives.py's climb/descent primitives have a FIXED dz_m = "
                        "+-config.z_step_m; multi-hop paths accumulate exact integer multiples of this "
                        "step. CanonicalState.z_index must stay an exact, hashable integer for A*'s "
                        "g-score/closed-set dictionaries to remain correct (a float-typed altitude "
                        "accumulated via repeated primitive addition risks floating-point drift breaking "
                        "state-equality/closed-set membership -- a genuine correctness risk to A*'s "
                        "optimality guarantee, not a style preference).",
            "what_semantic": "The z_step_m lattice is the state space itself, not a display/quantization "
                             "convenience layered on top of something else.",
            "why_candidatez_alone_insufficient": "generate()'s sparse event set has no mechanism to "
                                                   "enumerate 'every ladder rung between floor and ceiling' "
                                                   "without becoming the dense enumeration it was designed "
                                                   "to avoid -- and even if it did, primitives.py's fixed "
                                                   "+-z_step_m arithmetic would still need that dense ladder "
                                                   "to land on, not an arbitrary sparse set.",
        },
    "questions": {
        "1_current_candidatez_representation": "is_representable(): CLASS-A terrain floor through mission "
            "ceiling on the z_step_m ladder, OR CLASS-B exact mission start/goal event at that cell. Real "
            "production authority in _generate_neighbors (Step REP-1), replacing the floor-only check.",
        "2_regular_base_lattice_role": "Still required as the actual search state space (see blocker).",
        "3_aircraft_physical_motion_changes_candidatez_or_only_edge": "Only edge feasibility -- CLASS-C's "
            "decision A stands unchanged; not revisited this stage.",
        "4_search_history_independent_motion_events_needed": False,
        "5_residual_vertical_progress_state_needed": False,
        "6_representable_vs_physically_reachable_closed": "Already closed in Step CLASS-C; unaffected here.",
        "7_mission_exact_altitude_bridge_policy_net": True,
        "8_terrain_floor_conservative": True,
        "9_heading_aware_primitives_use_contract_directly": True,
    },
}


def main() -> None:
    RESULTS_DIR.mkdir(exist_ok=True)
    with open(RESULTS_DIR / "rep1_dead_code_cleanup.json", "w") as f:
        json.dump(DEAD_CODE_CLEANUP, f, indent=2)
    with open(RESULTS_DIR / "rep1_decision.json", "w") as f:
        json.dump(DECISION, f, indent=2)
    print("Wrote results/rep1_dead_code_cleanup.json and results/rep1_decision.json")


if __name__ == "__main__":
    main()
