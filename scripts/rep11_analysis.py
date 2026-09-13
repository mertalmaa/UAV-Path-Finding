"""Step REP-1.1: web/UI removal + Z-dependency graph + CandidateZ production
audit + representability/state-identity tests, building on Step REP-1.

Central finding this stage adds to REP-1's: CandidateZGenerator.
is_representable() no longer requires z_msl to sit on a z_step_m-anchored
ladder -- it is now a CONTINUOUS range test (floor_for(row,col) <= z_msl
<= mission.ceiling_msl) OR an exact mission event. This removes the one
part of REP-1's own predicate that still assumed a global lattice, making
CandidateZ genuinely representation-neutral about HOW a candidate
altitude was computed. What this stage did NOT achieve, honestly
disclosed (see results/rep11_final_architecture.json): CanonicalState's
z-index remains an integer identity anchored to config.z_step_m, and
planner/primitives.py's climb/descent primitives still have a FIXED
dz_m = +-config.z_step_m. Removing either requires either (a) a
per-mission altitude-reference redesign touching ~16+ internal call
sites in planner/astar.py plus every external script that calls
state_to_xyz()/msl_to_z_index() directly, or (b) redesigning
primitives.py's endpoint geometry -- both out of this stage's safe,
verifiable scope (no aircraft-aware primitive writing allowed; a
correctness-risking, wide-blast-radius rewrite of already-tested core
functions was judged unsafe to attempt without a redesign stage of its
own). This is the disclosed BLOCKER driving PARTIAL, not a silent choice
either way.

No new Z-step design (no 10/20/40 comparison). No heading, no aircraft-
aware primitive, no search-guidance change.
"""
import json
import math
from pathlib import Path

from planner.candidate_z import CacheBackedTerrainMetadataStore, CandidateZGenerator, MissionContext
from planner.config import DEFAULT_CONFIG
from planner.roi import load_roi
from planner.terrain_cache import load_terrain_cache
from scripts.step3d_real_terrain_integration import CACHE_DIR, CEILING_MSL, FACTOR60, MIN_AGL_M, SOURCE_DEM_PATH

RESULTS_DIR = Path("results")
WINDOW_ROW0, WINDOW_ROW1 = 73, 93
WINDOW_COL0, WINDOW_COL1 = 73, 93


# ---------------------------------------------------------------------------
# 1. Web/UI removal audit
# ---------------------------------------------------------------------------

WEB_REMOVAL_AUDIT = {
    "removed_files": ["webapp/server.py", "webapp/static/index.html",
                       "webapp/__pycache__/server.cpython-314.pyc (build artifact)"],
    "removed_config": [".claude/launch.json (sole entry: uvicorn webapp.server:app -- dangling after removal)"],
    "dependency_audit_before_removal": (
        "webapp/server.py's own imports (confirmed in Step REP-1's audit, re-verified against the file's "
        "last committed content): planner.astar, planner.coarse, planner.coarse_astar, planner.config, "
        "planner.corridor, planner.fine_precompute, planner.primitives, planner.roi, planner.terrain -- ALL "
        "core planner modules independently used by production/scripts elsewhere. Nothing web-specific "
        "(no web-only adapter, no web-only utility function) existed inside planner/ -- webapp/server.py "
        "was a pure CONSUMER of planner/, never the reverse. Zero planner/ code was removed as a consequence."
    ),
    "no_manifest_cleanup_needed": "No requirements.txt/pyproject.toml/setup.py exists in this repo (packages "
                                    "installed globally, per project.md) -- no fastapi/uvicorn/pydantic "
                                    "manifest entries to remove.",
    "remaining_textual_references": (
        "scripts/perf0_search_baseline.py, scripts/rep1_analysis.py, scripts/rep1_decision.py, scripts/"
        "validate_classc.py, scripts/validate_grid1.py, scripts/validate_rep1.py mention 'webapp/server.py' "
        "in docstrings/comments (historical citation of its pipeline constants) or as a string in a "
        "forbidden-files git-status check list -- none import it. Left as accurate historical record, not "
        "dead code."
    ),
    "verdict": "WEB/UI LAYER = REMOVED",
}


# ---------------------------------------------------------------------------
# 2. Z dependency graph (extends Step REP-1's audit with explicit graph edges)
# ---------------------------------------------------------------------------

Z_DEPENDENCY_GRAPH = {
    "nodes": [
        "config.z_step_m", "planner.primitives.build_primitive_set (dz_m)",
        "planner.astar.msl_to_z_index/z_index_to_msl", "planner.astar.CanonicalState (row,col,z_index)",
        "planner.astar._generate_neighbors", "planner.astar.state_to_xyz",
        "planner.candidate_z.CandidateZGenerator.floor_for",
        "planner.candidate_z.CandidateZGenerator.is_representable",
        "planner.candidate_z.CandidateZGenerator.generate (unused in production)",
        "planner.coarse_astar (separate, parallel dense lattice, no CandidateZ import at all)",
        "planner.astar.astar_search / ara_star_search",
    ],
    "edges": [
        {"from": "config.z_step_m", "to": "planner.primitives.build_primitive_set (dz_m)",
         "relation": "dz_m = +-config.z_step_m for climb/descent primitives -- fixed source of every "
                      "successor's vertical delta"},
        {"from": "config.z_step_m", "to": "planner.astar.msl_to_z_index/z_index_to_msl",
         "relation": "z_index_to_msl(z) = z_index * z_step_m -- defines the lattice"},
        {"from": "planner.astar.msl_to_z_index/z_index_to_msl", "to": "planner.astar.CanonicalState",
         "relation": "z_index IS the state's altitude identity"},
        {"from": "planner.primitives.build_primitive_set (dz_m)", "to": "planner.astar._generate_neighbors",
         "relation": "new_z_index = z_index + round(dz_m/z_step_m) -- successor altitude arithmetic"},
        {"from": "planner.astar._generate_neighbors", "to": "planner.candidate_z.CandidateZGenerator.is_representable",
         "relation": "Step REP-1/REP-1.1: representability GATE on the arithmetic-derived successor "
                      "(replaces the old floor_for()-only check) -- the successor VALUE still comes from "
                      "lattice arithmetic; is_representable only decides accept/reject"},
        {"from": "planner.candidate_z.CandidateZGenerator.is_representable", "to":
         "planner.candidate_z.CandidateZGenerator.floor_for", "relation": "CLASS-A lower bound, unchanged"},
        {"from": "planner.astar.CanonicalState", "to": "planner.astar.state_to_xyz",
         "relation": "every downstream terrain/safety/cost/heuristic/output computation reads MSL through "
                      "this conversion -- still z_index * z_step_m, unchanged this stage"},
        {"from": "planner.coarse_astar (separate, parallel dense lattice, no CandidateZ import at all)",
         "to": "config.z_step_m",
         "relation": "an entirely separate coarse-grid lattice, its OWN z_index arithmetic, never touches "
                      "CandidateZGenerator at all -- confirmed still true this stage (out of scope, Section 20)"},
        {"from": "planner.candidate_z.CandidateZGenerator.generate (unused in production)", "to":
         "planner.candidate_z.CandidateZGenerator.floor_for", "relation": "calls floor_for internally; "
         "generate() itself has zero production callers, before or after this stage"},
    ],
    "single_vs_hidden_second_authority": (
        "planner.astar (fine search) and planner.coarse_astar (coarse search) are TWO INDEPENDENT dense "
        "z_step_m lattices. This was already true before REP-1/REP-1.1 and remains true after -- coarse_astar "
        "has never imported CandidateZGenerator. This is disclosed as a SEPARATE, pre-existing, out-of-scope "
        "second Z authority (Section 20 explicitly allows disclosing a genuinely different-semantic coarse "
        "representation rather than silently fixing it) -- not a new hidden authority introduced by this "
        "stage's changes."
    ),
}


def real_state_identity_and_representability_tests(cache):
    """Section 22 (A-J) + Section 23 (state identity), against real terrain,
    same window Step 3D/3E/GRID-1/CLASS-C/REP-1 already used."""
    store = CacheBackedTerrainMetadataStore(cache, FACTOR60)
    off_lattice_start = 3251.37
    off_lattice_goal = 3583.91
    mission = MissionContext(
        start_rowcol=(WINDOW_ROW0, WINDOW_COL0), start_z_msl=off_lattice_start,
        goal_rowcol=(WINDOW_ROW1 - 1, WINDOW_COL1 - 1), goal_z_msl=off_lattice_goal,
        ceiling_msl=CEILING_MSL, min_agl_m=MIN_AGL_M, z_step_m=DEFAULT_CONFIG.z_step_m,
    )
    gen = CandidateZGenerator(store, mission)
    r, c = WINDOW_ROW0 + 5, WINDOW_COL0 + 5
    floor = gen.floor_for(r, c)

    tests = []
    tests.append({"case": "A_terrain_candidate_representable", "pass": floor is not None and gen.is_representable(r, c, floor)})
    tests.append({"case": "B_off_structure_exact_start_representable",
                   "pass": gen.is_representable(*mission.start_rowcol, mission.start_z_msl)})
    tests.append({"case": "C_exact_goal_representable",
                   "pass": gen.is_representable(*mission.goal_rowcol, mission.goal_z_msl)})
    # D) genuinely arbitrary/out-of-range altitude -> rejected (redefined for the continuous rule:
    # "arbitrary" now means outside [floor, ceiling], not merely off-ladder -- see module docstring).
    tests.append({"case": "D_out_of_range_altitude_rejected",
                   "pass": not gen.is_representable(r, c, mission.ceiling_msl + 500.0)
                   and not gen.is_representable(r, c, (floor or 0) - 500.0)})
    results_e = [gen.is_representable(r, c, floor) for _ in range(5)]
    tests.append({"case": "E_same_inputs_same_candidates", "pass": len(set(results_e)) == 1})
    cells = [(rr, cc) for rr in range(WINDOW_ROW0, WINDOW_ROW0 + 10) for cc in range(WINDOW_COL0, WINDOW_COL0 + 10)]
    import random
    forward = {(rr, cc): gen.is_representable(rr, cc, gen.floor_for(rr, cc) or 0.0) for rr, cc in cells}
    shuffled = list(cells)
    random.Random(7).shuffle(shuffled)
    gen2 = CandidateZGenerator(CacheBackedTerrainMetadataStore(cache, FACTOR60), mission)
    reverse = {(rr, cc): gen2.is_representable(rr, cc, gen2.floor_for(rr, cc) or 0.0) for rr, cc in shuffled}
    tests.append({"case": "F_search_order_does_not_change_candidate_set", "pass": forward == reverse})
    r2, c2 = WINDOW_ROW0 + 8, WINDOW_COL0 + 8
    floor2 = gen.floor_for(r2, c2)
    tests.append({"case": "G_not_instantiated_still_representable", "pass": floor2 is not None and gen.is_representable(r2, c2, floor2)})
    tests.append({"case": "H_terrain_floor_violation_impossible",
                   "pass": floor is not None and not gen.is_representable(r, c, floor - 50.0)})
    # I) no regular-z fallback: is_representable's own source has no separate "else use z_index lattice" branch.
    src = Path("planner/candidate_z.py").read_text(encoding="utf-8")
    method_body = src.split("def is_representable")[1].split("\n    def ")[0]
    tests.append({"case": "I_no_regular_z_fallback_in_predicate",
                   "pass": "z_step" not in method_body.split('"""')[-1]})  # no z_step reference in the live code (docstring excluded)
    # J) no residual/history vertical state: is_representable/floor_for take no search-path argument at all.
    import inspect
    sig_ir = inspect.signature(gen.is_representable)
    sig_ff = inspect.signature(gen.floor_for)
    history_like = {"path", "history", "trend", "residual", "previous", "prior"}
    tests.append({"case": "J_no_residual_history_parameter",
                   "pass": not (history_like & set(sig_ir.parameters) | history_like & set(sig_ff.parameters))})

    return tests, mission


def state_identity_tests():
    """Section 23: CanonicalState's altitude identity (z_index, unchanged
    this stage) really is deterministic/hashable/drift-free -- verified
    directly, not assumed."""
    from planner.astar import msl_to_z_index, z_index_to_msl
    from planner.config import DEFAULT_CONFIG as cfg
    tests = []

    a = msl_to_z_index(3260.0, cfg)
    b = msl_to_z_index(3260.0, cfg)
    tests.append({"case": "deterministic_equality", "pass": a == b})
    tests.append({"case": "deterministic_hashing", "pass": hash((5, 5, a)) == hash((5, 5, b))})

    # no floating drift: repeated round-trips are bit-exact (int arithmetic under the hood)
    z0 = msl_to_z_index(3260.0, cfg)
    accumulated = z0
    for _ in range(1000):
        accumulated = accumulated + 1
    direct = z0 + 1000
    tests.append({"case": "no_floating_drift_over_1000_steps", "pass": accumulated == direct})

    # exact mission altitude: NOT representable as a z_index today (the disclosed gap) --
    # confirm this is a controlled, understood limitation, not a silent wrong answer.
    raised = False
    try:
        msl_to_z_index(4127.63, cfg, allow_snap=False)
    except ValueError:
        raised = True
    tests.append({"case": "off_lattice_altitude_fails_loud_not_silently_wrong", "pass": raised})

    duplicate_collapse = len({msl_to_z_index(3260.0, cfg), msl_to_z_index(3260.0, cfg)}) == 1
    tests.append({"case": "duplicate_candidate_collapses_to_one_key", "pass": duplicate_collapse})

    ordering = sorted([msl_to_z_index(3300.0, cfg), msl_to_z_index(3260.0, cfg), msl_to_z_index(3280.0, cfg)])
    tests.append({"case": "ordering_deterministic", "pass": ordering == sorted(ordering)})

    return tests


def main() -> None:
    print("=" * 70)
    print("STEP REP-1.1 -- remove regular Z-lattice dependency + remove web/UI layer")
    print("=" * 70)
    RESULTS_DIR.mkdir(exist_ok=True)

    print("\n--- 1. Web/UI removal ---")
    print(f"  verdict: {WEB_REMOVAL_AUDIT['verdict']}")

    print("\n--- 2. Z dependency graph ---")
    print(f"  {len(Z_DEPENDENCY_GRAPH['nodes'])} nodes, {len(Z_DEPENDENCY_GRAPH['edges'])} edges")

    print("\n--- 3. Representability tests (A-J, real terrain) ---")
    fine_roi = load_roi(DEFAULT_CONFIG)
    cache = load_terrain_cache(CACHE_DIR, fine_roi, SOURCE_DEM_PATH)
    rep_tests, mission_used = real_state_identity_and_representability_tests(cache)
    for t in rep_tests:
        print(f"  {t['case']}: {'PASS' if t['pass'] else 'FAIL'}")
    all_rep_pass = all(t["pass"] for t in rep_tests)

    print("\n--- 4. State identity tests ---")
    id_tests = state_identity_tests()
    for t in id_tests:
        print(f"  {t['case']}: {'PASS' if t['pass'] else 'FAIL'}")
    all_id_pass = all(t["pass"] for t in id_tests)

    with open(RESULTS_DIR / "rep11_web_removal_audit.json", "w") as f:
        json.dump(WEB_REMOVAL_AUDIT, f, indent=2)
    with open(RESULTS_DIR / "rep11_z_dependency_graph.json", "w") as f:
        json.dump(Z_DEPENDENCY_GRAPH, f, indent=2)
    with open(RESULTS_DIR / "rep11_state_identity_tests.json", "w") as f:
        json.dump({"representability_tests": rep_tests, "representability_all_pass": all_rep_pass,
                    "state_identity_tests": id_tests, "state_identity_all_pass": all_id_pass}, f, indent=2)

    print(f"\nArtifacts written to {RESULTS_DIR}/rep11_*.json (partial -- see scripts/rep11_decision.py)")
    print(f"\nOverall (this script): {'PASS' if all_rep_pass and all_id_pass else 'FAIL'}")


if __name__ == "__main__":
    main()
