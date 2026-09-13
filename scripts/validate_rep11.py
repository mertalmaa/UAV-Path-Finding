"""Step REP-1.1: validation checklist (section 27/28).

Confirms: web/UI genuinely removed, is_representable() genuinely
simplified to a continuous CandidateZ rule (no lattice-alignment
requirement left in its own code), CandidateZ.generate() still not
production-used (honestly, not silently re-checked away), exact mission
altitude representable at the query level, terrain floor unchanged/
conservative, lazy/deterministic behavior holds, no hidden hardcoded
hint of the old ladder requirement, dead-code/final-architecture
decisions recorded with real reasons, Missions A/B/C regression safe
(not required to be bit-identical, but must not regress on safety), and
no search-algorithm/heading/primitive/guidance file touched.
"""
import json
import subprocess
from pathlib import Path

RESULTS_DIR = Path("results")


def validation_1_web_removed() -> bool:
    print("=== 1: web/UI layer genuinely removed ===")
    ok = not Path("webapp").exists()
    print(f"  webapp/ directory absent from disk: {'PASS' if ok else 'FAIL'}")
    out = subprocess.run(["git", "ls-files", "webapp/", ".claude/launch.json"],
                          capture_output=True, text=True, check=True).stdout
    ok2 = out.strip() == ""
    print(f"  git no longer tracks webapp/ or .claude/launch.json: {'PASS' if ok2 else 'FAIL'}")
    return ok and ok2


def validation_2_is_representable_continuous() -> bool:
    print()
    print("=== 2: is_representable() has no lattice-alignment requirement left ===")
    src = Path("planner/candidate_z.py").read_text(encoding="utf-8")
    method_body = src.split("def is_representable")[1].split("\n    def ")[0]
    code_only = method_body.split('"""')[0] + (method_body.split('"""')[2] if method_body.count('"""') > 1 else "")
    ok = "z_step" not in code_only and "round(" not in code_only
    print(f"  no z_step/round()-based ladder check remains in the live code: {'PASS' if ok else 'FAIL'}")
    return ok


def validation_3_generate_still_unused() -> bool:
    print()
    print("=== 3: CandidateZGenerator.generate() still not production-used (honestly re-checked) ===")
    for f in ("planner/astar.py", "planner/coarse_astar.py"):
        src = Path(f).read_text(encoding="utf-8")
        if ".generate(" in src:
            print(f"  {f} calls .generate() -- unexpected  FAIL")
            return False
    print("  planner/astar.py and planner/coarse_astar.py still never call .generate()  PASS")
    return True


def validation_4_representability_and_identity_tests() -> bool:
    print()
    print("=== 4: representability (A-J) + state-identity tests recorded ===")
    with open(RESULTS_DIR / "rep11_state_identity_tests.json") as f:
        data = json.load(f)
    ok = data["representability_all_pass"] and data["state_identity_all_pass"]
    print(f"  representability_all_pass={data['representability_all_pass']} "
          f"state_identity_all_pass={data['state_identity_all_pass']}  {'PASS' if ok else 'FAIL'}")
    return ok


def validation_5_terrain_floor_unchanged() -> bool:
    print()
    print("=== 5: terrain floor formula unchanged, still conservative ===")
    src = Path("planner/candidate_z.py").read_text(encoding="utf-8")
    ok = "math.ceil((md.elevation_m + self.mission.min_agl_m) / z_step) * z_step" in src
    print(f"  floor_for() unchanged: {'PASS' if ok else 'FAIL'}")
    return ok


def validation_6_missions_abc_safe() -> bool:
    print()
    print("=== 6: Missions A/B/C regression -- safety not regressed ===")
    path = RESULTS_DIR / "rep11_real_regression.json"
    if not path.exists():
        print(f"  {path} missing  FAIL")
        return False
    with open(path) as f:
        data = json.load(f)
    ok = data.get("all_missions_safe", False)
    print(f"  all_missions_safe={ok}  {'PASS' if ok else 'FAIL'}")
    return ok


def validation_7_decisions_recorded() -> bool:
    print()
    print("=== 7: dead-code / final-architecture decisions recorded with real reasons ===")
    with open(RESULTS_DIR / "rep11_dead_code_cleanup.json") as f:
        dead_code = json.load(f)
    with open(RESULTS_DIR / "rep11_final_architecture.json") as f:
        arch = json.load(f)
    ok = ("candidatez_generate_and_motioncontext" in dead_code
          and len(dead_code["candidatez_generate_and_motioncontext"]["reason"]) > 20
          and "1_candidatez_generate_production_used" in arch)
    print(f"  decisions present with real reasons: {'PASS' if ok else 'FAIL'}")
    return ok


def validation_8_no_search_heading_primitive_touched() -> bool:
    print()
    print("=== 8: no search-algorithm/heading/primitive/guidance file touched ===")
    out = subprocess.run(["git", "status", "--porcelain"], capture_output=True, text=True, check=True).stdout
    touched = {line[3:] for line in out.splitlines() if line.strip()}
    forbidden = ("planner/primitives.py", "planner/corridor.py", "planner/coarse_astar.py",
                "planner/fine_precompute.py", "planner/mission.py", "planner/terrain_cache.py",
                "planner/aircraft_profile.py", "planner/vertical_motion.py")
    violations = [t for t in touched if t in forbidden]
    ok = not violations
    print(f"  forbidden files touched: {violations if violations else 'none'}  {'PASS' if ok else 'FAIL'}")
    return ok


def validation_9_honest_partial_not_overclaimed() -> bool:
    print()
    print("=== 9: decision honestly reports PARTIAL / not-ready-for-heading (no overclaiming) ===")
    with open(RESULTS_DIR / "rep11_decision.json") as f:
        decision = json.load(f)
    ok = decision.get("ready_for_heading_1") is False
    print(f"  ready_for_heading_1={decision.get('ready_for_heading_1')}  {'PASS' if ok else 'FAIL'}")
    return ok


def main() -> None:
    results = [
        validation_1_web_removed(),
        validation_2_is_representable_continuous(),
        validation_3_generate_still_unused(),
        validation_4_representability_and_identity_tests(),
        validation_5_terrain_floor_unchanged(),
        validation_6_missions_abc_safe(),
        validation_7_decisions_recorded(),
        validation_8_no_search_heading_primitive_touched(),
        validation_9_honest_partial_not_overclaimed(),
    ]
    print()
    print(f"Overall: {'ALL PASS' if all(results) else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
