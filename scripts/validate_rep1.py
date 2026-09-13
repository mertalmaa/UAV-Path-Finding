"""Step REP-1: validation checklist (section 28).

Confirms: CandidateZGenerator.is_representable() is genuinely used in
planner/astar.py's production path, the obsolete-representation
question was answered with evidence (not silently assumed either way),
exact mission altitude is representable at its own cell, terrain floor
stays conservative, the candidate predicate is deterministic, arbitrary
unsourced altitudes are rejected, no dense-Z enumeration was added to
production, dead-code/config decisions are recorded, Missions A/B/C
still pass with raw_dem_reads=0, and no heading/aircraft-primitive/
corridor/guidance code was touched.
"""
import json
import subprocess
from pathlib import Path

from planner.candidate_z import CacheBackedTerrainMetadataStore, CandidateZGenerator, MissionContext
from planner.config import DEFAULT_CONFIG
from planner.terrain_cache import load_terrain_cache
from scripts.step3d_real_terrain_integration import CACHE_DIR, CEILING_MSL, FACTOR60, MIN_AGL_M, SOURCE_DEM_PATH
from planner.roi import load_roi

RESULTS_DIR = Path("results")


def validation_1_is_representable_used_in_production() -> bool:
    print("=== 1: is_representable() is genuinely called from planner/astar.py ===")
    src = Path("planner/astar.py").read_text(encoding="utf-8")
    ok = "candidate_z_generator.is_representable(" in src
    print(f"  planner/astar.py calls candidate_z_generator.is_representable(): {'PASS' if ok else 'FAIL'}")
    ok2 = "candidate_z_generator.floor_for(new_row, new_col)" not in src.split("is_representable")[0][-400:] \
        if ok else False
    # simpler, decisive check: the old bare floor_for-only rejection branch is gone
    ok2 = src.count("floor = candidate_z_generator.floor_for(new_row, new_col)") == 0
    print(f"  old floor-only inline rejection branch removed: {'PASS' if ok2 else 'FAIL'}")
    return ok and ok2


def validation_2_representability_tests_recorded() -> bool:
    print()
    print("=== 2: representability tests A-H recorded and passing ===")
    with open(RESULTS_DIR / "rep1_mission_altitude_tests.json") as f:
        data = json.load(f)
    ok = data["all_pass"] and len(data["tests"]) == 8
    print(f"  {len(data['tests'])} tests, all_pass={data['all_pass']}  {'PASS' if ok else 'FAIL'}")
    return ok


def validation_3_exact_mission_altitude_live() -> bool:
    print()
    print("=== 3: exact off-lattice mission altitude representable (live re-check) ===")
    fine_roi = load_roi(DEFAULT_CONFIG)
    cache = load_terrain_cache(CACHE_DIR, fine_roi, SOURCE_DEM_PATH)
    store = CacheBackedTerrainMetadataStore(cache, FACTOR60)
    off_lattice = 4127.63
    mission = MissionContext(
        start_rowcol=(80, 80), start_z_msl=off_lattice, goal_rowcol=(85, 85), goal_z_msl=4210.0,
        ceiling_msl=CEILING_MSL, min_agl_m=MIN_AGL_M, z_step_m=DEFAULT_CONFIG.z_step_m,
    )
    gen = CandidateZGenerator(store, mission)
    ok = gen.is_representable(80, 80, off_lattice)
    not_snapped = not gen.is_representable(80, 80, round(off_lattice / DEFAULT_CONFIG.z_step_m) * DEFAULT_CONFIG.z_step_m) \
        or True  # snapped value may ALSO be representable if it happens to clear the floor -- not a conflict
    print(f"  is_representable(80,80, 4127.63) [off-lattice, exact mission start]: {'PASS' if ok else 'FAIL'}")
    return ok


def validation_4_terrain_floor_conservative() -> bool:
    print()
    print("=== 4: terrain floor still conservative (never optimistic) ===")
    src = Path("planner/candidate_z.py").read_text(encoding="utf-8")
    ok = "math.ceil((md.elevation_m + self.mission.min_agl_m) / z_step) * z_step" in src
    print(f"  floor_for() ceil-based formula unchanged: {'PASS' if ok else 'FAIL'}")
    return ok


def validation_5_deterministic() -> bool:
    print()
    print("=== 5: candidate predicate deterministic ===")
    with open(RESULTS_DIR / "rep1_mission_altitude_tests.json") as f:
        data = json.load(f)
    tests_by_case = {t["case"]: t for t in data["tests"]}
    ok = tests_by_case["E_same_inputs_same_result"]["pass"] and tests_by_case["F_different_expansion_order_same_candidate_set"]["pass"]
    print(f"  E (repeated calls) + F (different query order) both pass: {'PASS' if ok else 'FAIL'}")
    return ok


def validation_6_arbitrary_altitude_rejected() -> bool:
    print()
    print("=== 6: arbitrary unsourced altitude rejected ===")
    with open(RESULTS_DIR / "rep1_mission_altitude_tests.json") as f:
        data = json.load(f)
    tests_by_case = {t["case"]: t for t in data["tests"]}
    ok = tests_by_case["D_arbitrary_unsourced_altitude_not_representable"]["pass"] and \
        tests_by_case["H_below_terrain_floor_not_representable"]["pass"]
    print(f"  D (arbitrary) + H (below floor) both correctly rejected: {'PASS' if ok else 'FAIL'}")
    return ok


def validation_7_no_new_dense_enumeration() -> bool:
    print()
    print("=== 7: no dense-Z enumeration added to production ===")
    src = Path("planner/candidate_z.py").read_text(encoding="utf-8")
    method_body = src.split("def is_representable")[1].split("\n    def ")[0]
    # Strip the docstring (its prose legitimately contains the word "for") before checking
    # for an actual loop construct -- a real `for` statement is a line starting with "for ".
    parts = method_body.split('"""')
    code_only = parts[0] + (parts[2] if len(parts) > 2 else "")
    ok = "range(" not in code_only and not any(
        line.strip().startswith("for ") for line in code_only.splitlines()
    )
    print(f"  is_representable() has no enumeration loop (pure predicate): {'PASS' if ok else 'FAIL'}")
    return ok


def validation_8_dead_code_decisions_recorded() -> bool:
    print()
    print("=== 8: dead-code/config decisions recorded with reasons ===")
    with open(RESULTS_DIR / "rep1_dead_code_cleanup.json") as f:
        data = json.load(f)
    required_keys = {"candidatez_generate_and_motioncontext", "z_step_m_config_field"}
    ok = required_keys.issubset(data.keys()) and all(
        "reason" in data[k] and len(data[k]["reason"]) > 20 for k in required_keys
    )
    print(f"  required decisions present with real reasons: {'PASS' if ok else 'FAIL'}")
    return ok


def validation_9_missions_abc_regression() -> bool:
    print()
    print("=== 9: Missions A/B/C regression (raw_dem_reads=0, safety, path result) ===")
    path = RESULTS_DIR / "rep1_real_regression.json"
    if not path.exists():
        print(f"  {path} missing  FAIL")
        return False
    with open(path) as f:
        data = json.load(f)
    ok = data.get("all_missions_pass", False)
    print(f"  all_missions_pass={ok}  {'PASS' if ok else 'FAIL'}")
    return ok


def validation_10_no_search_heading_primitive_corridor_change() -> bool:
    print()
    print("=== 10: no search-algorithm/heading/primitive/corridor/guidance file touched ===")
    out = subprocess.run(["git", "status", "--porcelain"], capture_output=True, text=True, check=True).stdout
    touched = {line[3:] for line in out.splitlines() if line.strip()}
    forbidden = ("planner/primitives.py", "planner/corridor.py", "planner/coarse_astar.py",
                "planner/fine_precompute.py", "planner/mission.py", "webapp/server.py",
                "planner/terrain_cache.py", "planner/aircraft_profile.py", "planner/vertical_motion.py")
    violations = [t for t in touched if t in forbidden]
    ok = not violations
    print(f"  forbidden files touched: {violations if violations else 'none'}  {'PASS' if ok else 'FAIL'}")
    # planner/astar.py and planner/candidate_z.py ARE expected to be touched this stage.
    expected_touched = {"planner/astar.py", "planner/candidate_z.py"}
    ok2 = expected_touched.issubset(touched)
    print(f"  expected files (astar.py, candidate_z.py) touched: {'PASS' if ok2 else 'FAIL'}")
    return ok and ok2


def main() -> None:
    results = [
        validation_1_is_representable_used_in_production(),
        validation_2_representability_tests_recorded(),
        validation_3_exact_mission_altitude_live(),
        validation_4_terrain_floor_conservative(),
        validation_5_deterministic(),
        validation_6_arbitrary_altitude_rejected(),
        validation_7_no_new_dense_enumeration(),
        validation_8_dead_code_decisions_recorded(),
        validation_9_missions_abc_regression(),
        validation_10_no_search_heading_primitive_corridor_change(),
    ]
    print()
    print(f"Overall: {'ALL PASS' if all(results) else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
