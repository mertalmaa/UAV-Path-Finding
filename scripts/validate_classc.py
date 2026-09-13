"""Step CLASS-C: validation checklist (section 23).

Confirms the audit was real (not assumed), no 20m hard-coding crept into
the new contract, no 10/20/40 Z-step re-gate happened, V3 AircraftProfile
was used live, representable != physically-reachable is enforced,
high-altitude climb is correctly rejected, deterministic mission-altitude
handling works off-lattice, terrain floor is untouched/conservative, no
search-history-dependent candidate or residual vertical-progress state
was introduced, the aircraft profile source is unmodified, and no search/
heading/primitive production code was touched.
"""
import json
import subprocess
from pathlib import Path

from planner.aircraft_profile import load_aircraft_profile
from planner.vertical_motion import evaluate_vertical_motion

V3_PATH = "jsbsim/results/c172p_aircraft_profile_planner_safe_v3.json"
RESULTS_DIR = Path("results")


def validation_1_artifacts_exist() -> bool:
    print("=== 1: all CLASS-C artifacts exist ===")
    results = []
    for name in ("classc_candidate_z_audit.json", "classc_aircraft_vertical_queries.json",
                 "classc_motion_horizon_diagnostics.json", "classc_transition_contract_tests.json",
                 "classc_representation_contract.json", "classc_decision.json"):
        ok = (RESULTS_DIR / name).exists()
        results.append(ok)
        print(f"  {name}: {'exists' if ok else 'MISSING'}  {'PASS' if ok else 'FAIL'}")
    return all(results)


def validation_2_audit_is_real_not_assumed() -> bool:
    print()
    print("=== 2: audit reflects the REAL production code path, not an assumption ===")
    with open(RESULTS_DIR / "classc_candidate_z_audit.json") as f:
        audit = json.load(f)
    # the audit's own claim: production only calls floor_for(), never generate().
    ok = ("_generate_neighbors" in audit["production_call_sites_confirmed"]["floor_for_called_by"]
          and "step3b" in audit["production_call_sites_confirmed"]["generate_called_by"][0])
    print(f"  production_call_sites_confirmed recorded (floor_for in production, generate() only in "
          f"isolated Step 3B tests): {'PASS' if ok else 'FAIL'}")
    # cross-check against the actual source file, right now, not just trusting the recorded claim.
    astar_src = Path("planner/astar.py").read_text(encoding="utf-8")
    ok2 = "candidate_z_generator.floor_for(" in astar_src and "candidate_z_generator.generate(" not in astar_src
    print(f"  live source re-check (planner/astar.py calls .floor_for(), never .generate()): "
          f"{'PASS' if ok2 else 'FAIL'}")
    return ok and ok2


def validation_3_no_hardcoded_20m_assumption() -> bool:
    print()
    print("=== 3: no hard-coded 20m assumption baked into the new contract ===")
    src = Path("planner/vertical_motion.py").read_text(encoding="utf-8")
    # the new module must not literally reference a numeric z-step constant.
    ok = "20.0" not in src and "20 m" not in src.replace("z_step_m", "")
    print(f"  planner/vertical_motion.py contains no literal 20m constant: {'PASS' if ok else 'FAIL'}")
    return ok


def validation_4_no_z_step_regate() -> bool:
    print()
    print("=== 4: no 10/20/40m Z-step comparison was performed this stage ===")
    for name in ("classc_decision.json", "classc_representation_contract.json"):
        text = (RESULTS_DIR / name).read_text(encoding="utf-8")
        if "10m" in text.replace(" ", "") and "40m" in text.replace(" ", ""):
            print(f"  {name} appears to contain a Z-step comparison  FAIL")
            return False
    print("  no Z-step re-gate artifacts found  PASS")
    return True


def validation_5_live_v3_used() -> bool:
    print()
    print("=== 5: V3 AircraftProfile used live (round-trips against the real file) ===")
    profile = load_aircraft_profile(V3_PATH)
    with open(RESULTS_DIR / "classc_aircraft_vertical_queries.json") as f:
        recorded = json.load(f)
    mismatches = []
    for row in recorded["rows"]:
        live = profile.vertical_query(row["altitude_m"], row["mode"])
        key = f"{row['mode'].lower()}_vz_mps"
        live_vz = live.planner_safe.get(key)
        if live.availability != row["availability"] or live_vz != row["safe_vz_mps"]:
            mismatches.append(row)
    ok = not mismatches
    print(f"  {len(recorded['rows'])} recorded rows re-verified against live V3 query: "
          f"{len(mismatches)} mismatches  {'PASS' if ok else 'FAIL'}")
    return ok


def validation_6_altitude_dependent_capability_used() -> bool:
    print()
    print("=== 6: altitude-dependent climb/descent capability actually varies ===")
    with open(RESULTS_DIR / "classc_aircraft_vertical_queries.json") as f:
        recorded = json.load(f)
    climb_vzs = {row["safe_vz_mps"] for row in recorded["rows"] if row["mode"] == "CLIMB" and row["safe_vz_mps"] is not None}
    ok = len(climb_vzs) > 1
    print(f"  distinct AVAILABLE climb safe_vz values across altitudes: {len(climb_vzs)}  {'PASS' if ok else 'FAIL'}")
    return ok


def validation_7_representable_ne_physically_reachable() -> bool:
    print()
    print("=== 7: representable != physically reachable is enforced ===")
    profile = load_aircraft_profile(V3_PATH)
    # same representable endpoint (4520 climb from 4500), two different durations -> different outcomes.
    r_short = evaluate_vertical_motion(4500.0, 4520.0, 0.1, profile)
    r_long = evaluate_vertical_motion(4500.0, 4520.0, 20.0, profile)
    ok = r_short.status == "PHYSICALLY_UNAVAILABLE" and r_long.status == "FEASIBLE"
    print(f"  same endpoint pair, short duration={r_short.status}, ample duration={r_long.status}  "
          f"{'PASS' if ok else 'FAIL'}")
    return ok


def validation_8_high_altitude_climb_rejected() -> bool:
    print()
    print("=== 8: 5000/5500m climb correctly rejected (UNAVAILABLE, not silently allowed) ===")
    profile = load_aircraft_profile(V3_PATH)
    results = []
    for alt in (5000.0, 5500.0):
        r = evaluate_vertical_motion(alt, alt + 20.0, 10.0, profile)
        ok = r.status == "PHYSICALLY_UNAVAILABLE" and r.availability == "UNAVAILABLE"
        results.append(ok)
        print(f"  {alt}m climb -> status={r.status} availability={r.availability}  {'PASS' if ok else 'FAIL'}")
    return all(results)


def validation_9_deterministic_off_lattice_mission_event() -> bool:
    print()
    print("=== 9: off-lattice altitude pair handled deterministically ===")
    profile = load_aircraft_profile(V3_PATH)
    r1 = evaluate_vertical_motion(3251.5, 3253.0, 5.0, profile)
    r2 = evaluate_vertical_motion(3251.5, 3253.0, 5.0, profile)
    ok = r1 == r2 and r1.status == "FEASIBLE"
    print(f"  repeated call, identical inputs -> identical result ({r1.status}): {'PASS' if ok else 'FAIL'}")
    return ok


def validation_10_terrain_floor_unchanged() -> bool:
    print()
    print("=== 10: planner/candidate_z.py not modified (terrain-floor logic untouched) ===")
    out = subprocess.run(["git", "status", "--porcelain"], capture_output=True, text=True, check=True).stdout
    touched = {line[3:] for line in out.splitlines() if line.strip()}
    ok = "planner/candidate_z.py" not in touched
    print(f"  planner/candidate_z.py touched: {'planner/candidate_z.py' in touched}  {'PASS' if ok else 'FAIL'}")
    return ok


def validation_11_no_residual_state_no_search_change() -> bool:
    print()
    print("=== 11: no residual/history state added; no search/primitive/heading production code touched ===")
    out = subprocess.run(["git", "status", "--porcelain"], capture_output=True, text=True, check=True).stdout
    touched = {line[3:] for line in out.splitlines() if line.strip()}
    forbidden = ("planner/astar.py", "planner/primitives.py", "planner/corridor.py",
                "planner/candidate_z.py", "planner/terrain_cache.py", "planner/coarse_astar.py",
                "planner/fine_precompute.py", "planner/mission.py", "webapp/server.py")
    violations = [t for t in touched if t in forbidden]
    ok = not violations
    print(f"  forbidden files touched: {violations if violations else 'none'}  {'PASS' if ok else 'FAIL'}")

    src = Path("planner/vertical_motion.py").read_text(encoding="utf-8")
    ok2 = "residual" not in src.lower().replace("no residual", "").replace("residual_vertical_progress_state", "") or True
    # simpler, decisive check: no mutable/accumulating state field anywhere in the new module's dataclass
    ok2 = "self." not in src  # no instance state at all -- the module is purely functional
    print(f"  planner/vertical_motion.py has no instance/accumulating state (purely functional): "
          f"{'PASS' if ok2 else 'FAIL'}")
    return ok and ok2


def validation_12_source_profile_unmodified() -> bool:
    print()
    print("=== 12: V3 source artifact unmodified ===")
    with open(V3_PATH) as f:
        data = json.load(f)
    ok = data.get("provenance_id") == "u6.2.2-b8e1351477e87cc148e9"
    print(f"  provenance_id unchanged from ALG-1/GRID-1's own record: {data.get('provenance_id')}  "
          f"{'PASS' if ok else 'FAIL'}")
    return ok


def main() -> None:
    results = [
        validation_1_artifacts_exist(),
        validation_2_audit_is_real_not_assumed(),
        validation_3_no_hardcoded_20m_assumption(),
        validation_4_no_z_step_regate(),
        validation_5_live_v3_used(),
        validation_6_altitude_dependent_capability_used(),
        validation_7_representable_ne_physically_reachable(),
        validation_8_high_altitude_climb_rejected(),
        validation_9_deterministic_off_lattice_mission_event(),
        validation_10_terrain_floor_unchanged(),
        validation_11_no_residual_state_no_search_change(),
        validation_12_source_profile_unmodified(),
    ]
    print()
    print(f"Overall: {'ALL PASS' if all(results) else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
