"""Step GRID-1: validation checklist (section 22).

Confirms: real V3 AircraftProfile is used (no hard-coded capability
numbers in this repo's test code), LEFT/RIGHT stay separate, multiple
altitudes were queried, 30/60/90m spacings are exact, MAX terrain
aggregation stays conservative (zero unsafe-optimism), no search/heading/
primitive code was touched, and the historical Step 2E/2F artifacts this
stage reused are unchanged on disk.
"""
import hashlib
import json
import subprocess
from pathlib import Path

from planner.aircraft_profile import load_aircraft_profile

V3_PATH = "jsbsim/results/c172p_aircraft_profile_planner_safe_v3.json"
RESULTS_DIR = Path("results")


def _sha256(path: str) -> str:
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def validation_1_artifacts_exist_and_consistent() -> bool:
    print("=== 1: GRID-1 artifacts exist and are internally consistent ===")
    results = []
    for name in ("grid1_aircraft_geometry_metrics.json", "grid1_arc_representation_errors.json",
                 "grid1_terrain_aircraft_regate.json", "grid1_resolution_decision.json"):
        path = RESULTS_DIR / name
        ok = path.exists()
        results.append(ok)
        print(f"  {path}: {'exists' if ok else 'MISSING'}  {'PASS' if ok else 'FAIL'}")

    with open(RESULTS_DIR / "grid1_aircraft_geometry_metrics.json") as f:
        geometry = json.load(f)
    ok = geometry["grid_spacings_m"] == {"30m": 30.0, "60m": 60.0, "90m": 90.0}
    results.append(ok)
    print(f"  exact 30/60/90m spacings declared: {geometry['grid_spacings_m']}  {'PASS' if ok else 'FAIL'}")

    ok = geometry["guaranteed_bank_deg"] == 20.0
    results.append(ok)
    print(f"  guaranteed bank used = 20deg (not 30): {geometry['guaranteed_bank_deg']}  {'PASS' if ok else 'FAIL'}")
    return all(results)


def validation_2_v3_source_used() -> bool:
    print()
    print("=== 2: V3 AircraftProfile is the geometry source, no hard-coded numbers ===")
    with open(RESULTS_DIR / "grid1_resolution_decision.json") as f:
        decision = json.load(f)
    ok = decision["aircraft_source"] == V3_PATH and decision["aircraft_id"] == "c172p"
    print(f"  aircraft_source={decision['aircraft_source']} aircraft_id={decision['aircraft_id']}  "
          f"{'PASS' if ok else 'FAIL'}")

    # cross-check: every radius recorded in the artifact matches a LIVE V3 query,
    # proving the numbers were read from the profile, not typed in by hand.
    profile = load_aircraft_profile(V3_PATH)
    with open(RESULTS_DIR / "grid1_aircraft_geometry_metrics.json") as f:
        geometry = json.load(f)
    mismatches = []
    for row in geometry["turn_geometry"]:
        live = profile.turn_query(row["altitude_m"], row["direction"], row["bank_deg"])
        if live.planner_safe.get("turn_radius_m") != row["turn_radius_m"]:
            mismatches.append(row)
    ok2 = not mismatches
    print(f"  {len(geometry['turn_geometry'])} recorded turn radii re-verified against a live V3 query: "
          f"{len(mismatches)} mismatches  {'PASS' if ok2 else 'FAIL'}")
    return ok and ok2


def validation_3_left_right_separated() -> bool:
    print()
    print("=== 3: LEFT/RIGHT kept separate (never averaged) ===")
    with open(RESULTS_DIR / "grid1_aircraft_geometry_metrics.json") as f:
        geometry = json.load(f)
    by_alt = {}
    for row in geometry["turn_geometry"]:
        by_alt.setdefault(row["altitude_m"], {})[row["direction"]] = row["turn_radius_m"]
    all_different = all(v["LEFT"] != v["RIGHT"] for v in by_alt.values())
    n_altitudes = len(by_alt)
    print(f"  {n_altitudes} altitudes, LEFT != RIGHT radius at every one: {'PASS' if all_different else 'FAIL'}")
    return all_different and n_altitudes >= 6


def validation_4_multiple_altitudes() -> bool:
    print()
    print("=== 4: multiple representative altitudes queried ===")
    with open(RESULTS_DIR / "grid1_aircraft_geometry_metrics.json") as f:
        geometry = json.load(f)
    altitudes = sorted({row["altitude_m"] for row in geometry["turn_geometry"]})
    required = {0.0, 1000.0, 2500.0, 4000.0, 4500.0, 5500.0}
    ok = required.issubset(set(altitudes))
    print(f"  altitudes tested: {altitudes}  {'PASS' if ok else 'FAIL'}")
    return ok


def validation_5_conservative_aggregation() -> bool:
    print()
    print("=== 5: MAX terrain aggregation stays conservative (zero unsafe-optimism) ===")
    with open(RESULTS_DIR / "grid1_terrain_aircraft_regate.json") as f:
        terrain = json.load(f)
    n_violations = len(terrain["unsafe_optimism_violations"])
    ok = n_violations == 0
    print(f"  unsafe_optimism_violations={n_violations}  {'PASS' if ok else 'FAIL -- CRITICAL'}")

    # every recorded additional_conservatism_m must be >= 0 (coarse never optimistic)
    negative = []
    for wname, cases in terrain["windows"].items():
        for case_name, data in cases.items():
            for label in ("60m", "90m"):
                mn = data["additional_conservatism_m"][label]["min"]
                if mn is not None and mn < -1e-6:
                    negative.append((wname, case_name, label, mn))
    ok2 = not negative
    print(f"  negative additional_conservatism_m entries: {len(negative)}  {'PASS' if ok2 else 'FAIL'}")
    return ok and ok2


def validation_6_no_search_or_heading_changes() -> bool:
    print()
    print("=== 6: no search/heading/primitive files modified this stage ===")
    out = subprocess.run(["git", "status", "--porcelain"], capture_output=True, text=True, check=True).stdout
    touched = [line[3:] for line in out.splitlines() if line.strip()]
    forbidden_paths = ("planner/astar.py", "planner/primitives.py", "planner/corridor.py",
                       "planner/candidate_z.py", "planner/terrain_cache.py", "planner/coarse_astar.py",
                       "planner/fine_precompute.py", "planner/mission.py", "webapp/server.py")
    violations = [t for t in touched if t in forbidden_paths]
    ok = not violations
    print(f"  search/production files touched: {violations if violations else 'none'}  {'PASS' if ok else 'FAIL'}")
    return ok


def validation_7_historical_artifacts_unchanged() -> bool:
    print()
    print("=== 7: historical Step 2E/2F scripts reused, unchanged on disk ===")
    # git status is only meaningful for TRACKED files -- these 4 scripts are tracked
    # (committed in earlier stages), so a modified/staged flag here would be real.
    out = subprocess.run(["git", "status", "--porcelain"], capture_output=True, text=True, check=True).stdout
    touched = {line[3:] for line in out.splitlines() if line.strip()}
    historical_scripts = [
        "scripts/step2e_xy_resolution_comparison.py", "scripts/step2f_multi_region_xy_validation.py",
        "scripts/step2b_information_loss_diagnostics.py", "scripts/step2d_preservation_validation.py",
    ]
    violations = [h for h in historical_scripts if h in touched]
    ok = not violations
    print(f"  historical scripts unexpectedly modified: {violations if violations else 'none'}  {'PASS' if ok else 'FAIL'}")

    # V3 artifact is untracked (owned by the parallel JSBSim session, not yet committed) --
    # git status can't distinguish "I modified it" from "pre-existing untracked file", so
    # verify by content instead: this session only ever READ it (json.load / load_aircraft_
    # profile), and its own provenance_id must match the value recorded during Step ALG-1.
    with open(V3_PATH) as f:
        v3_data = json.load(f)
    ok2 = v3_data.get("provenance_id") == "u6.2.2-b8e1351477e87cc148e9"
    print(f"  V3 artifact provenance_id unchanged from Step ALG-1's own record: "
          f"{v3_data.get('provenance_id')}  {'PASS' if ok2 else 'FAIL'}")
    return ok and ok2


def main() -> None:
    results = [
        validation_1_artifacts_exist_and_consistent(),
        validation_2_v3_source_used(),
        validation_3_left_right_separated(),
        validation_4_multiple_altitudes(),
        validation_5_conservative_aggregation(),
        validation_6_no_search_or_heading_changes(),
        validation_7_historical_artifacts_unchanged(),
    ]
    print()
    print(f"Overall: {'ALL PASS' if all(results) else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
