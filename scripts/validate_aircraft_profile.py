"""Step ALG-1: integration tests for planner.aircraft_profile against the
real canonical V3 planner-safe artifact, plus a synthetic second-aircraft
fixture proving the loader/query code has no aircraft-specific branching.

No search code is touched or imported here. This does not change planner
behavior in any way.
"""
from planner.aircraft_profile import (
    AircraftProfileQueryError, LutNotPlannerSafeError, load_aircraft_profile,
    load_aircraft_profile_from_dict,
)

V3_PATH = "jsbsim/results/c172p_aircraft_profile_planner_safe_v3.json"
RAW_PATHS = [
    "jsbsim/results/aircraft_lut_raw.json",
    "jsbsim/results/c172p_core_aircraft_lut_raw.json",
    "jsbsim/results/c172p_combined_3d_raw.json",
]


def expect_raises(label, exc_type, fn) -> bool:
    try:
        fn()
    except exc_type:
        print(f"  {label}: raised {exc_type.__name__} as expected  PASS")
        return True
    except Exception as e:
        print(f"  {label}: raised {type(e).__name__} instead of {exc_type.__name__}  FAIL ({e})")
        return False
    print(f"  {label}: did NOT raise {exc_type.__name__}  FAIL")
    return False


def validation_1_v3_loads() -> bool:
    print("=== 1: V3 planner-safe artifact loads ===")
    profile = load_aircraft_profile(V3_PATH)
    ok = (
        profile.manifest.aircraft_id == "c172p"
        and profile.manifest.schema_version == 3
        and profile.manifest.planner_ready is True
        and len(profile.manifest.altitude_grid_m) == 12
    )
    print(f"  aircraft_id={profile.manifest.aircraft_id} schema_version={profile.manifest.schema_version} "
          f"planner_ready={profile.manifest.planner_ready} grid_len={len(profile.manifest.altitude_grid_m)}  "
          f"{'PASS' if ok else 'FAIL'}")
    return ok


def validation_2_raw_rejected() -> bool:
    print()
    print("=== 2: every RAW artifact is rejected (LutNotPlannerSafeError) ===")
    results = []
    for path in RAW_PATHS:
        results.append(expect_raises(f"load({path})", LutNotPlannerSafeError, lambda p=path: load_aircraft_profile(p)))
    return all(results)


def validation_3_turn_exact_anchors() -> bool:
    print()
    print("=== 3: turn exact-anchor tests ===")
    p = load_aircraft_profile(V3_PATH)
    results = []

    for alt, direction in [(0.0, "LEFT"), (0.0, "RIGHT"), (3000.0, "LEFT"), (3000.0, "RIGHT"),
                            (5500.0, "LEFT"), (5500.0, "RIGHT")]:
        r = p.turn_query(alt, direction, 20.0)
        ok = r.availability == "AVAILABLE" and "turn_radius_m" in r.planner_safe and "turn_rate_deg_s" in r.planner_safe
        results.append(ok)
        print(f"  turn_query({alt},{direction},20deg) -> {r.availability} radius={r.planner_safe.get('turn_radius_m')} "
              f"rate={r.planner_safe.get('turn_rate_deg_s')}  {'PASS' if ok else 'FAIL'}")

    for alt, direction in [(0.0, "LEFT"), (0.0, "RIGHT"), (5500.0, "LEFT"), (5500.0, "RIGHT")]:
        r = p.turn_query(alt, direction, 30.0)
        ok = r.availability == "UNAVAILABLE"
        results.append(ok)
        print(f"  turn_query({alt},{direction},30deg) [measured evidence exists, must still be UNAVAILABLE] "
              f"-> {r.availability}  {'PASS' if ok else 'FAIL'}")

    # LEFT/RIGHT must never be averaged -- their signed command_bank_deg differs, so a
    # structural check: querying LEFT and RIGHT separately must not require identical results.
    left = p.turn_query(0.0, "LEFT", 20.0)
    right = p.turn_query(0.0, "RIGHT", 20.0)
    direction_separated = left.planner_safe.get("turn_rate_deg_s") != right.planner_safe.get("turn_rate_deg_s")
    results.append(direction_separated)
    print(f"  LEFT/RIGHT kept separate (not averaged): rate_L={left.planner_safe.get('turn_rate_deg_s')} "
          f"rate_R={right.planner_safe.get('turn_rate_deg_s')}  {'PASS' if direction_separated else 'FAIL'}")

    results.append(expect_raises("turn_query direction='UP' (invalid)", AircraftProfileQueryError,
                                  lambda: p.turn_query(0.0, "UP", 20.0)))
    results.append(expect_raises("turn_query bank_deg=25 (never characterized)", AircraftProfileQueryError,
                                  lambda: p.turn_query(0.0, "LEFT", 25.0)))

    return all(results)


def validation_4_climb_altitude_dependent() -> bool:
    print()
    print("=== 4: climb exact-anchor tests (altitude-dependent family) ===")
    p = load_aircraft_profile(V3_PATH)
    results = []
    expected_family = {
        0.0: 4.0, 1500.0: 4.0, 2000.0: 3.0, 3000.0: 3.0, 3500.0: 2.0, 4500.0: 2.0,
    }
    for alt, expected_cmd in expected_family.items():
        r = p.vertical_query(alt, "CLIMB")
        got = r.planner_safe.get("highest_tested_safe_climb_command_mps")
        ok = r.availability == "AVAILABLE" and got == expected_cmd
        results.append(ok)
        print(f"  vertical_query({alt},CLIMB) -> {r.availability} command={got} (expect {expected_cmd})  "
              f"{'PASS' if ok else 'FAIL'}")

    for alt in (5000.0, 5500.0):
        r = p.vertical_query(alt, "CLIMB")
        ok = r.availability == "UNAVAILABLE" and not r.planner_safe
        # the critical measured-vs-planner-safe separation case (project.md "Step ALG-1"):
        has_measured_but_unsafe = r.measured is not None and r.measured.get("maximum_observed_stable_climb_vz_mps") is not None
        results.append(ok and has_measured_but_unsafe)
        print(f"  vertical_query({alt},CLIMB) -> {r.availability} planner_safe={r.planner_safe} "
              f"measured_climb={r.measured.get('maximum_observed_stable_climb_vz_mps') if r.measured else None} "
              f"(measured non-null but NOT used as safe)  {'PASS' if ok and has_measured_but_unsafe else 'FAIL'}")

    return all(results)


def validation_5_descent_altitude_dependent() -> bool:
    print()
    print("=== 5: descent exact-anchor tests (altitude-dependent, NON-monotonic family) ===")
    p = load_aircraft_profile(V3_PATH)
    results = []
    expected_family = {0.0: -3.0, 2500.0: -3.0, 3000.0: -4.0, 5000.0: -4.0, 5500.0: -3.0}
    for alt, expected_cmd in expected_family.items():
        r = p.vertical_query(alt, "DESCENT")
        got = r.planner_safe.get("highest_tested_safe_descent_command_mps")
        ok = r.availability == "AVAILABLE" and got == expected_cmd
        results.append(ok)
        print(f"  vertical_query({alt},DESCENT) -> {r.availability} command={got} (expect {expected_cmd})  "
              f"{'PASS' if ok else 'FAIL'}")
    return all(results)


def validation_6_combined() -> bool:
    print()
    print("=== 6: combined-maneuver exact-anchor tests ===")
    p = load_aircraft_profile(V3_PATH)
    results = []

    for alt in (0.0, 500.0, 1000.0, 1500.0, 2000.0, 2500.0, 3000.0, 3500.0, 4000.0, 4500.0, 5000.0, 5500.0):
        for direction in ("LEFT", "RIGHT"):
            r = p.combined_query(alt, direction, "CLIMB")
            ok = r.availability == "UNAVAILABLE"
            results.append(ok)
            if not ok:
                print(f"  combined_query({alt},{direction},CLIMB) -> {r.availability}  FAIL (must be UNAVAILABLE everywhere)")
    print(f"  climbing turn UNAVAILABLE across full domain (24 checks): {'PASS' if all(results) else 'FAIL'}")

    descent_expect = {3000.0: "UNAVAILABLE", 3500.0: "AVAILABLE", 5500.0: "AVAILABLE"}
    descent_results = []
    for alt, expected in descent_expect.items():
        for direction in ("LEFT", "RIGHT"):
            r = p.combined_query(alt, direction, "DESCENT")
            ok = r.availability == expected
            descent_results.append(ok)
            print(f"  combined_query({alt},{direction},DESCENT) -> {r.availability} (expect {expected})  "
                  f"{'PASS' if ok else 'FAIL'}")
    results.extend(descent_results)
    return all(results)


def validation_7_straight() -> bool:
    print()
    print("=== 7: straight query ===")
    p = load_aircraft_profile(V3_PATH)
    r = p.straight_query(2500.0)
    ok = r.availability == "AVAILABLE" and "expected_ias_mps" in r.planner_safe
    print(f"  straight_query(2500) -> {r.availability} ias={r.planner_safe.get('expected_ias_mps')}  "
          f"{'PASS' if ok else 'FAIL'}")
    return ok


def validation_8_intermediate() -> bool:
    print()
    print("=== 8: intermediate-altitude queries ===")
    p = load_aircraft_profile(V3_PATH)
    results = []

    # LINEAR family (level_turn): intermediate value must lie between the two bracketing rows.
    r = p.turn_query(250.0, "LEFT", 20.0)
    r_lo = p.turn_query(0.0, "LEFT", 20.0)
    r_hi = p.turn_query(500.0, "LEFT", 20.0)
    lo_rad, hi_rad = r_lo.planner_safe["turn_radius_m"], r_hi.planner_safe["turn_radius_m"]
    ok = (r.altitude_resolution == "LINEAR_INTERPOLATED" and r.availability == "AVAILABLE"
          and min(lo_rad, hi_rad) <= r.planner_safe["turn_radius_m"] <= max(lo_rad, hi_rad))
    results.append(ok)
    print(f"  turn_query(250,LEFT,20) [LINEAR, between 0/500] -> {r.altitude_resolution} "
          f"radius={r.planner_safe.get('turn_radius_m'):.2f} (bracket [{lo_rad:.2f},{hi_rad:.2f}])  "
          f"{'PASS' if ok else 'FAIL'}")

    # CONSERVATIVE_ENDPOINT family (straight_climb): must exactly equal the conservative bracket's own row.
    for alt, note in [(1750.0, "1500=+4/2000=+3 -> expect +3"), (2750.0, "both +3, trivial"),
                       (4750.0, "4500=+2/5000=UNAVAILABLE -> whole interval UNAVAILABLE")]:
        r = p.vertical_query(alt, "CLIMB")
        print(f"  vertical_query({alt},CLIMB) [{note}] -> {r.availability} {r.altitude_resolution} "
              f"command={r.planner_safe.get('highest_tested_safe_climb_command_mps')}")

    r_1750 = p.vertical_query(1750.0, "CLIMB")
    r_2000 = p.vertical_query(2000.0, "CLIMB")
    ok_1750 = (r_1750.availability == "AVAILABLE"
               and r_1750.planner_safe.get("highest_tested_safe_climb_command_mps") == 3.0
               and r_1750.planner_safe == r_2000.planner_safe)
    results.append(ok_1750)
    print(f"  1750 CLIMB conservative choice matches 2000's row (not 1500's +4): {'PASS' if ok_1750 else 'FAIL'}")

    r_4750 = p.vertical_query(4750.0, "CLIMB")
    ok_4750 = r_4750.availability == "UNAVAILABLE"
    results.append(ok_4750)
    print(f"  4750 CLIMB (between AVAILABLE 4500 and UNAVAILABLE 5000) -> {r_4750.availability}  "
          f"{'PASS' if ok_4750 else 'FAIL'}")

    r_3250 = p.vertical_query(3250.0, "CLIMB")
    ok_3250 = r_3250.availability == "AVAILABLE" and r_3250.planner_safe.get("highest_tested_safe_climb_command_mps") == 2.0
    results.append(ok_3250)
    print(f"  3250 CLIMB (3000=+3/3500=+2 -> expect conservative +2) -> {r_3250.availability} "
          f"command={r_3250.planner_safe.get('highest_tested_safe_climb_command_mps')}  {'PASS' if ok_3250 else 'FAIL'}")

    r_4250 = p.vertical_query(4250.0, "CLIMB")
    ok_4250 = r_4250.availability == "UNAVAILABLE" and r_4250.altitude_resolution == "INTERMEDIATE_BLOCKED"
    results.append(ok_4250)
    print(f"  4250 CLIMB (inside blocked range [4000,5500]) -> {r_4250.availability} {r_4250.altitude_resolution}  "
          f"{'PASS' if ok_4250 else 'FAIL'}")

    r_5250 = p.vertical_query(5250.0, "CLIMB")
    ok_5250 = r_5250.availability == "UNAVAILABLE"
    results.append(ok_5250)
    print(f"  5250 CLIMB (inside blocked range, both 5000/5500 UNAVAILABLE anyway) -> {r_5250.availability}  "
          f"{'PASS' if ok_5250 else 'FAIL'}")

    return all(results)


def validation_9_domain() -> bool:
    print()
    print("=== 9: domain / out-of-domain tests ===")
    p = load_aircraft_profile(V3_PATH)
    results = []
    for alt in (-1.0, 5501.0, 6000.0):
        r = p.straight_query(alt)
        ok = r.availability == "OUT_OF_DOMAIN"
        results.append(ok)
        print(f"  straight_query({alt}) -> {r.availability}  {'PASS' if ok else 'FAIL'}")
    r = p.straight_query(5500.0)
    ok = r.availability == "AVAILABLE" and r.altitude_resolution == "EXACT"
    results.append(ok)
    print(f"  straight_query(5500.0) [exact top-of-domain anchor] -> {r.availability} {r.altitude_resolution}  "
          f"{'PASS' if ok else 'FAIL'}")
    return all(results)


def make_synthetic_second_aircraft_fixture() -> dict:
    """A small, clearly-synthetic V3-schema profile for a fictional
    "test_aircraft" -- different identity, different (smaller) altitude
    domain, different (round, obviously-fake) capability numbers. Proves
    the SAME loader/query code works for an aircraft that is NOT c172p,
    with zero aircraft-specific branching anywhere in planner/
    aircraft_profile.py."""
    units = {"altitude": "m", "angle": "deg", "radius": "m", "speed": "m/s", "turn_rate": "deg/s"}

    def turn_row(alt, direction, bank, available):
        return {
            "altitude_m": alt, "direction": direction, "command_bank_deg": bank if direction == "RIGHT" else -bank,
            "availability": "AVAILABLE" if available else "UNAVAILABLE",
            "evidence": {"reason_unavailable": None if available else "synthetic_test_boundary"},
            "measured": {"note": "synthetic"},
            "planner_safe": {"turn_radius_m": 111.0 + alt, "turn_rate_deg_s": 9.0 if direction == "RIGHT" else -9.0} if available else None,
        }

    def vertical_row(alt, mode, cmd, available):
        key = "climb" if mode == "CLIMB" else "descent"
        return {
            "altitude_m": alt, "availability": "AVAILABLE" if available else "UNAVAILABLE",
            "evidence": {"limiting_reason": None if available else "synthetic_test_boundary"},
            "measured": {"note": "synthetic"},
            "planner_safe": {f"highest_tested_safe_{key}_command_mps": cmd, f"{key}_vz_mps": cmd} if available else None,
        }

    def combined_row(alt, direction, mode, available):
        return {
            "altitude_m": alt, "direction": direction,
            "availability": "AVAILABLE" if available else "UNAVAILABLE",
            "evidence": {"reason_unavailable": None if available else "synthetic_test_boundary"},
            "measured": {"note": "synthetic"},
            "planner_safe": {"turn_radius_m": 200.0, "turn_rate_deg_s": 5.0} if available else None,
        }

    alts = [0.0, 1000.0, 2000.0]
    return {
        "schema_version": 3,
        "profile_schema_id": "tested_safe_envelope_aircraft_capability_profile_v3",
        "profile_stage": "tested_planner_safe_envelope",
        "planner_ready": True,
        "aircraft_identity": {"aircraft_id": "test_aircraft", "controller_stack_id": "test-stack-v1"},
        "provenance_id": "synthetic-test-0001",
        "units": units,
        "domain": {
            "canonical_altitude_grid_m": alts, "min_altitude_m": 0.0, "max_altitude_m": 2000.0,
            "nominal_ias_context_mps": 25.0,
        },
        "capabilities": {
            "straight": {"query_policy": {"safe_interpolation": "LINEAR"}, "rows": [
                {"altitude_m": a, "availability": "AVAILABLE", "evidence": {}, "measured": {"note": "synthetic"},
                 "planner_safe": {"expected_ias_mps": 25.0}} for a in alts
            ]},
            "level_turn": {"query_policy": {"safe_interpolation": "LINEAR"}, "rows": [
                turn_row(a, d, 15.0, True) for a in alts for d in ("LEFT", "RIGHT")
            ]},
            "straight_climb": {"query_policy": {"safe_interpolation": "CONSERVATIVE_ENDPOINT",
                                                 "availability_interpolation_blocked_ranges_m": []},
                                "rows": [vertical_row(a, "CLIMB", c, True) for a, c in zip(alts, [3.0, 2.0, 1.0])]},
            "straight_descent": {"query_policy": {"safe_interpolation": "CONSERVATIVE_ENDPOINT",
                                                    "availability_interpolation_blocked_ranges_m": []},
                                  "rows": [vertical_row(a, "DESCENT", c, True) for a, c in zip(alts, [-2.0, -2.0, -3.0])]},
            "climbing_turn": {"query_policy": {"safe_interpolation": "NONE"}, "rows": [
                combined_row(a, d, "CLIMB", False) for a in alts for d in ("LEFT", "RIGHT")
            ]},
            "descending_turn": {"query_policy": {"safe_interpolation": "LINEAR"}, "rows": [
                combined_row(a, d, "DESCENT", True) for a in alts for d in ("LEFT", "RIGHT")
            ]},
        },
    }


def validation_10_second_aircraft_swappability() -> bool:
    print()
    print("=== 10: generic second-aircraft synthetic fixture (aircraft swappability) ===")
    data = make_synthetic_second_aircraft_fixture()
    profile = load_aircraft_profile_from_dict(
        data, expected_aircraft_id="test_aircraft", expected_controller_stack_id="test-stack-v1"
    )
    results = []

    ok = profile.manifest.aircraft_id == "test_aircraft" and profile.manifest.max_altitude_m == 2000.0
    results.append(ok)
    print(f"  loaded synthetic profile: aircraft_id={profile.manifest.aircraft_id} "
          f"domain=[{profile.manifest.min_altitude_m},{profile.manifest.max_altitude_m}]  {'PASS' if ok else 'FAIL'}")

    r = profile.turn_query(1000.0, "LEFT", 15.0)
    ok = r.availability == "AVAILABLE" and r.planner_safe.get("turn_radius_m") == 1111.0
    results.append(ok)
    print(f"  turn_query(1000,LEFT,15) -> {r.availability} radius={r.planner_safe.get('turn_radius_m')}  "
          f"{'PASS' if ok else 'FAIL'}")

    r = profile.vertical_query(2500.0, "CLIMB")
    ok = r.availability == "OUT_OF_DOMAIN"
    results.append(ok)
    print(f"  vertical_query(2500,CLIMB) [outside 0-2000 synthetic domain] -> {r.availability}  "
          f"{'PASS' if ok else 'FAIL'}")

    # correct wrong-identity rejection using the SAME loader, no c172p-specific code involved
    ok = expect_raises(
        "load synthetic profile expecting c172p (wrong identity)", Exception,
        lambda: load_aircraft_profile_from_dict(data, expected_aircraft_id="c172p",
                                                 expected_controller_stack_id="anything"),
    )
    results.append(ok)

    return all(results)


def main() -> None:
    results = [
        validation_1_v3_loads(),
        validation_2_raw_rejected(),
        validation_3_turn_exact_anchors(),
        validation_4_climb_altitude_dependent(),
        validation_5_descent_altitude_dependent(),
        validation_6_combined(),
        validation_7_straight(),
        validation_8_intermediate(),
        validation_9_domain(),
        validation_10_second_aircraft_swappability(),
    ]
    print()
    print(f"Overall: {'ALL PASS' if all(results) else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
