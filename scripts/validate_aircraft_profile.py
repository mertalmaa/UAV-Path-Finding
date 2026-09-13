"""Step ALG-0: interface/schema tests for planner.aircraft_profile.

Pure interface testing -- NO real aircraft physics values. The fixture
built below is explicitly synthetic (round, clearly-fake numbers); it
exists only to exercise the schema/provenance/query CONTRACT, never to
stand in for real c172r capability. When a real planner-safe LUT exists
(post U4.1/U4.2), it must satisfy the same contract these tests check.

No search code is touched or imported here. This does not change planner
behavior in any way.
"""
from planner.aircraft_profile import (
    AircraftProfileQueryError, EXPECTED_AIRCRAFT_ID, EXPECTED_CONTROLLER_STACK_ID,
    LutNotPlannerSafeError, LutProvenanceError, LutSchemaError, PLANNER_SAFE_LUT_STAGE,
    RAW_LUT_STAGE, SUPPORTED_SCHEMA_VERSION, load_aircraft_profile_from_dict,
)


def make_fixture() -> dict:
    """A tiny, clearly-synthetic planner-safe LUT: 2 altitudes, 1 speed
    context, 3 bank values (one STRAIGHT, one LEFT, one RIGHT), 3 vertical
    speeds. Numbers are round placeholders (radius=999, rate=9, vz=9,
    gamma=9) -- deliberately not plausible c172r physics, so nobody
    mistakes this fixture for real characterization data."""
    return {
        "schema_version": SUPPORTED_SCHEMA_VERSION,
        "lut_stage": PLANNER_SAFE_LUT_STAGE,
        "aircraft_id": EXPECTED_AIRCRAFT_ID,
        "controller_stack_id": EXPECTED_CONTROLLER_STACK_ID,
        "provenance_id": "synthetic-fixture-0001",
        "units": {"altitude": "m", "speed": "m/s", "angle": "deg", "time": "s"},
        "speed_contexts": ["cruise_test"],
        "altitude_grid_m": [0.0, 500.0],
        "turn_bank_grid_deg": [0.0, -10.0, 10.0],
        "turn_directions": ["STRAIGHT", "LEFT", "RIGHT"],
        "vertical_speed_grid_mps": [-9.0, 0.0, 9.0],
        "turn_table": [
            {"altitude_m": 0.0, "speed_context": "cruise_test", "bank_deg": 0.0, "direction": "STRAIGHT",
             "status": "VALID", "turn_radius_m": None, "turn_rate_deg_s": 0.0, "actual_bank_deg": 0.0},
            {"altitude_m": 0.0, "speed_context": "cruise_test", "bank_deg": -10.0, "direction": "LEFT",
             "status": "VALID", "turn_radius_m": 999.0, "turn_rate_deg_s": -9.0, "actual_bank_deg": -9.9},
            {"altitude_m": 0.0, "speed_context": "cruise_test", "bank_deg": 10.0, "direction": "RIGHT",
             "status": "UNKNOWN", "turn_radius_m": None, "turn_rate_deg_s": None, "actual_bank_deg": 9.9},
            {"altitude_m": 500.0, "speed_context": "cruise_test", "bank_deg": 10.0, "direction": "RIGHT",
             "status": "INFEASIBLE", "turn_radius_m": None, "turn_rate_deg_s": None, "actual_bank_deg": None},
        ],
        "vertical_table": [
            {"altitude_m": 0.0, "speed_context": "cruise_test", "desired_vz_mps": 0.0,
             "status": "VALID", "achievable_vz_mps": 0.0, "gamma_deg": 0.0},
            {"altitude_m": 0.0, "speed_context": "cruise_test", "desired_vz_mps": 9.0,
             "status": "INFEASIBLE", "achievable_vz_mps": 5.0, "gamma_deg": 4.0},
        ],
    }


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


def validation_1_valid_exact_query() -> bool:
    print("=== 1: valid exact query (turn + vertical) ===")
    profile = load_aircraft_profile_from_dict(make_fixture())

    t = profile.turn_query(0.0, "cruise_test", -10.0, "LEFT")
    ok_t = t.status == "VALID" and t.turn_radius_m == 999.0 and t.turn_rate_deg_s == -9.0
    print(f"  turn_query(0.0, LEFT, -10deg) -> status={t.status} radius={t.turn_radius_m} "
          f"rate={t.turn_rate_deg_s}  {'PASS' if ok_t else 'FAIL'}")

    v = profile.vertical_query(0.0, "cruise_test", 0.0)
    ok_v = v.status == "VALID" and v.achievable_vz_mps == 0.0
    print(f"  vertical_query(0.0, vz=0.0) -> status={v.status} achievable_vz={v.achievable_vz_mps}  "
          f"{'PASS' if ok_v else 'FAIL'}")

    # UNKNOWN must come through as UNKNOWN, not silently upgraded/downgraded.
    u = profile.turn_query(0.0, "cruise_test", 10.0, "RIGHT")
    ok_u = u.status == "UNKNOWN"
    print(f"  turn_query(0.0, RIGHT, 10deg) [UNKNOWN entry] -> status={u.status}  "
          f"{'PASS' if ok_u else 'FAIL'}")

    return ok_t and ok_v and ok_u


def validation_2_off_grid_altitude() -> bool:
    print()
    print("=== 2: off-grid altitude (no interpolation) ===")
    profile = load_aircraft_profile_from_dict(make_fixture())
    return expect_raises(
        "turn_query(altitude_m=250.0) [between 0 and 500, off-grid]",
        AircraftProfileQueryError,
        lambda: profile.turn_query(250.0, "cruise_test", 0.0, "STRAIGHT"),
    )


def validation_3_invalid_status() -> bool:
    print()
    print("=== 3: invalid status value rejected at load time ===")
    data = make_fixture()
    data["turn_table"][0]["status"] = "OK"  # not one of VALID/INFEASIBLE/UNKNOWN
    return expect_raises("load with status='OK'", LutSchemaError, lambda: load_aircraft_profile_from_dict(data))


def validation_4_wrong_aircraft_id() -> bool:
    print()
    print("=== 4: wrong aircraft_id rejected ===")
    data = make_fixture()
    data["aircraft_id"] = "some_other_aircraft"
    return expect_raises("load with wrong aircraft_id", LutProvenanceError, lambda: load_aircraft_profile_from_dict(data))


def validation_5_wrong_controller_stack() -> bool:
    print()
    print("=== 5: wrong controller_stack_id rejected ===")
    data = make_fixture()
    data["controller_stack_id"] = "some-other-controller-stack"
    return expect_raises(
        "load with wrong controller_stack_id", LutProvenanceError, lambda: load_aircraft_profile_from_dict(data)
    )


def validation_6_wrong_schema_version() -> bool:
    print()
    print("=== 6: wrong schema_version rejected ===")
    data = make_fixture()
    data["schema_version"] = SUPPORTED_SCHEMA_VERSION + 1
    return expect_raises("load with schema_version+1", LutSchemaError, lambda: load_aircraft_profile_from_dict(data))


def validation_7_raw_lut_refused() -> bool:
    print()
    print("=== 7: RAW LUT refused outright (never usable by the planner) ===")
    data = make_fixture()
    data["lut_stage"] = RAW_LUT_STAGE
    return expect_raises(
        "load with lut_stage='raw'", LutNotPlannerSafeError, lambda: load_aircraft_profile_from_dict(data)
    )


def validation_8_unit_schema_validation() -> bool:
    print()
    print("=== 8: unit + schema validation ===")
    results = []

    data = make_fixture()
    data["units"]["altitude"] = "ft"  # not SI
    results.append(expect_raises("load with altitude unit='ft'", LutSchemaError, lambda: load_aircraft_profile_from_dict(data)))

    data = make_fixture()
    data["units"]["angle"] = "rad"  # not the declared standard (deg)
    results.append(expect_raises("load with angle unit='rad'", LutSchemaError, lambda: load_aircraft_profile_from_dict(data)))

    data = make_fixture()
    del data["provenance_id"]
    results.append(expect_raises("load with missing provenance_id", LutSchemaError, lambda: load_aircraft_profile_from_dict(data)))

    data = make_fixture()
    del data["altitude_grid_m"]
    results.append(expect_raises("load with missing altitude_grid_m", LutSchemaError, lambda: load_aircraft_profile_from_dict(data)))

    data = make_fixture()
    data["turn_table"][0]["speed_context"] = "not_a_declared_context"
    results.append(expect_raises(
        "load with undeclared speed_context in turn_table row", LutSchemaError,
        lambda: load_aircraft_profile_from_dict(data),
    ))

    return all(results)


def validation_9_uncharacterized_combination() -> bool:
    print()
    print("=== 9: on-grid values but never-characterized combination ===")
    profile = load_aircraft_profile_from_dict(make_fixture())
    # bank=-10 exists, but only under direction=LEFT in the fixture -- RIGHT was never given -10.
    return expect_raises(
        "turn_query(bank=-10, direction=RIGHT) [combination absent from table]",
        AircraftProfileQueryError,
        lambda: profile.turn_query(0.0, "cruise_test", -10.0, "RIGHT"),
    )


def main() -> None:
    results = [
        validation_1_valid_exact_query(),
        validation_2_off_grid_altitude(),
        validation_3_invalid_status(),
        validation_4_wrong_aircraft_id(),
        validation_5_wrong_controller_stack(),
        validation_6_wrong_schema_version(),
        validation_7_raw_lut_refused(),
        validation_8_unit_schema_validation(),
        validation_9_uncharacterized_combination(),
    ]
    print()
    print(f"Overall: {'ALL PASS' if all(results) else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
