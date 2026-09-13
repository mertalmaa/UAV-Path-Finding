from __future__ import annotations

import hashlib
import json
import math
import sys
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
RESULTS = HERE / "results"
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT))

from planner.aircraft_profile import AircraftProfileQueryError, load_aircraft_profile
from planner_safe_measurements import radius_from_arc_and_course_change, shortest_angle_deg, unwrap_degrees


def _load(name: str):
    return json.loads((RESULTS / name).read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    config = yaml.safe_load(
        (HERE / "u6_2_planner_safe_profile_configuration.yaml").read_text(
            encoding="utf-8"
        )
    )
    profile = _load("c172p_aircraft_profile_planner_safe.json")
    derivation = _load("u6_2_derivation_audit.json")
    measurement = _load("u6_2_measurement_fix_validation.json")
    provenance = _load("u6_2_provenance.json")
    result = _load("u6_2_result.json")
    core = _load("c172p_core_aircraft_lut_raw.json")
    pid = result["provenance_id"]

    assert config["frozen_context"]["aircraft_id"] == "c172p"
    assert config["frozen_context"]["nominal_ias_mps"] == 40
    assert config["frozen_context"]["min_altitude_m"] == 0
    assert config["frozen_context"]["max_altitude_m"] == 5500
    assert config["derivation_policy"]["arbitrary_fixed_safety_factor"] is False
    assert config["derivation_policy"]["raw_unknown_silent_promotion"] is False
    assert config["derivation_policy"]["full_throttle_normal_primitive"] is False
    assert config["capability_policy"]["level_turn_left_right_averaged"] is False
    assert all(value is False for value in config["scope"].values())

    assert all(artifact["provenance_id"] == pid for artifact in (profile, derivation, measurement, provenance))
    assert profile["artifact_type"] == "C172P_AIRCRAFT_PROFILE_PLANNER_SAFE"
    assert profile["profile_schema_id"] == "aircraft_profile_planner_safe_v1"
    assert profile["lut_stage"] == profile["profile_stage"] == "planner_safe"
    assert profile["planner_ready"] is True
    assert profile["aircraft_id"] == "c172p"
    assert profile["nominal_ias_mps"] == 40.0
    assert profile["min_altitude_m"] == 0.0
    assert profile["max_altitude_m"] == 5500.0
    assert profile["altitude_grid_m"] == [float(value) for value in range(0, 5501, 500)]
    assert max(profile["altitude_grid_m"]) == 5500.0
    assert profile["query_semantics"]["availability"] == ["AVAILABLE", "UNAVAILABLE", "OUT_OF_DOMAIN"]
    assert profile["query_semantics"]["raw_unknown_silently_available"] is False
    assert profile["interface_contract"]["aircraft_specific_branching_forbidden"] is True
    assert profile["production_search_integrated"] is False
    assert profile["directly_infeasible_implies_unreachable"] is False
    assert set(profile["capabilities"]) == {
        "straight", "level_turn", "straight_climb", "straight_descent",
        "climbing_turn", "descending_turn",
    }

    turn_rows = profile["capabilities"]["level_turn"]["rows"]
    primary_turns = [row for row in turn_rows if abs(row["bank_deg"]) == 20.0]
    optional_turns = [row for row in turn_rows if abs(row["bank_deg"]) == 30.0]
    assert len(primary_turns) == len(optional_turns) == 24
    assert all(row["availability"] == "AVAILABLE" for row in primary_turns)
    assert all(row["availability"] == "UNAVAILABLE" for row in optional_turns)
    assert all(row["safe_turn_radius_m"] >= row["metadata"]["raw_reference_radius_m"] for row in primary_turns)
    assert all(row["metadata"]["uncertainty_allowance_m"] > 0.0 for row in primary_turns)
    assert {row["direction"] for row in primary_turns} == {"LEFT", "RIGHT"}
    for altitude in profile["altitude_grid_m"]:
        left = next(row for row in primary_turns if row["altitude_m"] == altitude and row["direction"] == "LEFT")
        right = next(row for row in primary_turns if row["altitude_m"] == altitude and row["direction"] == "RIGHT")
        assert not math.isclose(left["safe_turn_radius_m"], right["safe_turn_radius_m"])

    climb = profile["capabilities"]["straight_climb"]["rows"]
    assert all(row["availability"] == "AVAILABLE" and row["safe_command_vz_mps"] == 2.0 for row in climb if row["altitude_m"] <= 4500.0)
    assert all(row["availability"] == "UNAVAILABLE" and row["safe_command_vz_mps"] is None for row in climb if row["altitude_m"] >= 5000.0)
    for row in climb:
        raw = next(item for item in core["straight_vertical_table"] if item["altitude_m"] == row["altitude_m"] and item["target_vz_mps"] == 2.0)
        if row["availability"] == "AVAILABLE":
            assert row["safe_achievable_vz_mps"] <= raw["actual_vz_mps"]
        if raw["status"] == "UNKNOWN" and row["availability"] == "AVAILABLE":
            assert row["metadata"]["reviewed_unknown_to_available"] is True
            assert row["metadata"]["review_basis"]

    descent = profile["capabilities"]["straight_descent"]["rows"]
    assert len(descent) == 12
    assert all(row["availability"] == "AVAILABLE" and row["safe_command_vz_mps"] == -3.0 for row in descent)
    for row in descent:
        raw = next(item for item in core["straight_vertical_table"] if item["altitude_m"] == row["altitude_m"] and item["target_vz_mps"] == -3.0)
        assert abs(row["safe_achievable_vz_mps"]) <= abs(raw["actual_vz_mps"])
        assert row["metadata"]["reviewed_unknown_to_available"] is True
        assert row["metadata"]["review_basis"]

    combined_climb = profile["capabilities"]["climbing_turn"]["rows"]
    assert len(combined_climb) == 24
    assert all(row["availability"] == "UNAVAILABLE" for row in combined_climb)
    assert all(row["safe_turn_radius_m"] is None and row["safe_vz_mps"] is None for row in combined_climb)
    combined_descent = profile["capabilities"]["descending_turn"]["rows"]
    assert len(combined_descent) == 24
    assert sum(row["availability"] == "AVAILABLE" for row in combined_descent) == result["descending_turn_available_rows"] == 10
    assert all(row["availability"] == "UNAVAILABLE" for row in combined_descent if row["altitude_m"] <= 3000.0)
    assert all(row["availability"] == "AVAILABLE" for row in combined_descent if row["altitude_m"] >= 3500.0)
    assert all(row["safe_turn_radius_m"] >= row["metadata"]["raw_or_interpolated_reference_radius_m"] for row in combined_descent if row["availability"] == "AVAILABLE")

    assert shortest_angle_deg(247.27702298135128) == measurement["regression_cases"][0]["output_deg"]
    assert shortest_angle_deg(-190.0) == 170.0
    assert shortest_angle_deg(180.0) == -180.0
    assert unwrap_degrees([350.0, 355.0, 1.0, 7.0]) == [350.0, 355.0, 361.0, 367.0]
    corrected, wrapped = radius_from_arc_and_course_change(840.7850165048358, 247.27702298135128)
    assert math.isclose(corrected, 427.36125497816295, abs_tol=1.0e-9)
    assert math.isclose(wrapped, -112.72297701864875, abs_tol=1.0e-12)
    assert measurement["passed"] is True
    assert measurement["u6b_outlier_case"]["source_raw_modified"] is False
    assert profile["measurement_policy"]["historical_raw_modified"] is False

    loaded = load_aircraft_profile(
        str(RESULTS / "c172p_aircraft_profile_planner_safe.json"),
        expected_aircraft_id="c172p",
        expected_controller_stack_id=config["frozen_context"]["controller_stack_id"],
    )
    assert loaded.turn_query(0.0, "NOMINAL_40_MPS_IAS", -20.0, "LEFT").status == "VALID"
    assert loaded.turn_query(0.0, "NOMINAL_40_MPS_IAS", -30.0, "LEFT").status == "INFEASIBLE"
    assert loaded.vertical_query(4500.0, "NOMINAL_40_MPS_IAS", 2.0).status == "VALID"
    assert loaded.vertical_query(5000.0, "NOMINAL_40_MPS_IAS", 2.0).status == "INFEASIBLE"
    try:
        loaded.turn_query(6000.0, "NOMINAL_40_MPS_IAS", -20.0, "LEFT")
    except AircraftProfileQueryError:
        pass
    else:
        raise AssertionError(">5500 query did not return the interface's OUT_OF_DOMAIN error path")

    assert derivation["no_arbitrary_safety_factor"] is True
    assert derivation["no_new_jsbsim_runs"] is True
    assert derivation["combined_climb_policy"]["policy"] == "DISABLED_FIRST_PLANNER_SAFE_PROFILE"
    assert provenance["source_artifacts_unchanged"] is True
    assert provenance["source_hashes_before"] == provenance["source_hashes_after"]
    assert _sha256(Path(provenance["files"]["configuration"]["path"])) == provenance["files"]["configuration"]["sha256"]
    assert _sha256(Path(provenance["files"]["derivation_harness"]["path"])) == provenance["files"]["derivation_harness"]["sha256"]
    assert _sha256(Path(provenance["files"]["measurement_helper"]["path"])) == provenance["files"]["measurement_helper"]["sha256"]
    for name, source in config["source_artifacts"].items():
        assert _sha256(HERE / source["path"]) == source["sha256"]
        assert provenance["source_hashes_after"][name] == source["sha256"]

    assert result["step_status"] == "PASS"
    assert result["planner_safe_profile_ready"] is True
    assert result["measurement_pipeline_fixed"] is True
    assert result["aircraft_swappable_profile"] is True
    assert result["ready_for_aircraftprofile_integration"] is True
    assert result["new_jsbsim_runs"] == 0
    assert result["combined_climb_available_rows"] == 0
    assert result["source_artifacts_unchanged"] is True
    for forbidden in (
        "planner_or_search_integrated", "heading_or_primitives_implemented",
        "grid_regate_performed", "next_stage_started",
    ):
        assert result[forbidden] is False

    print("U6.2 planner-safe c172p profile and measurement regression checks passed")


if __name__ == "__main__":
    main()
