from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
sys.path.insert(0, str(HERE))

from tested_envelope_profile import TestedEnvelopeAircraftProfile


def _load(name: str):
    return json.loads((RESULTS / name).read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    config = yaml.safe_load((HERE / "u6_2_2_tested_safe_envelope_configuration.yaml").read_text(encoding="utf-8"))
    profile = _load("c172p_aircraft_profile_planner_safe_v3.json")
    audit = _load("u6_2_2_tested_safe_envelope_audit.json")
    preservation = _load("u6_2_2_non_vertical_preservation_audit.json")
    provenance = _load("u6_2_2_provenance.json")
    result = _load("u6_2_2_result.json")
    v2 = _load("c172p_aircraft_profile_planner_safe_v2.json")
    altitudes = [float(value) for value in range(0, 5501, 500)]

    assert profile["profile_schema_id"] == "tested_safe_envelope_aircraft_capability_profile_v3"
    assert profile["domain"]["canonical_altitude_grid_m"] == altitudes
    assert profile["global_aircraft_capability_constants"] == []
    assert profile["full_tested_vertical_command_grid_audited"] is True
    assert profile["vertical_safe_values_command_constant_independent"] is True
    assert profile["production_planner_integrated"] is False
    assert audit["audited_command_count"] == 96

    climb = profile["capabilities"]["straight_climb"]
    descent = profile["capabilities"]["straight_descent"]
    assert climb["tested_target_grid_mps"] == [2.0, 3.0, 4.0, 5.0]
    assert descent["tested_target_grid_mps"] == [-2.0, -3.0, -4.0, -5.0]
    assert len(climb["rows"]) == len(descent["rows"]) == 12
    assert all(len(row["measured"]["tested_command_audit"]) == 4 for row in climb["rows"] + descent["rows"])

    climb_commands = {
        row["planner_safe"]["highest_tested_safe_climb_command_mps"]
        for row in climb["rows"] if row["planner_safe"] is not None
    }
    descent_commands = {
        row["planner_safe"]["highest_tested_safe_descent_command_mps"]
        for row in descent["rows"] if row["planner_safe"] is not None
    }
    assert climb_commands == {2.0, 3.0, 4.0}
    assert descent_commands == {-3.0, -4.0}
    assert all(row["planner_safe"] is None and row["availability"] == "UNAVAILABLE" for row in climb["rows"] if row["altitude_m"] >= 5000.0)

    expected_climb = {0: 4, 500: 4, 1000: 4, 1500: 4, 2000: 3, 2500: 3, 3000: 3, 3500: 2, 4000: 2, 4500: 2}
    expected_descent = {0: -3, 500: -3, 1000: -3, 1500: -3, 2000: -3, 2500: -3, 3000: -4, 3500: -4, 4000: -4, 4500: -4, 5000: -4, 5500: -3}
    for row in climb["rows"]:
        if row["altitude_m"] in expected_climb:
            assert row["planner_safe"]["highest_tested_safe_climb_command_mps"] == expected_climb[row["altitude_m"]]
            assert row["planner_safe"]["climb_vz_mps"] <= row["planner_safe"]["selected_actual_vz_mps"]
    for row in descent["rows"]:
        assert row["planner_safe"]["highest_tested_safe_descent_command_mps"] == expected_descent[row["altitude_m"]]
        assert abs(row["planner_safe"]["descent_vz_mps"]) <= abs(row["planner_safe"]["selected_actual_vz_mps"])

    all_audits = [item for row in climb["rows"] + descent["rows"] for item in row["measured"]["tested_command_audit"]]
    assert all(not item["eligible_for_tested_safe_envelope"] for item in all_audits if item["strict_status"] == "INFEASIBLE")
    assert all(not item["eligible_for_tested_safe_envelope"] for item in all_audits if any(item["saturation"].values()))
    assert all(not item["eligible_for_tested_safe_envelope"] for item in all_audits if item["throttle"] >= 0.98)
    promoted = [item for item in all_audits if item["reviewed_unknown_promotion"]]
    assert promoted and all(item["reviewed_unknown_basis"] and item["strict_status"] == "UNKNOWN" for item in promoted)
    proxy = [item for item in all_audits if item["uncertainty_source"] == "NEAREST_VALIDATED_SEVERITY_U6_1_PROXY"]
    assert proxy and all(item["uncertainty_source_target_vz_mps"] in (-3.0, 3.0) for item in proxy)

    comparisons = {row["altitude_m"]: row for row in audit["comparison_rows"]}
    assert comparisons[0.0]["u6_2_2_safe_climb_vz_mps"] > comparisons[0.0]["u6_2_1_safe_climb_vz_mps"]
    assert comparisons[3000.0]["u6_2_2_safe_descent_vz_mps"] < comparisons[3000.0]["u6_2_1_safe_descent_vz_mps"]
    assert comparisons[3500.0]["u6_2_2_safe_climb_vz_mps"] == comparisons[3500.0]["u6_2_1_safe_climb_vz_mps"]
    assert comparisons[5500.0]["u6_2_2_safe_climb_vz_mps"] is None

    for family in ("straight", "level_turn", "climbing_turn", "descending_turn"):
        assert profile["capabilities"][family] == v2["capabilities"][family]
        assert preservation[f"{family}_preserved"] is True
    assert profile["measured_and_planner_safe_layers_separate"] is True
    assert profile["interface_contract"]["aircraft_specific_planner_branching_forbidden"] is True

    query = TestedEnvelopeAircraftProfile.load(RESULTS / "c172p_aircraft_profile_planner_safe_v3.json")
    assert query.vertical_query(1750.0, "CLIMB").availability == "AVAILABLE"
    assert query.vertical_query(1750.0, "CLIMB").row["planner_safe"]["highest_tested_safe_climb_command_mps"] == 3.0
    assert query.vertical_query(3250.0, "DESCENT").row["planner_safe"]["highest_tested_safe_descent_command_mps"] == -4.0
    assert query.vertical_query(4250.0, "CLIMB").availability == "UNAVAILABLE"
    assert query.turn_query(2750.0, "LEFT", -20.0).availability == "AVAILABLE"
    assert query.combined_query(3750.0, "RIGHT", "DESCENT").availability == "AVAILABLE"
    assert query.straight_query(6000.0).availability == "OUT_OF_DOMAIN"

    assert provenance["source_artifacts_unchanged"] is True
    assert provenance["source_hashes_before"] == provenance["source_hashes_after"]
    for name, source in config["source_artifacts"].items():
        assert _sha256(HERE / source["path"]) == source["sha256"] == provenance["source_hashes_after"][name]
    for item in provenance["files"].values():
        assert _sha256(Path(item["path"])) == item["sha256"]

    assert result["step_status"] == "PASS"
    assert result["full_tested_safe_envelope_extracted"] is True
    assert result["vertical_safe_values_command_constant_independent"] is True
    assert result["ready_for_aircraftprofile_integration"] is True
    assert result["audited_vertical_rows"] == 96 and result["new_jsbsim_runs"] == 0
    assert result["source_artifacts_unchanged"] is True
    assert result["aircraft_swappable"] is True
    assert result["production_planner_integrated"] is result["next_stage_started"] is False

    print("U6.2.2 tested planner-safe envelope extraction checks passed")


if __name__ == "__main__":
    main()
