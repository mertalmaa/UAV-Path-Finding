from __future__ import annotations

import hashlib
import json
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"


def _load(name: str):
    return json.loads((RESULTS / name).read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    config = yaml.safe_load(
        (HERE / "u5_1_multi_aircraft_envelope_audit_configuration.yaml").read_text(
            encoding="utf-8"
        )
    )
    inventory = _load("u5_1_data_inventory.json")
    maps = _load("u5_1_altitude_speed_maps.json")
    maneuvers = _load("u5_1_maneuver_coverage.json")
    envelopes = _load("u5_1_envelope_summaries.json")
    cross = _load("u5_1_cross_aircraft_comparison.json")
    decision = _load("u5_1_decision_recheck.json")
    provenance = _load("u5_1_provenance.json")
    result = _load("u5_1_result.json")
    pid = result["provenance_id"]

    assert config["candidate_order"] == [
        "c172r", "c172p", "c172x", "c182", "DHC6", "J3Cub", "pa28", "c310"
    ]
    assert config["ideal_comparison_altitudes_m"] == [0, 1000, 2000, 3000, 4000, 5000, 5500, 6000]
    assert config["display_speed_columns_mps"] == [30, 40, 50, 60]
    assert config["policy"]["existing_u5_data_first"] is True
    assert config["policy"]["mentor_values_are_hard_gates"] is False
    assert config["policy"]["new_simulation_required"] is False
    assert all(value is False for value in config["scope"].values())

    assert pid == inventory["provenance_id"] == maps["provenance_id"]
    assert pid == maneuvers["provenance_id"] == envelopes["provenance_id"]
    assert pid == cross["provenance_id"] == decision["provenance_id"]
    assert pid == provenance["provenance_id"]
    assert inventory["parent_u5_provenance_id"] == config["parent_provenance_id"]
    assert inventory["executed_point_count"] == result["source_operating_points_reused"] == 189
    assert inventory["executed_cold_start_run_count"] == result["source_cold_start_runs_reused"] == 567
    assert inventory["all_run_ids_unique"] is True
    assert inventory["new_simulation_runs"] == result["new_simulation_runs"] == 0
    assert inventory["ideal_levels_missing_from_all_u5_data_m"] == [1000, 2000, 4000]

    coverage = {row["aircraft"]: row for row in inventory["coverage"]}
    assert set(coverage) == set(config["candidate_order"])
    for model in config["candidate_order"][:-1]:
        assert coverage[model]["tested_altitudes_m"] == [0.0, 3000.0, 5000.0, 5500.0, 6000.0]
        assert coverage[model]["tested_ias_mps"] == [30.0, 40.0, 50.0, 60.0]
    assert coverage["c310"]["tested_altitudes_m"] == [0.0, 3000.0]
    assert sum(row["executed_point_count"] for row in coverage.values()) == 189
    assert sum(row["executed_cold_start_run_count"] for row in coverage.values()) == 567

    matrices = {row["aircraft"]: row["matrix"] for row in maps["aircraft"]}
    assert len(matrices) == 8
    assert all(len(matrix) == 8 for matrix in matrices.values())
    assert all(len(row["cells"]) == 4 for matrix in matrices.values() for row in matrix)
    for matrix in matrices.values():
        for altitude in (1000.0, 2000.0, 4000.0):
            row = next(x for x in matrix if x["altitude_m"] == altitude)
            assert {cell["classification"] for cell in row["cells"]} == {"NOT_TESTED"}

    def cell(model: str, altitude: float, speed: float):
        row = next(x for x in matrices[model] if x["altitude_m"] == altitude)
        return next(x for x in row["cells"] if x["ias_mps"] == speed)

    assert cell("c172p", 0.0, 40.0)["classification"] == "USABLE"
    assert cell("c172p", 3000.0, 40.0)["classification"] == "USABLE"
    assert cell("c172p", 5000.0, 40.0)["classification"] == "USABLE"
    assert cell("c172p", 5500.0, 40.0)["classification"] == "USABLE"
    assert cell("c172p", 6000.0, 40.0)["classification"] == "USABLE"
    assert cell("c172p", 3000.0, 60.0)["classification"] == "UNUSABLE"
    assert cell("DHC6", 6000.0, 60.0)["classification"] == "MARGINAL"
    assert "low_power_margin" in cell("DHC6", 6000.0, 60.0)["classification_reasons"]
    assert cell("c172x", 5000.0, 30.0)["classification"] == "MARGINAL"
    assert cell("c310", 5000.0, 50.0)["classification"] == "NOT_TESTED"
    assert maps["raw_status_semantics_retained"] is True

    regions = {(row["aircraft"], row["altitude_m"]): row for row in maneuvers["regions"]}
    for model in ("c172p", "DHC6"):
        for altitude in (5000.0, 5500.0, 6000.0):
            region = regions[(model, altitude)]
            assert len(region["turns"]) == 4
            assert len(region["vertical"]) == 4
            assert all(turn["trajectory"]["measured_radius_m"] > 0.0 for turn in region["turns"])
            assert all(turn["trajectory"]["turn_rate_coefficient_of_variation"] >= 0.0 for turn in region["turns"])
    c172p_climb = next(x for x in regions[("c172p", 5000.0)]["vertical"] if x["target"] == 2.5)
    assert c172p_climb["raw_strict_status"] == "INFEASIBLE"
    assert c172p_climb["actual_stable_response"] is True
    assert abs(c172p_climb["actual_vertical_speed_mps"] - 2.5) < 0.01
    assert all(regions[("c310", altitude)]["not_tested_reason"] == "phase1_early_rejection" for altitude in (5000.0, 5500.0, 6000.0))

    summary = {row["aircraft"]: row for row in envelopes["summaries"]}
    assert summary["c172p"]["general_usable_speed_region_mps"] == [40, 50]
    assert summary["c172p"]["general_stable_speed_region_mps"] == [30, 40, 50]
    assert summary["c172p"]["high_altitude_stable_speed_region_mps"] == [40.0]
    assert summary["DHC6"]["general_usable_speed_region_mps"] == [60]
    assert summary["DHC6"]["high_altitude_stable_speed_region_mps"] == [60.0]
    assert summary["c310"]["high_altitude_stable_speed_region_mps"] == []

    rows = cross["rows"]
    assert [row["aircraft"] for row in rows] == [
        "c172p", "DHC6", "c172r", "c172x", "c182", "J3Cub", "pa28", "c310"
    ]
    assert rows[0]["usability_5000_m"] == rows[0]["usability_5500_m"] == "USABLE"
    assert rows[0]["usability_6000_m"] == "LIMITED"
    assert rows[1]["usability_5000_m"] == rows[1]["usability_5500_m"] == "USABLE"
    assert rows[1]["usability_6000_m"] == "LIMITED"
    assert rows[0]["general_good_speed_region_mps"] == [40, 50]
    assert rows[1]["general_good_speed_region_mps"] == [60]

    assert decision["primary_decision"] == result["primary_decision"] == "CONFIRMED"
    assert decision["backup_decision"] == result["backup_decision"] == "CONFIRMED"
    assert decision["primary_aircraft"] == result["primary_aircraft"] == "c172p"
    assert decision["backup_aircraft"] == result["backup_aircraft"] == "DHC6"
    assert decision["selected_nominal_ias_mps"] == result["selected_nominal_ias_mps"] == 40.0
    assert decision["single_speed_policy_retained"] is True
    assert decision["speed_state_dimension_added"] is False
    assert decision["dense_0_to_6000_envelope_available"] is False
    assert decision["selection_confirmation_coverage_sufficient"] is True
    assert decision["minimal_new_runs_required"] is False
    assert decision["aircraft_selection_really_frozen"] is True

    assert provenance["source_artifact_hashes_before"] == provenance["source_artifact_hashes_after"]
    assert provenance["source_artifacts_unchanged"] is True
    assert provenance["new_simulation_runs"] == 0
    assert _sha256(Path(provenance["configuration"]["path"])) == provenance["configuration"]["sha256"]
    assert _sha256(Path(provenance["harness"]["path"])) == provenance["harness"]["sha256"]
    for name, relative in config["source_artifacts"].items():
        assert _sha256(HERE / relative) == provenance["source_artifact_hashes_after"][name]

    assert result["step_status"] == "PASS"
    assert result["aircraft_selection_really_frozen"] is True
    assert result["ready_to_return_to_path_planner"] is True
    assert result["source_artifacts_unchanged"] is True
    for forbidden in (
        "full_u5_rerun_performed",
        "full_lut_generated",
        "controller_tuning_performed",
        "aircraft_xml_modified",
        "planner_or_search_modified",
        "next_stage_started",
    ):
        assert result[forbidden] is False
    print("U5.1 existing-data altitude/speed envelope audit checks passed")


if __name__ == "__main__":
    main()
