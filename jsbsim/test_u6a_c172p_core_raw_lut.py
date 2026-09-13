from __future__ import annotations

import hashlib
import json
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
ALTITUDES = [float(value) for value in range(0, 5501, 500)]
BANKS = [0.0, -10.0, 10.0, -20.0, 20.0, -30.0, 30.0]
VERTICAL_SPEEDS = [-5.0, -4.0, -3.0, -2.0, 0.0, 2.0, 3.0, 4.0, 5.0]
STATUSES = {"VALID", "UNKNOWN", "INFEASIBLE"}


def _load(name: str):
    return json.loads((RESULTS / name).read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    config = yaml.safe_load(
        (HERE / "u6a_c172p_core_raw_lut_configuration.yaml").read_text(
            encoding="utf-8"
        )
    )
    lut = _load("c172p_core_aircraft_lut_raw.json")
    points = _load("u6a_new_points.json")
    runs = _load("u6a_new_runs.json")
    reuse = _load("u6a_reuse_audit.json")
    summary = _load("u6a_altitude_summary.json")
    provenance = _load("u6a_provenance.json")
    result = _load("u6a_result.json")
    pid = result["provenance_id"]

    assert config["frozen_stack"]["aircraft_id"] == "c172p"
    assert config["frozen_stack"]["nominal_ias_mps"] == 40
    assert config["altitude_grid_m"] == list(range(0, 5501, 500))
    assert config["turn_bank_targets_deg"] == [0, -10, 10, -20, 20, -30, 30]
    assert config["vertical_speed_targets_mps"] == [-5, -4, -3, -2, 0, 2, 3, 4, 5]
    assert config["simulation"]["cold_start_repetitions"] == 3
    assert config["scope"]["c172p_only"] is True
    assert all(
        value is False
        for key, value in config["scope"].items()
        if key not in {"c172p_only", "straight_level_only", "level_turn_only", "straight_vertical_only"}
    )

    artifacts = [lut, points, runs, reuse, summary, provenance]
    assert all(artifact["provenance_id"] == pid for artifact in artifacts)
    assert lut["artifact_type"] == "C172P_CORE_AIRCRAFT_LUT_RAW"
    assert lut["aircraft_id"] == "c172p"
    assert lut["nominal_ias_mps"] == 40.0
    assert lut["altitude_grid_m"] == ALTITUDES
    assert lut["min_altitude_m"] == 0.0
    assert lut["max_altitude_m"] == 5500.0
    assert lut["planner_ready"] is False
    assert lut["interpolated"] is False
    assert lut["derated"] is False
    assert lut["combined_3d_table_present"] is False
    assert lut["true_service_ceiling_claim"] is False
    assert lut["status_semantics"]["states"] == ["VALID", "UNKNOWN", "INFEASIBLE"]
    assert lut["mass_fuel_cg"]["observed"]["weight_lbs"] == 1865.0
    assert lut["mass_fuel_cg"]["observed"]["total_fuel_lbs"] == 185.0
    assert lut["atmosphere"]["atmosphere"] == "US_Standard"
    assert lut["timestep_s"] == 0.01
    assert len(lut["initialization_modes"]) == 2
    assert lut["code_version"]["repository_git_commit"]
    assert lut["generation_provenance"] == {"stage": "U6A", "provenance_id": pid}

    straight = lut["straight_table"]
    turns = lut["level_turn_table"]
    vertical = lut["straight_vertical_table"]
    assert len(straight) == result["straight_rows"] == 12
    assert len(turns) == result["level_turn_rows"] == 84
    assert len(vertical) == result["straight_vertical_rows"] == 108
    assert [row["altitude_m"] for row in straight] == ALTITUDES
    assert {(row["altitude_m"], row["target_bank_deg"]) for row in turns} == {
        (altitude, bank) for altitude in ALTITUDES for bank in BANKS
    }
    assert {(row["altitude_m"], row["target_vz_mps"]) for row in vertical} == {
        (altitude, vz) for altitude in ALTITUDES for vz in VERTICAL_SPEEDS
    }
    assert all(row["nominal_ias_mps"] == 40.0 for row in straight + turns + vertical)
    assert all(row["altitude_m"] <= 5500.0 for row in straight + turns + vertical)
    assert all(row["status"] in STATUSES for row in straight + turns + vertical)
    assert {row["status"] for row in straight + turns + vertical} == STATUSES
    assert all(row["status"] == "VALID" and row["actual_stable_response"] for row in straight)
    assert all(row["repeatable"] and row["settled"] for row in straight)

    for row in straight + turns + vertical:
        assert row["metadata"]["cold_start_repetitions"] == 3
        assert len(row["metadata"]["source_run_ids"]) == 3
        assert row["metadata"]["source_stage"] in {"U5", "U5.2", "U6A"}
        assert row["metadata"]["source_point_id"]
        assert row["metadata"]["source_provenance"]
        assert set(row["saturation"]) == {"aileron", "elevator", "rudder", "throttle"}
        assert "categories" in row["failure_reason"]
        assert "raw_reasons" in row["failure_reason"]

    assert {row["direction"] for row in turns} == {"LEFT", "RIGHT", "STRAIGHT"}
    assert all(row["turn_radius_m"] > 0.0 for row in turns if row["target_bank_deg"] != 0.0)
    assert all(row["theoretical_radius_m"] > 0.0 for row in turns if row["target_bank_deg"] != 0.0)
    assert all(row["turn_radius_m"] is None for row in turns if row["target_bank_deg"] == 0.0)
    assert all(row["vz_mps"] == row["actual_vz_mps"] for row in turns)
    assert all("actual_vz_mps" in row and "gamma_deg" in row for row in vertical)

    turn_5500_left20 = next(
        row for row in turns
        if row["altitude_m"] == 5500.0 and row["target_bank_deg"] == -20.0
    )
    climb_5500_two = next(
        row for row in vertical
        if row["altitude_m"] == 5500.0 and row["target_vz_mps"] == 2.0
    )
    assert turn_5500_left20["status"] == "UNKNOWN"
    assert turn_5500_left20["actual_stable_response"] is True
    assert climb_5500_two["status"] == "INFEASIBLE"
    assert climb_5500_two["actual_stable_response"] is True

    assert len(points["points"]) == result["new_unique_points"] == 161
    assert len(runs["runs"]) == result["new_cold_start_runs"] == 483
    assert len({row["run_id"] for row in runs["runs"]}) == 483
    assert all(len(point["run_ids"]) == 3 for point in points["points"])
    assert {point["aircraft_model"] for point in points["points"]} == {"c172p"}
    assert {point["requested"]["ias_mps"] for point in points["points"]} == {40.0}
    assert {point["maneuver"] for point in points["points"]} == {"straight", "turn", "vertical"}
    assert sum(point["maneuver"] == "straight" for point in points["points"]) == 5
    assert sum(point["maneuver"] == "turn" for point in points["points"]) == 60
    assert sum(point["maneuver"] == "vertical" for point in points["points"]) == 96
    assert all(point["maneuver"] != "combined" for point in points["points"])
    assert all(run["maneuver"] != "combined" for run in runs["runs"])

    assert reuse["equivalent"] is True
    assert reuse["reused_unique_source_point_count"] == result["reused_unique_points"] == 19
    assert len(reuse["reused_sources"]) == 19
    assert sum(row["source_stage"] == "U5" for row in reuse["reused_sources"]) == 12
    assert sum(row["source_stage"] == "U5.2" for row in reuse["reused_sources"]) == 7
    assert reuse["fixture_observed"]["weight_lbs"] == 1865.0
    assert reuse["fixture_observed"]["total_fuel_lbs"] == 185.0

    assert len(summary["rows"]) == 12
    assert [row["altitude_m"] for row in summary["rows"]] == ALTITUDES
    assert all(row["true_maximum_claimed"] is False for row in summary["rows"])
    assert summary["rows"][-1]["straight"]["status"] == "VALID"
    assert summary["rows"][-1]["climb"]["highest_meaningful_actual_stable_response_mps"] > 2.0
    assert summary["rows"][-1]["climb"]["highest_tested_sustainable_valid_target_mps"] is None

    assert provenance["historical_sources_unchanged"] is True
    assert provenance["source_hashes_before"] == provenance["source_hashes_after"]
    assert _sha256(Path(provenance["files"]["configuration"]["path"])) == provenance["files"]["configuration"]["sha256"]
    assert _sha256(Path(provenance["files"]["harness"]["path"])) == provenance["files"]["harness"]["sha256"]
    for name, source in config["source_artifacts"].items():
        assert _sha256(HERE / source["path"]) == source["sha256"]
        assert provenance["source_hashes_after"][name] == source["sha256"]

    assert result["step_status"] == "PASS"
    assert result["core_raw_lut_ready"] is True
    assert result["current_domain_m"] == [0.0, 5500.0]
    assert result["ready_for_u6b"] is True
    assert result["historical_sources_unchanged"] is True
    for forbidden in (
        "combined_3d_executed", "controller_tuning_performed", "aircraft_xml_modified",
        "interpolation_performed", "holdout_validation_performed", "derating_performed",
        "planner_integration_performed", "u6b_started",
    ):
        assert result[forbidden] is False

    print("U6A c172p core raw LUT artifact checks passed")


if __name__ == "__main__":
    main()
