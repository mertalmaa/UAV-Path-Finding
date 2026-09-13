from __future__ import annotations

import hashlib
import json
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
ALTITUDES = [1000.0, 2500.0, 4000.0, 5000.0, 5500.0]
STATUSES = {"VALID", "UNKNOWN", "INFEASIBLE"}


def _load(name: str):
    return json.loads((RESULTS / name).read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    config = yaml.safe_load(
        (HERE / "u6b_c172p_combined_3d_configuration.yaml").read_text(
            encoding="utf-8"
        )
    )
    raw = _load("c172p_combined_3d_raw.json")
    points = _load("u6b_new_points.json")
    runs = _load("u6b_new_runs.json")
    reuse = _load("u6b_reuse_audit.json")
    summary = _load("u6b_altitude_summary.json")
    provenance = _load("u6b_provenance.json")
    result = _load("u6b_result.json")
    pid = result["provenance_id"]

    assert config["frozen_stack"]["aircraft_id"] == "c172p"
    assert config["frozen_stack"]["nominal_ias_mps"] == 40
    assert list(config["representative_altitudes"].values()) == [1000, 2500, 4000, 5000, 5500]
    assert config["primary_grid"] == {
        "bank_targets_deg": [-20, 20],
        "vertical_speed_targets_mps": [-2.5, 2.5],
        "points_per_altitude": 4,
        "expected_rows": 20,
    }
    assert config["second_severity"]["selected_altitudes_m"] == [1000]
    assert config["second_severity"]["bank_targets_deg"] == [-20, 20]
    assert config["second_severity"]["vertical_speed_targets_mps"] == [-4, 4]
    assert config["second_severity"]["alternative_bank_30_sweep_executed"] is False
    assert config["simulation"]["cold_start_repetitions"] == 3
    assert config["scope"]["c172p_only"] is True
    assert all(
        value is False
        for key, value in config["scope"].items()
        if key not in {"c172p_only", "combined_climbing_turn_only", "combined_descending_turn_only"}
    )

    assert all(
        artifact["provenance_id"] == pid
        for artifact in (raw, points, runs, reuse, summary, provenance)
    )
    assert raw["artifact_type"] == "C172P_COMBINED_3D_RAW"
    assert raw["aircraft_id"] == "c172p"
    assert raw["nominal_ias_mps"] == 40.0
    assert raw["current_domain_m"] == [0.0, 5500.0]
    assert raw["representative_altitudes_m"] == ALTITUDES
    assert raw["u6a_core_lut_provenance_id"] == config["parent_u6a_provenance_id"]
    assert raw["planner_ready"] is False
    assert raw["full_envelope_claim"] is False
    assert raw["interpolated"] is False
    assert raw["derated"] is False
    assert raw["holdout_validated"] is False
    assert raw["directly_infeasible_implies_unreachable"] is False

    rows = raw["rows"]
    primary = [row for row in rows if row["grid_role"] == "PRIMARY"]
    severity = [row for row in rows if row["grid_role"] == "SECOND_SEVERITY"]
    assert len(rows) == result["total_rows"] == 24
    assert len(primary) == result["primary_rows"] == 20
    assert len(severity) == result["second_severity_rows"] == 4
    assert {
        (row["altitude_m"], row["target_bank_deg"], row["target_vz_mps"])
        for row in primary
    } == {
        (altitude, bank, vz)
        for altitude in ALTITUDES for bank in (-20.0, 20.0) for vz in (-2.5, 2.5)
    }
    assert {
        (row["altitude_m"], row["target_bank_deg"], row["target_vz_mps"])
        for row in severity
    } == {(1000.0, bank, vz) for bank in (-20.0, 20.0) for vz in (-4.0, 4.0)}
    assert {row["direction"] for row in rows} == {"LEFT", "RIGHT"}
    assert {row["maneuver_family"] for row in rows} == {"CLIMBING_TURN", "DESCENDING_TURN"}
    assert {row["status"] for row in rows} == STATUSES
    assert all(row["nominal_ias_mps"] == 40.0 for row in rows)
    assert all(row["altitude_m"] <= 5500.0 for row in rows)
    assert all(row["turn_radius_m"] > 0.0 for row in rows)
    assert all(row["level_turn_reference_radius_m"] > 0.0 for row in rows)
    assert all(row["radius_ratio"] > 0.0 for row in rows)
    assert all(row["theoretical_turn_radius_m"] > 0.0 for row in rows)
    assert all(row["repeatable"] for row in rows)
    assert all(len(row["metadata"]["source_run_ids"]) == 3 for row in rows)
    assert all(row["metadata"]["cold_start_repetitions"] == 3 for row in rows)
    assert all(row["straight_vertical_reference_vz_mps"] is None for row in primary)
    assert all(row["vz_retention_ratio"] is None for row in primary)
    assert all(len(row["straight_vertical_bracketing_raw_references"]) == 2 for row in primary)
    assert all(row["straight_vertical_reference_vz_mps"] is not None for row in severity)
    assert all(row["vz_retention_ratio"] > 0.0 for row in severity)
    assert any(row["actual_stable_response"] and row["status"] != "VALID" for row in rows)

    assert len(points["points"]) == result["new_points"] == 12
    assert len(runs["runs"]) == result["new_cold_start_runs"] == 36
    assert len({run["run_id"] for run in runs["runs"]}) == 36
    assert all(point["aircraft_model"] == "c172p" for point in points["points"])
    assert all(point["maneuver"] == "combined" for point in points["points"])
    assert all(len(point["run_ids"]) == 3 for point in points["points"])
    assert all(run["aircraft_model"] == "c172p" and run["maneuver"] == "combined" for run in runs["runs"])
    assert all(run["requested"]["ias_mps"] == 40.0 for run in runs["runs"])

    assert reuse["equivalent"] is True
    assert reuse["u5_2_reused_point_count"] == result["reused_points"] == 12
    assert len(reuse["reused_points"]) == 12
    assert all(row["source_stage"] == "U5.2" for row in reuse["reused_points"])
    assert reuse["u6a_reference_lut_unchanged"] is True
    assert reuse["same_aircraft_ias_controller_fcs_mixture_fixture_atmosphere_initialization_timestep_measurement"] is True

    expected_classes = {
        1000.0: ("MARGINAL", "MARGINAL"),
        2500.0: ("MARGINAL", "MARGINAL"),
        4000.0: ("UNUSABLE", "USABLE"),
        5000.0: ("MARGINAL", "USABLE"),
        5500.0: ("UNUSABLE", "USABLE"),
    }
    assert len(summary["rows"]) == 5
    for item in summary["rows"]:
        expected_climb, expected_descent = expected_classes[item["altitude_m"]]
        assert item["climbing_turn"]["interpretation"] == expected_climb
        assert item["descending_turn"]["interpretation"] == expected_descent
        assert item["planner_safe"] is False
        assert item["directly_infeasible_implies_unreachable"] is False

    row_5000_climb = [row for row in primary if row["altitude_m"] == 5000.0 and row["target_vz_mps"] > 0.0]
    row_5500_climb = [row for row in primary if row["altitude_m"] == 5500.0 and row["target_vz_mps"] > 0.0]
    assert all(row["throttle"] >= 0.99 and row["status"] == "INFEASIBLE" for row in row_5000_climb + row_5500_climb)
    assert all(row["actual_stable_response"] for row in row_5000_climb)
    assert all(not row["actual_stable_response"] for row in row_5500_climb)

    assert provenance["historical_sources_unchanged"] is True
    assert provenance["source_hashes_before"] == provenance["source_hashes_after"]
    assert _sha256(Path(provenance["files"]["configuration"]["path"])) == provenance["files"]["configuration"]["sha256"]
    assert _sha256(Path(provenance["files"]["harness"]["path"])) == provenance["files"]["harness"]["sha256"]
    for name, source in config["source_artifacts"].items():
        assert _sha256(HERE / source["path"]) == source["sha256"]
        assert provenance["source_hashes_after"][name] == source["sha256"]

    assert result["step_status"] == "PASS"
    assert result["combined_3d_raw_data_ready"] is True
    assert result["ready_for_u6_1_holdout_validation"] is True
    assert result["u6a_artifact_unchanged"] is True
    for forbidden in (
        "full_combined_cartesian_sweep_performed", "speed_sweep_performed",
        "controller_tuning_performed", "aircraft_xml_modified",
        "interpolation_performed", "holdout_validation_performed",
        "derating_performed", "planner_integration_performed", "u6_1_started",
    ):
        assert result[forbidden] is False

    print("U6B c172p combined 3D raw validation checks passed")


if __name__ == "__main__":
    main()
