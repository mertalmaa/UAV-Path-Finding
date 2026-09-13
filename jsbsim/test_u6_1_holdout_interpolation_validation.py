from __future__ import annotations

import hashlib
import json
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
CORE_ALTITUDES = [250.0, 1250.0, 2250.0, 3250.0, 4250.0, 5250.0]
COMBINED_ALTITUDES = [1750.0, 3250.0, 4500.0, 5250.0]


def _load(name: str):
    return json.loads((RESULTS / name).read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    config = yaml.safe_load(
        (HERE / "u6_1_holdout_interpolation_configuration.yaml").read_text(
            encoding="utf-8"
        )
    )
    core = _load("u6_1_core_holdout_results.json")
    combined = _load("u6_1_combined_holdout_results.json")
    errors = _load("u6_1_interpolation_error_summary.json")
    suitability = _load("u6_1_interpolation_suitability.json")
    outlier = _load("u6_1_outlier_diagnostics.json")
    points = _load("u6_1_new_points.json")
    runs = _load("u6_1_new_runs.json")
    provenance = _load("u6_1_provenance.json")
    result = _load("u6_1_result.json")
    pid = result["provenance_id"]

    assert config["frozen_stack"]["aircraft_id"] == "c172p"
    assert config["frozen_stack"]["nominal_ias_mps"] == 40
    assert config["frozen_stack"]["current_domain_m"] == [0, 5500]
    assert config["core_holdout_altitudes_m"] == [250, 1250, 2250, 3250, 4250, 5250]
    assert config["combined_holdout_altitudes_m"] == [1750, 3250, 4500, 5250]
    assert config["expected_execution"]["total_points"] == 62
    assert config["expected_execution"]["total_runs"] == 186
    assert config["interpolation"]["categorical_status_interpolated"] is False
    assert config["scope"]["c172p_only"] is True
    assert all(
        value is False
        for key, value in config["scope"].items()
        if key not in {"c172p_only", "unseen_midpoint_holdouts_only"}
    )

    artifacts = [core, combined, errors, suitability, outlier, points, runs, provenance]
    assert all(artifact["provenance_id"] == pid for artifact in artifacts)
    assert core["holdout_altitudes_m"] == CORE_ALTITUDES
    assert combined["holdout_altitudes_m"] == COMBINED_ALTITUDES
    assert core["predictions_generated_before_execution"] is True
    assert combined["predictions_generated_before_execution"] is True
    assert core["status_interpolated"] is False
    assert combined["status_interpolated"] is False
    assert core["planner_safe"] is False and core["derated"] is False
    assert combined["planner_safe"] is False and combined["derated"] is False

    core_rows = core["rows"]
    combined_rows = combined["rows"]
    assert len(core_rows) == result["core_holdout_points"] == 46
    assert len(combined_rows) == result["combined_holdout_points"] == 16
    assert sum(row["family"] == "straight" for row in core_rows) == 6
    assert sum(row["family"] == "level_turn" for row in core_rows) == 18
    assert sum(row["family"] in {"climb", "descent"} for row in core_rows) == 22
    assert {row["altitude_m"] for row in core_rows} == set(CORE_ALTITUDES)
    assert {row["altitude_m"] for row in combined_rows} == set(COMBINED_ALTITUDES)
    assert all(row["altitude_m"] % 500.0 == 250.0 for row in core_rows)
    assert all(row["prediction"]["lower_anchor_altitude_m"] < row["altitude_m"] < row["prediction"]["upper_anchor_altitude_m"] for row in core_rows + combined_rows)
    assert all(row["prediction"]["generated_before_holdout_execution"] for row in core_rows + combined_rows)
    assert all(row["repeatable"] for row in core_rows + combined_rows)
    assert all(len(row["run_ids"]) == 3 for row in core_rows + combined_rows)
    assert all(row["holdout_status"] in {"VALID", "UNKNOWN", "INFEASIBLE"} for row in core_rows + combined_rows)
    assert all(row["prediction_direction_sign_correct"] for row in core_rows + combined_rows)
    assert {row["direction"] for row in combined_rows} == {"LEFT", "RIGHT"}
    assert {
        (row["altitude_m"], row["target_bank_deg"], row["target_vz_mps"])
        for row in combined_rows
    } == {
        (altitude, bank, vz)
        for altitude in COMBINED_ALTITUDES
        for bank in (-20.0, 20.0)
        for vz in (-2.5, 2.5)
    }

    assert len(points["points"]) == result["total_holdout_points"] == 62
    assert len(runs["runs"]) == result["cold_start_runs"] == 186
    assert len({run["run_id"] for run in runs["runs"]}) == 186
    assert all(point["aircraft_model"] == "c172p" for point in points["points"])
    assert all(point["requested"]["ias_mps"] == 40.0 for point in points["points"])
    assert all(len(point["run_ids"]) == 3 for point in points["points"])
    assert all(run["aircraft_model"] == "c172p" for run in runs["runs"])

    keys = {(row["dataset"], row["family"], row["variable"]): row for row in errors["rows"]}
    assert len(keys) == len(errors["rows"])
    for key in (
        ("CORE", "straight", "throttle"),
        ("CORE", "level_turn", "turn_radius_m"),
        ("CORE", "climb", "actual_vz_mps"),
        ("CORE", "descent", "actual_vz_mps"),
        ("COMBINED", "climbing_turn", "turn_radius_m"),
        ("COMBINED", "climbing_turn", "actual_vz_mps"),
        ("COMBINED", "descending_turn", "turn_radius_m"),
        ("COMBINED", "descending_turn", "actual_vz_mps"),
    ):
        row = keys[key]
        assert row["count"] > 0
        assert row["mean_absolute_error"] is not None
        assert row["median_absolute_error"] is not None
        assert row["max_absolute_error"] is not None
        assert row["mean_relative_error_pct"] is not None
        assert row["p90_relative_error_pct"] is not None
    assert errors["relative_error_denominator"] == "absolute_actual_value"

    critical_4500 = [row for row in combined_rows if row["altitude_m"] == 4500.0 and row["target_vz_mps"] > 0.0]
    critical_5250 = [row for row in combined_rows if row["altitude_m"] == 5250.0 and row["target_vz_mps"] > 0.0]
    assert len(critical_4500) == len(critical_5250) == 2
    assert all(row["holdout_status"] == "INFEASIBLE" and row["actual_stable_response"] for row in critical_4500)
    assert all(row["holdout_status"] == "INFEASIBLE" for row in critical_5250)
    assert {row["actual_stable_response"] for row in critical_5250} == {False, True}
    assert not any(
        flag
        for row in core_rows + combined_rows
        for flag in row["status_discontinuity"].values()
    )

    assert outlier["repeat_count"] == 3
    assert outlier["repeat_trajectories_identical"] is True
    assert outlier["heading_unwrap_audit"]["turn_rate_sign_correct"] is True
    assert outlier["radius_extraction_audit"]["stored_ground_track_change_deg"] > 180.0
    assert outlier["radius_extraction_audit"]["equivalent_wrapped_ground_track_change_deg"] < 0.0
    assert outlier["radius_extraction_audit"]["audited_theory_relative_error"] < outlier["radius_extraction_audit"]["stored_theory_relative_error"]
    assert outlier["conclusion"] == "MEASUREMENT_OUTLIER_NOT_PHYSICAL_LEFT_RIGHT_DYNAMIC_ASYMMETRY"
    assert outlier["diagnostic_rerun_performed"] is False
    assert outlier["raw_u6b_point_modified"] is False
    adjusted = [
        adjustment
        for row in combined_rows
        for adjustment in row["prediction"]["source_adjustments"]
    ]
    assert len(adjusted) == 1
    assert adjusted[0]["anchor_altitude_m"] == 1000.0

    labels = {row["maneuver_region"]: row["label"] for row in suitability["labels"]}
    assert set(labels.values()) == {"INTERPOLATION_SUPPORTED", "LIMITED", "UNSUPPORTED"}
    assert labels["straight_0_5500"] == "INTERPOLATION_SUPPORTED"
    assert labels["level_turn_pm20_0_5500"] == "INTERPOLATION_SUPPORTED"
    assert labels["level_turn_pm30_sampled"] == "LIMITED"
    assert labels["combined_descending_turn_1000_5500"] == "INTERPOLATION_SUPPORTED"
    assert labels["combined_climbing_turn_4000_5500"] == "UNSUPPORTED"
    assert suitability["threshold_policy"] == "NO_ARBITRARY_NUMERIC_SAFETY_THRESHOLD"

    assert provenance["source_artifacts_unchanged"] is True
    assert provenance["source_hashes_before"] == provenance["source_hashes_after"]
    assert _sha256(Path(provenance["files"]["configuration"]["path"])) == provenance["files"]["configuration"]["sha256"]
    assert _sha256(Path(provenance["files"]["harness"]["path"])) == provenance["files"]["harness"]["sha256"]
    for name, source in config["source_artifacts"].items():
        assert _sha256(HERE / source["path"]) == source["sha256"]
        assert provenance["source_hashes_after"][name] == source["sha256"]

    assert result["step_status"] == "PASS"
    assert result["core_interpolation_validated"] == "PARTIAL"
    assert result["combined_interpolation_validated"] == "PARTIAL"
    assert result["outlier_resolved"] == "YES"
    assert result["ready_for_u6_2_planner_safe_derating"] is True
    assert result["source_artifacts_unchanged"] is True
    for forbidden in (
        "derating_performed", "planner_safe_lut_generated",
        "planner_integration_performed", "controller_tuning_performed",
        "aircraft_xml_modified", "u6_2_started",
    ):
        assert result[forbidden] is False

    print("U6.1 holdout and interpolation validation checks passed")


if __name__ == "__main__":
    main()
