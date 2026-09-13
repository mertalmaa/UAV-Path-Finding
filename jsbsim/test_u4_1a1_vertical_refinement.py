from __future__ import annotations

import hashlib
import json
import math
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
        (HERE / "u4_1a1_vertical_refinement_configuration.yaml").read_text(
            encoding="utf-8"
        )
    )
    artifact = _load("u4_1a1_vertical_refinement.json")
    provenance = _load("u4_1a1_provenance.json")
    runs_artifact = _load("u4_1a1_runs.json")
    points_artifact = _load("u4_1a1_points.json")
    result = _load("u4_1a1_result.json")
    raw_path = RESULTS / "aircraft_lut_raw.json"
    raw = _load("aircraft_lut_raw.json")
    pid = result["provenance_id"]

    assert config["representative_altitudes_m"] == [0, 1000, 2000]
    assert config["include_optional_2500_m"] is False
    assert config["climb_targets_mps"] == [2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0]
    assert config["descent_targets_mps"] == [-2.0, -2.5, -3.0, -3.5, -4.0, -4.5, -5.0]
    assert config["measurement_contract"]["same_as_u4"] is True
    assert config["reuse_existing_u4_integer_points"] is True
    assert config["scope"]["vertical_refinement_only"] is True
    assert all(
        value is False
        for key, value in config["scope"].items()
        if key != "vertical_refinement_only"
    )

    assert pid == artifact["provenance_id"] == provenance["provenance_id"]
    assert pid == runs_artifact["provenance_id"] == points_artifact["provenance_id"]
    assert artifact["parent_u4_provenance_id"] == raw["provenance_id"]
    assert artifact["parent_raw_lut_unchanged"] is True
    assert artifact["parent_raw_lut_sha256_before"] == _sha256(raw_path)
    assert artifact["parent_raw_lut_sha256_after"] == _sha256(raw_path)
    assert _sha256(raw_path) == config["parent_raw_lut_sha256"]
    for item in provenance["files"].values():
        path = Path(item["path"])
        assert path.is_file()
        assert _sha256(path) == item["sha256"]

    rows = artifact["rows"]
    expected_targets = set(config["climb_targets_mps"] + config["descent_targets_mps"])
    assert len(rows) == 42
    assert len(artifact["boundary_summary"]) == 3
    for altitude in (0.0, 1000.0, 2000.0):
        altitude_rows = [row for row in rows if row["altitude_m"] == altitude]
        assert len(altitude_rows) == 14
        assert {row["target_vz_mps"] for row in altitude_rows} == expected_targets

    reused = [row for row in rows if row["source"] == "U4_EXISTING_RAW_LUT_REUSED"]
    new = [row for row in rows if row["source"] == "U4_1A1_NEW_HALF_STEP_PROBE"]
    assert len(reused) == 24
    assert len(new) == 18
    assert all(float(row["target_vz_mps"]).is_integer() for row in reused)
    assert {row["target_vz_mps"] for row in new} == {-4.5, -3.5, -2.5, 2.5, 3.5, 4.5}
    assert all(row["provenance_id"] == raw["provenance_id"] for row in reused)
    assert all(row["provenance_id"] == pid for row in new)
    assert all(len(row["run_ids"]) == 3 for row in rows)
    assert all(row["repeatability"]["passed"] for row in rows)

    required = {
        "target_vz_mps",
        "actual_vz_mps",
        "actual_ias_mps",
        "ias_retention_ratio",
        "gamma_deg",
        "pitch_deg",
        "aoa_deg",
        "beta_deg",
        "throttle_norm",
        "engine_indicators",
        "elevator_usage",
        "controller_saturation_in_measurement",
        "settling",
        "measurement_window_stability",
        "repeatability",
        "status",
        "failure_analysis",
    }
    allowed_statuses = {"VALID", "INFEASIBLE", "UNKNOWN"}
    allowed_failures = set(config["failure_analysis_categories"])
    for row in rows:
        assert required <= set(row)
        assert row["status"] in allowed_statuses
        assert row["measurement_window_stability"]["all_windows_complete"] is True
        assert row["engine_indicators"]["propeller_rpm"]["mean"] > 0.0
        thrust = row["engine_indicators"]["thrust_lbs"]["mean"]
        assert math.isfinite(thrust) and thrust >= 0.0
        if row["status"] == "VALID":
            assert row["failure_analysis"]["category"] is None
            assert row["horizontal_distance_per_100m_altitude_change_m"] is not None
        else:
            assert row["failure_analysis"]["category"] in allowed_failures
            assert row["horizontal_distance_per_100m_altitude_change_m"] is None

    expected_boundaries = {
        0.0: (2.0, 2.5, -2.0, -2.5),
        1000.0: (2.5, 3.0, -2.0, -2.5),
        2000.0: (2.0, 2.5, -2.0, -2.5),
    }
    for item in artifact["boundary_summary"]:
        climb_valid, climb_nonvalid, descent_valid, descent_nonvalid = expected_boundaries[
            item["altitude_m"]
        ]
        assert item["climb"]["highest_or_largest_tested_sustainable_valid_target_vz_mps"] == climb_valid
        assert item["climb"]["first_non_valid_target_beyond_mps"] == climb_nonvalid
        assert item["descent"]["highest_or_largest_tested_sustainable_valid_target_vz_mps"] == descent_valid
        assert item["descent"]["first_non_valid_target_beyond_mps"] == descent_nonvalid
        assert item["climb"]["true_maximum_claimed"] is False
        assert item["descent"]["true_maximum_claimed"] is False

    interpretation = artifact["interpretation"]
    assert interpretation["capability_beyond_2_mps"] == {"climb": True, "descent": False}
    assert interpretation["plus_minus_2_is_hard_planner_limit"] is False
    assert interpretation["climb_descent_symmetry_assumed"] is False
    assert interpretation["true_physical_maximum_extracted"] is False
    assert interpretation["planner_safe_limit_selected"] is False
    assert interpretation["horizontal_distance_metric_bound_to_planner"] is False

    runs = runs_artifact["runs"]
    points = points_artifact["points"]
    assert result["refined_rows"] == len(rows) == 42
    assert result["reused_u4_points"] == 24
    assert result["executed_new_points"] == len(points) == 18
    assert result["executed_new_cold_start_runs"] == len(runs) == 54
    assert all(len(point["run_ids"]) == 3 for point in points)
    assert all(run["provenance_id"] == pid for run in runs)
    assert all(point["provenance_id"] == pid for point in points)
    assert result["step"] == "U4.1A.1"
    assert result["step_status"] == "PASS"
    assert result["vertical_capability_sufficiently_refined_for_holdout_stage"] is True
    assert result["raw_u4_lut_unchanged"] is True
    assert result["controller_tuning_performed"] is False
    assert result["stock_xml_modified"] is False
    assert result["planner_modified"] is False
    assert result["interpolation_performed"] is False
    assert result["derating_performed"] is False
    assert result["holdout_validation_started"] is False
    assert result["true_physical_maximum_extracted"] is False
    print("U4.1A.1 vertical-refinement artifact and contract checks passed")


if __name__ == "__main__":
    main()
