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
        (HERE / "u4_1a2_suitability_diagnostic_configuration.yaml").read_text(
            encoding="utf-8"
        )
    )
    vertical = _load("u4_1a2_vertical_unknown_audit.json")
    high = _load("u4_1a2_high_altitude_diagnostic.json")
    provenance = _load("u4_1a2_provenance.json")
    runs_artifact = _load("u4_1a2_runs.json")
    points_artifact = _load("u4_1a2_points.json")
    result = _load("u4_1a2_result.json")
    raw = _load("aircraft_lut_raw.json")
    pid = result["provenance_id"]

    assert config["scope"]["diagnostic_only"] is True
    assert all(
        value is False
        for key, value in config["scope"].items()
        if key != "diagnostic_only"
    )
    assert config["high_altitude"]["baseline_altitudes_m"] == [2500, 3000, 3500]
    assert config["high_altitude"]["mixture_only_diagnostic_altitudes_m"] == [3000, 3500, 4000]
    assert config["vertical_unknown_audit"]["representative_reruns_required"] is False

    assert pid == vertical["provenance_id"] == high["provenance_id"]
    assert pid == provenance["provenance_id"] == runs_artifact["provenance_id"]
    assert pid == points_artifact["provenance_id"]
    assert provenance["parent_u4_provenance_id"] == raw["provenance_id"]
    for item in provenance["files"].values():
        path = Path(item["path"])
        assert path.is_file()
        assert _sha256(path) == item["sha256"]

    expected_hashes = {
        "u4_raw_lut": config["parent_raw_lut_sha256"],
        "u4_1a": config["parent_u4_1a_sha256"],
        "u4_1a1": config["parent_u4_1a1_sha256"],
    }
    assert high["historical_artifact_hashes_before"] == expected_hashes
    assert high["historical_artifact_hashes_after"] == expected_hashes
    assert high["historical_artifacts_unchanged"] is True

    assert vertical["unknown_point_count"] == 28
    assert vertical["classification_counts"] == {
        "CONTROLLER_LIMITED": 5,
        "NEAR_TARGET_STABLE": 23,
    }
    assert vertical["all_non_valid_context_counts"] == {
        "CONTROLLER_LIMITED": 5,
        "POWER_LIMITED": 7,
        "TRACKING_ACCEPTANCE_LIMITED": 23,
    }
    assert len(vertical["power_limited_infeasible_context"]) == 7
    assert all(
        row["status"] == "INFEASIBLE"
        and row["failure_category"] == "POWER_LIMITED"
        for row in vertical["power_limited_infeasible_context"]
    )
    assert len(vertical["rows"]) == 28
    assert all(row["status"] == "UNKNOWN" for row in vertical["rows"])
    assert all(row["production_status_changed"] is False for row in vertical["rows"])
    assert all(row["repeatability"]["passed"] for row in vertical["rows"])
    assert all(math.isfinite(row["target_attainment_ratio"]) for row in vertical["rows"])
    assert all(
        row["horizontal_distance_per_100m_using_actual_response_m"] > 0.0
        for row in vertical["rows"]
    )
    assert vertical["representative_reruns"]["executed"] is False
    assert vertical["planner_interpretation"]["repeatable_response_beyond_2_mps_exists"] is True
    assert vertical["planner_interpretation"]["unknown_points_promoted_to_valid"] == 0
    acceptance = vertical["acceptance_criteria_audit"]
    assert acceptance["thresholds"]["ias_tracking_relative"] == 0.02
    assert acceptance["thresholds"]["ias_tracking_absolute_at_40_mps"] == 0.8
    assert acceptance["thresholds"]["vertical_speed_tracking_absolute_mps"] == 0.5
    assert acceptance["thresholds"]["bank_tracking_absolute_deg"] == 1.5
    assert acceptance["thresholds_modified_in_u4_1a2"] is False
    assert acceptance["physical_aircraft_boundary_criterion"] is False
    assert acceptance["physically_calibrated_planner_primitive_tolerance"] is False

    baseline = {row["altitude_m"]: row for row in high["baseline_runs"]}
    mixture = {row["altitude_m"]: row for row in high["mixture_only_diagnostic_runs"]}
    assert {altitude: row["status"] for altitude, row in baseline.items()} == {
        2500.0: "VALID",
        3000.0: "INFEASIBLE",
        3500.0: "INFEASIBLE",
    }
    assert all(row["matches_historical_u4_status"] for row in baseline.values())
    assert all(row["one_factor_changed"] is None for row in baseline.values())
    assert baseline[2500.0]["engine"]["rpm_mean"] > 0.0
    for altitude in (3000.0, 3500.0):
        assert baseline[altitude]["engine"]["rpm_mean"] == 0.0
        assert baseline[altitude]["engine"]["fuel_flow_gph_mean"] == 0.0
        assert baseline[altitude]["trim"]["fresh_untrimmed_fallback_used"] is True

    assert set(mixture) == {3000.0, 3500.0, 4000.0}
    for altitude, row in mixture.items():
        assert row["status"] == "VALID"
        assert row["one_factor_changed"] == "MIXTURE_HANDLING_ONLY"
        assert row["engine"]["rpm_mean"] > 0.0
        assert row["engine"]["thrust_lbs_mean"] > 0.0
        assert row["engine"]["fuel_flow_gph_mean"] > 0.0
        assert row["trim"]["all_succeeded"] is True
        assert row["settling"]["all_repeats_settled_before_measurement"] is True
        assert math.isclose(
            row["requested_mixture_command_norm"],
            row["standard_atmosphere_pressure_psf"] / 2117.0,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        )
    assert high["root_cause"]["primary_category"] == "ENGINE_MIXTURE_PROPULSION_FIXTURE"
    assert high["root_cause"]["mixture_only_resolves_3000_3500"] is True
    assert high["root_cause"]["mixture_only_4000_status"] == "VALID"
    assert high["root_cause"]["trim_initialization_only_explanation_rejected"] is True
    assert high["root_cause"]["controller_primary_cause_rejected"] is True
    assert high["root_cause"]["aircraft_capability_limit_proven"] is False
    assert high["root_cause"]["true_service_ceiling_claimed"] is False
    assert high["one_factor_at_a_time_contract"]["multiple_variables_changed_together"] is False
    assert high["one_factor_at_a_time_contract"]["diagnostic_promoted_to_production"] is False

    propulsion = high["propulsion_mixture_model_audit"]
    assert propulsion["engine_type"] == "piston_engine"
    assert propulsion["engine_file"] == "engIO360C"
    assert propulsion["maximum_hp"] == 180.0
    assert propulsion["supercharger_or_turbo_declared"] is False
    assert propulsion["c172r_aircraft_xml_automatic_mixture_or_altitude_compensation"] is False
    assert propulsion["propeller_is_fixed_pitch"] is True
    assert propulsion["propeller_min_pitch_deg"] == propulsion["propeller_max_pitch_deg"] == 21.6

    runs = runs_artifact["runs"]
    points = points_artifact["points"]
    assert result["executed_new_points"] == len(points) == 6
    assert result["executed_new_cold_start_runs"] == len(runs) == 18
    assert all(len(point["run_ids"]) == 3 for point in points)
    assert len({run["run_id"] for run in runs}) == 18
    assert all(run["provenance_id"] == pid for run in runs)
    assert all(point["provenance_id"] == pid for point in points)
    assert result["step"] == "U4.1A.2"
    assert result["step_status"] == "PASS"
    assert result["aircraft_suitability"] == high["aircraft_suitability"] == "CONTINUE"
    assert result["ready_for_u4_1b"] is high["ready_for_u4_1b"] is False
    assert result["historical_artifacts_unchanged"] is True
    assert result["controller_tuning_performed"] is False
    assert result["stock_xml_modified"] is False
    assert result["planner_modified"] is False
    assert result["interpolation_performed"] is False
    assert result["derating_performed"] is False
    assert result["holdout_validation_started"] is False
    assert result["aircraft_switched"] is False
    assert result["acceptance_thresholds_modified"] is False
    assert result["true_service_ceiling_claimed"] is False
    print("U4.1A.2 suitability diagnostic artifact and contract checks passed")


if __name__ == "__main__":
    main()
