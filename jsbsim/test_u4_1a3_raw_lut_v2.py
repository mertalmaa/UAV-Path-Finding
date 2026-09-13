from __future__ import annotations

import hashlib
import json
import math
import sys
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
sys.path.insert(0, str(HERE))

import production_mixture_policy as policy


def _load(name: str):
    return json.loads((RESULTS / name).read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    config = yaml.safe_load(
        (HERE / "u4_1a3_raw_lut_v2_configuration.yaml").read_text(encoding="utf-8")
    )
    raw = _load("aircraft_lut_raw_v2.json")
    sanity = _load("u4_1a3_sanity_gate.json")
    audit = _load("u4_1a3_mixture_policy_audit.json")
    comparison = _load("u4_1a3_old_vs_v2.json")
    ceiling = _load("u4_1a3_ceiling_assessment.json")
    provenance = _load("u4_1a3_provenance.json")
    runs = _load("u4_1a3_runs.json")
    points = _load("u4_1a3_points.json")
    result = _load("u4_1a3_result.json")
    pid = result["provenance_id"]

    assert config["main_altitude_grid_m"] == list(range(0, 6001, 500))
    assert config["sanity_altitude_grid_m"] == [
        0, 1000, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000
    ]
    assert config["turn_bank_grid_deg"] == [0, -10, 10, -15, 15, -20, 20, -25, 25]
    assert config["vertical_speed_grid_mps"] == [-5, -4, -3, -2, 0, 2, 3, 4, 5]
    assert config["reuse_contract"]["decision"] == "CASE_B_FULL_RAW_RERUN"
    assert config["scope"]["planner_integration"] is False
    assert config["scope"]["interpolation"] is False
    assert config["scope"]["derating"] is False

    assert policy.POLICY_ID == "c172r-production-mixture-pressure-ratio-v1"
    assert policy.command_from_pressure_psf(0.0) == 0.0
    assert policy.command_from_pressure_psf(1058.5) == 0.5
    assert policy.command_from_pressure_psf(2117.0) == 1.0
    assert policy.command_from_pressure_psf(3000.0) == 1.0
    assert audit["all_observations_match"] is True
    assert audit["u4_1a2_diagnostic_execution"]["clipping"] == "none in the diagnostic helper"
    for row in audit["u4_1a2_observations"]:
        assert row["matches_exact_formula_within_1e-12"] is True
        assert math.isclose(
            row["observed_command_norm"],
            row["pressure_psf"] / 2117.0,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        )

    assert pid == raw["provenance_id"] == sanity["provenance_id"]
    assert pid == audit["u4_1a2_observations"][0].get("provenance_id", pid)
    assert pid == comparison["provenance_id"] == ceiling["provenance_id"]
    assert pid == provenance["provenance_id"] == runs["provenance_id"]
    assert pid == points["provenance_id"]
    assert raw["schema_version"] == 2
    assert raw["aircraft_id"] == "c172r"
    assert raw["controller_stack_id"] == config["production_stack"]["stack_id"]
    assert raw["mixture_policy_id"] == policy.POLICY_ID
    assert raw["supersedes"]["provenance_id"] == config["parent_u4_provenance_id"]
    assert raw["supersedes"]["artifact_modified"] is False

    for item in provenance["files"].values():
        path = Path(item["path"])
        assert path.is_file()
        assert _sha256(path) == item["sha256"]
    expected_historical = {
        name: item["sha256"] for name, item in config["historical_artifacts"].items()
    }
    assert provenance["historical_artifact_hashes_before"] == expected_historical
    assert provenance["historical_artifact_hashes_after"] == expected_historical
    assert provenance["historical_artifacts_unchanged"] is True

    assert sanity["low_altitude_gate_passed"] is True
    assert sanity["raw_v2_allowed_to_continue"] is True
    assert len(sanity["rows"]) == 11
    assert all(row["gate_passed"] for row in sanity["rows"] if row["altitude_m"] <= 2500.0)
    assert all(row["cold_start_repetitions"] == 3 for row in sanity["rows"])
    assert all(
        row["engine_mixture_telemetry"]["healthy"] for row in sanity["rows"]
    )

    assert len(raw["straight_gates"]) == 13
    assert all(row["status"] == "VALID" for row in raw["straight_gates"])
    assert len(raw["turn_lut"]) == 13 * 9
    assert len(raw["vertical_lut"]) == 13 * 9
    assert len(points["points"]) == result["executed_new_points"] == 221
    assert len(runs["runs"]) == result["executed_new_cold_start_runs"] == 663
    assert len({row["run_id"] for row in runs["runs"]}) == 663
    assert all(len(point["run_ids"]) == 3 for point in points["points"])
    assert all(point["repeatability"]["passed"] for point in points["points"])
    for altitude in (5000.0, 5500.0, 6000.0):
        assert len([row for row in raw["turn_lut"] if row["altitude_m"] == altitude]) == 9
        assert len([row for row in raw["vertical_lut"] if row["altitude_m"] == altitude]) == 9

    assert comparison["reuse_decision"] == "CASE_B_FULL_RAW_RERUN"
    assert comparison["old_points_reused"] == 0
    assert comparison["all_0_to_2500_m_commands_changed"] is True
    assert len(comparison["low_altitude_command_history_audit"]) == 6
    assert all(
        row["command_changed"] and not row["history_identical"]
        for row in comparison["low_altitude_command_history_audit"]
    )
    assert comparison["overlap_altitudes_m"] == [0.0, 1000.0, 2000.0, 2500.0]
    assert len(comparison["straight"]) == 4
    assert len(comparison["turn"]) == 32
    assert len(comparison["vertical"]) == 32

    candidates = {row["altitude_m"]: row for row in ceiling["candidate_assessments"]}
    assert set(candidates) == {5000.0, 5500.0, 6000.0}
    assert candidates[5000.0]["straight_status"] == "VALID"
    assert set(candidates[5000.0]["valid_nonzero_turn_targets_deg"]) == {-10.0, 10.0, 15.0, 20.0, 25.0}
    assert candidates[5000.0]["valid_nonzero_vertical_targets_mps"] == [-2.0]
    assert set(candidates[5500.0]["valid_nonzero_turn_targets_deg"]) == {-10.0, 10.0, 15.0, 20.0}
    assert candidates[5500.0]["valid_nonzero_vertical_targets_mps"] == []
    assert candidates[6000.0]["valid_nonzero_turn_targets_deg"] == []
    assert candidates[6000.0]["valid_nonzero_vertical_targets_mps"] == []
    assert all(row["engine_mixture_telemetry"]["healthy"] for row in candidates.values())
    assert all(row["requirements"]["repeatable"] for row in candidates.values())
    assert all(row["usable"] is False for row in candidates.values())
    assert ceiling["recommended_tested_operational_ceiling_m"] == "NONE"
    assert ceiling["recommendation_is_service_ceiling"] is False

    assert result["step_status"] == "PASS"
    assert result["production_mixture_policy_status"] == "FROZEN"
    assert result["production_stack_v2_status"] == "FROZEN"
    assert result["old_u4_points_reused"] == 0
    assert result["recommended_tested_operational_ceiling_m"] == "NONE"
    assert result["ready_for_u4_1b"] is True
    assert result["historical_artifacts_unchanged"] is True
    assert result["controller_tuning_performed"] is False
    assert result["stock_xml_modified"] is False
    assert result["planner_modified"] is False
    assert result["planner_max_altitude_configuration_changed"] is False
    assert result["interpolation_performed"] is False
    assert result["derating_performed"] is False
    assert result["holdout_validation_started"] is False
    assert result["acceptance_thresholds_modified"] is False
    assert result["u4_1b_started"] is False
    print("U4.1A.3 Production Stack V2 and RAW LUT V2 checks passed")


if __name__ == "__main__":
    main()
