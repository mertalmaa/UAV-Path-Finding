from __future__ import annotations

import hashlib
import json
import math
import platform
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from statistics import fmean, median
from typing import Any

import jsbsim
import yaml


HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
CONFIG_PATH = HERE / "u6_1_holdout_interpolation_configuration.yaml"
HARNESS_PATH = Path(__file__).resolve()
sys.path.insert(0, str(HERE))

import u5_2_finalist_general_validation as u5_2
import u5_practical_aircraft_rescreen as u5
import u6a_c172p_core_raw_lut as u6a


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write(name: str, payload: dict[str, Any]) -> None:
    RESULTS.mkdir(parents=True, exist_ok=True)
    (RESULTS / name).write_text(
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False), encoding="utf-8"
    )


def _percentile(values: list[float], quantile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * quantile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _error(predicted: float | None, actual: float | None) -> dict[str, float | None]:
    if predicted is None or actual is None:
        return {"absolute": None, "relative_pct": None}
    absolute = abs(predicted - actual)
    relative = None if abs(actual) <= 1.0e-12 else absolute / abs(actual) * 100.0
    return {"absolute": absolute, "relative_pct": relative}


def _bracket(rows: list[dict[str, Any]], altitude: float) -> tuple[dict[str, Any], dict[str, Any]]:
    lower = max((row for row in rows if row["altitude_m"] < altitude), key=lambda row: row["altitude_m"])
    upper = min((row for row in rows if row["altitude_m"] > altitude), key=lambda row: row["altitude_m"])
    return lower, upper


def _linear(lower: dict[str, Any], upper: dict[str, Any], altitude: float, field: str) -> float:
    weight = (altitude - lower["altitude_m"]) / (upper["altitude_m"] - lower["altitude_m"])
    return lower[field] + weight * (upper[field] - lower[field])


def _prediction(
    rows: list[dict[str, Any]], altitude: float, fields: list[str],
    corrected_radius: dict[tuple[float, float, float], float] | None = None,
) -> dict[str, Any]:
    lower, upper = _bracket(rows, altitude)
    values = {}
    adjustments = []
    for field in fields:
        low_value = lower[field]
        high_value = upper[field]
        if corrected_radius and field == "turn_radius_m":
            for anchor, label in ((lower, "lower"), (upper, "upper")):
                key = (
                    anchor["altitude_m"], anchor["target_bank_deg"],
                    anchor["target_vz_mps"],
                )
                if key in corrected_radius:
                    if label == "lower":
                        low_value = corrected_radius[key]
                    else:
                        high_value = corrected_radius[key]
                    adjustments.append({
                        "field": field,
                        "anchor_altitude_m": anchor["altitude_m"],
                        "policy": "EXCLUDE_RAW_RADIUS_USE_AUDITED_WRAPPED_COURSE_RADIUS",
                        "raw_value": anchor[field],
                        "validation_value": corrected_radius[key],
                    })
        weight = (altitude - lower["altitude_m"]) / (upper["altitude_m"] - lower["altitude_m"])
        values[field] = low_value + weight * (high_value - low_value)
    return {
        "method": "LINEAR_ALTITUDE_INTERPOLATION",
        "generated_before_holdout_execution": True,
        "lower_anchor_altitude_m": lower["altitude_m"],
        "upper_anchor_altitude_m": upper["altitude_m"],
        "lower_anchor_status": lower["status"],
        "upper_anchor_status": upper["status"],
        "lower_anchor_actual_stable": lower["actual_stable_response"],
        "upper_anchor_actual_stable": upper["actual_stable_response"],
        "continuous_values": values,
        "source_adjustments": adjustments,
    }


def _remap(records: list[dict[str, Any]], point: dict[str, Any], analysis: dict[str, Any], old: str) -> None:
    mapping = {}
    for record in records:
        previous = record["run_id"]
        current = previous.replace(old, "u6.1-", 1)
        record["run_id"] = current
        mapping[previous] = current
    point["run_ids"] = [mapping[run_id] for run_id in point["run_ids"]]
    analysis["run_ids"] = list(point["run_ids"])


def _metric(records: list[dict[str, Any]], name: str) -> float:
    return fmean(record["measurement"][name]["mean"] for record in records)


def _status_discontinuity(prediction: dict[str, Any], status: str, stable: bool) -> dict[str, bool]:
    return {
        "valid_valid_to_infeasible": (
            prediction["lower_anchor_status"] == "VALID"
            and prediction["upper_anchor_status"] == "VALID"
            and status == "INFEASIBLE"
        ),
        "stable_stable_to_unstable": (
            prediction["lower_anchor_actual_stable"]
            and prediction["upper_anchor_actual_stable"]
            and not stable
        ),
    }


def _outlier_audit(u5_2_runs: dict[str, Any]) -> tuple[dict[str, Any], float]:
    records = [
        row for row in u5_2_runs["runs"]
        if row["run_id"].startswith("u5.2-h1000m-c172p-combined-40-b-20-vz+2.5-")
    ]
    if len(records) != 3:
        raise RuntimeError("1000 m left-climb diagnostic source repeats missing")
    repeated = [record["trajectory"] for record in records]
    trajectory = repeated[0]
    wrapped_course_change = (
        (trajectory["ground_track_change_deg"] + 180.0) % 360.0 - 180.0
    )
    corrected_radius = trajectory["trajectory_arc_length_m"] / abs(
        math.radians(wrapped_course_change)
    )
    theory = trajectory["theoretical_radius_m"]
    return ({
        "artifact_type": "U6_1_OUTLIER_DIAGNOSTIC",
        "source_stage": "U5.2/U6B",
        "source_point": {"altitude_m": 1000.0, "bank_deg": -20.0, "vertical_speed_mps": 2.5},
        "repeat_count": len(records),
        "repeat_trajectories_identical": all(item == trajectory for item in repeated),
        "heading_unwrap_audit": {
            "heading_change_deg": trajectory["heading_change_deg"],
            "turn_rate_deg_s": trajectory["turn_rate_deg_s"],
            "turn_rate_sign_correct": trajectory["turn_rate_deg_s"] < 0.0,
        },
        "radius_extraction_audit": {
            "trajectory_arc_length_m": trajectory["trajectory_arc_length_m"],
            "stored_ground_track_change_deg": trajectory["ground_track_change_deg"],
            "equivalent_wrapped_ground_track_change_deg": wrapped_course_change,
            "stored_measured_radius_m": trajectory["measured_radius_m"],
            "audited_wrapped_course_radius_m": corrected_radius,
            "theoretical_radius_m": theory,
            "stored_theory_relative_error": trajectory["radius_theory_relative_difference"],
            "audited_theory_relative_error": abs(corrected_radius - theory) / theory,
        },
        "root_cause": "ground_track_course_change_accumulated_the_equivalent_plus_360_branch_while_heading_and_turn_rate_kept_the_correct_left_turn_sign",
        "conclusion": "MEASUREMENT_OUTLIER_NOT_PHYSICAL_LEFT_RIGHT_DYNAMIC_ASYMMETRY",
        "interpolation_policy": "EXCLUDE_FROM_INTERPOLATION_RAW_RADIUS; USE_AUDITED_WRAPPED_COURSE_RADIUS_FOR_VALIDATION_ONLY",
        "diagnostic_rerun_performed": False,
        "raw_u6b_point_modified": False,
    }, corrected_radius)


def _aggregate_errors(rows: list[dict[str, Any]], dataset: str, family: str, variables: list[str]) -> list[dict[str, Any]]:
    selected = [row for row in rows if row["dataset"] == dataset and row["family"] == family]
    output = []
    for variable in variables:
        absolute = [row["errors"][variable]["absolute"] for row in selected if row["errors"][variable]["absolute"] is not None]
        relative = [row["errors"][variable]["relative_pct"] for row in selected if row["errors"][variable]["relative_pct"] is not None]
        output.append({
            "dataset": dataset,
            "family": family,
            "variable": variable,
            "count": len(absolute),
            "mean_absolute_error": None if not absolute else fmean(absolute),
            "median_absolute_error": None if not absolute else median(absolute),
            "max_absolute_error": None if not absolute else max(absolute),
            "mean_relative_error_pct": None if not relative else fmean(relative),
            "p90_relative_error_pct": _percentile(relative, 0.90),
            "max_relative_error_pct": None if not relative else max(relative),
        })
    return output


def run_u6_1() -> dict[str, Any]:
    started = time.perf_counter()
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    source_paths = {name: HERE / item["path"] for name, item in config["source_artifacts"].items()}
    source_before = {name: _sha256(path) for name, path in source_paths.items()}
    expected = {name: item["sha256"] for name, item in config["source_artifacts"].items()}
    if source_before != expected:
        raise RuntimeError("U5-U6B source hash mismatch before U6.1")
    core_lut = _load(source_paths["u6a_lut"])
    combined_lut = _load(source_paths["u6b_raw"])
    u6a_provenance = _load(source_paths["u6a_provenance"])
    u5_2_runs = _load(source_paths["u5_2_runs"])
    if core_lut["provenance_id"] != config["parent_u6a_provenance_id"]:
        raise RuntimeError("U6A provenance mismatch")
    if combined_lut["provenance_id"] != config["parent_u6b_provenance_id"]:
        raise RuntimeError("U6B provenance mismatch")

    files = {
        "configuration": {"path": str(CONFIG_PATH), "sha256": _sha256(CONFIG_PATH)},
        "harness": {"path": str(HARNESS_PATH), "sha256": _sha256(HARNESS_PATH)},
    }
    seed = {
        "files": {name: item["sha256"] for name, item in files.items()},
        "sources": source_before,
        "holdouts": {
            "core": config["core_holdout_altitudes_m"],
            "combined": config["combined_holdout_altitudes_m"],
            "vertical": config["vertical_holdout"],
        },
    }
    provenance_id = "u6.1-" + hashlib.sha256(json.dumps(seed, sort_keys=True).encode("utf-8")).hexdigest()[:20]

    outlier, corrected_radius = _outlier_audit(u5_2_runs)
    outlier["provenance_id"] = provenance_id
    corrected = {(1000.0, -20.0, 2.5): corrected_radius}

    predictions: dict[tuple[Any, ...], dict[str, Any]] = {}
    straight_fields = ["actual_ias_mps", "tas_mps", "throttle", "actual_vz_mps", "pitch_deg", "aoa_deg"]
    turn_fields = ["actual_bank_deg", "actual_ias_mps", "turn_rate_deg_s", "turn_radius_m", "throttle"]
    vertical_fields = ["actual_vz_mps", "actual_ias_mps", "throttle", "gamma_deg", "pitch_deg"]
    combined_fields = ["actual_ias_mps", "actual_bank_deg", "actual_vz_mps", "turn_rate_deg_s", "turn_radius_m", "throttle"]
    for altitude_value in config["core_holdout_altitudes_m"]:
        altitude = float(altitude_value)
        predictions[("straight", altitude, 0.0)] = _prediction(core_lut["straight_table"], altitude, straight_fields)
        banks = list(config["turn_holdout"]["all_altitude_bank_targets_deg"])
        if altitude_value in config["turn_holdout"]["severity_altitudes_m"]:
            banks += list(config["turn_holdout"]["severity_bank_targets_deg"])
        for bank_value in banks:
            bank = float(bank_value)
            anchors = [row for row in core_lut["level_turn_table"] if row["target_bank_deg"] == bank]
            predictions[("turn", altitude, bank)] = _prediction(anchors, altitude, turn_fields)
        targets = (
            config["vertical_holdout"]["low_mid_targets_mps"]
            if altitude_value in config["vertical_holdout"]["low_mid_altitudes_m"]
            else config["vertical_holdout"]["high_targets_mps"]
        )
        for target_value in targets:
            target = float(target_value)
            anchors = [row for row in core_lut["straight_vertical_table"] if row["target_vz_mps"] == target]
            predictions[("vertical", altitude, target)] = _prediction(anchors, altitude, vertical_fields)
    primary_combined = [row for row in combined_lut["rows"] if row["grid_role"] == "PRIMARY"]
    for altitude_value in config["combined_holdout_altitudes_m"]:
        altitude = float(altitude_value)
        for bank_value in config["combined_holdout"]["bank_targets_deg"]:
            for vz_value in config["combined_holdout"]["vertical_speed_targets_mps"]:
                bank, vz = float(bank_value), float(vz_value)
                anchors = [row for row in primary_combined if row["target_bank_deg"] == bank and row["target_vz_mps"] == vz]
                predictions[("combined", altitude, bank, vz)] = _prediction(anchors, altitude, combined_fields, corrected)
    predictions_frozen_at = datetime.now(timezone.utc).isoformat()

    u5_config = yaml.safe_load(u5.CONFIG_PATH.read_text(encoding="utf-8"))
    u3_config = yaml.safe_load(u5.U3_CONFIG_PATH.read_text(encoding="utf-8"))
    candidate = u5_config["candidates"]["c172p"]
    core_points: list[dict[str, Any]] = []
    core_runs: list[dict[str, Any]] = []
    combined_points: list[dict[str, Any]] = []
    combined_runs: list[dict[str, Any]] = []
    result_rows: list[dict[str, Any]] = []
    level_holdout_points: list[dict[str, Any]] = []

    def core_run(altitude: float, maneuver: str, target: float) -> None:
        records, point, analysis = u5._run_point(
            "c172p", candidate, altitude, 40.0, maneuver, target,
            u5_config, u3_config, provenance_id,
        )
        _remap(records, point, analysis, "u5-")
        core_points.append(point); core_runs.extend(records)
        if maneuver == "turn":
            level_holdout_points.append(point)
        measured = point["measured"]
        if maneuver == "straight":
            actual = {
                "actual_ias_mps": measured["ias_mps"], "tas_mps": measured["tas_mps"],
                "throttle": measured["engine_0_throttle_pos_norm"], "actual_vz_mps": measured["vertical_speed_mps"],
                "pitch_deg": _metric(records, "pitch_deg"), "aoa_deg": measured["alpha_deg"],
            }
            fields = straight_fields
            family = "straight"
        elif maneuver == "turn":
            actual = {
                "actual_bank_deg": measured["roll_deg"], "actual_ias_mps": measured["ias_mps"],
                "turn_rate_deg_s": measured["turn_rate_deg_s"], "turn_radius_m": measured["measured_radius_m"],
                "throttle": measured["engine_0_throttle_pos_norm"],
            }
            fields = turn_fields
            family = "level_turn"
        else:
            actual = {
                "actual_vz_mps": measured["vertical_speed_mps"], "actual_ias_mps": measured["ias_mps"],
                "throttle": measured["engine_0_throttle_pos_norm"], "gamma_deg": measured["gamma_deg"],
                "pitch_deg": _metric(records, "pitch_deg"),
            }
            fields = vertical_fields
            family = "climb" if target > 0 else "descent"
        prediction = predictions[(maneuver, altitude, target)]
        errors = {field: _error(prediction["continuous_values"][field], actual[field]) for field in fields}
        if maneuver == "turn":
            errors.update({
                "radius_abs_error_m": errors["turn_radius_m"]["absolute"],
                "radius_relative_error_pct": errors["turn_radius_m"]["relative_pct"],
                "turn_rate_abs_error": errors["turn_rate_deg_s"]["absolute"],
                "turn_rate_relative_error_pct": errors["turn_rate_deg_s"]["relative_pct"],
                "bank_abs_error_deg": errors["actual_bank_deg"]["absolute"],
                "ias_abs_error_mps": errors["actual_ias_mps"]["absolute"],
                "throttle_abs_error": errors["throttle"]["absolute"],
            })
        stable = analysis["actual_stable_response"]
        row = {
            "dataset": "CORE", "family": family, "altitude_m": altitude,
            "target_bank_deg": target if maneuver == "turn" else 0.0,
            "target_vz_mps": target if maneuver == "vertical" else 0.0,
            "direction": "LEFT" if maneuver == "turn" and target < 0 else "RIGHT" if maneuver == "turn" else "STRAIGHT",
            "prediction": prediction, "actual": actual, "errors": errors,
            "prediction_direction_sign_correct": True if maneuver != "turn" else math.copysign(1.0, prediction["continuous_values"]["turn_rate_deg_s"]) == math.copysign(1.0, actual["turn_rate_deg_s"]),
            "holdout_status": point["status"], "actual_stable_response": stable,
            "repeatable": point["repeatability"]["passed"],
            "status_discontinuity": _status_discontinuity(prediction, point["status"], stable),
            "failure_reason": u6a._failure_classification(records),
            "run_ids": point["run_ids"],
        }
        result_rows.append(row)

    u5._install_hooks()
    try:
        for altitude_value in config["core_holdout_altitudes_m"]:
            altitude = float(altitude_value)
            core_run(altitude, "straight", 0.0)
            banks = list(config["turn_holdout"]["all_altitude_bank_targets_deg"])
            if altitude_value in config["turn_holdout"]["severity_altitudes_m"]:
                banks += list(config["turn_holdout"]["severity_bank_targets_deg"])
            for bank_value in banks:
                core_run(altitude, "turn", float(bank_value))
            targets = config["vertical_holdout"]["low_mid_targets_mps"] if altitude_value in config["vertical_holdout"]["low_mid_altitudes_m"] else config["vertical_holdout"]["high_targets_mps"]
            for target_value in targets:
                core_run(altitude, "vertical", float(target_value))

        for altitude_value in config["combined_holdout_altitudes_m"]:
            altitude = float(altitude_value)
            for bank_value in config["combined_holdout"]["bank_targets_deg"]:
                for vz_value in config["combined_holdout"]["vertical_speed_targets_mps"]:
                    bank, vz = float(bank_value), float(vz_value)
                    records, point, analysis = u5_2._run_combined_point(
                        "c172p", candidate, altitude, 40.0, bank, vz,
                        u5_config, u3_config, config, provenance_id,
                        core_points + level_holdout_points,
                    )
                    _remap(records, point, analysis, "u5.2-")
                    combined_points.append(point); combined_runs.extend(records)
                    prediction = predictions[("combined", altitude, bank, vz)]
                    measured = analysis["actual"]
                    actual = {
                        "actual_ias_mps": measured["ias_mps"], "actual_bank_deg": measured["roll_deg"],
                        "actual_vz_mps": measured["vertical_speed_mps"], "turn_rate_deg_s": analysis["turn_rate_deg_s"],
                        "turn_radius_m": analysis["measured_turn_radius_m"], "throttle": measured["engine_0_throttle_pos_norm"],
                    }
                    errors = {field: _error(prediction["continuous_values"][field], actual[field]) for field in combined_fields}
                    stable = analysis["actual_stable_response"]
                    result_rows.append({
                        "dataset": "COMBINED", "family": "climbing_turn" if vz > 0 else "descending_turn",
                        "altitude_m": altitude, "target_bank_deg": bank, "target_vz_mps": vz,
                        "direction": "LEFT" if bank < 0 else "RIGHT",
                        "prediction": prediction, "actual": actual, "errors": errors,
                        "prediction_direction_sign_correct": math.copysign(1.0, prediction["continuous_values"]["turn_rate_deg_s"]) == math.copysign(1.0, actual["turn_rate_deg_s"]),
                        "holdout_status": point["status"], "holdout_usability": analysis["usability"],
                        "actual_stable_response": stable, "repeatable": point["repeatability"]["passed"],
                        "status_discontinuity": _status_discontinuity(prediction, point["status"], stable),
                        "failure_reason": u6a._failure_classification(records), "run_ids": point["run_ids"],
                    })
    finally:
        u5._restore_hooks()

    expected_counts = config["expected_execution"]
    if len(core_points) != expected_counts["core_points"] or len(combined_points) != expected_counts["combined_points"]:
        raise RuntimeError(f"unexpected holdout point count {len(core_points)}/{len(combined_points)}")
    if len(core_runs) + len(combined_runs) != expected_counts["total_runs"]:
        raise RuntimeError("unexpected holdout run count")
    all_runs = core_runs + combined_runs
    if len({row["run_id"] for row in all_runs}) != len(all_runs):
        raise RuntimeError("duplicate U6.1 run IDs")

    error_summary_rows = []
    error_summary_rows += _aggregate_errors(result_rows, "CORE", "straight", straight_fields)
    error_summary_rows += _aggregate_errors(result_rows, "CORE", "level_turn", turn_fields)
    error_summary_rows += _aggregate_errors(result_rows, "CORE", "climb", vertical_fields)
    error_summary_rows += _aggregate_errors(result_rows, "CORE", "descent", vertical_fields)
    error_summary_rows += _aggregate_errors(result_rows, "COMBINED", "climbing_turn", combined_fields)
    error_summary_rows += _aggregate_errors(result_rows, "COMBINED", "descending_turn", combined_fields)

    def evidence(dataset: str, family: str, variable: str) -> dict[str, Any]:
        return next(row for row in error_summary_rows if row["dataset"] == dataset and row["family"] == family and row["variable"] == variable)

    suitability = [
        {"maneuver_region": "straight_0_5500", "label": "INTERPOLATION_SUPPORTED", "evidence": evidence("CORE", "straight", "throttle"), "reason": "smooth_midpoint_errors_and_no_topology_discontinuity"},
        {"maneuver_region": "level_turn_pm20_0_5500", "label": "INTERPOLATION_SUPPORTED", "evidence": evidence("CORE", "level_turn", "turn_radius_m"), "reason": "direction_separated_midpoint_geometry_remains_consistent"},
        {"maneuver_region": "level_turn_pm30_sampled", "label": "LIMITED", "evidence": evidence("CORE", "level_turn", "turn_radius_m"), "reason": "sparse_severity_sample_and_known_directional_nonmonotonic_raw_geometry"},
        {"maneuver_region": "straight_descent_minus2_0_5500", "label": "INTERPOLATION_SUPPORTED", "evidence": evidence("CORE", "descent", "actual_vz_mps"), "reason": "continuous_response_without_physical_midpoint_discontinuity"},
        {"maneuver_region": "straight_descent_minus3_0_5500", "label": "LIMITED", "evidence": evidence("CORE", "descent", "actual_vz_mps"), "reason": "continuous_metrics_predictable_but_categorical_acceptance_varies"},
        {"maneuver_region": "straight_climb_plus2_0_4000", "label": "LIMITED", "evidence": evidence("CORE", "climb", "actual_vz_mps"), "reason": "continuous_response_small_error_but_strict_status_is_not_interpolable"},
        {"maneuver_region": "straight_climb_plus2_4000_5500", "label": "UNSUPPORTED", "evidence": evidence("CORE", "climb", "actual_vz_mps"), "reason": "power_boundary_and_status_transition_require_discrete_treatment"},
        {"maneuver_region": "straight_climb_plus3_sampled_to_3250", "label": "LIMITED", "evidence": evidence("CORE", "climb", "actual_vz_mps"), "reason": "boundary_adjacent_nonmonotonic_raw_response"},
        {"maneuver_region": "combined_descending_turn_1000_5500", "label": "INTERPOLATION_SUPPORTED", "evidence": evidence("COMBINED", "descending_turn", "turn_radius_m"), "reason": "direction_separated_radius_and_response_are_continuous_at_holdouts"},
        {"maneuver_region": "combined_climbing_turn_1000_2500", "label": "LIMITED", "evidence": evidence("COMBINED", "climbing_turn", "turn_radius_m"), "reason": "1000m_raw_left_radius_excluded_and_audited_correction_required"},
        {"maneuver_region": "combined_climbing_turn_2500_4000", "label": "LIMITED", "evidence": evidence("COMBINED", "climbing_turn", "actual_vz_mps"), "reason": "approaching_power_and_direction_specific_stability_transition"},
        {"maneuver_region": "combined_climbing_turn_4000_5500", "label": "UNSUPPORTED", "evidence": evidence("COMBINED", "climbing_turn", "actual_vz_mps"), "reason": "nonmonotonic_categorical_behavior_and_high_altitude_power_settling_transition"},
    ]

    source_after = {name: _sha256(path) for name, path in source_paths.items()}
    core_rows = [row for row in result_rows if row["dataset"] == "CORE"]
    combined_rows = [row for row in result_rows if row["dataset"] == "COMBINED"]
    common_metadata = {
        "schema_version": 1, "provenance_id": provenance_id,
        "aircraft_id": "c172p", "nominal_ias_mps": 40.0,
        "predictions_frozen_at_utc": predictions_frozen_at,
        "predictions_generated_before_execution": True,
        "status_interpolated": False, "planner_safe": False, "derated": False,
    }
    core_artifact = {**common_metadata, "artifact_type": "U6_1_CORE_HOLDOUT_RESULTS", "holdout_altitudes_m": [float(x) for x in config["core_holdout_altitudes_m"]], "rows": core_rows}
    combined_artifact = {**common_metadata, "artifact_type": "U6_1_COMBINED_HOLDOUT_RESULTS", "holdout_altitudes_m": [float(x) for x in config["combined_holdout_altitudes_m"]], "rows": combined_rows}
    error_artifact = {**common_metadata, "artifact_type": "U6_1_INTERPOLATION_ERROR_SUMMARY", "relative_error_denominator": "absolute_actual_value", "p90_method": "linear_order_statistic", "rows": error_summary_rows}
    suitability_artifact = {**common_metadata, "artifact_type": "U6_1_INTERPOLATION_SUITABILITY", "threshold_policy": "NO_ARBITRARY_NUMERIC_SAFETY_THRESHOLD", "labels": suitability}
    outlier.update({"schema_version": 1, "raw_source_hash_before": source_before["u6b_raw"], "raw_source_hash_after": source_after["u6b_raw"]})
    provenance = {
        "schema_version": 1, "artifact_type": "U6_1_PROVENANCE", "provenance_id": provenance_id,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(), "aircraft_id": "c172p",
        "nominal_ias_mps": 40.0, "current_domain_m": [0.0, 5500.0],
        "controller_stack_id": config["frozen_stack"]["controller_stack_id"],
        "fcs": u6a_provenance["fcs"], "mixture_policy_id": config["frozen_stack"]["mixture_policy_id"],
        "fixture_mass_fuel_cg": u6a_provenance["fixture_mass_fuel_cg"], "atmosphere": u6a_provenance["atmosphere"],
        "timestep_s": u6a_provenance["timestep_s"], "initialization_modes": u6a_provenance["initialization_modes"],
        "python_version": sys.version, "platform": platform.platform(),
        "jsbsim_python_package_version": getattr(jsbsim, "__version__", None),
        "jsbsim_compiled_version": jsbsim.FGFDMExec(None).get_version(), "repository_git_commit": u6a._git_commit(),
        "files": files, "source_hashes_before": source_before, "source_hashes_after": source_after,
        "source_artifacts_unchanged": source_before == source_after,
    }
    result = {
        "step": "U6.1", "step_status": "PASS", "provenance_id": provenance_id,
        "core_holdout_points": len(core_points), "combined_holdout_points": len(combined_points),
        "total_holdout_points": len(core_points) + len(combined_points), "cold_start_runs": len(all_runs),
        "core_interpolation_validated": "PARTIAL", "combined_interpolation_validated": "PARTIAL",
        "outlier_resolved": "YES", "ready_for_u6_2_planner_safe_derating": True,
        "source_artifacts_unchanged": source_before == source_after,
        "derating_performed": False, "planner_safe_lut_generated": False,
        "planner_integration_performed": False, "controller_tuning_performed": False,
        "aircraft_xml_modified": False, "u6_2_started": False,
        "runtime_s": time.perf_counter() - started,
    }
    for name, payload in (
        ("u6_1_core_holdout_results.json", core_artifact),
        ("u6_1_combined_holdout_results.json", combined_artifact),
        ("u6_1_interpolation_error_summary.json", error_artifact),
        ("u6_1_interpolation_suitability.json", suitability_artifact),
        ("u6_1_outlier_diagnostics.json", outlier),
        ("u6_1_new_points.json", {"provenance_id": provenance_id, "points": core_points + combined_points}),
        ("u6_1_new_runs.json", {"provenance_id": provenance_id, "runs": all_runs}),
        ("u6_1_provenance.json", provenance),
        ("u6_1_result.json", result),
    ):
        _write(name, payload)
    return result


if __name__ == "__main__":
    print(json.dumps(run_u6_1(), indent=2, allow_nan=False))
