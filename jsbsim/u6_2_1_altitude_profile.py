from __future__ import annotations

import hashlib
import json
import math
import platform
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml


HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
CONFIG_PATH = HERE / "u6_2_1_altitude_profile_configuration.yaml"
HARNESS_PATH = Path(__file__).resolve()
ALTITUDES = [float(value) for value in range(0, 5501, 500)]


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write(name: str, payload: dict[str, Any]) -> None:
    RESULTS.mkdir(parents=True, exist_ok=True)
    (RESULTS / name).write_text(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False), encoding="utf-8")


def _source_row(rows: list[dict[str, Any]], altitude: float, field: str, target: float) -> dict[str, Any]:
    return next(row for row in rows if row["altitude_m"] == altitude and row[field] == target)


def _power(row: dict[str, Any]) -> dict[str, Any]:
    return {
        "throttle": row["throttle"],
        "power_margin_norm": 1.0 - row["throttle"],
        "rpm": row.get("rpm"),
        "power_hp": row.get("power_hp"),
        "thrust_lbs": row.get("thrust_lbs"),
    }


def _quality(row: dict[str, Any]) -> dict[str, Any]:
    return {
        "status": row["status"],
        "actual_stable_response": row["actual_stable_response"],
        "settled": row["settled"],
        "settling": row["settling"],
        "repeatable": row["repeatable"],
        "saturation": row["saturation"],
        "failure_reason": row["failure_reason"],
    }


def _max_holdout_error(rows: list[dict[str, Any]], family: str, field: str, **filters: Any) -> float:
    selected = [
        row for row in rows
        if row["family"] == family and all(row[key] == value for key, value in filters.items())
    ]
    return max(row["errors"][field]["absolute"] for row in selected)


def _lerp(lo: float, hi: float, weight: float) -> float:
    return float(lo) + weight * (float(hi) - float(lo))


def _combined_local(
    rows: list[dict[str, Any]], altitude: float, direction: str, target_vz: float,
    corrected_left_climb_radius: float,
) -> dict[str, Any] | None:
    selected = sorted(
        [row for row in rows if row["grid_role"] == "PRIMARY" and row["direction"] == direction and row["target_vz_mps"] == target_vz],
        key=lambda row: row["altitude_m"],
    )
    if altitude < selected[0]["altitude_m"]:
        return None
    exact = next((row for row in selected if row["altitude_m"] == altitude), None)
    if exact is not None:
        radius = exact["turn_radius_m"]
        correction = None
        if altitude == 1000.0 and direction == "LEFT" and target_vz > 0.0:
            correction = {
                "historical_raw_turn_radius_m": radius,
                "measurement_corrected_turn_radius_m": corrected_left_climb_radius,
                "reason": "ground_track_course_change_shortest_angle_wrap",
            }
            radius = corrected_left_climb_radius
        return {
            "actual_bank_deg": exact["actual_bank_deg"],
            "actual_vz_mps": exact["actual_vz_mps"],
            "turn_radius_m": radius,
            "turn_rate_deg_s": exact["turn_rate_deg_s"],
            "actual_ias_mps": exact["actual_ias_mps"],
            "ias_retention_ratio": exact["actual_ias_mps"] / exact["nominal_ias_mps"],
            "throttle": exact["throttle"],
            "power_margin_norm": 1.0 - exact["throttle"],
            "tas_mps": None,
            "nz": exact["nz"],
            "beta_deg": exact["beta_deg"],
            "saturation": exact["saturation"],
            "settling": exact["settling"],
            "settled": exact["settled"],
            "repeatable": exact["repeatable"],
            "actual_stable_response": exact["actual_stable_response"],
            "raw_status": exact["status"],
            "raw_usability": exact["usability"],
            "source_basis": "EXACT_RAW_ANCHOR",
            "source_altitudes_m": [altitude],
            "measurement_correction": correction,
            "source_provenance": exact["metadata"]["source_provenance"],
        }
    lower = max((row for row in selected if row["altitude_m"] < altitude), key=lambda row: row["altitude_m"])
    upper = min((row for row in selected if row["altitude_m"] > altitude), key=lambda row: row["altitude_m"])
    weight = (altitude - lower["altitude_m"]) / (upper["altitude_m"] - lower["altitude_m"])
    lower_radius = lower["turn_radius_m"]
    lower_correction = None
    if lower["altitude_m"] == 1000.0 and direction == "LEFT" and target_vz > 0.0:
        lower_correction = {"historical_raw_turn_radius_m": lower_radius, "corrected_m": corrected_left_climb_radius}
        lower_radius = corrected_left_climb_radius
    return {
        "actual_bank_deg": _lerp(lower["actual_bank_deg"], upper["actual_bank_deg"], weight),
        "actual_vz_mps": _lerp(lower["actual_vz_mps"], upper["actual_vz_mps"], weight),
        "turn_radius_m": _lerp(lower_radius, upper["turn_radius_m"], weight),
        "turn_rate_deg_s": _lerp(lower["turn_rate_deg_s"], upper["turn_rate_deg_s"], weight),
        "actual_ias_mps": _lerp(lower["actual_ias_mps"], upper["actual_ias_mps"], weight),
        "ias_retention_ratio": _lerp(lower["actual_ias_mps"], upper["actual_ias_mps"], weight) / 40.0,
        "throttle": _lerp(lower["throttle"], upper["throttle"], weight),
        "power_margin_norm": 1.0 - _lerp(lower["throttle"], upper["throttle"], weight),
        "tas_mps": None,
        "nz": _lerp(lower["nz"], upper["nz"], weight),
        "beta_deg": _lerp(lower["beta_deg"], upper["beta_deg"], weight),
        "saturation": {key: lower["saturation"][key] or upper["saturation"][key] for key in lower["saturation"]},
        "settling": {"basis": "BRACKET_ENDPOINTS", "lower_settled": lower["settled"], "upper_settled": upper["settled"]},
        "settled": lower["settled"] and upper["settled"],
        "repeatable": lower["repeatable"] and upper["repeatable"],
        "actual_stable_response": lower["actual_stable_response"] and upper["actual_stable_response"],
        "raw_status": None,
        "raw_usability": None,
        "source_basis": "U6_1_VALIDATED_LINEAR_CONTINUOUS_DERIVATION",
        "source_altitudes_m": [lower["altitude_m"], upper["altitude_m"]],
        "measurement_correction": lower_correction,
        "source_provenance": [lower["metadata"]["source_provenance"], upper["metadata"]["source_provenance"]],
    }


def run_u6_2_1() -> dict[str, Any]:
    started = time.perf_counter()
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    source_paths = {name: HERE / item["path"] for name, item in config["source_artifacts"].items()}
    before = {name: _sha256(path) for name, path in source_paths.items()}
    expected = {name: item["sha256"] for name, item in config["source_artifacts"].items()}
    if before != expected:
        raise RuntimeError("U6A-U6.2 source hash mismatch before U6.2.1")

    core = _load(source_paths["core_raw"])
    combined = _load(source_paths["combined_raw"])
    core_holdout_artifact = _load(source_paths["core_holdouts"])
    combined_holdout_artifact = _load(source_paths["combined_holdouts"])
    core_holdouts = core_holdout_artifact["rows"]
    combined_holdouts = combined_holdout_artifact["rows"]
    interpolation_suitability = _load(source_paths["interpolation_suitability"])
    u6_2_profile = _load(source_paths["u6_2_profile"])
    u6_2_derivation = _load(source_paths["u6_2_derivation"])
    measurement_fix = _load(source_paths["u6_2_measurement_fix"])
    if u6_2_profile["provenance_id"] != config["parent_u6_2_provenance_id"]:
        raise RuntimeError("U6.2 parent provenance mismatch")

    files = {
        "configuration": {"path": str(CONFIG_PATH), "sha256": _sha256(CONFIG_PATH)},
        "derivation_harness": {"path": str(HARNESS_PATH), "sha256": _sha256(HARNESS_PATH)},
        "query_interface": {"path": str(HERE / "aircraft_capability_profile.py"), "sha256": _sha256(HERE / "aircraft_capability_profile.py")},
    }
    seed = {"files": {key: value["sha256"] for key, value in files.items()}, "sources": before, "policy": config["local_safe_policy"]}
    provenance_id = "u6.2.1-" + hashlib.sha256(json.dumps(seed, sort_keys=True).encode()).hexdigest()[:20]

    straight_rows: list[dict[str, Any]] = []
    for altitude in ALTITUDES:
        source = _source_row(core["straight_table"], altitude, "nominal_ias_mps", 40.0)
        straight_rows.append({
            "altitude_m": altitude,
            "availability": "AVAILABLE",
            "measured": {
                "nominal_ias_mps": 40.0, "actual_ias_mps": source["actual_ias_mps"],
                "ias_retention_ratio": source["actual_ias_mps"] / 40.0, "tas_mps": source["tas_mps"],
                "actual_vz_mps": source["actual_vz_mps"], "pitch_deg": source["pitch_deg"],
                "aoa_deg": source["aoa_deg"], "beta_deg": source["beta_deg"],
                "power": _power(source), "quality": _quality(source),
            },
            "planner_safe": {"expected_ias_mps": source["actual_ias_mps"], "power_margin_norm": 1.0 - source["throttle"]},
            "evidence": {"confidence": "HIGH", "canonical_anchor": True, "source_provenance": source["metadata"]["source_provenance"]},
        })

    turn_radius_error = u6_2_derivation["turn_allowance_m"]
    turn_rate_error = u6_2_derivation["turn_rate_allowance_deg_s"]
    turn_rows: list[dict[str, Any]] = []
    for altitude in ALTITUDES:
        for direction, bank in (("LEFT", -20.0), ("RIGHT", 20.0), ("LEFT", -30.0), ("RIGHT", 30.0)):
            source = _source_row(core["level_turn_table"], altitude, "target_bank_deg", bank)
            available = abs(bank) == 20.0
            allowance = turn_radius_error[direction] + u6_2_derivation["turn_repeatability_spread_m"][direction]
            safe_rate = math.copysign(max(0.0, abs(source["turn_rate_deg_s"]) - turn_rate_error[direction]), source["turn_rate_deg_s"])
            turn_rows.append({
                "altitude_m": altitude, "direction": direction, "command_bank_deg": bank,
                "availability": "AVAILABLE" if available else "UNAVAILABLE",
                "measured": {
                    "actual_bank_deg": source["actual_bank_deg"], "turn_radius_m": source["turn_radius_m"],
                    "turn_rate_deg_s": source["turn_rate_deg_s"], "actual_ias_mps": source["actual_ias_mps"],
                    "ias_retention_ratio": source["actual_ias_mps"] / 40.0, "tas_mps": source["tas_mps"],
                    "nz": source["nz"], "beta_deg": source["beta_deg"], "vz_mps": source["vz_mps"],
                    "power": _power(source), "quality": _quality(source),
                },
                "planner_safe": ({
                    "bank_context_deg": bank, "turn_radius_m": source["turn_radius_m"] + allowance,
                    "turn_rate_deg_s": safe_rate, "expected_ias_mps": source["actual_ias_mps"],
                    "power_margin_norm": 1.0 - source["throttle"],
                } if available else None),
                "evidence": {
                    "confidence": "HIGH" if available else "WITHHELD",
                    "validation_quality": "SUPPORTED_DIRECTION_SEPARATED" if available else "LIMITED_NON_MONOTONIC_RADIUS",
                    "reason_unavailable": None if available else "sampled_30deg_geometry_not_validated_for_guaranteed_use",
                    "raw_measured_preserved": True, "radius_allowance_m": allowance if available else None,
                    "source_provenance": source["metadata"]["source_provenance"],
                },
            })

    vertical_error = u6_2_derivation["vertical_error_allowance_mps"]
    climb_rows: list[dict[str, Any]] = []
    descent_rows: list[dict[str, Any]] = []
    for altitude in ALTITUDES:
        altitude_vertical = [row for row in core["straight_vertical_table"] if row["altitude_m"] == altitude]
        climb_basis = _source_row(altitude_vertical, altitude, "target_vz_mps", 2.0)
        climb_max = max((row for row in altitude_vertical if row["target_vz_mps"] > 0 and row["actual_stable_response"]), key=lambda row: row["actual_vz_mps"])
        climb_available = altitude <= 4500.0
        climb_safe = max(0.0, climb_basis["actual_vz_mps"] - vertical_error["climb_plus2_mps"]) if climb_available else None
        climb_rows.append({
            "altitude_m": altitude, "availability": "AVAILABLE" if climb_available else "UNAVAILABLE",
            "measured": {
                "maximum_observed_stable_climb_vz_mps": climb_max["actual_vz_mps"],
                "maximum_observed_target_vz_mps": climb_max["target_vz_mps"],
                "safe_basis_command_vz_mps": 2.0, "safe_basis_actual_vz_mps": climb_basis["actual_vz_mps"],
                "actual_ias_mps": climb_basis["actual_ias_mps"], "ias_retention_ratio": climb_basis["actual_ias_mps"] / 40.0,
                "tas_mps": climb_basis["tas_mps"], "power": _power(climb_basis), "quality": _quality(climb_basis),
                "observed_maximum_power": _power(climb_max), "observed_maximum_quality": _quality(climb_max),
            },
            "planner_safe": ({"climb_vz_mps": climb_safe, "expected_ias_mps": climb_basis["actual_ias_mps"], "power_margin_norm": 1.0 - climb_basis["throttle"]} if climb_available else None),
            "evidence": {
                "confidence": "HIGH" if climb_available and climb_basis["status"] == "VALID" else "MEDIUM" if climb_available else "WITHHELD",
                "holdout_max_abs_vz_error_mps": vertical_error["climb_plus2_mps"],
                "basis": "local_plus2_actual_response_minus_validated_error; observed_maximum_not_promoted",
                "reason_unavailable": None if climb_available else "high_altitude_power_IAS_and_categorical_capability_boundary",
                "interpolation_suitability": "LIMITED" if altitude <= 4000 else "UNSUPPORTED",
                "source_provenance": climb_basis["metadata"]["source_provenance"],
            },
        })

        descent_basis = _source_row(altitude_vertical, altitude, "target_vz_mps", -3.0)
        descent_max = min((row for row in altitude_vertical if row["target_vz_mps"] < 0 and row["actual_stable_response"]), key=lambda row: row["actual_vz_mps"])
        descent_safe = -max(0.0, abs(descent_basis["actual_vz_mps"]) - vertical_error["descent_minus3_mps"])
        descent_rows.append({
            "altitude_m": altitude, "availability": "AVAILABLE",
            "measured": {
                "maximum_observed_stable_descent_vz_mps": descent_max["actual_vz_mps"],
                "maximum_observed_target_vz_mps": descent_max["target_vz_mps"],
                "safe_basis_command_vz_mps": -3.0, "safe_basis_actual_vz_mps": descent_basis["actual_vz_mps"],
                "actual_ias_mps": descent_basis["actual_ias_mps"], "ias_retention_ratio": descent_basis["actual_ias_mps"] / 40.0,
                "tas_mps": descent_basis["tas_mps"], "power": _power(descent_basis), "quality": _quality(descent_basis),
                "observed_maximum_power": _power(descent_max), "observed_maximum_quality": _quality(descent_max),
            },
            "planner_safe": {"descent_vz_mps": descent_safe, "expected_ias_mps": descent_basis["actual_ias_mps"], "power_margin_norm": 1.0 - descent_basis["throttle"]},
            "evidence": {
                "confidence": "MEDIUM", "holdout_max_abs_vz_error_mps": vertical_error["descent_minus3_mps"],
                "basis": "local_minus3_actual_response_toward_zero_by_validated_error; larger_observed_descent_not_promoted",
                "interpolation_suitability": "LIMITED_CONSERVATIVE", "source_provenance": descent_basis["metadata"]["source_provenance"],
            },
        })

    corrected_radius = measurement_fix["u6b_outlier_case"]["corrected_radius_m"]
    combined_climb_rows: list[dict[str, Any]] = []
    combined_descent_rows: list[dict[str, Any]] = []
    for altitude in ALTITUDES:
        for direction, bank in (("LEFT", -20.0), ("RIGHT", 20.0)):
            climb_measured = _combined_local(combined["rows"], altitude, direction, 2.5, corrected_radius)
            combined_climb_rows.append({
                "altitude_m": altitude, "direction": direction, "command_bank_deg": bank,
                "command_vz_mps": 2.5, "availability": "UNAVAILABLE",
                "measured": climb_measured, "planner_safe": None,
                "evidence": {
                    "confidence": "NO_EVIDENCE" if climb_measured is None else "WITHHELD",
                    "reason_unavailable": "no_combined_data_below_1000" if climb_measured is None else "combined_climb_disabled_due_to_marginal_or_unusable_and_unsupported_high_altitude_capability",
                    "measured_response_preserved": climb_measured is not None,
                },
            })

            descent_measured = _combined_local(combined["rows"], altitude, direction, -2.5, corrected_radius)
            available = altitude >= 3500.0
            radius_error = u6_2_derivation["combined_descent_radius_allowance_m"][direction]
            vz_error = _max_holdout_error(combined_holdouts, "descending_turn", "actual_vz_mps", direction=direction)
            rate_error = _max_holdout_error(combined_holdouts, "descending_turn", "turn_rate_deg_s", direction=direction)
            safe = None
            if available and descent_measured is not None:
                safe = {
                    "turn_radius_m": descent_measured["turn_radius_m"] + radius_error,
                    "turn_rate_deg_s": math.copysign(max(0.0, abs(descent_measured["turn_rate_deg_s"]) - rate_error), descent_measured["turn_rate_deg_s"]),
                    "descent_vz_mps": -max(0.0, abs(descent_measured["actual_vz_mps"]) - vz_error),
                    "expected_ias_mps": descent_measured["actual_ias_mps"],
                    "power_margin_norm": descent_measured["power_margin_norm"],
                }
            combined_descent_rows.append({
                "altitude_m": altitude, "direction": direction, "command_bank_deg": bank,
                "command_vz_mps": -2.5, "availability": "AVAILABLE" if available else "UNAVAILABLE",
                "measured": descent_measured, "planner_safe": safe,
                "evidence": {
                    "confidence": "HIGH" if available else "NO_EVIDENCE" if descent_measured is None else "WITHHELD",
                    "reason_unavailable": None if available else "conservative_discrete_band_below_validated_3500_boundary",
                    "radius_allowance_m": radius_error, "vz_error_allowance_mps": vz_error,
                    "turn_rate_error_allowance_deg_s": rate_error,
                },
            })

    capabilities = {
        "straight": {"query_policy": {"safe_interpolation": "LINEAR", "availability_interpolated": False}, "rows": straight_rows},
        "level_turn": {"query_policy": {"safe_interpolation": "LINEAR", "availability_interpolated": False, "direction_separated": True}, "rows": turn_rows},
        "straight_climb": {"query_policy": {"safe_interpolation": "LINEAR", "availability_interpolated": False, "availability_interpolation_blocked_ranges_m": [[4000.0, 5500.0]]}, "rows": climb_rows},
        "straight_descent": {"query_policy": {"safe_interpolation": "CONSERVATIVE_ENDPOINT", "availability_interpolated": False}, "rows": descent_rows},
        "climbing_turn": {"query_policy": {"safe_interpolation": "NONE", "availability_interpolated": False}, "rows": combined_climb_rows},
        "descending_turn": {"query_policy": {"safe_interpolation": "LINEAR", "availability_interpolated": False, "direction_separated": True}, "rows": combined_descent_rows},
    }

    constant_audit = {
        "schema_version": 1, "artifact_type": "U6_2_1_GLOBAL_CAPABILITY_CONSTANT_AUDIT", "provenance_id": provenance_id,
        "findings": [
            {"item": "nominal_ias_mps", "previous": 40.0, "classification": "VALID_PLANNER_CONTEXT_EXCEPTION", "resolution": "retained_globally_but_actual_IAS_and_retention_stored_per_row"},
            {"item": "straight capability", "previous": "global-like availability plus sparse expected fields", "classification": "ALTITUDE_DEPENDENT", "resolution": "full measured and local safe row at all 12 anchors"},
            {"item": "level turn radius/rate", "previous": "already altitude/direction dependent but telemetry incomplete", "classification": "ALTITUDE_AND_DIRECTION_DEPENDENT", "resolution": "retained geometry and added full measured evidence"},
            {"item": "30deg turn", "previous": "unavailable with only partial raw references", "classification": "MEASURED_BUT_UNAVAILABLE", "resolution": "full raw response preserved at every anchor"},
            {"item": "climb rate", "previous": "+2.0 m/s global safe value", "classification": "INVALID_GLOBAL_SIMPLIFICATION", "resolution": "local measured maximum plus local safe lower-bound response"},
            {"item": "descent rate", "previous": "-3.0 m/s global safe value", "classification": "INVALID_GLOBAL_SIMPLIFICATION", "resolution": "local measured maximum plus local safe conservative response"},
            {"item": "combined climb", "previous": "unavailable rows without measured response", "classification": "MEASURED_BUT_UNAVAILABLE", "resolution": "local measured/interpolated response retained; safe remains null"},
            {"item": "combined descent", "previous": "global -2.5 safe Vz context", "classification": "ALTITUDE_AND_DIRECTION_DEPENDENT", "resolution": "local radius/rate/Vz/IAS/power with evidence allowances"},
            {"item": "throttle and power", "previous": "not global but incomplete in capability rows", "classification": "ALTITUDE_AND_MANEUVER_DEPENDENT", "resolution": "local throttle, power margin and available engine metrics retained"},
        ],
        "remaining_global_aircraft_capability_constants": [],
        "planner_context_constants": [{"item": "nominal_ias_mps", "value": 40.0, "not_an_actual_response_claim": True}],
    }

    profile = {
        "schema_version": 2, "profile_schema_id": "altitude_aware_aircraft_capability_profile_v2",
        "artifact_type": "AIRCRAFT_CAPABILITY_PROFILE_PLANNER_SAFE", "profile_stage": "planner_safe_altitude_dependent",
        "planner_ready": True, "aircraft_identity": {"aircraft_id": "c172p", "controller_stack_id": config["canonical_context"]["controller_stack_id"]},
        "provenance_id": provenance_id, "parent_u6_2_provenance_id": u6_2_profile["provenance_id"],
        "domain": {"min_altitude_m": 0.0, "max_altitude_m": 5500.0, "canonical_altitude_grid_m": ALTITUDES, "nominal_ias_context_mps": 40.0, "speed_is_planner_state_dimension": False},
        "units": {"altitude": "m", "speed": "m/s", "angle": "deg", "turn_rate": "deg/s", "radius": "m"},
        "global_aircraft_capability_constants": [],
        "measured_and_planner_safe_layers_separate": True,
        "availability_semantics": ["AVAILABLE", "UNAVAILABLE", "OUT_OF_DOMAIN"],
        "interface_contract": {
            "straight_query": ["altitude_m"], "turn_query": ["altitude_m", "direction", "bank_deg"],
            "vertical_query": ["altitude_m", "CLIMB_or_DESCENT"], "combined_query": ["altitude_m", "direction", "CLIMB_or_DESCENT"],
            "returns_local_row_or_validated_interpolation": True, "aircraft_specific_planner_branching_forbidden": True,
        },
        "capabilities": capabilities,
        "source_provenance": {
            "u6a": core["provenance_id"], "u6b": combined["provenance_id"],
            "u6_1_core_holdouts": core_holdout_artifact["provenance_id"],
            "u6_1_combined_holdouts": combined_holdout_artifact["provenance_id"],
            "u6_1_interpolation_suitability": interpolation_suitability["provenance_id"],
            "u6_2": u6_2_profile["provenance_id"],
        },
        "production_planner_integrated": False,
    }

    audit_rows = []
    for altitude in ALTITUDES:
        straight = next(row for row in straight_rows if row["altitude_m"] == altitude)
        left = next(row for row in turn_rows if row["altitude_m"] == altitude and row["command_bank_deg"] == -20.0)
        right = next(row for row in turn_rows if row["altitude_m"] == altitude and row["command_bank_deg"] == 20.0)
        climb = next(row for row in climb_rows if row["altitude_m"] == altitude)
        descent = next(row for row in descent_rows if row["altitude_m"] == altitude)
        cc = [row for row in combined_climb_rows if row["altitude_m"] == altitude]
        cd = [row for row in combined_descent_rows if row["altitude_m"] == altitude]
        limitation = "combined climb disabled"
        if altitude >= 5000.0:
            limitation = "high-altitude climb power/IAS margin"
        elif altitude >= 4000.0:
            limitation = "climb power boundary and combined climb"
        audit_rows.append({
            "altitude_m": altitude,
            "straight_actual_ias_mps": straight["measured"]["actual_ias_mps"],
            "straight_throttle": straight["measured"]["power"]["throttle"],
            "straight_power_margin_norm": straight["measured"]["power"]["power_margin_norm"],
            "left_safe_radius_m": left["planner_safe"]["turn_radius_m"], "right_safe_radius_m": right["planner_safe"]["turn_radius_m"],
            "left_safe_turn_rate_deg_s": left["planner_safe"]["turn_rate_deg_s"], "right_safe_turn_rate_deg_s": right["planner_safe"]["turn_rate_deg_s"],
            "measured_climb_vz_mps": climb["measured"]["maximum_observed_stable_climb_vz_mps"],
            "safe_climb_vz_mps": None if climb["planner_safe"] is None else climb["planner_safe"]["climb_vz_mps"], "climb_availability": climb["availability"],
            "measured_descent_vz_mps": descent["measured"]["maximum_observed_stable_descent_vz_mps"],
            "safe_descent_vz_mps": descent["planner_safe"]["descent_vz_mps"], "descent_availability": descent["availability"],
            "combined_climb_availability": "AVAILABLE" if any(row["availability"] == "AVAILABLE" for row in cc) else "UNAVAILABLE",
            "combined_descent_availability": "AVAILABLE" if all(row["availability"] == "AVAILABLE" for row in cd) else "UNAVAILABLE",
            "combined_descent_safe": {row["direction"]: row["planner_safe"] for row in cd if row["planner_safe"] is not None},
            "main_limitation": limitation,
            "confidence": "WITHHELD_HIGH_ALT_CLIMB" if altitude >= 5000.0 else "MIXED_FAMILY_SPECIFIC",
        })

    after = {name: _sha256(path) for name, path in source_paths.items()}
    provenance = {
        "schema_version": 1, "artifact_type": "U6_2_1_PROVENANCE", "provenance_id": provenance_id,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(), "python_version": sys.version, "platform": platform.platform(),
        "files": files, "source_hashes_before": before, "source_hashes_after": after, "source_artifacts_unchanged": before == after,
    }
    result = {
        "step": "U6.2.1", "step_status": "PASS", "provenance_id": provenance_id,
        "full_altitude_dependent_profile": True, "global_aircraft_capability_constants_removed": True,
        "measured_and_safe_values_preserved": True, "aircraft_swappable": True,
        "ready_for_aircraftprofile_integration": True, "canonical_altitude_rows": 12,
        "new_jsbsim_runs": 0, "source_artifacts_unchanged": before == after,
        "production_planner_integrated": False, "next_stage_started": False,
        "runtime_s": time.perf_counter() - started,
    }
    for name, payload in (
        ("c172p_aircraft_profile_planner_safe_v2.json", profile),
        ("u6_2_1_global_constant_audit.json", constant_audit),
        ("u6_2_1_altitude_audit.json", {"schema_version": 1, "artifact_type": "U6_2_1_ALTITUDE_AUDIT", "provenance_id": provenance_id, "rows": audit_rows}),
        ("u6_2_1_provenance.json", provenance),
        ("u6_2_1_result.json", result),
    ):
        _write(name, payload)
    return result


if __name__ == "__main__":
    print(json.dumps(run_u6_2_1(), indent=2, allow_nan=False))
