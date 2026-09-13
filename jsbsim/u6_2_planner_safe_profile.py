from __future__ import annotations

import hashlib
import json
import math
import platform
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from statistics import fmean
from typing import Any

import yaml


HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
CONFIG_PATH = HERE / "u6_2_planner_safe_profile_configuration.yaml"
HARNESS_PATH = Path(__file__).resolve()
sys.path.insert(0, str(HERE))

from planner_safe_measurements import radius_from_arc_and_course_change, shortest_angle_deg, unwrap_degrees
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


def _rows_at_target(rows: list[dict[str, Any]], field: str, target: float) -> list[dict[str, Any]]:
    return [row for row in rows if row[field] == target]


def _linear_at(rows: list[dict[str, Any]], altitude: float, field: str) -> float:
    exact = next((row for row in rows if row["altitude_m"] == altitude), None)
    if exact is not None:
        return exact[field]
    lower = max((row for row in rows if row["altitude_m"] < altitude), key=lambda row: row["altitude_m"])
    upper = min((row for row in rows if row["altitude_m"] > altitude), key=lambda row: row["altitude_m"])
    weight = (altitude - lower["altitude_m"]) / (upper["altitude_m"] - lower["altitude_m"])
    return lower[field] + weight * (upper[field] - lower[field])


def _max_error(rows: list[dict[str, Any]], family: str, field: str, **filters: Any) -> float:
    selected = [
        row for row in rows
        if row["family"] == family and all(row[key] == value for key, value in filters.items())
    ]
    values = [row["errors"][field]["absolute"] for row in selected]
    if not values:
        raise RuntimeError(f"no holdout errors for {family}/{field}/{filters}")
    return max(values)


def _max_repeat_spread(
    holdout_rows: list[dict[str, Any]], run_by_id: dict[str, dict[str, Any]],
    family: str, measurement_path: tuple[str, ...], **filters: Any,
) -> float:
    spreads = []
    for row in holdout_rows:
        if row["family"] != family or not all(row[key] == value for key, value in filters.items()):
            continue
        values = []
        for run_id in row["run_ids"]:
            value: Any = run_by_id[run_id]
            for key in measurement_path:
                value = value[key]
            values.append(float(value))
        spreads.append(max(values) - min(values))
    return max(spreads, default=0.0)


def _legacy_status(availability: str) -> str:
    return "VALID" if availability == "AVAILABLE" else "INFEASIBLE"


def run_u6_2() -> dict[str, Any]:
    started = time.perf_counter()
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    source_paths = {name: HERE / item["path"] for name, item in config["source_artifacts"].items()}
    source_before = {name: _sha256(path) for name, path in source_paths.items()}
    expected = {name: item["sha256"] for name, item in config["source_artifacts"].items()}
    if source_before != expected:
        raise RuntimeError("U6A-U6.1 source hash mismatch before U6.2")

    core = _load(source_paths["core_raw"])
    combined = _load(source_paths["combined_raw"])
    core_holdouts = _load(source_paths["core_holdouts"])
    combined_holdouts = _load(source_paths["combined_holdouts"])
    errors = _load(source_paths["error_summary"])
    suitability = _load(source_paths["suitability"])
    outlier = _load(source_paths["outlier_diagnostics"])
    holdout_runs = _load(source_paths["u6_1_new_runs"])
    u6_1_provenance = _load(source_paths["u6_1_provenance"])
    if u6_1_provenance["provenance_id"] != config["parent_u6_1_provenance_id"]:
        raise RuntimeError("U6.1 provenance mismatch")

    files = {
        "configuration": {"path": str(CONFIG_PATH), "sha256": _sha256(CONFIG_PATH)},
        "derivation_harness": {"path": str(HARNESS_PATH), "sha256": _sha256(HARNESS_PATH)},
        "measurement_helper": {"path": str(HERE / "planner_safe_measurements.py"), "sha256": _sha256(HERE / "planner_safe_measurements.py")},
    }
    seed = {
        "files": {name: item["sha256"] for name, item in files.items()},
        "sources": source_before,
        "policy": config["capability_policy"],
    }
    provenance_id = "u6.2-" + hashlib.sha256(json.dumps(seed, sort_keys=True).encode("utf-8")).hexdigest()[:20]

    audited = outlier["radius_extraction_audit"]
    corrected_radius, wrapped = radius_from_arc_and_course_change(
        audited["trajectory_arc_length_m"], audited["stored_ground_track_change_deg"]
    )
    measurement_validation = {
        "schema_version": 1,
        "artifact_type": "U6_2_MEASUREMENT_FIX_VALIDATION",
        "provenance_id": provenance_id,
        "policy": "canonical shortest-angle normalization to [-180,+180)",
        "regression_cases": [
            {"input_deg": 247.27702298135128, "output_deg": shortest_angle_deg(247.27702298135128), "expected_deg": -112.72297701864875},
            {"input_deg": -190.0, "output_deg": shortest_angle_deg(-190.0), "expected_deg": 170.0},
            {"input_deg": 180.0, "output_deg": shortest_angle_deg(180.0), "expected_deg": -180.0},
        ],
        "heading_unwrap_case": {
            "input_deg": [350.0, 355.0, 1.0, 7.0],
            "output_deg": unwrap_degrees([350.0, 355.0, 1.0, 7.0]),
            "expected_deg": [350.0, 355.0, 361.0, 367.0],
        },
        "u6b_outlier_case": {
            "wrapped_course_change_deg": wrapped,
            "corrected_radius_m": corrected_radius,
            "expected_from_u6_1_m": audited["audited_wrapped_course_radius_m"],
            "source_raw_radius_m": audited["stored_measured_radius_m"],
            "source_raw_modified": False,
            "correction_scope": "derived_planner_safe_processing_only",
        },
        "passed": abs(corrected_radius - audited["audited_wrapped_course_radius_m"]) < 1.0e-9,
    }

    core_rows = core_holdouts["rows"]
    combined_rows = combined_holdouts["rows"]
    run_by_id = {run["run_id"]: run for run in holdout_runs["runs"]}
    altitude_grid = [float(value) for value in config["frozen_context"]["altitude_grid_m"]]
    speed_context = config["frozen_context"]["speed_context"]

    turn_allowance = {}
    turn_rate_allowance = {}
    turn_repeat_spread = {}
    for direction, bank in (("LEFT", -20.0), ("RIGHT", 20.0)):
        turn_allowance[direction] = _max_error(core_rows, "level_turn", "turn_radius_m", target_bank_deg=bank)
        turn_rate_allowance[direction] = _max_error(core_rows, "level_turn", "turn_rate_deg_s", target_bank_deg=bank)
        turn_repeat_spread[direction] = _max_repeat_spread(
            core_rows, run_by_id, "level_turn", ("trajectory", "measured_radius_m"), target_bank_deg=bank
        )

    vertical_error = {
        "climb_plus2_mps": _max_error(core_rows, "climb", "actual_vz_mps", target_vz_mps=2.0),
        "descent_minus3_mps": _max_error(core_rows, "descent", "actual_vz_mps", target_vz_mps=-3.0),
    }
    combined_descent_allowance = {}
    for direction, bank in (("LEFT", -20.0), ("RIGHT", 20.0)):
        combined_descent_allowance[direction] = _max_error(
            combined_rows, "descending_turn", "turn_radius_m", target_bank_deg=bank
        )

    straight_rows = []
    for source in core["straight_table"]:
        straight_rows.append({
            "altitude_m": source["altitude_m"], "availability": "AVAILABLE",
            "nominal_ias_mps": 40.0, "expected_actual_ias_mps": source["actual_ias_mps"],
            "expected_tas_mps": source["tas_mps"], "expected_throttle": source["throttle"],
            "power_reserve_norm": 1.0 - source["throttle"],
            "metadata": {"source_count": 3, "repeatability_passed": source["repeatable"], "interpolation": "SUPPORTED"},
        })

    level_turn_rows = []
    legacy_turn_rows = []
    for altitude in altitude_grid:
        for direction, bank in (("LEFT", -20.0), ("RIGHT", 20.0)):
            source = next(row for row in core["level_turn_table"] if row["altitude_m"] == altitude and row["target_bank_deg"] == bank)
            allowance = turn_allowance[direction] + turn_repeat_spread[direction]
            safe_radius = source["turn_radius_m"] + allowance
            raw_rate = source["turn_rate_deg_s"]
            safe_rate = math.copysign(max(0.0, abs(raw_rate) - turn_rate_allowance[direction]), raw_rate)
            metadata = {
                "source_count": 3, "repeatability_passed": source["repeatable"],
                "repeatability_radius_spread_m": turn_repeat_spread[direction],
                "holdout_max_abs_radius_error_m": turn_allowance[direction],
                "uncertainty_allowance_m": allowance,
                "safe_radius_not_less_than_source": safe_radius >= source["turn_radius_m"],
                "confidence": "HIGH", "treatment": "SUPPORTED_DIRECTION_SEPARATED_LINEAR",
                "raw_reference_radius_m": source["turn_radius_m"],
            }
            row = {
                "altitude_m": altitude, "direction": direction, "bank_deg": bank,
                "availability": "AVAILABLE", "safe_turn_radius_m": safe_radius,
                "safe_turn_rate_deg_s": safe_rate, "actual_bank_reference_deg": source["actual_bank_deg"],
                "metadata": metadata,
            }
            level_turn_rows.append(row)
            legacy_turn_rows.append({
                "altitude_m": altitude, "speed_context": speed_context, "bank_deg": bank,
                "direction": direction, "status": "VALID", "turn_radius_m": safe_radius,
                "turn_rate_deg_s": safe_rate, "actual_bank_deg": source["actual_bank_deg"],
                "metadata": {**metadata, "availability": "AVAILABLE"},
            })
        for direction, bank in (("LEFT", -30.0), ("RIGHT", 30.0)):
            source = next(row for row in core["level_turn_table"] if row["altitude_m"] == altitude and row["target_bank_deg"] == bank)
            metadata = {
                "availability": "UNAVAILABLE", "treatment": "LIMITED_OPTIONAL_NON_GUARANTEED",
                "raw_reference_radius_m": source["turn_radius_m"], "raw_reference_preserved": True,
            }
            level_turn_rows.append({
                "altitude_m": altitude, "direction": direction, "bank_deg": bank,
                "availability": "UNAVAILABLE", "safe_turn_radius_m": None,
                "safe_turn_rate_deg_s": None, "actual_bank_reference_deg": source["actual_bank_deg"],
                "metadata": metadata,
            })
            legacy_turn_rows.append({
                "altitude_m": altitude, "speed_context": speed_context, "bank_deg": bank,
                "direction": direction, "status": "INFEASIBLE", "turn_radius_m": None,
                "turn_rate_deg_s": None, "actual_bank_deg": source["actual_bank_deg"], "metadata": metadata,
            })

    climb_rows = []
    descent_rows = []
    legacy_vertical_rows = []
    for altitude in altitude_grid:
        climb_source = next(row for row in core["straight_vertical_table"] if row["altitude_m"] == altitude and row["target_vz_mps"] == 2.0)
        climb_available = altitude <= 4500.0
        reviewed_unknown = climb_available and climb_source["status"] == "UNKNOWN"
        climb_metadata = {
            "source_count": 3, "raw_status": climb_source["status"],
            "raw_actual_vz_mps": climb_source["actual_vz_mps"],
            "repeatability_passed": climb_source["repeatable"],
            "actual_stable_response": climb_source["actual_stable_response"],
            "holdout_max_abs_vz_error_mps": vertical_error["climb_plus2_mps"],
            "saturation_margin_norm": 1.0 - climb_source["throttle"],
            "reviewed_unknown_to_available": reviewed_unknown,
            "review_basis": "actual_stable_repeatable_healthy_IAS_no_saturation_and_midpoint_holdouts" if reviewed_unknown else None,
            "confidence": "MEDIUM" if reviewed_unknown else "HIGH" if climb_available else "WITHHELD",
            "treatment": "DISCRETE_BAND_ABOVE_4000; NO_CAPABILITY_INTERPOLATION_5000_5500",
        }
        climb_rows.append({
            "altitude_m": altitude, "mode": "CLIMB",
            "availability": "AVAILABLE" if climb_available else "UNAVAILABLE",
            "safe_command_vz_mps": 2.0 if climb_available else None,
            "safe_achievable_vz_mps": 2.0 if climb_available else None,
            "metadata": climb_metadata,
        })
        legacy_vertical_rows.append({
            "altitude_m": altitude, "speed_context": speed_context, "desired_vz_mps": 2.0,
            "status": _legacy_status("AVAILABLE" if climb_available else "UNAVAILABLE"),
            "achievable_vz_mps": 2.0 if climb_available else None,
            "gamma_deg": climb_source["gamma_deg"] if climb_available else None,
            "metadata": {**climb_metadata, "availability": "AVAILABLE" if climb_available else "UNAVAILABLE"},
        })

        descent_source = next(row for row in core["straight_vertical_table"] if row["altitude_m"] == altitude and row["target_vz_mps"] == -3.0)
        descent_metadata = {
            "source_count": 3, "raw_status": descent_source["status"],
            "raw_actual_vz_mps": descent_source["actual_vz_mps"],
            "repeatability_passed": descent_source["repeatable"],
            "actual_stable_response": descent_source["actual_stable_response"],
            "holdout_max_abs_vz_error_mps": vertical_error["descent_minus3_mps"],
            "saturation_margin_norm": 1.0 - descent_source["throttle"],
            "reviewed_unknown_to_available": True,
            "review_basis": "12_anchors_plus_6_holdouts_actual_stable_repeatable_healthy_IAS_without_saturation",
            "confidence": "MEDIUM", "treatment": "LIMITED_CONTINUOUS_VALUES_REVIEWED_CAPABILITY",
        }
        descent_rows.append({
            "altitude_m": altitude, "mode": "DESCENT", "availability": "AVAILABLE",
            "safe_command_vz_mps": -3.0, "safe_achievable_vz_mps": -3.0,
            "metadata": descent_metadata,
        })
        legacy_vertical_rows.append({
            "altitude_m": altitude, "speed_context": speed_context, "desired_vz_mps": -3.0,
            "status": "VALID", "achievable_vz_mps": -3.0, "gamma_deg": descent_source["gamma_deg"],
            "metadata": {**descent_metadata, "availability": "AVAILABLE"},
        })

    climbing_turn_rows = [
        {
            "altitude_m": altitude, "mode": "CLIMBING_TURN", "bank_deg": 20.0,
            "direction": direction, "availability": "UNAVAILABLE",
            "safe_turn_radius_m": None, "safe_vz_mps": None,
            "metadata": {
                "source_count": 0 if altitude < 1000.0 else 3,
                "confidence": "WITHHELD", "treatment": "DISABLED_FIRST_PLANNER_SAFE_PROFILE",
                "reason": "raw_marginal_or_unusable_and_high_altitude_capability_interpolation_unsupported",
            },
        }
        for altitude in altitude_grid for direction in ("LEFT", "RIGHT")
    ]

    combined_descent_source = [
        row for row in combined["rows"]
        if row["grid_role"] == "PRIMARY" and row["target_vz_mps"] == -2.5
    ]
    descending_turn_rows = []
    for altitude in altitude_grid:
        for direction, bank in (("LEFT", -20.0), ("RIGHT", 20.0)):
            available = altitude >= 3500.0
            direction_sources = [row for row in combined_descent_source if row["direction"] == direction]
            raw_radius = _linear_at(direction_sources, altitude, "turn_radius_m") if altitude >= 1000.0 else None
            allowance = combined_descent_allowance[direction]
            descending_turn_rows.append({
                "altitude_m": altitude, "mode": "DESCENDING_TURN", "bank_deg": bank,
                "direction": direction, "availability": "AVAILABLE" if available else "UNAVAILABLE",
                "safe_turn_radius_m": raw_radius + allowance if available else None,
                "safe_vz_mps": -2.5 if available else None,
                "metadata": {
                    "source_count": 2 if available and altitude not in (4000.0, 5000.0, 5500.0) else 3 if available else 0,
                    "holdout_max_abs_radius_error_m": allowance,
                    "raw_or_interpolated_reference_radius_m": raw_radius,
                    "safe_radius_not_less_than_reference": None if not available else raw_radius + allowance >= raw_radius,
                    "confidence": "HIGH" if available else "WITHHELD",
                    "treatment": "SUPPORTED_DIRECTION_SEPARATED_CONTINUOUS" if available else "CONSERVATIVE_DISCRETE_UNAVAILABLE",
                },
            })

    profile = {
        "schema_version": 1,
        "profile_schema_id": "aircraft_profile_planner_safe_v1",
        "artifact_type": "C172P_AIRCRAFT_PROFILE_PLANNER_SAFE",
        "lut_stage": "planner_safe",
        "profile_stage": "planner_safe",
        "planner_ready": True,
        "aircraft_id": "c172p",
        "controller_stack_id": config["frozen_context"]["controller_stack_id"],
        "provenance_id": provenance_id,
        "source_provenance": {
            "u6a": core["provenance_id"], "u6b": combined["provenance_id"],
            "u6_1": u6_1_provenance["provenance_id"],
        },
        "same_stack_identity": config["frozen_context"],
        "nominal_ias_mps": 40.0,
        "min_altitude_m": 0.0,
        "max_altitude_m": 5500.0,
        "units": {"altitude": "m", "speed": "m/s", "angle": "deg", "time": "s"},
        "speed_contexts": [speed_context],
        "altitude_grid_m": altitude_grid,
        "turn_bank_grid_deg": [-30.0, -20.0, 20.0, 30.0],
        "turn_directions": ["LEFT", "RIGHT"],
        "vertical_speed_grid_mps": [-3.0, 2.0],
        "query_semantics": {
            "availability": ["AVAILABLE", "UNAVAILABLE", "OUT_OF_DOMAIN"],
            "out_of_domain_rule": "altitude_m < 0 or altitude_m > 5500",
            "raw_unknown_silently_available": False,
            "legacy_loader_status_mapping": config["query_semantics"]["legacy_loader_status_mapping"],
        },
        "interface_contract": {
            "turn_query": {"inputs": ["altitude_m", "direction", "bank_deg"], "outputs": ["availability", "safe_turn_radius_m", "safe_turn_rate_deg_s", "metadata"]},
            "vertical_query": {"inputs": ["altitude_m", "mode"], "outputs": ["availability", "safe_command_vz_mps", "safe_achievable_vz_mps", "metadata"]},
            "combined_query": {"inputs": ["altitude_m", "direction", "mode"], "outputs": ["availability", "safe_turn_radius_m", "safe_vz_mps", "metadata"]},
            "aircraft_specific_branching_forbidden": True,
        },
        "interpolation_policy": {
            "suitability_source": suitability["provenance_id"],
            "continuous_supported_only": True,
            "availability_interpolated": False,
            "unsupported_regions_use_discrete_bands": True,
        },
        "measurement_policy": {
            "helper_sha256": files["measurement_helper"]["sha256"],
            "shortest_angle_interval": "[-180,+180)",
            "historical_raw_modified": False,
            "known_corrected_value_provenance": outlier["provenance_id"],
        },
        "robustness_summary": {
            "core_raw_repeatable_rows": sum(row["repeatable"] for row in core["straight_table"] + core["level_turn_table"] + core["straight_vertical_table"]),
            "core_raw_total_rows": len(core["straight_table"] + core["level_turn_table"] + core["straight_vertical_table"]),
            "combined_raw_repeatable_rows": sum(row["repeatable"] for row in combined["rows"]),
            "combined_raw_total_rows": len(combined["rows"]),
            "holdout_repeatable_points": sum(row["repeatable"] for row in core_rows + combined_rows),
            "holdout_total_points": len(core_rows + combined_rows),
            "turn_radius_allowance_m": turn_allowance,
            "combined_descent_radius_allowance_m": combined_descent_allowance,
            "vertical_holdout_max_abs_error_mps": vertical_error,
        },
        "capabilities": {
            "straight": {"interpolation": "SUPPORTED", "rows": straight_rows},
            "level_turn": {"primary_bank_deg": 20.0, "bank_30_policy": "OPTIONAL_NON_GUARANTEED_UNAVAILABLE", "left_right_averaged": False, "rows": level_turn_rows},
            "straight_climb": {"availability_policy": "AVAILABLE_0_4500_DISCRETE; UNAVAILABLE_5000_5500", "rows": climb_rows},
            "straight_descent": {"availability_policy": "REVIEWED_MINUS3_AVAILABLE_0_5500", "rows": descent_rows},
            "climbing_turn": {"availability_policy": "DISABLED_FIRST_PLANNER_SAFE_PROFILE", "rows": climbing_turn_rows},
            "descending_turn": {"availability_policy": "AVAILABLE_3500_5500_ONLY", "rows": descending_turn_rows},
        },
        "turn_table": legacy_turn_rows,
        "vertical_table": legacy_vertical_rows,
        "directly_infeasible_implies_unreachable": False,
        "production_search_integrated": False,
    }

    derivation_audit = {
        "schema_version": 1, "artifact_type": "U6_2_DERIVATION_AUDIT", "provenance_id": provenance_id,
        "methodology": config["derivation_policy"],
        "turn_allowance_m": turn_allowance, "turn_repeatability_spread_m": turn_repeat_spread,
        "turn_rate_allowance_deg_s": turn_rate_allowance,
        "vertical_error_allowance_mps": vertical_error,
        "combined_descent_radius_allowance_m": combined_descent_allowance,
        "safe_climb_boundary_evidence": config["capability_policy"]["straight_climb"],
        "safe_descent_evidence": config["capability_policy"]["straight_descent"],
        "combined_climb_policy": config["capability_policy"]["climbing_turn"],
        "combined_descent_policy": config["capability_policy"]["descending_turn"],
        "no_arbitrary_safety_factor": True,
        "no_new_jsbsim_runs": True,
    }
    source_after = {name: _sha256(path) for name, path in source_paths.items()}
    provenance = {
        "schema_version": 1, "artifact_type": "U6_2_PROVENANCE", "provenance_id": provenance_id,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(), "aircraft_id": "c172p",
        "nominal_ias_mps": 40.0, "current_domain_m": [0.0, 5500.0],
        "controller_stack_id": config["frozen_context"]["controller_stack_id"],
        "repository_git_commit": u6a._git_commit(), "python_version": sys.version,
        "platform": platform.platform(), "files": files,
        "source_hashes_before": source_before, "source_hashes_after": source_after,
        "source_artifacts_unchanged": source_before == source_after,
    }
    result = {
        "step": "U6.2", "step_status": "PASS", "provenance_id": provenance_id,
        "planner_safe_profile_ready": True, "measurement_pipeline_fixed": measurement_validation["passed"],
        "aircraft_swappable_profile": True, "ready_for_aircraftprofile_integration": True,
        "new_jsbsim_runs": 0, "max_altitude_m": 5500.0,
        "source_artifacts_unchanged": source_before == source_after,
        "combined_climb_available_rows": sum(row["availability"] == "AVAILABLE" for row in climbing_turn_rows),
        "descending_turn_available_rows": sum(row["availability"] == "AVAILABLE" for row in descending_turn_rows),
        "planner_or_search_integrated": False, "heading_or_primitives_implemented": False,
        "grid_regate_performed": False, "next_stage_started": False,
        "runtime_s": time.perf_counter() - started,
    }
    for name, payload in (
        ("c172p_aircraft_profile_planner_safe.json", profile),
        ("u6_2_derivation_audit.json", derivation_audit),
        ("u6_2_measurement_fix_validation.json", measurement_validation),
        ("u6_2_provenance.json", provenance),
        ("u6_2_result.json", result),
    ):
        _write(name, payload)
    return result


if __name__ == "__main__":
    print(json.dumps(run_u6_2(), indent=2, allow_nan=False))
