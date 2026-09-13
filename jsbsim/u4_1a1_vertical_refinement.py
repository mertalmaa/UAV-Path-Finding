"""Refine planner-relevant vertical capability without mutating the U4 raw LUT."""

from __future__ import annotations

import copy
import hashlib
import json
import platform
import sys
import time
from pathlib import Path
from statistics import fmean
from typing import Any

import jsbsim
import yaml

import u4_raw_lut_characterization as u4


HERE = Path(__file__).resolve().parent
CONFIG_PATH = HERE / "u4_1a1_vertical_refinement_configuration.yaml"
U4_CONFIG_PATH = HERE / "u4_raw_lut_configuration.yaml"
U4_HARNESS_PATH = HERE / "u4_raw_lut_characterization.py"
U4_LUT_PATH = HERE / "results" / "aircraft_lut_raw.json"
U4_PROVENANCE_PATH = HERE / "results" / "u4_provenance.json"
U4_RUNS_PATH = HERE / "results" / "u4_runs.json"
U4_POINTS_PATH = HERE / "results" / "u4_points.json"
RESULTS_DIR = HERE / "results"


def _load_yaml(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write(name: str, value: Any) -> None:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    (RESULTS_DIR / name).write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _rename_probe_ids(
    records: list[dict[str, Any]], point: dict[str, Any], traces: list[dict[str, Any]]
) -> None:
    mapping: dict[str, str] = {}
    for record in records:
        old = record["run_id"]
        record["run_id"] = old.replace("u4-", "u4.1a1-", 1)
        mapping[old] = record["run_id"]
    point["run_ids"] = [mapping[item] for item in point["run_ids"]]
    for trace in traces:
        trace["run_id"] = mapping[trace["run_id"]]


def _field_summary(records: list[dict[str, Any]], field: str) -> dict[str, float]:
    measurements = [record["measurement"][field] for record in records]
    return {
        "mean": fmean(float(item["mean"]) for item in measurements),
        "minimum": min(float(item["min"]) for item in measurements),
        "maximum": max(float(item["max"]) for item in measurements),
        "maximum_absolute_window_delta": max(
            abs(float(item["delta"])) for item in measurements
        ),
    }


def _failed_checks(records: list[dict[str, Any]]) -> list[str]:
    return sorted(
        {
            name
            for record in records
            for name, passed in record["checks"].items()
            if not passed
        }
    )


def _failure_analysis(
    row: dict[str, Any], records: list[dict[str, Any]]
) -> dict[str, Any]:
    failed = _failed_checks(records)
    evidence = [record["aircraft_limit_evidence"] for record in records]
    if row["status"] == "VALID":
        category = None
    elif any(item.get("power_limited", False) for item in evidence):
        category = "POWER_LIMITED"
    elif any(item.get("stall_indicator", False) for item in evidence) or any(
        name in failed for name in ("stall_indicator_clear", "surface_usage_below_limit")
    ) or not row["repeatability"]["passed"]:
        category = "AERO_STABILITY_LIMITED"
    elif "outer_loop_not_clipped_in_measurement" in failed:
        category = "CONTROLLER_LIMITED"
    elif set(failed) <= {
        "ias_tracking",
        "bank_tracking",
        "vertical_speed_tracking",
    }:
        category = "TRACKING_ACCEPTANCE_LIMITED"
    else:
        category = "OTHER_DIAGNOSTIC"
    return {
        "category": category,
        "frozen_stack_failure_class": row["failure_class"],
        "failed_checks": failed,
        "power_limit_evidence": any(
            item.get("power_limited", False) for item in evidence
        ),
        "controller_clipped_in_measurement": (
            "outer_loop_not_clipped_in_measurement" in failed
        ),
        "stall_evidence": any(
            item.get("stall_indicator", False) for item in evidence
        ),
        "interpretation": (
            "sustainable within frozen U4 acceptance contract"
            if category is None
            else "repeatable throttle/engine boundary with tracking loss"
            if category == "POWER_LIMITED"
            else "bounded frozen controller reached a command clamp"
            if category == "CONTROLLER_LIMITED"
            else "repeatable flight response missed one or more tracking acceptance checks"
            if category == "TRACKING_ACCEPTANCE_LIMITED"
            else "aerodynamic, surface-authority, stability, or repeatability evidence"
            if category == "AERO_STABILITY_LIMITED"
            else "diagnostic evidence is insufficient for an aircraft-limit claim"
        ),
    }


def _enrich_row(
    row: dict[str, Any], records: list[dict[str, Any]], source: str
) -> dict[str, Any]:
    enriched = copy.deepcopy(row)
    enriched["source"] = source
    enriched["measurement_contract"] = "U4_FROZEN_REUSED_EXACTLY"
    enriched["ias_retention_ratio"] = enriched["actual_ias_mps"] / 40.0
    enriched["engine_indicators"] = {
        "engine_state_property": enriched["engine_state_property"],
        "engine_state_mean": enriched["engine_state_mean"],
        "propeller_rpm": _field_summary(records, "engine_0_propeller_rpm"),
        "thrust_lbs": _field_summary(records, "engine_0_thrust_lbs"),
    }
    enriched["elevator_usage"] = {
        "command_norm": _field_summary(records, "elevator_cmd_norm"),
        "physical_position_norm": _field_summary(records, "elevator_pos_norm"),
        "maximum_all_physical_surfaces_norm": enriched[
            "maximum_physical_surface_usage_norm"
        ],
    }
    ias_window = _field_summary(records, "ias_mps")
    vz_window = _field_summary(records, "vertical_speed_mps")
    enriched["measurement_window_stability"] = {
        "all_windows_complete": all(
            record["checks"]["measurement_window_complete"] for record in records
        ),
        "ias_mps": ias_window,
        "vertical_speed_mps": vz_window,
        "repeatable_across_cold_starts": enriched["repeatability"]["passed"],
        "settled_within_tracking_bands_before_window": enriched["settling"][
            "all_repeats_settled_before_measurement"
        ],
    }
    enriched["failure_analysis"] = _failure_analysis(enriched, records)
    if enriched["status"] == "VALID":
        enriched["horizontal_distance_per_100m_altitude_change_m"] = {
            "using_nominal_horizontal_speed_and_target_vz": (
                40.0 * 100.0 / abs(enriched["target_vz_mps"])
            ),
            "using_nominal_horizontal_speed_and_actual_sustained_vz": (
                40.0 * 100.0 / abs(enriched["actual_vz_mps"])
            ),
            "interpretation_only_not_bound_to_planner": True,
        }
    else:
        enriched["horizontal_distance_per_100m_altitude_change_m"] = None
    return enriched


def _boundary(rows: list[dict[str, Any]], climb: bool) -> dict[str, Any]:
    direction_rows = [
        row
        for row in rows
        if (row["target_vz_mps"] > 0 if climb else row["target_vz_mps"] < 0)
    ]
    valid = [row for row in direction_rows if row["status"] == "VALID"]
    if climb:
        sustainable = max(valid, key=lambda row: row["target_vz_mps"], default=None)
        beyond = [
            row
            for row in direction_rows
            if sustainable is not None
            and row["target_vz_mps"] > sustainable["target_vz_mps"]
            and row["status"] != "VALID"
        ]
        first_non_valid = min(beyond, key=lambda row: row["target_vz_mps"], default=None)
    else:
        sustainable = min(valid, key=lambda row: row["target_vz_mps"], default=None)
        beyond = [
            row
            for row in direction_rows
            if sustainable is not None
            and row["target_vz_mps"] < sustainable["target_vz_mps"]
            and row["status"] != "VALID"
        ]
        first_non_valid = max(beyond, key=lambda row: row["target_vz_mps"], default=None)
    return {
        "highest_or_largest_tested_sustainable_valid_target_vz_mps": (
            sustainable["target_vz_mps"] if sustainable else None
        ),
        "actual_achieved_vz_mps": (
            sustainable["actual_vz_mps"] if sustainable else None
        ),
        "horizontal_distance_per_100m_altitude_change_m_using_actual_vz": (
            sustainable["horizontal_distance_per_100m_altitude_change_m"][
                "using_nominal_horizontal_speed_and_actual_sustained_vz"
            ]
            if sustainable
            else None
        ),
        "first_non_valid_target_beyond_mps": (
            first_non_valid["target_vz_mps"] if first_non_valid else None
        ),
        "first_non_valid_status": (
            first_non_valid["status"] if first_non_valid else None
        ),
        "first_non_valid_failure_category": (
            first_non_valid["failure_analysis"]["category"]
            if first_non_valid
            else None
        ),
        "true_maximum_claimed": False,
    }


def run_u4_1a1() -> dict[str, Any]:
    started = time.perf_counter()
    config = _load_yaml(CONFIG_PATH)
    u4_config = _load_yaml(U4_CONFIG_PATH)
    raw_hash_before = _sha256(U4_LUT_PATH)
    if raw_hash_before != config["parent_raw_lut_sha256"]:
        raise RuntimeError("canonical U4 raw LUT hash changed")
    u4._freeze_check(u4_config)
    lut = _load_json(U4_LUT_PATH)
    u4_provenance = _load_json(U4_PROVENANCE_PATH)
    if u4_provenance["provenance_id"] != config["parent_u4_provenance_id"]:
        raise RuntimeError("U4 provenance changed")
    u4_runs = _load_json(U4_RUNS_PATH)["runs"]
    run_by_id = {record["run_id"]: record for record in u4_runs}

    seed = {
        "configuration": _sha256(CONFIG_PATH),
        "harness": _sha256(Path(__file__).resolve()),
        "parent_raw_lut": raw_hash_before,
        "parent_u4_provenance": _sha256(U4_PROVENANCE_PATH),
        "parent_u4_runs": _sha256(U4_RUNS_PATH),
    }
    provenance_id = "u4.1a1-" + hashlib.sha256(
        json.dumps(seed, sort_keys=True).encode("utf-8")
    ).hexdigest()[:20]

    altitudes = [float(value) for value in config["representative_altitudes_m"]]
    existing_targets = {
        float(value)
        for value in config["climb_targets_mps"] + config["descent_targets_mps"]
        if float(value).is_integer()
    }
    new_targets = [float(value) for value in config["new_half_step_targets_mps"]]
    rows: list[dict[str, Any]] = []
    new_runs: list[dict[str, Any]] = []
    new_points: list[dict[str, Any]] = []

    for altitude in altitudes:
        for target in sorted(existing_targets):
            raw_row = next(
                row
                for row in lut["vertical_lut"]
                if row["altitude_m"] == altitude
                and row["target_vz_mps"] == target
            )
            records = [run_by_id[run_id] for run_id in raw_row["run_ids"]]
            rows.append(_enrich_row(raw_row, records, "U4_EXISTING_RAW_LUT_REUSED"))
        for target in new_targets:
            records, point, traces = u4._run_point(
                altitude, "vertical", target, u4_config, provenance_id
            )
            _rename_probe_ids(records, point, traces)
            row = u4._vertical_entry(point, records, altitude, target, provenance_id)
            rows.append(_enrich_row(row, records, "U4_1A1_NEW_HALF_STEP_PROBE"))
            new_runs.extend(records)
            new_points.append(point)

    rows.sort(key=lambda row: (row["altitude_m"], row["target_vz_mps"]))
    raw_hash_after = _sha256(U4_LUT_PATH)
    if raw_hash_after != raw_hash_before:
        raise RuntimeError("U4 raw LUT was modified")

    boundary_summary = []
    for altitude in altitudes:
        altitude_rows = [row for row in rows if row["altitude_m"] == altitude]
        boundary_summary.append(
            {
                "altitude_m": altitude,
                "climb": _boundary(altitude_rows, climb=True),
                "descent": _boundary(altitude_rows, climb=False),
            }
        )
    capability_beyond_two = {
        "climb": any(
            row["status"] == "VALID" and row["target_vz_mps"] > 2.0
            for row in rows
        ),
        "descent": any(
            row["status"] == "VALID" and row["target_vz_mps"] < -2.0
            for row in rows
        ),
    }
    artifact = {
        "schema_version": 1,
        "artifact_type": "PLANNER_RELEVANT_VERTICAL_CAPABILITY_REFINEMENT",
        "provenance_id": provenance_id,
        "parent_u4_provenance_id": config["parent_u4_provenance_id"],
        "parent_raw_lut_sha256_before": raw_hash_before,
        "parent_raw_lut_sha256_after": raw_hash_after,
        "parent_raw_lut_unchanged": raw_hash_before == raw_hash_after,
        "aircraft": "c172r",
        "nominal_ias_mps": 40.0,
        "representative_altitudes_m": altitudes,
        "measurement_contract_matches_u4": True,
        "integer_points_reused_from_u4": True,
        "tested_target_grid_mps": {
            "climb": config["climb_targets_mps"],
            "descent": config["descent_targets_mps"],
        },
        "rows": rows,
        "boundary_summary": boundary_summary,
        "interpretation": {
            "capability_beyond_2_mps": capability_beyond_two,
            "plus_minus_2_is_hard_planner_limit": False,
            "climb_descent_symmetry_assumed": False,
            "true_physical_maximum_extracted": False,
            "planner_safe_limit_selected": False,
            "horizontal_distance_metric_bound_to_planner": False,
        },
    }
    provenance = {
        "provenance_id": provenance_id,
        "parent_u4_provenance_id": config["parent_u4_provenance_id"],
        "parent_raw_lut_sha256": raw_hash_before,
        "jsbsim_compiled_version": jsbsim.FGFDMExec(None).get_version(),
        "python_version": sys.version,
        "python_executable": sys.executable,
        "platform": platform.platform(),
        "files": {
            "configuration": {"path": str(CONFIG_PATH.resolve()), "sha256": _sha256(CONFIG_PATH)},
            "harness": {"path": str(Path(__file__).resolve()), "sha256": _sha256(Path(__file__).resolve())},
            "u4_raw_lut": {"path": str(U4_LUT_PATH.resolve()), "sha256": raw_hash_before},
            "u4_configuration": {"path": str(U4_CONFIG_PATH.resolve()), "sha256": _sha256(U4_CONFIG_PATH)},
            "u4_harness": {"path": str(U4_HARNESS_PATH.resolve()), "sha256": _sha256(U4_HARNESS_PATH)},
            "u4_provenance": {"path": str(U4_PROVENANCE_PATH.resolve()), "sha256": _sha256(U4_PROVENANCE_PATH)},
            "u4_runs": {"path": str(U4_RUNS_PATH.resolve()), "sha256": _sha256(U4_RUNS_PATH)},
            "u4_points": {"path": str(U4_POINTS_PATH.resolve()), "sha256": _sha256(U4_POINTS_PATH)},
        },
    }
    result = {
        "step": "U4.1A.1",
        "step_status": "PASS",
        "provenance_id": provenance_id,
        "representative_altitude_count": len(altitudes),
        "refined_rows": len(rows),
        "reused_u4_points": len(rows) - len(new_points),
        "executed_new_points": len(new_points),
        "executed_new_cold_start_runs": len(new_runs),
        "vertical_capability_sufficiently_refined_for_holdout_stage": True,
        "raw_u4_lut_unchanged": raw_hash_before == raw_hash_after,
        "controller_tuning_performed": False,
        "stock_xml_modified": False,
        "planner_modified": False,
        "interpolation_performed": False,
        "derating_performed": False,
        "holdout_validation_started": False,
        "true_physical_maximum_extracted": False,
        "runtime_s": time.perf_counter() - started,
    }
    _write("u4_1a1_vertical_refinement.json", artifact)
    _write("u4_1a1_provenance.json", provenance)
    _write("u4_1a1_runs.json", {"provenance_id": provenance_id, "runs": new_runs})
    _write("u4_1a1_points.json", {"provenance_id": provenance_id, "points": new_points})
    _write("u4_1a1_result.json", result)
    return result


if __name__ == "__main__":
    print(json.dumps(run_u4_1a1(), indent=2, allow_nan=False))
