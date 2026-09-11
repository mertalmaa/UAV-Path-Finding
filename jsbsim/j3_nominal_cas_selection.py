"""J3 nominal-CAS candidate sweep and baseline throttle-policy selection.

Only straight, level trim/replay is exercised. The J2 fresh-instance harness is
reused. No climb/descent capability, turn, lookup, planner integration, or
derating work is performed.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
from statistics import fmean
from typing import Any

import yaml

import j2_baseline_sanity as j2


HERE = Path(__file__).resolve().parent
J2_CONFIG_PATH = HERE / "j2_reference_configuration.yaml"
J3_CONFIG_PATH = HERE / "j3_candidate_configuration.yaml"
RESULTS_DIR = HERE / "results"
PSF_TO_PA = 47.88025898033584


def _load_yaml(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _extend_j2_snapshot() -> None:
    """Add J3 diagnostics without changing the closed J2 source/artifacts."""
    base_snapshot = j2._snapshot

    def extended(fdm: Any, elapsed_s: float) -> dict[str, float]:
        sample = base_snapshot(fdm, elapsed_s)
        sample.update(
            {
                "dynamic_pressure_pa": fdm["aero/qbar-psf"] * PSF_TO_PA,
                "elevator_cmd_norm": fdm["fcs/elevator-cmd-norm"],
                "aileron_cmd_norm": fdm["fcs/aileron-cmd-norm"],
                "rudder_cmd_norm": fdm["fcs/rudder-cmd-norm"],
                "pitch_trim_cmd_norm": fdm["fcs/pitch-trim-cmd-norm"],
                "roll_trim_cmd_norm": fdm["fcs/roll-trim-cmd-norm"],
                "yaw_trim_cmd_norm": fdm["fcs/yaw-trim-cmd-norm"],
            }
        )
        return sample

    j2._snapshot = extended


def _j3_provenance(j2_config: dict[str, Any], j3_config: dict[str, Any]) -> dict[str, Any]:
    base = j2._provenance(j2_config)
    j3_files = {
        "candidate_configuration": J3_CONFIG_PATH,
        "nominal_cas_harness": Path(__file__).resolve(),
    }
    hashes = {
        name: {"path": str(path), "sha256": _sha256(path)}
        for name, path in j3_files.items()
    }
    fingerprint = {
        "base_model_provenance_id": base["provenance_id"],
        "candidate_cas_kts": j3_config["candidate_cas_kts"],
        "main_altitudes_msl_m": j3_config["main_altitudes_msl_m"],
        "j3_files": {name: item["sha256"] for name, item in hashes.items()},
    }
    provenance_id = "j3-" + hashlib.sha256(
        json.dumps(fingerprint, sort_keys=True).encode("utf-8")
    ).hexdigest()[:20]
    return {
        "provenance_id": provenance_id,
        "parent_j2_provenance_id": base["provenance_id"],
        "base_model_provenance": base,
        "j3_files": hashes,
        "candidate_rationale": j3_config["candidate_rationale"],
    }


def _normalize_zero_wind_roundoff(record: dict[str, Any], tolerance_fps: float) -> None:
    """Treat coordinate-transform roundoff as zero, without masking real wind."""
    if record.get("checks", {}).get("zero_wind", True):
        return
    measurement = record.get("measurement")
    if not measurement:
        return
    residual = max(
        abs(measurement[key][bound])
        for key in ("wind_north_fps", "wind_east_fps", "wind_down_fps")
        for bound in ("min", "max")
    )
    record["diagnostic_zero_wind_residual_fps"] = residual
    record["diagnostic_zero_wind_tolerance_fps"] = tolerance_fps
    if residual > tolerance_fps:
        return
    record["checks"]["zero_wind"] = True
    record["status_reasons"] = [
        reason for reason in record["status_reasons"] if reason != "zero_wind"
    ]
    if all(record["checks"].values()):
        record["status"] = "VALID"


def _point_result(
    candidate_kcas: float,
    altitude_msl_m: float,
    records: list[dict[str, Any]],
    repeatability: dict[str, Any],
    provenance_id: str,
) -> dict[str, Any]:
    if repeatability["passed"] and all(record["status"] == "VALID" for record in records):
        status = "VALID"
    elif any(record["status"] == "UNKNOWN" for record in records) or not repeatability["passed"]:
        status = "UNKNOWN"
    elif all(record["status"] == "INFEASIBLE" for record in records):
        status = "INFEASIBLE"
    else:
        status = "UNKNOWN"

    def metric(key: str, stat: str) -> float:
        return fmean(record["measurement"][key][stat] for record in records)

    def maximum(key: str, stat: str, *, absolute: bool = False) -> float:
        values = [record["measurement"][key][stat] for record in records]
        return max(abs(value) for value in values) if absolute else max(values)

    max_control_usage = max(
        record["tracking"]["maximum_normalized_control_surface_usage"] for record in records
    )
    max_cas_error = max(record["tracking"]["maximum_cas_relative_error"] for record in records)
    max_gamma_error = max(
        record["tracking"]["maximum_gamma_absolute_error_deg"] for record in records
    )
    return {
        "provenance_id": provenance_id,
        "altitude_msl_m": altitude_msl_m,
        "maneuver_family": "straight_level",
        "requested_target": {"speed_type": "KCAS", "speed_value": candidate_kcas, "gamma_deg": 0.0},
        "status": status,
        "raw_capability": None,
        "derated_capability": None,
        "measured_values": {
            "actual_kcas_mean": metric("kcas", "mean"),
            "tas_mps_mean": metric("tas_mps", "mean"),
            "mach_mean": metric("mach", "mean"),
            "alpha_deg_mean": metric("alpha_deg", "mean"),
            "beta_deg_mean": metric("beta_deg", "mean"),
            "gamma_deg_mean": metric("gamma_deg", "mean"),
            "throttle_cmd_norm_mean": metric("throttle_cmd_norm", "mean"),
            "throttle_pos_norm_mean": metric("throttle_pos_norm", "mean"),
            "elevator_pos_norm_mean": metric("elevator_pos_norm", "mean"),
            "left_aileron_pos_norm_mean": metric("left_aileron_pos_norm", "mean"),
            "right_aileron_pos_norm_mean": metric("right_aileron_pos_norm", "mean"),
            "rudder_pos_norm_mean": metric("rudder_pos_norm", "mean"),
            "load_factor_nz_mean": metric("load_factor_nz", "mean"),
            "dynamic_pressure_pa_mean": metric("dynamic_pressure_pa", "mean"),
            "p_deg_s_mean": metric("p_deg_s", "mean"),
            "q_deg_s_mean": metric("q_deg_s", "mean"),
            "r_deg_s_mean": metric("r_deg_s", "mean"),
            "altitude_drift_m_max_abs": maximum("altitude_msl_m", "delta", absolute=True),
            "speed_drift_kcas_max_abs": maximum("kcas", "delta", absolute=True),
            "maximum_cas_tracking_error_relative": max_cas_error,
            "maximum_gamma_tracking_error_deg": max_gamma_error,
            "maximum_control_surface_usage_norm": max_control_usage,
        },
        "diagnostics": {
            "repeatability": repeatability,
            "trim_success_count": sum(record["trim"]["succeeded"] for record in records),
            "run_statuses": [record["status"] for record in records],
            "all_checks_passed": all(all(record["checks"].values()) for record in records),
            "control_surface_saturation_observed": max_control_usage > 0.90,
            "afterburner_observed": maximum("throttle_pos_norm", "max") > 1.0,
            "run_ids": [record["run_id"] for record in records],
        },
    }


def _candidate_summary(
    candidate_kcas: float, points: list[dict[str, Any]], j3_config: dict[str, Any]
) -> dict[str, Any]:
    boundaries = j3_config["model_observation_boundaries"]
    statuses = {
        status: sum(point["status"] == status for point in points)
        for status in ("VALID", "INFEASIBLE", "UNKNOWN")
    }

    def values(key: str) -> list[float]:
        return [point["measured_values"][key] for point in points]

    minimum_actual_kcas = min(values("actual_kcas_mean"))
    mach_max = max(values("mach_mean"))
    throttle_min = min(values("throttle_pos_norm_mean"))
    throttle_max = max(values("throttle_pos_norm_mean"))
    max_control = max(values("maximum_control_surface_usage_norm"))
    worst_alpha = max(abs(value) for value in values("alpha_deg_mean"))

    margin_components = {
        "lower_speed_schedule_clearance": (
            minimum_actual_kcas - boundaries["trailing_edge_flap_schedule_kcas"]
        ) / boundaries["trailing_edge_flap_schedule_kcas"],
        "mach_schedule_clearance": (
            boundaries["mach_schedule_breakpoint"] - mach_max
        ) / boundaries["mach_schedule_breakpoint"],
        "dry_throttle_two_sided_reserve": min(
            throttle_min,
            boundaries["afterburner_off_max_throttle_position_norm"] - throttle_max,
        ),
        "control_surface_reserve": (
            boundaries["normalized_control_surface_usage_max"] - max_control
        ) / boundaries["normalized_control_surface_usage_max"],
        "alpha_command_reduction_clearance": (
            boundaries["alpha_command_reduction_start_abs_deg"] - worst_alpha
        ) / boundaries["alpha_command_reduction_start_abs_deg"],
    }
    eligible = (
        statuses["VALID"] == len(j3_config["main_altitudes_msl_m"])
        and statuses["UNKNOWN"] == 0
        and statuses["INFEASIBLE"] == 0
        and minimum_actual_kcas > boundaries["trailing_edge_flap_schedule_kcas"]
        and mach_max < boundaries["mach_schedule_breakpoint"]
        and throttle_max <= boundaries["afterburner_off_max_throttle_position_norm"]
        and max_control <= boundaries["normalized_control_surface_usage_max"]
    )
    return {
        "candidate_kcas": candidate_kcas,
        "altitude_status_counts": statuses,
        "eligible_single_cas": eligible,
        "worst_abs_alpha_deg": worst_alpha,
        "throttle_position_norm_range": [throttle_min, throttle_max],
        "mach_range": [min(values("mach_mean")), mach_max],
        "dynamic_pressure_pa_range": [
            min(values("dynamic_pressure_pa_mean")),
            max(values("dynamic_pressure_pa_mean")),
        ],
        "maximum_cas_tracking_error_relative": max(values("maximum_cas_tracking_error_relative")),
        "maximum_gamma_error_deg": max(values("maximum_gamma_tracking_error_deg")),
        "maximum_altitude_drift_m": max(values("altitude_drift_m_max_abs")),
        "maximum_speed_drift_kcas": max(values("speed_drift_kcas_max_abs")),
        "maximum_control_surface_usage_norm": max_control,
        "margin_components": margin_components,
        "minimum_normalized_observed_margin": min(margin_components.values()),
    }


def _select_candidate(summaries: list[dict[str, Any]]) -> dict[str, Any]:
    eligible = [summary for summary in summaries if summary["eligible_single_cas"]]
    if not eligible:
        return {
            "decision": "single nominal CAS assumption invalid",
            "selected_nominal_cas_kts": None,
            "j3_pass": False,
            "reason": "No candidate was VALID with required margins across all main altitudes.",
        }
    selected = max(
        eligible,
        key=lambda item: (
            item["minimum_normalized_observed_margin"],
            -item["maximum_cas_tracking_error_relative"],
            -item["maximum_gamma_error_deg"],
            -item["maximum_altitude_drift_m"],
        ),
    )
    return {
        "decision": "single_nominal_cas_selected",
        "selected_nominal_cas_kts": selected["candidate_kcas"],
        "j3_pass": True,
        "selection_method": "maximize_minimum_normalized_observed_margin",
        "winning_minimum_normalized_observed_margin": selected[
            "minimum_normalized_observed_margin"
        ],
        "winning_margin_components": selected["margin_components"],
    }


def run_j3() -> dict[str, Any]:
    j2_config = _load_yaml(J2_CONFIG_PATH)
    j3_config = _load_yaml(J3_CONFIG_PATH)
    provenance = _j3_provenance(j2_config, j3_config)
    _extend_j2_snapshot()

    all_runs: list[dict[str, Any]] = []
    points: list[dict[str, Any]] = []
    summaries: list[dict[str, Any]] = []

    for candidate_kcas in j3_config["candidate_cas_kts"]:
        candidate_points: list[dict[str, Any]] = []
        for altitude_msl_m in j3_config["main_altitudes_msl_m"]:
            run_config = copy.deepcopy(j2_config)
            run_config["sanity_condition"]["speed_value"] = float(candidate_kcas)
            run_config["sanity_condition"]["representative_altitudes_msl_m"] = [
                float(altitude_msl_m)
            ]
            records = [
                j2._run_once(
                    float(altitude_msl_m),
                    repeat,
                    run_config,
                    provenance["provenance_id"],
                )
                for repeat in range(1, j3_config["cold_start_repetitions_per_point"] + 1)
            ]
            for record in records:
                record["run_id"] = (
                    f"j3-level-{float(candidate_kcas):.0f}kcas-"
                    f"{float(altitude_msl_m):.0f}m-r{record['repeat_index']}"
                )
                _normalize_zero_wind_roundoff(
                    record,
                    float(j3_config["numerical_tolerances"]["zero_wind_residual_fps"]),
                )
            repeatability = j2._repeatability(records, run_config)
            if not repeatability["passed"]:
                for record in records:
                    if record["status"] == "VALID":
                        record["status"] = "UNKNOWN"
                        record["status_reasons"].append("cold_start_repeatability_failed")
            point = _point_result(
                float(candidate_kcas),
                float(altitude_msl_m),
                records,
                repeatability,
                provenance["provenance_id"],
            )
            all_runs.extend(records)
            points.append(point)
            candidate_points.append(point)
        summaries.append(_candidate_summary(float(candidate_kcas), candidate_points, j3_config))

    selection = _select_candidate(summaries)
    selection.update(
        {
            "provenance_id": provenance["provenance_id"],
            "parent_j2_provenance_id": provenance["parent_j2_provenance_id"],
            "tested_candidate_cas_kts": j3_config["candidate_cas_kts"],
            "tested_altitudes_msl_m": j3_config["main_altitudes_msl_m"],
            "point_count": len(points),
            "run_count": len(all_runs),
            "throttle_policy_baseline": j3_config["throttle_policy_baseline"],
            "scope_guards": j3_config["scope"],
        }
    )

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    artifacts = {
        "j3_provenance.json": provenance,
        "j3_runs.json": all_runs,
        "j3_points.json": points,
        "j3_candidate_summary.json": summaries,
        "j3_selection.json": selection,
    }
    for filename, payload in artifacts.items():
        (RESULTS_DIR / filename).write_text(
            json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
    return selection


if __name__ == "__main__":
    result = run_j3()
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    raise SystemExit(0 if result["j3_pass"] else 1)
