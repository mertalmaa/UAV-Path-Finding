"""J4B straight sustained climb/descent boundary characterization."""

from __future__ import annotations

import copy
import hashlib
import json
import time
from pathlib import Path
from statistics import fmean
from typing import Any

import yaml

import j2_baseline_sanity as j2
import j4a_straight_vertical_sanity as j4a


HERE = Path(__file__).resolve().parent
RESULTS_DIR = HERE / "results"
CONFIG_PATH = HERE / "j4b_capability_configuration.yaml"
J4A_CONFIG_PATH = HERE / "j4a_sanity_configuration.yaml"
J2_CONFIG_PATH = HERE / "j2_reference_configuration.yaml"
PARENT_PROVENANCE_PATH = RESULTS_DIR / "j4a_provenance.json"


def _load_yaml(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _build_run_config(config: dict[str, Any]) -> dict[str, Any]:
    run_config = _load_yaml(J4A_CONFIG_PATH)
    run_config["reference"]["nominal_cas_kts"] = config["reference"]["nominal_cas_kts"]
    run_config["simulation"]["gamma_target_ramp_s"] = config["controller"]["gamma_target_ramp_s"]
    run_config["outer_loop_sanity_controller"]["gamma"] = copy.deepcopy(config["controller"]["gamma"])
    run_config["outer_loop_sanity_controller"]["cas"] = copy.deepcopy(config["controller"]["cas"])
    return run_config


def _provenance(config: dict[str, Any], j2_config: dict[str, Any]) -> dict[str, Any]:
    parent = json.loads(PARENT_PROVENANCE_PATH.read_text(encoding="utf-8"))
    if parent["provenance_id"] != config["parent_provenance_id"]:
        raise RuntimeError("J4A parent provenance mismatch")
    base = j2._provenance(j2_config)
    files = {
        "j4b_configuration": CONFIG_PATH,
        "j4b_harness": Path(__file__).resolve(),
        "j4a_configuration": J4A_CONFIG_PATH,
        "j4a_harness": HERE / "j4a_straight_vertical_sanity.py",
        "parent_j4a_provenance": PARENT_PROVENANCE_PATH,
    }
    hashes = {
        name: {"path": str(path), "sha256": _sha256(path)}
        for name, path in files.items()
    }
    fingerprint = {
        "parent": parent["provenance_id"],
        "base_model": base["provenance_id"],
        "files": {name: item["sha256"] for name, item in hashes.items()},
        "altitudes": config["reference"]["main_altitudes_msl_m"],
        "sweep": config["sweep"],
        "controller": config["controller"],
        "classification": config["classification"],
    }
    provenance_id = "j4b-" + hashlib.sha256(
        json.dumps(fingerprint, sort_keys=True).encode("utf-8")
    ).hexdigest()[:20]
    return {
        "provenance_id": provenance_id,
        "parent_j4a_provenance_id": parent["provenance_id"],
        "base_model_provenance_id": base["provenance_id"],
        "base_model_provenance": base,
        "files": hashes,
    }


def _controller_audit(config: dict[str, Any]) -> dict[str, Any]:
    return {
        "signal_chain": {
            "gamma": "target_gamma -> PI outer loop -> fcs/elevator-cmd-norm -> native F-16 pitch FCS -> elevator surface",
            "cas": "target_305_KCAS -> PI outer loop -> fcs/throttle-cmd-norm -> FCS throttle gain -> engine",
        },
        "gamma_controller_commands": "fcs/elevator-cmd-norm",
        "cas_controller_commands": "fcs/throttle-cmd-norm",
        "throttle_adjusted_by": "J4B CAS PI outer loop around level-trim baseline",
        "native_fcs_active": config["controller"]["native_fcs_active"],
        "fbw_override": config["controller"]["fbw_override"],
        "direct_surface_position_commanded": config["controller"]["direct_surface_position_commanded"],
        "command_limits": {
            "elevator_cmd_norm": [
                config["controller"]["gamma"]["elevator_cmd_min_norm"],
                config["controller"]["gamma"]["elevator_cmd_max_norm"],
            ],
            "throttle_cmd_norm": [
                config["controller"]["cas"]["throttle_cmd_min_norm"],
                config["controller"]["cas"]["throttle_cmd_max_dry_norm"],
            ],
            "throttle_position_norm": [0.0, 1.0],
            "normalized_control_surface_acceptance": 0.90,
        },
        "anti_windup": config["controller"]["anti_windup"],
        "classification_rule": {
            "controller_setup_failure": "UNKNOWN",
            "tracking_failure_alone": "UNKNOWN",
            "reliable_dry_throttle_boundary_with_CAS_failure_and_gamma_authority": "INFEASIBLE/thrust_limited",
        },
    }


def _classify_run(record: dict[str, Any], target_gamma: float, config: dict[str, Any]) -> dict[str, Any]:
    record["methodology_status_before_capability_evidence"] = record["status"]
    record["methodology_status_reasons"] = list(record["status_reasons"])
    if "measurement" not in record or "checks" not in record:
        record["status"] = "UNKNOWN"
        record["failure_classification"] = "trim_setup_failure"
        record["capability_evidence"] = {"reliable": False}
        return record

    checks = record["checks"]
    base_keys = (
        "trim_succeeded", "finite_outputs", "measurement_window_complete",
        "afterburner_off", "engine_running", "gear_up", "speedbrake_closed",
        "native_fcs", "zero_wind", "fixed_weight", "fixed_fuel",
        "control_surface_usage", "gamma_sign", "vertical_speed_sign",
        "altitude_evolution_sign",
    )
    base_reliable = all(checks.get(key, False) for key in base_keys)
    tracking_reliable = (
        checks["cas_tracking"] and checks["gamma_tracking"]
        and checks["settled_by_measurement_window"]
    )
    measurement = record["measurement"]
    controls = config["controller"]
    epsilon = config["classification"]["sustained_command_boundary_epsilon_norm"]
    mean_boundary_tolerance = config["classification"][
        "measurement_mean_boundary_tolerance_norm"
    ]
    sustained_dry_max = (
        measurement["throttle_cmd_norm"]["mean"]
        >= controls["cas"]["throttle_cmd_max_dry_norm"] - mean_boundary_tolerance
        and measurement["throttle_cmd_norm"]["end"]
        >= controls["cas"]["throttle_cmd_max_dry_norm"] - epsilon
    )
    sustained_min_throttle = (
        measurement["throttle_cmd_norm"]["mean"]
        <= controls["cas"]["throttle_cmd_min_norm"] + mean_boundary_tolerance
        and measurement["throttle_cmd_norm"]["end"]
        <= controls["cas"]["throttle_cmd_min_norm"] + epsilon
    )
    sustained_elevator_limit = (
        measurement["elevator_cmd_norm"]["min"]
        >= controls["gamma"]["elevator_cmd_max_norm"] - epsilon
        or measurement["elevator_cmd_norm"]["max"]
        <= controls["gamma"]["elevator_cmd_min_norm"] + epsilon
    )
    evidence = {
        "reliable_setup_engine_fcs_and_outputs": base_reliable,
        "cas_tracking_passed": checks["cas_tracking"],
        "gamma_tracking_passed": checks["gamma_tracking"],
        "settling_passed": checks["settled_by_measurement_window"],
        "sustained_dry_max_throttle": sustained_dry_max,
        "sustained_min_throttle": sustained_min_throttle,
        "sustained_outer_loop_elevator_limit": sustained_elevator_limit,
        "physical_surface_limit_exceeded": not checks["control_surface_usage"],
        "final_cas_below_tolerance": measurement["kcas"]["end"]
        < config["reference"]["nominal_cas_kts"]
        * (1.0 - config["classification"]["cas_tracking_relative"]),
        "final_cas_above_tolerance": measurement["kcas"]["end"]
        > config["reference"]["nominal_cas_kts"]
        * (1.0 + config["classification"]["cas_tracking_relative"]),
    }
    if base_reliable and tracking_reliable:
        status = "VALID"
        failure_class = "none"
        reasons: list[str] = []
    elif not base_reliable:
        status = "UNKNOWN"
        failure_class = "trim_setup_failure" if not checks.get("trim_succeeded", False) else "other_unknown"
        reasons = [key for key in base_keys if not checks.get(key, False)]
    elif sustained_elevator_limit:
        status = "UNKNOWN"
        failure_class = "controller_limited"
        reasons = ["outer_loop_gamma_command_authority_exhausted"]
    elif (
        target_gamma > 0.0 and sustained_dry_max and not checks["cas_tracking"]
        and evidence["final_cas_below_tolerance"] and checks["gamma_tracking"]
    ) or (
        target_gamma < 0.0 and sustained_min_throttle and not checks["cas_tracking"]
        and evidence["final_cas_above_tolerance"] and checks["gamma_tracking"]
    ):
        status = "INFEASIBLE"
        failure_class = "thrust_limited"
        reasons = [
            "dry_max_throttle_with_cas_failure"
            if target_gamma > 0.0
            else "minimum_throttle_with_cas_failure"
        ]
    else:
        status = "UNKNOWN"
        failure_class = "controller_limited"
        reasons = ["tracking_or_settling_failed_without_physical_boundary_evidence"]
    record["status"] = status
    record["failure_classification"] = failure_class
    record["status_reasons"] = reasons
    record["capability_evidence"] = evidence
    return record


def _repeatability(records: list[dict[str, Any]], tolerance: float) -> dict[str, Any]:
    return j4a._repeatability(records, tolerance)


def _aggregate_point(
    altitude: float, gamma: float, records: list[dict[str, Any]],
    boundary_critical: bool, provenance_id: str, tolerance: float,
) -> dict[str, Any]:
    repeatability = _repeatability(records, tolerance)
    statuses = [record["status"] for record in records]
    classes = [record["failure_classification"] for record in records]
    if all(status == "VALID" for status in statuses) and repeatability["passed"]:
        status = "VALID"
        failure_class = "none"
    elif all(status == "INFEASIBLE" for status in statuses) and len(set(classes)) == 1 and repeatability["passed"]:
        status = "INFEASIBLE"
        failure_class = classes[0]
    else:
        status = "UNKNOWN"
        failure_class = classes[0] if len(set(classes)) == 1 else "other_unknown"
    complete = all("measurement" in record for record in records)
    measured = None
    if complete:
        def mean(key: str, stat: str = "mean") -> float:
            return fmean(record["measurement"][key][stat] for record in records)
        measured = {
            "actual_gamma_deg": mean("gamma_deg"),
            "maximum_gamma_error_deg": max(record["tracking"]["maximum_gamma_absolute_error_deg"] for record in records),
            "actual_kcas": mean("kcas"),
            "maximum_cas_error_relative": max(record["tracking"]["maximum_cas_relative_error"] for record in records),
            "tas_mps": mean("tas_mps"),
            "mach": mean("mach"),
            "vertical_speed_mps": mean("vertical_speed_mps"),
            "throttle_cmd_norm": mean("throttle_cmd_norm"),
            "throttle_pos_norm": mean("throttle_pos_norm"),
            "alpha_deg": mean("alpha_deg"),
            "beta_deg": mean("beta_deg"),
            "load_factor_nz": mean("load_factor_nz"),
            "pitch_deg": mean("pitch_deg"),
            "p_deg_s": mean("p_deg_s"),
            "q_deg_s": mean("q_deg_s"),
            "r_deg_s": mean("r_deg_s"),
            "elevator_cmd_norm": mean("elevator_cmd_norm"),
            "elevator_pos_norm": mean("elevator_pos_norm"),
            "maximum_control_surface_usage_norm": max(record["tracking"]["maximum_normalized_control_surface_usage"] for record in records),
            "settled_at_s": max(record["settling_and_measurement"]["settled_at_s"] for record in records if record["settling_and_measurement"]["settled_at_s"] is not None) if any(record["settling_and_measurement"]["settled_at_s"] is not None for record in records) else None,
            "altitude_delta_m": mean("altitude_msl_m", "delta"),
            "speed_drift_kcas": max(abs(record["measurement"]["kcas"]["delta"]) for record in records),
        }
    return {
        "provenance_id": provenance_id,
        "altitude_msl_m": altitude,
        "maneuver_family": "straight_vertical_capability",
        "direction": "level" if gamma == 0.0 else ("climb" if gamma > 0.0 else "descent"),
        "requested_target": {"speed_type": "KCAS", "speed_value": 305.0, "gamma_deg": gamma},
        "status": status,
        "failure_classification": failure_class,
        "raw_capability": None,
        "derated_capability": None,
        "boundary_critical": boundary_critical,
        "repeat_count": len(records),
        "repeatability": repeatability,
        "measured_values": measured,
        "run_ids": [record["run_id"] for record in records],
        "run_statuses": statuses,
    }


def run_j4b() -> dict[str, Any]:
    started_at = time.perf_counter()
    config = _load_yaml(CONFIG_PATH)
    j2_config = _load_yaml(J2_CONFIG_PATH)
    run_config = _build_run_config(config)
    provenance = _provenance(config, j2_config)
    audit = _controller_audit(config)
    run_cache: dict[tuple[float, float], list[dict[str, Any]]] = {}
    critical_keys: set[tuple[float, float]] = set()

    def ensure(altitude: float, gamma: float, count: int) -> dict[str, Any]:
        key = (altitude, gamma)
        records = run_cache.setdefault(key, [])
        while len(records) < count:
            repeat = len(records) + 1
            maneuver = "level" if gamma == 0.0 else ("climb" if gamma > 0.0 else "descent")
            record = j4a._run_once(
                altitude, maneuver, gamma, repeat, run_config, j2_config,
                provenance["provenance_id"],
            )
            record["run_id"] = f"j4b-{maneuver}-{altitude:.0f}m-{gamma:+.1f}deg-r{repeat}"
            records.append(_classify_run(record, gamma, config))
        return _aggregate_point(
            altitude, gamma, records, key in critical_keys,
            provenance["provenance_id"],
            run_config["acceptance"]["cold_start_repeatability_relative_approx"],
        )

    boundaries: list[dict[str, Any]] = []
    sweep = config["sweep"]
    for altitude_value in config["reference"]["main_altitudes_msl_m"]:
        altitude = float(altitude_value)
        level = ensure(altitude, 0.0, 1)
        for direction, sign in (("climb", 1.0), ("descent", -1.0)):
            last_valid_gamma = 0.0 if level["status"] == "VALID" else None
            first_nonvalid: dict[str, Any] | None = None
            magnitude = float(sweep["initial_validated_magnitude_deg"])
            while magnitude <= sweep["maximum_abs_gamma_search_guard_deg"] + 1e-12:
                gamma = sign * magnitude
                point = ensure(altitude, gamma, sweep["discovery_repetitions"])
                if point["status"] == "UNKNOWN":
                    critical_keys.add((altitude, gamma))
                    point = ensure(altitude, gamma, sweep["boundary_critical_repetitions"])
                if point["status"] == "VALID":
                    last_valid_gamma = gamma
                    magnitude += sweep["coarse_step_deg"]
                    continue
                first_nonvalid = point
                break

            if last_valid_gamma is None or first_nonvalid is None or first_nonvalid["status"] != "INFEASIBLE":
                boundaries.append(
                    {
                        "altitude_msl_m": altitude,
                        "direction": direction,
                        "status": "UNKNOWN",
                        "highest_or_lowest_validated_gamma_deg": last_valid_gamma,
                        "first_tested_non_valid_gamma_deg": None if first_nonvalid is None else first_nonvalid["requested_target"]["gamma_deg"],
                        "failure_classification": "other_unknown" if first_nonvalid is None else first_nonvalid["failure_classification"],
                        "boundary_bracket_deg": None,
                    }
                )
                continue

            valid_magnitude = abs(last_valid_gamma)
            nonvalid_magnitude = abs(first_nonvalid["requested_target"]["gamma_deg"])
            candidate_magnitude = valid_magnitude + sweep["refinement_step_deg"]
            while candidate_magnitude < nonvalid_magnitude - 1e-12:
                gamma = sign * candidate_magnitude
                point = ensure(altitude, gamma, 1)
                if point["status"] == "UNKNOWN":
                    critical_keys.add((altitude, gamma))
                    point = ensure(altitude, gamma, sweep["boundary_critical_repetitions"])
                if point["status"] == "VALID":
                    last_valid_gamma = gamma
                    candidate_magnitude += sweep["refinement_step_deg"]
                else:
                    first_nonvalid = point
                    break

            infeasible_gamma = first_nonvalid["requested_target"]["gamma_deg"]
            critical_keys.update({(altitude, last_valid_gamma), (altitude, infeasible_gamma)})
            valid_point = ensure(altitude, last_valid_gamma, sweep["boundary_critical_repetitions"])
            infeasible_point = ensure(altitude, infeasible_gamma, sweep["boundary_critical_repetitions"])
            if valid_point["status"] != "VALID" or infeasible_point["status"] != "INFEASIBLE":
                boundary_status = "UNKNOWN"
                bracket = None
            else:
                boundary_status = "BRACKETED"
                bracket = (
                    [last_valid_gamma, infeasible_gamma]
                    if direction == "climb"
                    else [infeasible_gamma, last_valid_gamma]
                )
            valid_values = valid_point["measured_values"]
            fail_values = infeasible_point["measured_values"]
            boundaries.append(
                {
                    "altitude_msl_m": altitude,
                    "direction": direction,
                    "status": boundary_status,
                    "highest_or_lowest_validated_gamma_deg": last_valid_gamma,
                    "first_tested_non_valid_gamma_deg": infeasible_gamma,
                    "boundary_bracket_deg": bracket,
                    "failure_classification": infeasible_point["failure_classification"],
                    "validated_point": {
                        "actual_gamma_deg": valid_values["actual_gamma_deg"],
                        "actual_kcas": valid_values["actual_kcas"],
                        "mach": valid_values["mach"],
                        "throttle_pos_norm": valid_values["throttle_pos_norm"],
                        "throttle_margin_to_relevant_boundary_norm": (
                            1.0 - valid_values["throttle_pos_norm"]
                            if direction == "climb" else valid_values["throttle_pos_norm"]
                        ),
                        "alpha_deg": valid_values["alpha_deg"],
                        "control_surface_usage_norm": valid_values["maximum_control_surface_usage_norm"],
                        "control_surface_reserve_norm": 0.90 - valid_values["maximum_control_surface_usage_norm"],
                        "vertical_speed_mps": valid_values["vertical_speed_mps"],
                    },
                    "first_infeasible_point": {
                        "actual_gamma_deg": fail_values["actual_gamma_deg"],
                        "actual_kcas": fail_values["actual_kcas"],
                        "maximum_cas_error_relative": fail_values["maximum_cas_error_relative"],
                        "mach": fail_values["mach"],
                        "throttle_pos_norm": fail_values["throttle_pos_norm"],
                        "alpha_deg": fail_values["alpha_deg"],
                        "control_surface_usage_norm": fail_values["maximum_control_surface_usage_norm"],
                        "vertical_speed_mps": fail_values["vertical_speed_mps"],
                    },
                    "boundary_repeatability": {
                        "validated": valid_point["repeatability"],
                        "infeasible": infeasible_point["repeatability"],
                    },
                }
            )

    all_runs = [record for records in run_cache.values() for record in records]
    all_points = [
        _aggregate_point(
            altitude, gamma, records, (altitude, gamma) in critical_keys,
            provenance["provenance_id"],
            run_config["acceptance"]["cold_start_repeatability_relative_approx"],
        )
        for (altitude, gamma), records in sorted(run_cache.items())
    ]
    unknown_points = [point for point in all_points if point["status"] == "UNKNOWN"]
    all_bracketed = len(boundaries) == 26 and all(item["status"] == "BRACKETED" for item in boundaries)
    result = {
        "provenance_id": provenance["provenance_id"],
        "parent_j4a_provenance_id": provenance["parent_j4a_provenance_id"],
        "j4b_pass": all_bracketed and not unknown_points,
        "altitude_count": len(config["reference"]["main_altitudes_msl_m"]),
        "boundary_count": len(boundaries),
        "bracketed_boundary_count": sum(item["status"] == "BRACKETED" for item in boundaries),
        "run_count": len(all_runs),
        "tested_point_count": len(all_points),
        "status_counts": {
            status: sum(point["status"] == status for point in all_points)
            for status in ("VALID", "INFEASIBLE", "UNKNOWN")
        },
        "boundary_critical_point_count": sum(point["boundary_critical"] for point in all_points),
        "unique_discovery_point_runs": len(all_points),
        "additional_boundary_repeat_runs": len(all_runs) - len(all_points),
        "runtime_wall_s": time.perf_counter() - started_at,
        "unknown_points": [
            {"altitude_msl_m": point["altitude_msl_m"], "gamma_deg": point["requested_target"]["gamma_deg"], "failure_classification": point["failure_classification"]}
            for point in unknown_points
        ],
        "scope_guards": config["scope"],
        "j5_blocker": None if all_bracketed and not unknown_points else "resolve_unknown_or_unbracketed_vertical_boundaries",
    }
    artifacts = {
        "j4b_provenance.json": provenance,
        "j4b_controller_audit.json": audit,
        "j4b_runs.json": all_runs,
        "j4b_tested_points.json": all_points,
        "j4b_boundaries.json": boundaries,
        "j4b_result.json": result,
    }
    for filename, payload in artifacts.items():
        (RESULTS_DIR / filename).write_text(
            json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
    return result


if __name__ == "__main__":
    result = run_j4b()
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    raise SystemExit(0 if result["j4b_pass"] else 1)
