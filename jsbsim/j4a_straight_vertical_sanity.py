"""J4A straight climb/descent methodology sanity gate; not an envelope sweep."""

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
import j3_nominal_cas_selection as j3


HERE = Path(__file__).resolve().parent
RESULTS_DIR = HERE / "results"
CONFIG_PATH = HERE / "j4a_sanity_configuration.yaml"
J2_CONFIG_PATH = HERE / "j2_reference_configuration.yaml"
PARENT_PROVENANCE_PATH = RESULTS_DIR / "j3_1_provenance.json"


def _load_yaml(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _provenance(config: dict[str, Any], j2_config: dict[str, Any]) -> dict[str, Any]:
    parent = json.loads(PARENT_PROVENANCE_PATH.read_text(encoding="utf-8"))
    if parent["provenance_id"] != config["parent_provenance_id"]:
        raise RuntimeError("J3.1 parent provenance mismatch")
    base = j2._provenance(j2_config)
    files = {
        "j4a_configuration": CONFIG_PATH,
        "j4a_harness": Path(__file__).resolve(),
        "parent_j3_1_provenance": PARENT_PROVENANCE_PATH,
    }
    hashes = {
        name: {"path": str(path), "sha256": _sha256(path)}
        for name, path in files.items()
    }
    fingerprint = {
        "parent": parent["provenance_id"],
        "base_model": base["provenance_id"],
        "files": {name: item["sha256"] for name, item in hashes.items()},
        "altitudes": config["reference"]["representative_altitudes_msl_m"],
        "gamma_targets": config["reference"]["gamma_targets_deg"],
    }
    provenance_id = "j4a-" + hashlib.sha256(
        json.dumps(fingerprint, sort_keys=True).encode("utf-8")
    ).hexdigest()[:20]
    return {
        "provenance_id": provenance_id,
        "parent_j3_1_provenance_id": parent["provenance_id"],
        "base_model_provenance_id": base["provenance_id"],
        "base_model_provenance": base,
        "files": hashes,
        "controller_interpretation": config["outer_loop_sanity_controller"]["interpretation"],
    }


def _snapshot(fdm: Any, elapsed_s: float) -> dict[str, float]:
    sample = j2._snapshot(fdm, elapsed_s)
    sample.update(
        {
            "dynamic_pressure_pa": fdm["aero/qbar-psf"] * 47.88025898033584,
            "elevator_cmd_norm": fdm["fcs/elevator-cmd-norm"],
            "aileron_cmd_norm": fdm["fcs/aileron-cmd-norm"],
            "rudder_cmd_norm": fdm["fcs/rudder-cmd-norm"],
            "pitch_trim_cmd_norm": fdm["fcs/pitch-trim-cmd-norm"],
            "roll_trim_cmd_norm": fdm["fcs/roll-trim-cmd-norm"],
            "yaw_trim_cmd_norm": fdm["fcs/yaw-trim-cmd-norm"],
            "pitch_deg": fdm["attitude/pitch-rad"] * j2.RAD_TO_DEG,
            "vertical_speed_mps": fdm["velocities/h-dot-fps"] * j2.FPS_TO_MPS,
            "terrain_elevation_msl_m": (
                fdm["position/terrain-elevation-asl-ft"] * j2.FPS_TO_MPS
            ),
            "height_agl_m": fdm["position/h-agl-ft"] * j2.FPS_TO_MPS,
        }
    )
    return sample


def _earliest_settled_time(
    samples: list[dict[str, float]], target_kcas: float, target_gamma: float,
    config: dict[str, Any],
) -> float | None:
    window = config["simulation"]["steady_measurement_window_s"]
    cas_tolerance = config["acceptance"]["cas_tracking_relative"]
    gamma_tolerance = config["acceptance"]["gamma_tracking_absolute_deg"]
    final_time = samples[-1]["elapsed_s"]
    for index, sample in enumerate(samples):
        if final_time - sample["elapsed_s"] + 1e-9 < window:
            break
        remaining = samples[index:]
        if all(
            j2._relative_error(item["kcas"], target_kcas) <= cas_tolerance
            and abs(item["gamma_deg"] - target_gamma) <= gamma_tolerance
            for item in remaining
        ):
            return sample["elapsed_s"]
    return None


def _run_once(
    altitude_m: float, maneuver: str, target_gamma: float, repeat: int,
    config: dict[str, Any], j2_config: dict[str, Any], provenance_id: str,
) -> dict[str, Any]:
    target_kcas = float(config["reference"]["nominal_cas_kts"])
    dt = float(config["simulation"]["timestep_s"])
    record: dict[str, Any] = {
        "provenance_id": provenance_id,
        "run_id": f"j4a-{maneuver}-{altitude_m:.0f}m-r{repeat}",
        "repeat_index": repeat,
        "requested": {
            "altitude_msl_m": altitude_m,
            "maneuver": maneuver,
            "speed_type": "KCAS",
            "speed_value": target_kcas,
            "gamma_deg": target_gamma,
        },
        "status": "UNKNOWN",
        "status_reasons": [],
        "confirmed_unsustainable": False,
    }
    try:
        fdm = j2.jsbsim.FGFDMExec(None)
        fdm.set_debug_level(0)
        fdm.set_dt(dt)
        if not fdm.load_model(j2_config["aircraft"]["model"]):
            raise RuntimeError("load_model returned false")
        j2._set_reference_properties(fdm, j2_config)
        terrain_m = config["airborne_test_fixture"]["synthetic_terrain_elevation_msl_m"]
        fdm["position/terrain-elevation-asl-ft"] = terrain_m * j2.M_TO_FT
        fdm["ic/h-sl-ft"] = altitude_m * j2.M_TO_FT
        fdm["ic/vc-kts"] = target_kcas
        fdm["ic/gamma-deg"] = 0.0
        fdm["ic/phi-deg"] = 0.0
        fdm["ic/theta-deg"] = 0.0
        fdm["ic/psi-true-deg"] = 0.0
        if not fdm.run_ic():
            raise RuntimeError("run_ic returned false")
        j2._set_reference_properties(fdm, j2_config)
        fdm["position/terrain-elevation-asl-ft"] = terrain_m * j2.M_TO_FT
        fdm["propulsion/engine/set-running"] = 1.0
        fdm["propulsion/fuel_freeze"] = 1.0
        if not fdm.run():
            raise RuntimeError("initialized frame failed")
        initialized = _snapshot(fdm, 0.0)
        try:
            fdm.do_trim(1)
            trim_succeeded = True
            trim_error = None
        except (j2.jsbsim.TrimFailureError, RuntimeError) as exc:
            trim_succeeded = False
            trim_error = f"{type(exc).__name__}: {exc}"
        record["trim"] = {
            "mode": "full_level_trim_before_maneuver",
            "succeeded": trim_succeeded,
            "error": trim_error,
            "trim_completed_property": fdm["simulation/trim-completed"],
        }
        record["initialized"] = initialized
        record["reference_configuration_after_trim"] = j2._configuration_snapshot(fdm)
        if not trim_succeeded:
            record["status_reasons"].append("trim_failure_is_unknown_not_infeasible")
            return record

        baseline = {
            "throttle_cmd_norm": fdm["fcs/throttle-cmd-norm"],
            "throttle_pos_norm": fdm["fcs/throttle-pos-norm"],
            "pitch_trim_cmd_norm": fdm["fcs/pitch-trim-cmd-norm"],
            "elevator_pos_norm": fdm["fcs/elevator-pos-norm"],
            "gamma_deg": fdm["flight-path/gamma-deg"],
            "kcas": fdm["velocities/vc-kts"],
        }
        gamma_control = config["outer_loop_sanity_controller"]["gamma"]
        cas_control = config["outer_loop_sanity_controller"]["cas"]
        gamma_integral = 0.0
        cas_integral = 0.0
        samples: list[dict[str, float]] = []
        steps = round(config["simulation"]["replay_duration_s"] / dt)
        ramp_s = config["simulation"]["gamma_target_ramp_s"]
        elevator_command_saturated = False
        throttle_command_saturated = False
        for step in range(1, steps + 1):
            elapsed_s = step * dt
            commanded_gamma = target_gamma * min(1.0, elapsed_s / ramp_s)
            gamma_error = commanded_gamma - fdm["flight-path/gamma-deg"]
            cas_error = target_kcas - fdm["velocities/vc-kts"]
            gamma_integral = _clamp(
                gamma_integral + gamma_error * dt,
                -gamma_control["integral_limit_deg_s"],
                gamma_control["integral_limit_deg_s"],
            )
            cas_integral = _clamp(
                cas_integral + cas_error * dt,
                -cas_control["integral_limit_knot_s"],
                cas_control["integral_limit_knot_s"],
            )
            raw_elevator = (
                gamma_control["proportional_elevator_cmd_per_deg"] * gamma_error
                + gamma_control["integral_elevator_cmd_per_deg_s"] * gamma_integral
            )
            elevator_cmd = _clamp(
                raw_elevator,
                gamma_control["elevator_cmd_min_norm"],
                gamma_control["elevator_cmd_max_norm"],
            )
            raw_throttle = (
                baseline["throttle_cmd_norm"]
                + cas_control["proportional_throttle_cmd_per_knot"] * cas_error
                + cas_control["integral_throttle_cmd_per_knot_s"] * cas_integral
            )
            throttle_cmd = _clamp(
                raw_throttle,
                cas_control["throttle_cmd_min_norm"],
                cas_control["throttle_cmd_max_dry_norm"],
            )
            elevator_command_saturated |= not math.isclose(raw_elevator, elevator_cmd, abs_tol=1e-12)
            throttle_command_saturated |= not math.isclose(raw_throttle, throttle_cmd, abs_tol=1e-12)
            fdm["fcs/elevator-cmd-norm"] = elevator_cmd
            fdm["fcs/throttle-cmd-norm"] = throttle_cmd
            if not fdm.run():
                raise RuntimeError("replay terminated early")
            sample = _snapshot(fdm, elapsed_s)
            sample["commanded_gamma_deg"] = commanded_gamma
            sample["gamma_error_deg"] = target_gamma - sample["gamma_deg"]
            sample["cas_error_kts"] = target_kcas - sample["kcas"]
            samples.append(sample)

        measurement_start = (
            config["simulation"]["replay_duration_s"]
            - config["simulation"]["steady_measurement_window_s"]
        )
        measurement = [item for item in samples if item["elapsed_s"] >= measurement_start]
        numeric_keys = [key for key in measurement[0] if key not in {"elapsed_s", "sim_time_s"}]
        finite = j2._all_finite(item[key] for item in samples for key in numeric_keys)
        summary = {key: j2._stats([item[key] for item in measurement]) for key in numeric_keys}
        settled_at = _earliest_settled_time(samples, target_kcas, target_gamma, config)
        max_cas_error = max(j2._relative_error(item["kcas"], target_kcas) for item in measurement)
        max_gamma_error = max(abs(item["gamma_deg"] - target_gamma) for item in measurement)
        surface_keys = (
            "elevator_pos_norm", "left_aileron_pos_norm", "right_aileron_pos_norm",
            "rudder_pos_norm", "speedbrake_pos_norm",
        )
        max_surface = max(abs(item[key]) for item in measurement for key in surface_keys)
        altitude_delta = summary["altitude_msl_m"]["delta"]
        vertical_speed_mean = summary["vertical_speed_mps"]["mean"]
        gamma_mean = summary["gamma_deg"]["mean"]
        if target_gamma > 0.0:
            sign_checks = {
                "gamma_sign": gamma_mean > 0.0,
                "vertical_speed_sign": vertical_speed_mean > 0.0,
                "altitude_evolution_sign": altitude_delta > 0.0,
            }
        elif target_gamma < 0.0:
            sign_checks = {
                "gamma_sign": gamma_mean < 0.0,
                "vertical_speed_sign": vertical_speed_mean < 0.0,
                "altitude_evolution_sign": altitude_delta < 0.0,
            }
        else:
            sign_checks = {
                "gamma_sign": abs(gamma_mean) <= config["acceptance"]["gamma_tracking_absolute_deg"],
                "vertical_speed_sign": abs(vertical_speed_mean) <= max(
                    abs(summary["tas_mps"]["mean"] * math.sin(math.radians(config["acceptance"]["gamma_tracking_absolute_deg"]))),
                    1e-9,
                ),
                "altitude_evolution_sign": abs(altitude_delta) <= (
                    config["simulation"]["steady_measurement_window_s"]
                    * summary["tas_mps"]["mean"]
                    * math.sin(math.radians(config["acceptance"]["gamma_tracking_absolute_deg"]))
                ),
            }
        expected = j2_config["mass_and_fuel"]
        checks = {
            "trim_succeeded": trim_succeeded,
            "finite_outputs": finite,
            "settled_by_measurement_window": settled_at is not None and settled_at <= measurement_start + dt,
            "measurement_window_complete": len(measurement) >= round(config["simulation"]["steady_measurement_window_s"] / dt),
            "cas_tracking": max_cas_error <= config["acceptance"]["cas_tracking_relative"],
            "gamma_tracking": max_gamma_error <= config["acceptance"]["gamma_tracking_absolute_deg"],
            "control_surface_usage": max_surface <= config["acceptance"]["normalized_control_surface_usage_max"],
            "outer_loop_elevator_not_saturated": not elevator_command_saturated,
            "outer_loop_throttle_not_saturated": not throttle_command_saturated,
            "afterburner_off": summary["throttle_pos_norm"]["max"] <= config["acceptance"]["afterburner_off_max_throttle_position_norm"] + 1e-12,
            "engine_running": summary["engine_running_flag"]["min"] >= 1.0 and summary["engine_stalled"]["max"] == 0.0 and summary["engine_seized"]["max"] == 0.0,
            "gear_up": summary["gear_pos_norm"]["max"] <= 1e-12,
            "speedbrake_closed": summary["speedbrake_pos_norm"]["max"] <= 1e-12,
            "native_fcs": summary["fbw_override"]["max"] == 0.0,
            "zero_wind": max(
                abs(summary[key][bound])
                for key in ("wind_north_fps", "wind_east_fps", "wind_down_fps")
                for bound in ("min", "max")
            ) <= config["acceptance"]["zero_wind_residual_fps"],
            "fixed_weight": abs(summary["weight_lbs"]["mean"] - expected["expected_total_weight_lbs"]) <= 1e-6,
            "fixed_fuel": abs(summary["total_fuel_lbs"]["mean"] - expected["expected_total_fuel_lbs"]) <= 1e-6,
            **sign_checks,
        }
        reliable = all(checks.values())
        record.update(
            {
                "baseline_level_trim": baseline,
                "measurement": summary,
                "tracking": {
                    "maximum_cas_relative_error": max_cas_error,
                    "maximum_gamma_absolute_error_deg": max_gamma_error,
                    "maximum_normalized_control_surface_usage": max_surface,
                },
                "settling_and_measurement": {
                    "settled_at_s": settled_at,
                    "measurement_start_s": measurement_start,
                    "measurement_duration_s": config["simulation"]["steady_measurement_window_s"],
                    "measurement_sample_count": len(measurement),
                },
                "controller": {
                    "elevator_command_saturated": elevator_command_saturated,
                    "throttle_command_saturated": throttle_command_saturated,
                },
                "checks": checks,
                "status": j2.classify_characterization_result(
                    reliable_measurement=reliable, confirmed_unsustainable=False
                ),
                "status_reasons": [name for name, passed in checks.items() if not passed],
            }
        )
        return record
    except Exception as exc:
        record["status"] = "UNKNOWN"
        record["status_reasons"].append(f"{type(exc).__name__}: {exc}")
        return record


def _repeatability(records: list[dict[str, Any]], tolerance: float) -> dict[str, Any]:
    paths = {
        "actual_kcas": ("measurement", "kcas", "mean"),
        "actual_gamma_deg": ("measurement", "gamma_deg", "mean"),
        "vertical_speed_mps": ("measurement", "vertical_speed_mps", "mean"),
        "throttle_pos_norm": ("measurement", "throttle_pos_norm", "mean"),
        "alpha_deg": ("measurement", "alpha_deg", "mean"),
        "load_factor_nz": ("measurement", "load_factor_nz", "mean"),
    }
    if any("measurement" not in record for record in records):
        return {"passed": False, "tolerance_relative": tolerance, "metrics": {}}
    metrics = {}
    for name, path in paths.items():
        values = [record[path[0]][path[1]][path[2]] for record in records]
        mean = fmean(values)
        scale = max(abs(mean), 1e-9)
        maximum_relative_deviation = max(abs(value - mean) / scale for value in values)
        metrics[name] = {
            "values": values,
            "mean": mean,
            "maximum_relative_deviation": maximum_relative_deviation,
            "passed": maximum_relative_deviation <= tolerance,
        }
    return {
        "passed": all(item["passed"] for item in metrics.values()),
        "tolerance_relative": tolerance,
        "metrics": metrics,
    }


def _point_record(
    altitude: float, maneuver: str, target_gamma: float, records: list[dict[str, Any]],
    repeatability: dict[str, Any], provenance_id: str,
) -> dict[str, Any]:
    if not repeatability["passed"]:
        for record in records:
            if record["status"] == "VALID":
                record["status"] = "UNKNOWN"
                record["status_reasons"].append("cold_start_repeatability_failed")
    complete = all("measurement" in record for record in records)
    status = "VALID" if complete and repeatability["passed"] and all(record["status"] == "VALID" for record in records) else "UNKNOWN"
    measured = None
    if complete:
        measured = {
            key: fmean(record["measurement"][source][stat] for record in records)
            for key, source, stat in (
                ("actual_gamma_deg", "gamma_deg", "mean"),
                ("actual_kcas", "kcas", "mean"),
                ("tas_mps", "tas_mps", "mean"),
                ("mach", "mach", "mean"),
                ("altitude_msl_m", "altitude_msl_m", "mean"),
                ("altitude_delta_m", "altitude_msl_m", "delta"),
                ("vertical_speed_mps", "vertical_speed_mps", "mean"),
                ("throttle_cmd_norm", "throttle_cmd_norm", "mean"),
                ("throttle_pos_norm", "throttle_pos_norm", "mean"),
                ("alpha_deg", "alpha_deg", "mean"),
                ("beta_deg", "beta_deg", "mean"),
                ("load_factor_nz", "load_factor_nz", "mean"),
                ("pitch_deg", "pitch_deg", "mean"),
                ("p_deg_s", "p_deg_s", "mean"),
                ("q_deg_s", "q_deg_s", "mean"),
                ("r_deg_s", "r_deg_s", "mean"),
                ("elevator_pos_norm", "elevator_pos_norm", "mean"),
            )
        }
        measured.update(
            {
                "maximum_cas_error_relative": max(record["tracking"]["maximum_cas_relative_error"] for record in records),
                "maximum_gamma_error_deg": max(record["tracking"]["maximum_gamma_absolute_error_deg"] for record in records),
                "maximum_control_surface_usage_norm": max(record["tracking"]["maximum_normalized_control_surface_usage"] for record in records),
                "maximum_speed_drift_kcas": max(abs(record["measurement"]["kcas"]["delta"]) for record in records),
                "settled_at_s": max(record["settling_and_measurement"]["settled_at_s"] for record in records),
            }
        )
    return {
        "provenance_id": provenance_id,
        "altitude_msl_m": altitude,
        "maneuver_family": "straight_vertical_sanity",
        "maneuver": maneuver,
        "requested_target": {"speed_type": "KCAS", "speed_value": 305.0, "gamma_deg": target_gamma},
        "status": status,
        "raw_capability": None,
        "derated_capability": None,
        "measured_values": measured,
        "diagnostics": {
            "repeatability": repeatability,
            "run_ids": [record["run_id"] for record in records],
            "run_statuses": [record["status"] for record in records],
            "run_status_reasons": [record["status_reasons"] for record in records],
            "status_reasons": [],
        },
    }


def run_j4a() -> dict[str, Any]:
    config = _load_yaml(CONFIG_PATH)
    j2_config = _load_yaml(J2_CONFIG_PATH)
    provenance = _provenance(config, j2_config)
    all_runs: list[dict[str, Any]] = []
    points: list[dict[str, Any]] = []
    targets = config["reference"]["gamma_targets_deg"]
    for altitude in config["reference"]["representative_altitudes_msl_m"]:
        for maneuver, target_gamma in targets.items():
            records = [
                _run_once(float(altitude), maneuver, float(target_gamma), repeat, config, j2_config, provenance["provenance_id"])
                for repeat in range(1, config["reference"]["cold_start_repetitions_per_point"] + 1)
            ]
            repeatability = _repeatability(
                records, config["acceptance"]["cold_start_repeatability_relative_approx"]
            )
            point = _point_record(
                float(altitude), maneuver, float(target_gamma), records,
                repeatability, provenance["provenance_id"],
            )
            all_runs.extend(records)
            points.append(point)

    throttle_ordering = []
    for altitude in config["reference"]["representative_altitudes_msl_m"]:
        group = {point["maneuver"]: point for point in points if point["altitude_msl_m"] == altitude}
        complete = all(point["measured_values"] is not None for point in group.values())
        passed = complete and (
            group["modest_climb"]["measured_values"]["throttle_pos_norm"]
            > group["level"]["measured_values"]["throttle_pos_norm"]
            > group["modest_descent"]["measured_values"]["throttle_pos_norm"]
        )
        throttle_ordering.append(
            {
                "altitude_msl_m": altitude,
                "passed": passed,
                "descent_throttle_pos_norm": group["modest_descent"]["measured_values"]["throttle_pos_norm"] if complete else None,
                "level_throttle_pos_norm": group["level"]["measured_values"]["throttle_pos_norm"] if complete else None,
                "climb_throttle_pos_norm": group["modest_climb"]["measured_values"]["throttle_pos_norm"] if complete else None,
            }
        )
        for point in group.values():
            point["diagnostics"]["throttle_response_ordering_passed"] = passed
        if not passed:
            for point in group.values():
                if point["status"] == "VALID":
                    point["status"] = "UNKNOWN"
                    point["diagnostics"]["status_reasons"].append(
                        "throttle_response_ordering_failed"
                    )

    all_points_valid = all(point["status"] == "VALID" for point in points)
    all_runs_valid = all(run["status"] == "VALID" for run in all_runs)
    repeatable = all(point["diagnostics"]["repeatability"]["passed"] for point in points)
    sign_valid = all(
        all(run.get("checks", {}).get(key, False) for key in ("gamma_sign", "vertical_speed_sign", "altitude_evolution_sign"))
        for run in all_runs
    )
    result = {
        "provenance_id": provenance["provenance_id"],
        "parent_j3_1_provenance_id": provenance["parent_j3_1_provenance_id"],
        "j4a_pass": all_points_valid and all_runs_valid and repeatable and sign_valid and all(item["passed"] for item in throttle_ordering),
        "run_count": len(all_runs),
        "point_count": len(points),
        "status_counts": {
            status: sum(point["status"] == status for point in points)
            for status in ("VALID", "UNKNOWN", "INFEASIBLE")
        },
        "gamma_targets_deg": targets,
        "gamma_targets_are_capability_limits": False,
        "throttle_ordering": throttle_ordering,
        "sign_validation_passed": sign_valid,
        "repeatability_passed": repeatable,
        "scope_guards": config["scope"],
        "j4b_blocker": None if all_points_valid and all_runs_valid and repeatable and sign_valid and all(item["passed"] for item in throttle_ordering) else "resolve_j4a_unknown_results",
    }
    artifacts = {
        "j4a_provenance.json": provenance,
        "j4a_runs.json": all_runs,
        "j4a_points.json": points,
        "j4a_result.json": result,
    }
    for filename, payload in artifacts.items():
        (RESULTS_DIR / filename).write_text(
            json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
    return result


if __name__ == "__main__":
    result = run_j4a()
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    raise SystemExit(0 if result["j4a_pass"] else 1)
