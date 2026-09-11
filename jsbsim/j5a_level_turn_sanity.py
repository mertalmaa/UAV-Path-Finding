"""J5A level coordinated-turn methodology sanity gate; not an envelope sweep."""

from __future__ import annotations

import hashlib
import json
import math
import time
from pathlib import Path
from statistics import fmean
from typing import Any

import yaml

import j2_baseline_sanity as j2
import j4a_straight_vertical_sanity as j4a


HERE = Path(__file__).resolve().parent
RESULTS_DIR = HERE / "results"
CONFIG_PATH = HERE / "j5a_level_turn_sanity_configuration.yaml"
J2_CONFIG_PATH = HERE / "j2_reference_configuration.yaml"
J4B_CONFIG_PATH = HERE / "j4b_capability_configuration.yaml"
PARENT_PROVENANCE_PATH = RESULTS_DIR / "j4b_provenance.json"


def _load_yaml(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _relative_error(value: float, reference: float) -> float:
    return abs(value - reference) / max(abs(reference), 1.0e-12)


def classify_turn_sanity_result(
    reliable_measurement: bool, confirmed_aircraft_inability: bool = False,
) -> str:
    """Apply J1 three-state semantics without treating modest-target failure as a limit."""
    return j2.classify_characterization_result(
        reliable_measurement=reliable_measurement,
        confirmed_unsustainable=confirmed_aircraft_inability,
    )


def _provenance(config: dict[str, Any], j2_config: dict[str, Any]) -> dict[str, Any]:
    parent = json.loads(PARENT_PROVENANCE_PATH.read_text(encoding="utf-8"))
    if parent["provenance_id"] != config["parent_provenance_id"]:
        raise RuntimeError("J4B parent provenance mismatch")
    base = j2._provenance(j2_config)
    files = {
        "j5a_configuration": CONFIG_PATH,
        "j5a_harness": Path(__file__).resolve(),
        "j4b_configuration": J4B_CONFIG_PATH,
        "j4b_harness": HERE / "j4b_straight_vertical_capability.py",
        "parent_j4b_provenance": PARENT_PROVENANCE_PATH,
    }
    hashes = {
        name: {"path": str(path), "sha256": _sha256(path)}
        for name, path in files.items()
    }
    fingerprint = {
        "parent": parent["provenance_id"],
        "base_model": base["provenance_id"],
        "files": {name: item["sha256"] for name, item in hashes.items()},
        "reference": config["reference"],
        "simulation": config["simulation"],
        "controller": config["controller"],
        "measurement": config["measurement"],
        "acceptance": config["acceptance"],
    }
    provenance_id = "j5a-" + hashlib.sha256(
        json.dumps(fingerprint, sort_keys=True).encode("utf-8")
    ).hexdigest()[:20]
    return {
        "provenance_id": provenance_id,
        "parent_j4b_provenance_id": parent["provenance_id"],
        "base_model_provenance_id": base["provenance_id"],
        "base_model_provenance": base,
        "files": hashes,
    }


def _controller_audit(config: dict[str, Any]) -> dict[str, Any]:
    controller = config["controller"]
    return {
        "signal_chain": {
            "turn": "target_bank -> PI outer loop -> fcs/aileron-cmd-norm -> native F-16 roll-rate FCS -> ailerons/flaperons -> aircraft",
            "level": "target_gamma_0 -> J4B PI outer loop -> fcs/elevator-cmd-norm -> native F-16 pitch FCS -> elevator -> aircraft",
            "cas": "target_305_KCAS -> J4B PI outer loop -> fcs/throttle-cmd-norm -> FCS throttle gain -> engine -> aircraft",
            "coordination": "target_beta_0 -> PI outer loop -> fcs/rudder-cmd-norm -> native F-16 yaw-rate/yaw-load FCS -> rudder -> aircraft",
        },
        "turn_target_quantity": controller["turn_outer_loop_quantity"],
        "heading_rate_commanded": False,
        "lateral_stick_equivalent_command": controller["lateral_command"],
        "rudder_command_policy": controller["rudder_policy"],
        "rudder_managed_by_native_fcs": True,
        "altitude_gamma_hold": "gamma=0 PI elevator outer loop reused from J4B",
        "cas_throttle_controller": "same gains and dry command bounds as J4B",
        "native_fcs_active": controller["native_fcs_active"],
        "fbw_override": controller["fbw_override"],
        "direct_surface_position_commanded": controller["direct_surface_position_commanded"],
        "anti_windup": controller["anti_windup"],
        "command_limits": {
            "aileron_cmd_norm": [
                controller["bank"]["aileron_cmd_min_norm"],
                controller["bank"]["aileron_cmd_max_norm"],
            ],
            "rudder_cmd_norm": [
                controller["beta"]["rudder_cmd_min_norm"],
                controller["beta"]["rudder_cmd_max_norm"],
            ],
            "elevator_cmd_norm": [
                controller["gamma"]["elevator_cmd_min_norm"],
                controller["gamma"]["elevator_cmd_max_norm"],
            ],
            "throttle_cmd_norm": [
                controller["cas"]["throttle_cmd_min_norm"],
                controller["cas"]["throttle_cmd_max_dry_norm"],
            ],
        },
    }


def _snapshot(fdm: Any, elapsed_s: float) -> dict[str, float]:
    sample = j4a._snapshot(fdm, elapsed_s)
    roll_rad = fdm["attitude/roll-rad"]
    pitch_rad = fdm["attitude/pitch-rad"]
    q_rad_s = fdm["velocities/q-rad_sec"]
    r_rad_s = fdm["velocities/r-rad_sec"]
    heading_rate_rad_s = (
        q_rad_s * math.sin(roll_rad) + r_rad_s * math.cos(roll_rad)
    ) / max(math.cos(pitch_rad), 1.0e-9)
    sample.update(
        {
            "roll_deg": roll_rad * j2.RAD_TO_DEG,
            "heading_deg": fdm["attitude/heading-true-rad"] * j2.RAD_TO_DEG,
            "heading_rate_deg_s": heading_rate_rad_s * j2.RAD_TO_DEG,
            "north_from_start_m": fdm["position/distance-from-start-lat-mt"],
            "east_from_start_m": fdm["position/distance-from-start-lon-mt"],
            "ground_speed_mps": math.hypot(
                fdm["velocities/v-north-fps"], fdm["velocities/v-east-fps"]
            ) * j2.FPS_TO_MPS,
        }
    )
    return sample


def _unwrap_degrees(values: list[float]) -> list[float]:
    if not values:
        return []
    unwrapped = [values[0]]
    for value in values[1:]:
        delta = (value - unwrapped[-1] + 180.0) % 360.0 - 180.0
        unwrapped.append(unwrapped[-1] + delta)
    return unwrapped


def _linear_slope(xs: list[float], ys: list[float]) -> float:
    x_mean = fmean(xs)
    y_mean = fmean(ys)
    denominator = sum((value - x_mean) ** 2 for value in xs)
    if denominator <= 0.0:
        raise ValueError("cannot fit slope to zero-duration data")
    return sum(
        (x_value - x_mean) * (y_value - y_mean)
        for x_value, y_value in zip(xs, ys)
    ) / denominator


def _trajectory_metrics(samples: list[dict[str, float]], target_bank_deg: float) -> dict[str, Any]:
    times = [sample["elapsed_s"] for sample in samples]
    headings = _unwrap_degrees([sample["heading_deg"] for sample in samples])
    heading_rate_deg_s = _linear_slope(times, headings)
    arc_length_m = 0.0
    course_angles: list[float] = []
    for previous, current in zip(samples, samples[1:]):
        north_delta = current["north_from_start_m"] - previous["north_from_start_m"]
        east_delta = current["east_from_start_m"] - previous["east_from_start_m"]
        distance = math.hypot(north_delta, east_delta)
        arc_length_m += distance
        if distance > 1.0e-9:
            course_angles.append(math.degrees(math.atan2(east_delta, north_delta)))
    courses = _unwrap_degrees(course_angles)
    ground_track_change_deg = courses[-1] - courses[0] if len(courses) >= 2 else 0.0
    if target_bank_deg == 0.0 or abs(ground_track_change_deg) <= 1.0e-9:
        radius_m = None
    else:
        radius_m = arc_length_m / abs(math.radians(ground_track_change_deg))
    mean_tas = fmean(sample["tas_mps"] for sample in samples)
    mean_bank = fmean(sample["roll_deg"] for sample in samples)
    tangent = math.tan(math.radians(abs(mean_bank)))
    theory_radius_m = None if target_bank_deg == 0.0 or tangent <= 1.0e-12 else (
        mean_tas ** 2 / (9.80665 * tangent)
    )
    radius_theory_relative_difference = None
    if radius_m is not None and theory_radius_m is not None:
        radius_theory_relative_difference = abs(radius_m - theory_radius_m) / theory_radius_m
    mean_rate = fmean(sample["heading_rate_deg_s"] for sample in samples)
    maximum_rate_relative_deviation = None
    if target_bank_deg != 0.0:
        maximum_rate_relative_deviation = max(
            abs(sample["heading_rate_deg_s"] - mean_rate) for sample in samples
        ) / max(abs(mean_rate), 1.0e-12)
    return {
        "unwrapped_heading_change_deg": headings[-1] - headings[0],
        "measured_turn_rate_deg_s": heading_rate_deg_s,
        "mean_kinematic_heading_rate_deg_s": mean_rate,
        "maximum_turn_rate_relative_deviation": maximum_rate_relative_deviation,
        "horizontal_trajectory_arc_length_m": arc_length_m,
        "unwrapped_ground_track_change_deg": ground_track_change_deg,
        "measured_trajectory_radius_m": radius_m,
        "theoretical_coordinated_radius_m": theory_radius_m,
        "radius_theory_relative_difference": radius_theory_relative_difference,
    }


def _earliest_tracking_time(
    samples: list[dict[str, float]], target_kcas: float, target_bank_deg: float,
    config: dict[str, Any],
) -> float | None:
    acceptance = config["acceptance"]
    window_s = config["simulation"]["steady_measurement_window_s"]
    final_time = samples[-1]["elapsed_s"]
    for index, sample in enumerate(samples):
        if final_time - sample["elapsed_s"] + 1.0e-9 < window_s:
            break
        remaining = samples[index:]
        if all(
            j2._relative_error(item["kcas"], target_kcas)
            <= acceptance["cas_tracking_relative"]
            and abs(item["gamma_deg"]) <= acceptance["gamma_tracking_absolute_deg"]
            and abs(item["roll_deg"] - target_bank_deg)
            <= acceptance["bank_tracking_absolute_deg"]
            for item in remaining
        ):
            return sample["elapsed_s"]
    return None


def _run_once(
    altitude_m: float, maneuver: str, target_bank_deg: float, repeat: int,
    config: dict[str, Any], j2_config: dict[str, Any], provenance_id: str,
) -> dict[str, Any]:
    target_kcas = float(config["reference"]["nominal_cas_kts"])
    dt = float(config["simulation"]["timestep_s"])
    record: dict[str, Any] = {
        "provenance_id": provenance_id,
        "run_id": f"j5a-{maneuver}-{altitude_m:.0f}m-r{repeat}",
        "repeat_index": repeat,
        "requested": {
            "altitude_msl_m": altitude_m,
            "maneuver": maneuver,
            "speed_type": "KCAS",
            "speed_value": target_kcas,
            "gamma_deg": 0.0,
            "bank_deg": target_bank_deg,
            "heading_rate_commanded": False,
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
            "mode": "full_level_trim_before_turn",
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
            "roll_trim_cmd_norm": fdm["fcs/roll-trim-cmd-norm"],
            "yaw_trim_cmd_norm": fdm["fcs/yaw-trim-cmd-norm"],
            "kcas": fdm["velocities/vc-kts"],
        }
        bank_control = config["controller"]["bank"]
        beta_control = config["controller"]["beta"]
        gamma_control = config["controller"]["gamma"]
        cas_control = config["controller"]["cas"]
        bank_integral = beta_integral = gamma_integral = cas_integral = 0.0
        samples: list[dict[str, float]] = []
        steps = round(config["simulation"]["replay_duration_s"] / dt)
        ramp_s = float(config["simulation"]["bank_target_ramp_s"])
        command_saturation_any = {"aileron": False, "rudder": False, "elevator": False, "throttle": False}
        command_saturation_measurement = {"aileron": False, "rudder": False, "elevator": False, "throttle": False}
        measurement_start = (
            config["simulation"]["replay_duration_s"]
            - config["simulation"]["steady_measurement_window_s"]
        )
        epsilon = config["acceptance"]["measurement_command_boundary_epsilon_norm"]
        for step in range(1, steps + 1):
            elapsed_s = step * dt
            commanded_bank = target_bank_deg * min(1.0, elapsed_s / ramp_s)
            bank_error = commanded_bank - fdm["attitude/roll-rad"] * j2.RAD_TO_DEG
            beta_error = beta_control["target_deg"] - fdm["aero/beta-deg"]
            gamma_error = -fdm["flight-path/gamma-deg"]
            cas_error = target_kcas - fdm["velocities/vc-kts"]
            bank_integral = _clamp(
                bank_integral + bank_error * dt,
                -bank_control["integral_limit_deg_s"],
                bank_control["integral_limit_deg_s"],
            )
            beta_integral = _clamp(
                beta_integral + beta_error * dt,
                -beta_control["integral_limit_deg_s"],
                beta_control["integral_limit_deg_s"],
            )
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
            raw_aileron = (
                bank_control["proportional_aileron_cmd_per_deg"] * bank_error
                + bank_control["integral_aileron_cmd_per_deg_s"] * bank_integral
            )
            raw_rudder = (
                beta_control["proportional_rudder_cmd_per_deg"] * beta_error
                + beta_control["integral_rudder_cmd_per_deg_s"] * beta_integral
            )
            raw_elevator = (
                gamma_control["proportional_elevator_cmd_per_deg"] * gamma_error
                + gamma_control["integral_elevator_cmd_per_deg_s"] * gamma_integral
            )
            raw_throttle = (
                baseline["throttle_cmd_norm"]
                + cas_control["proportional_throttle_cmd_per_knot"] * cas_error
                + cas_control["integral_throttle_cmd_per_knot_s"] * cas_integral
            )
            aileron_cmd = _clamp(
                raw_aileron,
                bank_control["aileron_cmd_min_norm"],
                bank_control["aileron_cmd_max_norm"],
            )
            rudder_cmd = _clamp(
                raw_rudder,
                beta_control["rudder_cmd_min_norm"],
                beta_control["rudder_cmd_max_norm"],
            )
            elevator_cmd = _clamp(
                raw_elevator,
                gamma_control["elevator_cmd_min_norm"],
                gamma_control["elevator_cmd_max_norm"],
            )
            throttle_cmd = _clamp(
                raw_throttle,
                cas_control["throttle_cmd_min_norm"],
                cas_control["throttle_cmd_max_dry_norm"],
            )
            clipped = {
                "aileron": not math.isclose(raw_aileron, aileron_cmd, abs_tol=1.0e-12),
                "rudder": not math.isclose(raw_rudder, rudder_cmd, abs_tol=1.0e-12),
                "elevator": not math.isclose(raw_elevator, elevator_cmd, abs_tol=1.0e-12),
                "throttle": not math.isclose(raw_throttle, throttle_cmd, abs_tol=1.0e-12),
            }
            for key, value in clipped.items():
                command_saturation_any[key] |= value
                if elapsed_s >= measurement_start - epsilon:
                    command_saturation_measurement[key] |= value
            fdm["fcs/aileron-cmd-norm"] = aileron_cmd
            fdm["fcs/rudder-cmd-norm"] = rudder_cmd
            fdm["fcs/elevator-cmd-norm"] = elevator_cmd
            fdm["fcs/throttle-cmd-norm"] = throttle_cmd
            if not fdm.run():
                raise RuntimeError("replay terminated early")
            sample = _snapshot(fdm, elapsed_s)
            sample.update(
                {
                    "commanded_bank_deg": commanded_bank,
                    "bank_error_deg": target_bank_deg - sample["roll_deg"],
                    "beta_error_deg": beta_control["target_deg"] - sample["beta_deg"],
                    "gamma_error_deg": -sample["gamma_deg"],
                    "cas_error_kts": target_kcas - sample["kcas"],
                }
            )
            samples.append(sample)

        measurement = [item for item in samples if item["elapsed_s"] >= measurement_start]
        numeric_keys = [key for key in measurement[0] if key not in {"elapsed_s", "sim_time_s"}]
        finite = j2._all_finite(item[key] for item in samples for key in numeric_keys)
        summary = {key: j2._stats([item[key] for item in measurement]) for key in numeric_keys}
        trajectory = _trajectory_metrics(measurement, target_bank_deg)
        settled_at = _earliest_tracking_time(samples, target_kcas, target_bank_deg, config)
        max_cas_error = max(j2._relative_error(item["kcas"], target_kcas) for item in measurement)
        max_gamma_error = max(abs(item["gamma_deg"]) for item in measurement)
        max_bank_error = max(abs(item["roll_deg"] - target_bank_deg) for item in measurement)
        surface_keys = (
            "elevator_pos_norm", "left_aileron_pos_norm", "right_aileron_pos_norm",
            "rudder_pos_norm", "speedbrake_pos_norm",
        )
        max_surface = max(abs(item[key]) for item in measurement for key in surface_keys)
        mean_rate = trajectory["measured_turn_rate_deg_s"]
        sign = 0 if target_bank_deg == 0.0 else (1 if target_bank_deg > 0.0 else -1)
        if sign == 0:
            bank_sign = abs(summary["roll_deg"]["mean"]) <= config["acceptance"]["bank_tracking_absolute_deg"]
            heading_sign = abs(mean_rate) <= config["acceptance"]["straight_heading_rate_absolute_deg_s"]
            turn_rate_stable = heading_sign
            radius_valid = trajectory["measured_trajectory_radius_m"] is None
            theory_cross_check = trajectory["theoretical_coordinated_radius_m"] is None
        else:
            bank_sign = summary["roll_deg"]["mean"] * sign > 0.0
            heading_sign = mean_rate * sign > 0.0
            turn_rate_stable = (
                trajectory["maximum_turn_rate_relative_deviation"] is not None
                and trajectory["maximum_turn_rate_relative_deviation"]
                <= config["acceptance"]["turn_rate_stability_relative"]
            )
            radius_valid = (
                trajectory["measured_trajectory_radius_m"] is not None
                and math.isfinite(trajectory["measured_trajectory_radius_m"])
                and trajectory["measured_trajectory_radius_m"] > 0.0
            )
            theory_cross_check = (
                trajectory["radius_theory_relative_difference"] is not None
                and trajectory["radius_theory_relative_difference"]
                <= config["acceptance"]["theory_radius_relative_difference_max"]
            )
        expected = j2_config["mass_and_fuel"]
        checks = {
            "trim_succeeded": trim_succeeded,
            "finite_outputs": finite,
            "measurement_window_complete": len(measurement) >= round(config["simulation"]["steady_measurement_window_s"] / dt),
            "settled_by_measurement_window": settled_at is not None and settled_at <= measurement_start + dt,
            "cas_tracking": max_cas_error <= config["acceptance"]["cas_tracking_relative"],
            "gamma_tracking": max_gamma_error <= config["acceptance"]["gamma_tracking_absolute_deg"],
            "bank_tracking": max_bank_error <= config["acceptance"]["bank_tracking_absolute_deg"],
            "bank_sign": bank_sign,
            "heading_turn_rate_sign": heading_sign,
            "turn_rate_stability": turn_rate_stable,
            "measured_radius_valid": radius_valid,
            "theoretical_radius_cross_check": theory_cross_check,
            "control_surface_usage": max_surface <= config["acceptance"]["normalized_control_surface_usage_max"],
            "outer_loop_commands_not_saturated_in_measurement": not any(command_saturation_measurement.values()),
            "afterburner_off": summary["throttle_pos_norm"]["max"] <= 1.0 + 1.0e-12,
            "engine_running": summary["engine_running_flag"]["min"] >= 1.0 and summary["engine_stalled"]["max"] == 0.0 and summary["engine_seized"]["max"] == 0.0,
            "gear_up": summary["gear_pos_norm"]["max"] <= 1.0e-12,
            "speedbrake_closed": summary["speedbrake_pos_norm"]["max"] <= 1.0e-12,
            "native_fcs": summary["fbw_override"]["max"] == 0.0,
            "zero_wind": max(
                abs(summary[key][bound])
                for key in ("wind_north_fps", "wind_east_fps", "wind_down_fps")
                for bound in ("min", "max")
            ) <= config["acceptance"]["zero_wind_residual_fps"],
            "fixed_weight": abs(summary["weight_lbs"]["mean"] - expected["expected_total_weight_lbs"]) <= 1.0e-6,
            "fixed_fuel": abs(summary["total_fuel_lbs"]["mean"] - expected["expected_total_fuel_lbs"]) <= 1.0e-6,
        }
        reliable = all(checks.values())
        record.update(
            {
                "baseline_level_trim": baseline,
                "measurement": summary,
                "trajectory": trajectory,
                "tracking": {
                    "maximum_cas_relative_error": max_cas_error,
                    "maximum_gamma_absolute_error_deg": max_gamma_error,
                    "maximum_bank_absolute_error_deg": max_bank_error,
                    "maximum_normalized_control_surface_usage": max_surface,
                },
                "settling_and_measurement": {
                    "settled_at_s": settled_at,
                    "measurement_start_s": measurement_start,
                    "measurement_duration_s": config["simulation"]["steady_measurement_window_s"],
                    "measurement_sample_count": len(measurement),
                },
                "controller": {
                    "command_saturation_any_time": command_saturation_any,
                    "command_saturation_in_measurement": command_saturation_measurement,
                    "rudder_target_beta_deg": beta_control["target_deg"],
                },
                "checks": checks,
                "status": classify_turn_sanity_result(
                    reliable_measurement=reliable,
                    confirmed_aircraft_inability=False,
                ),
                "status_reasons": [name for name, passed in checks.items() if not passed],
            }
        )
        return record
    except Exception as exc:
        record["status"] = "UNKNOWN"
        record["status_reasons"].append(f"{type(exc).__name__}: {exc}")
        return record


def _repeatability(records: list[dict[str, Any]], target_bank_deg: float, tolerance: float) -> dict[str, Any]:
    metric_paths = {
        "actual_kcas": ("measurement", "kcas", "mean"),
        "actual_bank_deg": ("measurement", "roll_deg", "mean"),
        "actual_gamma_deg": ("measurement", "gamma_deg", "mean"),
        "throttle_pos_norm": ("measurement", "throttle_pos_norm", "mean"),
        "load_factor_nz": ("measurement", "load_factor_nz", "mean"),
        "beta_deg": ("measurement", "beta_deg", "mean"),
        "turn_rate_deg_s": ("trajectory", "measured_turn_rate_deg_s"),
    }
    if target_bank_deg != 0.0:
        metric_paths["measured_radius_m"] = ("trajectory", "measured_trajectory_radius_m")
    if any("measurement" not in record for record in records):
        return {"passed": False, "tolerance_relative": tolerance, "metrics": {}}
    metrics: dict[str, Any] = {}
    for name, path in metric_paths.items():
        values = []
        for record in records:
            value: Any = record
            for key in path:
                value = value[key]
            values.append(float(value))
        mean = fmean(values)
        scale = max(abs(mean), 1.0e-9)
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


def _aggregate_point(
    altitude: float, maneuver: str, target_bank_deg: float,
    records: list[dict[str, Any]], provenance_id: str, tolerance: float,
) -> dict[str, Any]:
    repeatability = _repeatability(records, target_bank_deg, tolerance)
    complete = all("measurement" in record for record in records)
    status = (
        "VALID" if complete and repeatability["passed"]
        and all(record["status"] == "VALID" for record in records)
        else "UNKNOWN"
    )
    measured = None
    if complete:
        def mean_measurement(key: str, stat: str = "mean") -> float:
            return fmean(record["measurement"][key][stat] for record in records)

        def mean_trajectory(key: str) -> float | None:
            values = [record["trajectory"][key] for record in records]
            return None if any(value is None for value in values) else fmean(values)

        measured = {
            "actual_bank_deg": mean_measurement("roll_deg"),
            "heading_deg": mean_measurement("heading_deg"),
            "heading_change_deg": mean_trajectory("unwrapped_heading_change_deg"),
            "turn_rate_deg_s": mean_trajectory("measured_turn_rate_deg_s"),
            "measured_radius_m": mean_trajectory("measured_trajectory_radius_m"),
            "theoretical_radius_m": mean_trajectory("theoretical_coordinated_radius_m"),
            "radius_theory_relative_difference": mean_trajectory("radius_theory_relative_difference"),
            "actual_kcas": mean_measurement("kcas"),
            "tas_mps": mean_measurement("tas_mps"),
            "mach": mean_measurement("mach"),
            "altitude_msl_m": mean_measurement("altitude_msl_m"),
            "altitude_delta_m": mean_measurement("altitude_msl_m", "delta"),
            "gamma_deg": mean_measurement("gamma_deg"),
            "vertical_speed_mps": mean_measurement("vertical_speed_mps"),
            "throttle_cmd_norm": mean_measurement("throttle_cmd_norm"),
            "throttle_pos_norm": mean_measurement("throttle_pos_norm"),
            "alpha_deg": mean_measurement("alpha_deg"),
            "beta_deg": mean_measurement("beta_deg"),
            "beta_range_deg": max(record["measurement"]["beta_deg"]["max"] for record in records) - min(record["measurement"]["beta_deg"]["min"] for record in records),
            "load_factor_nz": mean_measurement("load_factor_nz"),
            "pitch_deg": mean_measurement("pitch_deg"),
            "p_deg_s": mean_measurement("p_deg_s"),
            "q_deg_s": mean_measurement("q_deg_s"),
            "r_deg_s": mean_measurement("r_deg_s"),
            "aileron_cmd_norm": mean_measurement("aileron_cmd_norm"),
            "elevator_cmd_norm": mean_measurement("elevator_cmd_norm"),
            "rudder_cmd_norm": mean_measurement("rudder_cmd_norm"),
            "left_aileron_pos_norm": mean_measurement("left_aileron_pos_norm"),
            "right_aileron_pos_norm": mean_measurement("right_aileron_pos_norm"),
            "rudder_pos_norm": mean_measurement("rudder_pos_norm"),
            "elevator_pos_norm": mean_measurement("elevator_pos_norm"),
            "maximum_control_surface_usage_norm": max(record["tracking"]["maximum_normalized_control_surface_usage"] for record in records),
            "maximum_cas_error_relative": max(record["tracking"]["maximum_cas_relative_error"] for record in records),
            "maximum_gamma_error_deg": max(record["tracking"]["maximum_gamma_absolute_error_deg"] for record in records),
            "maximum_bank_error_deg": max(record["tracking"]["maximum_bank_absolute_error_deg"] for record in records),
            "turn_rate_stability_relative": max(
                record["trajectory"]["maximum_turn_rate_relative_deviation"] or 0.0
                for record in records
            ),
            "settled_at_s": max(record["settling_and_measurement"]["settled_at_s"] for record in records),
        }
    return {
        "provenance_id": provenance_id,
        "altitude_msl_m": altitude,
        "maneuver_family": "level_turn_sanity",
        "maneuver": maneuver,
        "requested_target": {
            "speed_type": "KCAS", "speed_value": 305.0,
            "gamma_deg": 0.0, "bank_deg": target_bank_deg,
        },
        "status": status,
        "failure_classification": "none" if status == "VALID" else "controller_or_methodology_unknown",
        "raw_capability": None,
        "derated_capability": None,
        "measured_values": measured,
        "diagnostics": {
            "repeatability": repeatability,
            "run_ids": [record["run_id"] for record in records],
            "run_statuses": [record["status"] for record in records],
            "run_status_reasons": [record["status_reasons"] for record in records],
        },
    }


def _left_right_comparison(
    altitude: float, left: dict[str, Any], right: dict[str, Any], config: dict[str, Any]
) -> dict[str, Any]:
    if left["measured_values"] is None or right["measured_values"] is None:
        return {"altitude_msl_m": altitude, "passed": False, "reason": "missing_measurement"}
    left_values = left["measured_values"]
    right_values = right["measured_values"]
    limit = config["acceptance"]["left_right_magnitude_difference_relative_max"]
    comparisons = {}
    for key in (
        "actual_bank_deg", "turn_rate_deg_s", "measured_radius_m",
        "throttle_pos_norm", "load_factor_nz",
    ):
        left_magnitude = abs(left_values[key])
        right_magnitude = abs(right_values[key])
        relative_difference = abs(left_magnitude - right_magnitude) / max(
            fmean([left_magnitude, right_magnitude]), 1.0e-12
        )
        comparisons[key] = {
            "left": left_values[key], "right": right_values[key],
            "relative_magnitude_difference": relative_difference,
            "passed": relative_difference <= limit,
        }
    cas_error_difference = abs(
        left_values["maximum_cas_error_relative"]
        - right_values["maximum_cas_error_relative"]
    )
    cas_error_budget_fraction = cas_error_difference / config["acceptance"]["cas_tracking_relative"]
    comparisons["maximum_cas_error_relative"] = {
        "left": left_values["maximum_cas_error_relative"],
        "right": right_values["maximum_cas_error_relative"],
        "absolute_difference": cas_error_difference,
        "difference_as_fraction_of_tracking_acceptance": cas_error_budget_fraction,
        "passed": cas_error_budget_fraction <= limit,
    }
    beta_threshold = config["acceptance"]["beta_near_zero_for_sign_test_deg"]
    beta_mirrored_or_small = (
        abs(left_values["beta_deg"]) <= beta_threshold
        and abs(right_values["beta_deg"]) <= beta_threshold
    ) or left_values["beta_deg"] * right_values["beta_deg"] < 0.0
    comparisons["beta"] = {
        "left_deg": left_values["beta_deg"],
        "right_deg": right_values["beta_deg"],
        "mirrored_sign_or_both_near_zero": beta_mirrored_or_small,
        "operational_limit_claimed": False,
        "passed": beta_mirrored_or_small,
    }
    return {
        "altitude_msl_m": altitude,
        "passed": all(item["passed"] for item in comparisons.values()),
        "relative_difference_limit": limit,
        "comparisons": comparisons,
    }


def run_j5a() -> dict[str, Any]:
    started_at = time.perf_counter()
    config = _load_yaml(CONFIG_PATH)
    j2_config = _load_yaml(J2_CONFIG_PATH)
    provenance = _provenance(config, j2_config)
    audit = _controller_audit(config)
    all_runs: list[dict[str, Any]] = []
    points: list[dict[str, Any]] = []
    repeats = config["reference"]["cold_start_repetitions_per_point"]
    tolerance = config["acceptance"]["cold_start_repeatability_relative_approx"]
    for altitude_value in config["reference"]["representative_altitudes_msl_m"]:
        altitude = float(altitude_value)
        for maneuver, target in config["reference"]["maneuvers"].items():
            target_bank = float(target["target_bank_deg"])
            records = [
                _run_once(
                    altitude, maneuver, target_bank, repeat,
                    config, j2_config, provenance["provenance_id"],
                )
                for repeat in range(1, repeats + 1)
            ]
            all_runs.extend(records)
            points.append(_aggregate_point(
                altitude, maneuver, target_bank, records,
                provenance["provenance_id"], tolerance,
            ))

    comparisons = []
    for altitude_value in config["reference"]["representative_altitudes_msl_m"]:
        altitude = float(altitude_value)
        left = next(point for point in points if point["altitude_msl_m"] == altitude and point["maneuver"] == "modest_left")
        right = next(point for point in points if point["altitude_msl_m"] == altitude and point["maneuver"] == "modest_right")
        comparison = _left_right_comparison(altitude, left, right, config)
        comparisons.append(comparison)
        if not comparison["passed"]:
            for point in (left, right):
                if point["status"] == "VALID":
                    point["status"] = "UNKNOWN"
                    point["failure_classification"] = "unexplained_left_right_asymmetry"
                    point["diagnostics"]["symmetry_failure"] = True

    status_counts = {
        status: sum(point["status"] == status for point in points)
        for status in ("VALID", "INFEASIBLE", "UNKNOWN")
    }
    j5a_pass = (
        status_counts == {"VALID": 12, "INFEASIBLE": 0, "UNKNOWN": 0}
        and all(comparison["passed"] for comparison in comparisons)
        and all(point["diagnostics"]["repeatability"]["passed"] for point in points)
    )
    result = {
        "provenance_id": provenance["provenance_id"],
        "parent_j4b_provenance_id": provenance["parent_j4b_provenance_id"],
        "j5a_pass": j5a_pass,
        "j5b_blocker": None if j5a_pass else "J5A methodology sanity gate did not pass",
        "altitude_count": len(config["reference"]["representative_altitudes_msl_m"]),
        "maneuver_count": len(config["reference"]["maneuvers"]),
        "run_count": len(all_runs),
        "point_count": len(points),
        "status_counts": status_counts,
        "runtime_wall_s": time.perf_counter() - started_at,
        "scope_guards": config["scope"],
    }
    RESULTS_DIR.mkdir(exist_ok=True)
    outputs = {
        "j5a_provenance.json": provenance,
        "j5a_controller_audit.json": audit,
        "j5a_runs.json": all_runs,
        "j5a_points.json": points,
        "j5a_left_right_comparison.json": comparisons,
        "j5a_result.json": result,
    }
    for filename, payload in outputs.items():
        (RESULTS_DIR / filename).write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    return result


if __name__ == "__main__":
    print(json.dumps(run_j5a(), indent=2, sort_keys=True))
