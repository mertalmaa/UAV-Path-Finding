"""J2 provenance and baseline level-flight sanity harness.

Scope is deliberately narrow: three fixed altitudes, one sanity-only KCAS,
full trim, 30 s replay, and three independent FGFDMExec instances per
condition. This module does not search a speed/envelope, test turns, build a
lookup, apply derating, or import planner code.
"""

from __future__ import annotations

import hashlib
import importlib.metadata
import json
import math
import platform
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from statistics import fmean
from typing import Any, Iterable

import jsbsim
import yaml


HERE = Path(__file__).resolve().parent
CONFIG_PATH = HERE / "j2_reference_configuration.yaml"
RESULTS_DIR = HERE / "results"
M_TO_FT = 1.0 / 0.3048
FPS_TO_MPS = 0.3048
RAD_TO_DEG = 180.0 / math.pi


def _read_config() -> dict[str, Any]:
    return yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _relative_error(value: float, target: float) -> float:
    return abs(value - target) / abs(target) if target else abs(value - target)


def _all_finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def _stats(values: list[float]) -> dict[str, float]:
    return {
        "min": min(values),
        "max": max(values),
        "mean": fmean(values),
        "start": values[0],
        "end": values[-1],
        "delta": values[-1] - values[0],
    }


def classify_characterization_result(
    *, reliable_measurement: bool, confirmed_unsustainable: bool
) -> str:
    """Apply the frozen J1 three-state semantics.

    INFEASIBLE requires affirmative, reliable evidence that the requested
    condition is unsustainable. Setup, trim, numerical, settling, or
    repeatability failures remain UNKNOWN.
    """
    if confirmed_unsustainable and reliable_measurement:
        return "INFEASIBLE"
    if reliable_measurement:
        return "VALID"
    return "UNKNOWN"


def _provenance(config: dict[str, Any]) -> dict[str, Any]:
    try:
        jsbsim.FGJSBBase().debug_lvl = 0
    except AttributeError:
        pass
    probe = jsbsim.FGFDMExec(None)
    probe.set_debug_level(0)
    compiled_version = probe.get_version()
    root = Path(jsbsim.get_default_root_dir()).resolve()
    package_binary = root / "_jsbsim.pyd"

    relevant_files = {
        "aircraft_model": root / "aircraft" / "f16" / "f16.xml",
        "aircraft_reset_example": root / "aircraft" / "f16" / "reset00.xml",
        "engine_model": root / "engine" / "F100-PW-229.xml",
        "thruster_model": root / "engine" / "direct.xml",
        "system_pushback": root / "aircraft" / "f16" / "Systems" / "pushback.xml",
        "system_hook": root / "aircraft" / "f16" / "Systems" / "hook.xml",
        "package_init": root / "__init__.py",
        "package_binary": package_binary,
        "reference_configuration": CONFIG_PATH,
        "baseline_harness": Path(__file__).resolve(),
    }
    missing = [str(path) for path in relevant_files.values() if not path.is_file()]
    if missing:
        raise FileNotFoundError(f"Missing provenance files: {missing}")

    aircraft_root = ET.parse(relevant_files["aircraft_model"]).getroot()
    engine_element = aircraft_root.find("./propulsion/engine")
    thruster_element = aircraft_root.find("./propulsion/engine/thruster")
    flight_control_element = aircraft_root.find("./flight_control")
    declarations = {
        "fdm_config_name": aircraft_root.attrib.get("name"),
        "engine_model": engine_element.attrib.get("file") if engine_element is not None else None,
        "thruster_model": thruster_element.attrib.get("file") if thruster_element is not None else None,
        "system_models": [element.attrib.get("file") for element in aircraft_root.findall("./system")],
        "fcs_name": flight_control_element.attrib.get("name") if flight_control_element is not None else None,
    }
    expected_aircraft = config["aircraft"]
    expected_declarations = {
        "fdm_config_name": expected_aircraft["expected_fdm_config_name"],
        "engine_model": expected_aircraft["expected_engine_model"],
        "thruster_model": expected_aircraft["expected_thruster_model"],
        "system_models": expected_aircraft["expected_system_models"],
        "fcs_name": expected_aircraft["expected_fcs_name"],
    }
    if declarations != expected_declarations:
        raise RuntimeError(
            f"F-16 model dependency declarations differ: {declarations!r} != {expected_declarations!r}"
        )

    hashes = {
        name: {"path": str(path), "sha256": _sha256(path)}
        for name, path in relevant_files.items()
    }
    commit_match = re.search(r"commit ([0-9a-f]{40})", compiled_version)
    build_match = re.search(r"GitHub build (\d+)", compiled_version)
    fingerprint_input = {
        "compiled_version": compiled_version,
        "python": sys.version,
        "files": {name: item["sha256"] for name, item in hashes.items()},
        "timestep_s": config["simulation"]["timestep_s"],
    }
    provenance_id = "j2-" + hashlib.sha256(
        json.dumps(fingerprint_input, sort_keys=True).encode("utf-8")
    ).hexdigest()[:20]

    return {
        "provenance_id": provenance_id,
        "jsbsim_python_package_version": importlib.metadata.version("jsbsim"),
        "jsbsim_compiled_version": compiled_version,
        "jsbsim_git_commit": commit_match.group(1) if commit_match else None,
        "jsbsim_github_build": int(build_match.group(1)) if build_match else None,
        "jsbsim_default_root": str(root),
        "python_version": sys.version,
        "python_executable": sys.executable,
        "platform": platform.platform(),
        "timestep_s": config["simulation"]["timestep_s"],
        "units_and_conventions": config["units_and_conventions"],
        "model_dependency_declarations": declarations,
        "files": hashes,
    }


def _set_reference_properties(fdm: jsbsim.FGFDMExec, config: dict[str, Any]) -> None:
    fuel = config["mass_and_fuel"]
    aircraft_config = config["configuration"]
    environment = config["environment"]

    tank_contents = fuel["internal_tank_contents_lbs"] + fuel["external_tank_contents_lbs"]
    for index, contents_lbs in enumerate(tank_contents):
        suffix = "" if index == 0 else f"[{index}]"
        fdm[f"propulsion/tank{suffix}/contents-lbs"] = contents_lbs

    fdm["gear/gear-cmd-norm"] = aircraft_config["landing_gear_command_norm"]
    fdm["gear/gear-pos-norm"] = aircraft_config["landing_gear_position_norm"]
    fdm["fcs/speedbrake-cmd-norm"] = aircraft_config["speedbrake_command_norm"]
    fdm["fcs/fbw-override"] = config["aircraft"]["fbw_override"]

    fdm["atmosphere/delta-T"] = environment["delta_temperature_R"]
    for axis in ("north", "east", "down"):
        fdm[f"atmosphere/wind-{axis}-fps"] = environment[f"wind_{axis}_fps"]
        fdm[f"atmosphere/gust-{axis}-fps"] = environment[f"gust_{axis}_fps"]
    fdm["atmosphere/turb-type"] = environment["turbulence_type"]


def _snapshot(fdm: jsbsim.FGFDMExec, elapsed_s: float) -> dict[str, float]:
    return {
        "elapsed_s": elapsed_s,
        "sim_time_s": fdm["simulation/sim-time-sec"],
        "altitude_msl_m": fdm["position/h-sl-meters"],
        "kcas": fdm["velocities/vc-kts"],
        "tas_mps": fdm["velocities/vt-fps"] * FPS_TO_MPS,
        "mach": fdm["velocities/mach"],
        "alpha_deg": fdm["aero/alpha-deg"],
        "beta_deg": fdm["aero/beta-deg"],
        "gamma_deg": fdm["flight-path/gamma-deg"],
        "throttle_cmd_norm": fdm["fcs/throttle-cmd-norm"],
        "throttle_pos_norm": fdm["fcs/throttle-pos-norm"],
        "elevator_pos_norm": fdm["fcs/elevator-pos-norm"],
        "left_aileron_pos_norm": fdm["fcs/left-aileron-pos-norm"],
        "right_aileron_pos_norm": fdm["fcs/right-aileron-pos-norm"],
        "rudder_pos_norm": fdm["fcs/rudder-pos-norm"],
        "flap_pos_norm": fdm["fcs/flap-pos-norm"],
        "speedbrake_pos_norm": fdm["fcs/speedbrake-pos-norm"],
        "gear_pos_norm": fdm["gear/gear-pos-norm"],
        "load_factor_nx": fdm["accelerations/Nx"],
        "load_factor_ny": fdm["accelerations/Ny"],
        "load_factor_nz": fdm["accelerations/Nz"],
        "p_deg_s": fdm["velocities/p-rad_sec"] * RAD_TO_DEG,
        "q_deg_s": fdm["velocities/q-rad_sec"] * RAD_TO_DEG,
        "r_deg_s": fdm["velocities/r-rad_sec"] * RAD_TO_DEG,
        "engine_running_flag": fdm["propulsion/engine/set-running"],
        "engine_n1": fdm["propulsion/engine/n1"],
        "engine_n2": fdm["propulsion/engine/n2"],
        "engine_thrust_lbs": fdm["propulsion/engine/thrust-lbs"],
        "engine_stalled": fdm["propulsion/engine/stalled"],
        "engine_seized": fdm["propulsion/engine/seized"],
        "total_fuel_lbs": fdm["propulsion/total-fuel-lbs"],
        "weight_lbs": fdm["inertia/weight-lbs"],
        "cg_x_in": fdm["inertia/cg-x-in"],
        "cg_y_in": fdm["inertia/cg-y-in"],
        "cg_z_in": fdm["inertia/cg-z-in"],
        "fbw_override": fdm["fcs/fbw-override"],
        "wind_north_fps": fdm["atmosphere/total-wind-north-fps"],
        "wind_east_fps": fdm["atmosphere/total-wind-east-fps"],
        "wind_down_fps": fdm["atmosphere/total-wind-down-fps"],
    }


def _earliest_settled_time(
    samples: list[dict[str, float]], requested_kcas: float, config: dict[str, Any]
) -> float | None:
    measurement_duration = config["simulation"]["steady_measurement_window_s"]
    cas_tolerance = config["acceptance"]["cas_tracking_relative"]
    gamma_tolerance = config["acceptance"]["gamma_tracking_absolute_deg"]
    final_time = samples[-1]["elapsed_s"]
    for index, sample in enumerate(samples):
        if final_time - sample["elapsed_s"] + 1e-9 < measurement_duration:
            break
        remaining = samples[index:]
        if all(
            _relative_error(item["kcas"], requested_kcas) <= cas_tolerance
            and abs(item["gamma_deg"]) <= gamma_tolerance
            for item in remaining
        ):
            return sample["elapsed_s"]
    return None


def _configuration_snapshot(fdm: jsbsim.FGFDMExec) -> dict[str, Any]:
    return {
        "aircraft_model": "f16",
        "weight_lbs": fdm["inertia/weight-lbs"],
        "mass_kg": fdm["inertia/mass-slugs"] * 14.5939029372,
        "cg_in": [
            fdm["inertia/cg-x-in"],
            fdm["inertia/cg-y-in"],
            fdm["inertia/cg-z-in"],
        ],
        "tank_contents_lbs": [
            fdm["propulsion/tank/contents-lbs"],
            fdm["propulsion/tank[1]/contents-lbs"],
            fdm["propulsion/tank[2]/contents-lbs"],
            fdm["propulsion/tank[3]/contents-lbs"],
        ],
        "gear_command_norm": fdm["gear/gear-cmd-norm"],
        "gear_position_norm": fdm["gear/gear-pos-norm"],
        "speedbrake_command_norm": fdm["fcs/speedbrake-cmd-norm"],
        "speedbrake_position_norm": fdm["fcs/speedbrake-pos-norm"],
        "fbw_override": fdm["fcs/fbw-override"],
        "turbulence_type": fdm["atmosphere/turb-type"],
    }


def _run_once(
    altitude_msl_m: float,
    repeat_index: int,
    config: dict[str, Any],
    provenance_id: str,
) -> dict[str, Any]:
    requested_kcas = float(config["sanity_condition"]["speed_value"])
    requested_gamma = float(config["sanity_condition"]["gamma_deg"])
    dt = float(config["simulation"]["timestep_s"])
    record: dict[str, Any] = {
        "provenance_id": provenance_id,
        "run_id": f"level-{altitude_msl_m:.0f}m-r{repeat_index}",
        "repeat_index": repeat_index,
        "requested": {
            "altitude_msl_m": altitude_msl_m,
            "speed_type": config["sanity_condition"]["speed_type"],
            "speed_value": requested_kcas,
            "gamma_deg": requested_gamma,
        },
        "status": "UNKNOWN",
        "status_reasons": [],
        "confirmed_unsustainable": False,
    }

    try:
        fdm = jsbsim.FGFDMExec(None)
        fdm.set_debug_level(0)
        fdm.set_dt(dt)
        if not fdm.load_model(config["aircraft"]["model"]):
            raise RuntimeError("load_model returned false")
        loaded_model = {
            "model_id": fdm.get_model_name(),
            "aircraft_path": fdm.get_full_aircraft_path(),
        }
        if loaded_model["model_id"] != config["aircraft"]["expected_loaded_model_id"]:
            raise RuntimeError(f"Unexpected loaded model: {loaded_model!r}")
        record["loaded_model"] = loaded_model

        _set_reference_properties(fdm, config)
        fdm["ic/h-sl-ft"] = altitude_msl_m * M_TO_FT
        fdm["ic/vc-kts"] = requested_kcas
        fdm["ic/gamma-deg"] = requested_gamma
        fdm["ic/phi-deg"] = 0.0
        fdm["ic/theta-deg"] = 0.0
        fdm["ic/psi-true-deg"] = 0.0
        if not fdm.run_ic():
            raise RuntimeError("run_ic returned false")

        _set_reference_properties(fdm, config)
        fdm["propulsion/engine/set-running"] = 1.0
        if config["mass_and_fuel"]["freeze_fuel_during_replay"]:
            fdm["propulsion/fuel_freeze"] = 1.0

        # One initialized frame applies the requested configuration and makes
        # engine state observable before trim; it is not part of replay time.
        if not fdm.run():
            raise RuntimeError("initialized frame failed")
        initialized = _snapshot(fdm, 0.0)

        trim_succeeded = False
        trim_error = None
        try:
            fdm.do_trim(1)
            trim_succeeded = True
        except (jsbsim.TrimFailureError, RuntimeError) as exc:
            trim_error = f"{type(exc).__name__}: {exc}"

        record["trim"] = {
            "mode": "full",
            "succeeded": trim_succeeded,
            "trim_completed_property": fdm["simulation/trim-completed"],
            "error": trim_error,
        }
        record["initialized"] = initialized
        record["reference_configuration_after_trim"] = _configuration_snapshot(fdm)

        if not trim_succeeded:
            record["status_reasons"].append("trim_failure_is_unknown_not_infeasible")
            record["status"] = classify_characterization_result(
                reliable_measurement=False, confirmed_unsustainable=False
            )
            return record

        samples: list[dict[str, float]] = []
        step_count = round(config["simulation"]["replay_duration_s"] / dt)
        for step in range(1, step_count + 1):
            if not fdm.run():
                raise RuntimeError("replay terminated before requested duration")
            samples.append(_snapshot(fdm, step * dt))

        measurement_start = (
            config["simulation"]["replay_duration_s"]
            - config["simulation"]["steady_measurement_window_s"]
        )
        measurement = [item for item in samples if item["elapsed_s"] >= measurement_start]
        numeric_keys = [key for key in measurement[0] if key not in {"elapsed_s", "sim_time_s"}]
        finite = _all_finite(item[key] for item in samples for key in numeric_keys)
        summary = {key: _stats([item[key] for item in measurement]) for key in numeric_keys}
        settled_at_s = _earliest_settled_time(samples, requested_kcas, config)

        max_cas_error = max(_relative_error(item["kcas"], requested_kcas) for item in measurement)
        max_gamma_error = max(abs(item["gamma_deg"] - requested_gamma) for item in measurement)
        control_keys = (
            "elevator_pos_norm",
            "left_aileron_pos_norm",
            "right_aileron_pos_norm",
            "rudder_pos_norm",
        )
        max_control_usage = max(abs(item[key]) for item in measurement for key in control_keys)
        final_config = _configuration_snapshot(fdm)
        expected = config["mass_and_fuel"]
        aircraft_config = config["configuration"]

        checks = {
            "trim_succeeded": trim_succeeded,
            "finite_outputs": finite,
            "initialized_altitude_within_1m": abs(initialized["altitude_msl_m"] - altitude_msl_m) <= 1.0,
            "initialized_speed_within_acceptance": _relative_error(initialized["kcas"], requested_kcas)
            <= config["acceptance"]["cas_tracking_relative"],
            "settled_within_allowance": settled_at_s is not None
            and settled_at_s <= config["simulation"]["maximum_settling_time_s"] + dt,
            "measurement_window_complete": len(measurement)
            >= round(config["simulation"]["steady_measurement_window_s"] / dt),
            "cas_tracking": max_cas_error <= config["acceptance"]["cas_tracking_relative"],
            "gamma_tracking": max_gamma_error <= config["acceptance"]["gamma_tracking_absolute_deg"],
            "control_surface_usage": max_control_usage
            <= config["acceptance"]["normalized_control_surface_usage_max"],
            "engine_running": summary["engine_running_flag"]["min"] >= 1.0
            and summary["engine_n1"]["min"] > 0.0
            and summary["engine_n2"]["min"] > 0.0
            and summary["engine_stalled"]["max"] == 0.0
            and summary["engine_seized"]["max"] == 0.0,
            "afterburner_off": summary["throttle_pos_norm"]["max"]
            <= aircraft_config["afterburner_off_max_throttle_position_norm"] + 1e-12,
            "gear_up": summary["gear_pos_norm"]["max"] <= 1e-12,
            "speedbrake_closed": summary["speedbrake_pos_norm"]["max"] <= 1e-12,
            "native_fcs": summary["fbw_override"]["max"] == 0.0,
            "zero_wind": max(
                abs(summary[key][bound])
                for key in ("wind_north_fps", "wind_east_fps", "wind_down_fps")
                for bound in ("min", "max")
            ) <= 1e-12,
            "fixed_weight": abs(summary["weight_lbs"]["max"] - expected["expected_total_weight_lbs"])
            <= 1e-6
            and abs(summary["weight_lbs"]["min"] - expected["expected_total_weight_lbs"]) <= 1e-6,
            "fixed_fuel": abs(summary["total_fuel_lbs"]["max"] - expected["expected_total_fuel_lbs"])
            <= 1e-6
            and abs(summary["total_fuel_lbs"]["min"] - expected["expected_total_fuel_lbs"]) <= 1e-6,
            "external_tanks_empty": final_config["tank_contents_lbs"][2:] == [0.0, 0.0],
        }
        reliable = all(checks.values())
        record.update(
            {
                "settling_and_measurement": {
                    "replay_duration_s": config["simulation"]["replay_duration_s"],
                    "settled_at_s": settled_at_s,
                    "measurement_start_s": measurement_start,
                    "measurement_duration_s": config["simulation"]["steady_measurement_window_s"],
                    "measurement_sample_count": len(measurement),
                },
                "measurement": summary,
                "actual_speed": {
                    "type": "KCAS",
                    "initialized_value": initialized["kcas"],
                    "measurement_mean_value": summary["kcas"]["mean"],
                    "measurement_min_value": summary["kcas"]["min"],
                    "measurement_max_value": summary["kcas"]["max"],
                },
                "tracking": {
                    "maximum_cas_relative_error": max_cas_error,
                    "maximum_gamma_absolute_error_deg": max_gamma_error,
                    "maximum_normalized_control_surface_usage": max_control_usage,
                },
                "final_configuration": final_config,
                "checks": checks,
                "status": classify_characterization_result(
                    reliable_measurement=reliable, confirmed_unsustainable=False
                ),
                "status_reasons": [name for name, passed in checks.items() if not passed],
            }
        )
        return record
    except Exception as exc:  # Preserve diagnostics and classify setup/numerics as UNKNOWN.
        record["status"] = "UNKNOWN"
        record["status_reasons"].append(f"{type(exc).__name__}: {exc}")
        return record


def _repeatability(records: list[dict[str, Any]], config: dict[str, Any]) -> dict[str, Any]:
    metric_paths = {
        "actual_altitude_msl_m": ("measurement", "altitude_msl_m", "mean"),
        "actual_kcas": ("measurement", "kcas", "mean"),
        "tas_mps": ("measurement", "tas_mps", "mean"),
        "mach": ("measurement", "mach", "mean"),
        "alpha_deg": ("measurement", "alpha_deg", "mean"),
        "throttle_pos_norm": ("measurement", "throttle_pos_norm", "mean"),
        "load_factor_nz": ("measurement", "load_factor_nz", "mean"),
        "weight_lbs": ("measurement", "weight_lbs", "mean"),
    }
    if len(records) != config["simulation"]["cold_start_repetitions"] or any(
        record["status"] != "VALID" for record in records
    ):
        return {"passed": False, "reason": "not_all_repeats_valid", "metrics": {}}

    tolerance = config["acceptance"]["key_metric_repeatability_relative_approx"]
    metrics: dict[str, Any] = {}
    passed = True
    for name, path in metric_paths.items():
        values = []
        for record in records:
            value: Any = record
            for key in path:
                value = value[key]
            values.append(float(value))
        mean = fmean(values)
        deviations = [abs(value - mean) / abs(mean) if mean else abs(value - mean) for value in values]
        metric_passed = max(deviations) <= tolerance
        passed = passed and metric_passed
        metrics[name] = {
            "values": values,
            "mean": mean,
            "maximum_relative_deviation": max(deviations),
            "passed": metric_passed,
        }
    return {"passed": passed, "tolerance_relative": tolerance, "metrics": metrics}


def run_j2() -> dict[str, Any]:
    config = _read_config()
    provenance = _provenance(config)
    all_records: list[dict[str, Any]] = []
    repeatability: dict[str, Any] = {}

    for altitude in config["sanity_condition"]["representative_altitudes_msl_m"]:
        records = [
            _run_once(float(altitude), repeat, config, provenance["provenance_id"])
            for repeat in range(1, config["simulation"]["cold_start_repetitions"] + 1)
        ]
        group_repeatability = _repeatability(records, config)
        repeatability[f"{float(altitude):.0f}m"] = group_repeatability
        if not group_repeatability["passed"]:
            for record in records:
                if record["status"] == "VALID":
                    record["status"] = "UNKNOWN"
                    record["status_reasons"].append("cold_start_repeatability_failed")
        all_records.extend(records)

    status_counts = {
        status: sum(record["status"] == status for record in all_records)
        for status in ("VALID", "INFEASIBLE", "UNKNOWN")
    }
    all_altitudes_have_three_valid_repeats = all(
        item["passed"] for item in repeatability.values()
    ) and status_counts == {"VALID": len(all_records), "INFEASIBLE": 0, "UNKNOWN": 0}
    summary = {
        "stage": "J2_JSBSim_F16_provenance_and_baseline_sanity",
        "scope_guards": {
            "nominal_cas_sweep_performed": False,
            "climb_descent_characterization_performed": False,
            "turn_characterization_performed": False,
            "lookup_created": False,
            "planner_integration_performed": False,
            "derating_applied": False,
        },
        "provenance_id": provenance["provenance_id"],
        "sanity_speed": config["sanity_condition"],
        "reference_configuration_id": config["configuration_id"],
        "run_count": len(all_records),
        "status_counts": status_counts,
        "repeatability": repeatability,
        "classification_semantics_self_test": {
            "reliable": classify_characterization_result(
                reliable_measurement=True, confirmed_unsustainable=False
            ),
            "trim_or_setup_failure": classify_characterization_result(
                reliable_measurement=False, confirmed_unsustainable=False
            ),
            "confirmed_unsustainable": classify_characterization_result(
                reliable_measurement=True, confirmed_unsustainable=True
            ),
        },
        "j2_pass": all_altitudes_have_three_valid_repeats,
    }

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    (RESULTS_DIR / "j2_provenance.json").write_text(
        json.dumps(provenance, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    (RESULTS_DIR / "j2_runs.json").write_text(
        json.dumps(all_records, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    (RESULTS_DIR / "j2_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return summary


if __name__ == "__main__":
    result = run_j2()
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    raise SystemExit(0 if result["j2_pass"] else 1)
