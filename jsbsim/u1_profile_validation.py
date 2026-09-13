"""U1 fail-fast suitability validation for the frozen generic UAV profile.

This is deliberately not an envelope sweep or a controller tuning program.
The DHC6 matrix is executed in fail-fast order. The c182 backup subset is only
eligible when DHC6 is objectively REJECTED_FOR_PROFILE.
"""

from __future__ import annotations

import copy
import hashlib
import importlib.metadata
import json
import math
import platform
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from statistics import fmean, pstdev
from typing import Any, Iterable

import jsbsim
import yaml


HERE = Path(__file__).resolve().parent
CONFIG_PATH = HERE / "u1_profile_validation_configuration.yaml"
RESULTS_DIR = HERE / "results"
M_TO_FT = 1.0 / 0.3048
FPS_TO_MPS = 0.3048
KTS_TO_MPS = 0.5144444444444445
RAD_TO_DEG = 180.0 / math.pi


def _load_config() -> dict[str, Any]:
    return yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _stats(values: list[float]) -> dict[str, float]:
    return {
        "min": min(values),
        "max": max(values),
        "mean": fmean(values),
        "start": values[0],
        "end": values[-1],
        "delta": values[-1] - values[0],
    }


def _all_finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def _suffix(index: int) -> str:
    return "" if index == 0 else f"[{index}]"


def _provenance(config: dict[str, Any]) -> dict[str, Any]:
    probe = jsbsim.FGFDMExec(None)
    probe.set_debug_level(0)
    compiled = probe.get_version()
    root = Path(jsbsim.get_default_root_dir()).resolve()
    model_root = root / "aircraft" / "DHC6"
    files = {
        "aircraft_model": model_root / "DHC6.xml",
        "local_engine": model_root / "Engines" / "PT6A-27.xml",
        "local_thruster": model_root / "Engines" / "Propeller.xml",
        "system_propulsion": model_root / "Systems" / "Propulsion.xml",
        "system_controls": model_root / "Systems" / "Conventional Controls.xml",
        "system_gear": model_root / "Systems" / "Landing Gear.xml",
        "system_flaps": model_root / "Systems" / "Flaps.xml",
        "package_init": root / "__init__.py",
        "package_binary": root / "_jsbsim.pyd",
        "u1_configuration": CONFIG_PATH,
        "u1_harness": Path(__file__).resolve(),
        "backup_aircraft_model": root / "aircraft" / "c182" / "c182.xml",
        "backup_engine": root / "engine" / "engIO540AB1A5.xml",
        "backup_thruster": root / "engine" / "prop_81in2v.xml",
    }
    missing = [str(path) for path in files.values() if not path.is_file()]
    if missing:
        raise FileNotFoundError(f"missing provenance files: {missing}")

    aircraft_root = ET.parse(files["aircraft_model"]).getroot()
    engine = aircraft_root.find("./propulsion/engine")
    thruster = aircraft_root.find("./propulsion/engine/thruster")
    fcs = aircraft_root.find("./flight_control")
    declarations = {
        "fdm_config_name": aircraft_root.attrib.get("name"),
        "engine_model": engine.attrib.get("file") if engine is not None else None,
        "thruster_model": thruster.attrib.get("file") if thruster is not None else None,
        "system_models": [item.attrib.get("file") for item in aircraft_root.findall("./system")],
        "fcs_name": fcs.attrib.get("name") if fcs is not None else None,
    }
    expected = config["aircraft"]
    expected_declarations = {
        "fdm_config_name": expected["expected_fdm_config_name"],
        "engine_model": expected["expected_engine_model"],
        "thruster_model": expected["expected_thruster_model"],
        "system_models": expected["expected_system_models"],
        "fcs_name": expected["expected_fcs_name"],
    }
    if declarations != expected_declarations:
        raise RuntimeError(f"DHC6 dependency mismatch: {declarations!r}")

    hashes = {name: {"path": str(path), "sha256": _sha256(path)} for name, path in files.items()}
    fingerprint = {
        "compiled": compiled,
        "python": sys.version,
        "hashes": {name: value["sha256"] for name, value in hashes.items()},
        "fixture": config["fixture"],
    }
    provenance_id = "u1-" + hashlib.sha256(
        json.dumps(fingerprint, sort_keys=True).encode("utf-8")
    ).hexdigest()[:20]
    commit = re.search(r"commit ([0-9a-f]{40})", compiled)
    build = re.search(r"GitHub build (\d+)", compiled)
    return {
        "provenance_id": provenance_id,
        "jsbsim_python_package_version": importlib.metadata.version("jsbsim"),
        "jsbsim_compiled_version": compiled,
        "jsbsim_git_commit": commit.group(1) if commit else None,
        "jsbsim_github_build": int(build.group(1)) if build else None,
        "jsbsim_default_root": str(root),
        "python_version": sys.version,
        "python_executable": sys.executable,
        "platform": platform.platform(),
        "model_dependency_declarations": declarations,
        "fixture": config["fixture"],
        "ias_measurement_contract": config["ias_measurement_contract"],
        "files": hashes,
    }


def _set_environment_and_aircraft(fdm: Any, config: dict[str, Any]) -> None:
    fixture = config["fixture"]
    for index, contents in enumerate(fixture["tank_contents_lbs"]):
        fdm[f"propulsion/tank{_suffix(index)}/contents-lbs"] = float(contents)
    fdm["propulsion/fuel_freeze"] = 1.0 if fixture["fuel_frozen"] else 0.0
    fdm["fcs/flap-cmd-norm"] = float(fixture["flap_command_norm"])
    fdm["fcs/flap-pos-norm"] = float(fixture["flap_position_norm"])
    for index in range(int(fixture["engine_count"])):
        suffix = _suffix(index)
        fdm[f"fcs/throttle-cmd-norm{suffix}"] = float(
            fixture["initial_symmetric_throttle_command_norm"]
        )
        fdm[f"fcs/mixture-cmd-norm{suffix}"] = float(fixture["mixture_command_norm"])
        fdm[f"fcs/advance-cmd-norm{suffix}"] = float(fixture["propeller_advance_command_norm"])
        fdm[f"fcs/feather-cmd-norm{suffix}"] = float(fixture["feather_command_norm"])
    fdm["atmosphere/delta-T"] = float(fixture["delta_temperature_R"])
    for axis in ("north", "east", "down"):
        fdm[f"atmosphere/wind-{axis}-fps"] = float(fixture[f"wind_{axis}_fps"])
        fdm[f"atmosphere/gust-{axis}-fps"] = float(fixture[f"gust_{axis}_fps"])
    fdm["atmosphere/turb-type"] = float(fixture["turbulence_type"])


def _set_all_engines_running(fdm: Any, config: dict[str, Any]) -> None:
    # The propulsion-level selector is the portable JSBSim interface for
    # starting every engine; the per-engine property is retained for turbine
    # models and as an observable state.
    fdm["propulsion/set-running"] = -1.0
    for index in range(int(config["fixture"]["engine_count"])):
        fdm[f"propulsion/engine{_suffix(index)}/set-running"] = 1.0


def _set_symmetric_throttle(fdm: Any, config: dict[str, Any], command: float) -> None:
    for index in range(int(config["fixture"]["engine_count"])):
        fdm[f"fcs/throttle-cmd-norm{_suffix(index)}"] = command


def _snapshot(fdm: Any, elapsed_s: float, config: dict[str, Any]) -> dict[str, float]:
    roll = fdm["attitude/roll-rad"]
    pitch = fdm["attitude/pitch-rad"]
    q = fdm["velocities/q-rad_sec"]
    r = fdm["velocities/r-rad_sec"]
    heading_rate = (q * math.sin(roll) + r * math.cos(roll)) / max(math.cos(pitch), 1.0e-9)
    sample = {
        "elapsed_s": elapsed_s,
        "altitude_msl_m": fdm["position/h-sl-meters"],
        "ias_mps": fdm["velocities/vc-kts"] * KTS_TO_MPS,
        "cas_kts": fdm["velocities/vc-kts"],
        "tas_mps": fdm["velocities/vt-fps"] * FPS_TO_MPS,
        "vertical_speed_mps": fdm["velocities/h-dot-fps"] * FPS_TO_MPS,
        "gamma_deg": fdm["flight-path/gamma-deg"],
        "alpha_deg": fdm["aero/alpha-deg"],
        "beta_deg": fdm["aero/beta-deg"],
        "roll_deg": roll * RAD_TO_DEG,
        "pitch_deg": pitch * RAD_TO_DEG,
        "heading_deg": fdm["attitude/heading-true-rad"] * RAD_TO_DEG,
        "heading_rate_deg_s": heading_rate * RAD_TO_DEG,
        "north_m": fdm["position/distance-from-start-lat-mt"],
        "east_m": fdm["position/distance-from-start-lon-mt"],
        "nz": fdm["accelerations/Nz"],
        "stall_indicator": fdm["aero/stall-hyst-norm"],
        "aileron_cmd_norm": fdm["fcs/aileron-cmd-norm"],
        "rudder_cmd_norm": fdm["fcs/rudder-cmd-norm"],
        "elevator_cmd_norm": fdm["fcs/elevator-cmd-norm"],
        "left_aileron_pos_norm": fdm["fcs/left-aileron-pos-norm"],
        "right_aileron_pos_norm": fdm["fcs/right-aileron-pos-norm"],
        "rudder_pos_norm": fdm["fcs/rudder-pos-norm"],
        "elevator_pos_norm": fdm["fcs/elevator-pos-norm"],
        "flap_pos_norm": fdm["fcs/flap-pos-norm"],
        "total_fuel_lbs": fdm["propulsion/total-fuel-lbs"],
        "weight_lbs": fdm["inertia/weight-lbs"],
        "cg_x_in": fdm["inertia/cg-x-in"],
        "cg_y_in": fdm["inertia/cg-y-in"],
        "cg_z_in": fdm["inertia/cg-z-in"],
        "wind_north_fps": fdm["atmosphere/total-wind-north-fps"],
        "wind_east_fps": fdm["atmosphere/total-wind-east-fps"],
        "wind_down_fps": fdm["atmosphere/total-wind-down-fps"],
    }
    for index in range(int(config["fixture"]["engine_count"])):
        suffix = _suffix(index)
        label = f"engine_{index}"
        sample[f"{label}_running"] = fdm[f"propulsion/engine{suffix}/set-running"]
        sample[f"{label}_state"] = fdm[
            f"propulsion/engine{suffix}/{config['fixture']['engine_state_property']}"
        ]
        sample[f"{label}_thrust_lbs"] = fdm[f"propulsion/engine{suffix}/thrust-lbs"]
        sample[f"{label}_throttle_cmd_norm"] = fdm[f"fcs/throttle-cmd-norm{suffix}"]
        sample[f"{label}_throttle_pos_norm"] = fdm[f"fcs/throttle-pos-norm{suffix}"]
        sample[f"{label}_propeller_rpm"] = fdm[f"propulsion/engine{suffix}/propeller-rpm"]
    return sample


def _new_trimmed_fdm(model: str, target_ias_mps: float, config: dict[str, Any]) -> tuple[Any, dict[str, Any]]:
    fdm = jsbsim.FGFDMExec(None)
    fdm.set_debug_level(0)
    fdm.set_dt(float(config["simulation"]["timestep_s"]))
    if not fdm.load_model(model):
        raise RuntimeError("load_model returned false")
    _set_environment_and_aircraft(fdm, config)
    fixture = config["fixture"]
    fdm["position/terrain-elevation-asl-ft"] = float(fixture["synthetic_terrain_elevation_msl_m"]) * M_TO_FT
    fdm["ic/h-sl-ft"] = float(fixture["representative_altitude_msl_m"]) * M_TO_FT
    fdm["ic/vc-kts"] = target_ias_mps / KTS_TO_MPS
    fdm["ic/gamma-deg"] = 0.0
    fdm["ic/phi-deg"] = 0.0
    fdm["ic/theta-deg"] = 0.0
    fdm["ic/psi-true-deg"] = 0.0
    if not fdm.run_ic():
        raise RuntimeError("run_ic returned false")
    _set_environment_and_aircraft(fdm, config)
    fdm["position/terrain-elevation-asl-ft"] = float(fixture["synthetic_terrain_elevation_msl_m"]) * M_TO_FT
    _set_all_engines_running(fdm, config)
    if not fdm.run():
        raise RuntimeError("initialization frame failed")
    try:
        fdm.do_trim(1)
        trim_error = None
    except (jsbsim.TrimFailureError, RuntimeError) as exc:
        trim_error = f"{type(exc).__name__}: {exc}"
    trim = {
        "succeeded": trim_error is None,
        "error": trim_error,
        "completed_property": fdm["simulation/trim-completed"],
        "loaded_model_id": fdm.get_model_name(),
    }
    if trim_error is None:
        # JSBSim's trim control is scalar and updates engine 0. Mirroring that
        # value to all engines is an interface normalization, not gain tuning.
        symmetric = fdm["fcs/throttle-cmd-norm"]
        _set_symmetric_throttle(fdm, config, symmetric)
        _set_all_engines_running(fdm, config)
    return fdm, trim


def _measurement(samples: list[dict[str, float]], config: dict[str, Any]) -> tuple[list[dict[str, float]], dict[str, Any]]:
    start = float(config["simulation"]["replay_duration_s"]) - float(
        config["simulation"]["steady_measurement_window_s"]
    )
    window = [sample for sample in samples if sample["elapsed_s"] >= start - 1.0e-9]
    keys = [key for key in window[0] if key != "elapsed_s"]
    return window, {key: _stats([sample[key] for sample in window]) for key in keys}


def _surface_max(summary: dict[str, Any]) -> float:
    return max(
        abs(summary[key][bound])
        for key in ("left_aileron_pos_norm", "right_aileron_pos_norm", "rudder_pos_norm", "elevator_pos_norm")
        for bound in ("min", "max")
    )


def _base_checks(samples: list[dict[str, float]], summary: dict[str, Any], config: dict[str, Any]) -> dict[str, bool]:
    fixture = config["fixture"]
    acceptance = config["acceptance"]
    finite_keys = [key for key in samples[0] if key != "elapsed_s"]
    checks = {
        "finite_outputs": _all_finite(sample[key] for sample in samples for key in finite_keys),
        "measurement_window_complete": len(samples) >= round(
            config["simulation"]["steady_measurement_window_s"] / config["simulation"]["timestep_s"]
        ),
        "surface_usage_below_limit": _surface_max(summary) <= acceptance["normalized_surface_usage_max"],
        "stall_indicator_clear": summary["stall_indicator"]["max"] <= acceptance["stall_indicator_max"],
        "flaps_frozen_clean": abs(summary["flap_pos_norm"]["mean"] - fixture["flap_position_norm"]) <= 1.0e-9,
        "fuel_frozen": abs(summary["total_fuel_lbs"]["mean"] - fixture["expected_total_fuel_lbs"]) <= 1.0e-6,
        "weight_frozen": abs(summary["weight_lbs"]["mean"] - fixture["expected_weight_lbs"]) <= 1.0e-6,
        "zero_wind": max(
            abs(summary[key][bound])
            for key in ("wind_north_fps", "wind_east_fps", "wind_down_fps")
            for bound in ("min", "max")
        ) <= acceptance["zero_wind_residual_fps"],
    }
    for index in range(int(fixture["engine_count"])):
        checks[f"engine_{index}_running"] = summary[f"engine_{index}_running"]["min"] >= 1.0
        checks[f"engine_{index}_state_positive"] = summary[f"engine_{index}_state"]["min"] > 0.0
        checks[f"engine_{index}_propeller_rotating"] = summary[f"engine_{index}_propeller_rpm"]["min"] > 0.0
    return checks


def _classify(checks: dict[str, bool], *, aircraft_limit_evidence: bool) -> tuple[str, list[str]]:
    failed = [name for name, passed in checks.items() if not passed]
    if not failed:
        return "VALID", []
    if aircraft_limit_evidence:
        return "INFEASIBLE", failed
    return "UNKNOWN", failed


def _run_straight_once(model: str, target_ias: float, repeat: int, config: dict[str, Any], provenance_id: str) -> dict[str, Any]:
    record: dict[str, Any] = {
        "provenance_id": provenance_id,
        "run_id": f"u1-{model}-straight-{target_ias:.0f}mps-r{repeat}",
        "aircraft_model": model,
        "maneuver": "straight_level",
        "repeat_index": repeat,
        "requested": {"ias_mps": target_ias, "bank_deg": 0.0, "vertical_speed_mps": 0.0},
        "status": "UNKNOWN",
        "status_reasons": [],
    }
    try:
        fdm, trim = _new_trimmed_fdm(model, target_ias, config)
        record["trim"] = trim
        baseline_throttle = (
            fdm["fcs/throttle-cmd-norm"]
            if trim["succeeded"]
            else float(config["fixture"]["initial_symmetric_throttle_command_norm"])
        )
        record["initialization_mode"] = (
            "built_in_level_trim_then_feedback"
            if trim["succeeded"]
            else "untrimmed_ic_then_bounded_feedback"
        )
        samples = []
        steps = round(config["simulation"]["replay_duration_s"] / config["simulation"]["timestep_s"])
        for step in range(1, steps + 1):
            _set_symmetric_throttle(fdm, config, baseline_throttle)
            if not fdm.run():
                raise RuntimeError("replay terminated early")
            samples.append(_snapshot(fdm, step * config["simulation"]["timestep_s"], config))
        window, summary = _measurement(samples, config)
        checks = _base_checks(window, summary, config)
        checks.update(
            {
                "ias_tracking": max(abs(sample["ias_mps"] - target_ias) / target_ias for sample in window)
                <= config["acceptance"]["ias_tracking_relative"],
                "level_vertical_speed": max(abs(sample["vertical_speed_mps"]) for sample in window)
                <= config["acceptance"]["level_vertical_speed_absolute_mps"],
                "bank_near_zero": max(abs(sample["roll_deg"]) for sample in window)
                <= config["acceptance"]["bank_tracking_absolute_deg"],
                "beta_bounded": max(abs(sample["beta_deg"]) for sample in window)
                <= config["acceptance"]["beta_absolute_deg"],
            }
        )
        power_limited = any(
            summary[f"engine_{i}_throttle_pos_norm"]["mean"]
            >= config["acceptance"]["throttle_position_severe_boundary_norm"]
            for i in range(config["fixture"]["engine_count"])
        ) and not checks["ias_tracking"]
        stalled = not checks["stall_indicator_clear"]
        status, reasons = _classify(checks, aircraft_limit_evidence=power_limited or stalled)
        record.update(
            {
                "baseline_trim_throttle_norm": baseline_throttle,
                "measurement": summary,
                "checks": checks,
                "aircraft_limit_evidence": {"power_limited": power_limited, "stall_indicator": stalled},
                "status": status,
                "status_reasons": reasons,
            }
        )
    except Exception as exc:
        record["status_reasons"] = [f"{type(exc).__name__}: {exc}"]
    return record


def _controller_value(error: float, integral: float, spec: dict[str, Any]) -> tuple[float, bool]:
    raw = spec["kp_cmd_per_deg"] * error + spec["ki_cmd_per_deg_s"] * integral
    command = _clamp(raw, spec["command_min_norm"], spec["command_max_norm"])
    return command, not math.isclose(raw, command, abs_tol=1.0e-12)


def _trajectory(window: list[dict[str, float]]) -> dict[str, float | None]:
    headings = [window[0]["heading_deg"]]
    for sample in window[1:]:
        delta = (sample["heading_deg"] - headings[-1] + 180.0) % 360.0 - 180.0
        headings.append(headings[-1] + delta)
    times = [sample["elapsed_s"] for sample in window]
    tm = fmean(times)
    hm = fmean(headings)
    slope = sum((t - tm) * (h - hm) for t, h in zip(times, headings)) / sum((t - tm) ** 2 for t in times)
    arc = sum(
        math.hypot(current["north_m"] - previous["north_m"], current["east_m"] - previous["east_m"])
        for previous, current in zip(window, window[1:])
    )
    courses = []
    for previous, current in zip(window, window[1:]):
        dn = current["north_m"] - previous["north_m"]
        de = current["east_m"] - previous["east_m"]
        if math.hypot(dn, de) > 1.0e-9:
            angle = math.degrees(math.atan2(de, dn))
            if courses:
                angle = courses[-1] + (angle - courses[-1] + 180.0) % 360.0 - 180.0
            courses.append(angle)
    course_change = courses[-1] - courses[0]
    radius = arc / abs(math.radians(course_change)) if abs(course_change) > 1.0e-9 else None
    mean_tas = fmean(sample["tas_mps"] for sample in window)
    mean_bank = fmean(sample["roll_deg"] for sample in window)
    tangent = math.tan(math.radians(abs(mean_bank)))
    theory = mean_tas * mean_tas / (9.80665 * tangent) if tangent > 1.0e-12 else None
    difference = abs(radius - theory) / theory if radius is not None and theory is not None else None
    rates = [sample["heading_rate_deg_s"] for sample in window]
    rate_cv = pstdev(rates) / max(abs(fmean(rates)), 1.0e-12)
    return {
        "heading_change_deg": headings[-1] - headings[0],
        "turn_rate_deg_s": slope,
        "turn_rate_coefficient_of_variation": rate_cv,
        "trajectory_arc_length_m": arc,
        "ground_track_change_deg": course_change,
        "measured_radius_m": radius,
        "theoretical_radius_m": theory,
        "radius_theory_relative_difference": difference,
    }


def _run_controlled_once(
    model: str,
    maneuver: str,
    target_ias: float,
    target_value: float,
    repeat: int,
    config: dict[str, Any],
    provenance_id: str,
) -> dict[str, Any]:
    target_bank = target_value if maneuver == "turn" else 0.0
    target_vz = target_value if maneuver == "vertical" else 0.0
    record: dict[str, Any] = {
        "provenance_id": provenance_id,
        "run_id": f"u1-{model}-{maneuver}-{target_ias:.0f}-{target_value:+.0f}-r{repeat}",
        "aircraft_model": model,
        "maneuver": maneuver,
        "repeat_index": repeat,
        "requested": {
            "ias_mps": target_ias,
            "bank_deg": target_bank,
            "vertical_speed_mps": target_vz,
            "heading_rate_commanded": False,
            "gamma_used_as_command": False,
        },
        "status": "UNKNOWN",
        "status_reasons": [],
    }
    try:
        fdm, trim = _new_trimmed_fdm(model, target_ias, config)
        record["trim"] = trim
        baseline_throttle = (
            fdm["fcs/throttle-cmd-norm"]
            if trim["succeeded"]
            else float(config["fixture"]["initial_symmetric_throttle_command_norm"])
        )
        record["initialization_mode"] = (
            "built_in_level_trim_then_feedback"
            if trim["succeeded"]
            else "untrimmed_ic_then_bounded_feedback"
        )
        dt = config["simulation"]["timestep_s"]
        steps = round(config["simulation"]["replay_duration_s"] / dt)
        measurement_start = config["simulation"]["replay_duration_s"] - config["simulation"]["steady_measurement_window_s"]
        ramp = config["simulation"]["target_ramp_s"]
        bank_spec = config["controller"]["bank"]
        beta_spec = config["controller"]["beta"]
        vz_spec = config["controller"]["vertical_speed"]
        ias_spec = config["controller"]["ias"]
        integrals = {"bank": 0.0, "beta": 0.0, "vz": 0.0, "ias": 0.0}
        saturation_any = {"aileron": False, "rudder": False, "elevator": False, "throttle": False}
        saturation_window = dict.fromkeys(saturation_any, False)
        samples = []
        for step in range(1, steps + 1):
            elapsed = step * dt
            fraction = min(1.0, elapsed / ramp)
            commanded_bank = target_bank * fraction
            commanded_vz = target_vz * fraction
            errors = {
                "bank": commanded_bank - fdm["attitude/roll-rad"] * RAD_TO_DEG,
                "beta": beta_spec["target_deg"] - fdm["aero/beta-deg"],
                "vz": commanded_vz - fdm["velocities/h-dot-fps"] * FPS_TO_MPS,
                "ias": target_ias - fdm["velocities/vc-kts"] * KTS_TO_MPS,
            }
            integrals["bank"] = _clamp(integrals["bank"] + errors["bank"] * dt, -bank_spec["integral_limit_deg_s"], bank_spec["integral_limit_deg_s"])
            integrals["beta"] = _clamp(integrals["beta"] + errors["beta"] * dt, -beta_spec["integral_limit_deg_s"], beta_spec["integral_limit_deg_s"])
            integrals["vz"] = _clamp(integrals["vz"] + errors["vz"] * dt, -vz_spec["integral_limit_m"], vz_spec["integral_limit_m"])
            integrals["ias"] = _clamp(integrals["ias"] + errors["ias"] * dt, -ias_spec["integral_limit_m"], ias_spec["integral_limit_m"])
            raw = {
                "aileron": bank_spec["kp_cmd_per_deg"] * errors["bank"] + bank_spec["ki_cmd_per_deg_s"] * integrals["bank"],
                "rudder": beta_spec["kp_cmd_per_deg"] * errors["beta"] + beta_spec["ki_cmd_per_deg_s"] * integrals["beta"],
                "elevator": vz_spec["kp_cmd_per_mps"] * errors["vz"] + vz_spec["ki_cmd_per_m"] * integrals["vz"],
                "throttle": baseline_throttle + ias_spec["kp_cmd_per_mps"] * errors["ias"] + ias_spec["ki_cmd_per_m"] * integrals["ias"],
            }
            commands = {
                "aileron": _clamp(raw["aileron"], bank_spec["command_min_norm"], bank_spec["command_max_norm"]),
                "rudder": _clamp(raw["rudder"], beta_spec["command_min_norm"], beta_spec["command_max_norm"]),
                "elevator": _clamp(raw["elevator"], vz_spec["command_min_norm"], vz_spec["command_max_norm"]),
                "throttle": _clamp(raw["throttle"], ias_spec["command_min_norm"], ias_spec["command_max_norm"]),
            }
            for key in saturation_any:
                clipped = not math.isclose(raw[key], commands[key], abs_tol=1.0e-12)
                saturation_any[key] |= clipped
                if elapsed >= measurement_start:
                    saturation_window[key] |= clipped
            fdm["fcs/aileron-cmd-norm"] = commands["aileron"]
            fdm["fcs/rudder-cmd-norm"] = commands["rudder"]
            fdm["fcs/elevator-cmd-norm"] = commands["elevator"]
            _set_symmetric_throttle(fdm, config, commands["throttle"])
            if not fdm.run():
                raise RuntimeError("replay terminated early")
            sample = _snapshot(fdm, elapsed, config)
            sample["commanded_bank_deg"] = commanded_bank
            sample["commanded_vertical_speed_mps"] = commanded_vz
            samples.append(sample)
        window, summary = _measurement(samples, config)
        checks = _base_checks(window, summary, config)
        checks.update(
            {
                "ias_tracking": max(abs(sample["ias_mps"] - target_ias) / target_ias for sample in window)
                <= config["acceptance"]["ias_tracking_relative"],
                "bank_tracking": max(abs(sample["roll_deg"] - target_bank) for sample in window)
                <= config["acceptance"]["bank_tracking_absolute_deg"],
                "vertical_speed_tracking": max(abs(sample["vertical_speed_mps"] - target_vz) for sample in window)
                <= config["acceptance"]["vertical_speed_tracking_absolute_mps"],
                "beta_bounded": max(abs(sample["beta_deg"]) for sample in window)
                <= config["acceptance"]["beta_absolute_deg"],
                "outer_loop_not_clipped_in_measurement": not any(saturation_window.values()),
            }
        )
        trajectory = _trajectory(window) if maneuver == "turn" else None
        if trajectory is not None:
            checks.update(
                {
                    "turn_rate_correct_sign": trajectory["turn_rate_deg_s"] * target_bank > 0.0,
                    "turn_rate_stable": trajectory["turn_rate_coefficient_of_variation"]
                    <= config["acceptance"]["turn_rate_coefficient_of_variation_max"],
                    "measured_radius_valid": trajectory["measured_radius_m"] is not None
                    and trajectory["measured_radius_m"] > 0.0,
                    "theory_radius_cross_check": trajectory["radius_theory_relative_difference"] is not None
                    and trajectory["radius_theory_relative_difference"]
                    <= config["acceptance"]["theory_radius_relative_difference_max"],
                }
            )
        throttle_boundary = any(
            summary[f"engine_{i}_throttle_pos_norm"]["mean"] >= config["acceptance"]["throttle_position_severe_boundary_norm"]
            for i in range(config["fixture"]["engine_count"])
        )
        power_limited = throttle_boundary and (not checks["ias_tracking"] or not checks["vertical_speed_tracking"])
        stalled = not checks["stall_indicator_clear"]
        idle_boundary = all(
            summary[f"engine_{i}_throttle_pos_norm"]["max"] <= 1.0e-9
            for i in range(config["fixture"]["engine_count"])
        )
        model_mismatch = (
            not trim["succeeded"]
            and idle_boundary
            and not checks["ias_tracking"]
            and not checks["vertical_speed_tracking"]
            and not saturation_window["elevator"]
            and not stalled
        )
        status, reasons = _classify(checks, aircraft_limit_evidence=power_limited or stalled)
        record.update(
            {
                "baseline_trim_throttle_norm": baseline_throttle,
                "measurement": summary,
                "trajectory": trajectory,
                "controller": {
                    "policy": config["controller"]["policy"],
                    "saturation_any_time": saturation_any,
                    "saturation_in_measurement": saturation_window,
                },
                "checks": checks,
                "aircraft_limit_evidence": {
                    "power_limited": power_limited,
                    "throttle_at_severe_boundary": throttle_boundary,
                    "idle_throttle_boundary": idle_boundary,
                    "stall_indicator": stalled,
                    "repeatable_model_fixture_mismatch_candidate": model_mismatch,
                },
                "status": status,
                "status_reasons": reasons,
            }
        )
    except Exception as exc:
        record["status_reasons"] = [f"{type(exc).__name__}: {exc}"]
    return record


def _repeatability(records: list[dict[str, Any]], maneuver: str, config: dict[str, Any]) -> dict[str, Any]:
    if any("measurement" not in record for record in records):
        return {"passed": False, "reason": "missing_measurement", "metrics": {}}
    names = ["ias_mps", "vertical_speed_mps", "roll_deg", "alpha_deg", "beta_deg"]
    if maneuver == "turn":
        names.extend(["turn_rate_deg_s", "measured_radius_m"])
    metrics = {}
    for name in names:
        if name == "turn_rate_deg_s":
            values = [float(record["trajectory"]["turn_rate_deg_s"]) for record in records]
        elif name == "measured_radius_m":
            values = [float(record["trajectory"]["measured_radius_m"]) for record in records]
        else:
            values = [float(record["measurement"][name]["mean"]) for record in records]
        mean = fmean(values)
        scale = max(abs(mean), 1.0) if name in {"vertical_speed_mps", "roll_deg", "beta_deg"} else max(abs(mean), 1.0e-9)
        deviation = max(abs(value - mean) / scale for value in values)
        metrics[name] = {"values": values, "mean": mean, "maximum_relative_deviation": deviation, "passed": deviation <= config["acceptance"]["cold_start_repeatability_relative"]}
    return {"passed": all(item["passed"] for item in metrics.values()), "metrics": metrics}


def _aggregate(records: list[dict[str, Any]], maneuver: str, requested: dict[str, Any], config: dict[str, Any]) -> dict[str, Any]:
    repeatability = _repeatability(records, maneuver, config)
    if not repeatability["passed"]:
        for record in records:
            if record["status"] == "VALID":
                record["status"] = "UNKNOWN"
                record["status_reasons"].append("cold_start_repeatability_failed")
    statuses = [record["status"] for record in records]
    status = "VALID" if all(value == "VALID" for value in statuses) and repeatability["passed"] else (
        "INFEASIBLE" if all(value == "INFEASIBLE" for value in statuses) else "UNKNOWN"
    )
    measured = None
    if all("measurement" in record for record in records):
        measured_names = [
            "ias_mps", "cas_kts", "tas_mps", "altitude_msl_m", "vertical_speed_mps",
            "gamma_deg", "alpha_deg", "beta_deg", "roll_deg", "nz",
            "elevator_cmd_norm", "aileron_cmd_norm", "rudder_cmd_norm",
            "elevator_pos_norm", "left_aileron_pos_norm", "right_aileron_pos_norm",
            "rudder_pos_norm", "stall_indicator",
        ]
        measured_names.extend(
            f"engine_{index}_throttle_pos_norm"
            for index in range(config["fixture"]["engine_count"])
        )
        measured = {
            name: fmean(record["measurement"][name]["mean"] for record in records)
            for name in measured_names
        }
        measured["maximum_surface_usage_norm"] = max(_surface_max(record["measurement"]) for record in records)
        if maneuver == "turn":
            measured.update(
                {
                    "turn_rate_deg_s": fmean(record["trajectory"]["turn_rate_deg_s"] for record in records),
                    "measured_radius_m": fmean(record["trajectory"]["measured_radius_m"] for record in records),
                    "theoretical_radius_m": fmean(record["trajectory"]["theoretical_radius_m"] for record in records),
                    "radius_theory_relative_difference": fmean(record["trajectory"]["radius_theory_relative_difference"] for record in records),
                }
            )
    return {
        "maneuver": maneuver,
        "requested": requested,
        "status": status,
        "run_ids": [record["run_id"] for record in records],
        "run_statuses": statuses,
        "run_status_reasons": [record["status_reasons"] for record in records],
        "repeatability": repeatability,
        "model_fixture_mismatch_evidence": all(
            record.get("aircraft_limit_evidence", {}).get(
                "repeatable_model_fixture_mismatch_candidate", False
            )
            for record in records
        ) and repeatability["passed"],
        "measured": measured,
        "raw_capability": None,
        "derated_capability": None,
    }


def _run_point(model: str, maneuver: str, target_ias: float, target_value: float, config: dict[str, Any], provenance_id: str) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    run_config = copy.deepcopy(config)
    if model == config["aircraft"]["backup_model"]:
        run_config["fixture"].update(config["backup_fixture"])
    repeats = int(run_config["simulation"]["cold_start_repetitions"])
    records = [
        _run_controlled_once(
            model, maneuver, target_ias, target_value, repeat, run_config, provenance_id
        )
        for repeat in range(1, repeats + 1)
    ]
    requested = {
        "ias_mps": target_ias,
        "bank_deg": target_value if maneuver == "turn" else 0.0,
        "vertical_speed_mps": target_value if maneuver == "vertical" else 0.0,
    }
    return records, _aggregate(records, maneuver, requested, run_config)


def run_u1() -> dict[str, Any]:
    config = _load_config()
    provenance = _provenance(config)
    all_runs: list[dict[str, Any]] = []
    points: list[dict[str, Any]] = []
    executed = {"dhc6_straight": False, "dhc6_turn": False, "dhc6_vertical": False, "c182_backup": False}

    executed["dhc6_straight"] = True
    for speed in config["test_matrix"]["straight_level_ias_mps"]:
        runs, point = _run_point("DHC6", "straight", float(speed), 0.0, config, provenance["provenance_id"])
        all_runs.extend(runs)
        points.append(point)

    straight_points = [point for point in points if point["maneuver"] == "straight"]
    if all(point["status"] == "VALID" for point in straight_points):
        executed["dhc6_turn"] = True
        for item in config["test_matrix"]["turn_points"]:
            runs, point = _run_point("DHC6", "turn", float(item["ias_mps"]), float(item["bank_deg"]), config, provenance["provenance_id"])
            all_runs.extend(runs)
            points.append(point)

    turn_points = [point for point in points if point["maneuver"] == "turn"]
    if executed["dhc6_turn"] and all(point["status"] == "VALID" for point in turn_points):
        executed["dhc6_vertical"] = True
        for item in config["test_matrix"]["vertical_points"]:
            runs, point = _run_point("DHC6", "vertical", float(item["ias_mps"]), float(item["vertical_speed_mps"]), config, provenance["provenance_id"])
            all_runs.extend(runs)
            points.append(point)

    dhc6_infeasible = any(point["status"] == "INFEASIBLE" for point in points)
    dhc6_profile_mismatch = any(
        point["maneuver"] == "straight"
        and point["requested"]["ias_mps"] == 35.0
        and point["model_fixture_mismatch_evidence"]
        for point in points
    )
    dhc6_unknown = any(point["status"] == "UNKNOWN" for point in points)
    required_count = len(config["test_matrix"]["straight_level_ias_mps"]) + len(config["test_matrix"]["turn_points"]) + len(config["test_matrix"]["vertical_points"])
    if len(points) == required_count and all(point["status"] == "VALID" for point in points):
        suitability = "ACCEPTED_FOR_PLANNER_VALIDATION"
    elif dhc6_infeasible or dhc6_profile_mismatch:
        suitability = "REJECTED_FOR_PROFILE"
    else:
        suitability = "INCONCLUSIVE"

    backup_points: list[dict[str, Any]] = []
    if suitability == "REJECTED_FOR_PROFILE":
        executed["c182_backup"] = True
        for item in config["test_matrix"]["backup_fail_fast_points"]:
            maneuver = item["maneuver"]
            value = float(item.get("bank_deg", item.get("vertical_speed_mps", 0.0)))
            runs, point = _run_point("c182", maneuver, float(item["ias_mps"]), value, config, provenance["provenance_id"])
            all_runs.extend(runs)
            point["aircraft_model"] = "c182"
            backup_points.append(point)
        points.extend(backup_points)

    backup_status = "NOT_RUN_DHC6_NOT_REJECTED"
    if backup_points:
        backup_status = "PROMISING_BACKUP" if all(point["status"] == "VALID" for point in backup_points) else "BACKUP_INCONCLUSIVE"
    step_pass = suitability == "ACCEPTED_FOR_PLANNER_VALIDATION" or suitability == "REJECTED_FOR_PROFILE"
    result = {
        "provenance_id": provenance["provenance_id"],
        "step": "U1",
        "step_pass": step_pass,
        "step_status": "PASS" if step_pass else "PARTIAL",
        "dhc6_suitability": suitability,
        "c182_backup_status": backup_status,
        "current_aircraft_status": (
            "DHC6 ACCEPTED" if suitability == "ACCEPTED_FOR_PLANNER_VALIDATION" else
            "DHC6 REJECTED / c182 PROMISING" if backup_status == "PROMISING_BACKUP" else
            "INCONCLUSIVE"
        ),
        "executed_gates": executed,
        "dhc6_unknown_present": dhc6_unknown,
        "dhc6_profile_mismatch_evidence": dhc6_profile_mismatch,
        "point_count": len(points),
        "run_count": len(all_runs),
        "status_counts": {status: sum(point["status"] == status for point in points) for status in ("VALID", "INFEASIBLE", "UNKNOWN")},
        "operational_limits_are_raw_aircraft_capability": False,
        "full_characterization_performed": False,
        "controller_tuning_performed": False,
        "planner_modified": False,
        "next_stage_started": False,
    }
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    for filename, payload in {
        "u1_provenance.json": provenance,
        "u1_runs.json": all_runs,
        "u1_points.json": points,
        "u1_result.json": result,
    }.items():
        (RESULTS_DIR / filename).write_text(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
    return result


if __name__ == "__main__":
    outcome = run_u1()
    print(json.dumps(outcome, indent=2, sort_keys=True, allow_nan=False))
    raise SystemExit(0 if outcome["step_pass"] else 1)
