from __future__ import annotations

import hashlib
import json
import math
import platform
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from statistics import fmean, pstdev
from typing import Any

import yaml


HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
CONFIG_PATH = HERE / "u5_2_finalist_general_validation_configuration.yaml"
HARNESS_PATH = Path(__file__).resolve()
sys.path.insert(0, str(HERE))

import u1_profile_validation as u1
import u3_aircraft_selection as u3
import u5_practical_aircraft_rescreen as u5


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write(name: str, payload: dict[str, Any]) -> None:
    RESULTS.mkdir(parents=True, exist_ok=True)
    (RESULTS / name).write_text(
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False),
        encoding="utf-8",
    )


def _remap_u5_ids(
    records: list[dict[str, Any]], point: dict[str, Any], analysis: dict[str, Any]
) -> None:
    mapping = {}
    for record in records:
        old = record["run_id"]
        new = old.replace("u5-", "u5.2-", 1)
        record["run_id"] = new
        mapping[old] = new
    point["run_ids"] = [mapping[x] for x in point["run_ids"]]
    analysis["run_ids"] = list(point["run_ids"])


def _run_existing_maneuver_adapter(
    model: str,
    candidate: dict[str, Any],
    altitude_m: float,
    speed_mps: float,
    maneuver: str,
    target: float,
    u5_config: dict[str, Any],
    u3_config: dict[str, Any],
    provenance_id: str,
) -> tuple[list[dict[str, Any]], dict[str, Any], dict[str, Any]]:
    records, point, analysis = u5._run_point(
        model,
        candidate,
        altitude_m,
        speed_mps,
        maneuver,
        target,
        u5_config,
        u3_config,
        provenance_id,
    )
    _remap_u5_ids(records, point, analysis)
    return records, point, analysis


def _run_combined_once(
    model: str,
    altitude_m: float,
    target_ias: float,
    target_bank: float,
    target_vz: float,
    repeat: int,
    config: dict[str, Any],
    provenance_id: str,
) -> dict[str, Any]:
    record: dict[str, Any] = {
        "provenance_id": provenance_id,
        "run_id": (
            f"u5.2-h{altitude_m:g}m-{model}-combined-{target_ias:.0f}-"
            f"b{target_bank:+.0f}-vz{target_vz:+.1f}-r{repeat}"
        ),
        "aircraft_model": model,
        "maneuver": "combined",
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
        fdm, trim = u1._new_trimmed_fdm(model, target_ias, config)
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
        measurement_start = (
            config["simulation"]["replay_duration_s"]
            - config["simulation"]["steady_measurement_window_s"]
        )
        ramp = config["simulation"]["target_ramp_s"]
        bank_spec = config["controller"]["bank"]
        beta_spec = config["controller"]["beta"]
        vz_spec = config["controller"]["vertical_speed"]
        ias_spec = config["controller"]["ias"]
        integrals = {"bank": 0.0, "beta": 0.0, "vz": 0.0, "ias": 0.0}
        saturation_any = {"aileron": False, "rudder": False, "elevator": False, "throttle": False}
        saturation_window = dict.fromkeys(saturation_any, False)
        clamp_counts = dict.fromkeys(saturation_any, 0)
        measurement_steps = 0
        samples = []
        for step in range(1, steps + 1):
            elapsed = step * dt
            fraction = min(1.0, elapsed / ramp)
            commanded_bank = target_bank * fraction
            commanded_vz = target_vz * fraction
            errors = {
                "bank": commanded_bank - fdm["attitude/roll-rad"] * u1.RAD_TO_DEG,
                "beta": beta_spec["target_deg"] - fdm["aero/beta-deg"],
                "vz": commanded_vz - fdm["velocities/h-dot-fps"] * u1.FPS_TO_MPS,
                "ias": target_ias - fdm["velocities/vc-kts"] * u1.KTS_TO_MPS,
            }
            integrals["bank"] = u1._clamp(
                integrals["bank"] + errors["bank"] * dt,
                -bank_spec["integral_limit_deg_s"], bank_spec["integral_limit_deg_s"],
            )
            integrals["beta"] = u1._clamp(
                integrals["beta"] + errors["beta"] * dt,
                -beta_spec["integral_limit_deg_s"], beta_spec["integral_limit_deg_s"],
            )
            integrals["vz"] = u1._clamp(
                integrals["vz"] + errors["vz"] * dt,
                -vz_spec["integral_limit_m"], vz_spec["integral_limit_m"],
            )
            integrals["ias"] = u1._clamp(
                integrals["ias"] + errors["ias"] * dt,
                -ias_spec["integral_limit_m"], ias_spec["integral_limit_m"],
            )
            raw = {
                "aileron": bank_spec["kp_cmd_per_deg"] * errors["bank"] + bank_spec["ki_cmd_per_deg_s"] * integrals["bank"],
                "rudder": beta_spec["kp_cmd_per_deg"] * errors["beta"] + beta_spec["ki_cmd_per_deg_s"] * integrals["beta"],
                "elevator": vz_spec["kp_cmd_per_mps"] * errors["vz"] + vz_spec["ki_cmd_per_m"] * integrals["vz"],
                "throttle": baseline_throttle + ias_spec["kp_cmd_per_mps"] * errors["ias"] + ias_spec["ki_cmd_per_m"] * integrals["ias"],
            }
            commands = {
                "aileron": u1._clamp(raw["aileron"], bank_spec["command_min_norm"], bank_spec["command_max_norm"]),
                "rudder": u1._clamp(raw["rudder"], beta_spec["command_min_norm"], beta_spec["command_max_norm"]),
                "elevator": u1._clamp(raw["elevator"], vz_spec["command_min_norm"], vz_spec["command_max_norm"]),
                "throttle": u1._clamp(raw["throttle"], ias_spec["command_min_norm"], ias_spec["command_max_norm"]),
            }
            if elapsed >= measurement_start:
                measurement_steps += 1
            for key in saturation_any:
                clipped = not math.isclose(raw[key], commands[key], abs_tol=1.0e-12)
                saturation_any[key] |= clipped
                if elapsed >= measurement_start:
                    saturation_window[key] |= clipped
                    clamp_counts[key] += int(clipped)
            fdm["fcs/aileron-cmd-norm"] = commands["aileron"]
            fdm["fcs/rudder-cmd-norm"] = commands["rudder"]
            fdm["fcs/elevator-cmd-norm"] = commands["elevator"]
            u1._set_symmetric_throttle(fdm, config, commands["throttle"])
            if not fdm.run():
                raise RuntimeError("replay terminated early")
            sample = u1._snapshot(fdm, elapsed, config)
            sample["commanded_bank_deg"] = commanded_bank
            sample["commanded_vertical_speed_mps"] = commanded_vz
            samples.append(sample)
        window, summary = u1._measurement(samples, config)
        checks = u1._base_checks(window, summary, config)
        checks.update(
            {
                "ias_tracking": max(abs(s["ias_mps"] - target_ias) / target_ias for s in window)
                <= config["acceptance"]["ias_tracking_relative"],
                "bank_tracking": max(abs(s["roll_deg"] - target_bank) for s in window)
                <= config["acceptance"]["bank_tracking_absolute_deg"],
                "vertical_speed_tracking": max(abs(s["vertical_speed_mps"] - target_vz) for s in window)
                <= config["acceptance"]["vertical_speed_tracking_absolute_mps"],
                "beta_bounded": max(abs(s["beta_deg"]) for s in window)
                <= config["acceptance"]["beta_absolute_deg"],
                "outer_loop_not_clipped_in_measurement": not any(saturation_window.values()),
            }
        )
        trajectory = u1._trajectory(window)
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
            summary[f"engine_{i}_throttle_pos_norm"]["mean"]
            >= config["acceptance"]["throttle_position_severe_boundary_norm"]
            for i in range(config["fixture"]["engine_count"])
        )
        power_limited = throttle_boundary and (
            not checks["ias_tracking"] or not checks["vertical_speed_tracking"]
        )
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
        status, reasons = u1._classify(
            checks, aircraft_limit_evidence=power_limited or stalled
        )
        record.update(
            {
                "baseline_trim_throttle_norm": baseline_throttle,
                "measurement": summary,
                "trajectory": trajectory,
                "controller": {
                    "policy": config["controller"]["policy"],
                    "saturation_any_time": saturation_any,
                    "saturation_in_measurement": saturation_window,
                    "clamp_fraction_in_measurement": {
                        key: count / max(measurement_steps, 1)
                        for key, count in clamp_counts.items()
                    },
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


def _combined_settling(
    group: list[dict[str, float]],
    speed: float,
    bank: float,
    vz: float,
    contract: dict[str, Any],
) -> dict[str, Any]:
    tolerance_vz = max(contract["vertical_mean_error_max_mps"], 0.25 * abs(vz))

    def within(sample: dict[str, float]) -> bool:
        return (
            sample["ias_mps"] / speed >= contract["ias_retention_min"]
            and abs(sample["roll_deg"] - bank) <= contract["bank_mean_error_max_deg"]
            and abs(sample["vertical_speed_mps"] - vz) <= tolerance_vz
        )

    after_ramp = [sample for sample in group if sample["elapsed_s"] >= 8.0]
    last_bad = max(
        (sample["elapsed_s"] for sample in after_ramp if not within(sample)),
        default=None,
    )
    settled = bool(after_ramp and within(after_ramp[-1]))
    settling_time = (8.0 if last_bad is None else last_bad + 0.01) if settled else None
    return {
        "settled_by_screening_deadline": settled
        and settling_time <= contract["settling_deadline_s"],
        "settling_time_s": settling_time,
        "screening_settling_deadline_s": contract["settling_deadline_s"],
    }


def _run_combined_point(
    model: str,
    candidate: dict[str, Any],
    altitude_m: float,
    speed_mps: float,
    bank_deg: float,
    vz_mps: float,
    u5_config: dict[str, Any],
    u3_config: dict[str, Any],
    config: dict[str, Any],
    provenance_id: str,
    existing_points: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], dict[str, Any], dict[str, Any]]:
    run_config = u5._run_config(model, candidate, altitude_m, u5_config, u3_config)
    u5._ACTIVE_MIXTURE_MODE = candidate["mixture_mode"]
    u5._CAPTURE = []
    try:
        records = [
            _run_combined_once(
                model, altitude_m, speed_mps, bank_deg, vz_mps, repeat,
                run_config, provenance_id,
            )
            for repeat in range(1, int(run_config["simulation"]["cold_start_repetitions"]) + 1)
        ]
        groups = u5._split_repeats(
            u5._CAPTURE, int(run_config["simulation"]["cold_start_repetitions"])
        )
    finally:
        u5._CAPTURE = None
    requested = {
        "ias_mps": speed_mps,
        "bank_deg": bank_deg,
        "vertical_speed_mps": vz_mps,
    }
    point = u1._aggregate(records, "turn", requested, run_config)
    point.update(
        {
            "maneuver": "combined",
            "aircraft_model": model,
            "altitude_msl_m": altitude_m,
            "provenance_id": provenance_id,
        }
    )
    contract = config["actual_stable_combined_contract"]
    measured = point["measured"]
    windows = [
        [sample for sample in group if sample["elapsed_s"] >= 20.0 - 1.0e-9]
        for group in groups
    ]
    stability = {
        "ias_mps": [u5._stats([s["ias_mps"] for s in window]) for window in windows],
        "bank_deg": [u5._stats([s["roll_deg"] for s in window]) for window in windows],
        "vertical_speed_mps": [u5._stats([s["vertical_speed_mps"] for s in window]) for window in windows],
    }
    settling = [
        _combined_settling(group, speed_mps, bank_deg, vz_mps, contract)
        for group in groups
    ]
    saturation = {
        key: any(r.get("controller", {}).get("saturation_in_measurement", {}).get(key, False) for r in records)
        for key in ("aileron", "rudder", "elevator", "throttle")
    }
    finite = measured is not None and all(
        not isinstance(value, float) or math.isfinite(value)
        for value in measured.values()
    )
    stable_window = (
        all(item["finite"] for values in stability.values() for item in values)
        and max(item["std"] for item in stability["ias_mps"]) <= contract["measurement_ias_std_max_mps"]
        and max(item["std"] for item in stability["bank_deg"]) <= contract["measurement_bank_std_max_deg"]
        and max(item["std"] for item in stability["vertical_speed_mps"]) <= contract["measurement_vertical_speed_std_max_mps"]
    )
    engine = u5._engine_summary(records, int(run_config["fixture"]["engine_count"]))
    tolerance_vz = max(contract["vertical_mean_error_max_mps"], 0.25 * abs(vz_mps))
    actual_stable = (
        finite
        and engine["healthy"]
        and point["repeatability"]["passed"]
        and stable_window
        and all(item["settled_by_screening_deadline"] for item in settling)
        and measured["stall_indicator"] <= 0.0
        and measured["maximum_surface_usage_norm"] < contract["normalized_surface_usage_max"]
        and measured["ias_mps"] / speed_mps >= contract["ias_retention_min"]
        and math.copysign(1.0, measured["roll_deg"]) == math.copysign(1.0, bank_deg)
        and abs(measured["roll_deg"] - bank_deg) <= contract["bank_mean_error_max_deg"]
        and math.copysign(1.0, measured["vertical_speed_mps"]) == math.copysign(1.0, vz_mps)
        and abs(measured["vertical_speed_mps"] - vz_mps) <= tolerance_vz
    )
    throttle = max(
        measured[f"engine_{i}_throttle_pos_norm"]
        for i in range(run_config["fixture"]["engine_count"])
    )
    cautions = []
    if point["status"] != "VALID":
        cautions.append(f"strict_status_{point['status']}")
    if any(saturation.values()):
        cautions.append("controller_saturation")
    if throttle >= contract["throttle_caution_threshold_norm"]:
        cautions.append("low_power_margin")
    usability = "UNUSABLE" if not actual_stable else "MARGINAL" if cautions else "USABLE"
    level_point = next(
        (
            p for p in existing_points
            if p["aircraft_model"] == model
            and p["altitude_msl_m"] == altitude_m
            and p["maneuver"] == "turn"
            and p["requested"]["ias_mps"] == speed_mps
            and p["requested"]["bank_deg"] == bank_deg
        ),
        None,
    )
    control_smoothness = {
        key: max(
            pstdev([sample[key] for sample in window]) for window in windows
        )
        for key in ("aileron_cmd_norm", "rudder_cmd_norm", "elevator_cmd_norm")
    }
    propulsion = {
        "engines": [
            {
                "index": i,
                "throttle_norm": fmean(
                    r["measurement"][f"engine_{i}_throttle_pos_norm"]["mean"]
                    for r in records
                ),
                "propeller_rpm": fmean(
                    r["measurement"][f"engine_{i}_propeller_rpm"]["mean"]
                    for r in records
                ),
                "power_hp": fmean(
                    r["measurement"][f"engine_{i}_power_hp"]["mean"]
                    for r in records
                ),
                "thrust_lbs": fmean(
                    r["measurement"][f"engine_{i}_thrust_lbs"]["mean"]
                    for r in records
                ),
            }
            for i in range(run_config["fixture"]["engine_count"])
        ]
    }
    analysis = {
        "aircraft": model,
        "altitude_m": altitude_m,
        "ias_target_mps": speed_mps,
        "bank_target_deg": bank_deg,
        "vertical_speed_target_mps": vz_mps,
        "raw_strict_status": point["status"],
        "usability": usability,
        "usability_cautions": cautions,
        "actual_stable_response": actual_stable,
        "actual": measured,
        "engine": engine,
        "propulsion": propulsion,
        "stability": stability,
        "settling": settling,
        "repeatability": point["repeatability"],
        "controller_saturation_in_measurement": saturation,
        "controller_clamp_fraction_in_measurement": {
            key: max(r["controller"]["clamp_fraction_in_measurement"][key] for r in records)
            for key in saturation
        },
        "control_command_std_max": control_smoothness,
        "turn_rate_deg_s": measured["turn_rate_deg_s"],
        "measured_turn_radius_m": measured["measured_radius_m"],
        "level_turn_radius_m": None if level_point is None else level_point["measured"]["measured_radius_m"],
        "combined_to_level_radius_ratio": (
            None if level_point is None
            else measured["measured_radius_m"] / level_point["measured"]["measured_radius_m"]
        ),
        "run_ids": point["run_ids"],
    }
    return records, point, analysis


def _straight_class(row: dict[str, Any], caution: float) -> str:
    if not row["actual_stable_response"]:
        return "UNUSABLE"
    throttle = row["actual"].get("engine_0_throttle_pos_norm", 0.0)
    if (
        row["strict_status"] != "VALID"
        or any(row["controller_saturation_in_measurement"].values())
        or throttle >= caution
    ):
        return "MARGINAL"
    return "USABLE"


def run_validation() -> dict[str, Any]:
    started = time.perf_counter()
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    u5_config = yaml.safe_load(u5.CONFIG_PATH.read_text(encoding="utf-8"))
    u3_config = yaml.safe_load(u5.U3_CONFIG_PATH.read_text(encoding="utf-8"))
    source_paths = {name: HERE / relative for name, relative in config["source_artifacts"].items()}
    source_before = {name: _sha256(path) for name, path in source_paths.items()}
    u5_points = _load_json(source_paths["u5_points"])
    u5_runs = _load_json(source_paths["u5_runs"])
    u5_phase1 = _load_json(source_paths["u5_phase1"])
    u5_high = _load_json(source_paths["u5_high_altitude"])
    u5_comparison = _load_json(source_paths["u5_comparison"])
    u5_result = _load_json(source_paths["u5_result"])
    u5_1_result = _load_json(source_paths["u5_1_result"])
    u5_runs_by_id = {row["run_id"]: row for row in u5_runs["runs"]}
    if u5_result["provenance_id"] != config["parent_u5_provenance_id"]:
        raise RuntimeError("U5 provenance mismatch")
    if u5_1_result["provenance_id"] != config["parent_u5_1_provenance_id"]:
        raise RuntimeError("U5.1 provenance mismatch")
    if len(u5_points["points"]) != config["reuse_contract"]["existing_u5_points_expected"]:
        raise RuntimeError("unexpected U5 point count")
    if len(u5_runs["runs"]) != config["reuse_contract"]["existing_u5_runs_expected"]:
        raise RuntimeError("unexpected U5 run count")
    seed = {
        "configuration": _sha256(CONFIG_PATH),
        "harness": _sha256(HARNESS_PATH),
        "sources": source_before,
    }
    provenance_id = "u5.2-" + hashlib.sha256(
        json.dumps(seed, sort_keys=True).encode("utf-8")
    ).hexdigest()[:20]
    existing_keys = {
        (
            p["aircraft_model"], p["altitude_msl_m"], p["requested"]["ias_mps"],
            p["maneuver"], p["requested"]["bank_deg"], p["requested"]["vertical_speed_mps"],
        )
        for p in u5_points["points"]
    }
    all_runs: list[dict[str, Any]] = []
    all_points: list[dict[str, Any]] = []
    straight_rows: list[dict[str, Any]] = []
    new_level_rows: list[dict[str, Any]] = []
    combined_rows: list[dict[str, Any]] = []
    u5._install_hooks()
    try:
        for model, finalist in config["finalists"].items():
            candidate = u5_config["candidates"][model]
            speed = float(finalist["nominal_ias_mps"])
            for altitude in config["new_straight_points"][model]["altitudes_m"]:
                key = (model, float(altitude), speed, "straight", 0.0, 0.0)
                if key in existing_keys:
                    raise RuntimeError(f"duplicate existing point requested: {key}")
                records, point, analysis = _run_existing_maneuver_adapter(
                    model, candidate, float(altitude), speed, "straight", 0.0,
                    u5_config, u3_config, provenance_id,
                )
                all_runs.extend(records); all_points.append(point); straight_rows.append(analysis)
            for altitude in config["new_level_turn_points_for_direct_radius_comparison"]["altitudes_m"]:
                for bank in config["new_level_turn_points_for_direct_radius_comparison"]["bank_targets_deg"]:
                    key = (model, float(altitude), speed, "turn", float(bank), 0.0)
                    if key in existing_keys:
                        raise RuntimeError(f"duplicate existing point requested: {key}")
                    records, point, analysis = _run_existing_maneuver_adapter(
                        model, candidate, float(altitude), speed, "turn", float(bank),
                        u5_config, u3_config, provenance_id,
                    )
                    all_runs.extend(records); all_points.append(point); new_level_rows.append(analysis)
                    existing_keys.add(key)
            existing_and_new_level = u5_points["points"] + all_points
            for region, altitude in config["combined_3d_validation"]["representative_regions"].items():
                for bank in config["combined_3d_validation"]["bank_targets_deg"]:
                    for vz in config["combined_3d_validation"]["vertical_speed_targets_mps"]:
                        records, point, analysis = _run_combined_point(
                            model, candidate, float(altitude), speed, float(bank), float(vz),
                            u5_config, u3_config, config, provenance_id,
                            existing_and_new_level,
                        )
                        analysis["representative_region"] = region
                        all_runs.extend(records); all_points.append(point); combined_rows.append(analysis)
    finally:
        u5._restore_hooks()

    if len(all_points) != 46 or len(all_runs) != 138:
        raise RuntimeError(f"unexpected new execution count: {len(all_points)} / {len(all_runs)}")
    if len({r["run_id"] for r in all_runs}) != len(all_runs):
        raise RuntimeError("new run IDs are not unique")

    merged_straight = [
        row for row in u5_phase1["rows"] + u5_high["rows"]
        if row["model"] in config["finalists"] and row["maneuver"] == "straight"
    ] + straight_rows
    caution = config["actual_stable_combined_contract"]["throttle_caution_threshold_norm"]
    maps = []
    for model, finalist in config["finalists"].items():
        matrix = []
        for altitude in config["altitude_anchors_m"]:
            cells = []
            for speed in finalist["reference_speed_probes_mps"]:
                row = next(
                    (x for x in merged_straight if x["model"] == model
                     and x["altitude_m"] == float(altitude)
                     and x["target_ias_mps"] == float(speed)),
                    None,
                )
                cells.append(
                    {
                        "ias_mps": float(speed),
                        "usability": "NOT_TESTED" if row is None else _straight_class(row, caution),
                        "raw_strict_status": None if row is None else row["strict_status"],
                        "actual_stable_response": None if row is None else row["actual_stable_response"],
                        "actual": None if row is None else row["actual"],
                        "engine": None if row is None else row["engine"],
                        "stability": None if row is None else row["stability"],
                        "settling": None if row is None else row["settling"],
                        "repeatability": None if row is None else row["repeatability"],
                        "controller_saturation_in_measurement": None if row is None else row["controller_saturation_in_measurement"],
                        "source": None if row is None else ("U5.2_NEW" if row in straight_rows else "U5_REUSED"),
                    }
                )
            matrix.append({"altitude_m": float(altitude), "cells": cells})
        maps.append({"aircraft": model, "nominal_ias_mps": float(finalist["nominal_ias_mps"]), "matrix": matrix})

    reused_finalist_points = [p for p in u5_points["points"] if p["aircraft_model"] in config["finalists"]]
    reuse_inventory = {
        "schema_version": 1,
        "artifact_type": "U5_2_EXISTING_DATA_REUSE",
        "provenance_id": provenance_id,
        "u5_total_points_available": len(u5_points["points"]),
        "u5_total_runs_available": len(u5_runs["runs"]),
        "finalist_points_reused": len(reused_finalist_points),
        "finalist_runs_reused": sum(len(p["run_ids"]) for p in reused_finalist_points),
        "reused_maneuver_types": sorted({p["maneuver"] for p in reused_finalist_points}),
        "duplicate_existing_points_executed": 0,
    }
    combined_artifact = {
        "schema_version": 1,
        "artifact_type": "U5_2_REPRESENTATIVE_COMBINED_3D_VALIDATION",
        "provenance_id": provenance_id,
        "target_subset": {"bank_deg": [-20, 20], "vertical_speed_mps": [-2.5, 2.5]},
        "rows": combined_rows,
    }
    rank = {r["model"]: r for r in u5_comparison["ranking"]}
    comparison_rows = []
    for model, finalist in config["finalists"].items():
        model_combined = [r for r in combined_rows if r["aircraft"] == model]
        climb = [r for r in model_combined if r["vertical_speed_target_mps"] > 0]
        descent = [r for r in model_combined if r["vertical_speed_target_mps"] < 0]
        high_straight = [
            row for row in merged_straight
            if row["model"] == model and row["target_ias_mps"] == float(finalist["nominal_ias_mps"])
            and row["altitude_m"] >= 5000.0
        ]
        high_altitude_propulsion = {}
        for straight_row in high_straight:
            source_point = next(
                p for p in u5_points["points"]
                if p["aircraft_model"] == model
                and p["altitude_msl_m"] == straight_row["altitude_m"]
                and p["maneuver"] == "straight"
                and p["requested"]["ias_mps"] == float(finalist["nominal_ias_mps"])
            )
            source_records = [u5_runs_by_id[run_id] for run_id in source_point["run_ids"]]
            high_altitude_propulsion[str(int(straight_row["altitude_m"]))] = {
                "engines": [
                    {
                        "index": i,
                        "throttle_norm": fmean(r["measurement"][f"engine_{i}_throttle_pos_norm"]["mean"] for r in source_records),
                        "propeller_rpm": fmean(r["measurement"][f"engine_{i}_propeller_rpm"]["mean"] for r in source_records),
                        "power_hp": fmean(r["measurement"][f"engine_{i}_power_hp"]["mean"] for r in source_records),
                    }
                    for i in range(1 if model == "c172p" else 2)
                ]
            }
        combined_all_direction_actual_stable = [
            altitude for altitude in config["combined_3d_validation"]["representative_regions"].values()
            if all(
                r["actual_stable_response"]
                for r in model_combined if r["altitude_m"] == float(altitude)
            )
        ]
        combined_all_direction_strict_usable = [
            altitude for altitude in config["combined_3d_validation"]["representative_regions"].values()
            if all(
                r["usability"] == "USABLE"
                for r in model_combined if r["altitude_m"] == float(altitude)
            )
        ]
        comparison_rows.append(
            {
                "aircraft": model,
                "tested_altitude_coverage_m": config["altitude_anchors_m"],
                "reference_speed_evidence_mps": finalist["reference_speed_probes_mps"],
                "nominal_speed_mps": float(finalist["nominal_ias_mps"]),
                "nominal_straight_all_altitude_anchors_actual_stable": all(
                    any(row["model"] == model and row["altitude_m"] == float(alt)
                        and row["target_ias_mps"] == float(finalist["nominal_ias_mps"])
                        and row["actual_stable_response"] for row in merged_straight)
                    for alt in config["altitude_anchors_m"]
                ),
                "u5_usable_altitude_region_m": [x["altitude_m"] for x in rank[model]["high_altitude"] if x["usable"]],
                "combined_climbing_turn_actual_stable_count": sum(r["actual_stable_response"] for r in climb),
                "combined_climbing_turn_test_count": len(climb),
                "combined_descending_turn_actual_stable_count": sum(r["actual_stable_response"] for r in descent),
                "combined_descending_turn_test_count": len(descent),
                "combined_usable_count": sum(r["usability"] == "USABLE" for r in model_combined),
                "combined_marginal_count": sum(r["usability"] == "MARGINAL" for r in model_combined),
                "combined_unusable_count": sum(r["usability"] == "UNUSABLE" for r in model_combined),
                "combined_all_direction_actual_stable_altitudes_m": combined_all_direction_actual_stable,
                "combined_all_direction_strict_usable_altitudes_m": combined_all_direction_strict_usable,
                "high_altitude_straight_throttle_norm": {str(int(r["altitude_m"])): r["actual"]["engine_0_throttle_pos_norm"] for r in high_straight},
                "high_altitude_straight_propulsion": high_altitude_propulsion,
                "controller_stability_score_u5": rank[model]["scores"]["control_quality_and_stability"],
                "repeatability_score_u5": rank[model]["scores"]["repeatability"],
                "minimum_tuning_score_u5": rank[model]["scores"]["minimum_tuning_burden"],
                "uav_like_character_score_u5": rank[model]["scores"]["uav_like_speed_and_behavior"],
                "model_complexity": "single_engine_piston_low" if model == "c172p" else "twin_engine_turboprop_higher",
            }
        )
    final_comparison = {
        "schema_version": 1,
        "artifact_type": "U5_2_FINALIST_COMPARISON",
        "provenance_id": provenance_id,
        "selection_priority_order": config["selection_philosophy"]["priority_order"],
        "rows": comparison_rows,
        "primary_aircraft": "c172p",
        "backup_aircraft": "DHC6",
        "nominal_ias_context_mps": 40.0,
        "tested_separate_maneuver_usable_altitude_region_m": [5000.0, 5500.0],
        "combined_3d_planner_safe_altitude_region_m": [],
        "combined_3d_region_note": "raw representative screen only; neither finalist validated bidirectional climbing turns at 5500/6000 m",
        "why_primary_won": [
            "equal_repeatability_and_controller_stability",
            "nominal_speed_is_more_uav_like",
            "smaller_turn_geometry",
            "larger_high_altitude_power_margin",
            "lower_single_engine_model_complexity",
            "neither_finalist_provides_a_validated_high_altitude_bidirectional_combined_climb_advantage",
            "c172p_has_complete_actual_stable_combined_responses_at_the_low_representative_region",
        ],
        "aircraft_selection_finally_closed": True,
    }
    source_after = {name: _sha256(path) for name, path in source_paths.items()}
    provenance = {
        "schema_version": 1,
        "artifact_type": "U5_2_PROVENANCE",
        "provenance_id": provenance_id,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "python_version": sys.version,
        "platform": platform.platform(),
        "configuration": {"path": str(CONFIG_PATH), "sha256": _sha256(CONFIG_PATH)},
        "harness": {"path": str(HARNESS_PATH), "sha256": _sha256(HARNESS_PATH)},
        "source_hashes_before": source_before,
        "source_hashes_after": source_after,
        "source_artifacts_unchanged": source_before == source_after,
    }
    result = {
        "step": "U5.2",
        "step_status": "PASS",
        "provenance_id": provenance_id,
        "existing_finalist_points_reused": len(reused_finalist_points),
        "existing_finalist_runs_reused": sum(len(p["run_ids"]) for p in reused_finalist_points),
        "new_points_executed": len(all_points),
        "new_cold_start_runs_executed": len(all_runs),
        "new_straight_points": len(straight_rows),
        "new_level_turn_points": len(new_level_rows),
        "new_combined_3d_points": len(combined_rows),
        "primary_aircraft": "c172p",
        "backup_aircraft": "DHC6",
        "nominal_ias_context_mps": 40.0,
        "tested_separate_maneuver_usable_altitude_region_m": [5000.0, 5500.0],
        "combined_3d_planner_safe_altitude_region_m": [],
        "aircraft_selection_finally_closed": True,
        "ready_for_selected_aircraft_lut": True,
        "full_u5_rerun_performed": False,
        "new_aircraft_added": False,
        "full_lut_generated": False,
        "controller_tuning_performed": False,
        "aircraft_xml_modified": False,
        "planner_or_search_modified": False,
        "next_stage_started": False,
        "source_artifacts_unchanged": source_before == source_after,
        "runtime_s": time.perf_counter() - started,
    }
    for name, payload in (
        ("u5_2_reuse_inventory.json", reuse_inventory),
        ("u5_2_new_runs.json", {"provenance_id": provenance_id, "runs": all_runs}),
        ("u5_2_new_points.json", {"provenance_id": provenance_id, "points": all_points}),
        ("u5_2_new_straight_coverage.json", {"provenance_id": provenance_id, "rows": straight_rows}),
        ("u5_2_new_level_turn_coverage.json", {"provenance_id": provenance_id, "rows": new_level_rows}),
        ("u5_2_altitude_speed_maps.json", {"provenance_id": provenance_id, "aircraft": maps}),
        ("u5_2_combined_3d_validation.json", combined_artifact),
        ("u5_2_final_comparison.json", final_comparison),
        ("u5_2_provenance.json", provenance),
        ("u5_2_result.json", result),
    ):
        _write(name, payload)
    return result


if __name__ == "__main__":
    print(json.dumps(run_validation(), indent=2, allow_nan=False))
