"""Sparse, stability-first stock-aircraft re-screen and final freeze."""

from __future__ import annotations

import copy
import hashlib
import importlib.metadata
import json
import math
import platform
import re
import subprocess
import sys
import time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from statistics import fmean, pstdev
from typing import Any

import jsbsim
import yaml

import production_mixture_policy as pressure_policy
import u3_aircraft_selection as u3


HERE = Path(__file__).resolve().parent
RESULTS_DIR = HERE / "results"
CONFIG_PATH = HERE / "u5_practical_aircraft_rescreen_configuration.yaml"
U1_CONFIG_PATH = HERE / "u1_profile_validation_configuration.yaml"
U1_HARNESS_PATH = HERE / "u1_profile_validation.py"
U2_CONFIG_PATH = HERE / "u2_candidate_screening_configuration.yaml"
U2_HARNESS_PATH = HERE / "u2_candidate_screening.py"
U3_CONFIG_PATH = HERE / "u3_aircraft_selection_configuration.yaml"
U3_HARNESS_PATH = HERE / "u3_aircraft_selection.py"
POLICY_PATH = HERE / "production_mixture_policy.py"

u1 = u3.u2.u1
_BASE_SNAPSHOT = u1._snapshot
_BASE_SET_RUNNING = u1._set_all_engines_running
_BASE_SET_THROTTLE = u1._set_symmetric_throttle
_CAPTURE: list[dict[str, float]] | None = None
_ACTIVE_MIXTURE_MODE = ""


def _load_yaml(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write(name: str, value: Any) -> None:
    def json_safe(item: Any) -> Any:
        if isinstance(item, float) and not math.isfinite(item):
            return None
        if isinstance(item, dict):
            return {key: json_safe(child) for key, child in item.items()}
        if isinstance(item, list):
            return [json_safe(child) for child in item]
        if isinstance(item, tuple):
            return [json_safe(child) for child in item]
        return item

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    (RESULTS_DIR / name).write_text(
        json.dumps(json_safe(value), indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _git_commit() -> str | None:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=HERE.parent, text=True
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def _pressure_ratio_applies() -> bool:
    return _ACTIVE_MIXTURE_MODE in {
        "production_pressure_ratio_v1",
        "pressure_ratio_screening",
    }


def _apply_pressure_ratio(fdm: Any, config: dict[str, Any]) -> None:
    if not _pressure_ratio_applies():
        return
    command = pressure_policy.command_for_fdm(fdm)
    for index in range(int(config["fixture"]["engine_count"])):
        suffix = u1._suffix(index)
        fdm[f"fcs/mixture-cmd-norm{suffix}"] = command
        fdm[f"fcs/mixture-pos-norm{suffix}"] = command


def _set_running_with_mixture(fdm: Any, config: dict[str, Any]) -> None:
    _BASE_SET_RUNNING(fdm, config)
    _apply_pressure_ratio(fdm, config)


def _set_throttle_with_mixture(
    fdm: Any, config: dict[str, Any], command: float
) -> None:
    _BASE_SET_THROTTLE(fdm, config, command)
    _apply_pressure_ratio(fdm, config)


def _screen_snapshot(
    fdm: Any, elapsed_s: float, config: dict[str, Any]
) -> dict[str, float]:
    sample = _BASE_SNAPSHOT(fdm, elapsed_s, config)
    sample["ambient_pressure_psf"] = float(fdm["atmosphere/P-psf"])
    for index in range(int(config["fixture"]["engine_count"])):
        suffix = u1._suffix(index)
        label = f"engine_{index}"
        sample[f"{label}_mixture_cmd_norm"] = float(
            fdm[f"fcs/mixture-cmd-norm{suffix}"]
        )
        sample[f"{label}_mixture_pos_norm"] = float(
            fdm[f"fcs/mixture-pos-norm{suffix}"]
        )
        sample[f"{label}_power_hp"] = float(
            fdm[f"propulsion/engine{suffix}/power-hp"]
        )
        sample[f"{label}_fuel_flow_gph"] = float(
            fdm[f"propulsion/engine{suffix}/fuel-flow-rate-gph"]
        )
    if _CAPTURE is not None:
        _CAPTURE.append(copy.deepcopy(sample))
    return sample


def _install_hooks() -> None:
    u1._snapshot = _screen_snapshot
    u1._set_all_engines_running = _set_running_with_mixture
    u1._set_symmetric_throttle = _set_throttle_with_mixture


def _restore_hooks() -> None:
    u1._snapshot = _BASE_SNAPSHOT
    u1._set_all_engines_running = _BASE_SET_RUNNING
    u1._set_symmetric_throttle = _BASE_SET_THROTTLE


def _split_repeats(
    samples: list[dict[str, float]], repeats: int
) -> list[list[dict[str, float]]]:
    groups: list[list[dict[str, float]]] = []
    for sample in samples:
        if not groups or sample["elapsed_s"] <= groups[-1][-1]["elapsed_s"]:
            groups.append([])
        groups[-1].append(sample)
    if len(groups) != repeats:
        raise RuntimeError(f"captured {len(groups)} repeats; expected {repeats}")
    return groups


def _run_config(
    model: str,
    candidate: dict[str, Any],
    altitude_m: float,
    config: dict[str, Any],
    u3_config: dict[str, Any],
) -> dict[str, Any]:
    candidate_adapter = {"fixture_source": candidate["fixture_source"]}
    run_config = u3._run_config(model, candidate_adapter, u3_config)
    run_config["configuration_id"] = config["configuration_id"]
    run_config["run_prefix"] = "u5"
    run_config["simulation"] = copy.deepcopy(config["simulation"])
    run_config["fixture"]["representative_altitude_msl_m"] = float(altitude_m)
    return run_config


def _rename_ids(records: list[dict[str, Any]], point: dict[str, Any]) -> None:
    mapping = {}
    altitude_token = f"h{float(point['altitude_msl_m']):g}m"
    for record in records:
        old = record["run_id"]
        new = old.replace("u1-", f"u5-{altitude_token}-", 1)
        record["run_id"] = new
        mapping[old] = new
    point["run_ids"] = [mapping[value] for value in point["run_ids"]]


def _stats(values: list[float]) -> dict[str, Any]:
    if not values or not all(math.isfinite(value) for value in values):
        return {
            "finite": False,
            "mean": None,
            "min": None,
            "max": None,
            "std": None,
            "peak_to_peak": None,
        }
    return {
        "finite": True,
        "mean": fmean(values),
        "min": min(values),
        "max": max(values),
        "std": pstdev(values),
        "peak_to_peak": max(values) - min(values),
    }


def _settling(
    group: list[dict[str, float]],
    maneuver: str,
    target_speed: float,
    target_value: float,
    contract: dict[str, Any],
) -> dict[str, Any]:
    target_bank = target_value if maneuver == "turn" else 0.0
    target_vz = target_value if maneuver == "vertical" else 0.0
    vz_tolerance = (
        contract["straight_vertical_speed_abs_max_mps"]
        if maneuver == "straight"
        else max(
            contract["vertical_mean_error_max_mps"],
            0.25 * abs(target_vz),
        )
    )

    def within(sample: dict[str, float]) -> bool:
        ias_ok = (
            abs(sample["ias_mps"] - target_speed) / target_speed
            <= contract["ias_mean_relative_error_max"]
            if maneuver == "straight"
            else sample["ias_mps"] / target_speed >= contract["ias_retention_min"]
        )
        return (
            ias_ok
            and abs(sample["roll_deg"] - target_bank)
            <= contract["bank_mean_error_max_deg"]
            and abs(sample["vertical_speed_mps"] - target_vz) <= vz_tolerance
        )

    after_ramp = [sample for sample in group if sample["elapsed_s"] >= 8.0]
    last_bad = max(
        (sample["elapsed_s"] for sample in after_ramp if not within(sample)),
        default=None,
    )
    settled = bool(after_ramp and within(after_ramp[-1]))
    settling_time = (8.0 if last_bad is None else last_bad + 0.01) if settled else None
    return {
        "settling_time_s": settling_time,
        "screening_settling_deadline_s": contract["settling_deadline_s"],
        "settled_by_screening_deadline": settling_time is not None
        and settling_time <= contract["settling_deadline_s"],
    }


def _engine_summary(
    records: list[dict[str, Any]], engine_count: int
) -> dict[str, Any]:
    engines = []
    for index in range(engine_count):
        label = f"engine_{index}"

        def combined(field: str) -> dict[str, Any]:
            return {
                "mean": fmean(
                    float(record["measurement"][field]["mean"])
                    for record in records
                ),
                "min": min(
                    float(record["measurement"][field]["min"])
                    for record in records
                ),
                "max": max(
                    float(record["measurement"][field]["max"])
                    for record in records
                ),
                "per_repeat_mean": [
                    float(record["measurement"][field]["mean"])
                    for record in records
                ],
            }

        engines.append(
            {
                "index": index,
                "state": combined(f"{label}_state"),
                "thrust_lbs": combined(f"{label}_thrust_lbs"),
                "power_hp": combined(f"{label}_power_hp"),
                "fuel_flow_gph": combined(f"{label}_fuel_flow_gph"),
                "throttle_position_norm": combined(
                    f"{label}_throttle_pos_norm"
                ),
                "mixture_command_norm": combined(
                    f"{label}_mixture_cmd_norm"
                ),
                "mixture_position_norm": combined(
                    f"{label}_mixture_pos_norm"
                ),
            }
        )
    healthy = all(
        engine["state"]["min"] > 0.0
        and engine["thrust_lbs"]["mean"] > 0.0
        and engine["power_hp"]["mean"] > 0.0
        for engine in engines
    )
    return {"healthy": healthy, "engines": engines}


def _analyze_point(
    model: str,
    altitude_m: float,
    speed_mps: float,
    maneuver: str,
    target_value: float,
    records: list[dict[str, Any]],
    point: dict[str, Any],
    groups: list[list[dict[str, float]]],
    run_config: dict[str, Any],
    config: dict[str, Any],
) -> dict[str, Any]:
    contract = config["actual_stable_response_contract"]
    measured = point.get("measured")
    engine = _engine_summary(records, int(run_config["fixture"]["engine_count"]))
    windows = [
        [sample for sample in group if sample["elapsed_s"] >= 20.0 - 1.0e-9]
        for group in groups
    ]
    stability = {
        "ias_mps": [_stats([sample["ias_mps"] for sample in window]) for window in windows],
        "bank_deg": [_stats([sample["roll_deg"] for sample in window]) for window in windows],
        "vertical_speed_mps": [
            _stats([sample["vertical_speed_mps"] for sample in window])
            for window in windows
        ],
    }
    settling = [
        _settling(group, maneuver, speed_mps, target_value, contract)
        for group in groups
    ]
    saturation = {
        key: any(
            record.get("controller", {})
            .get("saturation_in_measurement", {})
            .get(key, False)
            for record in records
        )
        for key in ("aileron", "rudder", "elevator", "throttle")
    }
    finite = measured is not None and all(
        not isinstance(value, float) or math.isfinite(value)
        for value in measured.values()
    )
    stable_window = (
        all(
            item["finite"]
            for field in stability.values()
            for item in field
        )
        and max(item["std"] for item in stability["ias_mps"])
        <= contract["measurement_ias_std_max_mps"]
        and max(item["std"] for item in stability["bank_deg"])
        <= contract["measurement_bank_std_max_deg"]
        and max(item["std"] for item in stability["vertical_speed_mps"])
        <= contract["measurement_vertical_speed_std_max_mps"]
    )
    repeatable = bool(point.get("repeatability", {}).get("passed", False))
    settled = all(item["settled_by_screening_deadline"] for item in settling)
    base = (
        finite
        and engine["healthy"]
        and repeatable
        and stable_window
        and settled
        and measured["stall_indicator"] <= 0.0
        and measured["maximum_surface_usage_norm"]
        < contract["normalized_surface_usage_max"]
        and measured["ias_mps"] / speed_mps >= contract["ias_retention_min"]
    )
    if maneuver == "straight":
        response = base and (
            abs(measured["ias_mps"] - speed_mps) / speed_mps
            <= contract["ias_mean_relative_error_max"]
            and abs(measured["vertical_speed_mps"])
            <= contract["straight_vertical_speed_abs_max_mps"]
            and abs(measured["roll_deg"]) <= contract["bank_mean_error_max_deg"]
        )
    elif maneuver == "turn":
        response = base and (
            math.copysign(1.0, measured["roll_deg"])
            == math.copysign(1.0, target_value)
            and abs(measured["roll_deg"])
            >= contract["minimum_meaningful_turn_bank_deg"]
            and abs(measured["roll_deg"] - target_value)
            <= contract["bank_mean_error_max_deg"]
        )
    else:
        tolerance = max(
            contract["vertical_mean_error_max_mps"], 0.25 * abs(target_value)
        )
        response = base and (
            math.copysign(1.0, measured["vertical_speed_mps"])
            == math.copysign(1.0, target_value)
            and abs(measured["vertical_speed_mps"])
            >= contract["minimum_meaningful_vertical_response_mps"]
            and abs(measured["vertical_speed_mps"] - target_value) <= tolerance
        )
    failure_reasons = sorted(
        {reason for record in records for reason in record["status_reasons"]}
    )
    return {
        "model": model,
        "altitude_m": altitude_m,
        "target_ias_mps": speed_mps,
        "maneuver": maneuver,
        "target_value": target_value,
        "strict_status": point["status"],
        "strict_failure_reasons": failure_reasons,
        "finite_outputs": finite,
        "actual_stable_response": response,
        "actual": measured,
        "engine": engine,
        "mixture_mode": _ACTIVE_MIXTURE_MODE,
        "controller_saturation_in_measurement": saturation,
        "stability": stability,
        "settling": settling,
        "repeatability": point.get("repeatability"),
        "run_ids": point["run_ids"],
    }


def _run_point(
    model: str,
    candidate: dict[str, Any],
    altitude_m: float,
    speed_mps: float,
    maneuver: str,
    target_value: float,
    config: dict[str, Any],
    u3_config: dict[str, Any],
    provenance_id: str,
) -> tuple[list[dict[str, Any]], dict[str, Any], dict[str, Any]]:
    global _CAPTURE, _ACTIVE_MIXTURE_MODE
    run_config = _run_config(
        model, candidate, altitude_m, config, u3_config
    )
    _ACTIVE_MIXTURE_MODE = candidate["mixture_mode"]
    _CAPTURE = []
    try:
        records, point = u1._run_point(
            model, maneuver, speed_mps, target_value, run_config, provenance_id
        )
        groups = _split_repeats(
            _CAPTURE, int(config["simulation"]["cold_start_repetitions"])
        )
    finally:
        _CAPTURE = None
    point["aircraft_model"] = model
    point["altitude_msl_m"] = altitude_m
    point["provenance_id"] = provenance_id
    _rename_ids(records, point)
    analysis = _analyze_point(
        model,
        altitude_m,
        speed_mps,
        maneuver,
        target_value,
        records,
        point,
        groups,
        run_config,
        config,
    )
    return records, point, analysis


def _speed_choice(
    model: str, rows: list[dict[str, Any]], config: dict[str, Any]
) -> dict[str, Any]:
    altitudes = [float(value) for value in config["phase1"]["altitudes_m"]]
    candidates = []
    for speed in sorted({row["target_ias_mps"] for row in rows}):
        subset = [row for row in rows if row["target_ias_mps"] == speed]
        finite_subset = [
            row for row in subset
            if row["finite_outputs"] and row["actual"] is not None
        ]
        stable_count = sum(row["actual_stable_response"] for row in subset)
        saturation_count = sum(
            any(row["controller_saturation_in_measurement"].values())
            for row in subset
        )
        mean_ias_error = (
            fmean(abs(row["actual"]["ias_mps"] - speed) / speed for row in finite_subset)
            if finite_subset else 1.0e9
        )
        throttle_values = [
            engine["throttle_position_norm"]["mean"]
            for row in finite_subset
            for engine in row["engine"]["engines"]
            if math.isfinite(engine["throttle_position_norm"]["mean"])
        ]
        mean_throttle = fmean(throttle_values) if throttle_values else 1.0e9
        surface_values = [
            row["actual"]["maximum_surface_usage_norm"] for row in finite_subset
            if math.isfinite(row["actual"]["maximum_surface_usage_norm"])
        ]
        max_surface = max(surface_values) if surface_values else 1.0e9
        mentor_distance = max(35.0 - speed, 0.0, speed - 50.0)
        candidates.append(
            {
                "speed_mps": speed,
                "stable_altitude_count": stable_count,
                "stable_at_both_reference_altitudes": stable_count
                == len(altitudes),
                "saturation_point_count": saturation_count,
                "mean_relative_ias_error": mean_ias_error,
                "mean_throttle_norm": mean_throttle,
                "maximum_surface_usage_norm": max_surface,
                "mentor_band_distance_mps": mentor_distance,
            }
        )
    ordered = sorted(
        candidates,
        key=lambda row: (
            -row["stable_altitude_count"],
            row["saturation_point_count"],
            row["maximum_surface_usage_norm"],
            abs(row["mean_throttle_norm"] - 0.65),
            row["mean_relative_ias_error"],
            row["mentor_band_distance_mps"],
            abs(row["speed_mps"] - 42.5),
        ),
    )
    selected = next(
        (row["speed_mps"] for row in ordered if row["stable_at_both_reference_altitudes"]),
        None,
    )
    return {
        "model": model,
        "speed_candidates": ordered,
        "selected_speed_mps": selected,
        "early_rejected": selected is None,
        "early_rejection_reason": (
            "no_actual_stable_speed_at_both_0_and_3000_m"
            if selected is None else None
        ),
    }


def _candidate_summary(
    model: str,
    speed_choice: dict[str, Any],
    phase1_rows: list[dict[str, Any]],
    phase2_rows: list[dict[str, Any]],
    config: dict[str, Any],
) -> dict[str, Any]:
    if speed_choice["early_rejected"]:
        return {
            "model": model,
            "selected_speed_mps": None,
            "early_rejected": True,
            "major_limitation": speed_choice["early_rejection_reason"],
            "scores": {
                name: 0 for name in (
                    "control_quality_and_stability", "repeatability",
                    "high_altitude_usability", "minimum_tuning_burden",
                    "reasonable_turn_climb_descent", "uav_like_speed_and_behavior",
                    "mentor_reference_proximity",
                )
            },
            "high_altitude": [],
        }
    speed = speed_choice["selected_speed_mps"]
    altitude_rows = []
    for altitude in (float(value) for value in config["phase2"]["altitudes_m"]):
        subset = [row for row in phase2_rows if row["altitude_m"] == altitude]
        straight = next(row for row in subset if row["maneuver"] == "straight")
        turns = [row for row in subset if row["maneuver"] == "turn"]
        vertical = [row for row in subset if row["maneuver"] == "vertical"]
        left = any(row["actual_stable_response"] and row["target_value"] < 0 for row in turns)
        right = any(row["actual_stable_response"] and row["target_value"] > 0 for row in turns)
        climb = any(row["actual_stable_response"] and row["target_value"] > 0 for row in vertical)
        descent = any(row["actual_stable_response"] and row["target_value"] < 0 for row in vertical)
        requirements = {
            "straight": straight["actual_stable_response"],
            "turn_left": left,
            "turn_right": right,
            "climb": climb,
            "descent": descent,
            "engine": straight["engine"]["healthy"],
        }
        usable = all(requirements.values())
        strict_counts = Counter(row["strict_status"] for row in subset)
        stable_vertical = [row for row in vertical if row["actual_stable_response"]]
        altitude_rows.append(
            {
                "altitude_m": altitude,
                "usable": usable,
                "requirements": requirements,
                "straight_strict_status": straight["strict_status"],
                "straight_actual_ias_mps": straight["actual"]["ias_mps"],
                "straight_throttle_norm": fmean(
                    engine["throttle_position_norm"]["mean"]
                    for engine in straight["engine"]["engines"]
                ),
                "strict_status_counts": dict(sorted(strict_counts.items())),
                "stable_turn_targets_deg": [
                    row["target_value"] for row in turns if row["actual_stable_response"]
                ],
                "stable_vertical_targets_mps": [
                    row["target_value"] for row in stable_vertical
                ],
                "actual_climb_responses_mps": [
                    row["actual"]["vertical_speed_mps"]
                    for row in vertical if row["target_value"] > 0
                ],
                "actual_descent_responses_mps": [
                    row["actual"]["vertical_speed_mps"]
                    for row in vertical if row["target_value"] < 0
                ],
                "saturation_point_count": sum(
                    any(row["controller_saturation_in_measurement"].values())
                    for row in subset
                ),
                "repeatable": all(
                    row["repeatability"]["passed"] for row in subset
                ),
            }
        )
    all_rows = [row for row in phase1_rows + phase2_rows if row["model"] == model]
    all_repeatable = all(row["repeatability"]["passed"] for row in all_rows)
    phase1_stable = all(
        row["actual_stable_response"]
        for row in phase1_rows
        if row["model"] == model and row["target_ias_mps"] == speed
    )
    straight_high = [
        row for row in phase2_rows
        if row["model"] == model and row["maneuver"] == "straight"
    ]
    calm_surfaces = not any(
        row["controller_saturation_in_measurement"][key]
        for row in all_rows
        for key in ("aileron", "rudder", "elevator")
    )
    control_score = (
        5 if phase1_stable and all(row["actual_stable_response"] for row in straight_high)
        and calm_surfaces and all_repeatable else
        4 if phase1_stable and all_repeatable else
        3 if phase1_stable else 1
    )
    usable_count = sum(row["usable"] for row in altitude_rows)
    straight_count = sum(row["requirements"]["straight"] for row in altitude_rows)
    high_score = 5 if usable_count == 3 else (4 if usable_count == 2 else (3 if usable_count == 1 else (2 if straight_count == 3 else 1)))
    maneuver_count = sum(
        row["requirements"][name]
        for row in altitude_rows
        for name in ("turn_left", "turn_right", "climb", "descent")
    )
    maneuver_score = 5 if maneuver_count >= 11 else (4 if maneuver_count >= 9 else (3 if maneuver_count >= 6 else (2 if maneuver_count >= 3 else 1)))
    tuning = {
        "c172r": 5, "c172p": 5, "c182": 5, "J3Cub": 5,
        "c172x": 4, "DHC6": 4, "pa28": 3, "c310": 3,
    }[model]
    uav_character = {
        "c172r": 5, "c172p": 5, "c172x": 5, "c182": 5,
        "J3Cub": 5, "pa28": 5, "DHC6": 3, "c310": 3,
    }[model]
    mentor = 5 if 35.0 <= speed <= 50.0 else (4 if 30.0 <= speed <= 60.0 else 3)
    highest = max((row["altitude_m"] for row in altitude_rows if row["usable"]), default=None)
    missing_at_highest = next(
        (
            [name for name, passed in row["requirements"].items() if not passed]
            for row in reversed(altitude_rows) if not row["usable"]
        ),
        [],
    )
    return {
        "model": model,
        "selected_speed_mps": speed,
        "early_rejected": False,
        "low_altitude_stability": phase1_stable,
        "high_altitude": altitude_rows,
        "highest_tested_usable_altitude_m": highest,
        "all_repeatable": all_repeatable,
        "gain_tuning_performed": False,
        "interface_or_startup_complexity": 5 - tuning,
        "major_limitation": (
            "none_in_sparse_screen" if len(missing_at_highest) == 0
            else "highest_candidate_missing_" + "_".join(missing_at_highest)
        ),
        "scores": {
            "control_quality_and_stability": control_score,
            "repeatability": 5 if all_repeatable else 1,
            "high_altitude_usability": high_score,
            "minimum_tuning_burden": tuning,
            "reasonable_turn_climb_descent": maneuver_score,
            "uav_like_speed_and_behavior": uav_character,
            "mentor_reference_proximity": mentor,
        },
    }


def _ranking_key(row: dict[str, Any], priority: list[str]) -> tuple[Any, ...]:
    scores = row["scores"]
    usable_count = sum(item["usable"] for item in row.get("high_altitude", []))
    return tuple(
        [-scores[name] for name in priority]
        + [
            -usable_count,
            -(row.get("highest_tested_usable_altitude_m") or -1.0),
            row.get("interface_or_startup_complexity", 99),
            row["model"].lower(),
        ]
    )


def run_u5() -> dict[str, Any]:
    started = time.perf_counter()
    generated_at = datetime.now(timezone.utc).isoformat()
    config = _load_yaml(CONFIG_PATH)
    u3_config = _load_yaml(U3_CONFIG_PATH)
    inventory = _load_json(RESULTS_DIR / "u2_inventory.json")
    inventory_models = {row["model"] for row in inventory}
    if missing := set(config["candidates"]) - inventory_models:
        raise RuntimeError(f"configured candidates absent from U2 inventory: {missing}")
    historical_paths = {
        name: HERE / item["path"]
        for name, item in config["historical_artifacts"].items()
    }
    historical_before = {name: _sha256(path) for name, path in historical_paths.items()}
    historical_expected = {
        name: item["sha256"] for name, item in config["historical_artifacts"].items()
    }
    if historical_before != historical_expected:
        raise RuntimeError("historical artifact hash mismatch before U5")
    root = Path(jsbsim.get_default_root_dir()).resolve()
    audits = {model: u3._audit(model, root) for model in config["candidates"]}
    files = {
        "configuration": {"path": str(CONFIG_PATH.resolve()), "sha256": _sha256(CONFIG_PATH)},
        "harness": {"path": str(Path(__file__).resolve()), "sha256": _sha256(Path(__file__).resolve())},
        "pressure_policy": {"path": str(POLICY_PATH.resolve()), "sha256": _sha256(POLICY_PATH)},
        "u1_configuration": {"path": str(U1_CONFIG_PATH.resolve()), "sha256": _sha256(U1_CONFIG_PATH)},
        "u1_harness": {"path": str(U1_HARNESS_PATH.resolve()), "sha256": _sha256(U1_HARNESS_PATH)},
        "u2_configuration": {"path": str(U2_CONFIG_PATH.resolve()), "sha256": _sha256(U2_CONFIG_PATH)},
        "u2_harness": {"path": str(U2_HARNESS_PATH.resolve()), "sha256": _sha256(U2_HARNESS_PATH)},
        "u3_configuration": {"path": str(U3_CONFIG_PATH.resolve()), "sha256": _sha256(U3_CONFIG_PATH)},
        "u3_harness": {"path": str(U3_HARNESS_PATH.resolve()), "sha256": _sha256(U3_HARNESS_PATH)},
    }
    seed = {
        "files": {name: item["sha256"] for name, item in files.items()},
        "historical": historical_before,
        "aircraft": {model: audit["main_xml_sha256"] for model, audit in audits.items()},
        "selection_philosophy": config["selection_philosophy"],
    }
    provenance_id = "u5-" + hashlib.sha256(
        json.dumps(seed, sort_keys=True).encode("utf-8")
    ).hexdigest()[:20]
    all_runs: list[dict[str, Any]] = []
    all_points: list[dict[str, Any]] = []
    phase1_rows: list[dict[str, Any]] = []
    speed_choices: list[dict[str, Any]] = []
    phase2_rows: list[dict[str, Any]] = []
    _install_hooks()
    try:
        for model, candidate in config["candidates"].items():
            model_rows = []
            for altitude_value in config["phase1"]["altitudes_m"]:
                for speed_value in config["phase1"]["initial_speed_probes_mps"]:
                    records, point, analysis = _run_point(
                        model, candidate, float(altitude_value), float(speed_value),
                        "straight", 0.0, config, u3_config, provenance_id,
                    )
                    all_runs.extend(records); all_points.append(point)
                    phase1_rows.append(analysis); model_rows.append(analysis)
            choice = _speed_choice(model, model_rows, config)
            if choice["selected_speed_mps"] is None:
                best = choice["speed_candidates"][0]
                extra = None
                if best["speed_mps"] == min(config["phase1"]["initial_speed_probes_mps"]):
                    extra = float(config["phase1"]["adaptive_extra_probe_low_mps"])
                elif best["speed_mps"] == max(config["phase1"]["initial_speed_probes_mps"]):
                    extra = float(config["phase1"]["adaptive_extra_probe_high_mps"])
                if extra is not None:
                    for altitude_value in config["phase1"]["altitudes_m"]:
                        records, point, analysis = _run_point(
                            model, candidate, float(altitude_value), extra,
                            "straight", 0.0, config, u3_config, provenance_id,
                        )
                        all_runs.extend(records); all_points.append(point)
                        phase1_rows.append(analysis); model_rows.append(analysis)
                    choice = _speed_choice(model, model_rows, config)
                    choice["adaptive_extra_probe_mps"] = extra
            speed_choices.append(choice)

        for choice in speed_choices:
            if choice["early_rejected"]:
                continue
            model = choice["model"]
            candidate = config["candidates"][model]
            speed = float(choice["selected_speed_mps"])
            for altitude_value in config["phase2"]["altitudes_m"]:
                altitude = float(altitude_value)
                records, point, straight = _run_point(
                    model, candidate, altitude, speed, "straight", 0.0,
                    config, u3_config, provenance_id,
                )
                all_runs.extend(records); all_points.append(point); phase2_rows.append(straight)
                if not straight["actual_stable_response"]:
                    continue
                for bank_value in config["phase2"]["turn_bank_targets_deg"]:
                    records, point, analysis = _run_point(
                        model, candidate, altitude, speed, "turn", float(bank_value),
                        config, u3_config, provenance_id,
                    )
                    all_runs.extend(records); all_points.append(point); phase2_rows.append(analysis)
                for vz_value in config["phase2"]["vertical_speed_targets_mps"]:
                    records, point, analysis = _run_point(
                        model, candidate, altitude, speed, "vertical", float(vz_value),
                        config, u3_config, provenance_id,
                    )
                    all_runs.extend(records); all_points.append(point); phase2_rows.append(analysis)
    finally:
        _restore_hooks()

    summaries = [
        _candidate_summary(
            model,
            next(row for row in speed_choices if row["model"] == model),
            phase1_rows,
            [row for row in phase2_rows if row["model"] == model],
            config,
        )
        for model in config["candidates"]
    ]
    priority = config["selection_philosophy"]["priority_order"]
    ranking = sorted(summaries, key=lambda row: _ranking_key(row, priority))
    viable = [row for row in ranking if not row["early_rejected"]]
    if len(viable) < 2:
        raise RuntimeError("U5 did not produce two viable final-freeze candidates")
    for rank, row in enumerate(ranking, 1):
        row["rank"] = rank
        row["selection"] = (
            "PRIMARY" if row is viable[0] else "BACKUP" if row is viable[1] else None
        )
    primary, backup = viable[0], viable[1]
    historical_after = {name: _sha256(path) for name, path in historical_paths.items()}
    if historical_after != historical_before:
        raise RuntimeError("historical artifacts changed during U5")
    freeze = {
        "schema_version": 1,
        "artifact_type": "FINAL_AIRCRAFT_SELECTION_FREEZE",
        "provenance_id": provenance_id,
        "primary_aircraft": primary["model"],
        "backup_aircraft": backup["model"],
        "selected_nominal_speed_policy": {
            "type": "fixed_nominal_ias_context",
            "nominal_ias_mps": primary["selected_speed_mps"],
            "speed_is_planner_state_dimension": False,
        },
        "primary_high_altitude_usable_region_m": [
            row["altitude_m"] for row in primary["high_altitude"] if row["usable"]
        ],
        "primary_major_limitation": primary["major_limitation"],
        "selected_stack": {
            "aircraft": primary["model"],
            "outer_loop": "unchanged_U1_U3_bounded_IAS_Vz_bank_beta_architecture",
            "stock_fcs": True,
            "mixture_mode": config["candidates"][primary["model"]]["mixture_mode"],
            "mixture_policy_id": config["candidates"][primary["model"]].get(
                "mixture_policy_id",
                "stock_aircraft_mixture_or_turbine_fuel_control",
            ),
            "mixture_formula": (
                pressure_policy.metadata()["formula"]
                if config["candidates"][primary["model"]]["mixture_mode"]
                == "production_pressure_ratio_v1"
                else "stock_aircraft_defined"
            ),
            "mixture_update_cadence": (
                "after engine start and before every simulation frame"
                if config["candidates"][primary["model"]]["mixture_mode"]
                == "production_pressure_ratio_v1"
                else "stock_aircraft_defined"
            ),
            "gain_tuning_performed": False,
        },
        "aircraft_selection_closed": True,
        "reopen_only_for_new_blocking_evidence": True,
        "same_stack_required_for_lut_and_final_replay": True,
    }
    phase1_artifact = {
        "schema_version": 1,
        "artifact_type": "U5_STABLE_SPEED_SCREEN",
        "provenance_id": provenance_id,
        "altitudes_m": config["phase1"]["altitudes_m"],
        "initial_speed_probes_mps": config["phase1"]["initial_speed_probes_mps"],
        "rows": phase1_rows,
        "speed_choices": speed_choices,
    }
    phase2_artifact = {
        "schema_version": 1,
        "artifact_type": "U5_HIGH_ALTITUDE_MISSION_SCREEN",
        "provenance_id": provenance_id,
        "altitudes_m": config["phase2"]["altitudes_m"],
        "turn_targets_deg": config["phase2"]["turn_bank_targets_deg"],
        "vertical_targets_mps": config["phase2"]["vertical_speed_targets_mps"],
        "rows": phase2_rows,
    }
    comparison = {
        "schema_version": 1,
        "artifact_type": "U5_PRACTICAL_AIRCRAFT_COMPARISON",
        "provenance_id": provenance_id,
        "selection_priority": priority,
        "actual_stable_response_contract": config["actual_stable_response_contract"],
        "ranking": ranking,
        "strict_status_semantics_preserved": True,
        "selection_is_not_boolean_valid_count": True,
        "freeze": freeze,
    }
    provenance = {
        "provenance_id": provenance_id,
        "generated_at_utc": generated_at,
        "repository_git_commit": _git_commit(),
        "jsbsim_python_package_version": importlib.metadata.version("jsbsim"),
        "jsbsim_compiled_version": jsbsim.FGFDMExec(None).get_version(),
        "jsbsim_git_commit": re.search(
            r"commit ([0-9a-f]{40})", jsbsim.FGFDMExec(None).get_version()
        ).group(1),
        "python_version": sys.version,
        "python_executable": sys.executable,
        "platform": platform.platform(),
        "files": files,
        "aircraft_audits": audits,
        "historical_artifact_hashes_before": historical_before,
        "historical_artifact_hashes_after": historical_after,
        "historical_artifacts_unchanged": historical_before == historical_after,
    }
    result = {
        "step": "U5",
        "step_status": "PASS",
        "provenance_id": provenance_id,
        "candidate_count": len(config["candidates"]),
        "tested_candidates": list(config["candidates"]),
        "early_rejected_candidates": [
            row["model"] for row in speed_choices if row["early_rejected"]
        ],
        "executed_points": len(all_points),
        "executed_cold_start_runs": len(all_runs),
        "primary_aircraft": primary["model"],
        "backup_aircraft": backup["model"],
        "selected_nominal_ias_mps": primary["selected_speed_mps"],
        "primary_high_altitude_usable_region_m": freeze[
            "primary_high_altitude_usable_region_m"
        ],
        "aircraft_selection_frozen": True,
        "ready_to_return_to_path_planner_work": True,
        "full_lut_generated": False,
        "controller_gain_tuning_performed": False,
        "stock_xml_modified": False,
        "planner_modified": False,
        "interpolation_performed": False,
        "derating_performed": False,
        "heading_added": False,
        "next_stage_started": False,
        "historical_artifacts_unchanged": True,
        "runtime_s": time.perf_counter() - started,
    }
    _write("u5_phase1_speed_screen.json", phase1_artifact)
    _write("u5_high_altitude_screen.json", phase2_artifact)
    _write("u5_candidate_comparison.json", comparison)
    _write("u5_aircraft_freeze.json", freeze)
    _write("u5_provenance.json", provenance)
    _write("u5_runs.json", {"provenance_id": provenance_id, "runs": all_runs})
    _write("u5_points.json", {"provenance_id": provenance_id, "points": all_points})
    _write("u5_result.json", result)
    return result


if __name__ == "__main__":
    print(json.dumps(run_u5(), indent=2, allow_nan=False))
