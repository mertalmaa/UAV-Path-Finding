"""Generate the raw c172r maneuver LUT on the frozen U3 stack.

No interpolation, derating, planner integration, controller tuning, or aircraft
XML mutation is performed here.
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
import time
from collections import Counter
from pathlib import Path
from statistics import fmean
from typing import Any

import jsbsim
import yaml

import u3_aircraft_selection as u3


HERE = Path(__file__).resolve().parent
CONFIG_PATH = HERE / "u4_raw_lut_configuration.yaml"
U1_CONFIG_PATH = HERE / "u1_profile_validation_configuration.yaml"
U1_HARNESS_PATH = HERE / "u1_profile_validation.py"
U2_CONFIG_PATH = HERE / "u2_candidate_screening_configuration.yaml"
U2_HARNESS_PATH = HERE / "u2_candidate_screening.py"
U3_CONFIG_PATH = HERE / "u3_aircraft_selection_configuration.yaml"
U3_HARNESS_PATH = HERE / "u3_aircraft_selection.py"
U3_PROVENANCE_PATH = HERE / "results" / "u3_provenance.json"
U3_RESULT_PATH = HERE / "results" / "u3_result.json"
RESULTS_DIR = HERE / "results"

u1 = u3.u2.u1
_BASE_SNAPSHOT = u1._snapshot
_BASE_NEW_TRIMMED_FDM = u1._new_trimmed_fdm
_CAPTURE: list[dict[str, float]] | None = None


def _load_yaml(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write(name: str, value: Any) -> None:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    (RESULTS_DIR / name).write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _snapshot(fdm: Any, elapsed_s: float, config: dict[str, Any]) -> dict[str, float]:
    sample = _BASE_SNAPSHOT(fdm, elapsed_s, config)
    sample["mach"] = fdm["velocities/mach"]
    if _CAPTURE is not None:
        _CAPTURE.append(copy.deepcopy(sample))
    return sample


u1._snapshot = _snapshot


def _new_trimmed_fdm(
    model: str, target_ias_mps: float, config: dict[str, Any]
) -> tuple[Any, dict[str, Any]]:
    """Discard trim-corrupted propulsion state and retry from a fresh IC.

    JSBSim's failed c172r trim can leave the piston engine stopped. Continuing
    that object would turn a fixture failure into false power-limit evidence.
    The fallback changes no model/controller value and is consistent with the
    frozen policy that trim is only an initialization aid.
    """
    fdm, trim = _BASE_NEW_TRIMMED_FDM(model, target_ias_mps, config)
    if trim["succeeded"]:
        trim["fresh_untrimmed_fallback"] = False
        return fdm, trim

    fdm = jsbsim.FGFDMExec(None)
    fdm.set_debug_level(0)
    fdm.set_dt(float(config["simulation"]["timestep_s"]))
    if not fdm.load_model(model):
        raise RuntimeError("fallback load_model returned false")
    u1._set_environment_and_aircraft(fdm, config)
    fixture = config["fixture"]
    fdm["position/terrain-elevation-asl-ft"] = float(
        fixture["synthetic_terrain_elevation_msl_m"]
    ) * u1.M_TO_FT
    fdm["ic/h-sl-ft"] = float(fixture["representative_altitude_msl_m"]) * u1.M_TO_FT
    fdm["ic/vc-kts"] = target_ias_mps / u1.KTS_TO_MPS
    fdm["ic/gamma-deg"] = 0.0
    fdm["ic/phi-deg"] = 0.0
    fdm["ic/theta-deg"] = 0.0
    fdm["ic/psi-true-deg"] = 0.0
    if not fdm.run_ic():
        raise RuntimeError("fallback run_ic returned false")
    u1._set_environment_and_aircraft(fdm, config)
    fdm["position/terrain-elevation-asl-ft"] = float(
        fixture["synthetic_terrain_elevation_msl_m"]
    ) * u1.M_TO_FT
    u1._set_all_engines_running(fdm, config)
    if not fdm.run():
        raise RuntimeError("fallback initialization frame failed")
    if fdm["propulsion/engine/engine-rpm"] <= 0.0:
        raise RuntimeError("fresh untrimmed fallback did not start c172r engine")
    trim["fresh_untrimmed_fallback"] = True
    return fdm, trim


u1._new_trimmed_fdm = _new_trimmed_fdm


def _freeze_check(config: dict[str, Any]) -> dict[str, Any]:
    u3_config = _load_yaml(U3_CONFIG_PATH)
    u3_provenance = json.loads(U3_PROVENANCE_PATH.read_text(encoding="utf-8"))
    u3_result = json.loads(U3_RESULT_PATH.read_text(encoding="utf-8"))
    u1_config = _load_yaml(U1_CONFIG_PATH)
    candidate = u3_config["candidates"]["c172r"]
    fixture = u3._candidate_fixture("c172r", candidate["fixture_source"])
    # Altitude is the U4 independent variable; U2 selection scores are metadata,
    # not physical fixture inputs. Every other U3 fixture value must match.
    fixture.pop("representative_altitude_msl_m")
    fixture.pop("native_model_quality_score")
    fixture.pop("model_simplicity_score")
    fixture.pop("generic_uav_profile_score")
    expected_fixture = copy.deepcopy(config["frozen_fixture"])
    expected_cg = expected_fixture.pop("expected_cg_in")
    if fixture != expected_fixture:
        raise RuntimeError("U3 c172r fixture no longer matches the U4 freeze")
    expected_controller = copy.deepcopy(config["frozen_controller"])
    actual_controller = copy.deepcopy(u1_config["controller"])
    actual_controller["policy"] = expected_controller["policy"]
    if actual_controller != expected_controller:
        raise RuntimeError("U3/U1 controller gains or clamps changed")
    if u3_result["primary"] != "c172r" or u3_result["backup"] != "c172p":
        raise RuntimeError("U3 selection changed")
    if u3_provenance["provenance_id"] != config["parent_provenance_id"]:
        raise RuntimeError("U3 provenance id changed")
    return {
        "u3_provenance_id": u3_provenance["provenance_id"],
        "u3_primary": u3_result["primary"],
        "u3_stack": u3_result["selected_stack"],
        "fixture": fixture,
        "controller": expected_controller,
        "expected_cg_in": expected_cg,
    }


def _run_config(config: dict[str, Any], altitude_m: float) -> dict[str, Any]:
    u3_config = _load_yaml(U3_CONFIG_PATH)
    candidate = u3_config["candidates"]["c172r"]
    run_config = u3._run_config("c172r", candidate, u3_config)
    run_config["configuration_id"] = config["configuration_id"]
    run_config["run_prefix"] = "u4"
    run_config["simulation"] = copy.deepcopy(config["simulation"])
    run_config["simulation"].pop("persisted_trace_interval_s")
    run_config["controller"] = copy.deepcopy(config["frozen_controller"])
    run_config["fixture"]["representative_altitude_msl_m"] = float(altitude_m)
    return run_config


def _split_repeats(samples: list[dict[str, float]], repeats: int) -> list[list[dict[str, float]]]:
    groups: list[list[dict[str, float]]] = []
    for sample in samples:
        if not groups or sample["elapsed_s"] <= groups[-1][-1]["elapsed_s"]:
            groups.append([])
        groups[-1].append(sample)
    if len(groups) != repeats:
        raise RuntimeError(f"captured {len(groups)} repeats, expected {repeats}")
    return groups


def _settling(
    samples: list[dict[str, float]], maneuver: str, target_ias: float, target_value: float,
    run_config: dict[str, Any],
) -> dict[str, Any]:
    acceptance = run_config["acceptance"]
    ramp = float(run_config["simulation"]["target_ramp_s"])
    measurement_start = float(run_config["simulation"]["replay_duration_s"]) - float(
        run_config["simulation"]["steady_measurement_window_s"]
    )
    bank = target_value if maneuver == "turn" else 0.0
    vz = target_value if maneuver == "vertical" else 0.0
    vz_tol = (
        acceptance["level_vertical_speed_absolute_mps"]
        if maneuver == "straight" else acceptance["vertical_speed_tracking_absolute_mps"]
    )

    def within(sample: dict[str, float]) -> bool:
        return (
            abs(sample["ias_mps"] - target_ias) / target_ias <= acceptance["ias_tracking_relative"]
            and abs(sample["roll_deg"] - bank) <= acceptance["bank_tracking_absolute_deg"]
            and abs(sample["vertical_speed_mps"] - vz) <= vz_tol
        )

    after_ramp = [sample for sample in samples if sample["elapsed_s"] >= ramp]
    last_bad = max((sample["elapsed_s"] for sample in after_ramp if not within(sample)), default=None)
    if after_ramp and within(after_ramp[-1]):
        settled_s = ramp if last_bad is None else last_bad + float(run_config["simulation"]["timestep_s"])
    else:
        settled_s = None
    return {
        "definition": "earliest_post_ramp_time_after_which_IAS_bank_Vz_remain_within_U1_tracking_bands",
        "settling_time_s": settled_s,
        "settled_before_measurement_window": settled_s is not None and settled_s <= measurement_start,
        "measurement_window_start_s": measurement_start,
    }


def _trace_samples(
    samples: list[dict[str, float]], interval_s: float, run_id: str, provenance_id: str
) -> list[dict[str, Any]]:
    stride = max(1, round(interval_s / (samples[1]["elapsed_s"] - samples[0]["elapsed_s"])))
    keys = [
        "elapsed_s", "altitude_msl_m", "ias_mps", "cas_kts", "tas_mps", "mach",
        "vertical_speed_mps", "gamma_deg", "alpha_deg", "beta_deg", "roll_deg",
        "pitch_deg", "heading_deg", "heading_rate_deg_s", "nz", "stall_indicator",
        "aileron_cmd_norm", "rudder_cmd_norm", "elevator_cmd_norm",
        "left_aileron_pos_norm", "right_aileron_pos_norm", "rudder_pos_norm",
        "elevator_pos_norm", "engine_0_state", "engine_0_throttle_cmd_norm",
        "engine_0_throttle_pos_norm", "engine_0_propeller_rpm",
    ]
    selected = samples[::stride]
    if selected[-1] is not samples[-1]:
        selected.append(samples[-1])
    return [
        {"provenance_id": provenance_id, "run_id": run_id} | {key: sample[key] for key in keys}
        for sample in selected
    ]


def _repeatability(records: list[dict[str, Any]], maneuver: str) -> dict[str, Any]:
    fields = ["ias_mps", "roll_deg", "vertical_speed_mps", "engine_0_throttle_pos_norm"]
    if maneuver == "turn":
        fields.extend(["turn_rate_deg_s", "measured_radius_m"])
    metrics: dict[str, Any] = {}
    for field in fields:
        if field in {"turn_rate_deg_s", "measured_radius_m"}:
            values = [float(record["trajectory"][field]) for record in records]
        else:
            values = [float(record["measurement"][field]["mean"]) for record in records]
        mean = fmean(values)
        scale = max(abs(mean), 1.0) if field in {"roll_deg", "vertical_speed_mps"} else max(abs(mean), 1.0e-9)
        deviation = max(abs(value - mean) / scale for value in values)
        metrics[field] = {
            "values": values,
            "mean": mean,
            "maximum_relative_deviation": deviation,
            "passed": deviation <= 0.01,
        }
    return {"passed": all(item["passed"] for item in metrics.values()), "metrics": metrics}


def _failure_class(point: dict[str, Any], records: list[dict[str, Any]]) -> str | None:
    if point["status"] == "VALID":
        return None
    checks = [record.get("checks", {}) for record in records]
    evidence = [record.get("aircraft_limit_evidence", {}) for record in records]
    if any(
        not value
        for item in checks
        for name, value in item.items()
        if name.startswith("engine_")
    ):
        return "other"
    if any(item.get("stall_indicator") for item in evidence):
        return "stall_or_aero"
    if any(item.get("power_limited") for item in evidence):
        return "power_limited"
    if any(not item.get("surface_usage_below_limit", True) for item in checks):
        return "surface_authority"
    if not point["repeatability"]["passed"] or any(not item.get("turn_rate_stable", True) for item in checks):
        return "unstable"
    if any(not item.get("outer_loop_not_clipped_in_measurement", True) for item in checks):
        return "controller_limited"
    if any(not item.get("ias_tracking", True) for item in checks):
        return "speed_retention_failure"
    if any(
        not item.get(name, True)
        for item in checks
        for name in ("bank_tracking", "vertical_speed_tracking")
    ):
        return "controller_limited"
    return "other"


def _point_enhancement(
    point: dict[str, Any], records: list[dict[str, Any]], groups: list[list[dict[str, float]]],
    maneuver: str, target_ias: float, target_value: float, altitude_m: float,
    run_config: dict[str, Any], provenance_id: str,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    traces: list[dict[str, Any]] = []
    settlements = []
    expected_cg = _load_yaml(CONFIG_PATH)["frozen_fixture"]["expected_cg_in"]
    for record, samples in zip(records, groups):
        record["run_id"] = record["run_id"].replace("u1-", "u4-", 1) + f"-h{altitude_m:.0f}"
        record["requested"]["altitude_msl_m"] = altitude_m
        record["measurement"]["mach"] = u1._stats([sample["mach"] for sample in samples if sample["elapsed_s"] >= 20.0])
        record["settling"] = _settling(samples, maneuver, target_ias, target_value, run_config)
        settlements.append(record["settling"])
        cg_ok = (
            abs(record["measurement"]["cg_x_in"]["mean"] - expected_cg["x"]) <= 1.0e-9
            and abs(record["measurement"]["cg_y_in"]["mean"] - expected_cg["y"]) <= 1.0e-9
            and abs(record["measurement"]["cg_z_in"]["mean"] - expected_cg["z"]) <= 1.0e-9
        )
        record["checks"]["cg_frozen"] = cg_ok
        if not cg_ok and record["status"] == "VALID":
            record["status"] = "UNKNOWN"
            record["status_reasons"].append("cg_frozen")
        engine_failed = any(
            not passed for name, passed in record["checks"].items()
            if name.startswith("engine_")
        )
        if engine_failed and record["trim"].get("fresh_untrimmed_fallback"):
            # The fresh set-running retry also stopped in this frozen full-rich
            # configuration. This is repeatable stack-level negative evidence,
            # not throttle-limited evidence and not a true aircraft ceiling.
            record["status"] = "INFEASIBLE"
            record["aircraft_limit_evidence"]["power_limited"] = False
            if "engine_not_sustained_in_frozen_fixture" not in record["status_reasons"]:
                record["status_reasons"].append("engine_not_sustained_in_frozen_fixture")
        elif not [name for name, passed in record["checks"].items() if not passed]:
            record["status"] = "VALID"
            record["status_reasons"] = []
        traces.extend(
            _trace_samples(
                samples, float(_load_yaml(CONFIG_PATH)["simulation"]["persisted_trace_interval_s"]),
                record["run_id"], provenance_id,
            )
        )
    point["run_ids"] = [record["run_id"] for record in records]
    point["run_statuses"] = [record["status"] for record in records]
    point["run_status_reasons"] = [record["status_reasons"] for record in records]
    point["status"] = (
        "VALID" if all(record["status"] == "VALID" for record in records) else
        "INFEASIBLE" if all(record["status"] == "INFEASIBLE" for record in records) else "UNKNOWN"
    )
    point["aircraft_model"] = "c172r"
    point["altitude_msl_m"] = altitude_m
    point["provenance_id"] = provenance_id
    point["repeatability"] = _repeatability(records, maneuver)
    if not point["repeatability"]["passed"]:
        point["status"] = "UNKNOWN"
    point["settling"] = {
        "all_repeats_settled_before_measurement": all(item["settled_before_measurement_window"] for item in settlements),
        "maximum_settling_time_s": max((item["settling_time_s"] for item in settlements if item["settling_time_s"] is not None), default=None),
        "per_repeat": settlements,
    }
    point["failure_class"] = _failure_class(point, records)
    return point, traces


def _run_point(
    altitude_m: float, maneuver: str, target_value: float, config: dict[str, Any], provenance_id: str
) -> tuple[list[dict[str, Any]], dict[str, Any], list[dict[str, Any]]]:
    global _CAPTURE
    run_config = _run_config(config, altitude_m)
    _CAPTURE = []
    records, point = u1._run_point(
        "c172r", maneuver, float(config["aircraft"]["nominal_ias_mps"]), float(target_value),
        run_config, provenance_id,
    )
    captured = _CAPTURE
    _CAPTURE = None
    groups = _split_repeats(captured, int(config["simulation"]["cold_start_repetitions"]))
    point, traces = _point_enhancement(
        point, records, groups, maneuver, float(config["aircraft"]["nominal_ias_mps"]),
        target_value, altitude_m, run_config, provenance_id,
    )
    return records, point, traces


def _mean(record: dict[str, Any], field: str) -> float:
    return float(record["measurement"][field]["mean"])


def _common_lut_fields(
    point: dict[str, Any], records: list[dict[str, Any]], altitude_m: float, provenance_id: str
) -> dict[str, Any]:
    measured = point["measured"]
    return {
        "provenance_id": provenance_id,
        "altitude_m": altitude_m,
        "target_ias_mps": 40.0,
        "status": point["status"],
        "failure_class": point["failure_class"],
        "actual_ias_mps": measured["ias_mps"],
        "actual_cas_kts": measured["cas_kts"],
        "actual_tas_mps": measured["tas_mps"],
        "mach": fmean(_mean(record, "mach") for record in records),
        "actual_altitude_msl_m": measured["altitude_msl_m"],
        "altitude_error_from_initial_m": measured["altitude_msl_m"] - altitude_m,
        "actual_bank_deg": measured["roll_deg"],
        "actual_vz_mps": measured["vertical_speed_mps"],
        "gamma_deg": measured["gamma_deg"],
        "pitch_deg": fmean(_mean(record, "pitch_deg") for record in records),
        "aoa_deg": measured["alpha_deg"],
        "beta_deg": measured["beta_deg"],
        "nz": measured["nz"],
        "throttle_norm": measured["engine_0_throttle_pos_norm"],
        "engine_state_mean": fmean(_mean(record, "engine_0_state") for record in records),
        "engine_state_property": "engine-rpm",
        "maximum_physical_surface_usage_norm": measured["maximum_surface_usage_norm"],
        "controller_saturation_in_measurement": {
            key: any(record["controller"]["saturation_in_measurement"][key] for record in records)
            for key in ("aileron", "rudder", "elevator", "throttle")
        },
        "settling": point["settling"],
        "repeatability": point["repeatability"],
        "run_ids": point["run_ids"],
    }


def _straight_entry(
    point: dict[str, Any], records: list[dict[str, Any]], altitude_m: float, provenance_id: str
) -> dict[str, Any]:
    return _common_lut_fields(point, records, altitude_m, provenance_id) | {
        "target_bank_deg": 0.0,
        "target_vz_mps": 0.0,
    }


def _turn_entry(
    point: dict[str, Any], records: list[dict[str, Any]], altitude_m: float,
    bank_deg: float, provenance_id: str,
) -> dict[str, Any]:
    entry = _common_lut_fields(point, records, altitude_m, provenance_id)
    if bank_deg == 0.0:
        turn_rate = fmean(_mean(record, "heading_rate_deg_s") for record in records)
        radius = None
        theory = None
        theory_difference = None
    else:
        turn_rate = point["measured"]["turn_rate_deg_s"]
        radius = point["measured"]["measured_radius_m"]
        theory = point["measured"]["theoretical_radius_m"]
        theory_difference = point["measured"]["radius_theory_relative_difference"]
    return entry | {
        "target_bank_deg": bank_deg,
        "direction": "LEFT" if bank_deg < 0 else ("RIGHT" if bank_deg > 0 else "STRAIGHT"),
        "turn_rate_deg_s": turn_rate,
        "measured_radius_m": radius,
        "theoretical_radius_m": theory,
        "radius_theory_relative_difference": theory_difference,
        "zero_bank_reuses_straight_gate_runs": bank_deg == 0.0,
    }


def _vertical_entry(
    point: dict[str, Any], records: list[dict[str, Any]], altitude_m: float,
    target_vz: float, provenance_id: str,
) -> dict[str, Any]:
    entry = _common_lut_fields(point, records, altitude_m, provenance_id)
    actual = entry["actual_vz_mps"]
    return entry | {
        "target_vz_mps": target_vz,
        "altitude_trend": "CLIMB" if actual > 0.1 else ("DESCENT" if actual < -0.1 else "LEVEL"),
        "speed_retention_ratio": entry["actual_ias_mps"] / entry["target_ias_mps"],
        "speed_error_mps": entry["actual_ias_mps"] - entry["target_ias_mps"],
        "zero_vz_reuses_straight_gate_runs": target_vz == 0.0,
    }


def _symmetry(turn_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    result = []
    altitudes = sorted({row["altitude_m"] for row in turn_rows})
    for altitude in altitudes:
        by_bank = {row["target_bank_deg"]: row for row in turn_rows if row["altitude_m"] == altitude}
        for magnitude in (10.0, 15.0, 20.0, 25.0):
            left, right = by_bank.get(-magnitude), by_bank.get(magnitude)
            if left is None or right is None:
                continue
            result.append({
                "altitude_m": altitude,
                "bank_magnitude_deg": magnitude,
                "left_status": left["status"],
                "right_status": right["status"],
                "turn_rate_magnitude_difference_deg_s": abs(abs(left["turn_rate_deg_s"]) - abs(right["turn_rate_deg_s"])),
                "radius_difference_m": abs(left["measured_radius_m"] - right["measured_radius_m"]),
                "actual_bank_magnitude_difference_deg": abs(abs(left["actual_bank_deg"]) - abs(right["actual_bank_deg"])),
                "nz_difference": abs(left["nz"] - right["nz"]),
                "beta_left_deg": left["beta_deg"],
                "beta_right_deg": right["beta_deg"],
                "beta_magnitude_difference_deg": abs(abs(left["beta_deg"]) - abs(right["beta_deg"])),
                "throttle_difference_norm": abs(left["throttle_norm"] - right["throttle_norm"]),
            })
    return result


def _altitude_summary(
    straight_rows: list[dict[str, Any]], turn_rows: list[dict[str, Any]], vertical_rows: list[dict[str, Any]]
) -> list[dict[str, Any]]:
    summaries = []
    for straight in straight_rows:
        altitude = straight["altitude_m"]
        turns = [row for row in turn_rows if row["altitude_m"] == altitude and row["target_bank_deg"] != 0]
        vertical = [row for row in vertical_rows if row["altitude_m"] == altitude and row["target_vz_mps"] != 0]
        bilateral = []
        for magnitude in (10.0, 15.0, 20.0, 25.0):
            pair = [row for row in turns if abs(row["target_bank_deg"]) == magnitude]
            if len(pair) == 2 and all(row["status"] == "VALID" for row in pair):
                bilateral.append(magnitude)
        largest = max(bilateral, default=None)
        boundary_turns = [row for row in turns if largest is not None and abs(row["target_bank_deg"]) == largest]
        valid_by_direction = {
            direction: [row for row in turns if row["direction"] == direction and row["status"] == "VALID"]
            for direction in ("LEFT", "RIGHT")
        }
        largest_by_direction = {
            direction: max((abs(row["target_bank_deg"]) for row in rows), default=None)
            for direction, rows in valid_by_direction.items()
        }
        largest_any = max(
            (value for value in largest_by_direction.values() if value is not None), default=None
        )
        any_boundary_turns = [
            row for rows in valid_by_direction.values() for row in rows
            if largest_any is not None and abs(row["target_bank_deg"]) == largest_any
        ]
        climbs = [row for row in vertical if row["target_vz_mps"] > 0 and row["status"] == "VALID"]
        descents = [row for row in vertical if row["target_vz_mps"] < 0 and row["status"] == "VALID"]
        failures = Counter(
            row["failure_class"] for row in turns + vertical if row["status"] != "VALID" and row["failure_class"]
        )
        if straight["status"] != "VALID" and straight["failure_class"]:
            failures[straight["failure_class"]] += 1
        summaries.append({
            "altitude_m": altitude,
            "straight_status": straight["status"],
            "sweeps_executed": bool(turns or vertical),
            "largest_tested_bilateral_valid_bank_deg": largest,
            "measured_radii_at_largest_bank_m": {row["direction"]: row["measured_radius_m"] for row in boundary_turns},
            "measured_turn_rates_at_largest_bank_deg_s": {row["direction"]: row["turn_rate_deg_s"] for row in boundary_turns},
            "largest_tested_valid_abs_bank_by_direction_deg": largest_by_direction,
            "largest_tested_any_direction_valid_abs_bank_deg": largest_any,
            "measured_radii_at_largest_any_direction_bank_m": {
                row["direction"]: row["measured_radius_m"] for row in any_boundary_turns
            },
            "measured_turn_rates_at_largest_any_direction_bank_deg_s": {
                row["direction"]: row["turn_rate_deg_s"] for row in any_boundary_turns
            },
            "highest_tested_valid_climb_target_mps": max((row["target_vz_mps"] for row in climbs), default=None),
            "highest_tested_valid_climb_actual_mps": max((row["actual_vz_mps"] for row in climbs), default=None),
            "largest_tested_valid_descent_target_magnitude_mps": max((-row["target_vz_mps"] for row in descents), default=None),
            "largest_tested_valid_descent_actual_magnitude_mps": max((-row["actual_vz_mps"] for row in descents), default=None),
            "major_failure_class": failures.most_common(1)[0][0] if failures else None,
            "tested_grid_values_are_true_limits": False,
        })
    return summaries


def run_u4() -> dict[str, Any]:
    started = time.perf_counter()
    config = _load_yaml(CONFIG_PATH)
    freeze = _freeze_check(config)
    root = Path(jsbsim.get_default_root_dir()).resolve()
    dependency_closure = u3._dependency_closure("c172r", root)
    files = {
        "u4_configuration": {"path": str(CONFIG_PATH.resolve()), "sha256": _sha256(CONFIG_PATH)},
        "u4_harness": {"path": str(Path(__file__).resolve()), "sha256": _sha256(Path(__file__).resolve())},
        "u3_configuration": {"path": str(U3_CONFIG_PATH.resolve()), "sha256": _sha256(U3_CONFIG_PATH)},
        "u3_harness": {"path": str(U3_HARNESS_PATH.resolve()), "sha256": _sha256(U3_HARNESS_PATH)},
        "u3_provenance": {"path": str(U3_PROVENANCE_PATH.resolve()), "sha256": _sha256(U3_PROVENANCE_PATH)},
        "u3_result": {"path": str(U3_RESULT_PATH.resolve()), "sha256": _sha256(U3_RESULT_PATH)},
        "u2_configuration": {"path": str(U2_CONFIG_PATH.resolve()), "sha256": _sha256(U2_CONFIG_PATH)},
        "u2_harness": {"path": str(U2_HARNESS_PATH.resolve()), "sha256": _sha256(U2_HARNESS_PATH)},
        "u1_configuration": {"path": str(U1_CONFIG_PATH.resolve()), "sha256": _sha256(U1_CONFIG_PATH)},
        "u1_harness": {"path": str(U1_HARNESS_PATH.resolve()), "sha256": _sha256(U1_HARNESS_PATH)},
    }
    seed = {
        "files": {name: value["sha256"] for name, value in files.items()},
        "dependencies": dependency_closure,
        "freeze": freeze,
    }
    provenance_id = "u4-" + hashlib.sha256(json.dumps(seed, sort_keys=True).encode()).hexdigest()[:20]

    all_runs: list[dict[str, Any]] = []
    all_points: list[dict[str, Any]] = []
    all_traces: list[dict[str, Any]] = []
    straight_rows: list[dict[str, Any]] = []
    probe_rows: list[dict[str, Any]] = []
    turn_rows: list[dict[str, Any]] = []
    vertical_rows: list[dict[str, Any]] = []

    for altitude in config["main_altitude_grid_m"]:
        records, point, traces = _run_point(float(altitude), "straight", 0.0, config, provenance_id)
        all_runs.extend(records); all_points.append(point); all_traces.extend(traces)
        straight = _straight_entry(point, records, float(altitude), provenance_id)
        straight_rows.append(straight)
        if point["status"] != "VALID":
            continue

        turn_rows.append(_turn_entry(point, records, float(altitude), 0.0, provenance_id))
        vertical_rows.append(_vertical_entry(point, records, float(altitude), 0.0, provenance_id))
        for bank in config["turn_bank_grid_deg"]:
            if float(bank) == 0.0:
                continue
            turn_records, turn_point, turn_traces = _run_point(float(altitude), "turn", float(bank), config, provenance_id)
            all_runs.extend(turn_records); all_points.append(turn_point); all_traces.extend(turn_traces)
            turn_rows.append(_turn_entry(turn_point, turn_records, float(altitude), float(bank), provenance_id))
        for vz in config["vertical_speed_grid_mps"]:
            if float(vz) == 0.0:
                continue
            vertical_records, vertical_point, vertical_traces = _run_point(float(altitude), "vertical", float(vz), config, provenance_id)
            all_runs.extend(vertical_records); all_points.append(vertical_point); all_traces.extend(vertical_traces)
            vertical_rows.append(_vertical_entry(vertical_point, vertical_records, float(altitude), float(vz), provenance_id))

    for altitude in config["optional_straight_probe_altitudes_m"]:
        records, point, traces = _run_point(float(altitude), "straight", 0.0, config, provenance_id)
        all_runs.extend(records); all_points.append(point); all_traces.extend(traces)
        row = _straight_entry(point, records, float(altitude), provenance_id)
        row["operational_domain_observation"] = (
            "STRAIGHT_LEVEL_FEASIBLE_PROBE_ONLY" if row["status"] == "VALID"
            else "UNSUPPORTED_OR_OUTSIDE_TESTED_OPERATIONAL_DOMAIN"
        )
        probe_rows.append(row)

    symmetry = _symmetry(turn_rows)
    altitude_summary = _altitude_summary(straight_rows, turn_rows, vertical_rows)
    supported_sweep_altitudes = [row["altitude_m"] for row in straight_rows if row["status"] == "VALID"]
    provenance = {
        "provenance_id": provenance_id,
        "parent_u3_provenance_id": config["parent_provenance_id"],
        "jsbsim_python_package_version": importlib.metadata.version("jsbsim"),
        "jsbsim_compiled_version": jsbsim.FGFDMExec(None).get_version(),
        "jsbsim_git_commit": re.search(r"commit ([0-9a-f]{40})", jsbsim.FGFDMExec(None).get_version()).group(1),
        "jsbsim_default_root": str(root),
        "python_version": sys.version,
        "python_executable": sys.executable,
        "platform": platform.platform(),
        "frozen_stack": freeze,
        "files": files,
        "aircraft_dependency_closure": dependency_closure,
    }
    raw_lut = {
        "schema_version": 1,
        "artifact_type": "RAW_AIRCRAFT_LUT",
        "provenance_id": provenance_id,
        "aircraft": "c172r",
        "nominal_ias_mps": 40.0,
        "speed_is_state_dimension": False,
        "main_altitude_grid_m": config["main_altitude_grid_m"],
        "turn_bank_grid_deg": config["turn_bank_grid_deg"],
        "vertical_speed_grid_mps": config["vertical_speed_grid_mps"],
        "interpolated": False,
        "derated": False,
        "planner_ready": False,
        "true_maximum_claimed": False,
        "straight_gates": straight_rows,
        "optional_straight_probes": probe_rows,
        "turn_lut": turn_rows,
        "vertical_lut": vertical_rows,
        "left_right_symmetry": symmetry,
        "altitude_boundary_summary": altitude_summary,
        "supported_sweep_altitudes_m": supported_sweep_altitudes,
        "future_holdout_plan": config["future_holdout_plan"],
        "planner_interface_preview": {
            "turn_query": ["altitude", "nominal_speed_context", "bank_or_turn_class", "direction"],
            "turn_return": ["status", "measured_turn_radius", "measured_turn_rate"],
            "vertical_query": ["altitude", "nominal_speed_context", "desired_vz"],
            "vertical_return": ["status", "achievable_vz", "gamma"],
        },
    }
    result = {
        "step": "U4",
        "step_status": "PASS" if turn_rows and vertical_rows else "PARTIAL",
        "provenance_id": provenance_id,
        "main_altitudes_tested_m": config["main_altitude_grid_m"],
        "straight_valid_altitudes_m": supported_sweep_altitudes,
        "turn_vertical_sweeps_completed_at_m": sorted({row["altitude_m"] for row in turn_rows}),
        "optional_probe_statuses": {str(int(row["altitude_m"])): row["status"] for row in probe_rows},
        "executed_points": len(all_points),
        "executed_cold_start_runs": len(all_runs),
        "turn_lut_rows": len(turn_rows),
        "vertical_lut_rows": len(vertical_rows),
        "controller_tuning_performed": False,
        "stock_xml_modified": False,
        "planner_modified": False,
        "interpolation_performed": False,
        "derating_performed": False,
        "holdouts_executed": False,
        "final_replay_implemented": False,
        "raw_lut_ready_for_holdout_validation": bool(turn_rows and vertical_rows),
        "runtime_s": time.perf_counter() - started,
    }
    _write("u4_provenance.json", provenance)
    _write("u4_runs.json", {"provenance_id": provenance_id, "runs": all_runs})
    _write("u4_points.json", {"provenance_id": provenance_id, "points": all_points})
    _write("u4_traces.json", {
        "provenance_id": provenance_id,
        "sample_interval_s": config["simulation"]["persisted_trace_interval_s"],
        "source_simulation_timestep_s": config["simulation"]["timestep_s"],
        "samples": all_traces,
    })
    _write("aircraft_lut_raw.json", raw_lut)
    _write("u4_result.json", result)
    return result


if __name__ == "__main__":
    print(json.dumps(run_u4(), indent=2, allow_nan=False))
