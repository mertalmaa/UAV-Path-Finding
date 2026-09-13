"""Freeze Production Stack V2 and regenerate the raw c172r LUT.

This stage changes only mixture handling.  It deliberately reuses the frozen
U3 outer loop, stock c172r FCS, U4 acceptance logic, and three-cold-start
measurement contract.
"""

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
from statistics import fmean
from typing import Any

import jsbsim
import yaml

import production_mixture_policy as mixture_policy
import u4_raw_lut_characterization as u4


HERE = Path(__file__).resolve().parent
RESULTS_DIR = HERE / "results"
CONFIG_PATH = HERE / "u4_1a3_raw_lut_v2_configuration.yaml"
POLICY_PATH = HERE / "production_mixture_policy.py"
U4_CONFIG_PATH = HERE / "u4_raw_lut_configuration.yaml"
U4_HARNESS_PATH = HERE / "u4_raw_lut_characterization.py"
U1_CONFIG_PATH = HERE / "u1_profile_validation_configuration.yaml"
U1_HARNESS_PATH = HERE / "u1_profile_validation.py"
U3_CONFIG_PATH = HERE / "u3_aircraft_selection_configuration.yaml"
U3_HARNESS_PATH = HERE / "u3_aircraft_selection.py"
U4_PROVENANCE_PATH = RESULTS_DIR / "u4_provenance.json"
U4_1A2_PROVENANCE_PATH = RESULTS_DIR / "u4_1a2_provenance.json"

_BASE_U4_SNAPSHOT = u4._snapshot
_BASE_SET_RUNNING = u4.u1._set_all_engines_running
_BASE_SET_THROTTLE = u4.u1._set_symmetric_throttle


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


def _git_commit() -> str | None:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=HERE.parent, text=True
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def _production_snapshot(
    fdm: Any, elapsed_s: float, config: dict[str, Any]
) -> dict[str, float]:
    sample = _BASE_U4_SNAPSHOT(fdm, elapsed_s, config)
    pressure = float(fdm[mixture_policy.PRESSURE_PROPERTY])
    raw_command = pressure / mixture_policy.REFERENCE_PRESSURE_PSF
    command = float(fdm[mixture_policy.COMMAND_PROPERTY])
    position = float(fdm[mixture_policy.POSITION_PROPERTY])
    extra = {
        "ambient_pressure_psf": pressure,
        "mixture_policy_raw_norm": raw_command,
        "mixture_policy_expected_norm": mixture_policy.command_from_pressure_psf(pressure),
        "mixture_cmd_norm": command,
        "mixture_pos_norm": position,
        "mixture_cmd_position_error_norm": command - position,
        "engine_0_fuel_flow_gph": float(fdm["propulsion/engine/fuel-flow-rate-gph"]),
        "engine_0_fuel_flow_pps": float(fdm["propulsion/engine/fuel-flow-rate-pps"]),
        "engine_0_power_hp": float(fdm["propulsion/engine/power-hp"]),
        "engine_0_map_pa": float(fdm["propulsion/engine/map-pa"]),
    }
    sample.update(extra)
    if u4._CAPTURE is not None:
        u4._CAPTURE[-1].update(extra)
    return sample


def _set_running_with_policy(fdm: Any, config: dict[str, Any]) -> None:
    _BASE_SET_RUNNING(fdm, config)
    mixture_policy.apply(fdm, set_position=True)


def _set_throttle_with_policy(
    fdm: Any, config: dict[str, Any], command: float
) -> None:
    _BASE_SET_THROTTLE(fdm, config, command)
    mixture_policy.apply(fdm, set_position=True)


def _install_policy_hooks() -> None:
    u4.u1._snapshot = _production_snapshot
    u4.u1._set_all_engines_running = _set_running_with_policy
    u4.u1._set_symmetric_throttle = _set_throttle_with_policy


def _restore_policy_hooks() -> None:
    u4.u1._snapshot = u4._snapshot
    u4.u1._set_all_engines_running = _BASE_SET_RUNNING
    u4.u1._set_symmetric_throttle = _BASE_SET_THROTTLE


def _rename_ids(
    records: list[dict[str, Any]],
    point: dict[str, Any],
    traces: list[dict[str, Any]],
) -> None:
    mapping: dict[str, str] = {}
    for record in records:
        old = record["run_id"]
        new = old.replace("u4-", "u4.1a3-", 1)
        record["run_id"] = new
        mapping[old] = new
    point["run_ids"] = [mapping[value] for value in point["run_ids"]]
    for trace in traces:
        trace["run_id"] = mapping[trace["run_id"]]


def _run_point(
    altitude_m: float,
    maneuver: str,
    target_value: float,
    execution_config: dict[str, Any],
    provenance_id: str,
) -> tuple[list[dict[str, Any]], dict[str, Any], list[dict[str, Any]]]:
    records, point, traces = u4._run_point(
        altitude_m, maneuver, target_value, execution_config, provenance_id
    )
    _rename_ids(records, point, traces)
    return records, point, traces


def _field_summary(records: list[dict[str, Any]], field: str) -> dict[str, Any]:
    return {
        "mean": fmean(float(record["measurement"][field]["mean"]) for record in records),
        "min": min(float(record["measurement"][field]["min"]) for record in records),
        "max": max(float(record["measurement"][field]["max"]) for record in records),
        "per_repeat_mean": [
            float(record["measurement"][field]["mean"]) for record in records
        ],
    }


def _engine_mixture_telemetry(records: list[dict[str, Any]]) -> dict[str, Any]:
    fields = {
        "ambient_pressure_psf": "ambient_pressure_psf",
        "policy_raw_command_norm": "mixture_policy_raw_norm",
        "policy_expected_command_norm": "mixture_policy_expected_norm",
        "mixture_command_norm": "mixture_cmd_norm",
        "mixture_position_norm": "mixture_pos_norm",
        "mixture_command_position_error_norm": "mixture_cmd_position_error_norm",
        "engine_running": "engine_0_running",
        "engine_rpm": "engine_0_state",
        "propeller_rpm": "engine_0_propeller_rpm",
        "thrust_lbs": "engine_0_thrust_lbs",
        "power_hp": "engine_0_power_hp",
        "fuel_flow_gph": "engine_0_fuel_flow_gph",
        "fuel_flow_pps": "engine_0_fuel_flow_pps",
        "manifold_pressure_pa": "engine_0_map_pa",
        "throttle_command_norm": "engine_0_throttle_cmd_norm",
        "throttle_position_norm": "engine_0_throttle_pos_norm",
    }
    result = {name: _field_summary(records, field) for name, field in fields.items()}
    result["healthy"] = (
        result["engine_running"]["min"] >= 1.0
        and result["engine_rpm"]["min"] > 0.0
        and result["propeller_rpm"]["min"] > 0.0
        and result["power_hp"]["min"] > 0.0
        and result["fuel_flow_gph"]["min"] > 0.0
        and max(
            abs(result["mixture_command_position_error_norm"]["min"]),
            abs(result["mixture_command_position_error_norm"]["max"]),
        )
        <= 1.0e-12
    )
    return result


def _augment_row(row: dict[str, Any], records: list[dict[str, Any]]) -> dict[str, Any]:
    row["production_mixture_policy_id"] = mixture_policy.POLICY_ID
    row["engine_mixture_telemetry"] = _engine_mixture_telemetry(records)
    return row


def _straight_summary(
    row: dict[str, Any], records: list[dict[str, Any]]
) -> dict[str, Any]:
    telemetry = row["engine_mixture_telemetry"]
    return {
        "altitude_m": row["altitude_m"],
        "duration_s": 40.0,
        "cold_start_repetitions": len(records),
        "status": row["status"],
        "failure_class": row["failure_class"],
        "actual_ias_mps": row["actual_ias_mps"],
        "actual_vz_mps": row["actual_vz_mps"],
        "actual_altitude_msl_m": row["actual_altitude_msl_m"],
        "pitch_deg": row["pitch_deg"],
        "aoa_deg": row["aoa_deg"],
        "throttle_norm": row["throttle_norm"],
        "engine_mixture_telemetry": telemetry,
        "controller_saturation_in_measurement": row[
            "controller_saturation_in_measurement"
        ],
        "settling": row["settling"],
        "repeatability": row["repeatability"],
        "trim": [record["trim"] for record in records],
        "status_reasons": sorted(
            {reason for record in records for reason in record["status_reasons"]}
        ),
        "failed_checks": sorted(
            {
                name
                for record in records
                for name, passed in record["checks"].items()
                if not passed
            }
        ),
        "run_ids": row["run_ids"],
        "low_altitude_gate_member": row["altitude_m"] <= 2500.0,
        "gate_passed": row["status"] == "VALID" and telemetry["healthy"],
    }


def _historical_paths(config: dict[str, Any]) -> dict[str, Path]:
    return {
        name: HERE / item["path"]
        for name, item in config["historical_artifacts"].items()
    }


def _verify_historical_hashes(config: dict[str, Any]) -> dict[str, str]:
    paths = _historical_paths(config)
    actual = {name: _sha256(path) for name, path in paths.items()}
    expected = {
        name: item["sha256"]
        for name, item in config["historical_artifacts"].items()
    }
    if actual != expected:
        raise RuntimeError(
            f"historical artifact hash mismatch: expected={expected!r}, actual={actual!r}"
        )
    return actual


def _mixture_rule_audit(
    config: dict[str, Any], root: Path
) -> dict[str, Any]:
    diagnostic = _load_json(RESULTS_DIR / "u4_1a2_high_altitude_diagnostic.json")
    observed = []
    for row in diagnostic["mixture_only_diagnostic_runs"]:
        pressure = float(row["standard_atmosphere_pressure_psf"])
        requested = float(row["requested_mixture_command_norm"])
        expected = mixture_policy.command_from_pressure_psf(pressure)
        observed.append(
            {
                "altitude_m": row["altitude_m"],
                "pressure_psf": pressure,
                "observed_command_norm": requested,
                "formula_command_norm": expected,
                "absolute_difference": abs(requested - expected),
                "matches_exact_formula_within_1e-12": abs(requested - expected)
                <= 1.0e-12,
            }
        )
    c172x_path = root / "aircraft" / "c172x" / "c172x.xml"
    c172r_path = root / "aircraft" / "c172r" / "c172r.xml"
    return {
        "u4_1a2_observations": observed,
        "all_observations_match": all(
            row["matches_exact_formula_within_1e-12"] for row in observed
        ),
        "u4_1a2_diagnostic_execution": {
            "formula": "atmosphere/P-psf / 2117.0",
            "input_sampling": "once from a target-altitude c172r run_ic pressure probe",
            "clipping": "none in the diagnostic helper",
            "initialization": (
                "the computed value replaced fixture mixture_command_norm before run_ic; "
                "after engine-start initialization the diagnostic wrapper explicitly wrote "
                "both fcs/mixture-cmd-norm and fcs/mixture-pos-norm"
            ),
            "replay_update_frequency": "no explicit per-frame recomputation; the target-altitude command remained fixed",
            "source_harness": str((HERE / "u4_1a2_suitability_diagnostic.py").resolve()),
        },
        "frozen_policy": mixture_policy.metadata(),
        "jsbsim_property_semantics": {
            "command": "fcs/mixture-cmd-norm is the normalized pilot/FCS mixture command",
            "position": "fcs/mixture-pos-norm is the normalized position consumed by piston propulsion",
            "production_application": "command and position are both written because stock c172r declares no mixture actuator/dynamic system",
        },
        "local_stock_model_audit": {
            "c172x_reference_path": str(c172x_path.resolve()),
            "c172x_reference_sha256": _sha256(c172x_path),
            "c172x_reference_lines": "automatic mixture table maps atmosphere/P-psf: 0->0.0 and 2117->1.0, output fcs/mixture-cmd-norm",
            "c172r_path": str(c172r_path.resolve()),
            "c172r_sha256": _sha256(c172r_path),
            "c172r_has_automatic_mixture_system": False,
            "engine": "engIO360C naturally aspirated piston, 180 hp",
            "propeller": "prop_Clark_Y7570 fixed pitch 21.6 deg",
        },
        "decision": (
            "Adopt the stock-c172x pressure-ratio behavior as a c172r harness policy; "
            "do not edit stock XML and do not use an altitude lookup table."
        ),
        "same_policy_required_for_final_replay": True,
    }


def _compare_old_v2(
    old: dict[str, Any], new: dict[str, Any], provenance_id: str
) -> dict[str, Any]:
    overlap = [0.0, 1000.0, 2000.0, 2500.0]
    old_straight = {
        float(row["altitude_m"]): row
        for row in old["straight_gates"] + old.get("optional_straight_probes", [])
    }
    new_straight = {float(row["altitude_m"]): row for row in new["straight_gates"]}
    straight = []
    turns = []
    vertical = []
    for altitude in overlap:
        before, after = old_straight[altitude], new_straight[altitude]
        straight.append(
            {
                "altitude_m": altitude,
                "old_status": before["status"],
                "v2_status": after["status"],
                "old_mixture_command_norm": 1.0,
                "v2_mixture_command_norm": after["engine_mixture_telemetry"][
                    "mixture_command_norm"
                ]["mean"],
                "metrics": {
                    "ias_mps": {
                        "old": before["actual_ias_mps"],
                        "v2": after["actual_ias_mps"],
                        "delta_v2_minus_old": after["actual_ias_mps"]
                        - before["actual_ias_mps"],
                    },
                    "vertical_speed_mps": {
                        "old": before["actual_vz_mps"],
                        "v2": after["actual_vz_mps"],
                        "delta_v2_minus_old": after["actual_vz_mps"]
                        - before["actual_vz_mps"],
                    },
                    "engine_rpm": {
                        "old": before["engine_state_mean"],
                        "v2": after["engine_state_mean"],
                        "delta_v2_minus_old": after["engine_state_mean"]
                        - before["engine_state_mean"],
                    },
                    "throttle_norm": {
                        "old": before["throttle_norm"],
                        "v2": after["throttle_norm"],
                        "delta_v2_minus_old": after["throttle_norm"]
                        - before["throttle_norm"],
                    },
                },
            }
        )
        for target in (-10.0, 10.0, -15.0, 15.0, -20.0, 20.0, -25.0, 25.0):
            before = next(
                row for row in old["turn_lut"]
                if float(row["altitude_m"]) == altitude
                and float(row["target_bank_deg"]) == target
            )
            after = next(
                row for row in new["turn_lut"]
                if float(row["altitude_m"]) == altitude
                and float(row["target_bank_deg"]) == target
            )
            turns.append(
                {
                    "altitude_m": altitude,
                    "target_bank_deg": target,
                    "old_status": before["status"],
                    "v2_status": after["status"],
                    "measured_radius_m": {
                        "old": before["measured_radius_m"],
                        "v2": after["measured_radius_m"],
                        "delta_v2_minus_old": after["measured_radius_m"]
                        - before["measured_radius_m"],
                    },
                    "turn_rate_deg_s": {
                        "old": before["turn_rate_deg_s"],
                        "v2": after["turn_rate_deg_s"],
                        "delta_v2_minus_old": after["turn_rate_deg_s"]
                        - before["turn_rate_deg_s"],
                    },
                }
            )
        for target in (-5.0, -4.0, -3.0, -2.0, 2.0, 3.0, 4.0, 5.0):
            before = next(
                row for row in old["vertical_lut"]
                if float(row["altitude_m"]) == altitude
                and float(row["target_vz_mps"]) == target
            )
            after = next(
                row for row in new["vertical_lut"]
                if float(row["altitude_m"]) == altitude
                and float(row["target_vz_mps"]) == target
            )
            vertical.append(
                {
                    "altitude_m": altitude,
                    "target_vz_mps": target,
                    "old_status": before["status"],
                    "v2_status": after["status"],
                    "achieved_vertical_speed_mps": {
                        "old": before["actual_vz_mps"],
                        "v2": after["actual_vz_mps"],
                        "delta_v2_minus_old": after["actual_vz_mps"]
                        - before["actual_vz_mps"],
                    },
                }
            )
    low_altitude_command_history = []
    for altitude in (0.0, 500.0, 1000.0, 1500.0, 2000.0, 2500.0):
        after = new_straight[altitude]
        new_command = after["engine_mixture_telemetry"]["mixture_command_norm"]["mean"]
        low_altitude_command_history.append(
            {
                "altitude_m": altitude,
                "old_full_rich_command_norm": 1.0,
                "v2_mean_command_norm": new_command,
                "command_changed": not math.isclose(
                    new_command, 1.0, rel_tol=0.0, abs_tol=1.0e-12
                ),
                "history_identical": False,
            }
        )
    return {
        "schema_version": 1,
        "artifact_type": "OLD_VS_RAW_LUT_V2_OVERLAP_COMPARISON",
        "provenance_id": provenance_id,
        "overlap_altitudes_m": overlap,
        "reuse_decision": "CASE_B_FULL_RAW_RERUN",
        "reason": (
            "Production Mixture Policy V1 changes mixture command and command "
            "history at every overlap altitude; the exact-data reuse contract is not met."
        ),
        "old_points_reused": 0,
        "low_altitude_command_history_audit": low_altitude_command_history,
        "all_0_to_2500_m_commands_changed": all(
            row["command_changed"] for row in low_altitude_command_history
        ),
        "straight": straight,
        "turn": turns,
        "vertical": vertical,
    }


def _ceiling_assessment(
    raw_lut: dict[str, Any], config: dict[str, Any], provenance_id: str
) -> dict[str, Any]:
    assessments = []
    for altitude in (float(value) for value in config["ceiling_candidates_m"]):
        straight = next(
            row for row in raw_lut["straight_gates"] if row["altitude_m"] == altitude
        )
        turns = [
            row for row in raw_lut["turn_lut"]
            if row["altitude_m"] == altitude and row["target_bank_deg"] != 0.0
        ]
        vertical = [
            row for row in raw_lut["vertical_lut"]
            if row["altitude_m"] == altitude and row["target_vz_mps"] != 0.0
        ]
        left = any(row["status"] == "VALID" and row["target_bank_deg"] < 0 for row in turns)
        right = any(row["status"] == "VALID" and row["target_bank_deg"] > 0 for row in turns)
        climb = any(row["status"] == "VALID" and row["target_vz_mps"] > 0 for row in vertical)
        descent = any(row["status"] == "VALID" and row["target_vz_mps"] < 0 for row in vertical)
        maneuver_rows = turns + vertical
        status_counts = Counter(row["status"] for row in maneuver_rows)
        repeatable = bool(maneuver_rows) and all(
            row["repeatability"]["passed"] for row in maneuver_rows
        ) and straight["repeatability"]["passed"]
        saturation_counts = Counter(
            key
            for row in maneuver_rows
            for key, saturated in row["controller_saturation_in_measurement"].items()
            if saturated
        )
        requirements = {
            "stable_straight": straight["status"] == "VALID"
            and straight["settling"]["all_repeats_settled_before_measurement"],
            "healthy_engine": straight["engine_mixture_telemetry"]["healthy"],
            "basic_turn_left_and_right": left and right,
            "basic_vertical_climb_and_descent": climb and descent,
            "repeatable": repeatable,
        }
        assessments.append(
            {
                "altitude_m": altitude,
                "straight_status": straight["status"],
                "actual_ias_mps": straight["actual_ias_mps"],
                "actual_vz_mps": straight["actual_vz_mps"],
                "engine_mixture_telemetry": straight["engine_mixture_telemetry"],
                "valid_nonzero_turn_targets_deg": [
                    row["target_bank_deg"] for row in turns if row["status"] == "VALID"
                ],
                "valid_nonzero_vertical_targets_mps": [
                    row["target_vz_mps"] for row in vertical if row["status"] == "VALID"
                ],
                "maneuver_status_counts": dict(sorted(status_counts.items())),
                "unknown_ratio": (
                    status_counts["UNKNOWN"] / len(maneuver_rows)
                    if maneuver_rows else None
                ),
                "infeasible_ratio": (
                    status_counts["INFEASIBLE"] / len(maneuver_rows)
                    if maneuver_rows else None
                ),
                "controller_saturation_counts": dict(sorted(saturation_counts.items())),
                "requirements": requirements,
                "usable": all(requirements.values()),
                "excessive_conservatism_override_used": False,
            }
        )
    usable = [row["altitude_m"] for row in assessments if row["usable"]]
    recommendation: float | str = max(usable) if usable else "NONE"
    return {
        "schema_version": 1,
        "artifact_type": "MISSION_RELEVANT_TESTED_ALTITUDE_CEILING_ASSESSMENT",
        "provenance_id": provenance_id,
        "candidate_altitudes_m": config["ceiling_candidates_m"],
        "usability_contract": config["ceiling_usability_contract"],
        "candidate_assessments": assessments,
        "recommended_tested_operational_ceiling_m": recommendation,
        "recommendation_is_service_ceiling": False,
        "true_aircraft_maximum_claimed": False,
        "planner_max_altitude_configuration_changed": False,
        "interpretation": (
            "Highest explicitly tested candidate satisfying straight, engine, bilateral-turn, "
            "bidirectional-vertical, and repeatability requirements."
        ),
    }


def run_u4_1a3() -> dict[str, Any]:
    started = time.perf_counter()
    generated_at = datetime.now(timezone.utc).isoformat()
    config = _load_yaml(CONFIG_PATH)
    old_u4_config = _load_yaml(U4_CONFIG_PATH)
    u4._freeze_check(old_u4_config)
    historical_before = _verify_historical_hashes(config)
    old_lut = _load_json(RESULTS_DIR / "aircraft_lut_raw.json")
    u4_provenance = _load_json(U4_PROVENANCE_PATH)
    u4_1a2_provenance = _load_json(U4_1A2_PROVENANCE_PATH)
    if u4_provenance["provenance_id"] != config["parent_u4_provenance_id"]:
        raise RuntimeError("parent U4 provenance changed")
    if u4_1a2_provenance["provenance_id"] != config["parent_u4_1a2_provenance_id"]:
        raise RuntimeError("parent U4.1A.2 provenance changed")

    root = Path(jsbsim.get_default_root_dir()).resolve()
    dependency_closure = u4.u3._dependency_closure("c172r", root)
    files = {
        "configuration": {"path": str(CONFIG_PATH.resolve()), "sha256": _sha256(CONFIG_PATH)},
        "harness": {"path": str(Path(__file__).resolve()), "sha256": _sha256(Path(__file__).resolve())},
        "production_mixture_policy": {"path": str(POLICY_PATH.resolve()), "sha256": _sha256(POLICY_PATH)},
        "u4_configuration": {"path": str(U4_CONFIG_PATH.resolve()), "sha256": _sha256(U4_CONFIG_PATH)},
        "u4_harness": {"path": str(U4_HARNESS_PATH.resolve()), "sha256": _sha256(U4_HARNESS_PATH)},
        "u1_configuration": {"path": str(U1_CONFIG_PATH.resolve()), "sha256": _sha256(U1_CONFIG_PATH)},
        "u1_harness": {"path": str(U1_HARNESS_PATH.resolve()), "sha256": _sha256(U1_HARNESS_PATH)},
        "u3_configuration": {"path": str(U3_CONFIG_PATH.resolve()), "sha256": _sha256(U3_CONFIG_PATH)},
        "u3_harness": {"path": str(U3_HARNESS_PATH.resolve()), "sha256": _sha256(U3_HARNESS_PATH)},
        "u4_provenance": {"path": str(U4_PROVENANCE_PATH.resolve()), "sha256": _sha256(U4_PROVENANCE_PATH)},
        "u4_1a2_provenance": {"path": str(U4_1A2_PROVENANCE_PATH.resolve()), "sha256": _sha256(U4_1A2_PROVENANCE_PATH)},
    }
    seed = {
        "files": {name: value["sha256"] for name, value in files.items()},
        "historical_artifacts": historical_before,
        "aircraft_dependency_closure": dependency_closure,
        "production_stack": config["production_stack"],
        "mixture_policy": mixture_policy.metadata(),
    }
    provenance_id = "u4.1a3-" + hashlib.sha256(
        json.dumps(seed, sort_keys=True).encode("utf-8")
    ).hexdigest()[:20]

    execution_config = copy.deepcopy(old_u4_config)
    execution_config["configuration_id"] = config["configuration_id"]
    execution_config["main_altitude_grid_m"] = config["main_altitude_grid_m"]
    execution_config["turn_bank_grid_deg"] = config["turn_bank_grid_deg"]
    execution_config["vertical_speed_grid_mps"] = config["vertical_speed_grid_mps"]
    execution_config["simulation"] = copy.deepcopy(config["simulation"])

    all_runs: list[dict[str, Any]] = []
    all_points: list[dict[str, Any]] = []
    straight_cache: dict[float, tuple[list[dict[str, Any]], dict[str, Any], list[dict[str, Any]]]] = {}
    sanity_rows: list[dict[str, Any]] = []
    straight_rows: list[dict[str, Any]] = []
    turn_rows: list[dict[str, Any]] = []
    vertical_rows: list[dict[str, Any]] = []

    _install_policy_hooks()
    try:
        for altitude_value in config["sanity_altitude_grid_m"]:
            altitude = float(altitude_value)
            records, point, traces = _run_point(
                altitude, "straight", 0.0, execution_config, provenance_id
            )
            straight_cache[altitude] = (records, point, traces)
            row = _augment_row(
                u4._straight_entry(point, records, altitude, provenance_id), records
            )
            sanity_rows.append(_straight_summary(row, records))
            all_runs.extend(records)
            all_points.append(point)

        low_altitude_gate_passed = all(
            row["gate_passed"]
            for row in sanity_rows
            if row["low_altitude_gate_member"]
        )
        sanity_artifact = {
            "schema_version": 1,
            "artifact_type": "PRODUCTION_MIXTURE_POLICY_SANITY_GATE",
            "provenance_id": provenance_id,
            "policy": mixture_policy.metadata(),
            "altitude_grid_m": config["sanity_altitude_grid_m"],
            "rows": sanity_rows,
            "low_altitude_gate_max_m": config["low_altitude_sanity_gate_max_m"],
            "low_altitude_gate_passed": low_altitude_gate_passed,
            "raw_v2_allowed_to_continue": low_altitude_gate_passed,
        }
        _write("u4_1a3_sanity_gate.json", sanity_artifact)
        if not low_altitude_gate_passed:
            raise RuntimeError("Production Mixture Policy V1 broke a low-altitude sanity gate")

        for altitude_value in config["main_altitude_grid_m"]:
            altitude = float(altitude_value)
            if altitude in straight_cache:
                records, point, _ = straight_cache[altitude]
            else:
                records, point, _ = _run_point(
                    altitude, "straight", 0.0, execution_config, provenance_id
                )
                all_runs.extend(records)
                all_points.append(point)
            straight = _augment_row(
                u4._straight_entry(point, records, altitude, provenance_id), records
            )
            straight["sanity_gate_run_reused_in_raw_v2"] = altitude in straight_cache
            straight_rows.append(straight)
            if point["status"] != "VALID":
                continue

            turn_rows.append(
                _augment_row(
                    u4._turn_entry(point, records, altitude, 0.0, provenance_id), records
                )
            )
            vertical_rows.append(
                _augment_row(
                    u4._vertical_entry(point, records, altitude, 0.0, provenance_id), records
                )
            )
            for bank_value in config["turn_bank_grid_deg"]:
                bank = float(bank_value)
                if bank == 0.0:
                    continue
                run_records, run_point, _ = _run_point(
                    altitude, "turn", bank, execution_config, provenance_id
                )
                all_runs.extend(run_records)
                all_points.append(run_point)
                turn_rows.append(
                    _augment_row(
                        u4._turn_entry(
                            run_point, run_records, altitude, bank, provenance_id
                        ),
                        run_records,
                    )
                )
            for vz_value in config["vertical_speed_grid_mps"]:
                vz = float(vz_value)
                if vz == 0.0:
                    continue
                run_records, run_point, _ = _run_point(
                    altitude, "vertical", vz, execution_config, provenance_id
                )
                all_runs.extend(run_records)
                all_points.append(run_point)
                vertical_rows.append(
                    _augment_row(
                        u4._vertical_entry(
                            run_point, run_records, altitude, vz, provenance_id
                        ),
                        run_records,
                    )
                )
    finally:
        _restore_policy_hooks()

    supported = [row["altitude_m"] for row in straight_rows if row["status"] == "VALID"]
    raw_lut = {
        "schema_version": 2,
        "artifact_type": "RAW_AIRCRAFT_LUT",
        "lut_stage": "raw",
        "provenance_id": provenance_id,
        "parent_u4_provenance_id": config["parent_u4_provenance_id"],
        "parent_u4_1a2_provenance_id": config["parent_u4_1a2_provenance_id"],
        "supersedes": {
            "path": str((RESULTS_DIR / "aircraft_lut_raw.json").resolve()),
            "provenance_id": old_lut["provenance_id"],
            "sha256": historical_before["u4_raw_lut"],
            "artifact_modified": False,
        },
        "generated_at_utc": generated_at,
        "code_version": {
            "repository_git_commit": _git_commit(),
            "harness_sha256": files["harness"]["sha256"],
            "policy_sha256": files["production_mixture_policy"]["sha256"],
        },
        "aircraft": "c172r",
        "aircraft_id": "c172r",
        "controller_stack_id": config["production_stack"]["stack_id"],
        "mixture_policy_id": mixture_policy.POLICY_ID,
        "controller_stack": config["production_stack"],
        "mixture_policy": mixture_policy.metadata(),
        "nominal_ias_mps": 40.0,
        "ias_context": "40 m/s indicated/calibrated airspeed target; speed is not a LUT state dimension",
        "speed_is_state_dimension": False,
        "main_altitude_grid_m": config["main_altitude_grid_m"],
        "turn_bank_grid_deg": config["turn_bank_grid_deg"],
        "vertical_speed_grid_mps": config["vertical_speed_grid_mps"],
        "cold_start_repetitions": config["simulation"]["cold_start_repetitions"],
        "measurement_contract": config["simulation"],
        "reuse_contract": config["reuse_contract"],
        "interpolated": False,
        "derated": False,
        "planner_ready": False,
        "true_maximum_claimed": False,
        "straight_gates": straight_rows,
        "optional_straight_probes": [],
        "turn_lut": turn_rows,
        "vertical_lut": vertical_rows,
        "left_right_symmetry": u4._symmetry(turn_rows),
        "altitude_boundary_summary": u4._altitude_summary(
            straight_rows, turn_rows, vertical_rows
        ),
        "supported_sweep_altitudes_m": supported,
        "future_holdout_plan": config["future_holdout_plan"],
    }
    comparison = _compare_old_v2(old_lut, raw_lut, provenance_id)
    ceiling = _ceiling_assessment(raw_lut, config, provenance_id)
    raw_lut["mission_relevant_ceiling_assessment"] = {
        "artifact": "u4_1a3_ceiling_assessment.json",
        "recommended_tested_operational_ceiling_m": ceiling[
            "recommended_tested_operational_ceiling_m"
        ],
        "recommendation_is_service_ceiling": False,
    }
    historical_after = _verify_historical_hashes(config)
    provenance = {
        "provenance_id": provenance_id,
        "generated_at_utc": generated_at,
        "repository_git_commit": _git_commit(),
        "parent_u4_provenance_id": config["parent_u4_provenance_id"],
        "parent_u4_1a2_provenance_id": config["parent_u4_1a2_provenance_id"],
        "jsbsim_python_package_version": importlib.metadata.version("jsbsim"),
        "jsbsim_compiled_version": jsbsim.FGFDMExec(None).get_version(),
        "jsbsim_git_commit": re.search(
            r"commit ([0-9a-f]{40})", jsbsim.FGFDMExec(None).get_version()
        ).group(1),
        "jsbsim_default_root": str(root),
        "python_version": sys.version,
        "python_executable": sys.executable,
        "platform": platform.platform(),
        "production_stack": config["production_stack"],
        "mixture_policy": mixture_policy.metadata(),
        "files": files,
        "aircraft_dependency_closure": dependency_closure,
        "historical_artifact_hashes_before": historical_before,
        "historical_artifact_hashes_after": historical_after,
        "historical_artifacts_unchanged": historical_before == historical_after,
        "external_primary_references": {
            "jsbsim_fgfcs_mixture_reference": "https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGFCS.html",
            "jsbsim_piston_reference": "https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGPiston.html",
            "jsbsim_engine_start_discussion": "https://github.com/JSBSim-Team/jsbsim/discussions/1264",
        },
    }
    mixture_audit = _mixture_rule_audit(config, root)
    result = {
        "step": "U4.1A.3",
        "step_status": "PASS",
        "provenance_id": provenance_id,
        "production_mixture_policy_status": "FROZEN",
        "production_stack_v2_status": "FROZEN",
        "reuse_case": "CASE_B_FULL_RAW_RERUN",
        "old_u4_points_reused": 0,
        "sanity_gate_passed": True,
        "main_altitudes_tested_m": config["main_altitude_grid_m"],
        "straight_valid_altitudes_m": supported,
        "straight_nonvalid_altitudes_m": [
            row["altitude_m"] for row in straight_rows if row["status"] != "VALID"
        ],
        "maneuver_sweep_altitudes_m": sorted(
            {row["altitude_m"] for row in turn_rows}
        ),
        "executed_new_points": len(all_points),
        "executed_new_cold_start_runs": len(all_runs),
        "turn_lut_rows": len(turn_rows),
        "vertical_lut_rows": len(vertical_rows),
        "recommended_tested_operational_ceiling_m": ceiling[
            "recommended_tested_operational_ceiling_m"
        ],
        "true_service_ceiling_claimed": False,
        "ready_for_u4_1b": True,
        "u4_1b_domain_constraint": (
            "Holdouts must remain inside V2's validated pointwise operational domain; "
            "NONE among the 5000/5500/6000 candidates is not a RAW LUT coherence failure."
        ),
        "historical_artifacts_unchanged": historical_before == historical_after,
        "controller_tuning_performed": False,
        "stock_xml_modified": False,
        "planner_modified": False,
        "planner_max_altitude_configuration_changed": False,
        "interpolation_performed": False,
        "derating_performed": False,
        "holdout_validation_started": False,
        "aircraft_switched": False,
        "acceptance_thresholds_modified": False,
        "u4_1b_started": False,
        "runtime_s": time.perf_counter() - started,
    }
    _write("aircraft_lut_raw_v2.json", raw_lut)
    _write("u4_1a3_mixture_policy_audit.json", mixture_audit)
    _write("u4_1a3_old_vs_v2.json", comparison)
    _write("u4_1a3_ceiling_assessment.json", ceiling)
    _write("u4_1a3_provenance.json", provenance)
    _write("u4_1a3_runs.json", {"provenance_id": provenance_id, "runs": all_runs})
    _write("u4_1a3_points.json", {"provenance_id": provenance_id, "points": all_points})
    _write("u4_1a3_result.json", result)
    return result


if __name__ == "__main__":
    print(json.dumps(run_u4_1a3(), indent=2, allow_nan=False))
