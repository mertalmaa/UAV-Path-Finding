"""U2 fail-fast screening of the installed stock JSBSim aircraft inventory.

The candidate set and fixtures are frozen in the U2 YAML before dynamic runs.
This script reuses U1's bounded controller without per-aircraft gain changes.
"""

from __future__ import annotations

import copy
import hashlib
import importlib.metadata
import json
import platform
import re
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from statistics import fmean
from typing import Any

import jsbsim
import yaml

import u1_profile_validation as u1


HERE = Path(__file__).resolve().parent
CONFIG_PATH = HERE / "u2_candidate_screening_configuration.yaml"
U1_CONFIG_PATH = HERE / "u1_profile_validation_configuration.yaml"
RESULTS_DIR = HERE / "results"
LB_PER_KG = 2.2046226218487757


_U1_SET_ENVIRONMENT_AND_AIRCRAFT = u1._set_environment_and_aircraft
_U1_BASE_CHECKS = u1._base_checks
_U1_NEW_TRIMMED_FDM = u1._new_trimmed_fdm
_U1_SNAPSHOT = u1._snapshot


def _u2_set_environment_and_aircraft(fdm: Any, config: dict[str, Any]) -> None:
    """Thin fixture/property adapter around the unchanged U1 interface."""
    _U1_SET_ENVIRONMENT_AND_AIRCRAFT(fdm, config)
    fixture = config["fixture"]
    for index, weight in enumerate(fixture.get("pointmass_weights_lbs", [])):
        fdm[f"inertia/pointmass-weight-lbs[{index}]"] = float(weight)
    if "gear_command_norm" in fixture:
        fdm["gear/gear-cmd-norm"] = float(fixture["gear_command_norm"])
    if "gear_position_norm" in fixture:
        fdm["gear/gear-pos-norm"] = float(fixture["gear_position_norm"])
    for name, value in fixture.get("interface_properties", {}).items():
        fdm[name] = float(value)
    if not fixture.get("propeller_state_required", True):
        for index in range(int(fixture["engine_count"])):
            fdm[f"propulsion/engine{u1._suffix(index)}/propeller-rpm"] = 0.0


def _u2_base_checks(
    samples: list[dict[str, float]], summary: dict[str, Any], config: dict[str, Any]
) -> dict[str, bool]:
    checks = _U1_BASE_CHECKS(samples, summary, config)
    if not config["fixture"].get("propeller_state_required", True):
        for index in range(int(config["fixture"]["engine_count"])):
            checks.pop(f"engine_{index}_propeller_rotating", None)
    if not config["fixture"].get("set_running_observable_required", True):
        for index in range(int(config["fixture"]["engine_count"])):
            checks.pop(f"engine_{index}_running", None)
    return checks


def _u2_snapshot(fdm: Any, elapsed_s: float, config: dict[str, Any]) -> dict[str, float]:
    sample = _U1_SNAPSHOT(fdm, elapsed_s, config)
    limits = config["fixture"].get("surface_max_degrees")
    if limits:
        sample["elevator_pos_norm"] = fdm["fcs/elevator-pos-deg"] / limits["elevator"]
        sample["left_aileron_pos_norm"] = fdm["fcs/left-aileron-pos-deg"] / limits["left_aileron"]
        sample["right_aileron_pos_norm"] = fdm["fcs/right-aileron-pos-deg"] / limits["right_aileron"]
        sample["rudder_pos_norm"] = fdm["fcs/rudder-pos-deg"] / limits["rudder"]
    return sample


def _u2_new_trimmed_fdm(
    model: str, target_ias_mps: float, config: dict[str, Any]
) -> tuple[Any, dict[str, Any]]:
    fdm, trim = _U1_NEW_TRIMMED_FDM(model, target_ias_mps, config)
    for name, value in config["fixture"].get("interface_properties", {}).items():
        fdm[name] = float(value)
    # Some stock piston models stop during an unsuccessful built-in trim. A
    # fresh set-running request restores the requested cold-start condition;
    # it does not alter controller gains or aerodynamic data.
    u1._set_all_engines_running(fdm, config)
    startup_steps = 100 if config["fixture"].get("startup_mode") == "piston_magneto_starter" else 1
    for _ in range(startup_steps):
        if not fdm.run():
            raise RuntimeError("post-trim engine-start frame failed")
    if config["fixture"].get("startup_mode") == "piston_magneto_starter":
        fdm["propulsion/starter_cmd"] = 0.0
        fdm["propulsion/engine/starter-norm"] = 0.0
    return fdm, trim


# Reuse the U1 controller/run path verbatim; only the explicitly documented
# property/fixture adapters above are replaced for U2 candidates.
u1._set_environment_and_aircraft = _u2_set_environment_and_aircraft
u1._base_checks = _u2_base_checks
u1._new_trimmed_fdm = _u2_new_trimmed_fdm
u1._snapshot = _u2_snapshot


def _load_yaml(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _number(node: ET.Element | None) -> float | None:
    if node is None or node.text is None:
        return None
    try:
        value = float(node.text.strip())
    except ValueError:
        return None
    return value * LB_PER_KG if node.attrib.get("unit", "").upper() == "KG" else value


def _main_model_xml(directory: Path) -> Path:
    preferred = directory / f"{directory.name}.xml"
    if preferred.is_file():
        return preferred
    choices = sorted(directory.glob("*.xml"))
    if not choices:
        raise FileNotFoundError(f"no aircraft XML in {directory}")
    return choices[0]


def _metadata(model: str, path: Path) -> dict[str, Any]:
    root = ET.parse(path).getroot()
    engines = root.findall("./propulsion/engine")
    description_node = root.find("./fileheader/description")
    description = " ".join("".join(description_node.itertext()).split()) if description_node is not None else None
    return {
        "model": model,
        "model_xml": str(path.resolve()),
        "model_xml_sha256": _sha256(path),
        "fdm_config_name": root.attrib.get("name"),
        "description": description,
        "empty_weight_lbs": _number(root.find("./mass_balance/emptywt")),
        "engine_count": len(engines),
        "engine_files": [engine.attrib.get("file") for engine in engines],
        "thruster_files": [
            engine.find("./thruster").attrib.get("file")
            if engine.find("./thruster") is not None else None
            for engine in engines
        ],
        "tank_count": len(root.findall("./propulsion/tank")),
        "system_files": [item.attrib.get("file") for item in root.findall("./system")],
        "flight_control_name": (
            root.find("./flight_control").attrib.get("name")
            if root.find("./flight_control") is not None else None
        ),
    }


def _inventory(config: dict[str, Any], aircraft_root: Path) -> list[dict[str, Any]]:
    reject_groups = config["static_filter"]["reject_groups"]
    reasons = config["static_filter"]["reasons"]
    rejection: dict[str, tuple[str, str]] = {}
    for group, models in reject_groups.items():
        for model in models:
            if model in rejection:
                raise ValueError(f"static rejection duplicated for {model}")
            rejection[model] = (group, reasons[group])
    candidates = set(config["dynamic_candidates"])
    overlap = candidates.intersection(rejection)
    if overlap:
        raise ValueError(f"models both kept and rejected: {sorted(overlap)}")

    rows = []
    for directory in sorted((item for item in aircraft_root.iterdir() if item.is_dir()), key=lambda p: p.name.lower()):
        row = _metadata(directory.name, _main_model_xml(directory))
        if directory.name in rejection:
            group, reason = rejection[directory.name]
            row.update({"static_decision": "REJECT_STATIC", "static_group": group, "static_reason": reason})
        else:
            row.update(
                {
                    "static_decision": "KEEP",
                    "static_group": "configured_dynamic_candidate" if directory.name in candidates else "uncertain_unclassified",
                    "static_reason": (
                        "powered_fixed_wing_and_not_excluded_by_available_static_metadata"
                        if directory.name in candidates
                        else "uncertain_model_not_arbitrarily_rejected"
                    ),
                }
            )
        rows.append(row)
    return rows


def _provenance(
    config: dict[str, Any], inventory: list[dict[str, Any]], aircraft_root: Path
) -> dict[str, Any]:
    probe = jsbsim.FGFDMExec(None)
    probe.set_debug_level(0)
    compiled = probe.get_version()
    files = {
        "u2_configuration": CONFIG_PATH,
        "u2_harness": Path(__file__).resolve(),
        "u1_configuration": U1_CONFIG_PATH,
        "u1_reused_harness": HERE / "u1_profile_validation.py",
    }
    hashes = {name: {"path": str(path), "sha256": _sha256(path)} for name, path in files.items()}
    fingerprint = {
        "compiled": compiled,
        "configuration": hashes["u2_configuration"]["sha256"],
        "harness": hashes["u2_harness"]["sha256"],
        "inventory": {row["model"]: row["model_xml_sha256"] for row in inventory},
    }
    commit = re.search(r"commit ([0-9a-f]{40})", compiled)
    build = re.search(r"GitHub build (\d+)", compiled)
    return {
        "provenance_id": "u2-" + hashlib.sha256(json.dumps(fingerprint, sort_keys=True).encode()).hexdigest()[:20],
        "jsbsim_python_package_version": importlib.metadata.version("jsbsim"),
        "jsbsim_compiled_version": compiled,
        "jsbsim_git_commit": commit.group(1) if commit else None,
        "jsbsim_github_build": int(build.group(1)) if build else None,
        "jsbsim_default_root": str(Path(jsbsim.get_default_root_dir()).resolve()),
        "aircraft_root": str(aircraft_root.resolve()),
        "python_version": sys.version,
        "python_executable": sys.executable,
        "platform": platform.platform(),
        "ias_measurement_contract": _load_yaml(U1_CONFIG_PATH)["ias_measurement_contract"],
        "controller_source": "u1_profile_validation.py::_run_point/_run_controlled_once",
        "files": hashes,
    }


def _run_config(model: str, config: dict[str, Any]) -> dict[str, Any]:
    run_config = _load_yaml(U1_CONFIG_PATH)
    run_config["configuration_id"] = config["configuration_id"]
    run_config["run_prefix"] = "u2"
    run_config["aircraft"]["backup_model"] = "__u2_no_backup__"
    run_config["fixture"] = {
        **config["fixture_common"],
        **config["dynamic_candidates"][model],
    }
    run_config["simulation"] = copy.deepcopy(config["simulation"])
    return run_config


_FIXTURE_CHECKS = {
    "finite_outputs",
    "measurement_window_complete",
    "flaps_frozen_clean",
    "fuel_frozen",
    "weight_frozen",
    "zero_wind",
}


def _point_has_fixture_failure(point: dict[str, Any], records: list[dict[str, Any]]) -> bool:
    if point.get("measured") is None:
        return True
    for record in records:
        checks = record.get("checks")
        if checks is None:
            return True
        for name, passed in checks.items():
            if (name in _FIXTURE_CHECKS or name.startswith("engine_")) and not passed:
                return True
    return False


def _point_summary(point: dict[str, Any]) -> dict[str, Any]:
    measured = point.get("measured")
    return {
        "maneuver": point["maneuver"],
        "requested": point["requested"],
        "status": point["status"],
        "run_statuses": point["run_statuses"],
        "run_status_reasons": point["run_status_reasons"],
        "repeatability": point["repeatability"],
        "measured": measured,
        "model_fixture_mismatch_evidence": point["model_fixture_mismatch_evidence"],
    }


def _rank(passers: list[str], points_by_model: dict[str, list[dict[str, Any]]], config: dict[str, Any]) -> list[dict[str, Any]]:
    rows = []
    for model in passers:
        points = points_by_model[model]
        speed = [item for item in points if item["gate"] == "speed"]
        climb = next(item for item in points if item["gate"] == "climb")
        turns = [item for item in points if item["gate"] == "turn"]
        speed_error = max(
            abs(item["measured"]["ias_mps"] - item["requested"]["ias_mps"]) / item["requested"]["ias_mps"]
            for item in speed
        )
        climb_throttles = [value for key, value in climb["measured"].items() if key.endswith("throttle_pos_norm")]
        climb_margin = 1.0 - max(climb_throttles)
        turn_error = max(
            abs(abs(item["measured"]["roll_deg"]) - 25.0) / 25.0
            + abs(item["measured"]["ias_mps"] - 35.0) / 35.0
            for item in turns
        )
        max_surface = max(item["measured"]["maximum_surface_usage_norm"] for item in points)
        fixture = config["dynamic_candidates"][model]
        repeatability_margin = min(
            1.0 - max(metric["maximum_relative_deviation"] for metric in item["repeatability"]["metrics"].values())
            for item in points
        )
        priority_tuple = [
            speed_error,
            -climb_margin,
            turn_error,
            -(1.0 - max_surface),
            -fixture["native_model_quality_score"],
            -repeatability_margin,
            -fixture["model_simplicity_score"],
            -fixture["generic_uav_profile_score"],
        ]
        rows.append(
            {
                "model": model,
                "speed_fit_max_relative_error": speed_error,
                "climb_throttle_margin_norm": climb_margin,
                "turn_fit_combined_max_relative_error": turn_error,
                "surface_saturation_margin_norm": 1.0 - max_surface,
                "native_model_quality_score": fixture["native_model_quality_score"],
                "initialization_repeatability_margin": repeatability_margin,
                "model_simplicity_score": fixture["model_simplicity_score"],
                "generic_uav_profile_score": fixture["generic_uav_profile_score"],
                "priority_tuple": priority_tuple,
            }
        )
    rows.sort(key=lambda item: item["priority_tuple"])
    for index, row in enumerate(rows, 1):
        row["rank"] = index
        row["selection"] = "PRIMARY" if index == 1 else ("BACKUP" if index == 2 else None)
    return rows


def _write(name: str, value: Any) -> None:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    (RESULTS_DIR / name).write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8"
    )


def run_u2() -> dict[str, Any]:
    started = time.perf_counter()
    config = _load_yaml(CONFIG_PATH)
    aircraft_root = Path(jsbsim.get_default_root_dir()) / "aircraft"
    inventory = _inventory(config, aircraft_root)
    provenance = _provenance(config, inventory, aircraft_root)
    kept = [row["model"] for row in inventory if row["static_decision"] == "KEEP"]
    configured = set(config["dynamic_candidates"])
    unconfigured_kept = sorted(set(kept) - configured)
    missing_candidates = sorted(configured - {row["model"] for row in inventory})

    all_runs: list[dict[str, Any]] = []
    all_points: list[dict[str, Any]] = []
    model_results: list[dict[str, Any]] = []
    points_by_model: dict[str, list[dict[str, Any]]] = {}
    speed_passers: list[str] = []
    climb_passers: list[str] = []
    turn_passers: list[str] = []

    for model in kept:
        model_started = time.perf_counter()
        result = {"model": model, "dynamic_status": "NOT_RUN", "failed_gate": None, "point_ids": []}
        if model not in configured:
            result.update({"dynamic_status": "UNKNOWN_MISSING_FROZEN_FIXTURE", "failed_gate": "interface"})
            result["runtime_s"] = time.perf_counter() - model_started
            model_results.append(result)
            continue
        run_config = _run_config(model, config)
        points_by_model[model] = []
        stop = False
        for gate in ("speed", "climb", "turn"):
            if stop:
                break
            for request in config["gates"][gate]:
                point_started = time.perf_counter()
                records, point = u1._run_point(
                    model,
                    request["maneuver"],
                    float(request["ias_mps"]),
                    float(request["target_value"]),
                    run_config,
                    provenance["provenance_id"],
                )
                for record in records:
                    record["run_id"] = record["run_id"].replace("u1-", "u2-", 1)
                point["run_ids"] = [run_id.replace("u1-", "u2-", 1) for run_id in point["run_ids"]]
                point["aircraft_model"] = model
                point["gate"] = gate
                point["runtime_s"] = time.perf_counter() - point_started
                point["fixture_failure"] = _point_has_fixture_failure(point, records)
                all_runs.extend(records)
                all_points.append(point)
                points_by_model[model].append(point)
                result["point_ids"].append(point["run_ids"])
                if point["fixture_failure"]:
                    result.update({"dynamic_status": "UNKNOWN_INTERFACE_OR_FIXTURE", "failed_gate": gate})
                    stop = True
                    break
                if point["status"] != "VALID":
                    result.update({"dynamic_status": f"REJECT_DYNAMIC_{gate.upper()}", "failed_gate": gate})
                    stop = True
                    break
            if stop:
                break
            if gate == "speed":
                speed_passers.append(model)
            elif gate == "climb":
                climb_passers.append(model)
            elif gate == "turn":
                turn_passers.append(model)
                result["dynamic_status"] = "PASS_ALL_GATES"
        result["runtime_s"] = time.perf_counter() - model_started
        model_results.append(result)

    ranking = _rank(turn_passers, points_by_model, config)
    selected = [item for item in ranking if item["rank"] <= config["ranking"]["maximum_selected"]]
    screening_unknown = bool(unconfigured_kept or missing_candidates) or any(
        item["dynamic_status"].startswith("UNKNOWN_") for item in model_results
    )
    step_status = "PARTIAL" if screening_unknown else "PASS"
    current_status = (
        f"{selected[0]['model']} SELECTED" if selected else
        ("SCREENING INCONCLUSIVE" if screening_unknown else "NO SUITABLE STOCK JSBSIM MODEL")
    )
    elapsed = time.perf_counter() - started
    result = {
        "step": "U2",
        "step_status": step_status,
        "step_pass": step_status == "PASS",
        "current_aircraft_status": current_status,
        "no_suitable_stock_model": not selected and not screening_unknown,
        "selected": selected,
        "ranking": ranking,
        "counts": {
            "inventory": len(inventory),
            "static_rejected": sum(row["static_decision"] == "REJECT_STATIC" for row in inventory),
            "static_kept": len(kept),
            "dynamically_tested": sum(bool(item["point_ids"]) for item in model_results),
            "speed_gate_passed": len(speed_passers),
            "climb_gate_passed": len(climb_passers),
            "turn_gate_passed": len(turn_passers),
            "executed_points": len(all_points),
            "executed_cold_start_runs": len(all_runs),
        },
        "gate_passers": {"speed": speed_passers, "climb": climb_passers, "turn": turn_passers},
        "model_results": model_results,
        "unconfigured_kept_models": unconfigured_kept,
        "missing_configured_candidates": missing_candidates,
        "runtime_s": elapsed,
        "controller_tuning_performed": False,
        "aircraft_xml_modified": False,
        "planner_modified": False,
        "next_stage_started": False,
        "full_characterization_performed": False,
        "profile_changed": False,
        "interface_adaptations": [
            "U1 scalar trim throttle broadcast to every engine",
            "per-candidate piston engine-rpm or turbine n1 state property",
            "per-candidate tank and pointmass property lists",
            "retractable-aircraft gear command/position initialized clean",
            "post-trim engine-start command reasserted for stock models whose failed trim stops propulsion",
            "dr1 missing FlightGear pushback input properties initialized to inactive zero values",
            "pa28 stock magneto/starter startup and radian-to-normalized surface telemetry mapping",
            "L410 nonstandard and direct-thrust OV10 inapplicable propeller-rpm checks omitted while n1 remains checked",
        ],
    }
    _write("u2_inventory.json", inventory)
    _write("u2_provenance.json", provenance)
    _write("u2_runs.json", all_runs)
    _write("u2_points.json", [_point_summary(point) | {"aircraft_model": point["aircraft_model"], "gate": point["gate"], "runtime_s": point["runtime_s"], "fixture_failure": point["fixture_failure"]} for point in all_points])
    _write("u2_result.json", result)
    return result


if __name__ == "__main__":
    print(json.dumps(run_u2(), indent=2, allow_nan=False))
