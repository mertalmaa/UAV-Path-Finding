"""U3 aircraft-driven JSBSim stock-aircraft selection.

This is a cheap comparison, not an envelope or LUT sweep. It reuses the U2
inventory, U1 controller, U2 property adapters, and three-repeat methodology.
"""

from __future__ import annotations

import copy
import hashlib
import importlib.metadata
import json
import math
import platform
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from statistics import fmean
from typing import Any

import jsbsim
import yaml

import u2_candidate_screening as u2


HERE = Path(__file__).resolve().parent
CONFIG_PATH = HERE / "u3_aircraft_selection_configuration.yaml"
U1_CONFIG_PATH = HERE / "u1_profile_validation_configuration.yaml"
U2_CONFIG_PATH = HERE / "u2_candidate_screening_configuration.yaml"
U2_INVENTORY_PATH = HERE / "results" / "u2_inventory.json"
RESULTS_DIR = HERE / "results"
_U2_SNAPSHOT = u2.u1._snapshot


def _u3_snapshot(fdm: Any, elapsed_s: float, config: dict[str, Any]) -> dict[str, float]:
    """Preserve U2 adapters and normalize c172x's radian-only surface outputs."""
    sample = _U2_SNAPSHOT(fdm, elapsed_s, config)
    limits = config["fixture"].get("surface_max_radians")
    if limits:
        sample["elevator_pos_norm"] = fdm["fcs/elevator-pos-rad"] / limits["elevator"]
        sample["left_aileron_pos_norm"] = fdm["fcs/left-aileron-pos-rad"] / limits["left_aileron"]
        sample["right_aileron_pos_norm"] = fdm["fcs/right-aileron-pos-rad"] / limits["right_aileron"]
        sample["rudder_pos_norm"] = fdm["fcs/rudder-pos-rad"] / limits["rudder"]
    return sample


u2.u1._snapshot = _u3_snapshot


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


def _candidate_fixture(model: str, fixture_source: str) -> dict[str, Any]:
    u1_config = _load_yaml(U1_CONFIG_PATH)
    u2_config = _load_yaml(U2_CONFIG_PATH)
    common = copy.deepcopy(u2_config["fixture_common"])
    if fixture_source == "u2":
        fixture = common | copy.deepcopy(u2_config["dynamic_candidates"][model])
        if model == "c172x":
            fixture["surface_max_radians"] = {
                "elevator": 0.34,
                "left_aileron": 0.35,
                "right_aileron": 0.35,
                "rudder": math.radians(16.0),
            }
        return fixture
    if fixture_source == "u1_primary":
        return copy.deepcopy(u1_config["fixture"])
    if fixture_source == "u1_backup":
        fixture = copy.deepcopy(u1_config["fixture"])
        fixture.update(copy.deepcopy(u1_config["backup_fixture"]))
        fixture["gear_configuration"] = "fixed_deployed"
        return fixture
    raise ValueError(f"unknown fixture source: {fixture_source}")


def _run_config(model: str, candidate: dict[str, Any], config: dict[str, Any]) -> dict[str, Any]:
    run_config = _load_yaml(U1_CONFIG_PATH)
    run_config["configuration_id"] = config["configuration_id"]
    run_config["run_prefix"] = "u3"
    run_config["aircraft"]["backup_model"] = "__u3_no_backup__"
    run_config["fixture"] = _candidate_fixture(model, candidate["fixture_source"])
    run_config["simulation"] = copy.deepcopy(config["simulation"])
    run_config["controller"]["policy"] = config["controller"]["policy"]
    return run_config


def _resolve_reference(root: Path, aircraft_dir: Path, kind: str, name: str) -> Path | None:
    suffix = "" if name.lower().endswith(".xml") else ".xml"
    filename = name + suffix
    folders = {
        "system": [aircraft_dir / "Systems", aircraft_dir, root / "systems"],
        "autopilot": [aircraft_dir, aircraft_dir / "Systems", root / "systems"],
        "engine": [aircraft_dir / "Engines", root / "engine"],
        "thruster": [aircraft_dir / "Engines", root / "engine"],
    }[kind]
    for folder in folders:
        path = folder / filename
        if path.is_file():
            return path.resolve()
    return None


def _dependency_closure(model: str, root: Path) -> list[dict[str, str]]:
    aircraft_dir = root / "aircraft" / model
    pending = [(aircraft_dir / f"{model}.xml").resolve()]
    seen: set[Path] = set()
    missing: set[str] = set()
    while pending:
        path = pending.pop()
        if path in seen:
            continue
        seen.add(path)
        tree = ET.parse(path)
        for kind in ("system", "autopilot", "engine", "thruster"):
            for node in tree.findall(f".//{kind}[@file]"):
                name = node.attrib["file"]
                resolved = _resolve_reference(root, aircraft_dir, kind, name)
                if resolved is None:
                    missing.add(f"{kind}:{name}")
                elif resolved not in seen:
                    pending.append(resolved)
    rows = [
        {"path": str(path), "sha256": _sha256(path)}
        for path in sorted(seen, key=lambda item: str(item).lower())
    ]
    rows.extend({"missing_reference": item} for item in sorted(missing))
    return rows


def _component_names(path: Path) -> set[str]:
    root = ET.parse(path).getroot()
    return {
        (node.attrib.get("name") or "").lower()
        for node in root.iter()
        if node.tag in {"pid", "summer", "switch", "pure_gain", "scheduled_gain", "lag_filter"}
    }


def _audit(model: str, root: Path) -> dict[str, Any]:
    aircraft_dir = root / "aircraft" / model
    main = aircraft_dir / f"{model}.xml"
    xml_root = ET.parse(main).getroot()
    flight_control = xml_root.find("./flight_control")
    systems = [node.attrib["file"] for node in xml_root.findall("./system[@file]")]
    autopilots = [node.attrib["file"] for node in xml_root.findall("./autopilot[@file]")]
    component_names: set[str] = set()
    for item in _dependency_closure(model, root):
        path_value = item.get("path")
        if path_value:
            component_names.update(_component_names(Path(path_value)))
    conventional = any(
        token in " ".join(component_names)
        for token in ("pitch trim sum", "roll trim sum", "yaw trim sum")
    ) or model == "c172x"
    c172x = model == "c172x"
    return {
        "model": model,
        "main_xml": str(main.resolve()),
        "main_xml_sha256": _sha256(main),
        "flight_control_declared": flight_control is not None or bool(systems),
        "flight_control_name": flight_control.attrib.get("name") if flight_control is not None else None,
        "system_files": systems,
        "autopilot_files": autopilots,
        "native_fcs_type": (
            "conventional_command_to_actuator_mapping_plus_bundled_autopilots"
            if c172x
            else "conventional_command_to_surface_mapping"
        ),
        "accepted_command_level": (
            "normalized pilot commands; bundled heading/roll and altitude autopilots also exist"
            if c172x
            else "normalized elevator/aileron/rudder/throttle commands"
        ),
        "roll_stabilization": c172x,
        "pitch_stabilization": c172x,
        "yaw_or_rudder_coordination": False,
        "throttle_or_speed_control": False,
        "altitude_or_vertical_control": c172x,
        "native_high_level_control": c172x,
        "direct_surface_forcing_required": False,
        "outer_loop_requirement": (
            "minimal bank/Vz/IAS/beta outer loop retained; bundled autopilots disabled because no direct sustained-bank interface and no implemented speed channel"
            if c172x
            else "minimal bank/Vz/IAS/beta outer loop required above normalized stock FCS inputs"
        ),
        "conventional_mapping_detected": conventional,
        "dependency_closure": _dependency_closure(model, root),
    }


def _point_summary(point: dict[str, Any]) -> dict[str, Any]:
    return {
        "maneuver": point["maneuver"],
        "requested": point["requested"],
        "status": point["status"],
        "run_statuses": point["run_statuses"],
        "run_status_reasons": point["run_status_reasons"],
        "repeatability": point["repeatability"],
        "model_fixture_mismatch_evidence": point["model_fixture_mismatch_evidence"],
        "measured": point["measured"],
    }


def _finite_point(point: dict[str, Any]) -> bool:
    measured = point.get("measured")
    return measured is not None and all(
        not isinstance(value, float) or math.isfinite(value) for value in measured.values()
    )


def _mean_tracking_clean(point: dict[str, Any]) -> bool:
    measured = point["measured"]
    requested = point["requested"]
    ias_ok = abs(measured["ias_mps"] - requested["ias_mps"]) / requested["ias_mps"] <= 0.05
    bank_ok = abs(measured["roll_deg"] - requested["bank_deg"]) <= 3.0
    vz_limit = 0.75 if point["maneuver"] == "straight" else 0.8
    vz_ok = abs(measured["vertical_speed_mps"] - requested["vertical_speed_mps"]) <= vz_limit
    return ias_ok and bank_ok and vz_ok


def _candidate_metrics(
    model: str,
    points: list[dict[str, Any]],
    runs: list[dict[str, Any]],
    nominal_ias: float,
) -> dict[str, Any]:
    point_by_maneuver: dict[str, list[dict[str, Any]]] = {}
    for point in points:
        point_by_maneuver.setdefault(point["maneuver"], []).append(point)
    clean = [
        _finite_point(point)
        and point["repeatability"]["passed"]
        and point["measured"]["stall_indicator"] <= 0.0
        and point["measured"]["maximum_surface_usage_norm"] < 0.9
        and _mean_tracking_clean(point)
        for point in points
    ]
    run_saturations = [
        run.get("controller", {}).get("saturation_in_measurement", {}) for run in runs
    ]
    saturation_fraction = (
        sum(any(item.values()) for item in run_saturations) / len(run_saturations)
        if run_saturations else 1.0
    )
    turn = point_by_maneuver["turn"][0]
    vertical = point_by_maneuver["vertical"]
    all_repeatable = all(point["repeatability"]["passed"] for point in points)
    numerical_stability = all(_finite_point(point) for point in points)
    strict_valid_count = sum(point["status"] == "VALID" for point in points)
    return {
        "all_four_logical_tests_clean": all(clean),
        "clean_test_count": sum(clean),
        "test_count": len(points),
        "all_repeatable": all_repeatable,
        "numerical_stability": numerical_stability,
        "strict_u1_valid_count": strict_valid_count,
        "measurement_saturation_run_fraction": saturation_fraction,
        "maximum_surface_usage_norm": max(point["measured"]["maximum_surface_usage_norm"] for point in points if point.get("measured")),
        "nominal_speed_distance_from_preferred_band_mps": max(35.0 - nominal_ias, 0.0, nominal_ias - 50.0),
        "turn_rate_deg_s": turn["measured"]["turn_rate_deg_s"],
        "turn_radius_m": turn["measured"]["measured_radius_m"],
        "turn_rate_cv": fmean(
            run["trajectory"]["turn_rate_coefficient_of_variation"]
            for run in runs if run["maneuver"] == "turn" and run.get("trajectory")
        ),
        "vertical_tracking_mean_abs_error_mps": fmean(
            abs(item["measured"]["vertical_speed_mps"] - item["requested"]["vertical_speed_mps"])
            for item in vertical
        ),
    }


def _score(model: str, metrics: dict[str, Any], audit: dict[str, Any]) -> dict[str, int]:
    clean = metrics["clean_test_count"]
    sat = metrics["measurement_saturation_run_fraction"]
    strict_valid = metrics["strict_u1_valid_count"]
    control = (
        5 if clean == 4 and strict_valid >= 3 and sat == 0.0 else
        4 if clean == 4 and strict_valid >= 2 and sat == 0.0 else
        3 if clean == 4 and sat == 0.0 else
        2 if clean >= 3 else
        1 if metrics["numerical_stability"] else 0
    )
    speed = 5 if metrics["nominal_speed_distance_from_preferred_band_mps"] == 0.0 else 3
    turn = 5 if metrics["turn_rate_cv"] <= 0.01 and clean >= 2 else (4 if metrics["turn_rate_cv"] <= 0.05 else (3 if metrics["numerical_stability"] else 0))
    vertical = 5 if metrics["vertical_tracking_mean_abs_error_mps"] <= 0.4 else (4 if metrics["vertical_tracking_mean_abs_error_mps"] <= 0.8 else 2)
    tuning = 5 if clean == metrics["test_count"] and sat == 0.0 else (4 if clean >= 3 else 2)
    simplicity = {"c172r": 5, "c172p": 5, "J3Cub": 5, "c182": 5, "c172x": 4, "DHC6": 3}[model]
    repeatability = 5 if metrics["all_repeatable"] else 1
    return {
        "control_quality": control,
        "speed_proximity": speed,
        "turn_quality": turn,
        "vertical_quality": vertical,
        "tuning_burden": tuning,
        "model_simplicity": simplicity,
        "repeatability": repeatability,
    }


def _ranking_key(row: dict[str, Any], priority: list[str]) -> tuple[Any, ...]:
    scores = row["scores"]
    metrics = row["metrics"]
    return tuple(
        [-scores[name] for name in priority]
        + [
            -metrics["strict_u1_valid_count"],
            metrics["measurement_saturation_run_fraction"],
            metrics["turn_rate_cv"],
            metrics["vertical_tracking_mean_abs_error_mps"],
            metrics["maximum_surface_usage_norm"],
            row["model"].lower(),
        ]
    )


def run_u3() -> dict[str, Any]:
    started = time.perf_counter()
    config = _load_yaml(CONFIG_PATH)
    inventory = json.loads(U2_INVENTORY_PATH.read_text(encoding="utf-8"))
    inventory_models = {row["model"] for row in inventory}
    candidates = config["candidates"]
    if len(candidates) > config["selection_policy"]["maximum_serious_candidates"]:
        raise ValueError("serious candidate cap exceeded")
    if missing := set(candidates) - inventory_models:
        raise ValueError(f"candidate absent from reused U2 inventory: {sorted(missing)}")

    jsbsim_root = Path(jsbsim.get_default_root_dir()).resolve()
    audits = {model: _audit(model, jsbsim_root) for model in candidates}
    all_runs: list[dict[str, Any]] = []
    all_points: list[dict[str, Any]] = []
    rows: list[dict[str, Any]] = []

    provenance_seed = {
        "configuration": _sha256(CONFIG_PATH),
        "harness": _sha256(Path(__file__).resolve()),
        "u2_inventory": _sha256(U2_INVENTORY_PATH),
        "aircraft": {model: audit["main_xml_sha256"] for model, audit in audits.items()},
    }
    provenance_id = "u3-" + hashlib.sha256(
        json.dumps(provenance_seed, sort_keys=True).encode("utf-8")
    ).hexdigest()[:20]

    for model, candidate in candidates.items():
        run_config = _run_config(model, candidate, config)
        requests = [
            ("straight", candidate["nominal_ias_mps"], 0.0),
            ("turn", candidate["nominal_ias_mps"], candidate["bank_deg"]),
            ("vertical", candidate["nominal_ias_mps"], candidate["climb_mps"]),
            ("vertical", candidate["nominal_ias_mps"], candidate["descent_mps"]),
        ]
        model_runs: list[dict[str, Any]] = []
        model_points: list[dict[str, Any]] = []
        for maneuver, speed, target in requests:
            records, point = u2.u1._run_point(
                model, maneuver, float(speed), float(target), run_config, provenance_id
            )
            for record in records:
                record["run_id"] = record["run_id"].replace("u1-", "u3-", 1)
            point["run_ids"] = [item.replace("u1-", "u3-", 1) for item in point["run_ids"]]
            point["aircraft_model"] = model
            model_runs.extend(records)
            model_points.append(point)
        metrics = _candidate_metrics(
            model, model_points, model_runs, float(candidate["nominal_ias_mps"])
        )
        rows.append(
            {
                "model": model,
                "working_point": copy.deepcopy(candidate),
                "metrics": metrics,
                "scores": _score(model, metrics, audits[model]),
            }
        )
        all_runs.extend(model_runs)
        all_points.extend(model_points)

    priority = config["ranking"]["priority_order"]
    rows.sort(key=lambda row: _ranking_key(row, priority))
    for rank, row in enumerate(rows, 1):
        row["rank"] = rank
        row["selection"] = "PRIMARY" if rank == 1 else ("BACKUP" if rank == 2 else None)

    clean_rows = [row for row in rows if row["metrics"]["all_four_logical_tests_clean"]]
    step_status = "PASS" if len(clean_rows) >= 1 else "PARTIAL"
    selected = rows[:2] if clean_rows else []
    result = {
        "step": "U3",
        "step_status": step_status,
        "inventory_reused": str(U2_INVENTORY_PATH.resolve()),
        "inventory_count": len(inventory),
        "serious_candidate_count": len(candidates),
        "executed_points": len(all_points),
        "executed_cold_start_runs": len(all_runs),
        "ranking": rows,
        "primary": selected[0]["model"] if selected else None,
        "backup": selected[1]["model"] if len(selected) > 1 else None,
        "selected_stack": config["controller"]["selected_stack_contract"] if selected else None,
        "same_stack_lut_and_final_replay_required": True,
        "full_lut_started": False,
        "final_replay_implemented": False,
        "planner_modified": False,
        "aircraft_xml_modified": False,
        "controller_gain_tuning_performed": False,
        "runtime_s": time.perf_counter() - started,
    }
    provenance = {
        "provenance_id": provenance_id,
        "jsbsim_python_package_version": importlib.metadata.version("jsbsim"),
        "jsbsim_compiled_version": jsbsim.FGFDMExec(None).get_version(),
        "jsbsim_default_root": str(jsbsim_root),
        "python_version": sys.version,
        "python_executable": sys.executable,
        "platform": platform.platform(),
        "files": provenance_seed,
    }
    _write("u3_control_audit.json", list(audits.values()))
    _write("u3_provenance.json", provenance)
    _write("u3_runs.json", all_runs)
    _write("u3_points.json", [_point_summary(point) | {"aircraft_model": point["aircraft_model"]} for point in all_points])
    _write("u3_result.json", result)
    return result


if __name__ == "__main__":
    print(json.dumps(run_u3(), indent=2, allow_nan=False))
