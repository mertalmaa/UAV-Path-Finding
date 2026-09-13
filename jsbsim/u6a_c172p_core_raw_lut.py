from __future__ import annotations

import hashlib
import json
import math
import platform
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from statistics import fmean
from typing import Any

import jsbsim
import yaml


HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
CONFIG_PATH = HERE / "u6a_c172p_core_raw_lut_configuration.yaml"
HARNESS_PATH = Path(__file__).resolve()
sys.path.insert(0, str(HERE))

import u3_aircraft_selection as u3
import u5_practical_aircraft_rescreen as u5


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write(name: str, payload: dict[str, Any]) -> None:
    RESULTS.mkdir(parents=True, exist_ok=True)
    (RESULTS / name).write_text(
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False),
        encoding="utf-8",
    )


def _git_commit() -> str | None:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=HERE.parent, text=True
        ).strip()
    except Exception:
        return None


def _point_key(point: dict[str, Any]) -> tuple[Any, ...]:
    requested = point["requested"]
    return (
        point["aircraft_model"],
        float(point["altitude_msl_m"]),
        float(requested["ias_mps"]),
        point["maneuver"],
        float(requested["bank_deg"]),
        float(requested["vertical_speed_mps"]),
    )


def _analysis_key(row: dict[str, Any]) -> tuple[Any, ...]:
    maneuver = row["maneuver"]
    return (
        row["model"],
        float(row["altitude_m"]),
        float(row["target_ias_mps"]),
        maneuver,
        float(row["target_value"] if maneuver == "turn" else 0.0),
        float(row["target_value"] if maneuver == "vertical" else 0.0),
    )


def _source_point_id(point: dict[str, Any]) -> str:
    first = point["run_ids"][0]
    return first[:-3] if first.endswith("-r1") else first


def _remap_new_ids(
    records: list[dict[str, Any]], point: dict[str, Any], analysis: dict[str, Any]
) -> None:
    mapping = {}
    for record in records:
        old = record["run_id"]
        new = old.replace("u5-", "u6a-", 1)
        record["run_id"] = new
        mapping[old] = new
    point["run_ids"] = [mapping[value] for value in point["run_ids"]]
    analysis["run_ids"] = list(point["run_ids"])


def _failure_classification(records: list[dict[str, Any]]) -> dict[str, Any]:
    reasons = sorted({reason for record in records for reason in record["status_reasons"]})
    categories = set()
    if any(record.get("aircraft_limit_evidence", {}).get("power_limited", False) for record in records):
        categories.add("power_limited")
    if any(record.get("aircraft_limit_evidence", {}).get("stall_indicator", False) for record in records):
        categories.add("aero_stability")
    if "ias_tracking" in reasons:
        categories.add("speed_retention_failure")
    if "bank_tracking" in reasons or "vertical_speed_tracking" in reasons:
        categories.add("tracking_acceptance_limited")
    if "outer_loop_not_clipped_in_measurement" in reasons:
        categories.update(("controller_limited", "saturation"))
    if any(reason in reasons for reason in ("turn_rate_stable", "beta_bounded", "surface_usage_bounded")):
        categories.add("aero_stability")
    if any(":" in reason for reason in reasons):
        categories.add("diagnostic")
    if reasons and not categories:
        categories.add("other")
    return {"categories": sorted(categories), "raw_reasons": reasons}


def _metric(records: list[dict[str, Any]], name: str) -> float:
    return fmean(record["measurement"][name]["mean"] for record in records)


def _canonical_row(
    family: str,
    altitude_m: float,
    target: float,
    point: dict[str, Any],
    records: list[dict[str, Any]],
    analysis: dict[str, Any],
    source_stage: str,
    derived_zero_target: bool,
) -> dict[str, Any]:
    measured = point["measured"]
    common = {
        "altitude_m": altitude_m,
        "nominal_ias_mps": 40.0,
        "status": point["status"],
        "actual_ias_mps": measured["ias_mps"],
        "tas_mps": measured["tas_mps"],
        "actual_vz_mps": measured["vertical_speed_mps"],
        "gamma_deg": measured["gamma_deg"],
        "pitch_deg": _metric(records, "pitch_deg"),
        "aoa_deg": measured["alpha_deg"],
        "beta_deg": measured["beta_deg"],
        "throttle": measured["engine_0_throttle_pos_norm"],
        "rpm": _metric(records, "engine_0_propeller_rpm"),
        "power_hp": _metric(records, "engine_0_power_hp"),
        "thrust_lbs": _metric(records, "engine_0_thrust_lbs"),
        "controller_outputs": {
            "elevator_cmd_norm": measured["elevator_cmd_norm"],
            "aileron_cmd_norm": measured["aileron_cmd_norm"],
            "rudder_cmd_norm": measured["rudder_cmd_norm"],
        },
        "maximum_surface_usage_norm": measured["maximum_surface_usage_norm"],
        "saturation": analysis["controller_saturation_in_measurement"],
        "settled": all(item["settled_by_screening_deadline"] for item in analysis["settling"]),
        "settling": analysis["settling"],
        "repeatable": point["repeatability"]["passed"],
        "actual_stable_response": analysis["actual_stable_response"],
        "failure_reason": _failure_classification(records),
        "metadata": {
            "source_stage": source_stage,
            "source_point_id": _source_point_id(point),
            "source_provenance": point["provenance_id"],
            "source_run_ids": point["run_ids"],
            "execution": "NEW_U6A" if source_stage == "U6A" else "REUSED",
            "derived_zero_target_equivalent": derived_zero_target,
            "cold_start_repetitions": len(point["run_ids"]),
        },
    }
    if family == "straight":
        common["altitude_error_m"] = measured["altitude_msl_m"] - altitude_m
        return common
    if family == "turn":
        common.update(
            {
                "target_bank_deg": target,
                "direction": "LEFT" if target < 0 else "RIGHT" if target > 0 else "STRAIGHT",
                "actual_bank_deg": measured["roll_deg"],
                "turn_rate_deg_s": 0.0 if target == 0 else measured["turn_rate_deg_s"],
                "turn_radius_m": None if target == 0 else measured["measured_radius_m"],
                "theoretical_radius_m": None if target == 0 else measured["theoretical_radius_m"],
                "radius_theory_relative_difference": None if target == 0 else measured["radius_theory_relative_difference"],
                "nz": measured["nz"],
                "vz_mps": measured["vertical_speed_mps"],
            }
        )
        return common
    common["target_vz_mps"] = target
    return common


def _first_limitation(rows: list[dict[str, Any]], direction: str) -> dict[str, Any] | None:
    ordered = (
        sorted((r for r in rows if r["target_vz_mps"] > 0), key=lambda r: r["target_vz_mps"])
        if direction == "climb"
        else sorted((r for r in rows if r["target_vz_mps"] < 0), key=lambda r: abs(r["target_vz_mps"]))
    )
    item = next((row for row in ordered if row["status"] != "VALID"), None)
    if item is None:
        return None
    return {
        "target_vz_mps": item["target_vz_mps"],
        "status": item["status"],
        "actual_vz_mps": item["actual_vz_mps"],
        "failure_reason": item["failure_reason"],
    }


def run_u6a() -> dict[str, Any]:
    started = time.perf_counter()
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    u5_config = yaml.safe_load(u5.CONFIG_PATH.read_text(encoding="utf-8"))
    u3_config = yaml.safe_load(u5.U3_CONFIG_PATH.read_text(encoding="utf-8"))
    source_paths = {
        name: HERE / item["path"] for name, item in config["source_artifacts"].items()
    }
    source_before = {name: _sha256(path) for name, path in source_paths.items()}
    expected = {name: item["sha256"] for name, item in config["source_artifacts"].items()}
    if source_before != expected:
        raise RuntimeError("historical U5/U5.2 source hash mismatch before U6A")
    u5_points = _load(source_paths["u5_points"])
    u5_runs = _load(source_paths["u5_runs"])
    u5_provenance = _load(source_paths["u5_provenance"])
    u5_result = _load(source_paths["u5_result"])
    u5_2_points = _load(source_paths["u5_2_points"])
    u5_2_runs = _load(source_paths["u5_2_runs"])
    u5_2_provenance = _load(source_paths["u5_2_provenance"])
    u5_2_result = _load(source_paths["u5_2_result"])
    if u5_result["provenance_id"] != config["parent_u5_provenance_id"]:
        raise RuntimeError("parent U5 provenance mismatch")
    if u5_2_result["provenance_id"] != config["parent_u5_2_provenance_id"]:
        raise RuntimeError("parent U5.2 provenance mismatch")

    files = {
        "configuration": {"path": str(CONFIG_PATH), "sha256": _sha256(CONFIG_PATH)},
        "harness": {"path": str(HARNESS_PATH), "sha256": _sha256(HARNESS_PATH)},
        "u5_configuration": u5_provenance["files"]["configuration"],
        "u5_harness": u5_provenance["files"]["harness"],
        "u1_configuration": u5_provenance["files"]["u1_configuration"],
        "u1_harness": u5_provenance["files"]["u1_harness"],
        "u2_configuration": u5_provenance["files"]["u2_configuration"],
        "u3_configuration": u5_provenance["files"]["u3_configuration"],
    }
    seed = {
        "files": {name: item["sha256"] for name, item in files.items()},
        "sources": source_before,
        "stack": config["frozen_stack"],
        "grid": {
            "altitude": config["altitude_grid_m"],
            "turn": config["turn_bank_targets_deg"],
            "vertical": config["vertical_speed_targets_mps"],
        },
    }
    provenance_id = "u6a-" + hashlib.sha256(
        json.dumps(seed, sort_keys=True).encode("utf-8")
    ).hexdigest()[:20]

    source_points: dict[tuple[Any, ...], tuple[dict[str, Any], str]] = {}
    source_runs = {row["run_id"]: row for row in u5_runs["runs"] + u5_2_runs["runs"]}
    for point in u5_points["points"]:
        if point["aircraft_model"] == "c172p":
            source_points[_point_key(point)] = (point, "U5")
    for point in u5_2_points["points"]:
        if point["aircraft_model"] == "c172p" and point["maneuver"] != "combined":
            source_points[_point_key(point)] = (point, "U5.2")

    u5_phase1 = _load(RESULTS / "u5_phase1_speed_screen.json")
    u5_high = _load(RESULTS / "u5_high_altitude_screen.json")
    u5_2_straight = _load(RESULTS / "u5_2_new_straight_coverage.json")
    u5_2_turn = _load(RESULTS / "u5_2_new_level_turn_coverage.json")
    source_analyses = {
        _analysis_key(row): row
        for row in u5_phase1["rows"] + u5_high["rows"]
        + u5_2_straight["rows"] + u5_2_turn["rows"]
        if row["model"] == "c172p"
    }
    candidate = u5_config["candidates"]["c172p"]
    new_runs: list[dict[str, Any]] = []
    new_points: list[dict[str, Any]] = []
    new_analyses: dict[tuple[Any, ...], dict[str, Any]] = {}
    straight_table = []
    turn_table = []
    vertical_table = []
    reused_source_ids = set()

    def obtain(
        altitude: float, maneuver: str, target: float
    ) -> tuple[dict[str, Any], list[dict[str, Any]], dict[str, Any], str]:
        bank = target if maneuver == "turn" else 0.0
        vz = target if maneuver == "vertical" else 0.0
        key = ("c172p", altitude, 40.0, maneuver, bank, vz)
        if key in source_points:
            point, stage = source_points[key]
            records = [source_runs[run_id] for run_id in point["run_ids"]]
            analysis = source_analyses[key]
            reused_source_ids.add((stage, _source_point_id(point)))
            return point, records, analysis, stage
        records, point, analysis = u5._run_point(
            "c172p", candidate, altitude, 40.0, maneuver, target,
            u5_config, u3_config, provenance_id,
        )
        _remap_new_ids(records, point, analysis)
        new_runs.extend(records)
        new_points.append(point)
        new_analyses[key] = analysis
        source_points[key] = (point, "U6A")
        source_runs.update({record["run_id"]: record for record in records})
        return point, records, analysis, "U6A"

    u5._install_hooks()
    try:
        for altitude_value in config["altitude_grid_m"]:
            altitude = float(altitude_value)
            point, records, analysis, stage = obtain(altitude, "straight", 0.0)
            straight_table.append(
                _canonical_row("straight", altitude, 0.0, point, records, analysis, stage, False)
            )
            if not analysis["actual_stable_response"]:
                continue
            for target_value in config["turn_bank_targets_deg"]:
                target = float(target_value)
                if target == 0.0:
                    turn_table.append(
                        _canonical_row("turn", altitude, target, point, records, analysis, stage, True)
                    )
                else:
                    p, rr, aa, ss = obtain(altitude, "turn", target)
                    turn_table.append(_canonical_row("turn", altitude, target, p, rr, aa, ss, False))
            for target_value in config["vertical_speed_targets_mps"]:
                target = float(target_value)
                if target == 0.0:
                    vertical_table.append(
                        _canonical_row("vertical", altitude, target, point, records, analysis, stage, True)
                    )
                else:
                    p, rr, aa, ss = obtain(altitude, "vertical", target)
                    vertical_table.append(
                        _canonical_row("vertical", altitude, target, p, rr, aa, ss, False)
                    )
    finally:
        u5._restore_hooks()

    expected_new = config["reuse_contract"]["expected_new_unique_points"]
    expected_runs = config["reuse_contract"]["expected_new_cold_start_runs"]
    if len(new_points) != expected_new or len(new_runs) != expected_runs:
        raise RuntimeError(f"unexpected U6A execution count {len(new_points)}/{len(new_runs)}")
    if len({row["run_id"] for row in new_runs}) != len(new_runs):
        raise RuntimeError("duplicate U6A run IDs")

    root = Path(jsbsim.get_default_root_dir()).resolve()
    aircraft_audit = u3._audit("c172p", root)
    run_config = u5._run_config("c172p", candidate, 0.0, u5_config, u3_config)
    sample_record = next(
        record for record in source_runs.values()
        if record["aircraft_model"] == "c172p"
    )
    fixture_observed = {
        "weight_lbs": _metric([sample_record], "weight_lbs"),
        "total_fuel_lbs": _metric([sample_record], "total_fuel_lbs"),
        "cg_x_in": _metric([sample_record], "cg_x_in"),
        "cg_y_in": _metric([sample_record], "cg_y_in"),
        "cg_z_in": _metric([sample_record], "cg_z_in"),
    }
    reuse_audit = {
        "schema_version": 1,
        "artifact_type": "U6A_SAME_STACK_REUSE_AUDIT",
        "provenance_id": provenance_id,
        "equivalent": True,
        "aircraft_id": "c172p",
        "nominal_ias_mps": 40.0,
        "controller_policy": run_config["controller"]["policy"],
        "controller_configuration": run_config["controller"],
        "stock_fcs_identity": aircraft_audit["flight_control_name"],
        "mixture_policy_id": config["frozen_stack"]["mixture_policy_id"],
        "mixture_formula": config["frozen_stack"]["mixture_formula"],
        "fixture_configuration": run_config["fixture"],
        "fixture_observed": fixture_observed,
        "atmosphere_configuration": {
            key: run_config["fixture"][key]
            for key in (
                "atmosphere", "delta_temperature_R", "wind_north_fps",
                "wind_east_fps", "wind_down_fps", "gust_north_fps",
                "gust_east_fps", "gust_down_fps", "turbulence_type",
            )
        },
        "simulation": run_config["simulation"],
        "initialization_modes_observed": sorted(
            {record["initialization_mode"] for record in source_runs.values() if record["aircraft_model"] == "c172p" and "initialization_mode" in record}
        ),
        "measurement_contract": {
            "ias": "velocities/vc-kts * 0.5144444444444445",
            "tas": "velocities/vt-fps * 0.3048",
            "steady_window_s": run_config["simulation"]["steady_measurement_window_s"],
            "acceptance": run_config["acceptance"],
        },
        "reused_unique_source_point_count": len(reused_source_ids),
        "reused_sources": [
            {"source_stage": stage, "source_point_id": point_id}
            for stage, point_id in sorted(reused_source_ids)
        ],
        "u5_2_noncombined_points_use_u5_runner": True,
    }

    summaries = []
    for altitude in map(float, config["altitude_grid_m"]):
        straight = next(row for row in straight_table if row["altitude_m"] == altitude)
        turns = [row for row in turn_table if row["altitude_m"] == altitude and row["target_bank_deg"] != 0.0]
        verticals = [row for row in vertical_table if row["altitude_m"] == altitude]
        useful_turns = [row for row in turns if row["actual_stable_response"]]
        valid_climb = [row for row in verticals if row["target_vz_mps"] > 0 and row["status"] == "VALID"]
        valid_descent = [row for row in verticals if row["target_vz_mps"] < 0 and row["status"] == "VALID"]
        stable_climb = [row for row in verticals if row["target_vz_mps"] > 0 and row["actual_stable_response"]]
        stable_descent = [row for row in verticals if row["target_vz_mps"] < 0 and row["actual_stable_response"]]
        all_failures = [category for row in turns + verticals for category in row["failure_reason"]["categories"]]
        summaries.append(
            {
                "altitude_m": altitude,
                "straight": {
                    "status": straight["status"],
                    "actual_ias_mps": straight["actual_ias_mps"],
                    "throttle": straight["throttle"],
                    "power_margin_norm": 1.0 - straight["throttle"],
                },
                "level_turn": {
                    "useful_left_tested_targets_deg": [r["target_bank_deg"] for r in useful_turns if r["target_bank_deg"] < 0],
                    "useful_right_tested_targets_deg": [r["target_bank_deg"] for r in useful_turns if r["target_bank_deg"] > 0],
                    "measured_radius_range_m": None if not useful_turns else [min(r["turn_radius_m"] for r in useful_turns), max(r["turn_radius_m"] for r in useful_turns)],
                    "turn_rate_range_deg_s": None if not useful_turns else [min(r["turn_rate_deg_s"] for r in useful_turns), max(r["turn_rate_deg_s"] for r in useful_turns)],
                },
                "climb": {
                    "highest_tested_sustainable_valid_target_mps": None if not valid_climb else max(r["target_vz_mps"] for r in valid_climb),
                    "highest_meaningful_actual_stable_response_mps": None if not stable_climb else max(r["actual_vz_mps"] for r in stable_climb),
                    "first_important_limitation": _first_limitation(verticals, "climb"),
                },
                "descent": {
                    "largest_tested_sustainable_valid_target_mps": None if not valid_descent else min(r["target_vz_mps"] for r in valid_descent),
                    "largest_meaningful_actual_stable_response_mps": None if not stable_descent else min(r["actual_vz_mps"] for r in stable_descent),
                    "first_important_limitation": _first_limitation(verticals, "descent"),
                },
                "main_limitation_categories": sorted(set(all_failures)),
                "true_maximum_claimed": False,
            }
        )

    raw_lut = {
        "schema_version": 1,
        "artifact_type": "C172P_CORE_AIRCRAFT_LUT_RAW",
        "lut_stage": "raw",
        "planner_ready": False,
        "aircraft_id": "c172p",
        "nominal_ias_mps": 40.0,
        "min_altitude_m": 0.0,
        "max_altitude_m": 5500.0,
        "altitude_grid_m": [float(value) for value in config["altitude_grid_m"]],
        "controller_stack_id": config["frozen_stack"]["controller_stack_id"],
        "fcs_identity": aircraft_audit["flight_control_name"],
        "mixture_policy_id": config["frozen_stack"]["mixture_policy_id"],
        "mass_fuel_cg": {
            "configuration": run_config["fixture"],
            "observed": fixture_observed,
        },
        "atmosphere": reuse_audit["atmosphere_configuration"],
        "timestep_s": run_config["simulation"]["timestep_s"],
        "initialization_modes": reuse_audit["initialization_modes_observed"],
        "code_version": {
            "repository_git_commit": _git_commit(),
            "harness_sha256": files["harness"]["sha256"],
            "configuration_sha256": files["configuration"]["sha256"],
        },
        "generation_provenance": {
            "stage": "U6A",
            "provenance_id": provenance_id,
        },
        "same_stack_required_for_final_jsbsim_replay": True,
        "provenance_id": provenance_id,
        "status_semantics": config["raw_status_semantics"],
        "straight_table": straight_table,
        "level_turn_table": turn_table,
        "straight_vertical_table": vertical_table,
        "combined_3d_table_present": False,
        "interpolated": False,
        "derated": False,
        "true_service_ceiling_claim": False,
    }
    source_after = {name: _sha256(path) for name, path in source_paths.items()}
    provenance = {
        "schema_version": 1,
        "artifact_type": "U6A_PROVENANCE",
        "provenance_id": provenance_id,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "aircraft_id": "c172p",
        "nominal_ias_mps": 40.0,
        "altitude_domain_m": [0.0, 5500.0],
        "controller_stack_id": config["frozen_stack"]["controller_stack_id"],
        "fcs": aircraft_audit,
        "mixture_policy_id": config["frozen_stack"]["mixture_policy_id"],
        "fixture_mass_fuel_cg": {"configuration": run_config["fixture"], "observed": fixture_observed},
        "atmosphere": reuse_audit["atmosphere_configuration"],
        "timestep_s": run_config["simulation"]["timestep_s"],
        "initialization_modes": reuse_audit["initialization_modes_observed"],
        "python_version": sys.version,
        "platform": platform.platform(),
        "jsbsim_python_package_version": getattr(jsbsim, "__version__", None),
        "jsbsim_compiled_version": jsbsim.FGFDMExec(None).get_version(),
        "repository_git_commit": _git_commit(),
        "files": files,
        "source_hashes_before": source_before,
        "source_hashes_after": source_after,
        "historical_sources_unchanged": source_before == source_after,
    }
    result = {
        "step": "U6A",
        "step_status": "PASS",
        "provenance_id": provenance_id,
        "aircraft_id": "c172p",
        "nominal_ias_mps": 40.0,
        "current_domain_m": [0.0, 5500.0],
        "reused_unique_points": len(reused_source_ids),
        "new_unique_points": len(new_points),
        "new_cold_start_runs": len(new_runs),
        "straight_rows": len(straight_table),
        "level_turn_rows": len(turn_table),
        "straight_vertical_rows": len(vertical_table),
        "straight_grid_complete": len(straight_table) == 12,
        "turn_lut_complete": len(turn_table) == 84,
        "vertical_lut_complete": len(vertical_table) == 108,
        "core_raw_lut_ready": True,
        "ready_for_u6b": True,
        "combined_3d_executed": False,
        "interpolation_performed": False,
        "holdout_validation_performed": False,
        "derating_performed": False,
        "planner_integration_performed": False,
        "controller_tuning_performed": False,
        "aircraft_xml_modified": False,
        "u6b_started": False,
        "historical_sources_unchanged": source_before == source_after,
        "runtime_s": time.perf_counter() - started,
    }
    for name, payload in (
        ("c172p_core_aircraft_lut_raw.json", raw_lut),
        ("u6a_new_points.json", {"provenance_id": provenance_id, "points": new_points}),
        ("u6a_new_runs.json", {"provenance_id": provenance_id, "runs": new_runs}),
        ("u6a_reuse_audit.json", reuse_audit),
        ("u6a_altitude_summary.json", {"provenance_id": provenance_id, "rows": summaries}),
        ("u6a_provenance.json", provenance),
        ("u6a_result.json", result),
    ):
        _write(name, payload)
    return result


if __name__ == "__main__":
    print(json.dumps(run_u6a(), indent=2, allow_nan=False))
