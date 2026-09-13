from __future__ import annotations

import hashlib
import json
import math
import platform
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
CONFIG_PATH = HERE / "u6b_c172p_combined_3d_configuration.yaml"
HARNESS_PATH = Path(__file__).resolve()
sys.path.insert(0, str(HERE))

import u5_2_finalist_general_validation as u5_2
import u5_practical_aircraft_rescreen as u5
import u6a_c172p_core_raw_lut as u6a


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


def _key(altitude: float, bank: float, vz: float) -> tuple[float, float, float]:
    return float(altitude), float(bank), float(vz)


def _source_point_id(point: dict[str, Any]) -> str:
    return point["run_ids"][0].rsplit("-r", 1)[0]


def _remap_new_ids(
    records: list[dict[str, Any]], point: dict[str, Any], analysis: dict[str, Any]
) -> None:
    mapping = {}
    for record in records:
        old = record["run_id"]
        new = old.replace("u5.2-", "u6b-", 1)
        record["run_id"] = new
        mapping[old] = new
    point["run_ids"] = [mapping[run_id] for run_id in point["run_ids"]]
    analysis["run_ids"] = list(point["run_ids"])


def _metric(records: list[dict[str, Any]], name: str) -> float:
    return fmean(record["measurement"][name]["mean"] for record in records)


def _vertical_reference(
    vertical_rows: list[dict[str, Any]], altitude: float, target_vz: float
) -> tuple[dict[str, Any] | None, list[dict[str, Any]]]:
    signed = [
        row for row in vertical_rows
        if row["altitude_m"] == altitude
        and row["target_vz_mps"] * target_vz > 0.0
    ]
    exact = next(
        (row for row in signed if row["target_vz_mps"] == target_vz), None
    )
    if exact is not None:
        return exact, []
    ordered = sorted(signed, key=lambda row: row["target_vz_mps"])
    lower = [row for row in ordered if row["target_vz_mps"] < target_vz]
    upper = [row for row in ordered if row["target_vz_mps"] > target_vz]
    bracket = []
    if lower:
        bracket.append(lower[-1])
    if upper:
        bracket.append(upper[0])
    return None, bracket


def _canonical_row(
    point: dict[str, Any],
    records: list[dict[str, Any]],
    analysis: dict[str, Any],
    source_stage: str,
    grid_role: str,
    u6a_lut: dict[str, Any],
) -> dict[str, Any]:
    altitude = float(analysis["altitude_m"])
    bank = float(analysis["bank_target_deg"])
    target_vz = float(analysis["vertical_speed_target_mps"])
    actual = analysis["actual"]
    level = next(
        row for row in u6a_lut["level_turn_table"]
        if row["altitude_m"] == altitude and row["target_bank_deg"] == bank
    )
    exact_vertical, bracket = _vertical_reference(
        u6a_lut["straight_vertical_table"], altitude, target_vz
    )
    reference_vz = (
        None if exact_vertical is None else exact_vertical["actual_vz_mps"]
    )
    radius = analysis["measured_turn_radius_m"]
    return {
        "altitude_m": altitude,
        "nominal_ias_mps": 40.0,
        "target_bank_deg": bank,
        "target_vz_mps": target_vz,
        "direction": "LEFT" if bank < 0.0 else "RIGHT",
        "maneuver_family": "CLIMBING_TURN" if target_vz > 0.0 else "DESCENDING_TURN",
        "grid_role": grid_role,
        "status": point["status"],
        "usability": analysis["usability"],
        "usability_cautions": analysis["usability_cautions"],
        "actual_stable_response": analysis["actual_stable_response"],
        "actual_ias_mps": actual["ias_mps"],
        "actual_bank_deg": actual["roll_deg"],
        "actual_vz_mps": actual["vertical_speed_mps"],
        "target_vz_achievement_ratio": actual["vertical_speed_mps"] / target_vz,
        "turn_rate_deg_s": analysis["turn_rate_deg_s"],
        "turn_radius_m": radius,
        "theoretical_turn_radius_m": actual["theoretical_radius_m"],
        "radius_theory_relative_difference": actual["radius_theory_relative_difference"],
        "level_turn_reference_radius_m": level["turn_radius_m"],
        "radius_ratio": radius / level["turn_radius_m"],
        "straight_vertical_reference_target_vz_mps": (
            None if exact_vertical is None else exact_vertical["target_vz_mps"]
        ),
        "straight_vertical_reference_vz_mps": reference_vz,
        "vz_retention_ratio": (
            None if reference_vz in (None, 0.0)
            else actual["vertical_speed_mps"] / reference_vz
        ),
        "straight_vertical_bracketing_raw_references": [
            {
                "target_vz_mps": row["target_vz_mps"],
                "actual_vz_mps": row["actual_vz_mps"],
                "status": row["status"],
                "actual_stable_response": row["actual_stable_response"],
            }
            for row in bracket
        ],
        "gamma_deg": actual["gamma_deg"],
        "pitch_deg": _metric(records, "pitch_deg"),
        "aoa_deg": actual["alpha_deg"],
        "beta_deg": actual["beta_deg"],
        "nz": actual["nz"],
        "throttle": actual["engine_0_throttle_pos_norm"],
        "rpm": analysis["propulsion"]["engines"][0]["propeller_rpm"],
        "power_hp": analysis["propulsion"]["engines"][0]["power_hp"],
        "thrust_lbs": analysis["propulsion"]["engines"][0]["thrust_lbs"],
        "controller_outputs": {
            "aileron_cmd_norm": actual["aileron_cmd_norm"],
            "elevator_cmd_norm": actual["elevator_cmd_norm"],
            "rudder_cmd_norm": actual["rudder_cmd_norm"],
        },
        "maximum_surface_usage_norm": actual["maximum_surface_usage_norm"],
        "saturation": analysis["controller_saturation_in_measurement"],
        "settled": all(
            item["settled_by_screening_deadline"] for item in analysis["settling"]
        ),
        "settling": analysis["settling"],
        "repeatable": analysis["repeatability"]["passed"],
        "failure_reason": u6a._failure_classification(records),
        "metadata": {
            "source_stage": source_stage,
            "source_point_id": _source_point_id(point),
            "source_provenance": point["provenance_id"],
            "source_run_ids": point["run_ids"],
            "execution": "REUSED" if source_stage == "U5.2" else "NEW_U6B",
            "cold_start_repetitions": len(point["run_ids"]),
            "u6a_level_turn_reference_source": level["metadata"],
            "u6a_vertical_reference_exact": exact_vertical is not None,
        },
    }


def _family_class(rows: list[dict[str, Any]]) -> str:
    if any(row["usability"] == "UNUSABLE" for row in rows):
        return "UNUSABLE"
    if all(row["usability"] == "USABLE" for row in rows):
        return "USABLE"
    return "MARGINAL"


def run_u6b() -> dict[str, Any]:
    started = time.perf_counter()
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    source_paths = {
        name: HERE / item["path"] for name, item in config["source_artifacts"].items()
    }
    source_before = {name: _sha256(path) for name, path in source_paths.items()}
    expected = {name: item["sha256"] for name, item in config["source_artifacts"].items()}
    if source_before != expected:
        raise RuntimeError("U5/U5.2/U6A source hash mismatch before U6B")

    u5_points = _load(source_paths["u5_points"])
    u5_runs = _load(source_paths["u5_runs"])
    u5_2_points = _load(source_paths["u5_2_points"])
    u5_2_runs = _load(source_paths["u5_2_runs"])
    u5_2_combined = _load(source_paths["u5_2_combined"])
    u5_2_result = _load(source_paths["u5_2_result"])
    u6a_lut = _load(source_paths["u6a_lut"])
    u6a_points = _load(source_paths["u6a_points"])
    u6a_runs = _load(source_paths["u6a_runs"])
    u6a_provenance = _load(source_paths["u6a_provenance"])
    u6a_result = _load(source_paths["u6a_result"])
    if u5_2_result["provenance_id"] != config["parent_u5_2_provenance_id"]:
        raise RuntimeError("parent U5.2 provenance mismatch")
    if u6a_result["provenance_id"] != config["parent_u6a_provenance_id"]:
        raise RuntimeError("parent U6A provenance mismatch")

    files = {
        "configuration": {"path": str(CONFIG_PATH), "sha256": _sha256(CONFIG_PATH)},
        "harness": {"path": str(HARNESS_PATH), "sha256": _sha256(HARNESS_PATH)},
    }
    seed = {
        "files": {name: item["sha256"] for name, item in files.items()},
        "sources": source_before,
        "stack": config["frozen_stack"],
        "primary_grid": config["primary_grid"],
        "second_severity": config["second_severity"],
    }
    provenance_id = "u6b-" + hashlib.sha256(
        json.dumps(seed, sort_keys=True).encode("utf-8")
    ).hexdigest()[:20]

    old_points = {
        _key(
            point["altitude_msl_m"], point["requested"]["bank_deg"],
            point["requested"]["vertical_speed_mps"],
        ): point
        for point in u5_2_points["points"]
        if point["aircraft_model"] == "c172p" and point["maneuver"] == "combined"
    }
    all_source_runs = {
        row["run_id"]: row
        for row in u5_runs["runs"] + u5_2_runs["runs"] + u6a_runs["runs"]
    }
    old_analysis = {
        _key(row["altitude_m"], row["bank_target_deg"], row["vertical_speed_target_mps"]): row
        for row in u5_2_combined["rows"] if row["aircraft"] == "c172p"
    }
    level_reference_points = (
        u5_points["points"] + u5_2_points["points"] + u6a_points["points"]
    )
    u5_config = yaml.safe_load(u5.CONFIG_PATH.read_text(encoding="utf-8"))
    u3_config = yaml.safe_load(u5.U3_CONFIG_PATH.read_text(encoding="utf-8"))
    candidate = u5_config["candidates"]["c172p"]
    new_points: list[dict[str, Any]] = []
    new_runs: list[dict[str, Any]] = []
    rows: list[dict[str, Any]] = []
    reused = []

    def obtain(altitude: float, bank: float, vz: float, role: str) -> None:
        key = _key(altitude, bank, vz)
        if key in old_points:
            point = old_points[key]
            records = [all_source_runs[run_id] for run_id in point["run_ids"]]
            analysis = old_analysis[key]
            stage = "U5.2"
            reused.append({
                "source_stage": stage,
                "source_point_id": _source_point_id(point),
                "source_provenance": point["provenance_id"],
            })
        else:
            records, point, analysis = u5_2._run_combined_point(
                "c172p", candidate, altitude, 40.0, bank, vz,
                u5_config, u3_config, config, provenance_id, level_reference_points,
            )
            _remap_new_ids(records, point, analysis)
            new_runs.extend(records)
            new_points.append(point)
            all_source_runs.update({record["run_id"]: record for record in records})
            stage = "U6B"
        rows.append(_canonical_row(point, records, analysis, stage, role, u6a_lut))

    u5._install_hooks()
    try:
        for altitude_value in config["representative_altitudes"].values():
            altitude = float(altitude_value)
            for bank_value in config["primary_grid"]["bank_targets_deg"]:
                for vz_value in config["primary_grid"]["vertical_speed_targets_mps"]:
                    obtain(altitude, float(bank_value), float(vz_value), "PRIMARY")
        if config["second_severity"]["enabled"]:
            for altitude_value in config["second_severity"]["selected_altitudes_m"]:
                for bank_value in config["second_severity"]["bank_targets_deg"]:
                    for vz_value in config["second_severity"]["vertical_speed_targets_mps"]:
                        obtain(
                            float(altitude_value), float(bank_value), float(vz_value),
                            "SECOND_SEVERITY",
                        )
    finally:
        u5._restore_hooks()

    if len(new_points) != config["reuse_contract"]["expected_new_points"]:
        raise RuntimeError(f"unexpected U6B new point count {len(new_points)}")
    if len(new_runs) != config["reuse_contract"]["expected_new_cold_start_runs"]:
        raise RuntimeError(f"unexpected U6B run count {len(new_runs)}")
    if len(reused) != config["reuse_contract"]["expected_u5_2_reused_primary_points"]:
        raise RuntimeError(f"unexpected U6B reuse count {len(reused)}")
    if len({record["run_id"] for record in new_runs}) != len(new_runs):
        raise RuntimeError("duplicate U6B run IDs")

    primary_rows = [row for row in rows if row["grid_role"] == "PRIMARY"]
    summaries = []
    for region, altitude_value in config["representative_altitudes"].items():
        altitude = float(altitude_value)
        altitude_rows = [row for row in primary_rows if row["altitude_m"] == altitude]
        climb = [row for row in altitude_rows if row["target_vz_mps"] > 0.0]
        descent = [row for row in altitude_rows if row["target_vz_mps"] < 0.0]
        summaries.append({
            "region": region,
            "altitude_m": altitude,
            "climbing_turn": {
                "interpretation": _family_class(climb),
                "actual_stable_count": sum(row["actual_stable_response"] for row in climb),
                "strict_valid_count": sum(row["status"] == "VALID" for row in climb),
                "actual_vz_range_mps": [min(row["actual_vz_mps"] for row in climb), max(row["actual_vz_mps"] for row in climb)],
                "ias_range_mps": [min(row["actual_ias_mps"] for row in climb), max(row["actual_ias_mps"] for row in climb)],
                "throttle_range": [min(row["throttle"] for row in climb), max(row["throttle"] for row in climb)],
                "radius_ratio_range": [min(row["radius_ratio"] for row in climb), max(row["radius_ratio"] for row in climb)],
            },
            "descending_turn": {
                "interpretation": _family_class(descent),
                "actual_stable_count": sum(row["actual_stable_response"] for row in descent),
                "strict_valid_count": sum(row["status"] == "VALID" for row in descent),
                "actual_vz_range_mps": [min(row["actual_vz_mps"] for row in descent), max(row["actual_vz_mps"] for row in descent)],
                "ias_range_mps": [min(row["actual_ias_mps"] for row in descent), max(row["actual_ias_mps"] for row in descent)],
                "throttle_range": [min(row["throttle"] for row in descent), max(row["throttle"] for row in descent)],
                "radius_ratio_range": [min(row["radius_ratio"] for row in descent), max(row["radius_ratio"] for row in descent)],
            },
            "planner_safe": False,
            "directly_infeasible_implies_unreachable": False,
        })

    source_after = {name: _sha256(path) for name, path in source_paths.items()}
    raw = {
        "schema_version": 1,
        "artifact_type": "C172P_COMBINED_3D_RAW",
        "lut_stage": "raw_representative_validation",
        "planner_ready": False,
        "aircraft_id": "c172p",
        "nominal_ias_mps": 40.0,
        "current_domain_m": [0.0, 5500.0],
        "representative_altitudes_m": [float(value) for value in config["representative_altitudes"].values()],
        "controller_stack_id": config["frozen_stack"]["controller_stack_id"],
        "fcs_identity": config["frozen_stack"]["stock_fcs_identity"],
        "mixture_policy_id": config["frozen_stack"]["mixture_policy_id"],
        "u6a_core_lut_provenance_id": u6a_lut["provenance_id"],
        "provenance_id": provenance_id,
        "status_semantics": config["status_semantics"],
        "reference_policy": config["reference_policy"],
        "rows": rows,
        "full_envelope_claim": False,
        "interpolated": False,
        "derated": False,
        "holdout_validated": False,
        "directly_infeasible_implies_unreachable": False,
    }
    reuse_audit = {
        "schema_version": 1,
        "artifact_type": "U6B_SAME_STACK_REUSE_AUDIT",
        "provenance_id": provenance_id,
        "equivalent": True,
        "controller_stack_id": config["frozen_stack"]["controller_stack_id"],
        "u5_2_reused_point_count": len(reused),
        "reused_points": reused,
        "u6a_reference_lut_unchanged": source_before["u6a_lut"] == source_after["u6a_lut"],
        "u6a_provenance_id": u6a_lut["provenance_id"],
        "same_aircraft_ias_controller_fcs_mixture_fixture_atmosphere_initialization_timestep_measurement": True,
    }
    provenance = {
        "schema_version": 1,
        "artifact_type": "U6B_PROVENANCE",
        "provenance_id": provenance_id,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "aircraft_id": "c172p",
        "nominal_ias_mps": 40.0,
        "current_domain_m": [0.0, 5500.0],
        "controller_stack_id": config["frozen_stack"]["controller_stack_id"],
        "fcs": u6a_provenance["fcs"],
        "mixture_policy_id": config["frozen_stack"]["mixture_policy_id"],
        "fixture_mass_fuel_cg": u6a_provenance["fixture_mass_fuel_cg"],
        "atmosphere": u6a_provenance["atmosphere"],
        "timestep_s": u6a_provenance["timestep_s"],
        "initialization_modes": u6a_provenance["initialization_modes"],
        "measurement_contract_inherited_from_u6a": True,
        "python_version": sys.version,
        "platform": platform.platform(),
        "jsbsim_python_package_version": getattr(jsbsim, "__version__", None),
        "jsbsim_compiled_version": jsbsim.FGFDMExec(None).get_version(),
        "repository_git_commit": u6a._git_commit(),
        "files": files,
        "source_hashes_before": source_before,
        "source_hashes_after": source_after,
        "historical_sources_unchanged": source_before == source_after,
    }
    result = {
        "step": "U6B",
        "step_status": "PASS",
        "provenance_id": provenance_id,
        "aircraft_id": "c172p",
        "nominal_ias_mps": 40.0,
        "representative_altitudes_m": raw["representative_altitudes_m"],
        "primary_rows": len(primary_rows),
        "second_severity_rows": len(rows) - len(primary_rows),
        "total_rows": len(rows),
        "reused_points": len(reused),
        "new_points": len(new_points),
        "new_cold_start_runs": len(new_runs),
        "combined_3d_raw_data_ready": True,
        "ready_for_u6_1_holdout_validation": True,
        "u6a_artifact_unchanged": source_before["u6a_lut"] == source_after["u6a_lut"],
        "historical_sources_unchanged": source_before == source_after,
        "full_combined_cartesian_sweep_performed": False,
        "speed_sweep_performed": False,
        "controller_tuning_performed": False,
        "aircraft_xml_modified": False,
        "interpolation_performed": False,
        "holdout_validation_performed": False,
        "derating_performed": False,
        "planner_integration_performed": False,
        "u6_1_started": False,
        "runtime_s": time.perf_counter() - started,
    }
    for name, payload in (
        ("c172p_combined_3d_raw.json", raw),
        ("u6b_new_points.json", {"provenance_id": provenance_id, "points": new_points}),
        ("u6b_new_runs.json", {"provenance_id": provenance_id, "runs": new_runs}),
        ("u6b_reuse_audit.json", reuse_audit),
        ("u6b_altitude_summary.json", {"provenance_id": provenance_id, "rows": summaries}),
        ("u6b_provenance.json", provenance),
        ("u6b_result.json", result),
    ):
        _write(name, payload)
    return result


if __name__ == "__main__":
    print(json.dumps(run_u6b(), indent=2, allow_nan=False))
