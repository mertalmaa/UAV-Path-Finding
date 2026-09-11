"""Extend the closed J3 level-flight domain without rerunning its main grid."""

from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
from typing import Any

import yaml

import j2_baseline_sanity as j2
import j3_nominal_cas_selection as j3


HERE = Path(__file__).resolve().parent
RESULTS_DIR = HERE / "results"
CONFIG_PATH = HERE / "j3_1_domain_extension.yaml"
J2_CONFIG_PATH = HERE / "j2_reference_configuration.yaml"
J3_CONFIG_PATH = HERE / "j3_candidate_configuration.yaml"
M_TO_FT = 3.280839895013123


def _load_yaml(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _provenance(config: dict[str, Any]) -> dict[str, Any]:
    source_paths = {
        "parent_provenance": RESULTS_DIR / "j3_provenance.json",
        "parent_points": RESULTS_DIR / "j3_points.json",
        "parent_summary": RESULTS_DIR / "j3_candidate_summary.json",
        "parent_selection": RESULTS_DIR / "j3_selection.json",
    }
    stage_paths = {
        "domain_extension_configuration": CONFIG_PATH,
        "domain_extension_harness": Path(__file__).resolve(),
    }
    source_hashes = {
        name: {"path": str(path), "sha256": _sha256(path)}
        for name, path in source_paths.items()
    }
    stage_hashes = {
        name: {"path": str(path), "sha256": _sha256(path)}
        for name, path in stage_paths.items()
    }
    fingerprint = {
        "parent_provenance_id": config["parent_provenance_id"],
        "source_hashes": {name: item["sha256"] for name, item in source_hashes.items()},
        "stage_hashes": {name: item["sha256"] for name, item in stage_hashes.items()},
        "new_main_altitudes_m": config["domain"]["new_main_altitudes_m"],
        "new_midpoint_holdouts_m": config["domain"]["new_midpoint_holdouts_m"],
    }
    provenance_id = "j3.1-" + hashlib.sha256(
        json.dumps(fingerprint, sort_keys=True).encode("utf-8")
    ).hexdigest()[:20]
    return {
        "provenance_id": provenance_id,
        "parent_j3_provenance_id": config["parent_provenance_id"],
        "source_artifacts": source_hashes,
        "stage_files": stage_hashes,
        "domain": config["domain"],
    }


def _validate_parent(config: dict[str, Any], parent: dict[str, Any], points: list[dict[str, Any]]) -> None:
    expected = config["parent_provenance_id"]
    if parent["provenance_id"] != expected:
        raise RuntimeError("J3 parent provenance mismatch")
    expected_altitudes = set(config["domain"]["existing_main_altitudes_m"])
    expected_candidates = set(config["candidate_cas_kts"])
    actual_pairs = {
        (point["altitude_msl_m"], point["requested_target"]["speed_value"])
        for point in points
    }
    expected_pairs = {
        (altitude, candidate)
        for altitude in expected_altitudes
        for candidate in expected_candidates
    }
    if actual_pairs != expected_pairs:
        raise RuntimeError("J3 source points do not match the frozen grid")
    if any(point["provenance_id"] != expected for point in points):
        raise RuntimeError("J3 source point provenance mismatch")


def _install_airborne_test_fixture(config: dict[str, Any]) -> None:
    """Keep 0 m MSL airborne by separating synthetic ground from MSL altitude."""
    terrain_msl_m = float(
        config["airborne_test_fixture"]["synthetic_terrain_elevation_msl_m"]
    )
    base_set_reference = j2._set_reference_properties

    def set_reference_with_terrain(fdm: Any, reference_config: dict[str, Any]) -> None:
        base_set_reference(fdm, reference_config)
        fdm["position/terrain-elevation-asl-ft"] = terrain_msl_m * M_TO_FT

    j2._set_reference_properties = set_reference_with_terrain

    base_snapshot = j2._snapshot

    def snapshot_with_fixture(fdm: Any, elapsed_s: float) -> dict[str, float]:
        sample = base_snapshot(fdm, elapsed_s)
        sample["terrain_elevation_msl_m"] = (
            fdm["position/terrain-elevation-asl-ft"] / M_TO_FT
        )
        sample["height_agl_m"] = fdm["position/h-agl-ft"] / M_TO_FT
        return sample

    j2._snapshot = snapshot_with_fixture


def _run_point(
    altitude_m: float,
    candidate_kcas: float,
    repeats: int,
    j2_config: dict[str, Any],
    j3_config: dict[str, Any],
    provenance_id: str,
    run_prefix: str,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    run_config = copy.deepcopy(j2_config)
    run_config["sanity_condition"]["speed_value"] = candidate_kcas
    run_config["sanity_condition"]["representative_altitudes_msl_m"] = [altitude_m]
    records = [
        j2._run_once(altitude_m, repeat, run_config, provenance_id)
        for repeat in range(1, repeats + 1)
    ]
    for record in records:
        record["run_id"] = (
            f"j3.1-{run_prefix}-{candidate_kcas:.0f}kcas-{altitude_m:.0f}m-"
            f"r{record['repeat_index']}"
        )
        j3._normalize_zero_wind_roundoff(
            record,
            float(j3_config["numerical_tolerances"]["zero_wind_residual_fps"]),
        )
    repeatability = j2._repeatability(records, run_config)
    if not repeatability["passed"]:
        for record in records:
            if record["status"] == "VALID":
                record["status"] = "UNKNOWN"
                record["status_reasons"].append("cold_start_repeatability_failed")
    complete = all("measurement" in record and "tracking" in record for record in records)
    if complete:
        point = j3._point_result(
            candidate_kcas, altitude_m, records, repeatability, provenance_id
        )
    else:
        point = {
            "provenance_id": provenance_id,
            "altitude_msl_m": altitude_m,
            "maneuver_family": "straight_level",
            "requested_target": {
                "speed_type": "KCAS", "speed_value": candidate_kcas, "gamma_deg": 0.0,
            },
            "status": "UNKNOWN",
            "raw_capability": None,
            "derated_capability": None,
            "measured_values": None,
            "diagnostics": {
                "repeatability": repeatability,
                "trim_success_count": sum(record.get("trim", {}).get("succeeded", False) for record in records),
                "run_statuses": [record["status"] for record in records],
                "run_status_reasons": [record["status_reasons"] for record in records],
                "run_ids": [record["run_id"] for record in records],
            },
        }
    point["dataset_origin"] = "j3_1_new_measurement"
    return records, point


def _candidate_summary_with_unknowns(
    candidate: float,
    points: list[dict[str, Any]],
    combined_j3_config: dict[str, Any],
) -> dict[str, Any]:
    complete_points = [point for point in points if point["measured_values"] is not None]
    summary = j3._candidate_summary(candidate, complete_points, combined_j3_config)
    statuses = {
        status: sum(point["status"] == status for point in points)
        for status in ("VALID", "INFEASIBLE", "UNKNOWN")
    }
    summary["altitude_status_counts"] = statuses
    if statuses["VALID"] != len(combined_j3_config["main_altitudes_msl_m"]):
        summary["eligible_single_cas"] = False
    return summary


def _point_margin(point: dict[str, Any], j3_config: dict[str, Any]) -> dict[str, Any]:
    values = point["measured_values"]
    boundaries = j3_config["model_observation_boundaries"]
    components = {
        "lower_speed_schedule_clearance": (
            values["actual_kcas_mean"] - boundaries["trailing_edge_flap_schedule_kcas"]
        ) / boundaries["trailing_edge_flap_schedule_kcas"],
        "mach_schedule_clearance": (
            boundaries["mach_schedule_breakpoint"] - values["mach_mean"]
        ) / boundaries["mach_schedule_breakpoint"],
        "dry_throttle_two_sided_reserve": min(
            values["throttle_pos_norm_mean"],
            boundaries["afterburner_off_max_throttle_position_norm"]
            - values["throttle_pos_norm_mean"],
        ),
        "control_surface_reserve": (
            boundaries["normalized_control_surface_usage_max"]
            - values["maximum_control_surface_usage_norm"]
        ) / boundaries["normalized_control_surface_usage_max"],
        "alpha_command_reduction_clearance": (
            boundaries["alpha_command_reduction_start_abs_deg"]
            - abs(values["alpha_deg_mean"])
        ) / boundaries["alpha_command_reduction_start_abs_deg"],
    }
    return {"components": components, "minimum": min(components.values())}


def _validate_holdout(
    holdout: dict[str, Any],
    lower: dict[str, Any],
    upper: dict[str, Any],
    j3_config: dict[str, Any],
    extension_config: dict[str, Any],
) -> dict[str, Any]:
    holdout_margin = _point_margin(holdout, j3_config)
    lower_margin = _point_margin(lower, j3_config)
    upper_margin = _point_margin(upper, j3_config)
    endpoint_floor = min(lower_margin["minimum"], upper_margin["minimum"])
    epsilon = extension_config["holdout_validation"]["margin_comparison_epsilon"]
    margin_conservative = holdout_margin["minimum"] + epsilon >= endpoint_floor
    valid = holdout["status"] == "VALID"
    needs_refinement = not (valid and margin_conservative)
    return {
        "altitude_msl_m": holdout["altitude_msl_m"],
        "adjacent_main_band_msl_m": [lower["altitude_msl_m"], upper["altitude_msl_m"]],
        "status": holdout["status"],
        "holdout_minimum_margin": holdout_margin["minimum"],
        "adjacent_endpoint_minimum_margin_floor": endpoint_floor,
        "margin_components": holdout_margin["components"],
        "margin_not_worse_than_adjacent_endpoints": margin_conservative,
        "local_250m_refinement_required": needs_refinement,
        "selection_or_fitting_input": False,
    }


def run_j3_1() -> dict[str, Any]:
    extension = _load_yaml(CONFIG_PATH)
    j2_config = _load_yaml(J2_CONFIG_PATH)
    j3_config = _load_yaml(J3_CONFIG_PATH)
    parent_provenance = _load_json(RESULTS_DIR / "j3_provenance.json")
    parent_points = _load_json(RESULTS_DIR / "j3_points.json")
    _validate_parent(extension, parent_provenance, parent_points)
    provenance = _provenance(extension)
    j3._extend_j2_snapshot()
    _install_airborne_test_fixture(extension)

    new_runs: list[dict[str, Any]] = []
    new_points: list[dict[str, Any]] = []
    for candidate in extension["candidate_cas_kts"]:
        for altitude in extension["domain"]["new_main_altitudes_m"]:
            records, point = _run_point(
                float(altitude), float(candidate),
                extension["cold_start_repetitions_per_point"],
                j2_config, j3_config, provenance["provenance_id"], "main",
            )
            new_runs.extend(records)
            new_points.append(point)

    reused_points = [copy.deepcopy(point) for point in parent_points]
    for point in reused_points:
        point["dataset_origin"] = "j3_reused_measurement"
    combined_points = reused_points + new_points
    combined_j3_config = copy.deepcopy(j3_config)
    combined_j3_config["main_altitudes_msl_m"] = extension["domain"][
        "combined_main_altitudes_m"
    ]
    summaries = []
    for candidate in extension["candidate_cas_kts"]:
        candidate_points = [
            point for point in combined_points
            if point["requested_target"]["speed_value"] == candidate
        ]
        summaries.append(
            _candidate_summary_with_unknowns(
                float(candidate), candidate_points, combined_j3_config
            )
        )
    selection = j3._select_candidate(summaries)
    prior_nominal = extension["prior_nominal_cas_kts"]
    proposed = selection["selected_nominal_cas_kts"]
    nominal_confirmed = selection["j3_pass"] and proposed == prior_nominal

    holdout_runs: list[dict[str, Any]] = []
    holdout_points: list[dict[str, Any]] = []
    holdout_validations: list[dict[str, Any]] = []
    if nominal_confirmed:
        for altitude in extension["domain"]["new_midpoint_holdouts_m"]:
            records, point = _run_point(
                float(altitude), float(prior_nominal),
                extension["cold_start_repetitions_per_point"],
                j2_config, j3_config, provenance["provenance_id"], "holdout",
            )
            holdout_runs.extend(records)
            holdout_points.append(point)
            lower_altitude = float(altitude) - extension["domain"]["midpoint_offset_m"]
            upper_altitude = float(altitude) + extension["domain"]["midpoint_offset_m"]
            lower = next(
                item for item in combined_points
                if item["altitude_msl_m"] == lower_altitude
                and item["requested_target"]["speed_value"] == prior_nominal
            )
            upper = next(
                item for item in combined_points
                if item["altitude_msl_m"] == upper_altitude
                and item["requested_target"]["speed_value"] == prior_nominal
            )
            holdout_validations.append(
                _validate_holdout(point, lower, upper, j3_config, extension)
            )

    refinement_bands = [
        item["adjacent_main_band_msl_m"] for item in holdout_validations
        if item["local_250m_refinement_required"]
    ]
    if not selection["j3_pass"]:
        decision = "single nominal CAS assumption invalid"
        pass_gate = False
        blocker = "speed_band_required"
    elif proposed != prior_nominal:
        decision = "nominal_cas_change_requires_user_decision"
        pass_gate = False
        blocker = f"proposed_nominal_cas_{proposed:g}_kcas"
    elif refinement_bands:
        decision = "nominal_cas_retained_but_local_refinement_required"
        pass_gate = False
        blocker = "resolve_holdout_refinement_bands"
    else:
        decision = "nominal_cas_305_kcas_revalidated_for_0_6000m"
        pass_gate = True
        blocker = None

    result = {
        "provenance_id": provenance["provenance_id"],
        "parent_j3_provenance_id": provenance["parent_j3_provenance_id"],
        "decision": decision,
        "j3_1_pass": pass_gate,
        "prior_nominal_cas_kts": prior_nominal,
        "proposed_nominal_cas_kts": proposed,
        "effective_nominal_cas_kts": prior_nominal if nominal_confirmed else None,
        "new_main_run_count": len(new_runs),
        "reused_main_point_count": len(reused_points),
        "new_main_point_count": len(new_points),
        "combined_main_point_count": len(combined_points),
        "holdout_run_count": len(holdout_runs),
        "holdout_point_count": len(holdout_points),
        "holdouts_used_for_selection_or_fitting": False,
        "local_refinement_bands_msl_m": refinement_bands,
        "j4_blocker": blocker,
        "scope_guards": extension["scope"],
    }
    artifacts = {
        "j3_1_provenance.json": provenance,
        "j3_1_new_main_runs.json": new_runs,
        "j3_1_new_main_points.json": new_points,
        "j3_1_combined_main_points.json": combined_points,
        "j3_1_combined_candidate_summary.json": summaries,
        "j3_1_selection.json": selection,
        "j3_1_holdout_runs.json": holdout_runs,
        "j3_1_holdout_points.json": holdout_points,
        "j3_1_holdout_validation.json": holdout_validations,
        "j3_1_result.json": result,
    }
    for filename, payload in artifacts.items():
        (RESULTS_DIR / filename).write_text(
            json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
    return result


if __name__ == "__main__":
    result = run_j3_1()
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    raise SystemExit(0 if result["j3_1_pass"] else 1)
