"""Minimal outward sanity probes from the immutable U4 raw LUT edges."""

from __future__ import annotations

import copy
import hashlib
import json
import platform
import sys
import time
from pathlib import Path
from typing import Any

import jsbsim
import yaml

import u4_raw_lut_characterization as u4


HERE = Path(__file__).resolve().parent
CONFIG_PATH = HERE / "u4_1a_boundary_sanity_configuration.yaml"
U4_CONFIG_PATH = HERE / "u4_raw_lut_configuration.yaml"
U4_HARNESS_PATH = HERE / "u4_raw_lut_characterization.py"
U4_LUT_PATH = HERE / "results" / "aircraft_lut_raw.json"
U4_PROVENANCE_PATH = HERE / "results" / "u4_provenance.json"
U4_RESULT_PATH = HERE / "results" / "u4_result.json"
RESULTS_DIR = HERE / "results"


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


def _existing_turn(lut: dict[str, Any], altitude: float, bank: float) -> dict[str, Any]:
    return next(
        row for row in lut["turn_lut"]
        if row["altitude_m"] == altitude and row["target_bank_deg"] == bank
    )


def _existing_vertical(lut: dict[str, Any], altitude: float, vz: float) -> dict[str, Any]:
    return next(
        row for row in lut["vertical_lut"]
        if row["altitude_m"] == altitude and row["target_vz_mps"] == vz
    )


def _rename_probe_ids(
    records: list[dict[str, Any]], point: dict[str, Any], traces: list[dict[str, Any]]
) -> None:
    mapping = {}
    for record in records:
        old = record["run_id"]
        record["run_id"] = old.replace("u4-", "u4.1a-", 1)
        mapping[old] = record["run_id"]
    point["run_ids"] = [mapping[item] for item in point["run_ids"]]
    for trace in traces:
        trace["run_id"] = mapping[trace["run_id"]]


def _assessment(
    existing: list[dict[str, Any]], probes: list[dict[str, Any]]
) -> str:
    if any(row["status"] == "VALID" for row in probes):
        return "POSSIBLY ARTIFICIAL"
    if any(row["status"] == "UNKNOWN" for row in existing + probes):
        return "INCONCLUSIVE"
    return "NOT ARTIFICIAL"


def _probe_summary(row: dict[str, Any], target_key: str) -> dict[str, Any]:
    """Keep the per-altitude decision table compact and human-auditable."""
    summary = {
        target_key: row[target_key],
        "status": row["status"],
        "failure_class": row["failure_class"],
        "actual_vz_mps": row["actual_vz_mps"],
    }
    if target_key == "target_bank_deg":
        summary.update(
            {
                "actual_bank_deg": row["actual_bank_deg"],
                "turn_rate_deg_s": row["turn_rate_deg_s"],
                "measured_radius_m": row["measured_radius_m"],
            }
        )
    return summary


def _altitude_summary(
    lut: dict[str, Any],
    altitude: float,
    turn_edges: list[dict[str, Any]],
    turn_probes: list[dict[str, Any]],
    vertical_edges: list[dict[str, Any]],
    vertical_probes: list[dict[str, Any]],
) -> dict[str, Any]:
    turns_at_altitude = [
        row for row in lut["turn_lut"] if row["altitude_m"] == altitude
    ]
    vertical_at_altitude = [
        row for row in lut["vertical_lut"] if row["altitude_m"] == altitude
    ]
    summary: dict[str, Any] = {
        "altitude_m": altitude,
        "turn": {},
        "vertical": {},
    }
    for direction in ("LEFT", "RIGHT"):
        edge = next(
            row
            for row in turn_edges
            if row["altitude_m"] == altitude and row["direction"] == direction
        )
        valid = [
            row
            for row in turns_at_altitude
            if row["direction"] == direction and row["status"] == "VALID"
        ]
        probes = [
            _probe_summary(row, "target_bank_deg")
            for row in turn_probes
            if row["altitude_m"] == altitude and row["direction"] == direction
        ]
        summary["turn"][direction] = {
            "existing_outermost_tested_valid_bank_deg": (
                max(valid, key=lambda row: abs(row["target_bank_deg"]))[
                    "target_bank_deg"
                ]
                if valid
                else None
            ),
            "existing_grid_edge_bank_deg": edge["target_bank_deg"],
            "existing_grid_edge_status": edge["status"],
            "existing_grid_edge_failure_class": edge["failure_class"],
            "extension_eligible": edge["status"] == "VALID",
            "additional_probes": probes,
            "stop_reason": (
                "FIRST_NON_VALID_PROBE"
                if probes and probes[-1]["status"] != "VALID"
                else "EXISTING_GRID_EDGE_NON_VALID"
                if edge["status"] != "VALID"
                else "OPTIONAL_PROBE_LIMIT_REACHED"
            ),
        }

    for label, edge_target, sign in (
        ("CLIMB", 5.0, 1.0),
        ("DESCENT", -5.0, -1.0),
    ):
        edge = next(
            row
            for row in vertical_edges
            if row["altitude_m"] == altitude
            and row["target_vz_mps"] == edge_target
        )
        valid = [
            row
            for row in vertical_at_altitude
            if sign * row["target_vz_mps"] > 0 and row["status"] == "VALID"
        ]
        probes = [
            _probe_summary(row, "target_vz_mps")
            for row in vertical_probes
            if row["altitude_m"] == altitude
            and sign * row["target_vz_mps"] > 0
        ]
        summary["vertical"][label] = {
            "existing_outermost_tested_valid_vz_mps": (
                max(valid, key=lambda row: sign * row["target_vz_mps"])[
                    "target_vz_mps"
                ]
                if valid
                else None
            ),
            "existing_grid_edge_vz_mps": edge["target_vz_mps"],
            "existing_grid_edge_status": edge["status"],
            "existing_grid_edge_failure_class": edge["failure_class"],
            "extension_eligible": edge["status"] == "VALID",
            "additional_probes": probes,
            "stop_reason": (
                "FIRST_NON_VALID_PROBE"
                if probes and probes[-1]["status"] != "VALID"
                else "EXISTING_GRID_EDGE_NON_VALID"
                if edge["status"] != "VALID"
                else "SEQUENCE_LIMIT_REACHED"
            ),
        }
    return summary


def run_u4_1a() -> dict[str, Any]:
    started = time.perf_counter()
    config = _load_yaml(CONFIG_PATH)
    raw_hash_before = _sha256(U4_LUT_PATH)
    if raw_hash_before != config["parent_raw_lut_sha256"]:
        raise RuntimeError("canonical U4 raw LUT hash changed")
    lut = json.loads(U4_LUT_PATH.read_text(encoding="utf-8"))
    u4_provenance = json.loads(U4_PROVENANCE_PATH.read_text(encoding="utf-8"))
    u4_result = json.loads(U4_RESULT_PATH.read_text(encoding="utf-8"))
    u4_config = _load_yaml(U4_CONFIG_PATH)
    u4._freeze_check(u4_config)
    if u4_provenance["provenance_id"] != config["parent_u4_provenance_id"]:
        raise RuntimeError("U4 provenance changed")
    if not u4_result["raw_lut_ready_for_holdout_validation"]:
        raise RuntimeError("U4 raw LUT is not complete")

    altitudes = [float(value) for value in config["representative_altitudes_m"]]
    if config["include_optional_2500_m_existing_edge_audit"]:
        altitudes.append(2500.0)
    seed = {
        "configuration": _sha256(CONFIG_PATH),
        "harness": _sha256(Path(__file__).resolve()),
        "parent_raw_lut": raw_hash_before,
        "parent_u4_provenance": _sha256(U4_PROVENANCE_PATH),
        "parent_u4_harness": _sha256(U4_HARNESS_PATH),
    }
    provenance_id = "u4.1a-" + hashlib.sha256(
        json.dumps(seed, sort_keys=True).encode("utf-8")
    ).hexdigest()[:20]

    existing_turn_edges: list[dict[str, Any]] = []
    existing_vertical_edges: list[dict[str, Any]] = []
    turn_probes: list[dict[str, Any]] = []
    vertical_probes: list[dict[str, Any]] = []
    all_runs: list[dict[str, Any]] = []
    all_points: list[dict[str, Any]] = []
    all_traces: list[dict[str, Any]] = []

    for altitude in altitudes:
        for direction, sign in (("LEFT", -1.0), ("RIGHT", 1.0)):
            edge = copy.deepcopy(_existing_turn(lut, altitude, sign * config["turn"]["existing_edge_bank_magnitude_deg"]))
            edge["source"] = "U4_EXISTING_RAW_LUT"
            existing_turn_edges.append(edge)
            if edge["status"] != "VALID":
                continue
            sequence = list(config["turn"]["outward_sequence_deg"])
            for magnitude in sequence:
                records, point, traces = u4._run_point(
                    altitude, "turn", sign * float(magnitude), u4_config, provenance_id
                )
                _rename_probe_ids(records, point, traces)
                row = u4._turn_entry(point, records, altitude, sign * float(magnitude), provenance_id)
                row["source"] = "U4_1A_NEW_PROBE"
                row["parent_edge_bank_deg"] = edge["target_bank_deg"]
                turn_probes.append(row)
                all_runs.extend(records)
                all_points.append(point)
                all_traces.extend(traces)
                if row["status"] != "VALID":
                    break
            if (
                turn_probes
                and turn_probes[-1]["altitude_m"] == altitude
                and turn_probes[-1]["direction"] == direction
                and turn_probes[-1]["target_bank_deg"] == sign * sequence[-1]
                and turn_probes[-1]["status"] == "VALID"
            ):
                magnitude = float(config["turn"]["optional_single_point_deg"])
                records, point, traces = u4._run_point(
                    altitude, "turn", sign * magnitude, u4_config, provenance_id
                )
                _rename_probe_ids(records, point, traces)
                row = u4._turn_entry(point, records, altitude, sign * magnitude, provenance_id)
                row["source"] = "U4_1A_OPTIONAL_SINGLE_PROBE"
                row["parent_edge_bank_deg"] = edge["target_bank_deg"]
                turn_probes.append(row)
                all_runs.extend(records)
                all_points.append(point)
                all_traces.extend(traces)

        for signed_edge, sequence in (
            (5.0, config["vertical"]["climb_outward_sequence_mps"]),
            (-5.0, config["vertical"]["descent_outward_sequence_mps"]),
        ):
            edge = copy.deepcopy(_existing_vertical(lut, altitude, signed_edge))
            edge["source"] = "U4_EXISTING_RAW_LUT"
            existing_vertical_edges.append(edge)
            if edge["status"] != "VALID":
                continue
            for target in sequence:
                records, point, traces = u4._run_point(
                    altitude, "vertical", float(target), u4_config, provenance_id
                )
                _rename_probe_ids(records, point, traces)
                row = u4._vertical_entry(point, records, altitude, float(target), provenance_id)
                row["source"] = "U4_1A_NEW_PROBE"
                row["parent_edge_vz_mps"] = signed_edge
                vertical_probes.append(row)
                all_runs.extend(records)
                all_points.append(point)
                all_traces.extend(traces)
                if row["status"] != "VALID":
                    break

    raw_hash_after = _sha256(U4_LUT_PATH)
    if raw_hash_after != raw_hash_before:
        raise RuntimeError("U4 raw LUT was modified")

    turn_right_existing = [row for row in existing_turn_edges if row["direction"] == "RIGHT"]
    turn_left_existing = [row for row in existing_turn_edges if row["direction"] == "LEFT"]
    turn_right_probes = [row for row in turn_probes if row["direction"] == "RIGHT"]
    turn_left_probes = [row for row in turn_probes if row["direction"] == "LEFT"]
    climbs_existing = [row for row in existing_vertical_edges if row["target_vz_mps"] > 0]
    descents_existing = [row for row in existing_vertical_edges if row["target_vz_mps"] < 0]
    climbs_probes = [row for row in vertical_probes if row["target_vz_mps"] > 0]
    descents_probes = [row for row in vertical_probes if row["target_vz_mps"] < 0]
    representative_turn_rows = [
        row for row in lut["turn_lut"] if row["altitude_m"] in altitudes
    ]
    representative_vertical_rows = [
        row for row in lut["vertical_lut"] if row["altitude_m"] in altitudes
    ]
    highest_valid_turn = max(
        (
            abs(row["target_bank_deg"])
            for row in representative_turn_rows + turn_probes
            if row["status"] == "VALID"
        ),
        default=None,
    )
    highest_valid_climb = max(
        (
            row["target_vz_mps"]
            for row in representative_vertical_rows + vertical_probes
            if row["status"] == "VALID" and row["target_vz_mps"] > 0
        ),
        default=None,
    )
    highest_valid_descent = max(
        (
            -row["target_vz_mps"]
            for row in representative_vertical_rows + vertical_probes
            if row["status"] == "VALID" and row["target_vz_mps"] < 0
        ),
        default=None,
    )
    altitude_results = [
        _altitude_summary(
            lut,
            altitude,
            existing_turn_edges,
            turn_probes,
            existing_vertical_edges,
            vertical_probes,
        )
        for altitude in altitudes
    ]
    artifact = {
        "schema_version": 1,
        "artifact_type": "CAPABILITY_BOUNDARY_SANITY",
        "provenance_id": provenance_id,
        "parent_u4_provenance_id": config["parent_u4_provenance_id"],
        "parent_raw_lut_sha256_before": raw_hash_before,
        "parent_raw_lut_sha256_after": raw_hash_after,
        "parent_raw_lut_unchanged": raw_hash_before == raw_hash_after,
        "aircraft": "c172r",
        "nominal_ias_mps": 40.0,
        "representative_altitudes_m": altitudes,
        "existing_turn_edges": existing_turn_edges,
        "new_turn_probes": turn_probes,
        "existing_vertical_edges": existing_vertical_edges,
        "new_vertical_probes": vertical_probes,
        "altitude_results": altitude_results,
        "assessment": {
            "turn_grid_edge": _assessment(existing_turn_edges, turn_probes),
            "turn_right_grid_edge": _assessment(turn_right_existing, turn_right_probes),
            "turn_left_grid_edge": _assessment(turn_left_existing, turn_left_probes),
            "vertical_grid_edge": _assessment(existing_vertical_edges, vertical_probes),
            "climb_grid_edge": _assessment(climbs_existing, climbs_probes),
            "descent_grid_edge": _assessment(descents_existing, descents_probes),
            "highest_tested_valid_bank_magnitude_deg": highest_valid_turn,
            "highest_tested_valid_climb_mps": highest_valid_climb,
            "largest_tested_valid_descent_magnitude_mps": highest_valid_descent,
            "true_physical_maximum_extracted": False,
            "planner_safe_envelope_selected": False,
        },
    }
    provenance = {
        "provenance_id": provenance_id,
        "parent_u4_provenance_id": config["parent_u4_provenance_id"],
        "parent_raw_lut_sha256": raw_hash_before,
        "jsbsim_compiled_version": jsbsim.FGFDMExec(None).get_version(),
        "python_version": sys.version,
        "python_executable": sys.executable,
        "platform": platform.platform(),
        "files": {
            "configuration": {"path": str(CONFIG_PATH.resolve()), "sha256": _sha256(CONFIG_PATH)},
            "harness": {"path": str(Path(__file__).resolve()), "sha256": _sha256(Path(__file__).resolve())},
            "u4_raw_lut": {"path": str(U4_LUT_PATH.resolve()), "sha256": raw_hash_before},
            "u4_configuration": {"path": str(U4_CONFIG_PATH.resolve()), "sha256": _sha256(U4_CONFIG_PATH)},
            "u4_harness": {"path": str(U4_HARNESS_PATH.resolve()), "sha256": _sha256(U4_HARNESS_PATH)},
            "u4_provenance": {"path": str(U4_PROVENANCE_PATH.resolve()), "sha256": _sha256(U4_PROVENANCE_PATH)},
        },
    }
    result = {
        "step": "U4.1A",
        "step_status": "PASS",
        "provenance_id": provenance_id,
        "new_turn_probe_count": len(turn_probes),
        "new_vertical_probe_count": len(vertical_probes),
        "executed_points": len(all_points),
        "executed_cold_start_runs": len(all_runs),
        "turn_grid_edge": artifact["assessment"]["turn_grid_edge"],
        "vertical_grid_edge": artifact["assessment"]["vertical_grid_edge"],
        "raw_u4_lut_unchanged": raw_hash_before == raw_hash_after,
        "controller_tuning_performed": False,
        "stock_xml_modified": False,
        "planner_modified": False,
        "interpolation_performed": False,
        "derating_performed": False,
        "holdout_validation_started": False,
        "true_physical_maximum_extracted": False,
        "ready_for_u4_1b_holdout_validation": True,
        "runtime_s": time.perf_counter() - started,
    }
    _write("u4_1a_boundary_sanity.json", artifact)
    _write("u4_1a_provenance.json", provenance)
    _write("u4_1a_runs.json", {"provenance_id": provenance_id, "runs": all_runs})
    _write("u4_1a_points.json", {"provenance_id": provenance_id, "points": all_points})
    _write("u4_1a_result.json", result)
    return result


if __name__ == "__main__":
    print(json.dumps(run_u4_1a(), indent=2, allow_nan=False))
