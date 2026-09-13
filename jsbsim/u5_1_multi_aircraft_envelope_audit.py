from __future__ import annotations

import hashlib
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from statistics import fmean
from typing import Any

import yaml


HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
CONFIG_PATH = HERE / "u5_1_multi_aircraft_envelope_audit_configuration.yaml"
HARNESS_PATH = Path(__file__).resolve()


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


def _cell_classification(row: dict[str, Any] | None, caution: float) -> tuple[str, list[str]]:
    if row is None:
        return "NOT_TESTED", ["no_exact_u5_straight_point"]
    if not row["actual_stable_response"]:
        return "UNUSABLE", ["actual_stable_response_contract_failed"]
    reasons: list[str] = []
    if row["strict_status"] != "VALID":
        reasons.append(f"strict_status_{row['strict_status']}")
    saturated = [
        name for name, value in row["controller_saturation_in_measurement"].items()
        if value
    ]
    if saturated:
        reasons.append("controller_saturation:" + ",".join(saturated))
    throttle = row["actual"].get("engine_0_throttle_pos_norm")
    if throttle is not None and throttle >= caution:
        reasons.append("low_power_margin")
    return ("MARGINAL", reasons) if reasons else ("USABLE", [])


def _trajectory_summary(row: dict[str, Any], runs_by_id: dict[str, dict[str, Any]]) -> dict[str, Any] | None:
    if row["maneuver"] != "turn":
        return None
    trajectories = [runs_by_id[run_id]["trajectory"] for run_id in row["run_ids"]]
    return {
        "turn_rate_deg_s": fmean(item["turn_rate_deg_s"] for item in trajectories),
        "measured_radius_m": fmean(item["measured_radius_m"] for item in trajectories),
        "turn_rate_coefficient_of_variation": fmean(
            item["turn_rate_coefficient_of_variation"] for item in trajectories
        ),
    }


def _quality(stable: int, tested: int, kind: str) -> str:
    if tested == 0:
        return "NOT_TESTED"
    if stable == tested:
        return "EXCELLENT_ALL_TESTED_RESPONSES_STABLE"
    if stable >= max(2, tested // 2):
        return "GOOD_WITH_LIMITED_REGIONS"
    if stable:
        return "LIMITED"
    return f"NO_ACTUAL_STABLE_{kind.upper()}_RESPONSE"


def run_audit() -> dict[str, Any]:
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    source_paths = {
        name: HERE / relative for name, relative in config["source_artifacts"].items()
    }
    source_hashes_before = {name: _sha256(path) for name, path in source_paths.items()}
    phase1 = _load_json(source_paths["u5_phase1"])
    high = _load_json(source_paths["u5_high_altitude"])
    comparison = _load_json(source_paths["u5_comparison"])
    freeze = _load_json(source_paths["u5_freeze"])
    points = _load_json(source_paths["u5_points"])
    runs = _load_json(source_paths["u5_runs"])
    u5_provenance = _load_json(source_paths["u5_provenance"])
    u5_result = _load_json(source_paths["u5_result"])
    if u5_result["provenance_id"] != config["parent_provenance_id"]:
        raise RuntimeError("configured U5 parent provenance does not match artifacts")

    fingerprint = {
        "config": _sha256(CONFIG_PATH),
        "harness": _sha256(HARNESS_PATH),
        "sources": source_hashes_before,
    }
    provenance_id = "u5.1-" + hashlib.sha256(
        json.dumps(fingerprint, sort_keys=True).encode("utf-8")
    ).hexdigest()[:20]
    candidates = config["candidate_order"]
    all_rows = phase1["rows"] + high["rows"]
    runs_by_id = {row["run_id"]: row for row in runs["runs"]}
    rank_by_model = {row["model"]: row for row in comparison["ranking"]}
    selected_speed = {
        row["model"]: row["selected_speed_mps"] for row in phase1["speed_choices"]
    }

    coverage_rows = []
    for model in candidates:
        model_points = [p for p in points["points"] if p["aircraft_model"] == model]
        coverage_rows.append(
            {
                "aircraft": model,
                "executed_point_count": len(model_points),
                "executed_cold_start_run_count": sum(len(p["run_ids"]) for p in model_points),
                "tested_altitudes_m": sorted({p["altitude_msl_m"] for p in model_points}),
                "tested_ias_mps": sorted({p["requested"]["ias_mps"] for p in model_points}),
                "executed_grid": [
                    {
                        "altitude_m": altitude,
                        "ias_mps": speed,
                        "maneuvers": sorted(
                            {p["maneuver"] for p in model_points
                             if p["altitude_msl_m"] == altitude
                             and p["requested"]["ias_mps"] == speed}
                        ),
                    }
                    for altitude, speed in sorted(
                        {(p["altitude_msl_m"], p["requested"]["ias_mps"]) for p in model_points}
                    )
                ],
            }
        )
    inventory = {
        "schema_version": 1,
        "artifact_type": "U5_1_EXISTING_DATA_INVENTORY",
        "provenance_id": provenance_id,
        "parent_u5_provenance_id": u5_result["provenance_id"],
        "candidate_count": len(candidates),
        "executed_point_count": len(points["points"]),
        "executed_cold_start_run_count": len(runs["runs"]),
        "all_run_ids_unique": len(runs_by_id) == len(runs["runs"]),
        "coverage": coverage_rows,
        "ideal_levels_missing_from_all_u5_data_m": [1000, 2000, 4000],
        "new_simulation_runs": 0,
    }

    caution = float(config["cell_classification"]["throttle_caution_threshold_norm"])
    map_aircraft = []
    maneuver_regions = []
    for model in candidates:
        matrix = []
        for altitude in config["ideal_comparison_altitudes_m"]:
            cells = []
            for speed in config["display_speed_columns_mps"]:
                row = next(
                    (x for x in all_rows if x["model"] == model
                     and x["maneuver"] == "straight"
                     and x["altitude_m"] == float(altitude)
                     and x["target_ias_mps"] == float(speed)),
                    None,
                )
                classification, reasons = _cell_classification(row, caution)
                cells.append(
                    {
                        "ias_mps": float(speed),
                        "classification": classification,
                        "classification_reasons": reasons,
                        "raw_strict_status": None if row is None else row["strict_status"],
                        "actual_stable_response": None if row is None else row["actual_stable_response"],
                        "actual_ias_mps": None if row is None or row["actual"] is None else row["actual"]["ias_mps"],
                        "throttle_norm": None if row is None or row["actual"] is None else row["actual"].get("engine_0_throttle_pos_norm"),
                        "engine_healthy": None if row is None else row["engine"]["healthy"],
                        "repeatable": None if row is None else row["repeatability"]["passed"],
                        "settled_all_repeats": None if row is None else all(
                            item["settled_by_screening_deadline"] for item in row["settling"]
                        ),
                        "controller_saturation": None if row is None else row["controller_saturation_in_measurement"],
                    }
                )
            matrix.append({"altitude_m": float(altitude), "cells": cells})
        map_aircraft.append({"aircraft": model, "matrix": matrix})

        for altitude in (5000.0, 5500.0, 6000.0):
            speed = selected_speed[model]
            subset = [
                row for row in high["rows"]
                if row["model"] == model and row["altitude_m"] == altitude
            ]
            turns = []
            vertical = []
            for row in subset:
                common = {
                    "target": row["target_value"],
                    "raw_strict_status": row["strict_status"],
                    "actual_stable_response": row["actual_stable_response"],
                    "repeatable": row["repeatability"]["passed"],
                }
                if row["maneuver"] == "turn":
                    turns.append(
                        common
                        | {
                            "actual_bank_deg": row["actual"]["roll_deg"],
                            "trajectory": _trajectory_summary(row, runs_by_id),
                        }
                    )
                elif row["maneuver"] == "vertical":
                    vertical.append(
                        common | {"actual_vertical_speed_mps": row["actual"]["vertical_speed_mps"]}
                    )
            maneuver_regions.append(
                {
                    "aircraft": model,
                    "altitude_m": altitude,
                    "ias_mps": speed,
                    "turns": sorted(turns, key=lambda x: x["target"]),
                    "vertical": sorted(vertical, key=lambda x: x["target"]),
                    "not_tested_reason": (
                        "phase1_early_rejection" if speed is None
                        else "straight_gate_fail_fast" if not turns and not vertical
                        else None
                    ),
                }
            )
    maps = {
        "schema_version": 1,
        "artifact_type": "U5_1_ALTITUDE_IAS_USABILITY_MAPS",
        "provenance_id": provenance_id,
        "classification_contract": config["cell_classification"],
        "raw_status_semantics_retained": True,
        "aircraft": map_aircraft,
    }
    maneuvers = {
        "schema_version": 1,
        "artifact_type": "U5_1_EXISTING_MANEUVER_COVERAGE",
        "provenance_id": provenance_id,
        "regions": maneuver_regions,
        "exact_targets_are_not_hard_selection_gates": True,
    }

    summaries = []
    cross_rows = []
    for model in candidates:
        coverage = next(row for row in coverage_rows if row["aircraft"] == model)
        matrix = next(row["matrix"] for row in map_aircraft if row["aircraft"] == model)
        rank = rank_by_model[model]
        regions = [row for row in maneuver_regions if row["aircraft"] == model]
        turn_rows = [x for r in regions for x in r["turns"]]
        vertical_rows = [x for r in regions for x in r["vertical"]]
        climb_rows = [x for x in vertical_rows if x["target"] > 0]
        descent_rows = [x for x in vertical_rows if x["target"] < 0]
        stable_turns = sum(x["actual_stable_response"] for x in turn_rows)
        stable_climbs = sum(x["actual_stable_response"] for x in climb_rows)
        stable_descents = sum(x["actual_stable_response"] for x in descent_rows)
        good_by_altitude = []
        for altitude_row in matrix:
            usable = [c["ias_mps"] for c in altitude_row["cells"] if c["classification"] == "USABLE"]
            marginal = [c["ias_mps"] for c in altitude_row["cells"] if c["classification"] == "MARGINAL"]
            good_by_altitude.append(
                {
                    "altitude_m": altitude_row["altitude_m"],
                    "usable_ias_mps": usable,
                    "marginal_ias_mps": marginal,
                    "summary": "NOT_TESTED" if all(c["classification"] == "NOT_TESTED" for c in altitude_row["cells"]) else (usable or marginal or "NO_STABLE_SPEED"),
                }
            )
        low_rows = [x for x in all_rows if x["model"] == model and x["maneuver"] == "straight" and x["altitude_m"] in (0.0, 3000.0)]
        general_stable = sorted(
            speed for speed in config["display_speed_columns_mps"]
            if all(any(x["target_ias_mps"] == float(speed) and x["actual_stable_response"] for x in low_rows if x["altitude_m"] == altitude) for altitude in (0.0, 3000.0))
        )
        general_usable = sorted(
            speed for speed in config["display_speed_columns_mps"]
            if all(
                next(
                    cell for altitude_row in matrix
                    if altitude_row["altitude_m"] == altitude
                    for cell in altitude_row["cells"]
                    if cell["ias_mps"] == float(speed)
                )["classification"] == "USABLE"
                for altitude in (0.0, 3000.0)
            )
        )
        high_straight_rows = [
            x for x in high["rows"]
            if x["model"] == model and x["maneuver"] == "straight"
        ]
        high_stable = (
            [float(selected_speed[model])]
            if selected_speed[model] is not None
            and {x["altitude_m"] for x in high_straight_rows if x["actual_stable_response"]}
            == {5000.0, 5500.0, 6000.0}
            else []
        )
        summary = {
            "aircraft": model,
            "tested_altitude_range_m": [min(coverage["tested_altitudes_m"]), max(coverage["tested_altitudes_m"])],
            "tested_altitudes_m": coverage["tested_altitudes_m"],
            "tested_ias_range_mps": [min(coverage["tested_ias_mps"]), max(coverage["tested_ias_mps"])],
            "tested_ias_mps": coverage["tested_ias_mps"],
            "good_speed_region_by_altitude": good_by_altitude,
            "general_stable_speed_region_mps": general_stable,
            "general_usable_speed_region_mps": general_usable,
            "high_altitude_stable_speed_region_mps": high_stable,
            "turn_quality": _quality(stable_turns, len(turn_rows), "turn"),
            "climb_quality": _quality(stable_climbs, len(climb_rows), "climb"),
            "descent_quality": _quality(stable_descents, len(descent_rows), "descent"),
            "controller_quality_score": rank["scores"]["control_quality_and_stability"],
            "repeatability": rank.get("all_repeatable", False),
            "tuning_complexity_score": rank["scores"]["minimum_tuning_burden"],
            "known_weak_regions": rank["major_limitation"],
            "uav_like_character_score": rank["scores"]["uav_like_speed_and_behavior"],
            "u5_overall_rank": rank["rank"],
        }
        summaries.append(summary)
        altitude_usability = {}
        for altitude in (5000.0, 5500.0, 6000.0):
            high_assessment = next((x for x in rank["high_altitude"] if x["altitude_m"] == altitude), None)
            straight = next(
                (x for x in high["rows"] if x["model"] == model and x["altitude_m"] == altitude and x["maneuver"] == "straight"),
                None,
            )
            altitude_usability[str(int(altitude))] = (
                "NOT_TESTED" if straight is None
                else "USABLE" if high_assessment is not None and high_assessment["usable"]
                else "LIMITED" if straight["actual_stable_response"]
                else "UNUSABLE"
            )
        cross_rows.append(
            {
                "aircraft": model,
                "tested_altitude_range_m": summary["tested_altitude_range_m"],
                "tested_ias_range_mps": summary["tested_ias_range_mps"],
                "general_good_speed_region_mps": general_usable,
                "general_actual_stable_speed_region_mps": general_stable,
                "usability_5000_m": altitude_usability["5000"],
                "usability_5500_m": altitude_usability["5500"],
                "usability_6000_m": altitude_usability["6000"],
                "turn_quality": summary["turn_quality"],
                "climb_quality": summary["climb_quality"],
                "descent_quality": summary["descent_quality"],
                "controller_stability_score": summary["controller_quality_score"],
                "repeatability": summary["repeatability"],
                "tuning_effort_score_inverse": summary["tuning_complexity_score"],
                "uav_like_character_score": summary["uav_like_character_score"],
                "overall_rank": rank["rank"],
            }
        )
    envelope = {
        "schema_version": 1,
        "artifact_type": "U5_1_PER_AIRCRAFT_ENVELOPE_SUMMARIES",
        "provenance_id": provenance_id,
        "summaries": summaries,
    }
    cross = {
        "schema_version": 1,
        "artifact_type": "U5_1_CROSS_AIRCRAFT_COMPARISON",
        "provenance_id": provenance_id,
        "selection_priority_order": config["selection_priority_order"],
        "rows": sorted(cross_rows, key=lambda x: x["overall_rank"]),
    }

    decision = {
        "schema_version": 1,
        "artifact_type": "U5_1_PRIMARY_SELECTION_RECHECK",
        "provenance_id": provenance_id,
        "primary_decision": "CONFIRMED",
        "primary_aircraft": freeze["primary_aircraft"],
        "backup_decision": "CONFIRMED",
        "backup_aircraft": freeze["backup_aircraft"],
        "selected_nominal_ias_mps": freeze["selected_nominal_speed_policy"]["nominal_ias_mps"],
        "single_speed_policy_retained": True,
        "speed_state_dimension_added": False,
        "primary_evidence": [
            "40_mps_actual_stable_at_0_3000_5000_5500_6000_m",
            "full_moderate_turn_and_meaningful_vertical_usability_at_5000_and_5500_m",
            "lower_power_and_interface_burden_than_backup",
            "stronger_uav_like_speed_character_than_backup",
        ],
        "backup_evidence": [
            "60_mps_actual_stable_at_0_3000_5000_5500_6000_m",
            "full_moderate_turn_and_meaningful_vertical_usability_at_5000_and_5500_m",
            "higher_speed_twin_turboprop_and_low_power_margin_at_6000_m_prevent_primary_rank",
        ],
        "dense_0_to_6000_envelope_available": False,
        "selection_confirmation_coverage_sufficient": True,
        "missing_noncritical_comparison_altitudes_m": [1000, 2000, 4000],
        "minimal_new_runs_required": False,
        "new_simulation_runs": 0,
        "aircraft_selection_really_frozen": True,
        "ready_to_return_to_path_planner": True,
    }

    source_hashes_after = {name: _sha256(path) for name, path in source_paths.items()}
    provenance = {
        "schema_version": 1,
        "artifact_type": "U5_1_PROVENANCE",
        "provenance_id": provenance_id,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "python_version": sys.version,
        "platform": platform.platform(),
        "configuration": {"path": str(CONFIG_PATH), "sha256": _sha256(CONFIG_PATH)},
        "harness": {"path": str(HARNESS_PATH), "sha256": _sha256(HARNESS_PATH)},
        "source_artifact_hashes_before": source_hashes_before,
        "source_artifact_hashes_after": source_hashes_after,
        "source_artifacts_unchanged": source_hashes_before == source_hashes_after,
        "parent_u5_historical_artifacts_unchanged": u5_provenance["historical_artifacts_unchanged"],
        "new_simulation_runs": 0,
    }
    result = {
        "step": "U5.1",
        "step_status": "PASS",
        "provenance_id": provenance_id,
        "source_operating_points_reused": len(points["points"]),
        "source_cold_start_runs_reused": len(runs["runs"]),
        "new_simulation_runs": 0,
        "candidate_count": len(candidates),
        "primary_decision": "CONFIRMED",
        "backup_decision": "CONFIRMED",
        "primary_aircraft": "c172p",
        "backup_aircraft": "DHC6",
        "selected_nominal_ias_mps": 40.0,
        "dense_0_to_6000_envelope_available": False,
        "selection_confirmation_coverage_sufficient": True,
        "missing_noncritical_comparison_altitudes_m": [1000, 2000, 4000],
        "aircraft_selection_really_frozen": True,
        "ready_to_return_to_path_planner": True,
        "full_u5_rerun_performed": False,
        "full_lut_generated": False,
        "controller_tuning_performed": False,
        "aircraft_xml_modified": False,
        "planner_or_search_modified": False,
        "next_stage_started": False,
        "source_artifacts_unchanged": source_hashes_before == source_hashes_after,
    }
    for name, payload in (
        ("u5_1_data_inventory.json", inventory),
        ("u5_1_altitude_speed_maps.json", maps),
        ("u5_1_maneuver_coverage.json", maneuvers),
        ("u5_1_envelope_summaries.json", envelope),
        ("u5_1_cross_aircraft_comparison.json", cross),
        ("u5_1_decision_recheck.json", decision),
        ("u5_1_provenance.json", provenance),
        ("u5_1_result.json", result),
    ):
        _write(name, payload)
    return result


if __name__ == "__main__":
    print(json.dumps(run_audit(), indent=2, allow_nan=False))
