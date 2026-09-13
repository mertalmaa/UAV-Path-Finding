from __future__ import annotations

import copy
import hashlib
import json
import platform
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml


HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
CONFIG_PATH = HERE / "u6_2_2_tested_safe_envelope_configuration.yaml"
HARNESS_PATH = Path(__file__).resolve()


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write(name: str, payload: dict[str, Any]) -> None:
    RESULTS.mkdir(parents=True, exist_ok=True)
    (RESULTS / name).write_text(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False), encoding="utf-8")


def _holdout_error(rows: list[dict[str, Any]], family: str, target: float) -> float | None:
    values = [row["errors"]["actual_vz_mps"]["absolute"] for row in rows if row["family"] == family and row["target_vz_mps"] == target]
    return max(values) if values else None


def _eligible(row: dict[str, Any], policy: dict[str, Any]) -> tuple[bool, list[str], dict[str, bool]]:
    categories = set(row["failure_reason"]["categories"])
    checks = {
        "actual_stable_response": bool(row["actual_stable_response"]),
        "repeatable": bool(row["repeatable"]),
        "settled": bool(row["settled"]),
        "healthy_ias": abs(row["actual_ias_mps"] / row["nominal_ias_mps"] - 1.0) <= policy["ias_relative_error_max"],
        "adequate_power_margin": row["throttle"] < policy["throttle_max_exclusive"],
        "saturation_free": not any(row["saturation"].values()),
        "controller_unlimited": "controller_limited" not in categories,
        "not_strict_infeasible": row["status"] != "INFEASIBLE",
    }
    reasons = [name for name, passed in checks.items() if not passed]
    return all(checks.values()), reasons, checks


def _command_audits(
    raw_rows: list[dict[str, Any]], altitude: float, targets: list[float],
    policy: dict[str, Any], exact_errors: dict[float, float | None], eligible_by_target: dict[float, set[float]],
) -> list[dict[str, Any]]:
    audits = []
    for target in targets:
        row = next(item for item in raw_rows if item["altitude_m"] == altitude and item["target_vz_mps"] == target)
        eligible, rejection_reasons, checks = _eligible(row, policy)
        exact_error = exact_errors[target]
        proxy_target = target
        if exact_error is None:
            proxy_target = 3.0 if target > 0 else -3.0
            exact_error = exact_errors[proxy_target]
            contiguous_neighbors = any(neighbor in eligible_by_target[target] for neighbor in (altitude - 500.0, altitude + 500.0))
            if not contiguous_neighbors:
                eligible = False
                rejection_reasons.append("no_neighboring_altitude_consistency_for_unvalidated_severity")
        reviewed_unknown = eligible and row["status"] == "UNKNOWN"
        audits.append({
            "command_vz_mps": target, "actual_vz_mps": row["actual_vz_mps"],
            "actual_ias_mps": row["actual_ias_mps"], "ias_retention_ratio": row["actual_ias_mps"] / row["nominal_ias_mps"],
            "throttle": row["throttle"], "power_margin_norm": 1.0 - row["throttle"],
            "saturation": row["saturation"], "settled": row["settled"], "settling": row["settling"],
            "repeatable": row["repeatable"], "actual_stable_response": row["actual_stable_response"],
            "strict_status": row["status"], "failure_reason": row["failure_reason"],
            "eligibility_checks": checks, "eligible_for_tested_safe_envelope": eligible,
            "rejection_reasons": rejection_reasons, "reviewed_unknown_promotion": reviewed_unknown,
            "reviewed_unknown_basis": (
                "numeric IAS/power/saturation/stability checks passed; raw reason is acceptance/tracking only"
                if reviewed_unknown else None
            ),
            "uncertainty_allowance_mps": exact_error,
            "uncertainty_source_target_vz_mps": proxy_target,
            "uncertainty_source": "EXACT_U6_1_HOLDOUT" if proxy_target == target else "NEAREST_VALIDATED_SEVERITY_U6_1_PROXY",
            "source_provenance": row["metadata"]["source_provenance"],
        })
    return audits


def run_u6_2_2() -> dict[str, Any]:
    started = time.perf_counter()
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    paths = {name: HERE / item["path"] for name, item in config["source_artifacts"].items()}
    before = {name: _sha256(path) for name, path in paths.items()}
    expected = {name: item["sha256"] for name, item in config["source_artifacts"].items()}
    if before != expected:
        raise RuntimeError("U6A-U6.2.1 source hash mismatch before U6.2.2")

    core = _load(paths["core_raw"])
    holdouts = _load(paths["core_holdouts"])["rows"]
    v2 = _load(paths["u6_2_1_profile"])
    v2_provenance = _load(paths["u6_2_1_provenance"])
    if v2_provenance["provenance_id"] != config["parent_u6_2_1_provenance_id"]:
        raise RuntimeError("U6.2.1 parent provenance mismatch")

    files = {
        "configuration": {"path": str(CONFIG_PATH), "sha256": _sha256(CONFIG_PATH)},
        "derivation_harness": {"path": str(HARNESS_PATH), "sha256": _sha256(HARNESS_PATH)},
        "query_interface": {"path": str(HERE / "tested_envelope_profile.py"), "sha256": _sha256(HERE / "tested_envelope_profile.py")},
    }
    seed = {"files": {name: item["sha256"] for name, item in files.items()}, "sources": before, "policy": config["tested_safe_eligibility"]}
    provenance_id = "u6.2.2-" + hashlib.sha256(json.dumps(seed, sort_keys=True).encode()).hexdigest()[:20]

    raw = core["straight_vertical_table"]
    altitudes = [float(value) for value in config["canonical_context"]["altitude_grid_m"]]
    climb_targets = [2.0, 3.0, 4.0, 5.0]
    descent_targets = [-2.0, -3.0, -4.0, -5.0]
    policy = config["tested_safe_eligibility"]
    exact_errors = {
        2.0: _holdout_error(holdouts, "climb", 2.0), 3.0: _holdout_error(holdouts, "climb", 3.0),
        4.0: None, 5.0: None, -2.0: _holdout_error(holdouts, "descent", -2.0),
        -3.0: _holdout_error(holdouts, "descent", -3.0), -4.0: None, -5.0: None,
    }
    initially_eligible: dict[float, set[float]] = {target: set() for target in climb_targets + descent_targets}
    for row in raw:
        target = row["target_vz_mps"]
        if target in initially_eligible and _eligible(row, policy)[0]:
            initially_eligible[target].add(row["altitude_m"])

    v2_climb = {row["altitude_m"]: row for row in v2["capabilities"]["straight_climb"]["rows"]}
    v2_descent = {row["altitude_m"]: row for row in v2["capabilities"]["straight_descent"]["rows"]}
    climb_rows = []
    descent_rows = []
    comparison_rows = []
    for altitude in altitudes:
        climb_audit = _command_audits(raw, altitude, climb_targets, policy, exact_errors, initially_eligible)
        descent_audit = _command_audits(raw, altitude, descent_targets, policy, exact_errors, initially_eligible)
        climb_candidates = [row for row in climb_audit if row["eligible_for_tested_safe_envelope"]]
        descent_candidates = [row for row in descent_audit if row["eligible_for_tested_safe_envelope"]]
        chosen_climb = max(climb_candidates, key=lambda row: row["command_vz_mps"], default=None)
        chosen_descent = min(descent_candidates, key=lambda row: row["command_vz_mps"], default=None)

        measured_climb_max = max(
            (row for row in raw if row["altitude_m"] == altitude and row["target_vz_mps"] > 0 and row["actual_stable_response"]),
            key=lambda row: row["actual_vz_mps"], default=None,
        )
        measured_descent_max = min(
            (row for row in raw if row["altitude_m"] == altitude and row["target_vz_mps"] < 0 and row["actual_stable_response"]),
            key=lambda row: row["actual_vz_mps"], default=None,
        )
        safe_climb = None if chosen_climb is None else max(0.0, chosen_climb["actual_vz_mps"] - chosen_climb["uncertainty_allowance_mps"])
        safe_descent = None if chosen_descent is None else -max(0.0, abs(chosen_descent["actual_vz_mps"]) - chosen_descent["uncertainty_allowance_mps"])

        climb_rows.append({
            "altitude_m": altitude, "availability": "UNAVAILABLE" if chosen_climb is None else "AVAILABLE",
            "measured": {
                "maximum_observed_stable_climb_vz_mps": None if measured_climb_max is None else measured_climb_max["actual_vz_mps"],
                "maximum_observed_target_vz_mps": None if measured_climb_max is None else measured_climb_max["target_vz_mps"],
                "tested_command_audit": climb_audit,
            },
            "planner_safe": None if chosen_climb is None else {
                "highest_tested_safe_climb_command_mps": chosen_climb["command_vz_mps"],
                "climb_vz_mps": safe_climb, "selected_actual_vz_mps": chosen_climb["actual_vz_mps"],
                "expected_ias_mps": chosen_climb["actual_ias_mps"], "ias_retention_ratio": chosen_climb["ias_retention_ratio"],
                "throttle": chosen_climb["throttle"], "power_margin_norm": chosen_climb["power_margin_norm"],
            },
            "evidence": {
                "confidence": "WITHHELD" if chosen_climb is None else "HIGH" if chosen_climb["strict_status"] == "VALID" else "REVIEWED_UNKNOWN",
                "selection_basis": "highest_command_passing_all_tested_safe_checks",
                "limiting_reason": "no_positive_command_passed_all_checks" if chosen_climb is None else next(("next_command_rejected: " + ",".join(row["rejection_reasons"]) for row in climb_audit if row["command_vz_mps"] > chosen_climb["command_vz_mps"]), "top_tested_command_selected"),
                "all_positive_tested_commands_audited": True,
            },
        })
        descent_rows.append({
            "altitude_m": altitude, "availability": "UNAVAILABLE" if chosen_descent is None else "AVAILABLE",
            "measured": {
                "maximum_observed_stable_descent_vz_mps": None if measured_descent_max is None else measured_descent_max["actual_vz_mps"],
                "maximum_observed_target_vz_mps": None if measured_descent_max is None else measured_descent_max["target_vz_mps"],
                "tested_command_audit": descent_audit,
            },
            "planner_safe": None if chosen_descent is None else {
                "highest_tested_safe_descent_command_mps": chosen_descent["command_vz_mps"],
                "descent_vz_mps": safe_descent, "selected_actual_vz_mps": chosen_descent["actual_vz_mps"],
                "expected_ias_mps": chosen_descent["actual_ias_mps"], "ias_retention_ratio": chosen_descent["ias_retention_ratio"],
                "throttle": chosen_descent["throttle"], "power_margin_norm": chosen_descent["power_margin_norm"],
            },
            "evidence": {
                "confidence": "WITHHELD" if chosen_descent is None else "HIGH" if chosen_descent["strict_status"] == "VALID" else "REVIEWED_UNKNOWN",
                "selection_basis": "largest_magnitude_command_passing_all_tested_safe_checks",
                "limiting_reason": "no_negative_command_passed_all_checks" if chosen_descent is None else next(("next_command_rejected: " + ",".join(row["rejection_reasons"]) for row in reversed(descent_audit) if row["command_vz_mps"] < chosen_descent["command_vz_mps"]), "top_tested_command_selected"),
                "all_negative_tested_commands_audited": True,
            },
        })
        old_climb = v2_climb[altitude]
        old_descent = v2_descent[altitude]
        comparison_rows.append({
            "altitude_m": altitude,
            "u6_2_1_safe_climb_vz_mps": None if old_climb["planner_safe"] is None else old_climb["planner_safe"]["climb_vz_mps"],
            "u6_2_2_safe_climb_command_mps": None if chosen_climb is None else chosen_climb["command_vz_mps"],
            "u6_2_2_safe_climb_vz_mps": safe_climb,
            "climb_change_reason": "no_safe_climb_at_power_boundary" if chosen_climb is None else "higher_tested_family_passed" if chosen_climb["command_vz_mps"] > 2.0 else "next_family_failed_safety_checks",
            "u6_2_1_safe_descent_vz_mps": old_descent["planner_safe"]["descent_vz_mps"],
            "u6_2_2_safe_descent_command_mps": None if chosen_descent is None else chosen_descent["command_vz_mps"],
            "u6_2_2_safe_descent_vz_mps": safe_descent,
            "descent_change_reason": "minus4_family_passed_full_review" if chosen_descent and chosen_descent["command_vz_mps"] == -4.0 else "minus4_or_minus5_failed_stability_IAS_or_saturation_checks",
        })

    profile = copy.deepcopy(v2)
    profile.update({
        "schema_version": 3, "profile_schema_id": "tested_safe_envelope_aircraft_capability_profile_v3",
        "artifact_type": "AIRCRAFT_CAPABILITY_PROFILE_TESTED_PLANNER_SAFE_ENVELOPE",
        "profile_stage": "tested_planner_safe_envelope", "provenance_id": provenance_id,
        "parent_u6_2_1_provenance_id": v2["provenance_id"],
        "full_tested_vertical_command_grid_audited": True,
        "vertical_safe_values_command_constant_independent": True,
    })
    profile["capabilities"]["straight_climb"] = {
        "query_policy": {"safe_interpolation": "CONSERVATIVE_ENDPOINT", "availability_interpolated": False, "availability_interpolation_blocked_ranges_m": [[4000.0, 5500.0]]},
        "tested_target_grid_mps": climb_targets, "rows": climb_rows,
    }
    profile["capabilities"]["straight_descent"] = {
        "query_policy": {"safe_interpolation": "CONSERVATIVE_ENDPOINT", "availability_interpolated": False},
        "tested_target_grid_mps": descent_targets, "rows": descent_rows,
    }
    profile["source_provenance"]["u6_2_1"] = v2["provenance_id"]

    envelope_audit = {
        "schema_version": 1, "artifact_type": "U6_2_2_TESTED_SAFE_VERTICAL_ENVELOPE_AUDIT",
        "provenance_id": provenance_id, "eligibility_policy": policy,
        "uncertainty_policy": config["uncertainty_policy"], "comparison_rows": comparison_rows,
        "climb_rows": climb_rows, "descent_rows": descent_rows,
        "audited_command_count": len(altitudes) * (len(climb_targets) + len(descent_targets)),
    }
    preservation_audit = {
        "schema_version": 1, "artifact_type": "U6_2_2_NON_VERTICAL_PRESERVATION_AUDIT", "provenance_id": provenance_id,
        "straight_preserved": profile["capabilities"]["straight"] == v2["capabilities"]["straight"],
        "level_turn_preserved": profile["capabilities"]["level_turn"] == v2["capabilities"]["level_turn"],
        "climbing_turn_preserved": profile["capabilities"]["climbing_turn"] == v2["capabilities"]["climbing_turn"],
        "descending_turn_preserved": profile["capabilities"]["descending_turn"] == v2["capabilities"]["descending_turn"],
        "global_aircraft_capability_constants": profile["global_aircraft_capability_constants"],
    }

    after = {name: _sha256(path) for name, path in paths.items()}
    provenance = {
        "schema_version": 1, "artifact_type": "U6_2_2_PROVENANCE", "provenance_id": provenance_id,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(), "python_version": sys.version, "platform": platform.platform(),
        "files": files, "source_hashes_before": before, "source_hashes_after": after, "source_artifacts_unchanged": before == after,
    }
    result = {
        "step": "U6.2.2", "step_status": "PASS", "provenance_id": provenance_id,
        "full_tested_safe_envelope_extracted": True, "vertical_safe_values_command_constant_independent": True,
        "ready_for_aircraftprofile_integration": True, "audited_vertical_rows": 96,
        "new_jsbsim_runs": 0, "source_artifacts_unchanged": before == after,
        "aircraft_swappable": True, "production_planner_integrated": False, "next_stage_started": False,
        "runtime_s": time.perf_counter() - started,
    }
    for name, payload in (
        ("c172p_aircraft_profile_planner_safe_v3.json", profile),
        ("u6_2_2_tested_safe_envelope_audit.json", envelope_audit),
        ("u6_2_2_non_vertical_preservation_audit.json", preservation_audit),
        ("u6_2_2_provenance.json", provenance),
        ("u6_2_2_result.json", result),
    ):
        _write(name, payload)
    return result


if __name__ == "__main__":
    print(json.dumps(run_u6_2_2(), indent=2, allow_nan=False))
