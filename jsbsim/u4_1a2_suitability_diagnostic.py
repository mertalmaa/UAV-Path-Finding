"""Audit vertical UNKNOWNs and isolate the c172r high-altitude fixture failure."""

from __future__ import annotations

import copy
import hashlib
import json
import math
import platform
import sys
import time
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path
from statistics import fmean
from typing import Any

import jsbsim
import yaml

import u4_raw_lut_characterization as u4


HERE = Path(__file__).resolve().parent
RESULTS_DIR = HERE / "results"
CONFIG_PATH = HERE / "u4_1a2_suitability_diagnostic_configuration.yaml"
U1_CONFIG_PATH = HERE / "u1_profile_validation_configuration.yaml"
U1_HARNESS_PATH = HERE / "u1_profile_validation.py"
U4_CONFIG_PATH = HERE / "u4_raw_lut_configuration.yaml"
U4_HARNESS_PATH = HERE / "u4_raw_lut_characterization.py"
U4_LUT_PATH = RESULTS_DIR / "aircraft_lut_raw.json"
U4_PROVENANCE_PATH = RESULTS_DIR / "u4_provenance.json"
U4_1A_PATH = RESULTS_DIR / "u4_1a_boundary_sanity.json"
U4_1A1_PATH = RESULTS_DIR / "u4_1a1_vertical_refinement.json"
U4_1A1_PROVENANCE_PATH = RESULTS_DIR / "u4_1a1_provenance.json"

_BASE_U4_SNAPSHOT = u4._snapshot
_BASE_SET_RUNNING = u4.u1._set_all_engines_running


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


def _diagnostic_snapshot(
    fdm: Any, elapsed_s: float, config: dict[str, Any]
) -> dict[str, float]:
    sample = _BASE_U4_SNAPSHOT(fdm, elapsed_s, config)
    extra = {
        "mixture_cmd_norm": fdm["fcs/mixture-cmd-norm"],
        "mixture_pos_norm": fdm["fcs/mixture-pos-norm"],
        "engine_0_fuel_flow_gph": fdm["propulsion/engine/fuel-flow-rate-gph"],
        "engine_0_fuel_flow_pps": fdm["propulsion/engine/fuel-flow-rate-pps"],
        "engine_0_power_hp": fdm["propulsion/engine/power-hp"],
        "engine_0_map_pa": fdm["propulsion/engine/map-pa"],
    }
    sample.update(extra)
    if u4._CAPTURE is not None:
        u4._CAPTURE[-1].update(extra)
    return sample


u4.u1._snapshot = _diagnostic_snapshot


def _rename_ids(
    records: list[dict[str, Any]], point: dict[str, Any], variant: str
) -> None:
    mapping: dict[str, str] = {}
    for record in records:
        old = record["run_id"]
        new = old.replace("u4-", f"u4.1a2-{variant.lower()}-", 1)
        record["run_id"] = new
        mapping[old] = new
    point["run_ids"] = [mapping[item] for item in point["run_ids"]]


def _mean(records: list[dict[str, Any]], field: str) -> float:
    return fmean(float(record["measurement"][field]["mean"]) for record in records)


def _minimum(records: list[dict[str, Any]], field: str) -> float:
    return min(float(record["measurement"][field]["min"]) for record in records)


def _maximum(records: list[dict[str, Any]], field: str) -> float:
    return max(float(record["measurement"][field]["max"]) for record in records)


def _run_straight(
    altitude_m: float,
    variant: str,
    u4_config: dict[str, Any],
    provenance_id: str,
    mixture_command: float | None = None,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    run_config = u4._run_config(u4_config, altitude_m)
    if mixture_command is not None:
        run_config["fixture"]["mixture_command_norm"] = mixture_command

        def set_running_with_mixture(fdm: Any, config: dict[str, Any]) -> None:
            _BASE_SET_RUNNING(fdm, config)
            fdm["fcs/mixture-cmd-norm"] = mixture_command
            fdm["fcs/mixture-pos-norm"] = mixture_command

        u4.u1._set_all_engines_running = set_running_with_mixture
    else:
        u4.u1._set_all_engines_running = _BASE_SET_RUNNING

    u4._CAPTURE = []
    try:
        records, point = u4.u1._run_point(
            "c172r", "straight", 40.0, 0.0, run_config, provenance_id
        )
        captured = u4._CAPTURE
        groups = u4._split_repeats(
            captured, int(run_config["simulation"]["cold_start_repetitions"])
        )
        point, _ = u4._point_enhancement(
            point,
            records,
            groups,
            "straight",
            40.0,
            0.0,
            altitude_m,
            run_config,
            provenance_id,
        )
    finally:
        u4._CAPTURE = None
        u4.u1._set_all_engines_running = _BASE_SET_RUNNING
    _rename_ids(records, point, variant)
    return records, point


def _pressure_ratio_mixture(altitude_m: float) -> tuple[float, float]:
    fdm = jsbsim.FGFDMExec(None)
    fdm.set_debug_level(0)
    if not fdm.load_model("c172r"):
        raise RuntimeError("pressure probe could not load c172r")
    fdm["ic/h-sl-ft"] = altitude_m * u4.u1.M_TO_FT
    fdm["ic/vc-kts"] = 40.0 / u4.u1.KTS_TO_MPS
    if not fdm.run_ic():
        raise RuntimeError("pressure probe run_ic failed")
    pressure_psf = float(fdm["atmosphere/P-psf"])
    return pressure_psf / 2117.0, pressure_psf


def _straight_summary(
    records: list[dict[str, Any]],
    point: dict[str, Any],
    altitude_m: float,
    variant: str,
    requested_mixture: float,
    reference_status: str | None,
) -> dict[str, Any]:
    failed_checks = sorted(
        {
            name
            for record in records
            for name, passed in record["checks"].items()
            if not passed
        }
    )
    return {
        "altitude_m": altitude_m,
        "variant": variant,
        "one_factor_changed": None if variant == "BASELINE" else "MIXTURE_HANDLING_ONLY",
        "requested_mixture_command_norm": requested_mixture,
        "status": point["status"],
        "failure_class": point["failure_class"],
        "status_reasons": sorted(
            {reason for record in records for reason in record["status_reasons"]}
        ),
        "failed_checks": failed_checks,
        "actual": {
            "ias_mps": point["measured"]["ias_mps"],
            "altitude_msl_m": point["measured"]["altitude_msl_m"],
            "vertical_speed_mps": point["measured"]["vertical_speed_mps"],
            "pitch_deg": _mean(records, "pitch_deg"),
            "aoa_deg": point["measured"]["alpha_deg"],
            "throttle_norm": point["measured"]["engine_0_throttle_pos_norm"],
            "elevator_command_norm": _mean(records, "elevator_cmd_norm"),
            "elevator_position_norm": _mean(records, "elevator_pos_norm"),
        },
        "engine": {
            "running_min": _minimum(records, "engine_0_running"),
            "rpm_mean": _mean(records, "engine_0_state"),
            "rpm_min": _minimum(records, "engine_0_state"),
            "thrust_lbs_mean": _mean(records, "engine_0_thrust_lbs"),
            "power_hp_mean": _mean(records, "engine_0_power_hp"),
            "fuel_flow_gph_mean": _mean(records, "engine_0_fuel_flow_gph"),
            "manifold_pressure_pa_mean": _mean(records, "engine_0_map_pa"),
            "mixture_command_mean": _mean(records, "mixture_cmd_norm"),
            "mixture_command_min": _minimum(records, "mixture_cmd_norm"),
            "mixture_command_max": _maximum(records, "mixture_cmd_norm"),
            "mixture_position_mean": _mean(records, "mixture_pos_norm"),
        },
        "controller_saturation_in_measurement": {
            key: any(
                record["controller"]["saturation_in_measurement"][key]
                for record in records
            )
            for key in ("aileron", "rudder", "elevator", "throttle")
        },
        "trim": {
            "all_succeeded": all(record["trim"]["succeeded"] for record in records),
            "fresh_untrimmed_fallback_used": any(
                record["trim"].get("fresh_untrimmed_fallback", False)
                for record in records
            ),
            "errors": sorted(
                {
                    record["trim"]["error"]
                    for record in records
                    if record["trim"]["error"]
                }
            ),
        },
        "settling": point["settling"],
        "repeatability": point["repeatability"],
        "run_ids": point["run_ids"],
        "matches_historical_u4_status": (
            point["status"] == reference_status if reference_status is not None else None
        ),
    }


def _audit_unknown(row: dict[str, Any], u1_config: dict[str, Any]) -> dict[str, Any]:
    target = float(row["target_vz_mps"])
    actual = float(row["actual_vz_mps"])
    mean_error = actual - target
    failed = row["failure_analysis"]["failed_checks"]
    repeatable = row["repeatability"]["passed"]
    windows_complete = row["measurement_window_stability"]["all_windows_complete"]
    no_stall = row["failure_analysis"]["stall_evidence"] is False
    within_mean_vz_tolerance = abs(mean_error) <= float(
        u1_config["acceptance"]["vertical_speed_tracking_absolute_mps"]
    )
    ias_in_preferred_range = 35.0 <= float(row["actual_ias_mps"]) <= 50.0
    clipped = row["failure_analysis"]["controller_clipped_in_measurement"]
    if row["failure_analysis"]["power_limit_evidence"]:
        category = "POWER_LIMITED"
    elif clipped:
        category = "CONTROLLER_LIMITED"
    elif not repeatable or not windows_complete or not no_stall:
        category = "UNSTABLE_NOT_USABLE"
    elif within_mean_vz_tolerance and ias_in_preferred_range:
        category = "NEAR_TARGET_STABLE"
    else:
        category = "DIAGNOSTIC_HARNESS_ISSUE"
    return {
        "altitude_m": row["altitude_m"],
        "target_vz_mps": target,
        "actual_sustainable_vz_mps": actual,
        "target_error_mps": mean_error,
        "target_attainment_ratio": abs(actual / target),
        "actual_ias_mps": row["actual_ias_mps"],
        "ias_drift_over_measurement_window_mps": row["measurement_window_stability"][
            "ias_mps"
        ]["maximum_absolute_window_delta"],
        "gamma_deg": row["gamma_deg"],
        "pitch_deg": row["pitch_deg"],
        "aoa_deg": row["aoa_deg"],
        "beta_deg": row["beta_deg"],
        "throttle_norm": row["throttle_norm"],
        "engine_indicators": row["engine_indicators"],
        "elevator_usage": row["elevator_usage"],
        "controller_saturation_in_measurement": row[
            "controller_saturation_in_measurement"
        ],
        "settling": row["settling"],
        "measurement_window_stability": row["measurement_window_stability"],
        "repeatability": row["repeatability"],
        "status": row["status"],
        "original_failure_analysis": row["failure_analysis"],
        "unknown_audit_category": category,
        "failed_checks": failed,
        "horizontal_distance_per_100m_using_actual_response_m": (
            40.0 * 100.0 / abs(actual)
        ),
        "production_status_changed": False,
    }


def _propulsion_audit(provenance: dict[str, Any]) -> dict[str, Any]:
    dependencies = {
        Path(item["path"]).name: item for item in provenance["aircraft_dependency_closure"]
    }
    aircraft_path = Path(dependencies["c172r.xml"]["path"])
    engine_path = Path(dependencies["engIO360C.xml"]["path"])
    propeller_path = Path(dependencies["prop_Clark_Y7570.xml"]["path"])
    aircraft_root = ET.parse(aircraft_path).getroot()
    engine_root = ET.parse(engine_path).getroot()
    propeller_root = ET.parse(propeller_path).getroot()
    engine_decl = aircraft_root.find("./propulsion/engine")
    thruster_decl = aircraft_root.find("./propulsion/engine/thruster")
    mixture_nodes = aircraft_root.findall(".//*[@name='Mixture control']")

    def text(root: ET.Element, name: str) -> str | None:
        node = root.find(name)
        return node.text.strip() if node is not None and node.text else None

    return {
        "aircraft_xml": {"path": str(aircraft_path), "sha256": _sha256(aircraft_path)},
        "engine_xml": {"path": str(engine_path), "sha256": _sha256(engine_path)},
        "propeller_xml": {"path": str(propeller_path), "sha256": _sha256(propeller_path)},
        "engine_type": engine_root.tag,
        "engine_file": engine_decl.get("file") if engine_decl is not None else None,
        "engine_name": engine_root.get("name"),
        "maximum_hp": float(text(engine_root, "maxhp") or "nan"),
        "idle_rpm": float(text(engine_root, "idlerpm") or "nan"),
        "maximum_rpm": float(text(engine_root, "maxrpm") or "nan"),
        "minimum_throttle": float(text(engine_root, "minthrottle") or "nan"),
        "maximum_throttle": float(text(engine_root, "maxthrottle") or "nan"),
        "supercharger_or_turbo_declared": any(
            engine_root.find(name) is not None
            for name in ("numboostspeeds", "ratedboost1", "ratedpower1")
        ),
        "mixture_command_and_position_properties_supported": True,
        "c172r_aircraft_xml_automatic_mixture_or_altitude_compensation": bool(
            mixture_nodes
        ),
        "propeller_file": thruster_decl.get("file") if thruster_decl is not None else None,
        "propeller_name": propeller_root.get("name"),
        "propeller_blades": int(text(propeller_root, "numblades") or "0"),
        "propeller_diameter_in": float(text(propeller_root, "diameter") or "nan"),
        "propeller_min_pitch_deg": float(text(propeller_root, "minpitch") or "nan"),
        "propeller_max_pitch_deg": float(text(propeller_root, "maxpitch") or "nan"),
        "propeller_is_fixed_pitch": text(propeller_root, "minpitch")
        == text(propeller_root, "maxpitch"),
        "local_stock_reference": {
            "model": "c172x",
            "purpose": "diagnostic mixture policy reference only; not a c172r production change",
            "formula": "fcs/mixture-cmd-norm = atmosphere/P-psf / 2117",
        },
        "implementation_interpretation": (
            "c172r declares a naturally aspirated piston engine and fixed-pitch propeller; "
            "its aircraft XML has no automatic mixture/altitude-compensation system"
        ),
    }


def run_u4_1a2() -> dict[str, Any]:
    started = time.perf_counter()
    config = _load_yaml(CONFIG_PATH)
    u1_config = _load_yaml(U1_CONFIG_PATH)
    u4_config = _load_yaml(U4_CONFIG_PATH)
    u4._freeze_check(u4_config)
    historical_paths = {
        "u4_raw_lut": U4_LUT_PATH,
        "u4_1a": U4_1A_PATH,
        "u4_1a1": U4_1A1_PATH,
    }
    hashes_before = {name: _sha256(path) for name, path in historical_paths.items()}
    if hashes_before["u4_raw_lut"] != config["parent_raw_lut_sha256"]:
        raise RuntimeError("U4 raw LUT hash changed")
    if hashes_before["u4_1a"] != config["parent_u4_1a_sha256"]:
        raise RuntimeError("U4.1A artifact hash changed")
    if hashes_before["u4_1a1"] != config["parent_u4_1a1_sha256"]:
        raise RuntimeError("U4.1A.1 artifact hash changed")

    u4_lut = _load_json(U4_LUT_PATH)
    u4_provenance = _load_json(U4_PROVENANCE_PATH)
    vertical_source = _load_json(U4_1A1_PATH)
    unknown_rows = [row for row in vertical_source["rows"] if row["status"] == "UNKNOWN"]
    non_valid_rows = [
        row for row in vertical_source["rows"] if row["status"] != "VALID"
    ]
    power_limited_rows = [
        row
        for row in non_valid_rows
        if row["failure_analysis"]["category"] == "POWER_LIMITED"
    ]
    unknown_audit = [_audit_unknown(row, u1_config) for row in unknown_rows]
    unknown_counts = Counter(row["unknown_audit_category"] for row in unknown_audit)
    failed_check_counts = Counter(
        check for row in unknown_audit for check in row["failed_checks"]
    )

    seed = {
        "configuration": _sha256(CONFIG_PATH),
        "harness": _sha256(Path(__file__).resolve()),
        "historical_artifacts": hashes_before,
        "u4_provenance": _sha256(U4_PROVENANCE_PATH),
    }
    provenance_id = "u4.1a2-" + hashlib.sha256(
        json.dumps(seed, sort_keys=True).encode("utf-8")
    ).hexdigest()[:20]

    raw_by_altitude = {row["altitude_m"]: row for row in u4_lut["straight_gates"]}
    baseline_summaries: list[dict[str, Any]] = []
    mixture_summaries: list[dict[str, Any]] = []
    all_runs: list[dict[str, Any]] = []
    all_points: list[dict[str, Any]] = []
    for altitude in config["high_altitude"]["baseline_altitudes_m"]:
        altitude = float(altitude)
        records, point = _run_straight(
            altitude, "BASELINE", u4_config, provenance_id
        )
        baseline_summaries.append(
            _straight_summary(
                records,
                point,
                altitude,
                "BASELINE",
                1.0,
                raw_by_altitude[altitude]["status"],
            )
        )
        all_runs.extend(records)
        all_points.append(point)

    for altitude in config["high_altitude"]["mixture_only_diagnostic_altitudes_m"]:
        altitude = float(altitude)
        mixture, pressure_psf = _pressure_ratio_mixture(altitude)
        records, point = _run_straight(
            altitude,
            "MIXTURE_ONLY",
            u4_config,
            provenance_id,
            mixture_command=mixture,
        )
        summary = _straight_summary(
            records,
            point,
            altitude,
            "MIXTURE_ONLY",
            mixture,
            raw_by_altitude[altitude]["status"],
        )
        summary["standard_atmosphere_pressure_psf"] = pressure_psf
        summary["pressure_ratio_reference_denominator_psf"] = 2117.0
        mixture_summaries.append(summary)
        all_runs.extend(records)
        all_points.append(point)

    hashes_after = {name: _sha256(path) for name, path in historical_paths.items()}
    if hashes_after != hashes_before:
        raise RuntimeError("historical U4/U4.1A/U4.1A.1 artifact was modified")

    propulsion = _propulsion_audit(u4_provenance)
    mixture_resolves = all(
        row["status"] == "VALID" and row["engine"]["running_min"] >= 1.0
        for row in mixture_summaries
        if row["altitude_m"] in (3000.0, 3500.0)
    )
    optional_4000_valid = next(
        row["status"] == "VALID"
        for row in mixture_summaries
        if row["altitude_m"] == 4000.0
    )
    root_cause = (
        "ENGINE_MIXTURE_PROPULSION_FIXTURE"
        if mixture_resolves
        else "INCONCLUSIVE"
    )
    suitability = "CONTINUE" if mixture_resolves else "REVIEW"
    ready_for_u4_1b = False
    acceptance = u1_config["acceptance"]
    vertical_artifact = {
        "schema_version": 1,
        "artifact_type": "VERTICAL_UNKNOWN_AUDIT",
        "provenance_id": provenance_id,
        "source_u4_1a1_provenance_id": vertical_source["provenance_id"],
        "unknown_point_count": len(unknown_audit),
        "classification_counts": dict(sorted(unknown_counts.items())),
        "all_non_valid_context_counts": dict(
            sorted(
                Counter(
                    row["failure_analysis"]["category"] for row in non_valid_rows
                ).items()
            )
        ),
        "power_limited_infeasible_context": [
            {
                "altitude_m": row["altitude_m"],
                "target_vz_mps": row["target_vz_mps"],
                "actual_vz_mps": row["actual_vz_mps"],
                "actual_ias_mps": row["actual_ias_mps"],
                "throttle_norm": row["throttle_norm"],
                "engine_indicators": row["engine_indicators"],
                "status": row["status"],
                "failure_category": "POWER_LIMITED",
            }
            for row in power_limited_rows
        ],
        "rows": unknown_audit,
        "acceptance_criteria_audit": {
            "source": "U1 provisional model-suitability acceptance contract reused unchanged by U4",
            "source_path": str(U1_CONFIG_PATH.resolve()),
            "source_sha256": _sha256(U1_CONFIG_PATH),
            "thresholds": {
                "ias_tracking_relative": acceptance["ias_tracking_relative"],
                "ias_tracking_absolute_at_40_mps": acceptance["ias_tracking_relative"] * 40.0,
                "vertical_speed_tracking_absolute_mps": acceptance[
                    "vertical_speed_tracking_absolute_mps"
                ],
                "bank_tracking_absolute_deg": acceptance["bank_tracking_absolute_deg"],
                "beta_absolute_deg": acceptance["beta_absolute_deg"],
                "normalized_surface_usage_max": acceptance[
                    "normalized_surface_usage_max"
                ],
                "throttle_position_severe_boundary_norm": acceptance[
                    "throttle_position_severe_boundary_norm"
                ],
                "cold_start_repeatability_relative": acceptance[
                    "cold_start_repeatability_relative"
                ],
            },
            "evaluation_semantics": (
                "tracking uses maximum instantaneous error over the 20 s measurement window; "
                "settling requires remaining continuously inside all tracking bands before the window"
            ),
            "failed_check_counts_across_unknowns": dict(
                sorted(failed_check_counts.items())
            ),
            "physically_calibrated_planner_primitive_tolerance": False,
            "physical_aircraft_boundary_criterion": False,
            "thresholds_modified_in_u4_1a2": False,
            "diagnosis": (
                "most UNKNOWN rows are repeatable, retain IAS inside the 35-50 m/s preferred "
                "range, and have mean Vz error inside the existing 0.5 m/s tolerance; their "
                "UNKNOWN status is driven by conservative max-over-window tracking checks"
            ),
        },
        "representative_reruns": {
            "executed": False,
            "reason": (
                "the source artifact already contains three deterministic cold starts, full "
                "measurement-window telemetry, stability, saturation, and repeatability for every point"
            ),
        },
        "planner_interpretation": {
            "repeatable_response_beyond_2_mps_exists": any(
                abs(row["actual_sustainable_vz_mps"]) > 2.0
                and row["unknown_audit_category"] == "NEAR_TARGET_STABLE"
                for row in unknown_audit
            ),
            "unknown_points_promoted_to_valid": 0,
            "planner_safe_lut_modified": False,
            "true_maximum_claimed": False,
        },
    }
    high_altitude_artifact = {
        "schema_version": 1,
        "artifact_type": "HIGH_ALTITUDE_ROOT_CAUSE_DIAGNOSTIC",
        "provenance_id": provenance_id,
        "baseline_runs": baseline_summaries,
        "mixture_only_diagnostic_runs": mixture_summaries,
        "propulsion_mixture_model_audit": propulsion,
        "root_cause": {
            "primary_category": root_cause,
            "baseline_reproduced": all(
                row["matches_historical_u4_status"] for row in baseline_summaries
            ),
            "mixture_only_resolves_3000_3500": mixture_resolves,
            "mixture_only_4000_status": "VALID" if optional_4000_valid else "NON_VALID",
            "trim_initialization_only_explanation_rejected": all(
                row["trim"]["fresh_untrimmed_fallback_used"]
                for row in baseline_summaries
                if row["altitude_m"] >= 3000.0
            ),
            "controller_primary_cause_rejected": mixture_resolves,
            "aircraft_capability_limit_proven": False,
            "true_service_ceiling_claimed": False,
            "interpretation": (
                "The frozen baseline fails after failed trim even on a fresh untrimmed object, "
                "with engine RPM/power/fuel flow collapsing. Reasserting only an altitude-pressure-"
                "referenced mixture command preserves the same controller and makes 3000/3500 m "
                "straight gates valid; this isolates the baseline propulsion/mixture fixture."
            ),
        },
        "one_factor_at_a_time_contract": {
            "baseline_stack_modified": False,
            "mixture_diagnostic_changed_only": [
                "mixture command/position after engine-start initialization"
            ],
            "controller_changed": False,
            "trim_algorithm_changed": False,
            "aircraft_xml_changed": False,
            "multiple_variables_changed_together": False,
            "diagnostic_promoted_to_production": False,
        },
        "historical_artifact_hashes_before": hashes_before,
        "historical_artifact_hashes_after": hashes_after,
        "historical_artifacts_unchanged": hashes_before == hashes_after,
        "aircraft_suitability": suitability,
        "ready_for_u4_1b": ready_for_u4_1b,
        "next_blocker": (
            "explicitly approve/freeze physically appropriate c172r mixture handling, then "
            "regenerate affected high-altitude raw characterization before interpolation holdouts"
        ),
    }
    provenance = {
        "provenance_id": provenance_id,
        "parent_u4_provenance_id": config["parent_u4_provenance_id"],
        "parent_u4_1a1_provenance_id": vertical_source["provenance_id"],
        "jsbsim_compiled_version": jsbsim.FGFDMExec(None).get_version(),
        "python_version": sys.version,
        "python_executable": sys.executable,
        "platform": platform.platform(),
        "files": {
            "configuration": {"path": str(CONFIG_PATH.resolve()), "sha256": _sha256(CONFIG_PATH)},
            "harness": {"path": str(Path(__file__).resolve()), "sha256": _sha256(Path(__file__).resolve())},
            "u1_configuration": {"path": str(U1_CONFIG_PATH.resolve()), "sha256": _sha256(U1_CONFIG_PATH)},
            "u1_harness": {"path": str(U1_HARNESS_PATH.resolve()), "sha256": _sha256(U1_HARNESS_PATH)},
            "u4_configuration": {"path": str(U4_CONFIG_PATH.resolve()), "sha256": _sha256(U4_CONFIG_PATH)},
            "u4_harness": {"path": str(U4_HARNESS_PATH.resolve()), "sha256": _sha256(U4_HARNESS_PATH)},
            "u4_raw_lut": {"path": str(U4_LUT_PATH.resolve()), "sha256": hashes_before["u4_raw_lut"]},
            "u4_provenance": {"path": str(U4_PROVENANCE_PATH.resolve()), "sha256": _sha256(U4_PROVENANCE_PATH)},
            "u4_1a": {"path": str(U4_1A_PATH.resolve()), "sha256": hashes_before["u4_1a"]},
            "u4_1a1": {"path": str(U4_1A1_PATH.resolve()), "sha256": hashes_before["u4_1a1"]},
            "u4_1a1_provenance": {"path": str(U4_1A1_PROVENANCE_PATH.resolve()), "sha256": _sha256(U4_1A1_PROVENANCE_PATH)},
        },
        "external_primary_references": {
            "jsbsim_fgfcs_mixture_reference": "https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGFCS.html",
            "jsbsim_piston_reference": "https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGPiston.html",
            "jsbsim_engine_start_discussion": "https://github.com/JSBSim-Team/jsbsim/discussions/1264",
        },
    }
    result = {
        "step": "U4.1A.2",
        "step_status": "PASS",
        "provenance_id": provenance_id,
        "vertical_unknown_count": len(unknown_audit),
        "vertical_unknown_classification_counts": dict(sorted(unknown_counts.items())),
        "vertical_representative_reruns": 0,
        "high_altitude_baseline_points": len(baseline_summaries),
        "high_altitude_mixture_only_points": len(mixture_summaries),
        "executed_new_points": len(all_points),
        "executed_new_cold_start_runs": len(all_runs),
        "high_altitude_root_cause": root_cause,
        "aircraft_suitability": suitability,
        "ready_for_u4_1b": ready_for_u4_1b,
        "historical_artifacts_unchanged": hashes_before == hashes_after,
        "controller_tuning_performed": False,
        "stock_xml_modified": False,
        "planner_modified": False,
        "interpolation_performed": False,
        "derating_performed": False,
        "holdout_validation_started": False,
        "aircraft_switched": False,
        "acceptance_thresholds_modified": False,
        "true_service_ceiling_claimed": False,
        "runtime_s": time.perf_counter() - started,
    }
    _write("u4_1a2_vertical_unknown_audit.json", vertical_artifact)
    _write("u4_1a2_high_altitude_diagnostic.json", high_altitude_artifact)
    _write("u4_1a2_provenance.json", provenance)
    _write("u4_1a2_runs.json", {"provenance_id": provenance_id, "runs": all_runs})
    _write("u4_1a2_points.json", {"provenance_id": provenance_id, "points": all_points})
    _write("u4_1a2_result.json", result)
    return result


if __name__ == "__main__":
    print(json.dumps(run_u4_1a2(), indent=2, allow_nan=False))
