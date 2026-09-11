"""Contract, telemetry, semantics, and artifact tests for J5A."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import sys
import unittest
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
PROJECT_PATH = HERE.parent / "project.md"
sys.path.insert(0, str(HERE))
SPEC = importlib.util.spec_from_file_location(
    "j5a_level_turn_sanity", HERE / "j5a_level_turn_sanity.py"
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class J5ALevelTurnSanityTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = yaml.safe_load(
            (HERE / "j5a_level_turn_sanity_configuration.yaml").read_text(encoding="utf-8")
        )
        results = HERE / "results"
        self.result = json.loads((results / "j5a_result.json").read_text(encoding="utf-8"))
        self.runs = json.loads((results / "j5a_runs.json").read_text(encoding="utf-8"))
        self.points = json.loads((results / "j5a_points.json").read_text(encoding="utf-8"))
        self.comparisons = json.loads(
            (results / "j5a_left_right_comparison.json").read_text(encoding="utf-8")
        )

    def test_scope_targets_and_repetitions(self) -> None:
        reference = self.config["reference"]
        self.assertEqual(reference["nominal_cas_kts"], 305.0)
        self.assertEqual(reference["representative_altitudes_msl_m"], [0.0, 3000.0, 5000.0, 6000.0])
        self.assertEqual(
            {name: item["target_bank_deg"] for name, item in reference["maneuvers"].items()},
            {"straight": 0.0, "modest_left": -20.0, "modest_right": 20.0},
        )
        self.assertEqual(reference["cold_start_repetitions_per_point"], 3)
        self.assertFalse(reference["targets_are_capability_limits"])
        scope = self.config["scope"]
        self.assertTrue(scope["level_turn_sanity_only"])
        self.assertTrue(scope["representative_altitudes_only"])
        for key, value in scope.items():
            if key not in {"level_turn_sanity_only", "representative_altitudes_only"}:
                self.assertFalse(value, key)

    def test_controller_architecture_and_no_surface_forcing(self) -> None:
        audit = json.loads(
            (HERE / "results" / "j5a_controller_audit.json").read_text(encoding="utf-8")
        )
        self.assertEqual(audit["turn_target_quantity"], "bank_angle_deg")
        self.assertFalse(audit["heading_rate_commanded"])
        self.assertEqual(audit["lateral_stick_equivalent_command"], "fcs/aileron-cmd-norm")
        self.assertTrue(audit["rudder_managed_by_native_fcs"])
        self.assertTrue(audit["native_fcs_active"])
        self.assertEqual(audit["fbw_override"], 0.0)
        self.assertFalse(audit["direct_surface_position_commanded"])
        self.assertEqual(audit["command_limits"]["throttle_cmd_norm"], [0.0, 0.5])

    def test_result_gate_and_status_counts(self) -> None:
        self.assertTrue(self.result["j5a_pass"])
        self.assertIsNone(self.result["j5b_blocker"])
        self.assertEqual(self.result["run_count"], 36)
        self.assertEqual(self.result["point_count"], 12)
        self.assertEqual(
            self.result["status_counts"],
            {"VALID": 12, "INFEASIBLE": 0, "UNKNOWN": 0},
        )
        self.assertTrue(all(run["status"] == "VALID" for run in self.runs))

    def test_turn_sign_tracking_radius_and_theory_cross_check(self) -> None:
        tolerance = self.config["acceptance"]["theory_radius_relative_difference_max"]
        for point in self.points:
            values = point["measured_values"]
            target_bank = point["requested_target"]["bank_deg"]
            self.assertLessEqual(values["maximum_cas_error_relative"], 0.02)
            self.assertLessEqual(values["maximum_gamma_error_deg"], 0.5)
            self.assertLessEqual(values["maximum_bank_error_deg"], 1.0)
            if target_bank == 0.0:
                self.assertIsNone(values["measured_radius_m"])
                self.assertLessEqual(abs(values["turn_rate_deg_s"]), 0.10)
            else:
                self.assertGreater(values["actual_bank_deg"] * target_bank, 0.0)
                self.assertGreater(values["turn_rate_deg_s"] * target_bank, 0.0)
                self.assertGreater(values["measured_radius_m"], 0.0)
                self.assertGreater(values["theoretical_radius_m"], 0.0)
                self.assertLessEqual(values["radius_theory_relative_difference"], tolerance)
                self.assertLessEqual(values["turn_rate_stability_relative"], 0.05)

    def test_beta_coordination_and_left_right_consistency(self) -> None:
        self.assertEqual(len(self.comparisons), 4)
        self.assertTrue(all(comparison["passed"] for comparison in self.comparisons))
        for comparison in self.comparisons:
            beta = comparison["comparisons"]["beta"]
            self.assertTrue(beta["mirrored_sign_or_both_near_zero"])
            self.assertFalse(beta["operational_limit_claimed"])
            self.assertLessEqual(
                comparison["comparisons"]["measured_radius_m"]["relative_magnitude_difference"],
                0.10,
            )
            self.assertLessEqual(
                comparison["comparisons"]["turn_rate_deg_s"]["relative_magnitude_difference"],
                0.10,
            )

    def test_repeatability_and_no_measurement_saturation(self) -> None:
        for point in self.points:
            self.assertEqual(len(point["diagnostics"]["run_ids"]), 3)
            self.assertTrue(point["diagnostics"]["repeatability"]["passed"])
        for run in self.runs:
            self.assertFalse(any(run["controller"]["command_saturation_in_measurement"].values()))
            self.assertLessEqual(run["tracking"]["maximum_normalized_control_surface_usage"], 0.90)

    def test_run_diagnostic_schema(self) -> None:
        required_measurement = {
            "roll_deg", "heading_deg", "heading_rate_deg_s", "kcas", "tas_mps",
            "mach", "altitude_msl_m", "gamma_deg", "vertical_speed_mps",
            "throttle_cmd_norm", "throttle_pos_norm", "alpha_deg", "beta_deg",
            "load_factor_nz", "pitch_deg", "p_deg_s", "q_deg_s", "r_deg_s",
            "aileron_cmd_norm", "elevator_cmd_norm", "rudder_cmd_norm",
            "left_aileron_pos_norm", "right_aileron_pos_norm", "rudder_pos_norm",
            "elevator_pos_norm",
        }
        for run in self.runs:
            self.assertTrue(run["trim"]["succeeded"])
            self.assertTrue(required_measurement.issubset(run["measurement"]))
            self.assertIn("measured_trajectory_radius_m", run["trajectory"])
            self.assertIn("theoretical_coordinated_radius_m", run["trajectory"])
            self.assertIn("settled_at_s", run["settling_and_measurement"])
            self.assertIn("provenance_id", run)

    def test_three_state_semantics_and_no_capability_payload(self) -> None:
        self.assertEqual(MODULE.classify_turn_sanity_result(True, False), "VALID")
        self.assertEqual(MODULE.classify_turn_sanity_result(False, False), "UNKNOWN")
        self.assertEqual(MODULE.classify_turn_sanity_result(False, True), "UNKNOWN")
        self.assertEqual(MODULE.classify_turn_sanity_result(True, True), "INFEASIBLE")
        self.assertEqual(self.config["status_semantics"]["modest_target_failure_default"], "UNKNOWN")
        self.assertTrue(all(point["raw_capability"] is None for point in self.points))
        self.assertTrue(all(point["derated_capability"] is None for point in self.points))
        self.assertFalse(self.config["scope"]["lookup_generation"])
        self.assertFalse(self.config["scope"]["planner_integration"])

    def test_provenance_hashes_and_project_decisions(self) -> None:
        provenance = json.loads(
            (HERE / "results" / "j5a_provenance.json").read_text(encoding="utf-8")
        )
        self.assertEqual(provenance["parent_j4b_provenance_id"], "j4b-9fa3129fabfcda51c7a7")
        for item in provenance["files"].values():
            path = Path(item["path"])
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), item["sha256"])
        project = PROJECT_PATH.read_text(encoding="utf-8")
        self.assertIn("J4B/J5A Offline Aircraft Characterization", project)
        self.assertIn("J5A: PASS", project)


if __name__ == "__main__":
    unittest.main()
