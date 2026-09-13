from __future__ import annotations

import hashlib
import json
import unittest
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"


class U1ProfileValidationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = yaml.safe_load(
            (HERE / "u1_profile_validation_configuration.yaml").read_text(encoding="utf-8")
        )
        cls.provenance = json.loads((RESULTS / "u1_provenance.json").read_text(encoding="utf-8"))
        cls.runs = json.loads((RESULTS / "u1_runs.json").read_text(encoding="utf-8"))
        cls.points = json.loads((RESULTS / "u1_points.json").read_text(encoding="utf-8"))
        cls.result = json.loads((RESULTS / "u1_result.json").read_text(encoding="utf-8"))

    def test_profile_is_operational_not_physical(self) -> None:
        profile = self.config["operational_profile"]
        self.assertEqual(profile["ias_range_mps"], [35.0, 50.0])
        self.assertEqual(profile["nominal_ias_mps"], 40.0)
        self.assertEqual(profile["maximum_absolute_bank_deg"], 25.0)
        self.assertEqual(profile["maximum_absolute_vertical_speed_mps"], 5.0)
        self.assertFalse(profile["values_are_aircraft_physical_limits"])
        self.assertTrue(profile["values_are_planner_operational_constraints"])

    def test_ias_contract_is_explicit(self) -> None:
        contract = self.provenance["ias_measurement_contract"]
        self.assertEqual(contract["jsbsim_source_property"], "velocities/vc-kts")
        self.assertEqual(contract["reported_field"], "ias_mps")
        self.assertIn("proxy", contract["interpretation"])
        self.assertEqual(contract["tas_source_property"], "velocities/vt-fps")

    def test_provenance_dependency_hashes_exist(self) -> None:
        self.assertTrue(self.provenance["provenance_id"].startswith("u1-"))
        for item in self.provenance["files"].values():
            self.assertEqual(len(item["sha256"]), 64)
            path = Path(item["path"])
            self.assertTrue(path.is_file())
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), item["sha256"])

    def test_fail_fast_gate_order(self) -> None:
        gates = self.result["executed_gates"]
        self.assertTrue(gates["dhc6_straight"])
        self.assertFalse(gates["dhc6_turn"])
        self.assertFalse(gates["dhc6_vertical"])
        self.assertTrue(gates["c182_backup"])

    def test_dhc6_rejection_evidence(self) -> None:
        point = next(
            item for item in self.points
            if item.get("aircraft_model", "DHC6") == "DHC6"
            and item["maneuver"] == "straight"
            and item["requested"]["ias_mps"] == 35.0
        )
        self.assertEqual(point["status"], "UNKNOWN")
        self.assertTrue(point["model_fixture_mismatch_evidence"])
        self.assertEqual(point["run_statuses"], ["UNKNOWN"] * 3)
        self.assertGreater(point["measured"]["ias_mps"], 42.0)
        self.assertLess(point["measured"]["vertical_speed_mps"], -4.0)
        self.assertEqual(point["measured"]["stall_indicator"], 0.0)
        self.assertEqual(point["measured"]["engine_0_throttle_pos_norm"], 0.0)

    def test_backup_matrix_and_decision(self) -> None:
        backup = [item for item in self.points if item.get("aircraft_model") == "c182"]
        self.assertEqual(len(backup), 4)
        for point in backup:
            self.assertEqual(len(point["run_ids"]), 3)
        climb = next(item for item in backup if item["maneuver"] == "vertical")
        self.assertEqual(climb["status"], "INFEASIBLE")
        self.assertAlmostEqual(climb["measured"]["vertical_speed_mps"], 5.3248, places=3)
        self.assertLess(climb["measured"]["ias_mps"], 38.0)
        self.assertEqual(climb["measured"]["engine_0_throttle_pos_norm"], 1.0)
        self.assertEqual(self.result["c182_backup_status"], "BACKUP_INCONCLUSIVE")

    def test_final_scope_and_status(self) -> None:
        self.assertTrue(self.result["step_pass"])
        self.assertEqual(self.result["step_status"], "PASS")
        self.assertEqual(self.result["dhc6_suitability"], "REJECTED_FOR_PROFILE")
        self.assertEqual(self.result["current_aircraft_status"], "INCONCLUSIVE")
        self.assertFalse(self.result["full_characterization_performed"])
        self.assertFalse(self.result["controller_tuning_performed"])
        self.assertFalse(self.result["planner_modified"])
        self.assertFalse(self.result["next_stage_started"])


if __name__ == "__main__":
    unittest.main()
