from __future__ import annotations

import hashlib
import json
import unittest
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"


class U2CandidateScreeningTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = yaml.safe_load(
            (HERE / "u2_candidate_screening_configuration.yaml").read_text(encoding="utf-8")
        )
        cls.inventory = json.loads((RESULTS / "u2_inventory.json").read_text(encoding="utf-8"))
        cls.provenance = json.loads((RESULTS / "u2_provenance.json").read_text(encoding="utf-8"))
        cls.runs = json.loads((RESULTS / "u2_runs.json").read_text(encoding="utf-8"))
        cls.points = json.loads((RESULTS / "u2_points.json").read_text(encoding="utf-8"))
        cls.result = json.loads((RESULTS / "u2_result.json").read_text(encoding="utf-8"))

    def test_frozen_profile_and_scope(self) -> None:
        profile = self.config["operational_profile"]
        self.assertEqual(profile["ias_range_mps"], [35.0, 50.0])
        self.assertEqual(profile["nominal_ias_mps"], 40.0)
        self.assertEqual(profile["maximum_absolute_bank_deg"], 25.0)
        self.assertEqual(profile["maximum_absolute_vertical_speed_mps"], 5.0)
        self.assertFalse(profile["values_are_aircraft_physical_limits"])
        self.assertTrue(profile["values_are_planner_operational_constraints"])
        self.assertFalse(self.config["scope"]["controller_tuning"])
        self.assertFalse(self.config["scope"]["aircraft_xml_modification"])

    def test_complete_static_inventory(self) -> None:
        self.assertEqual(len(self.inventory), 60)
        self.assertEqual({row["static_decision"] for row in self.inventory}, {"KEEP", "REJECT_STATIC"})
        self.assertTrue(all(row["static_reason"] for row in self.inventory))
        self.assertEqual(sum(row["static_decision"] == "REJECT_STATIC" for row in self.inventory), 49)
        self.assertEqual(sum(row["static_decision"] == "KEEP" for row in self.inventory), 11)
        kept = {row["model"] for row in self.inventory if row["static_decision"] == "KEEP"}
        self.assertEqual(kept, set(self.config["dynamic_candidates"]))

    def test_u1_models_are_not_retested(self) -> None:
        prior = {row["model"]: row for row in self.inventory if row["model"] in {"DHC6", "c182"}}
        self.assertEqual(set(prior), {"DHC6", "c182"})
        self.assertTrue(all(row["static_group"] == "prior_u1_decision" for row in prior.values()))
        self.assertFalse(any(run["aircraft_model"] in prior for run in self.runs))

    def test_provenance_hashes(self) -> None:
        self.assertTrue(self.provenance["provenance_id"].startswith("u2-"))
        self.assertIn("_run_point", self.provenance["controller_source"])
        for item in self.provenance["files"].values():
            path = Path(item["path"])
            self.assertTrue(path.is_file())
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), item["sha256"])
        self.assertTrue(all(len(row["model_xml_sha256"]) == 64 for row in self.inventory))

    def test_fail_fast_counts_and_repeatability(self) -> None:
        counts = self.result["counts"]
        self.assertEqual(counts["dynamically_tested"], 11)
        self.assertEqual(counts["executed_points"], 17)
        self.assertEqual(counts["executed_cold_start_runs"], 51)
        self.assertTrue(all(len(point["run_statuses"]) == 3 for point in self.points))
        self.assertTrue(all(point["repeatability"]["passed"] for point in self.points))
        self.assertFalse(any(point["fixture_failure"] for point in self.points))

    def test_speed_and_climb_gates(self) -> None:
        self.assertEqual(self.result["gate_passers"]["speed"], ["c172r", "J3Cub"])
        self.assertEqual(self.result["gate_passers"]["climb"], [])
        climbs = {point["aircraft_model"]: point for point in self.points if point["gate"] == "climb"}
        self.assertEqual(set(climbs), {"c172r", "J3Cub"})
        self.assertTrue(all(point["status"] == "INFEASIBLE" for point in climbs.values()))
        self.assertTrue(all(point["measured"]["engine_0_throttle_pos_norm"] == 1.0 for point in climbs.values()))
        self.assertLess(climbs["c172r"]["measured"]["ias_mps"], 40.0)
        self.assertLess(climbs["J3Cub"]["measured"]["vertical_speed_mps"], 5.0)

    def test_turn_gate_skipped_and_no_stock_selection(self) -> None:
        self.assertFalse(any(point["gate"] == "turn" for point in self.points))
        self.assertEqual(self.result["ranking"], [])
        self.assertEqual(self.result["selected"], [])
        self.assertTrue(self.result["no_suitable_stock_model"])
        self.assertEqual(self.result["step_status"], "PASS")
        self.assertEqual(self.result["current_aircraft_status"], "NO SUITABLE STOCK JSBSIM MODEL")
        self.assertFalse(self.result["controller_tuning_performed"])
        self.assertFalse(self.result["planner_modified"])
        self.assertFalse(self.result["next_stage_started"])


if __name__ == "__main__":
    unittest.main()
