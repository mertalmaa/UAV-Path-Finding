"""Contract, evidence semantics, and artifact tests for J4B."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import sys
import unittest
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
SPEC = importlib.util.spec_from_file_location(
    "j4b_straight_vertical_capability", HERE / "j4b_straight_vertical_capability.py"
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class J4BCapabilityTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = yaml.safe_load(
            (HERE / "j4b_capability_configuration.yaml").read_text(encoding="utf-8")
        )
        results = HERE / "results"
        self.result = json.loads((results / "j4b_result.json").read_text(encoding="utf-8"))
        self.runs = json.loads((results / "j4b_runs.json").read_text(encoding="utf-8"))
        self.points = json.loads((results / "j4b_tested_points.json").read_text(encoding="utf-8"))
        self.boundaries = json.loads((results / "j4b_boundaries.json").read_text(encoding="utf-8"))

    def test_scope_grid_and_sweep_strategy(self) -> None:
        self.assertEqual(self.config["reference"]["nominal_cas_kts"], 305.0)
        self.assertEqual(self.config["reference"]["main_altitudes_msl_m"], list(range(0, 6001, 500)))
        self.assertEqual(self.config["sweep"]["coarse_step_deg"], 2.0)
        self.assertEqual(self.config["sweep"]["refinement_step_deg"], 0.5)
        self.assertFalse(self.config["sweep"]["search_guard_is_aircraft_limit"])
        scope = self.config["scope"]
        self.assertTrue(scope["straight_climb_descent_capability_only"])
        self.assertTrue(scope["full_main_altitude_grid"])
        for key, value in scope.items():
            if key not in {"straight_climb_descent_capability_only", "full_main_altitude_grid"}:
                self.assertFalse(value, key)

    def test_controller_architecture(self) -> None:
        audit = json.loads(
            (HERE / "results" / "j4b_controller_audit.json").read_text(encoding="utf-8")
        )
        self.assertTrue(audit["native_fcs_active"])
        self.assertEqual(audit["fbw_override"], 0.0)
        self.assertFalse(audit["direct_surface_position_commanded"])
        self.assertEqual(audit["command_limits"]["throttle_cmd_norm"], [0.0, 0.5])
        self.assertEqual(audit["classification_rule"]["tracking_failure_alone"], "UNKNOWN")

    def test_all_boundaries_are_repeatable_brackets(self) -> None:
        self.assertEqual(len(self.boundaries), 26)
        self.assertTrue(all(item["status"] == "BRACKETED" for item in self.boundaries))
        self.assertTrue(all(item["failure_classification"] == "thrust_limited" for item in self.boundaries))
        self.assertTrue(
            all(abs(item["boundary_bracket_deg"][1] - item["boundary_bracket_deg"][0]) == 0.5 for item in self.boundaries)
        )
        for item in self.boundaries:
            self.assertTrue(item["boundary_repeatability"]["validated"]["passed"])
            self.assertTrue(item["boundary_repeatability"]["infeasible"]["passed"])

    def test_result_counts_and_no_unknown(self) -> None:
        self.assertTrue(self.result["j4b_pass"])
        self.assertEqual(self.result["run_count"], 385)
        self.assertEqual(self.result["tested_point_count"], 281)
        self.assertEqual(self.result["boundary_critical_point_count"], 52)
        self.assertEqual(self.result["unknown_points"], [])
        self.assertEqual(self.result["status_counts"]["UNKNOWN"], 0)

    def test_tracking_failure_without_boundary_evidence_stays_unknown(self) -> None:
        source = next(run for run in self.runs if run["status"] == "VALID" and run["requested"]["gamma_deg"] > 0.0)
        probe = copy.deepcopy(source)
        probe["checks"]["cas_tracking"] = False
        probe["checks"]["settled_by_measurement_window"] = False
        probe["measurement"]["throttle_cmd_norm"].update(
            {"min": 0.25, "mean": 0.30, "max": 0.35, "end": 0.30}
        )
        probe["measurement"]["kcas"]["end"] = 290.0
        classified = MODULE._classify_run(probe, probe["requested"]["gamma_deg"], self.config)
        self.assertEqual(classified["status"], "UNKNOWN")
        self.assertEqual(classified["failure_classification"], "controller_limited")

    def test_physical_boundary_evidence_is_infeasible(self) -> None:
        run = next(item for item in self.runs if item["status"] == "INFEASIBLE")
        self.assertTrue(run["capability_evidence"]["reliable_setup_engine_fcs_and_outputs"])
        self.assertFalse(run["capability_evidence"]["cas_tracking_passed"])
        self.assertEqual(run["failure_classification"], "thrust_limited")

    def test_no_lookup_derating_or_planner_payload(self) -> None:
        self.assertTrue(all(point["derated_capability"] is None for point in self.points))
        self.assertTrue(all(point["raw_capability"] is None for point in self.points))
        self.assertFalse(self.config["scope"]["lookup_generation"])
        self.assertFalse(self.config["scope"]["planner_integration"])

    def test_provenance_hashes(self) -> None:
        provenance = json.loads(
            (HERE / "results" / "j4b_provenance.json").read_text(encoding="utf-8")
        )
        self.assertEqual(provenance["parent_j4a_provenance_id"], "j4a-2342f25aec9f6ff3314c")
        for item in provenance["files"].values():
            path = Path(item["path"])
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), item["sha256"])


if __name__ == "__main__":
    unittest.main()

