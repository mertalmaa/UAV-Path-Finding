"""Contract, semantics, and artifact tests for the J4A sanity gate."""

from __future__ import annotations

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
    "j4a_straight_vertical_sanity", HERE / "j4a_straight_vertical_sanity.py"
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class J4ASanityTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = yaml.safe_load(
            (HERE / "j4a_sanity_configuration.yaml").read_text(encoding="utf-8")
        )
        results = HERE / "results"
        self.result = json.loads((results / "j4a_result.json").read_text(encoding="utf-8"))
        self.runs = json.loads((results / "j4a_runs.json").read_text(encoding="utf-8"))
        self.points = json.loads((results / "j4a_points.json").read_text(encoding="utf-8"))

    def test_scope_and_sanity_targets(self) -> None:
        reference = self.config["reference"]
        self.assertEqual(reference["nominal_cas_kts"], 305.0)
        self.assertEqual(reference["representative_altitudes_msl_m"], [0.0, 3000.0, 5000.0, 6000.0])
        self.assertEqual(reference["gamma_targets_deg"], {"level": 0.0, "modest_climb": 2.0, "modest_descent": -2.0})
        self.assertFalse(reference["gamma_targets_are_capability_limits"])
        scope = self.config["scope"]
        self.assertTrue(scope["straight_climb_descent_sanity_only"])
        for key, value in scope.items():
            if key != "straight_climb_descent_sanity_only":
                self.assertFalse(value, key)

    def test_artifact_counts_and_statuses(self) -> None:
        self.assertEqual(len(self.runs), 36)
        self.assertEqual(len(self.points), 12)
        self.assertTrue(all(run["status"] == "VALID" for run in self.runs))
        self.assertTrue(all(point["status"] == "VALID" for point in self.points))
        self.assertEqual(self.result["status_counts"], {"VALID": 12, "UNKNOWN": 0, "INFEASIBLE": 0})
        self.assertTrue(self.result["j4a_pass"])

    def test_sign_and_throttle_ordering(self) -> None:
        for run in self.runs:
            self.assertTrue(run["checks"]["gamma_sign"])
            self.assertTrue(run["checks"]["vertical_speed_sign"])
            self.assertTrue(run["checks"]["altitude_evolution_sign"])
        self.assertTrue(self.result["sign_validation_passed"])
        self.assertTrue(all(item["passed"] for item in self.result["throttle_ordering"]))

    def test_tracking_repeatability_and_no_saturation(self) -> None:
        self.assertTrue(self.result["repeatability_passed"])
        for run in self.runs:
            self.assertTrue(run["checks"]["cas_tracking"])
            self.assertTrue(run["checks"]["gamma_tracking"])
            self.assertTrue(run["checks"]["afterburner_off"])
            self.assertTrue(run["checks"]["control_surface_usage"])
            self.assertFalse(run["controller"]["elevator_command_saturated"])
            self.assertFalse(run["controller"]["throttle_command_saturated"])

    def test_diagnostic_schema_and_no_capability_output(self) -> None:
        required = {
            "gamma_deg", "kcas", "tas_mps", "mach", "altitude_msl_m",
            "vertical_speed_mps", "throttle_pos_norm", "alpha_deg", "beta_deg",
            "load_factor_nz", "pitch_deg", "p_deg_s", "q_deg_s", "r_deg_s",
            "elevator_pos_norm", "left_aileron_pos_norm", "rudder_pos_norm",
        }
        self.assertTrue(required <= set(self.runs[0]["measurement"]))
        self.assertTrue(all(point["raw_capability"] is None for point in self.points))
        self.assertTrue(all(point["derated_capability"] is None for point in self.points))

    def test_unknown_is_not_infeasible(self) -> None:
        self.assertEqual(
            MODULE.j2.classify_characterization_result(
                reliable_measurement=False, confirmed_unsustainable=False
            ),
            "UNKNOWN",
        )

    def test_provenance_hashes(self) -> None:
        provenance = json.loads(
            (HERE / "results" / "j4a_provenance.json").read_text(encoding="utf-8")
        )
        self.assertEqual(provenance["parent_j3_1_provenance_id"], "j3.1-7761df15481027018aac")
        for item in provenance["files"].values():
            path = Path(item["path"])
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), item["sha256"])


if __name__ == "__main__":
    unittest.main()

