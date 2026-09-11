"""Contract and artifact tests for the J3.1 domain extension."""

from __future__ import annotations

import importlib.util
import hashlib
import json
import sys
import unittest
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
SPEC = importlib.util.spec_from_file_location(
    "j3_1_domain_extension", HERE / "j3_1_domain_extension.py"
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class J31ContractTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = yaml.safe_load(
            (HERE / "j3_1_domain_extension.yaml").read_text(encoding="utf-8")
        )

    def test_domain_and_new_only_grid(self) -> None:
        domain = self.config["domain"]
        self.assertEqual(domain["combined_main_altitudes_m"], list(range(0, 6001, 500)))
        self.assertEqual(domain["new_main_altitudes_m"], [0.0, 500.0, 1000.0])
        self.assertEqual(domain["new_midpoint_holdouts_m"], [250.0, 750.0, 1250.0])
        self.assertFalse(self.config["scope"]["rerun_existing_main_grid"])

    def test_candidate_formula_and_nominal_change_guards(self) -> None:
        self.assertEqual(self.config["candidate_cas_kts"], [275.0, 305.0, 335.0, 365.0, 395.0])
        self.assertEqual(self.config["prior_nominal_cas_kts"], 305.0)
        self.assertTrue(self.config["selection"]["reuse_parent_formula_and_tie_breakers"])
        self.assertFalse(self.config["selection"]["silent_nominal_cas_change_allowed"])
        self.assertFalse(self.config["selection"]["holdouts_used_for_selection"])

    def test_scope_guards(self) -> None:
        scope = self.config["scope"]
        self.assertTrue(scope["domain_extension_only"])
        for key, value in scope.items():
            if key != "domain_extension_only":
                self.assertFalse(value, key)

    def test_frozen_contract_records_extended_domain(self) -> None:
        contract = yaml.safe_load(
            (HERE / "characterization_contract.yaml").read_text(encoding="utf-8")
        )
        domain = contract["supported_characterization_altitude_domain"]
        self.assertEqual(domain["floor_msl_m"], 0.0)
        self.assertEqual(domain["ceiling_msl_m"], 6000.0)
        self.assertEqual(domain["main_grid_msl_m"], list(range(0, 6001, 500)))
        self.assertEqual(domain["j3_1_new_midpoint_holdouts_msl_m"], [250.0, 750.0, 1250.0])
        self.assertEqual(contract["fixed_test_context"]["reference_speed_value_kcas"], 305.0)

    def test_generated_artifacts(self) -> None:
        results = HERE / "results"
        result = json.loads((results / "j3_1_result.json").read_text(encoding="utf-8"))
        new_runs = json.loads(
            (results / "j3_1_new_main_runs.json").read_text(encoding="utf-8")
        )
        combined = json.loads(
            (results / "j3_1_combined_main_points.json").read_text(encoding="utf-8")
        )
        holdouts = json.loads(
            (results / "j3_1_holdout_validation.json").read_text(encoding="utf-8")
        )
        self.assertEqual(len(new_runs), 45)
        self.assertEqual(len(combined), 65)
        self.assertTrue(all(run["requested"]["altitude_msl_m"] in {0.0, 500.0, 1000.0} for run in new_runs))
        self.assertTrue(all(run["status"] == "VALID" for run in new_runs))
        self.assertEqual(len(holdouts), 3)
        self.assertFalse(any(item["selection_or_fitting_input"] for item in holdouts))
        self.assertFalse(any(item["local_250m_refinement_required"] for item in holdouts))
        self.assertTrue(result["j3_1_pass"])

    def test_parent_artifacts_were_reused_unchanged(self) -> None:
        provenance = json.loads(
            (HERE / "results" / "j3_1_provenance.json").read_text(encoding="utf-8")
        )
        for item in provenance["source_artifacts"].values():
            path = Path(item["path"])
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), item["sha256"])


if __name__ == "__main__":
    unittest.main()
