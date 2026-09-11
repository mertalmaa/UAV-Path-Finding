"""Static and semantic checks for the J2 baseline harness."""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
MODULE_PATH = HERE / "j2_baseline_sanity.py"
SPEC = importlib.util.spec_from_file_location("j2_baseline_sanity", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class J2ContractTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = yaml.safe_load(
            (HERE / "j2_reference_configuration.yaml").read_text(encoding="utf-8")
        )

    def test_scope_is_fixed_and_not_a_sweep(self) -> None:
        condition = self.config["sanity_condition"]
        self.assertEqual(condition["speed_type"], "KCAS")
        self.assertEqual(condition["speed_value"], 335.0)
        self.assertEqual(condition["role"], "sanity_only_not_nominal_speed_selection")
        self.assertEqual(condition["representative_altitudes_msl_m"], [1500.0, 5000.0, 6000.0])

    def test_reference_configuration(self) -> None:
        mass = self.config["mass_and_fuel"]
        model = self.config["aircraft"]
        aircraft = self.config["configuration"]
        environment = self.config["environment"]
        self.assertEqual(model["expected_loaded_model_id"], "f16")
        self.assertEqual(model["expected_engine_model"], "F100-PW-229")
        self.assertEqual(model["expected_thruster_model"], "direct")
        self.assertEqual(model["expected_system_models"], ["pushback", "hook"])
        self.assertEqual(model["expected_fcs_name"], "F-16 FC")
        self.assertEqual(mass["external_tank_contents_lbs"], [0.0, 0.0])
        self.assertEqual(mass["expected_total_weight_lbs"], 20630.0)
        self.assertTrue(mass["freeze_fuel_during_replay"])
        self.assertEqual(aircraft["landing_gear_position_norm"], 0.0)
        self.assertEqual(aircraft["expected_speedbrake_position_norm"], 0.0)
        self.assertFalse(aircraft["afterburner_allowed"])
        self.assertEqual(environment["turbulence_type"], 0)
        self.assertEqual(
            [environment[f"wind_{axis}_fps"] for axis in ("north", "east", "down")],
            [0.0, 0.0, 0.0],
        )

    def test_three_state_semantics(self) -> None:
        classify = MODULE.classify_characterization_result
        self.assertEqual(
            classify(reliable_measurement=True, confirmed_unsustainable=False), "VALID"
        )
        self.assertEqual(
            classify(reliable_measurement=False, confirmed_unsustainable=False), "UNKNOWN"
        )
        self.assertEqual(
            classify(reliable_measurement=False, confirmed_unsustainable=True), "UNKNOWN"
        )
        self.assertEqual(
            classify(reliable_measurement=True, confirmed_unsustainable=True), "INFEASIBLE"
        )

    def test_j1_acceptance_values_preserved(self) -> None:
        acceptance = self.config["acceptance"]
        simulation = self.config["simulation"]
        self.assertEqual(simulation["cold_start_repetitions"], 3)
        self.assertEqual(simulation["steady_measurement_window_s"], 20.0)
        self.assertEqual(simulation["replay_duration_s"], 30.0)
        self.assertEqual(acceptance["cas_tracking_relative"], 0.02)
        self.assertEqual(acceptance["gamma_tracking_absolute_deg"], 0.5)
        self.assertEqual(acceptance["normalized_control_surface_usage_max"], 0.90)
        self.assertEqual(acceptance["key_metric_repeatability_relative_approx"], 0.01)


if __name__ == "__main__":
    unittest.main()
