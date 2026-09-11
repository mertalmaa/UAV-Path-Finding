"""Contract tests for the J3 fixed nominal-CAS selection stage."""

from __future__ import annotations

import importlib.util
import json
import sys
import unittest
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
SPEC = importlib.util.spec_from_file_location(
    "j3_nominal_cas_selection", HERE / "j3_nominal_cas_selection.py"
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class J3ContractTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = yaml.safe_load(
            (HERE / "j3_candidate_configuration.yaml").read_text(encoding="utf-8")
        )

    def test_candidate_and_altitude_grid(self) -> None:
        self.assertEqual(self.config["candidate_cas_kts"], [275.0, 305.0, 335.0, 365.0, 395.0])
        self.assertEqual(self.config["main_altitudes_msl_m"], list(range(1500, 6001, 500)))
        self.assertEqual(self.config["cold_start_repetitions_per_point"], 3)

    def test_scope_guards(self) -> None:
        scope = self.config["scope"]
        self.assertTrue(scope["nominal_cas_selection_only"])
        for key, value in scope.items():
            if key not in {"maneuver_family", "nominal_cas_selection_only"}:
                self.assertFalse(value, key)

    def test_afterburner_and_future_throttle_policy(self) -> None:
        policy = self.config["throttle_policy_baseline"]
        self.assertEqual(policy["afterburner"], "prohibited")
        self.assertFalse(policy["j3_defines_maximum_climb_throttle"])
        self.assertFalse(policy["j3_defines_descent_idle_floor"])

    def test_zero_wind_roundoff_normalization_is_bounded(self) -> None:
        record = {
            "checks": {"zero_wind": False, "finite_outputs": True},
            "measurement": {
                key: {"min": -1.5e-12, "max": 4.0e-14}
                for key in ("wind_north_fps", "wind_east_fps", "wind_down_fps")
            },
            "status": "UNKNOWN",
            "status_reasons": ["zero_wind"],
        }
        MODULE._normalize_zero_wind_roundoff(record, 1.0e-9)
        self.assertTrue(record["checks"]["zero_wind"])
        self.assertEqual(record["status"], "VALID")
        self.assertEqual(record["status_reasons"], [])

        real_wind = {
            "checks": {"zero_wind": False},
            "measurement": {
                key: {"min": 0.0, "max": 1.0e-3}
                for key in ("wind_north_fps", "wind_east_fps", "wind_down_fps")
            },
            "status": "UNKNOWN",
            "status_reasons": ["zero_wind"],
        }
        MODULE._normalize_zero_wind_roundoff(real_wind, 1.0e-9)
        self.assertFalse(real_wind["checks"]["zero_wind"])
        self.assertEqual(real_wind["status"], "UNKNOWN")

    def test_single_cas_failure_is_explicit(self) -> None:
        result = MODULE._select_candidate([])
        self.assertFalse(result["j3_pass"])
        self.assertIsNone(result["selected_nominal_cas_kts"])
        self.assertEqual(result["decision"], "single nominal CAS assumption invalid")

    def test_generated_artifacts_cover_the_requested_grid(self) -> None:
        results = HERE / "results"
        runs = json.loads((results / "j3_runs.json").read_text(encoding="utf-8"))
        points = json.loads((results / "j3_points.json").read_text(encoding="utf-8"))
        selection = json.loads((results / "j3_selection.json").read_text(encoding="utf-8"))
        self.assertEqual(len(runs), 150)
        self.assertEqual(len(points), 50)
        self.assertTrue(all(run["status"] == "VALID" for run in runs))
        self.assertTrue(all(point["status"] == "VALID" for point in points))
        self.assertEqual(selection["selected_nominal_cas_kts"], 305.0)
        self.assertTrue(selection["j3_pass"])

    def test_run_diagnostics_and_provenance_are_present(self) -> None:
        results = HERE / "results"
        runs = json.loads((results / "j3_runs.json").read_text(encoding="utf-8"))
        provenance = json.loads(
            (results / "j3_provenance.json").read_text(encoding="utf-8")
        )
        required_measurements = {
            "altitude_msl_m", "kcas", "tas_mps", "mach", "alpha_deg", "beta_deg",
            "gamma_deg", "throttle_pos_norm", "elevator_pos_norm", "load_factor_nz",
            "dynamic_pressure_pa", "p_deg_s", "q_deg_s", "r_deg_s",
        }
        self.assertTrue(required_measurements <= set(runs[0]["measurement"]))
        self.assertTrue(all(run["provenance_id"] == provenance["provenance_id"] for run in runs))
        self.assertEqual(provenance["parent_j2_provenance_id"], "j2-c25dabb783f4ac1ffdef")


if __name__ == "__main__":
    unittest.main()
