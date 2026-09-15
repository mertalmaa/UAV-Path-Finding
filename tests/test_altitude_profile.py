"""Independent contracts for the fixed-path altitude envelope solver."""
from __future__ import annotations

from itertools import product
import math
import unittest

from planner.altitude_profile import solve_lowest_altitude_profile


class AltitudeProfileTests(unittest.TestCase):
    def test_valley_descends_to_floor_and_climbs_before_ridge(self) -> None:
        result = solve_lowest_altitude_profile(
            [100, 20, 20, 20, 20, 100], [30] * 5, [40] * 5,
            start_altitude_msl_m=100, end_altitude_msl_m=100,
        )
        self.assertTrue(result.feasible)
        self.assertEqual(result.altitudes_msl_m, (100, 60, 20, 40, 70, 100))

    def test_asymmetric_and_variable_interval_budgets(self) -> None:
        result = solve_lowest_altitude_profile(
            [0] * 5, [10, 10, 20, 30], [40, 30, 20, 10],
            start_altitude_msl_m=100, end_altitude_msl_m=100,
        )
        self.assertEqual(result.altitudes_msl_m, (100, 60, 50, 70, 100))

    def test_short_dip_is_bridged_without_violating_rates(self) -> None:
        result = solve_lowest_altitude_profile(
            [100, 0, 100], [10, 10], [10, 10],
            start_altitude_msl_m=100, end_altitude_msl_m=100,
        )
        self.assertEqual(result.altitudes_msl_m, (100, 90, 100))

    def test_high_floor_remains_hard_and_climb_is_anticipated(self) -> None:
        result = solve_lowest_altitude_profile(
            [50, 50, 120, 50, 50], [35] * 4, [35] * 4,
            start_altitude_msl_m=50, end_altitude_msl_m=50,
        )
        self.assertEqual(result.altitudes_msl_m, (50, 85, 120, 85, 50))

    def test_insufficient_climb_rejects_exact_start(self) -> None:
        result = solve_lowest_altitude_profile(
            [0, 100], [20], [20],
            start_altitude_msl_m=50, end_altitude_msl_m=100,
        )
        self.assertFalse(result.feasible)
        self.assertEqual(result.status, "ENDPOINT_CONFLICT")
        self.assertEqual(result.failure_index, 0)
        self.assertEqual(result.altitudes_msl_m, ())

    def test_insufficient_descent_rejects_exact_end(self) -> None:
        result = solve_lowest_altitude_profile(
            [0, 0], [20], [20],
            start_altitude_msl_m=100, end_altitude_msl_m=50,
        )
        self.assertFalse(result.feasible)
        self.assertEqual(result.failure_index, 1)

    def test_endpoint_below_floor_is_infeasible(self) -> None:
        result = solve_lowest_altitude_profile(
            [100, 100], [100], [100],
            start_altitude_msl_m=99, end_altitude_msl_m=100,
        )
        self.assertFalse(result.feasible)
        self.assertEqual(result.status, "ENDPOINT_CONFLICT")

    def test_ceiling_conflict_after_rate_propagation(self) -> None:
        result = solve_lowest_altitude_profile(
            [0, 0, 100], [20, 20], [20, 20],
            start_altitude_msl_m=60, end_altitude_msl_m=100,
            ceiling_msl_m=[60, 79, 100],
        )
        self.assertFalse(result.feasible)
        self.assertEqual(result.status, "CEILING_CONFLICT")
        self.assertEqual(result.failure_index, 1)

    def test_ceiling_equal_to_envelope_is_allowed(self) -> None:
        result = solve_lowest_altitude_profile(
            [0, 0, 100], [20, 20], [20, 20],
            start_altitude_msl_m=60, end_altitude_msl_m=100,
            ceiling_msl_m=[60, 80, 100],
        )
        self.assertTrue(result.feasible)
        self.assertEqual(result.altitudes_msl_m, (60, 80, 100))

    def test_zero_budgets_require_level_profile(self) -> None:
        result = solve_lowest_altitude_profile(
            [0, 50, 0], [0, 0], [0, 0],
            start_altitude_msl_m=75, end_altitude_msl_m=75,
        )
        self.assertEqual(result.altitudes_msl_m, (75, 75, 75))
        conflict = solve_lowest_altitude_profile(
            [0, 50, 0], [0, 0], [0, 0],
            start_altitude_msl_m=75, end_altitude_msl_m=74,
        )
        self.assertFalse(conflict.feasible)

    def test_single_station_and_conflicting_endpoints(self) -> None:
        result = solve_lowest_altitude_profile(
            [10], [], [], start_altitude_msl_m=20, end_altitude_msl_m=20,
        )
        self.assertEqual(result.altitudes_msl_m, (20,))
        for start, end in ((19, 20), (20, 19)):
            with self.subTest(start=start, end=end):
                conflict = solve_lowest_altitude_profile(
                    [10], [], [],
                    start_altitude_msl_m=start, end_altitude_msl_m=end,
                )
                self.assertFalse(conflict.feasible)

    def test_below_sea_level_floor(self) -> None:
        result = solve_lowest_altitude_profile(
            [-100, -200, -100], [200, 200], [200, 200],
            start_altitude_msl_m=-100, end_altitude_msl_m=-100,
        )
        self.assertEqual(result.altitudes_msl_m, (-100, -200, -100))

    def test_endpoint_propagation_tolerates_only_arithmetic_roundoff(self) -> None:
        # 0.3 - 0.1 - 0.1 produces 0.09999999999999998. In the opposite
        # direction, 0.2 - 0.1 - 0.1 can similarly cross an exact boundary.
        result = solve_lowest_altitude_profile(
            [0, 0, 0, 0], [0.1] * 3, [0.1] * 3,
            start_altitude_msl_m=0.4, end_altitude_msl_m=0.1,
        )
        self.assertTrue(result.feasible)
        self.assertEqual(result.altitudes_msl_m[0], 0.4)
        self.assertEqual(result.altitudes_msl_m[-1], 0.1)
        insufficient = solve_lowest_altitude_profile(
            [0, 0], [0.1], [0.1],
            start_altitude_msl_m=0.2, end_altitude_msl_m=0.099999,
        )
        self.assertFalse(insufficient.feasible)

    def test_roundoff_allowance_does_not_relax_floor(self) -> None:
        result = solve_lowest_altitude_profile(
            [1, 1], [0], [0],
            start_altitude_msl_m=1 - 1e-10, end_altitude_msl_m=1,
        )
        self.assertFalse(result.feasible)

    def test_invalid_inputs_raise_value_error(self) -> None:
        cases = [
            {"floor_msl_m": []},
            {"floor_msl_m": [0]},
            {"max_climb_m": []},
            {"max_descent_m": [1, 1]},
            {"max_climb_m": [-1]},
            {"max_descent_m": [-1]},
            {"floor_msl_m": [0, math.nan]},
            {"max_climb_m": [math.inf]},
            {"max_descent_m": [math.nan]},
            {"start_altitude_msl_m": math.nan},
            {"end_altitude_msl_m": -math.inf},
            {"ceiling_msl_m": [1]},
            {"ceiling_msl_m": [1, math.nan]},
            {"floor_msl_m": None},
            {"max_climb_m": [None]},
        ]
        for overrides in cases:
            with self.subTest(overrides=overrides):
                arguments = dict(
                    floor_msl_m=[0, 0], max_climb_m=[1], max_descent_m=[1],
                    start_altitude_msl_m=0, end_altitude_msl_m=0,
                )
                arguments.update(overrides)
                with self.assertRaises(ValueError):
                    solve_lowest_altitude_profile(**arguments)

    def test_componentwise_minimum_against_exhaustive_small_problem(self) -> None:
        # Enumerate candidate profiles independently of the two-pass algorithm.
        # Integer data imply an integer envelope, so this fully covers each case.
        climb, descent = (1, 2), (2, 1)
        for floor in product(range(3), repeat=3):
            for start, end in product(range(3), repeat=2):
                with self.subTest(floor=floor, start=start, end=end):
                    feasible = []
                    for middle in range(3):
                        candidate = (start, middle, end)
                        if all(z >= lower for z, lower in zip(candidate, floor)) and all(
                            -descent[i] <= candidate[i + 1] - candidate[i] <= climb[i]
                            for i in range(2)
                        ):
                            feasible.append(candidate)
                    result = solve_lowest_altitude_profile(
                        floor, climb, descent,
                        start_altitude_msl_m=start, end_altitude_msl_m=end,
                    )
                    self.assertEqual(result.feasible, bool(feasible))
                    if feasible:
                        expected = tuple(min(values) for values in zip(*feasible))
                        self.assertEqual(result.altitudes_msl_m, expected)


if __name__ == "__main__":
    unittest.main()
