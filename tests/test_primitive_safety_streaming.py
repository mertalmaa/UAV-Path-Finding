"""PERF-9B equivalence tests for allocation-light primitive safety."""

import dataclasses
import math
import unittest

import numpy as np

from planner.config import DEFAULT_CONFIG
from planner.primitives import MotionPrimitive, PrimitiveSample, evaluate_primitive
from tests.primitive_safety_reference import evaluate_primitive_reference
from tests.test_current_contracts import NODATA, terrain_from_array


class StreamingPrimitiveSafetyTests(unittest.TestCase):
    def assert_samples_equal(self, actual: PrimitiveSample | None, expected: PrimitiveSample | None) -> None:
        self.assertEqual(actual is None, expected is None)
        if actual is None:
            return
        assert expected is not None
        for actual_value, expected_value in zip(
            (actual.x, actual.y, actual.altitude_msl, actual.terrain_elevation_msl, actual.agl_m),
            (expected.x, expected.y, expected.altitude_msl, expected.terrain_elevation_msl, expected.agl_m),
        ):
            if math.isnan(expected_value):
                self.assertTrue(math.isnan(actual_value))
            else:
                self.assertEqual(actual_value, expected_value)
        self.assertEqual(actual.valid, expected.valid)
        self.assertEqual(actual.reason, expected.reason)

    def assert_equivalent(self, start, primitive, terrain, config) -> None:
        expected = evaluate_primitive_reference(start, primitive, terrain, config)
        actual = evaluate_primitive(start, primitive, terrain, config)
        self.assertEqual(actual.valid, expected.valid)
        self.assertEqual(actual.reason, expected.reason)
        self.assertEqual(actual.sample_count, expected.sample_count)
        if math.isnan(expected.min_agl_m):
            self.assertTrue(math.isnan(actual.min_agl_m))
        else:
            self.assertEqual(actual.min_agl_m, expected.min_agl_m)
        self.assert_samples_equal(actual.min_agl_sample, expected.min_agl_sample)
        self.assert_samples_equal(actual.first_failure, expected.first_failure)

    def test_reference_equivalence_for_safety_cases(self) -> None:
        cfg = dataclasses.replace(DEFAULT_CONFIG, xy_resolution_m=30.0, min_agl_m=100.0, primitive_sample_spacing_m=15.0)
        flat = terrain_from_array(np.full((8, 20), 1000.0, dtype=np.float32), 30.0)
        x, y = flat.rowcol_to_xy(4, 2)
        self.assert_equivalent((x, y, 1200.0), MotionPrimitive("E", 0, 8, 0.0, 240.0, "level"), flat, cfg)

        varying_values = np.full((8, 20), 1000.0, dtype=np.float32)
        varying_values[4, 6:10] = 1140.0
        varying = terrain_from_array(varying_values, 30.0)
        x, y = varying.rowcol_to_xy(4, 2)
        self.assert_equivalent((x, y, 1200.0), MotionPrimitive("E", 0, 12, 0.0, 360.0, "level"), varying, cfg)

        ridge_values = np.full((8, 20), 1000.0, dtype=np.float32)
        ridge_values[4, 7] = 1180.0
        ridge = terrain_from_array(ridge_values, 30.0)
        x, y = ridge.rowcol_to_xy(4, 2)
        self.assert_equivalent((x, y, 1200.0), MotionPrimitive("E", 0, 12, 0.0, 360.0, "level"), ridge, cfg)

        nodata = terrain_from_array(np.full((8, 20), NODATA, dtype=np.float32), 30.0)
        x, y = nodata.rowcol_to_xy(4, 2)
        self.assert_equivalent((x, y, 1200.0), MotionPrimitive("E", 0, 8, 0.0, 240.0, "level"), nodata, cfg)

        x, y = varying.rowcol_to_xy(6, 2)
        self.assert_equivalent((x, y, 1500.0), MotionPrimitive("NE", -5, 10, -180.0, 300.0, "descent"), varying, cfg)

    def test_equal_agl_tie_keeps_first_sample(self) -> None:
        cfg = dataclasses.replace(DEFAULT_CONFIG, xy_resolution_m=30.0, min_agl_m=100.0, primitive_sample_spacing_m=15.0)
        terrain = terrain_from_array(np.full((4, 8), 1000.0, dtype=np.float32), 30.0)
        x, y = terrain.rowcol_to_xy(1, 1)
        result = evaluate_primitive((x, y, 1200.0), MotionPrimitive("E", 0, 4, 0.0, 120.0, "level"), terrain, cfg)
        self.assertEqual(result.min_agl_sample.index, 0)
