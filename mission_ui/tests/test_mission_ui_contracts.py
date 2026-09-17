"""Fast contract tests for the mission UI adapter (no DEM, no browser).

Run from the repository root:  python -m pytest -q mission_ui/tests
"""
import io
import math

import numpy as np
import pytest
from PIL import Image

from mission_ui.server.planner_service import RequestError, _decimate_indices, _num
from mission_ui.server.terrain_tiles import encode_terrarium, tile_bounds_lonlat, _bilinear


def _decode(png: bytes) -> np.ndarray:
    a = np.asarray(Image.open(io.BytesIO(png))).astype(np.float64)
    return a[..., 0] * 256 + a[..., 1] + a[..., 2] / 256 - 32768


def test_terrarium_roundtrip_is_sub_decimetre():
    elev = np.linspace(-120.0, 4800.0, 256 * 256, dtype=np.float32).reshape(256, 256)
    back = _decode(encode_terrarium(elev))
    assert np.max(np.abs(back - elev)) < 0.01


def test_tile_bounds_cover_the_world_at_zoom_zero():
    w, s, e, n = tile_bounds_lonlat(0, 0, 0)
    assert (w, e) == (-180.0, 180.0)
    assert math.isclose(n, 85.0511, abs_tol=1e-3) and math.isclose(s, -85.0511, abs_tol=1e-3)


def test_bilinear_never_exceeds_source_range():
    rng = np.random.default_rng(0)
    arr = rng.uniform(0, 1000, size=(20, 20))
    fr = rng.uniform(-2, 22, size=500)
    fc = rng.uniform(-2, 22, size=500)
    out = _bilinear(arr, fr, fc)
    assert out.min() >= arr.min() - 1e-9 and out.max() <= arr.max() + 1e-9


def test_decimation_keeps_endpoints_and_extremes():
    idx = _decimate_indices(100_000, 5000, keep=[12_345, 99_998])
    assert idx[0] == 0 and idx[-1] == 99_999
    assert 12_345 in idx and 99_998 in idx
    assert len(idx) <= 5003 and np.all(np.diff(idx) > 0)


def test_short_series_is_not_decimated():
    assert list(_decimate_indices(10, 5000, keep=[3])) == list(range(10))


def test_number_validation_rejects_bad_input():
    with pytest.raises(RequestError):
        _num({"v": "abc"}, "v")
    with pytest.raises(RequestError):
        _num({"v": float("nan")}, "v")
    with pytest.raises(RequestError):
        _num({"v": -5}, "v", lo=0)
    assert _num({}, "v", 7.0) == 7.0
