"""Conservative terrain/AGL coverage for continuous physical trajectories.

Physical trajectory stays authoritative. Raster cells are only an evaluation
coverage representation: no x/y pose is snapped or reconstructed. Current XY
geometry is straight or circular, so every sample pair supplies enough actual
path information to bound curve-to-chord deviation with circular sagitta.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, Optional, Tuple

import numpy as np

from planner.physical import PhysicalTrajectory, TrajectorySample
from planner.terrain import TerrainQuery

Bounds = Tuple[float, float, float, float]
_EPS = 1e-9


@dataclass(frozen=True)
class TrajectorySafetySample:
    index: int
    x_m: float
    y_m: float
    z_msl_m: float
    terrain_elevation_msl: float
    agl_m: float
    reason: str


@dataclass(frozen=True)
class CoveredTerrainCell:
    segment_index: int
    row: int
    col: int
    terrain_elevation_msl: float
    minimum_aircraft_z_msl: float
    agl_m: float
    curve_to_chord_deviation_m: float


@dataclass(frozen=True)
class TrajectorySafetyResult:
    is_safe: bool
    min_agl_m: float
    first_failure_sample: Optional[TrajectorySafetySample]
    first_failure_cell: Optional[CoveredTerrainCell]
    failure_reason: Optional[str]
    sample_count: int
    observed_max_sample_spacing_m: float
    required_max_sample_spacing_m: float
    sampling_sufficient: bool
    covered_cell_count: int
    terrain_cell_evaluations: int
    max_curve_to_chord_deviation_m: float
    lateral_buffer_m: float


@dataclass(frozen=True)
class BufferedTerrainField:
    lateral_buffer_m: float
    elevation_msl: np.ndarray
    valid: np.ndarray
    outside_dem: np.ndarray


class TerrainInfluenceCache:
    """Reusable precomputed terrain dilations keyed by lateral-buffer value."""

    def __init__(self, terrain: TerrainQuery):
        self.terrain = terrain
        self._fields: Dict[float, BufferedTerrainField] = {}

    def field(self, lateral_buffer_m: float) -> BufferedTerrainField:
        buffer_m = _nonnegative_finite(lateral_buffer_m, "lateral_buffer_m")
        if buffer_m not in self._fields:
            self._fields[buffer_m] = _build_buffered_field(self.terrain, buffer_m)
        return self._fields[buffer_m]


def _nonnegative_finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")
    return value


def _validate_bounds(bounds: Bounds) -> Bounds:
    xmin, ymin, xmax, ymax = (float(v) for v in bounds)
    if not all(math.isfinite(v) for v in (xmin, ymin, xmax, ymax)) or not xmin < xmax or not ymin < ymax:
        raise ValueError("planning_bounds must be finite with positive extent")
    return xmin, ymin, xmax, ymax


def _in_bounds(x_m: float, y_m: float, bounds: Bounds) -> bool:
    xmin, ymin, xmax, ymax = bounds
    return xmin <= x_m < xmax and ymin <= y_m < ymax


def _offsets_for_buffer(dx: float, dy: float, buffer_m: float) -> Iterable[Tuple[int, int]]:
    if buffer_m == 0.0:
        return ((0, 0),)
    radius = math.ceil(buffer_m / min(dx, dy)) + 1
    offsets = []
    for dr in range(-radius, radius + 1):
        for dc in range(-radius, radius + 1):
            gap_x = max(0.0, (abs(dc) - 1) * dx)
            gap_y = max(0.0, (abs(dr) - 1) * dy)
            if math.hypot(gap_x, gap_y) <= buffer_m + _EPS:
                offsets.append((dr, dc))
    return tuple(offsets)


def _slices(length: int, offset: int) -> Tuple[slice, slice]:
    return (slice(0, length - offset), slice(offset, length)) if offset >= 0 else \
        (slice(-offset, length), slice(0, length + offset))


def _build_buffered_field(terrain: TerrainQuery, lateral_buffer_m: float) -> BufferedTerrainField:
    """Precompute maximum terrain within buffer distance of each cell footprint.

    NoData and buffer regions extending beyond the DEM invalidate the target
    cell. This is a fail-closed static transformation and can be reused for
    all trajectories sharing the same terrain/buffer pair.
    """
    transform = terrain.roi.transform
    if abs(transform.b) > _EPS or abs(transform.d) > _EPS:
        raise ValueError("buffered terrain requires an axis-aligned DEM transform")
    dx, dy = (float(v) for v in terrain.roi.resolution)
    offsets = tuple(_offsets_for_buffer(dx, dy, lateral_buffer_m))
    source = np.asarray(terrain.roi.elevation, dtype=np.float64)
    source_valid = np.isfinite(source)
    if terrain.roi.nodata is not None:
        source_valid &= source != terrain.roi.nodata
    height, width = source.shape
    maximum = np.full(source.shape, -np.inf, dtype=np.float64)
    count = np.zeros(source.shape, dtype=np.int16)
    invalid = np.zeros(source.shape, dtype=bool)
    for dr, dc in offsets:
        dst_r, src_r = _slices(height, dr)
        dst_c, src_c = _slices(width, dc)
        values, valid = source[src_r, src_c], source_valid[src_r, src_c]
        maximum[dst_r, dst_c] = np.maximum(maximum[dst_r, dst_c], np.where(valid, values, -np.inf))
        count[dst_r, dst_c] += 1
        invalid[dst_r, dst_c] |= ~valid
    outside_dem = count != len(offsets)
    return BufferedTerrainField(lateral_buffer_m, maximum, ~outside_dem & ~invalid, outside_dem)


def _heading_delta_rad(first: float, second: float) -> float:
    return math.radians((second - first + 180.0) % 360.0 - 180.0)


def curve_to_chord_deviation_m(first: TrajectorySample, second: TrajectorySample) -> float:
    """Actual-segment circular sagitta; straight pairs return zero."""
    distance = second.horizontal_distance_along_path_m - first.horizontal_distance_along_path_m
    delta = abs(_heading_delta_rad(first.heading_deg, second.heading_deg))
    if distance <= _EPS or delta <= _EPS:
        return 0.0
    radius = distance / delta
    return radius * (1.0 - math.cos(delta / 2.0))


def _point_in_rect(x: float, y: float, rect: Tuple[float, float, float, float]) -> bool:
    xmin, ymin, xmax, ymax = rect
    return xmin - _EPS <= x <= xmax + _EPS and ymin - _EPS <= y <= ymax + _EPS


def _orientation(a, b, c) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _segments_intersect(a, b, c, d) -> bool:
    return _orientation(a, b, c) * _orientation(a, b, d) <= _EPS and \
        _orientation(c, d, a) * _orientation(c, d, b) <= _EPS


def _point_rect_distance(point, rect) -> float:
    x, y = point
    xmin, ymin, xmax, ymax = rect
    return math.hypot(max(xmin - x, 0.0, x - xmax), max(ymin - y, 0.0, y - ymax))


def _point_segment_distance(point, start, end) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= _EPS:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    t = max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_sq))
    return math.hypot(point[0] - start[0] - t * dx, point[1] - start[1] - t * dy)


def _segment_rect_distance(start, end, rect) -> float:
    if _point_in_rect(*start, rect) or _point_in_rect(*end, rect):
        return 0.0
    xmin, ymin, xmax, ymax = rect
    corners = ((xmin, ymin), (xmin, ymax), (xmax, ymin), (xmax, ymax))
    edges = ((corners[0], corners[1]), (corners[1], corners[3]), (corners[3], corners[2]), (corners[2], corners[0]))
    if any(_segments_intersect(start, end, a, b) for a, b in edges):
        return 0.0
    return min(_point_rect_distance(start, rect), _point_rect_distance(end, rect),
               *(_point_segment_distance(corner, start, end) for corner in corners))


def _cell_rect(terrain: TerrainQuery, row: int, col: int) -> Tuple[float, float, float, float]:
    left, _, _, top = terrain.roi.bounds
    dx, dy = terrain.roi.resolution
    return left + col * dx, top - (row + 1) * dy, left + (col + 1) * dx, top - row * dy


def _covered_cells(terrain: TerrainQuery, first: TrajectorySample, second: TrajectorySample, allowance_m: float):
    left, _, _, top = terrain.roi.bounds
    dx, dy = terrain.roi.resolution
    xmin, xmax = min(first.x_m, second.x_m) - allowance_m, max(first.x_m, second.x_m) + allowance_m
    ymin, ymax = min(first.y_m, second.y_m) - allowance_m, max(first.y_m, second.y_m) + allowance_m
    col0, col1 = math.floor((xmin - left) / dx), math.floor((xmax - left) / dx)
    row0, row1 = math.floor((top - ymax) / dy), math.floor((top - ymin) / dy)
    start, end = (first.x_m, first.y_m), (second.x_m, second.y_m)
    for row in range(row0, row1 + 1):
        for col in range(col0, col1 + 1):
            if _segment_rect_distance(start, end, _cell_rect(terrain, row, col)) <= allowance_m + _EPS:
                yield row, col


def evaluate_physical_trajectory_safety(
    trajectory: PhysicalTrajectory, terrain: TerrainQuery, effective_min_agl_m: float,
    required_max_sample_spacing_m: float, planning_bounds: Optional[Bounds] = None,
    lateral_buffer_m: float = 0.0, terrain_influence_cache: Optional[TerrainInfluenceCache] = None,
) -> TrajectorySafetyResult:
    """Conservative actual-curve centerline safety with optional terrain dilation.

    Chord coverage expands by per-segment sagitta. Lateral buffer remains a
    distinct static maximum-elevation field, never an approximation allowance.
    """
    min_agl = _nonnegative_finite(effective_min_agl_m, "effective_min_agl_m")
    required_spacing = float(required_max_sample_spacing_m)
    if not math.isfinite(required_spacing) or required_spacing <= 0.0:
        raise ValueError("required_max_sample_spacing_m must be finite and positive")
    buffer_m = _nonnegative_finite(lateral_buffer_m, "lateral_buffer_m")
    bounds = _validate_bounds(planning_bounds if planning_bounds is not None else terrain.roi.bounds)
    cache = terrain_influence_cache if terrain_influence_cache is not None else TerrainInfluenceCache(terrain)
    if cache.terrain is not terrain:
        raise ValueError("terrain_influence_cache belongs to a different TerrainQuery")
    field = cache.field(buffer_m)
    sampling_sufficient = trajectory.max_sample_spacing_m <= required_spacing + _EPS
    sample_failure = None
    cell_failure = None
    cell_failure_source_sample = None
    min_observed_agl = float("nan")
    covered_count, max_sagitta = 0, 0.0
    for index, sample in enumerate(trajectory.samples):
        if not all(math.isfinite(v) for v in (sample.x_m, sample.y_m, sample.z_msl_m)) and sample_failure is None:
            sample_failure = TrajectorySafetySample(index, sample.x_m, sample.y_m, sample.z_msl_m, float("nan"), float("nan"), "INVALID_SAMPLE")
        elif not _in_bounds(sample.x_m, sample.y_m, bounds) and sample_failure is None:
            sample_failure = TrajectorySafetySample(index, sample.x_m, sample.y_m, sample.z_msl_m, float("nan"), float("nan"), "OUTSIDE_ROI")
    for segment_index, (first, second) in enumerate(zip(trajectory.samples, trajectory.samples[1:])):
        sagitta = curve_to_chord_deviation_m(first, second)
        max_sagitta = max(max_sagitta, sagitta)
        minimum_z = min(first.z_msl_m, second.z_msl_m)
        for row, col in _covered_cells(terrain, first, second, sagitta):
            covered_count += 1
            if not terrain.in_bounds_rowcol(row, col):
                if cell_failure is None:
                    cell_failure = CoveredTerrainCell(segment_index, row, col, float("nan"), minimum_z, float("nan"), sagitta)
                    cell_failure_source_sample = first if first.z_msl_m <= second.z_msl_m else second
                continue
            if not field.valid[row, col]:
                if cell_failure is None:
                    cell_failure = CoveredTerrainCell(segment_index, row, col, float("nan"), minimum_z, float("nan"), sagitta)
                    cell_failure_source_sample = first if first.z_msl_m <= second.z_msl_m else second
                continue
            elevation = float(field.elevation_msl[row, col])
            agl_m = minimum_z - elevation
            if not math.isfinite(min_observed_agl) or agl_m < min_observed_agl:
                min_observed_agl = agl_m
            if agl_m < min_agl and cell_failure is None:
                cell_failure = CoveredTerrainCell(segment_index, row, col, elevation, minimum_z, agl_m, sagitta)
                cell_failure_source_sample = first if first.z_msl_m <= second.z_msl_m else second
    if sample_failure is not None:
        return TrajectorySafetyResult(False, min_observed_agl, sample_failure, cell_failure, sample_failure.reason,
                                      len(trajectory.samples), trajectory.max_sample_spacing_m, required_spacing,
                                      sampling_sufficient, covered_count, covered_count, max_sagitta, buffer_m)
    if cell_failure is not None:
        reason = "OUTSIDE_DEM" if not terrain.in_bounds_rowcol(cell_failure.row, cell_failure.col) else \
            ("OUTSIDE_DEM" if field.outside_dem[cell_failure.row, cell_failure.col] else
             ("NODATA" if not field.valid[cell_failure.row, cell_failure.col] else "BELOW_MIN_AGL"))
        assert cell_failure_source_sample is not None
        source_index = cell_failure.segment_index if cell_failure_source_sample is trajectory.samples[cell_failure.segment_index] \
            else cell_failure.segment_index + 1
        source_failure = TrajectorySafetySample(source_index, cell_failure_source_sample.x_m, cell_failure_source_sample.y_m,
                                                 cell_failure_source_sample.z_msl_m, cell_failure.terrain_elevation_msl,
                                                 cell_failure.agl_m, reason)
        return TrajectorySafetyResult(False, min_observed_agl, source_failure, cell_failure, reason, len(trajectory.samples),
                                      trajectory.max_sample_spacing_m, required_spacing, sampling_sufficient,
                                      covered_count, covered_count, max_sagitta, buffer_m)
    if not sampling_sufficient:
        return TrajectorySafetyResult(False, min_observed_agl, None, None, "INSUFFICIENT_SAMPLE_DENSITY",
                                      len(trajectory.samples), trajectory.max_sample_spacing_m, required_spacing, False,
                                      covered_count, covered_count, max_sagitta, buffer_m)
    return TrajectorySafetyResult(True, min_observed_agl, None, None, None, len(trajectory.samples),
                                  trajectory.max_sample_spacing_m, required_spacing, True,
                                  covered_count, covered_count, max_sagitta, buffer_m)
