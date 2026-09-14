"""Point-wise terrain elevation queries against an already-loaded ROI.

Wraps a single ROIData in memory and answers UTM (x, y) -> elevation
lookups against it. No disk I/O happens here -- the working DEM is opened
once by planner.roi.load_roi(), and this module only reads the resulting
array. No new raster is produced and no interpolation is applied: a query
returns the value of the single raster cell that contains the point
(nearest/current-cell lookup), nothing more.

Row/col -> UTM (x, y) (rowcol_to_xy) still goes through the ROI's real
affine transform via rasterio.transform.xy, unchanged.

UTM (x, y) -> row/col (xy_to_rowcol), the Mission search hot path (PERF-5),
uses a precomputed inverse-affine formula instead of rasterio.transform.
rowcol(). This is not an approximation: the inverse coefficients are
derived with the exact same arithmetic affine.Affine.__invert__ uses
(idet = 1/(a*e - b*d); ra = e*idet; rb = -b*idet; rd = -d*idet;
re = a*idet; rc = -c*ra - f*rb; rf = -c*rd - f*re), computed ONCE per
TerrainQuery instead of rebuilt (via rasterio's generic attrs-based
Affine plumbing) on every call, and np.floor(...).astype(int32) is
replaced by the equivalent math.floor(...) -- never Python's int()
truncation, which disagrees with floor() for negative fractional values.
Proven bit-for-bit equivalent to rasterio.transform.rowcol() over 430,064
test points (random interior, exact pixel centers, exact pixel
boundaries, +-epsilon on both sides of boundaries down to 1e-9, raster
corners, out-of-bounds/negative coordinates, and every real Mission A
query coordinate) before this replaced the hot path -- see PERF-5.
"""
import math
from dataclasses import dataclass
from typing import Tuple

from rasterio.transform import xy as _xy

from planner.roi import ROIData


@dataclass(frozen=True)
class TerrainQueryResult:
    x: float
    y: float
    row: int
    col: int
    elevation: float  # NaN when not valid
    valid: bool
    reason: str  # "ok" | "out_of_bounds" | "nodata"


class TerrainQuery:
    """Read-only elevation lookups into one ROIData's elevation array."""

    def __init__(self, roi: ROIData):
        self.roi = roi
        # PERF-5: inverse-affine coefficients, precomputed once (never
        # per-call) with the same arithmetic as affine.Affine.__invert__.
        t = roi.transform
        det = t.a * t.e - t.b * t.d
        idet = 1.0 / det
        self._inv_a = t.e * idet
        self._inv_b = -t.b * idet
        self._inv_d = -t.d * idet
        self._inv_e = t.a * idet
        self._inv_c = -t.c * self._inv_a - t.f * self._inv_b
        self._inv_f = -t.c * self._inv_d - t.f * self._inv_e

    def xy_to_rowcol(self, x: float, y: float) -> Tuple[int, int]:
        """UTM (x, y) -> raster (row, col).

        Mathematically equivalent to rasterio.transform.rowcol(self.roi.
        transform, x, y) (its default op=None, i.e. floor -- see module
        docstring), via a precomputed inverse affine instead of rebuilding
        it every call.
        """
        col = math.floor(self._inv_a * x + self._inv_b * y + self._inv_c)
        row = math.floor(self._inv_d * x + self._inv_e * y + self._inv_f)
        return int(row), int(col)

    def rowcol_to_xy(self, row: int, col: int) -> Tuple[float, float]:
        """Raster (row, col) cell center -> UTM (x, y)."""
        x, y = _xy(self.roi.transform, row, col)  # offset='center' by default
        return float(x), float(y)

    def in_bounds_rowcol(self, row: int, col: int) -> bool:
        return 0 <= row < self.roi.height and 0 <= col < self.roi.width

    def in_bounds_xy(self, x: float, y: float) -> bool:
        row, col = self.xy_to_rowcol(x, y)
        return self.in_bounds_rowcol(row, col)

    def _read_cell(self, row: int, col: int) -> Tuple[float, bool, str]:
        if not self.in_bounds_rowcol(row, col):
            return float("nan"), False, "out_of_bounds"
        elev = float(self.roi.elevation[row, col])
        if self.roi.nodata is not None and elev == self.roi.nodata:
            return float("nan"), False, "nodata"
        return elev, True, "ok"

    def elevation_at_rowcol(self, row: int, col: int) -> TerrainQueryResult:
        elev, valid, reason = self._read_cell(row, col)
        x, y = self.rowcol_to_xy(row, col)
        return TerrainQueryResult(x, y, row, col, elev, valid, reason)

    def _query_payload(self, x: float, y: float) -> Tuple[int, int, float, bool, str]:
        """Return the scalar payload of :meth:`query` without allocating a
        TerrainQueryResult.

        This private helper is the shared authoritative coordinate-to-cell
        and cell-read path used by ``query()`` and primitive safety. Public
        callers continue to receive TerrainQueryResult objects from
        ``query()``.
        """
        row, col = self.xy_to_rowcol(x, y)
        elev, valid, reason = self._read_cell(row, col)
        return row, col, elev, valid, reason

    def query(self, x: float, y: float) -> TerrainQueryResult:
        """UTM (x, y) -> TerrainQueryResult. Out-of-ROI and NoData points
        come back with valid=False rather than raising, so a caller (e.g.
        a future A* expansion loop) can check .valid on a hot path without
        exception overhead."""
        row, col, elev, valid, reason = self._query_payload(x, y)
        return TerrainQueryResult(x, y, row, col, elev, valid, reason)
