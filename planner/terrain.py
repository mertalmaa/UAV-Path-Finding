"""Point-wise terrain elevation queries against an already-loaded ROI.

Wraps a single ROIData in memory and answers UTM (x, y) -> elevation
lookups against it. No disk I/O happens here -- the working DEM is opened
once by planner.roi.load_roi(), and this module only reads the resulting
array. No new raster is produced and no interpolation is applied: a query
returns the value of the single raster cell that contains the point
(nearest/current-cell lookup), nothing more.

Row/col <-> UTM (x, y) conversions go through the ROI's real affine
transform (rasterio.transform.rowcol / .xy), never a manual x/30, y/30
approximation -- that keeps this correct even if resolution or origin
ever changes.
"""
from dataclasses import dataclass
from typing import Tuple

from rasterio.transform import rowcol as _rowcol
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

    def xy_to_rowcol(self, x: float, y: float) -> Tuple[int, int]:
        """UTM (x, y) -> raster (row, col), via the ROI's affine transform."""
        row, col = _rowcol(self.roi.transform, x, y)
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

    def query(self, x: float, y: float) -> TerrainQueryResult:
        """UTM (x, y) -> TerrainQueryResult. Out-of-ROI and NoData points
        come back with valid=False rather than raising, so a caller (e.g.
        a future A* expansion loop) can check .valid on a hot path without
        exception overhead."""
        row, col = self.xy_to_rowcol(x, y)
        elev, valid, reason = self._read_cell(row, col)
        return TerrainQueryResult(x, y, row, col, elev, valid, reason)
