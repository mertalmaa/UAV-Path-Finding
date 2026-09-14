"""Conservative block statistics used by the persistent terrain cache."""
import math
from dataclasses import dataclass

import numpy as np
from affine import Affine
from planner.roi import ROIData


def _nodata_mask(array: np.ndarray, nodata) -> np.ndarray:
    """True where a cell is NoData -- either a real NaN in the array (checked
    regardless of the configured sentinel, since a NaN always means "no
    data" no matter what nodata metadata says) or equal to the explicit
    `nodata` sentinel value (skipped if that sentinel is itself NaN, to
    avoid a redundant/degenerate equality check -- NaN != NaN is always
    False in IEEE754, so `array == nodata` would never match anyway)."""
    mask = np.isnan(array) if np.issubdtype(array.dtype, np.floating) else np.zeros(array.shape, dtype=bool)
    if nodata is not None and not (isinstance(nodata, float) and math.isnan(nodata)):
        mask = mask | (array == nodata)
    return mask


def _crop_to_complete_blocks(fine: np.ndarray, factor: int):
    """Shared edge policy (Stage 34): crop `fine` down to whole factor x
    factor blocks, dropping any trailing partial row/column band rather
    than padding it.
    Returns (cropped, coarse_height, coarse_width, dropped_rows, dropped_cols).
    """
    height, width = fine.shape
    coarse_height = height // factor
    coarse_width = width // factor
    dropped_rows = height - coarse_height * factor
    dropped_cols = width - coarse_width * factor
    if coarse_height == 0 or coarse_width == 0:
        raise ValueError(
            f"fine ROI ({height}x{width}) is too small for factor={factor} -- "
            f"produces a {coarse_height}x{coarse_width} coarse grid"
        )
    cropped = fine[: coarse_height * factor, : coarse_width * factor]
    return cropped, coarse_height, coarse_width, dropped_rows, dropped_cols


def _coarse_transform(fine_roi: ROIData, factor: int) -> Affine:
    fine_t = fine_roi.transform
    return Affine(fine_t.a * factor, fine_t.b, fine_t.c, fine_t.d, fine_t.e * factor, fine_t.f)


@dataclass
class CoarseTerrainStats:
    """Stage 34.5: four per-coarse-cell terrain summaries, each shape
    (coarse_height, coarse_width) -- separate plain arrays (not one stacked
    tensor) so a caller can read/pass around whichever one it needs without
    having to know about the others.

    SAFETY ROLE (read before using any of these for a planning decision):
      max_elevation  -- the ONLY one allowed to inform a HARD safety
                         decision (e.g. aircraft_msl - min_agl >= max_elevation
                         style checks). Block maximum pooling preserves
                         every fine-cell peak in the corresponding block.
      min_elevation  -- GUIDANCE/characterization only (valley evidence).
                         NEVER use to clear a safety margin: a single low
                         30m fine cell inside an otherwise-high 90m block
                         can drag min_elevation far down (see this
                         function's own docstring for a worked example) --
                         it does NOT mean a safe corridor exists there.
      mean_elevation -- GUIDANCE/characterization only (coarse altitude
                         trend). Also never a safety bound -- higher terrain
                         elsewhere in the same cell can be well above the
                         mean and would be missed entirely.
      relief         -- max_elevation - min_elevation: a roughness/
                         heterogeneity indicator for the cell, NOT a slope
                         and NOT a required climb/descent angle. A relief of
                         650m means "elevation spans 650m somewhere in this
                         90x90m cell", not "the aircraft must climb 650m".
    """
    min_elevation: np.ndarray
    mean_elevation: np.ndarray
    max_elevation: np.ndarray
    relief: np.ndarray
    transform: Affine
    crs: str
    nodata: float
    factor: int
    dropped_rows: int
    dropped_cols: int


def build_coarse_terrain_stats(fine_roi: ROIData, factor: int = 3) -> CoarseTerrainStats:
    """Return min/mean/max/relief for complete ``factor`` square blocks.

    If any fine cell in a
    block is NoData (or NaN), ALL FOUR statistics for that coarse cell
    become NoData -- min/mean/max/relief never mix 8 valid values with 1
    unknown one and pretend the result is fully known (no partial-block
    inference in this prototype).

    mean_elevation is computed and stored as float32 regardless of the fine
    array's own dtype (fine is typically float32 already; the sum of 9
    float32 elevations near a few thousand meters has ample headroom before
    float32 mean-precision would matter for a terrain summary at this
    resolution). min_elevation/max_elevation/relief keep the fine array's
    own dtype.
    """
    if factor < 1:
        raise ValueError(f"factor must be >= 1, got {factor}")

    fine = fine_roi.elevation
    cropped, coarse_height, coarse_width, dropped_rows, dropped_cols = _crop_to_complete_blocks(fine, factor)
    is_nodata = _nodata_mask(cropped, fine_roi.nodata)

    blocks = cropped.reshape(coarse_height, factor, coarse_width, factor)
    block_min = blocks.min(axis=(1, 3))
    block_mean = blocks.mean(axis=(1, 3))
    block_max = blocks.max(axis=(1, 3))
    block_relief = block_max - block_min

    nodata_blocks = is_nodata.reshape(coarse_height, factor, coarse_width, factor)
    any_nodata = nodata_blocks.any(axis=(1, 3))

    nodata = fine_roi.nodata
    if nodata is not None:
        block_min = np.where(any_nodata, nodata, block_min)
        block_max = np.where(any_nodata, nodata, block_max)
        block_mean = np.where(any_nodata, nodata, block_mean)
        block_relief = np.where(any_nodata, nodata, block_relief)

    return CoarseTerrainStats(
        min_elevation=block_min.astype(fine.dtype),
        mean_elevation=block_mean.astype(np.float32),
        max_elevation=block_max.astype(fine.dtype),
        relief=block_relief.astype(fine.dtype),
        transform=_coarse_transform(fine_roi, factor),
        crs=fine_roi.crs,
        nodata=nodata,
        factor=factor,
        dropped_rows=dropped_rows,
        dropped_cols=dropped_cols,
    )
