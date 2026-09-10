"""Conservative coarse-resolution DEM construction (Stage 34).

Builds a coarser ROIData from an existing fine ROIData by MAX-pooling
each `factor` x `factor` block of fine cells into one coarse cell --
never average/bilinear/nearest/slicing. Max pooling is the only common
downsampling rule that can never make a real terrain peak disappear: any
peak inside a block is captured verbatim by that block's own maximum,
whereas averaging/bilinear/nearest can all silently smooth or skip past
one. That safety property is the entire point before any coarser grid is
ever allowed to inform a planner (not attempted in this stage -- see the
module/project docstrings for what Stage 34 deliberately does NOT do).

Produces a plain planner.roi.ROIData -- the SAME structure planner.
terrain.TerrainQuery already consumes -- so no other module needs to
change to use a coarse grid; a caller just does
TerrainQuery(build_coarse_dem(fine_roi).roi).
"""
import math
from dataclasses import dataclass

import numpy as np
from affine import Affine
from rasterio.transform import array_bounds

from planner.roi import ROIData


@dataclass
class CoarseDEMResult:
    roi: ROIData
    factor: int
    dropped_rows: int  # fine rows dropped because they didn't form a complete block
    dropped_cols: int  # fine cols dropped because they didn't form a complete block


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
    than padding it (see build_coarse_dem's own docstring for why).
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


def build_coarse_dem(fine_roi: ROIData, factor: int = 3) -> CoarseDEMResult:
    """Conservative max-pooling downsample by an integer `factor`.

    coarse[I, J] = MAX(fine elevations in rows I*factor:(I+1)*factor,
                        cols J*factor:(J+1)*factor)

    -- a small terrain peak that would vanish under average/bilinear/
    nearest resampling survives here, because the block that contains it
    reports its own true maximum.

    NoData (safety-conservative): if ANY fine cell in a block is NoData,
    the WHOLE coarse cell becomes NoData too -- an unknown-terrain patch is
    never presented as known-and-safe merely because its neighbors inside
    the same block happened to be valid.

    Edge policy (prototype choice, see project.md "Stage 34"): only
    COMPLETE factor x factor blocks are used. Trailing rows/columns that
    don't form a full block are DROPPED, never padded -- padding a partial
    block would have to invent elevation data for cells that don't exist,
    which is the opposite of conservative. The exact counts are returned
    (CoarseDEMResult.dropped_rows/dropped_cols) so a caller always sees
    this explicitly rather than it happening silently.

    The output affine transform is derived from the fine transform's own
    a/b/c/d/e/f entries (never hard-coded UTM numbers): pixel scale (a, e)
    simply multiplies by `factor`, while the origin (c, f -- the fine
    grid's own upper-left world corner) and any rotation/shear (b, d) are
    kept EXACTLY, so the coarse grid's upper-left corner lines up with the
    fine grid's rather than drifting.
    """
    if factor < 1:
        raise ValueError(f"factor must be >= 1, got {factor}")

    fine = fine_roi.elevation
    cropped, coarse_height, coarse_width, dropped_rows, dropped_cols = _crop_to_complete_blocks(fine, factor)
    is_nodata = _nodata_mask(cropped, fine_roi.nodata)

    blocks = cropped.reshape(coarse_height, factor, coarse_width, factor)
    coarse_elevation = blocks.max(axis=(1, 3))

    nodata_blocks = is_nodata.reshape(coarse_height, factor, coarse_width, factor)
    any_nodata = nodata_blocks.any(axis=(1, 3))

    if fine_roi.nodata is not None:
        coarse_elevation = np.where(any_nodata, fine_roi.nodata, coarse_elevation)
    coarse_elevation = coarse_elevation.astype(fine.dtype)

    coarse_transform = _coarse_transform(fine_roi, factor)

    left, bottom, right, top = array_bounds(coarse_height, coarse_width, coarse_transform)

    coarse_roi = ROIData(
        elevation=coarse_elevation,
        transform=coarse_transform,
        crs=fine_roi.crs,
        width=coarse_width,
        height=coarse_height,
        bounds=(left, bottom, right, top),
        resolution=(abs(coarse_transform.a), abs(coarse_transform.e)),
        nodata=fine_roi.nodata,
    )
    return CoarseDEMResult(roi=coarse_roi, factor=factor, dropped_rows=dropped_rows, dropped_cols=dropped_cols)


@dataclass
class CoarseTerrainStats:
    """Stage 34.5: four per-coarse-cell terrain summaries, each shape
    (coarse_height, coarse_width) -- separate plain arrays (not one stacked
    tensor) so a caller can read/pass around whichever one it needs without
    having to know about the others.

    SAFETY ROLE (read before using any of these for a planning decision):
      max_elevation  -- the ONLY one allowed to inform a HARD safety
                         decision (e.g. aircraft_msl - min_agl >= max_elevation
                         style checks). It is bit-for-bit the same value
                         build_coarse_dem() already produces (see Stage 34.5
                         validation) -- this class doesn't change that
                         guarantee, just also exposes min/mean/relief
                         alongside it.
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
    """Stage 34.5: min/mean/max/relief per coarse cell, reusing the exact
    same crop-to-complete-blocks + reshape + conservative-NoData machinery
    as build_coarse_dem() (this function does NOT modify or depend on that
    function's own behavior -- both independently derive from the shared
    `_crop_to_complete_blocks`/`_nodata_mask`/`_coarse_transform` helpers,
    so build_coarse_dem's Stage 34 behavior/validation is untouched).

    NoData policy is IDENTICAL to build_coarse_dem: if any fine cell in a
    block is NoData (or NaN), ALL FOUR statistics for that coarse cell
    become NoData -- min/mean/max/relief never mix 8 valid values with 1
    unknown one and pretend the result is fully known (no partial-block
    inference in this prototype).

    mean_elevation is computed and stored as float32 regardless of the fine
    array's own dtype (fine is typically float32 already; the sum of 9
    float32 elevations near a few thousand meters has ample headroom before
    float32 mean-precision would matter for a terrain summary at this
    resolution). min_elevation/max_elevation/relief keep the fine array's
    own dtype, matching build_coarse_dem's own MAX output exactly.
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
