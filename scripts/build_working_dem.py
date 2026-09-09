"""One-time DEM -> UTM working DEM reprojection.

Recreates the working DEM this project's pipeline depends on: warps the
source GLO-30 tile (EPSG:4326, ~1 arcsec) into the planner's working CRS at
a fixed 30 m pixel size, using block-maximum resampling so that peaks are
never averaged away (safety-conservative for terrain-avoidance use).

Run once. Output feeds scripts/validate_roi.py and, later, everything else
in planner/.
"""
from pathlib import Path

import numpy as np
import rasterio
from pyproj import Transformer
from rasterio.enums import Resampling
from rasterio.vrt import WarpedVRT
from rasterio.warp import calculate_default_transform
from rasterio.windows import from_bounds

from planner.config import DEFAULT_CONFIG

SOURCE_TILE = Path("copernicus_glo30_turkey/Copernicus_DSM_COG_10_N37_00_E035_00_DEM.tif")
OUTPUT_PATH = Path("working_dem/aladaglar_N37_E035_utm36n_max.tif")

# Working DEM covers a 20x20 km box around the ROI center: today's 10x10 km
# ROI plus a 5 km margin on every side, so the next ROI shift doesn't
# require re-running this script.
HALF_EXTENT_M = 10_000.0
DST_NODATA = -9999.0


def main() -> None:
    cfg = DEFAULT_CONFIG
    lon, lat = cfg.roi_center_lonlat

    with rasterio.open(SOURCE_TILE) as src:
        # WarpedVRT has no `resolution` kwarg in this rasterio version -- it
        # silently ignores unknown kwargs and GDAL picks its own default
        # pixel size. calculate_default_transform() is the documented way
        # to pin an exact target resolution.
        dst_transform, dst_width, dst_height = calculate_default_transform(
            src.crs, cfg.target_crs, src.width, src.height, *src.bounds,
            resolution=(cfg.xy_resolution_m, cfg.xy_resolution_m),
        )
        with WarpedVRT(
            src,
            crs=cfg.target_crs,
            transform=dst_transform,
            width=dst_width,
            height=dst_height,
            resampling=Resampling.max,
            nodata=DST_NODATA,
        ) as vrt:
            to_working = Transformer.from_crs("EPSG:4326", cfg.target_crs, always_xy=True)
            cx, cy = to_working.transform(lon, lat)

            window = from_bounds(
                cx - HALF_EXTENT_M, cy - HALF_EXTENT_M,
                cx + HALF_EXTENT_M, cy + HALF_EXTENT_M,
                transform=vrt.transform,
            ).round_lengths().round_offsets()

            data = vrt.read(1, window=window)
            transform = vrt.window_transform(window)

            profile = vrt.profile.copy()
            profile.update(
                count=1,
                height=data.shape[0],
                width=data.shape[1],
                transform=transform,
                nodata=DST_NODATA,
                dtype=data.dtype,
                driver="GTiff",
                compress="deflate",
            )

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    with rasterio.open(OUTPUT_PATH, "w", **profile) as dst:
        dst.write(data, 1)

    valid = data[data != DST_NODATA]
    print(f"Wrote {OUTPUT_PATH}")
    print(f"  size: {data.shape[1]} x {data.shape[0]} px")
    print(f"  elevation range (valid px): {valid.min():.1f} - {valid.max():.1f} m")
    print(f"  nodata px: {int(np.sum(data == DST_NODATA))} / {data.size}")


if __name__ == "__main__":
    main()
