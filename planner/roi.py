"""Load a fixed-size ROI window from the validated UTM working DEM.

Produces the plain ROIData structure that later planner modules (cost
function, A*, etc.) will consume. This module only loads and describes
data -- no masking, cost, or search logic lives here.
"""
from dataclasses import dataclass
from typing import Tuple

import numpy as np
import rasterio
from affine import Affine
from pyproj import Transformer
from rasterio.windows import bounds as window_bounds
from rasterio.windows import from_bounds

from planner.config import DEFAULT_CONFIG, PlannerConfig


@dataclass
class ROIData:
    elevation: np.ndarray
    transform: Affine
    crs: str
    width: int
    height: int
    bounds: Tuple[float, float, float, float]  # left, bottom, right, top
    resolution: Tuple[float, float]  # x, y
    nodata: float


def load_roi(config: PlannerConfig = DEFAULT_CONFIG) -> ROIData:
    lon, lat = config.roi_center_lonlat
    half = config.roi_size_m / 2.0

    with rasterio.open(config.working_dem_path) as ds:
        to_working = Transformer.from_crs("EPSG:4326", ds.crs, always_xy=True)
        cx, cy = to_working.transform(lon, lat)

        window = from_bounds(
            cx - half, cy - half, cx + half, cy + half, transform=ds.transform
        ).round_lengths().round_offsets()

        elevation = ds.read(1, window=window)
        transform = ds.window_transform(window)

        return ROIData(
            elevation=elevation,
            transform=transform,
            crs=str(ds.crs),
            width=elevation.shape[1],
            height=elevation.shape[0],
            bounds=window_bounds(window, ds.transform),
            resolution=(abs(transform.a), abs(transform.e)),
            nodata=ds.nodata,
        )
