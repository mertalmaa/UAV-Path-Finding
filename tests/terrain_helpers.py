"""Shared test helper: build a TerrainQuery from a plain elevation array.

Split out of the old test_current_contracts.py (removed together with the
legacy CandidateZ/grid-A* compatibility modules it exercised) because several
still-active test modules use just this helper, not the legacy contracts.
"""

from __future__ import annotations

import numpy as np
from affine import Affine

from planner.roi import ROIData
from planner.terrain import TerrainQuery

NODATA = -9999.0


def terrain_from_array(elevation: np.ndarray, resolution_m: float = 60.0) -> TerrainQuery:
    height, width = elevation.shape
    transform = Affine(resolution_m, 0.0, 0.0, 0.0, -resolution_m, height * resolution_m)
    roi = ROIData(
        elevation=elevation.astype(np.float32),
        transform=transform,
        crs="EPSG:32636",
        width=width,
        height=height,
        bounds=(0.0, 0.0, width * resolution_m, height * resolution_m),
        resolution=(resolution_m, resolution_m),
        nodata=NODATA,
    )
    return TerrainQuery(roi)
