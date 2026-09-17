"""Region registry and cached planner terrain contexts.

A region context holds exactly what the planner scripts build before calling
the planner: ``load_roi(config)`` -> ``TerrainQuery`` -> buffered terrain
field. Contexts are loaded lazily and cached, so repeated missions in the same
region do not reload the DEM.
"""
from __future__ import annotations

import dataclasses
import json
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import rasterio
from pyproj import Transformer
from rasterio.windows import bounds as window_bounds
from rasterio.windows import from_bounds

from planner.config import DEFAULT_CONFIG, PlannerConfig
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from planner.trajectory_safety import BufferedTerrainField, TerrainInfluenceCache

ROI_SIZE_M = 90_000.0  # regions/README.md: standard 90x90 km operation area


@dataclass
class RegionInfo:
    region_id: str
    name: str
    description: str
    center_lonlat: Tuple[float, float]
    crs: str
    dem_path: Path
    roi_size_m: float
    roi_bounds_utm: Tuple[float, float, float, float]
    elevation_stats: dict

    def config(self, **overrides) -> PlannerConfig:
        return dataclasses.replace(
            DEFAULT_CONFIG,
            working_dem_path=self.dem_path,
            roi_center_lonlat=tuple(self.center_lonlat),
            roi_size_m=self.roi_size_m,
            target_crs=self.crs,
            **overrides,
        )

    def contains_utm(self, x: float, y: float) -> bool:
        xmin, ymin, xmax, ymax = self.roi_bounds_utm
        return xmin <= x < xmax and ymin <= y < ymax


@dataclass
class RegionContext:
    info: RegionInfo
    terrain: TerrainQuery
    cache: TerrainInfluenceCache
    load_s: float
    to_utm: Transformer
    to_lonlat: Transformer
    lock: threading.Lock = field(default_factory=threading.Lock)

    def field(self, lateral_buffer_m: float) -> BufferedTerrainField:
        with self.lock:
            return self.cache.field(lateral_buffer_m)

    def has_field(self, lateral_buffer_m: float) -> bool:
        return float(lateral_buffer_m) in self.cache._fields  # read-only peek


class RegionRegistry:
    def __init__(self, project_root: Path):
        self.root = Path(project_root)
        manifest_path = self.root / "regions" / "regions_manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        self.regions: Dict[str, RegionInfo] = {}
        for rid, meta in manifest.items():
            dem = self.root / "regions" / rid / "working_dem.tif"
            if not dem.exists():
                continue
            center = tuple(meta["center_lonlat"])
            crs = meta.get("target_crs", "EPSG:32636")
            roi_size = float(meta.get("roi_size_m", ROI_SIZE_M))
            self.regions[rid] = RegionInfo(
                region_id=rid,
                name=meta.get("region_name", rid),
                description=meta.get("description", ""),
                center_lonlat=center,
                crs=crs,
                dem_path=dem,
                roi_size_m=roi_size,
                roi_bounds_utm=self._roi_bounds(dem, center, roi_size),
                elevation_stats=meta.get("elevation_stats", {}),
            )
        self._contexts: Dict[str, RegionContext] = {}
        self._lock = threading.Lock()
        self._transformers: Dict[str, Tuple[Transformer, Transformer]] = {}

    @staticmethod
    def _roi_bounds(dem: Path, center, roi_size_m: float):
        # Same window arithmetic as planner.roi.load_roi, without reading pixels.
        half = roi_size_m / 2.0
        with rasterio.open(dem) as ds:
            cx, cy = Transformer.from_crs("EPSG:4326", ds.crs, always_xy=True).transform(*center)
            window = from_bounds(cx - half, cy - half, cx + half, cy + half,
                                 transform=ds.transform).round_lengths().round_offsets()
            return tuple(float(v) for v in window_bounds(window, ds.transform))

    def transformers(self, crs: str) -> Tuple[Transformer, Transformer]:
        if crs not in self._transformers:
            self._transformers[crs] = (
                Transformer.from_crs("EPSG:4326", crs, always_xy=True),
                Transformer.from_crs(crs, "EPSG:4326", always_xy=True),
            )
        return self._transformers[crs]

    def find_region(self, lon: float, lat: float) -> Optional[RegionInfo]:
        for info in self.regions.values():
            to_utm, _ = self.transformers(info.crs)
            x, y = to_utm.transform(lon, lat)
            if info.contains_utm(x, y):
                return info
        return None

    def context(self, region_id: str) -> RegionContext:
        with self._lock:
            ctx = self._contexts.get(region_id)
            if ctx is not None:
                return ctx
            info = self.regions[region_id]
            t0 = time.perf_counter()
            roi = load_roi(info.config())
            terrain = TerrainQuery(roi)
            to_utm, to_ll = self.transformers(info.crs)
            ctx = RegionContext(info, terrain, TerrainInfluenceCache(terrain),
                                time.perf_counter() - t0, to_utm, to_ll)
            self._contexts[region_id] = ctx
            return ctx

    def is_loaded(self, region_id: str) -> bool:
        return region_id in self._contexts

    def roi_polygon_lonlat(self, info: RegionInfo, densify: int = 16) -> List[List[float]]:
        xmin, ymin, xmax, ymax = info.roi_bounds_utm
        _, to_ll = self.transformers(info.crs)
        pts = []
        edges = [((xmin, ymin), (xmax, ymin)), ((xmax, ymin), (xmax, ymax)),
                 ((xmax, ymax), (xmin, ymax)), ((xmin, ymax), (xmin, ymin))]
        for (x0, y0), (x1, y1) in edges:
            for i in range(densify):
                t = i / densify
                pts.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
        pts.append(pts[0])
        xs, ys = zip(*pts)
        lons, lats = to_ll.transform(np.array(xs), np.array(ys))
        return [[round(float(a), 6), round(float(b), 6)] for a, b in zip(lons, lats)]

    def ground_at(self, ctx: RegionContext, x: float, y: float, lateral_buffer_m: float) -> dict:
        tq = ctx.terrain
        row, col = tq.xy_to_rowcol(x, y)
        raw = tq.query(x, y)
        out = {"row": row, "col": col, "dem_cell_m": raw.elevation if raw.valid else None,
               "dem_valid": raw.valid, "dem_reason": raw.reason}
        fld = ctx.field(lateral_buffer_m)
        if tq.in_bounds_rowcol(row, col) and bool(fld.valid[row, col]):
            out["planner_ground_m"] = float(fld.elevation_msl[row, col])
        else:
            out["planner_ground_m"] = None
        return out
