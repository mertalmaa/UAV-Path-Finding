"""Display-terrain tile pyramid (Web Mercator XYZ, Terrarium-encoded PNG).

This module serves terrain for *visualisation only*. It never feeds the
planner: planning, AGL and safety always use the region working DEM through
``planner.roi`` / ``planner.trajectory_safety``.

Source data: Copernicus GLO-30 1x1 degree COGs in ``copernicus_glo30_turkey/``
(EPSG:4326, 1 arc-second, internal overviews 2/4/8).

Level-of-detail design
----------------------
* zoom <= MOSAIC_MAX_ZOOM: tiles are sampled from a national mosaic built once
  from the 1/8 COG overviews (8 arc-second, ~240 m) and persisted as ``.npy``.
  A whole-country view therefore never touches full-resolution rasters.
* zoom > MOSAIC_MAX_ZOOM: tiles are sampled from decimated windowed reads of
  only the COGs that intersect the tile. GDAL picks the matching overview.
* zoom > MAX_ZOOM (12, ~30 m/px at 40N): the client overzooms; no finer tiles
  exist because the source has no finer detail.
* Every rendered tile is written to a disk cache and served with long-lived
  HTTP caching, so a tile is computed at most once.

Visual terrain uses mean/bilinear resampling of the raw DSM. At full detail it
is never higher than the planner's block-max + lateral-buffer terrain. At
coarse zoom levels, averaging can fill narrow canyons, so a low valley route
may briefly look close to (or under) the coarse surface until finer tiles load.
The 3D terrain source therefore requests one zoom level finer than 2D shading
(see web/js/map-view.js), and the UI labels the surface as display LOD.
"""
from __future__ import annotations

import io
import json
import math
import re
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import rasterio
from PIL import Image
from rasterio.enums import Resampling
from rasterio.windows import Window

TILE_SIZE = 256
MIN_ZOOM = 0
MAX_ZOOM = 12
MOSAIC_MAX_ZOOM = 9
MOSAIC_PX_PER_DEG = 450  # 3600 / 8 -> reads the 1/8 overview exactly
CACHE_VERSION = "v1"

_TILE_RE = re.compile(r"Copernicus_DSM_COG_10_([NS])(\d{2})_00_([EW])(\d{3})_00_DEM\.tif$")


@dataclass(frozen=True)
class SourceTile:
    path: Path
    lat0: int  # south edge (deg)
    lon0: int  # west edge (deg)


def _lon_of_px(x_px: np.ndarray, z: int) -> np.ndarray:
    return x_px / (TILE_SIZE * 2 ** z) * 360.0 - 180.0


def _lat_of_px(y_px: np.ndarray, z: int) -> np.ndarray:
    n = math.pi - 2.0 * math.pi * y_px / (TILE_SIZE * 2 ** z)
    return np.degrees(np.arctan(np.sinh(n)))


def tile_bounds_lonlat(z: int, x: int, y: int) -> Tuple[float, float, float, float]:
    west = float(_lon_of_px(np.array(x * TILE_SIZE, dtype=np.float64), z))
    east = float(_lon_of_px(np.array((x + 1) * TILE_SIZE, dtype=np.float64), z))
    north = float(_lat_of_px(np.array(y * TILE_SIZE, dtype=np.float64), z))
    south = float(_lat_of_px(np.array((y + 1) * TILE_SIZE, dtype=np.float64), z))
    return west, south, east, north


def encode_terrarium(elev: np.ndarray) -> bytes:
    """Terrarium: height = (R*256 + G + B/256) - 32768."""
    v = np.clip(elev.astype(np.float64), -32768.0, 32767.0) + 32768.0
    r = np.floor(v / 256.0)
    g = np.floor(v - r * 256.0)
    b = np.floor((v - r * 256.0 - g) * 256.0)
    rgb = np.stack([r, g, b], axis=-1).astype(np.uint8)
    buf = io.BytesIO()
    Image.fromarray(rgb, "RGB").save(buf, format="PNG", compress_level=3)
    return buf.getvalue()


def _bilinear(arr: np.ndarray, fr: np.ndarray, fc: np.ndarray) -> np.ndarray:
    """Bilinear sample of ``arr`` at fractional pixel-centre coordinates."""
    h, w = arr.shape
    fr = np.clip(fr, 0.0, h - 1.0)
    fc = np.clip(fc, 0.0, w - 1.0)
    r0 = np.floor(fr).astype(np.int64)
    c0 = np.floor(fc).astype(np.int64)
    r1 = np.minimum(r0 + 1, h - 1)
    c1 = np.minimum(c0 + 1, w - 1)
    dr = fr - r0
    dc = fc - c0
    top = arr[r0, c0] * (1 - dc) + arr[r0, c1] * dc
    bot = arr[r1, c0] * (1 - dc) + arr[r1, c1] * dc
    return top * (1 - dr) + bot * dr


class TerrainTileService:
    def __init__(self, source_dir: Path, cache_dir: Path):
        self.source_dir = Path(source_dir)
        self.cache_dir = Path(cache_dir) / "terrain" / CACHE_VERSION
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        self.sources: Dict[Tuple[int, int], SourceTile] = {}
        for p in sorted(self.source_dir.glob("Copernicus_DSM_COG_10_*_DEM.tif")):
            m = _TILE_RE.search(p.name)
            if not m:
                continue
            lat = int(m.group(2)) * (1 if m.group(1) == "N" else -1)
            lon = int(m.group(4)) * (1 if m.group(3) == "E" else -1)
            self.sources[(lat, lon)] = SourceTile(p, lat, lon)
        if self.sources:
            lats = [k[0] for k in self.sources]
            lons = [k[1] for k in self.sources]
            self.coverage = (min(lons), min(lats), max(lons) + 1, max(lats) + 1)
        else:
            self.coverage = None
        # Cache-only mode. The source COGs (~5 GB) are optional at runtime: when
        # they are missing but a mosaic built earlier is present in the cache,
        # the service keeps serving display terrain from that mosaic, so a
        # packaged copy of the UI shows relief without shipping the sources.
        # Coverage then comes from the mosaic manifest instead of the filenames.
        self.source_mode = "cogs" if self.sources else "cache"
        if not self.sources:
            meta = self._read_mosaic_meta()
            if meta is not None:
                self.coverage = tuple(meta["coverage"])
            else:
                self.source_mode = "none"
        self._mosaic: Optional[np.ndarray] = None
        self._mosaic_lock = threading.Lock()
        # Bound concurrent cold renders so a burst of 3D tile requests cannot
        # saturate the CPU; cached tiles never wait on this.
        import os as _os
        self._render_slots = threading.BoundedSemaphore(max(2, min(8, (_os.cpu_count() or 4) - 1)))
        self._zero_tile = encode_terrarium(np.zeros((TILE_SIZE, TILE_SIZE), dtype=np.float32))
        self.stats = {"rendered": 0, "cache_hits": 0, "empty": 0, "render_ms_total": 0.0,
                      "by_zoom": {}}
        self.mosaic_build_s: Optional[float] = None

    # ------------------------------------------------------------------ meta
    @property
    def tile_epoch(self) -> str:
        """Identity of the terrain the server can currently produce.

        Tiles are served with a long immutable cache lifetime, so a browser that
        once fetched placeholder tiles (sources missing, nothing cached yet)
        would keep showing them for a week even after the sources come back.
        The epoch rides along in the tile URL template, so whenever the server's
        terrain situation changes the client asks for new URLs instead.
        """
        return f"{self.source_mode}{len(self.sources)}"

    def tilejson(self, tile_url: str) -> dict:
        west, south, east, north = self.coverage or (-180, -85, 180, 85)
        sep = "&" if "?" in tile_url else "?"
        tile_url = f"{tile_url}{sep}v={self.tile_epoch}"
        return {
            "tilejson": "3.0.0",
            "name": "copernicus-glo30-display",
            "tiles": [tile_url],
            "minzoom": MIN_ZOOM,
            "maxzoom": MAX_ZOOM,
            "tileSize": TILE_SIZE,
            "encoding": "terrarium",
            "bounds": [west, south, east, north],
            "source_tiles": len(self.sources),
            "source_mode": self.source_mode,
            "tile_epoch": self.tile_epoch,
            "mosaic_max_zoom": MOSAIC_MAX_ZOOM,
            "note": "Display terrain only. Planner terrain = region working DEM (block-max, buffered).",
        }

    # --------------------------------------------------------------- mosaic
    def _mosaic_paths(self) -> Tuple[Path, Path]:
        return self.cache_dir / "mosaic_8arcsec.npy", self.cache_dir / "mosaic_8arcsec.json"

    def _read_mosaic_meta(self) -> Optional[dict]:
        """The cached mosaic's manifest, or None when it is absent or unusable.

        Used in cache-only mode to recover the coverage box that would otherwise
        come from the source filenames.
        """
        npy, meta_path = self._mosaic_paths()
        if not (npy.exists() and meta_path.exists()):
            return None
        try:
            meta = json.loads(meta_path.read_text())
        except Exception:
            return None
        coverage = meta.get("coverage")
        if meta.get("px_per_deg") != MOSAIC_PX_PER_DEG or not coverage or len(coverage) != 4:
            return None
        return meta

    def ensure_mosaic(self) -> np.ndarray:
        if self._mosaic is not None:
            return self._mosaic
        with self._mosaic_lock:
            if self._mosaic is not None:
                return self._mosaic
            npy, meta_path = self._mosaic_paths()
            key = {"coverage": self.coverage, "files": len(self.sources), "px_per_deg": MOSAIC_PX_PER_DEG}
            meta = self._read_mosaic_meta()
            if meta is not None:
                # With sources present the manifest must match exactly, so a
                # changed source set rebuilds. In cache-only mode there is
                # nothing to rebuild from, so the cached mosaic is accepted as is.
                if not self.sources or meta == json.loads(json.dumps(key)):
                    self._mosaic = np.load(npy, mmap_mode="r")
                    return self._mosaic
            if not self.sources:
                # Refuse rather than write a zero-filled mosaic over the cache.
                raise RuntimeError(
                    "display terrain unavailable: no Copernicus source tiles in "
                    f"{self.source_dir} and no usable cached mosaic in {self.cache_dir}")
            t0 = time.perf_counter()
            west, south, east, north = self.coverage
            ppd = MOSAIC_PX_PER_DEG
            mosaic = np.zeros(((north - south) * ppd, (east - west) * ppd), dtype=np.int16)
            for (lat, lon), src in self.sources.items():
                with rasterio.open(src.path) as ds:
                    data = ds.read(1, out_shape=(ppd, ppd), resampling=Resampling.average)
                    if ds.nodata is not None:
                        data = np.where(data == ds.nodata, 0, data)
                r0 = (north - (lat + 1)) * ppd
                c0 = (lon - west) * ppd
                mosaic[r0:r0 + ppd, c0:c0 + ppd] = np.round(np.nan_to_num(data)).astype(np.int16)
            tmp = npy.with_suffix(".tmp.npy")
            np.save(tmp, mosaic)
            tmp.replace(npy)
            meta_path.write_text(json.dumps(key))
            self.mosaic_build_s = time.perf_counter() - t0
            self._mosaic = mosaic
            return mosaic

    # ---------------------------------------------------------------- render
    def _intersects_coverage(self, bounds) -> bool:
        if not self.coverage:
            return False
        w, s, e, n = bounds
        cw, cs, ce, cn = self.coverage
        return not (e <= cw or w >= ce or n <= cs or s >= cn)

    def _pixel_lonlat(self, z: int, x: int, y: int):
        px = (np.arange(TILE_SIZE, dtype=np.float64) + 0.5)
        lons = _lon_of_px(x * TILE_SIZE + px, z)
        lats = _lat_of_px(y * TILE_SIZE + px, z)
        return lons, lats

    def _render_from_mosaic(self, z: int, x: int, y: int) -> np.ndarray:
        mosaic = self.ensure_mosaic()
        west, south, east, north = self.coverage
        lons, lats = self._pixel_lonlat(z, x, y)
        ppd = MOSAIC_PX_PER_DEG
        fc = (lons - west) * ppd - 0.5
        fr = (north - lats) * ppd - 0.5
        FC, FR = np.meshgrid(fc, fr)
        out = _bilinear(mosaic, FR, FC).astype(np.float32)
        inside = (FC > -0.5) & (FC < mosaic.shape[1] - 0.5) & (FR > -0.5) & (FR < mosaic.shape[0] - 0.5)
        out[~inside] = 0.0
        return out

    def _render_from_sources(self, z: int, x: int, y: int) -> np.ndarray:
        west, south, east, north = tile_bounds_lonlat(z, x, y)
        lons, lats = self._pixel_lonlat(z, x, y)
        out = np.zeros((TILE_SIZE, TILE_SIZE), dtype=np.float32)
        out_res_deg = (east - west) / TILE_SIZE
        for lat in range(math.floor(south), math.ceil(north)):
            for lon in range(math.floor(west), math.ceil(east)):
                src = self.sources.get((lat, lon))
                if src is None:
                    continue
                col_mask = (lons >= lon) & (lons < lon + 1)
                row_mask = (lats >= lat) & (lats < lat + 1)
                if not col_mask.any() or not row_mask.any():
                    continue
                with rasterio.open(src.path) as ds:
                    t = ds.transform
                    res_x, res_y = t.a, -t.e
                    sub_lons = lons[col_mask]
                    sub_lats = lats[row_mask]
                    c_min = max(0, int(math.floor((sub_lons.min() - t.c) / res_x)) - 2)
                    c_max = min(ds.width, int(math.ceil((sub_lons.max() - t.c) / res_x)) + 2)
                    r_min = max(0, int(math.floor((t.f - sub_lats.max()) / res_y)) - 2)
                    r_max = min(ds.height, int(math.ceil((t.f - sub_lats.min()) / res_y)) + 2)
                    if c_max <= c_min or r_max <= r_min:
                        continue
                    factor = max(1, int(out_res_deg / res_x))
                    out_w = max(2, math.ceil((c_max - c_min) / factor))
                    out_h = max(2, math.ceil((r_max - r_min) / factor))
                    win = Window(c_min, r_min, c_max - c_min, r_max - r_min)
                    data = ds.read(1, window=win, out_shape=(out_h, out_w),
                                   resampling=Resampling.average if factor > 1 else Resampling.nearest)
                    data = data.astype(np.float32)
                    if ds.nodata is not None:
                        data[data == ds.nodata] = 0.0
                    data = np.nan_to_num(data)
                    eff_rx = (c_max - c_min) * res_x / out_w
                    eff_ry = (r_max - r_min) * res_y / out_h
                    left = t.c + c_min * res_x
                    top = t.f - r_min * res_y
                    fc = (sub_lons - left) / eff_rx - 0.5
                    fr = (top - sub_lats) / eff_ry - 0.5
                    FC, FR = np.meshgrid(fc, fr)
                    sampled = _bilinear(data, FR, FC)
                    rows = np.where(row_mask)[0]
                    cols = np.where(col_mask)[0]
                    out[np.ix_(rows, cols)] = sampled
        return out

    def get_tile(self, z: int, x: int, y: int) -> Tuple[bytes, str]:
        """Return (png_bytes, source) where source is cache|render|mosaic|empty.

        ``mosaic`` marks a tile sampled from the coarse national mosaic above the
        zoom it was meant for (cache-only mode). It is a stand-in for a tile the
        sources would render better, so callers should not cache it for long.
        ``empty`` is a flat placeholder and should not be cached at all.
        """
        if not (MIN_ZOOM <= z <= MAX_ZOOM) or not (0 <= x < 2 ** z) or not (0 <= y < 2 ** z):
            raise ValueError("tile out of range")
        bounds = tile_bounds_lonlat(z, x, y)
        # The disk cache is consulted before the coverage test: a packaged copy
        # can ship tiles without the sources that produced them.
        path = self.cache_dir / str(z) / str(x) / f"{y}.png"
        if path.exists():
            self.stats["cache_hits"] += 1
            return path.read_bytes(), "cache"
        if not self._intersects_coverage(bounds):
            self.stats["empty"] += 1
            return self._zero_tile, "empty"
        with self._render_slots:
            if path.exists():  # rendered by another request while we waited
                self.stats["cache_hits"] += 1
                return path.read_bytes(), "cache"
            t0 = time.perf_counter()
            # Never block on the one-off national mosaic build: until it exists,
            # low-zoom tiles are rendered from the COG overviews directly.
            try:
                if z <= MOSAIC_MAX_ZOOM and self._mosaic is not None:
                    elev = self._render_from_mosaic(z, x, y)
                elif z <= MOSAIC_MAX_ZOOM and not self._mosaic_lock.locked():
                    elev = self._render_from_mosaic(z, x, y)  # loads the cached .npy or builds it
                elif self.sources:
                    elev = self._render_from_sources(z, x, y)
                else:
                    # Cache-only mode above the mosaic zoom: sample the mosaic
                    # anyway. It is coarser than the sources would be (8" vs 1"),
                    # so it is served but never written to the cache, which a
                    # later run with the sources present would otherwise inherit.
                    self.stats["rendered"] += 1
                    return encode_terrarium(self._render_from_mosaic(z, x, y)), "mosaic"
            except Exception:
                # No sources and no usable mosaic: flat tile rather than a 500.
                self.stats["empty"] += 1
                return self._zero_tile, "empty"
            png = encode_terrarium(elev)
            path.parent.mkdir(parents=True, exist_ok=True)
            tmp = path.with_suffix(f".{threading.get_ident()}.tmp")
            tmp.write_bytes(png)
            try:
                tmp.replace(path)
            except OSError:  # Windows: a concurrent request already wrote / is reading it
                tmp.unlink(missing_ok=True)
            ms = (time.perf_counter() - t0) * 1000.0
        self.stats["rendered"] += 1
        self.stats["render_ms_total"] += ms
        bz = self.stats["by_zoom"].setdefault(str(z), 0)
        self.stats["by_zoom"][str(z)] = bz + 1
        return png, "render"

    def iter_coverage_tiles(self, max_zoom: int):
        """All (z, x, y) tiles intersecting the source coverage up to ``max_zoom``."""
        if not self.coverage:
            return
        yield from self.iter_bounds_tiles(self.coverage, MIN_ZOOM, max_zoom)

    @staticmethod
    def iter_bounds_tiles(bounds, min_zoom: int, max_zoom: int):
        """All (z, x, y) tiles intersecting a lon/lat ``bounds`` box, for ``min_zoom..max_zoom``.

        Used both for the national coverage pyramid and to warm a single mission
        region ahead of interactive zooming (see ``App.warm_region_tiles``).
        """
        west, south, east, north = bounds
        for z in range(min_zoom, max_zoom + 1):
            n = 2 ** z
            x0 = int((west + 180.0) / 360.0 * n)
            x1 = int(math.ceil((east + 180.0) / 360.0 * n)) - 1
            def ty(lat):
                return (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n
            y0, y1 = int(ty(north)), int(math.ceil(ty(south))) - 1
            for x in range(max(0, x0), min(n - 1, x1) + 1):
                for y in range(max(0, y0), min(n - 1, y1) + 1):
                    yield z, x, y

    def sample_elevation(self, lon: float, lat: float) -> Optional[float]:
        """Display-DEM elevation at one point (nearest 1" cell), for UI readout."""
        src = self.sources.get((math.floor(lat), math.floor(lon)))
        if src is None:
            return None
        with rasterio.open(src.path) as ds:
            r, c = ds.index(lon, lat)
            if not (0 <= r < ds.height and 0 <= c < ds.width):
                return None
            v = float(ds.read(1, window=Window(c, r, 1, 1))[0, 0])
            if ds.nodata is not None and v == ds.nodata:
                return None
            return v
