"""Roadmap Step 3C: persistent, static terrain preprocessing cache.

Builds and loads a small set of DEM-derived arrays ONCE, offline, so a
mission never has to re-read or re-derive raw terrain metadata. Reuses
planner.coarse.build_coarse_dem / build_coarse_terrain_stats verbatim for
every derived (coarser) level -- this module only adds persistence
(build/validate/load) around them, it does not reimplement any terrain
math. planner.roi.load_roi / planner.terrain.TerrainQuery are similarly
reused unchanged wherever this module needs to read the source DEM.

WHAT IS CACHED (per Step 3C, section 1):
  - source DEM identity/fingerprint (SHA256 of the file bytes), CRS,
    transform, dimensions, native resolution, nodata (manifest only).
  - the native (factor=1) fine elevation array itself.
  - per requested pooling factor (e.g. 2 -> 60m, 3 -> 90m): min/max/mean/
    relief, straight from build_coarse_terrain_stats. Factors are
    INDEPENDENT derived levels from the same 30m source -- this module
    never assumes a forced 90->60->30 parent/child hierarchy; each factor
    is computed directly from the fine ROI.

WHAT IS DELIBERATELY NOT CACHED (per Step 3C, section 2 -- different
lifecycle, see module docstring of scripts/step3c_persistent_terrain_cache.py
for the full reasoning): mission start/goal/ceiling/allowed-altitude-
interval, any min_agl-DERIVED value (clearance floor, P1 ambiguity band --
these are one O(1) addition away from the cached min/max, recomputing them
is far cheaper than the staleness risk of baking a config constant into
the cache key), search states, corridor data, CLASS-C motion/aircraft
events, JSBSim output, future aircraft-safety-box results.

CACHE SCOPE DECISION (Step 3C section 3): terrain cache = f(terrain) ONLY.
min_agl_m is NEVER part of the cache fingerprint, and clearance-floor-style
values are NEVER persisted -- changing min_agl_m must never force a terrain
rebuild. This is enforced structurally: TerrainCache exposes only raw
min/max/mean/relief/elevation; any min_agl-dependent quantity is computed
by the CALLER (e.g. CandidateZGenerator.floor_for, unchanged from Step 3B)
at load/mission time, a single cheap arithmetic op per cell.
"""
import hashlib
import json
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from planner.coarse import build_coarse_terrain_stats
from planner.roi import ROIData
from planner.terrain import TerrainQuery

SCHEMA_VERSION = 1
# Bump this string whenever the BUILD algorithm changes meaningfully (not
# for cosmetic script edits) -- a code_version mismatch invalidates the
# cache even if the DEM file itself is byte-identical.
CODE_VERSION = "terrain_cache_v1"

MANIFEST_FILENAME = "manifest.json"
ARRAYS_FILENAME = "arrays.npz"


class TerrainCacheError(Exception):
    """Base class -- a cache request could not be satisfied safely."""


class TerrainCacheMissingError(TerrainCacheError):
    """No cache directory/files found at the given path."""


class TerrainCacheStaleError(TerrainCacheError):
    """A cache exists but does not match the requested source/schema/code
    version -- MUST be treated as invalid, never silently used."""


@dataclass(frozen=True)
class CacheManifest:
    schema_version: int
    code_version: str
    created_at_utc: str
    source_dem_path: str
    source_sha256: str
    source_width: int
    source_height: int
    source_crs: str
    source_transform: Tuple[float, float, float, float, float, float]
    native_resolution_m: float
    nodata: Optional[float]
    pooling_factors: List[int]
    array_shapes: Dict[str, Tuple[int, ...]]
    array_dtypes: Dict[str, str]


def _sha256_of_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _transform_tuple(roi: ROIData) -> Tuple[float, float, float, float, float, float]:
    t = roi.transform
    return (float(t.a), float(t.b), float(t.c), float(t.d), float(t.e), float(t.f))


def build_terrain_cache(
    fine_roi: ROIData, source_dem_path: str, factors: List[int], out_dir: str,
) -> CacheManifest:
    """Offline/static preprocessing: compute the native elevation array plus
    min/max/mean/relief for each requested pooling factor (via the existing,
    unmodified build_coarse_terrain_stats), and persist everything as one
    .npz + one manifest.json. Returns the manifest that was written.
    """
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    arrays: Dict[str, np.ndarray] = {"fine_elevation": fine_roi.elevation.astype(np.float32)}
    for factor in factors:
        if factor == 1:
            continue  # native level IS fine_elevation -- no degenerate min=max=mean/relief=0 duplication
        stats = build_coarse_terrain_stats(fine_roi, factor=factor)
        arrays[f"min_elevation_f{factor}"] = stats.min_elevation
        arrays[f"max_elevation_f{factor}"] = stats.max_elevation
        arrays[f"mean_elevation_f{factor}"] = stats.mean_elevation
        arrays[f"relief_f{factor}"] = stats.relief

    np.savez_compressed(out / ARRAYS_FILENAME, **arrays)

    manifest = CacheManifest(
        schema_version=SCHEMA_VERSION,
        code_version=CODE_VERSION,
        created_at_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        source_dem_path=str(source_dem_path),
        source_sha256=_sha256_of_file(source_dem_path),
        source_width=fine_roi.width,
        source_height=fine_roi.height,
        source_crs=str(fine_roi.crs),
        source_transform=_transform_tuple(fine_roi),
        native_resolution_m=float(fine_roi.resolution[0]),
        nodata=float(fine_roi.nodata) if fine_roi.nodata is not None else None,
        pooling_factors=sorted(factors),
        array_shapes={k: tuple(v.shape) for k, v in arrays.items()},
        array_dtypes={k: str(v.dtype) for k, v in arrays.items()},
    )
    with open(out / MANIFEST_FILENAME, "w") as f:
        json.dump(asdict(manifest), f, indent=2)
    return manifest


def _read_manifest(out_dir: str) -> CacheManifest:
    path = Path(out_dir) / MANIFEST_FILENAME
    if not path.exists():
        raise TerrainCacheMissingError(f"no manifest at {path}")
    with open(path) as f:
        raw = json.load(f)
    raw["source_transform"] = tuple(raw["source_transform"])
    raw["array_shapes"] = {k: tuple(v) for k, v in raw["array_shapes"].items()}
    return CacheManifest(**raw)


def validate_terrain_cache(
    out_dir: str, fine_roi: Optional[ROIData] = None, source_dem_path: Optional[str] = None,
) -> Tuple[bool, str]:
    """Non-raising check: does the cache at out_dir exist and match the
    given source (if provided)? Returns (ok, reason). Never returns True
    for a cache that doesn't match the current DEM/schema/code version --
    cache correctness comes before planning correctness (Step 3C section 13)."""
    try:
        manifest = _read_manifest(out_dir)
    except TerrainCacheMissingError as e:
        return False, str(e)

    if not (Path(out_dir) / ARRAYS_FILENAME).exists():
        return False, f"manifest present but {ARRAYS_FILENAME} missing"
    if manifest.schema_version != SCHEMA_VERSION:
        return False, f"schema_version mismatch: cache={manifest.schema_version} current={SCHEMA_VERSION}"
    if manifest.code_version != CODE_VERSION:
        return False, f"code_version mismatch: cache={manifest.code_version} current={CODE_VERSION}"

    if source_dem_path is not None:
        if manifest.source_dem_path != str(source_dem_path):
            return False, f"source_dem_path mismatch: cache={manifest.source_dem_path} current={source_dem_path}"
        try:
            current_hash = _sha256_of_file(source_dem_path)
        except OSError as e:
            return False, f"cannot re-hash source DEM: {e}"
        if current_hash != manifest.source_sha256:
            return False, "source DEM content hash mismatch (file changed since cache was built)"

    if fine_roi is not None:
        if (fine_roi.width, fine_roi.height) != (manifest.source_width, manifest.source_height):
            return False, (f"dimensions mismatch: cache=({manifest.source_width},{manifest.source_height}) "
                            f"current=({fine_roi.width},{fine_roi.height})")
        if str(fine_roi.crs) != manifest.source_crs:
            return False, f"CRS mismatch: cache={manifest.source_crs} current={fine_roi.crs}"
        if _transform_tuple(fine_roi) != manifest.source_transform:
            return False, "affine transform mismatch"

    return True, "ok"


class TerrainCache:
    """Loaded, in-memory view of a persistent terrain cache. Read-only,
    never re-derives anything from the raw DEM -- every method here is a
    pure array lookup against arrays loaded once at construction time."""

    def __init__(self, manifest: CacheManifest, arrays: Dict[str, np.ndarray]):
        self.manifest = manifest
        self._arrays = arrays

    @property
    def available_factors(self) -> List[int]:
        return list(self.manifest.pooling_factors)

    def elevation_fine(self, row: int, col: int) -> float:
        arr = self._arrays["fine_elevation"]
        if not (0 <= row < arr.shape[0] and 0 <= col < arr.shape[1]):
            return float("nan")
        v = float(arr[row, col])
        return float("nan") if (self.manifest.nodata is not None and v == self.manifest.nodata) else v

    def _coarse_array(self, kind: str, factor: int) -> np.ndarray:
        key = f"{kind}_elevation_f{factor}" if kind != "relief" else f"relief_f{factor}"
        if key not in self._arrays:
            raise KeyError(f"factor={factor} not present in this cache (available: {self.available_factors})")
        return self._arrays[key]

    def min_elevation(self, factor: int, row: int, col: int) -> float:
        return float(self._coarse_array("min", factor)[row, col])

    def max_elevation(self, factor: int, row: int, col: int) -> float:
        return float(self._coarse_array("max", factor)[row, col])

    def mean_elevation(self, factor: int, row: int, col: int) -> float:
        return float(self._coarse_array("mean", factor)[row, col])

    def relief(self, factor: int, row: int, col: int) -> float:
        return float(self._coarse_array("relief", factor)[row, col])


def load_terrain_cache(
    out_dir: str, fine_roi: Optional[ROIData] = None, source_dem_path: Optional[str] = None, validate: bool = True,
) -> TerrainCache:
    """Load a persistent cache. Raises TerrainCacheMissingError /
    TerrainCacheStaleError rather than ever silently returning a cache that
    doesn't match the current DEM/schema/code -- a caller that wants a
    non-raising check should call validate_terrain_cache() directly first."""
    if validate and (fine_roi is not None or source_dem_path is not None):
        ok, reason = validate_terrain_cache(out_dir, fine_roi, source_dem_path)
        if not ok:
            raise TerrainCacheStaleError(reason)

    manifest = _read_manifest(out_dir)
    npz_path = Path(out_dir) / ARRAYS_FILENAME
    if not npz_path.exists():
        raise TerrainCacheMissingError(f"manifest present but {npz_path} missing")
    with np.load(npz_path) as data:
        arrays = {k: data[k] for k in data.files}
    return TerrainCache(manifest, arrays)
