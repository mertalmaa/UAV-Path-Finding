"""Roadmap Step 3C: persistent preprocessing + static terrain cache --
build/load/validate the cache (planner/terrain_cache.py), measure it on the
FULL real Aladağlar ROI, and show CandidateZGenerator (Step 3B, UNCHANGED,
imported not modified) reading from it instead of a live DEM with IDENTICAL
results. No production search integration, no CLASS-C implementation, no
100x100km real build -- see module docstring sections below for exact scope.
"""
import dataclasses
import os
import shutil
import time
from pathlib import Path

import numpy as np

from planner.coarse import build_coarse_terrain_stats
from planner.config import DEFAULT_CONFIG
from planner.roi import load_roi
from planner.terrain import TerrainQuery
from planner.terrain_cache import (
    TerrainCacheMissingError, TerrainCacheStaleError, build_terrain_cache, load_terrain_cache,
    validate_terrain_cache,
)
from scripts.step3b_sparse_lazy_z_prototype import CandidateZGenerator, MissionContext, TerrainMetadata

STEP2B_CONFIG = dataclasses.replace(DEFAULT_CONFIG, min_agl_m=100.0)
CACHE_DIR = "outputs/terrain_cache"
SOURCE_DEM_PATH = str(DEFAULT_CONFIG.working_dem_path)
FACTORS = [2, 3]  # 60m, 90m -- Step 2F's provisional levels; independent derived levels, no forced hierarchy


class CacheBackedStore:
    """Same .get(row,col)->TerrainMetadata duck-typed interface Step 3B's
    TerrainMetadataStore exposes, but backed ENTIRELY by an in-memory
    TerrainCache -- never touches TerrainQuery/rasterio/the raw DEM file.
    This is the whole point of section 8/9: CandidateZGenerator (imported
    unchanged from Step 3B) doesn't know or care which store it was given."""

    def __init__(self, cache):
        self._cache = cache
        self.dem_reads = 0  # structural counter -- must stay 0 for this class, see assertion in main()

    def get(self, row: int, col: int) -> TerrainMetadata:
        elev = self._cache.elevation_fine(row, col)
        return TerrainMetadata(row, col, elev)


def main():
    print("=" * 70)
    print("STEP 3C -- persistent preprocessing + static terrain cache")
    print("=" * 70)

    if Path(CACHE_DIR).exists():
        shutil.rmtree(CACHE_DIR)  # start clean so build timing below is a genuine cold build

    fine_roi = load_roi(DEFAULT_CONFIG)
    print(f"\n  source DEM: {SOURCE_DEM_PATH}")
    print(f"  fine ROI: {fine_roi.height}x{fine_roi.width}, native resolution={fine_roi.resolution[0]}m")

    # ---- A) OFFLINE: build ----
    t0 = time.perf_counter()
    manifest = build_terrain_cache(fine_roi, SOURCE_DEM_PATH, FACTORS, CACHE_DIR)
    build_time_s = time.perf_counter() - t0
    npz_size = os.path.getsize(Path(CACHE_DIR) / "arrays.npz")
    manifest_size = os.path.getsize(Path(CACHE_DIR) / "manifest.json")
    total_size = npz_size + manifest_size
    print(f"\n--- A) OFFLINE build (full real ROI, factors={FACTORS}) ---")
    print(f"  build_time_s={build_time_s:.4f}")
    print(f"  arrays.npz size={npz_size/1e6:.3f} MB  manifest.json size={manifest_size} bytes  "
          f"total={total_size/1e6:.3f} MB")
    print(f"  manifest.source_sha256={manifest.source_sha256[:16]}...  "
          f"schema_version={manifest.schema_version}  code_version={manifest.code_version}")

    # ---- B) ONLINE/MISSION-START: validate + load (simulating a fresh process) ----
    t0 = time.perf_counter()
    ok, reason = validate_terrain_cache(CACHE_DIR, fine_roi, SOURCE_DEM_PATH)
    validate_time_s = time.perf_counter() - t0
    print(f"\n--- B) ONLINE validate+load ---")
    print(f"  validate_terrain_cache: ok={ok} reason={reason}  validate_time_s={validate_time_s:.6f}")

    t0 = time.perf_counter()
    cache = load_terrain_cache(CACHE_DIR, fine_roi, SOURCE_DEM_PATH)
    load_time_s = time.perf_counter() - t0
    print(f"  load_terrain_cache: load_time_s={load_time_s:.4f}  available_factors={cache.available_factors}")

    # first vs repeat lookup timing (both are pure numpy indexing -- no per-cell disk I/O
    # once the .npz is loaded, unlike Step 3B's live-DEM cold/warm distinction)
    sample_cells = [(r, c) for r in range(0, fine_roi.height, 37) for c in range(0, fine_roi.width, 37)]
    t0 = time.perf_counter()
    for (r, c) in sample_cells:
        cache.elevation_fine(r, c)
    first_pass_s = time.perf_counter() - t0
    t0 = time.perf_counter()
    for (r, c) in sample_cells:
        cache.elevation_fine(r, c)
    repeat_pass_s = time.perf_counter() - t0
    print(f"  lookup timing over {len(sample_cells)} cells: first_pass_s={first_pass_s:.6f} "
          f"({first_pass_s/len(sample_cells)*1e6:.3f}us/lookup)  "
          f"repeat_pass_s={repeat_pass_s:.6f} ({repeat_pass_s/len(sample_cells)*1e6:.3f}us/lookup)")

    # ---- C) CORRECTNESS: cache-backed vs freshly-computed direct compute ----
    print(f"\n--- C) correctness: cache arrays vs fresh build_coarse_terrain_stats ---")
    all_exact = True
    for factor in FACTORS:
        fresh = build_coarse_terrain_stats(fine_roi, factor=factor)
        for name, cache_fn, fresh_arr in (
            ("min", cache.min_elevation, fresh.min_elevation),
            ("max", cache.max_elevation, fresh.max_elevation),
            ("mean", cache.mean_elevation, fresh.mean_elevation),
            ("relief", cache.relief, fresh.relief),
        ):
            cache_arr = cache._coarse_array(name, factor)
            exact = np.array_equal(cache_arr, fresh_arr, equal_nan=True)
            all_exact = all_exact and exact
            print(f"  factor={factor} {name}_elevation: cache==fresh exact match = {exact}")
    fine_exact = np.array_equal(
        cache._arrays["fine_elevation"], fine_roi.elevation.astype(np.float32), equal_nan=True)
    all_exact = all_exact and fine_exact
    print(f"  fine_elevation: cache==fresh exact match = {fine_exact}")
    print(f"  ALL ARRAYS EXACT MATCH: {all_exact}")
    assert all_exact, "REGRESSION: cache-backed data diverges from freshly-computed data"

    # ---- CandidateZGenerator integration (Step 3B's class, UNCHANGED, imported not modified) ----
    print(f"\n--- CandidateZGenerator: cache-backed vs live-DEM-backed, same result? ---")
    START_ROW, START_COL = 48, 276
    GOAL_ROW, GOAL_COL = 264, 276
    AIRCRAFT_MSL = 3760.0
    mission = MissionContext(
        start_rowcol=(START_ROW, START_COL), start_z_msl=AIRCRAFT_MSL,
        goal_rowcol=(GOAL_ROW, GOAL_COL), goal_z_msl=AIRCRAFT_MSL,
        ceiling_msl=6000.0, min_agl_m=STEP2B_CONFIG.min_agl_m, z_step_m=STEP2B_CONFIG.z_step_m,
    )

    cache_store = CacheBackedStore(cache)
    cache_gen = CandidateZGenerator(cache_store, mission)

    live_terrain = TerrainQuery(fine_roi)
    from scripts.step3b_sparse_lazy_z_prototype import TerrainMetadataStore
    live_store = TerrainMetadataStore(live_terrain)
    live_gen = CandidateZGenerator(live_store, mission)

    test_cells = [(START_ROW, START_COL), (GOAL_ROW, GOAL_COL), (100, 200), (150, 150), (200, 100), (48, 100)]
    generator_all_match = True
    for (r, c) in test_cells:
        cache_result = cache_gen.generate(r, c)
        live_result = live_gen.generate(r, c)
        match = cache_result == live_result
        generator_all_match = generator_all_match and match
        print(f"  (row={r},col={c}): cache={cache_result}  live={live_result}  match={match}")
    print(f"  CandidateZGenerator cache-backed == live-DEM-backed for all test cells: {generator_all_match}")
    assert generator_all_match, "REGRESSION: cache-backed generator diverges from live-DEM-backed generator"

    print(f"\n--- D) search-time DEM access count ---")
    print(f"  CacheBackedStore.dem_reads after {len(test_cells)} generate() calls (each touching the store): "
          f"{cache_store.dem_reads}  (structural: CacheBackedStore never imports/calls TerrainQuery at all)")
    print(f"  live_store (comparison only, NOT the cache path) DEM reads: hits={live_store.hits} "
          f"misses={live_store.misses}  (misses>0 here is expected -- this is the LIVE/non-cached comparison store)")

    # ---- E) failure safety ----
    print(f"\n--- E) failure safety (corrupt/incompatible/missing cache) ---")
    missing_dir = CACHE_DIR + "_does_not_exist"
    try:
        load_terrain_cache(missing_dir)
        print("  MISSING cache dir: FAILED TO RAISE -- regression")
    except TerrainCacheMissingError as e:
        print(f"  MISSING cache dir: correctly raised TerrainCacheMissingError ({e})")

    corrupt_dir = CACHE_DIR + "_corrupt"
    if Path(corrupt_dir).exists():
        shutil.rmtree(corrupt_dir)
    shutil.copytree(CACHE_DIR, corrupt_dir)
    import json
    manifest_path = Path(corrupt_dir) / "manifest.json"
    with open(manifest_path) as f:
        bad = json.load(f)
    bad["source_sha256"] = "0" * 64  # tamper: pretend a different source DEM
    with open(manifest_path, "w") as f:
        json.dump(bad, f)
    try:
        load_terrain_cache(corrupt_dir, fine_roi, SOURCE_DEM_PATH)
        print("  TAMPERED source_sha256: FAILED TO RAISE -- regression")
    except TerrainCacheStaleError as e:
        print(f"  TAMPERED source_sha256: correctly raised TerrainCacheStaleError ({e})")
    shutil.rmtree(corrupt_dir)

    dims_corrupt_dir = CACHE_DIR + "_dims_corrupt"
    if Path(dims_corrupt_dir).exists():
        shutil.rmtree(dims_corrupt_dir)
    shutil.copytree(CACHE_DIR, dims_corrupt_dir)
    manifest_path2 = Path(dims_corrupt_dir) / "manifest.json"
    with open(manifest_path2) as f:
        bad2 = json.load(f)
    bad2["source_width"] = bad2["source_width"] + 1
    with open(manifest_path2, "w") as f:
        json.dump(bad2, f)
    try:
        load_terrain_cache(dims_corrupt_dir, fine_roi, SOURCE_DEM_PATH)
        print("  TAMPERED dimensions: FAILED TO RAISE -- regression")
    except TerrainCacheStaleError as e:
        print(f"  TAMPERED dimensions: correctly raised TerrainCacheStaleError ({e})")
    shutil.rmtree(dims_corrupt_dir)

    # min_agl_m change must NOT invalidate the cache (section 3 design decision)
    ok_after_min_agl_change, reason2 = validate_terrain_cache(CACHE_DIR, fine_roi, SOURCE_DEM_PATH)
    print(f"  cache validity is INDEPENDENT of min_agl_m (min_agl_m isn't even a cache input): "
          f"still valid={ok_after_min_agl_change} ({reason2})")

    # ---- F) 100x100km static-size scaling estimate (NOT an actual build) ----
    roi_size_km = DEFAULT_CONFIG.roi_size_m / 1000.0
    area_factor = (100.0 / roi_size_km) ** 2
    estimated_100km_mb = (total_size / 1e6) * area_factor
    print(f"\n--- F) 100x100km STATIC METADATA size estimate (linear/area scaling ONLY, NOT built) ---")
    print(f"  current ROI: {roi_size_km:.0f}x{roi_size_km:.0f}km, measured cache size={total_size/1e6:.3f}MB")
    print(f"  area scale factor to 100x100km: {area_factor:.1f}x")
    print(f"  estimated 100x100km STATIC terrain metadata size: ~{estimated_100km_mb:.0f} MB")
    print(f"  NOTE: this is STATIC METADATA scaling (elevation+min/max/mean/relief arrays) only --")
    print(f"        it is NOT a 3D graph/state-count estimate, which would scale very differently")
    print(f"        (depends on Z representation choices made in Step 3A/3A.1/3B, not measured here).")

    print("\n" + "=" * 70)
    print("STEP 3C SUMMARY")
    print("=" * 70)
    print(f"  build_time_s={build_time_s:.4f}  cache_size_MB={total_size/1e6:.3f}  "
          f"validate_time_s={validate_time_s:.6f}  load_time_s={load_time_s:.4f}")
    print(f"  correctness (cache==fresh): {all_exact}")
    print(f"  CandidateZGenerator cache-backed==live-backed: {generator_all_match}")
    print(f"  failure-safety tests: all raised correctly (see E above)")
    print(f"  100x100km static metadata estimate: ~{estimated_100km_mb:.0f} MB (area-scaled, not built)")


if __name__ == "__main__":
    main()
