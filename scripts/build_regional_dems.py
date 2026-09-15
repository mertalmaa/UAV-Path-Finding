"""Build 30x30 km regional working DEMs with 5 km buffer margins for Bilecik, Mugla, and Ankara.

Replicates the exact Copernicus GLO-30 processing pipeline from scripts/build_working_dem.py:
  1. Reproject source GLO-30 tile (EPSG:4326) to UTM at pinned 30.0 m pixel resolution.
  2. Use block-maximum resampling (Resampling.max) to preserve peaks for terrain avoidance safety.
  3. Working DEM covers 40x40 km box: 30x30 km ROI + 5 km buffer margin on every side (HALF_EXTENT_M = 20_000.0).
  4. Extract 30x30 km fine ROI raster (1000x1000 px).
  5. Compute conservative coarse 90m stats (factor=3: max, min, mean, relief) via planner.coarse.
  6. Generate persistent terrain cache (manifest.json + arrays.npz) via planner.terrain_cache.
  7. Store outputs in separate folders: regions/<region>/ and working_dem/<region>/.
"""
from dataclasses import dataclass
import json
import os
from pathlib import Path
import shutil
import time
from typing import Dict, List, Tuple

import numpy as np
import rasterio
from affine import Affine
from pyproj import Transformer
from rasterio.enums import Resampling
from rasterio.vrt import WarpedVRT
from rasterio.warp import calculate_default_transform
from rasterio.windows import bounds as window_bounds
from rasterio.windows import from_bounds

import sys

# Ensure repository root is on sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from planner.coarse import build_coarse_terrain_stats
from planner.roi import ROIData
from planner.terrain_cache import build_terrain_cache, validate_terrain_cache

# Copernicus tiles search paths
COPERNICUS_DIRS = [
    Path("C:/Users/merta/copernicus_glo30_turkey"),
    Path("copernicus_glo30_turkey"),
]

# Extents
ROI_SIZE_M = 30_000.0
BUFFER_MARGIN_M = 5_000.0
WORKING_EXTENT_M = ROI_SIZE_M + 2 * BUFFER_MARGIN_M  # 40_000.0 m (40x40 km)
HALF_EXTENT_M = WORKING_EXTENT_M / 2.0  # 20_000.0 m
ROI_HALF_M = ROI_SIZE_M / 2.0  # 15_000.0 m

DST_NODATA = -9999.0
PIXEL_RES_M = 30.0


@dataclass(frozen=True)
class RegionSpec:
    id: str
    name: str
    source_tile_filename: str
    center_lonlat: Tuple[float, float]
    target_crs: str
    description: str
    canyon_features: str


REGIONS = [
    RegionSpec(
        id="bilecik",
        name="Bilecik - Sakarya Vadisi ve Kanyonları",
        source_tile_filename="Copernicus_DSM_COG_10_N40_00_E030_00_DEM.tif",
        center_lonlat=(30.30, 40.25),
        target_crs="EPSG:32636",
        description="Sakarya Nehri kanyon geçişi, Vezirhan, Osmaneli ve Gölpazarı vadileri. Taban rakımı 97m, çevre tepeler 600-1200m.",
        canyon_features="Kanyon derinliği 400-800m. Dik vadi yamaçları, nehir kıvrımları ve alçak irtifa vadi içi uçuş / alçalma manevraları için ideal.",
    ),
    RegionSpec(
        id="mugla",
        name="Muğla - Gökova Körfezi, Ula Kanyonu ve Sakar Geçidi",
        source_tile_filename="Copernicus_DSM_COG_10_N37_00_E028_00_DEM.tif",
        center_lonlat=(28.35, 37.22),
        target_crs="EPSG:32636",
        description="Gökova Körfezi kıyısından (0m) Sakar Geçidi ve Ula kanyonu boyunca Muğla platosuna (700-1891m) yükselen dik topoğrafya.",
        canyon_features="Deniz seviyesinden 1000+ metreye 3 km içinde tırmanan falezler, derin kanyonlar ve plato alçalma/tırmanma koridorları.",
    ),
    RegionSpec(
        id="ankara",
        name="Ankara - Güdül, Kirmir Çayı Kanyonu ve Ayaş",
        source_tile_filename="Copernicus_DSM_COG_10_N40_00_E032_00_DEM.tif",
        center_lonlat=(32.25, 40.25),
        target_crs="EPSG:32636",
        description="Kirmir Çayı Kanyonu, kaya yerleşimleri vadisi, Güdül ve Ayaş volkanik platoları. Taban 518m, tepeler 1000-1984m.",
        canyon_features="Kirmir Çayı boyunca uzanan dar kanyon tabanı, dalgalı tepeler ve tepe aşma/vadi içi süzülüş için mükemmel test alanı.",
    ),
]


def find_source_tile(filename: str) -> Path:
    for d in COPERNICUS_DIRS:
        p = d / filename
        if p.exists():
            return p
    raise FileNotFoundError(
        f"Tile {filename} not found in {[str(d) for d in COPERNICUS_DIRS]}"
    )


def process_region(spec: RegionSpec, base_output_dir: Path) -> Dict:
    print(f"\n{'='*70}")
    print(f"Processing Region: {spec.name} ({spec.id})")
    print(f"{'='*70}")

    source_path = find_source_tile(spec.source_tile_filename)
    print(f"  Source tile: {source_path}")
    print(f"  Center (lon, lat): {spec.center_lonlat}")
    print(f"  Target CRS: {spec.target_crs}")

    region_dir = base_output_dir / spec.id
    region_dir.mkdir(parents=True, exist_ok=True)

    working_dem_path = region_dir / "working_dem.tif"
    roi_dem_path = region_dir / "roi_30km_dem.tif"
    cache_dir = region_dir / "terrain_cache"

    # Step 1: Warp & crop 40x40 km Working DEM with 5km buffer
    with rasterio.open(source_path) as src:
        dst_transform, dst_width, dst_height = calculate_default_transform(
            src.crs,
            spec.target_crs,
            src.width,
            src.height,
            *src.bounds,
            resolution=(PIXEL_RES_M, PIXEL_RES_M),
        )

        with WarpedVRT(
            src,
            crs=spec.target_crs,
            transform=dst_transform,
            width=dst_width,
            height=dst_height,
            resampling=Resampling.max,
            nodata=DST_NODATA,
        ) as vrt:
            to_working = Transformer.from_crs("EPSG:4326", spec.target_crs, always_xy=True)
            cx, cy = to_working.transform(spec.center_lonlat[0], spec.center_lonlat[1])

            # 40x40 km working DEM window (30 km ROI + 5 km margin each side)
            window_40km = from_bounds(
                cx - HALF_EXTENT_M,
                cy - HALF_EXTENT_M,
                cx + HALF_EXTENT_M,
                cy + HALF_EXTENT_M,
                transform=vrt.transform,
            ).round_lengths().round_offsets()

            data_40km = vrt.read(1, window=window_40km)
            transform_40km = vrt.window_transform(window_40km)

            profile_40km = vrt.profile.copy()
            profile_40km.update(
                count=1,
                height=data_40km.shape[0],
                width=data_40km.shape[1],
                transform=transform_40km,
                nodata=DST_NODATA,
                dtype=data_40km.dtype,
                driver="GTiff",
                compress="deflate",
            )

            # Write working_dem.tif (40x40 km)
            with rasterio.open(working_dem_path, "w", **profile_40km) as dst:
                dst.write(data_40km, 1)

            # Step 2: Extract exact 30x30 km ROI from the working DEM
            window_30km = from_bounds(
                cx - ROI_HALF_M,
                cy - ROI_HALF_M,
                cx + ROI_HALF_M,
                cy + ROI_HALF_M,
                transform=vrt.transform,
            ).round_lengths().round_offsets()

            data_30km = vrt.read(1, window=window_30km)
            transform_30km = vrt.window_transform(window_30km)

            profile_30km = vrt.profile.copy()
            profile_30km.update(
                count=1,
                height=data_30km.shape[0],
                width=data_30km.shape[1],
                transform=transform_30km,
                nodata=DST_NODATA,
                dtype=data_30km.dtype,
                driver="GTiff",
                compress="deflate",
            )

            with rasterio.open(roi_dem_path, "w", **profile_30km) as dst:
                dst.write(data_30km, 1)

    # Validate 40km & 30km rasters
    valid_40km = data_40km[data_40km != DST_NODATA]
    valid_30km = data_30km[data_30km != DST_NODATA]
    nodata_count_40km = int(np.sum(data_40km == DST_NODATA))
    nodata_count_30km = int(np.sum(data_30km == DST_NODATA))

    print(f"  Working DEM (40x40 km with 5 km buffer): {working_dem_path}")
    print(f"    Dimensions: {data_40km.shape[1]}x{data_40km.shape[0]} px ({data_40km.shape[1]*PIXEL_RES_M/1000:.1f}x{data_40km.shape[0]*PIXEL_RES_M/1000:.1f} km)")
    print(f"    Elevation (valid px): {valid_40km.min():.1f} - {valid_40km.max():.1f} m (median {np.median(valid_40km):.1f} m)")
    print(f"    NoData px: {nodata_count_40km} / {data_40km.size}")

    print(f"  ROI DEM (30x30 km): {roi_dem_path}")
    print(f"    Dimensions: {data_30km.shape[1]}x{data_30km.shape[0]} px ({data_30km.shape[1]*PIXEL_RES_M/1000:.1f}x{data_30km.shape[0]*PIXEL_RES_M/1000:.1f} km)")
    print(f"    Elevation (valid px): {valid_30km.min():.1f} - {valid_30km.max():.1f} m (median {np.median(valid_30km):.1f} m)")
    print(f"    NoData px: {nodata_count_30km} / {data_30km.size}")

    # Step 3: Build ROIData object for 30x30 km ROI
    roi_data = ROIData(
        elevation=data_30km,
        transform=transform_30km,
        crs=spec.target_crs,
        width=data_30km.shape[1],
        height=data_30km.shape[0],
        bounds=window_bounds(window_30km, transform_30km),
        resolution=(abs(transform_30km.a), abs(transform_30km.e)),
        nodata=DST_NODATA,
    )

    # Step 4: Compute coarse 90m rasters (factor=3)
    print("  Generating coarse 90m terrain stats (factor=3)...")
    coarse_stats = build_coarse_terrain_stats(roi_data, factor=3)

    coarse_layers = {
        "coarse_90m_max.tif": coarse_stats.max_elevation,
        "coarse_90m_min.tif": coarse_stats.min_elevation,
        "coarse_90m_mean.tif": coarse_stats.mean_elevation,
        "coarse_90m_relief.tif": coarse_stats.relief,
    }

    coarse_profile = {
        "driver": "GTiff",
        "count": 1,
        "height": coarse_stats.max_elevation.shape[0],
        "width": coarse_stats.max_elevation.shape[1],
        "crs": coarse_stats.crs,
        "transform": coarse_stats.transform,
        "dtype": "float32",
        "nodata": DST_NODATA,
        "compress": "deflate",
    }

    for fname, arr in coarse_layers.items():
        cpath = region_dir / fname
        with rasterio.open(cpath, "w", **coarse_profile) as dst:
            dst.write(arr.astype(np.float32), 1)
        print(f"    Wrote {fname} ({arr.shape[1]}x{arr.shape[0]} px, 90m)")

    # Step 5: Build persistent terrain cache (factors 2 and 3)
    print("  Building persistent terrain cache (arrays.npz & manifest.json)...")
    manifest = build_terrain_cache(
        fine_roi=roi_data,
        source_dem_path=str(working_dem_path),
        factors=[2, 3],
        out_dir=str(cache_dir),
    )
    ok, reason = validate_terrain_cache(str(cache_dir), roi_data, str(working_dem_path))
    if not ok:
        raise RuntimeError(f"Terrain cache validation failed for {spec.id}: {reason}")
    print(f"    Cache built and validated successfully: {reason}")

    # Step 6: Write region_info.json
    info = {
        "region_id": spec.id,
        "region_name": spec.name,
        "description": spec.description,
        "canyon_features": spec.canyon_features,
        "source_tile": spec.source_tile_filename,
        "center_lonlat": spec.center_lonlat,
        "center_utm_xy": [cx, cy],
        "target_crs": spec.target_crs,
        "roi_size_m": ROI_SIZE_M,
        "buffer_margin_m": BUFFER_MARGIN_M,
        "working_extent_m": WORKING_EXTENT_M,
        "elevation_stats": {
            "working_dem_min_m": float(valid_40km.min()),
            "working_dem_max_m": float(valid_40km.max()),
            "working_dem_median_m": float(np.median(valid_40km)),
            "roi_min_m": float(valid_30km.min()),
            "roi_max_m": float(valid_30km.max()),
            "roi_median_m": float(np.median(valid_30km)),
        },
        "working_dem_shape": list(data_40km.shape),
        "roi_dem_shape": list(data_30km.shape),
        "coarse_90m_shape": list(coarse_stats.max_elevation.shape),
        "files": {
            "working_dem": str(working_dem_path.relative_to(base_output_dir)),
            "roi_30km_dem": str(roi_dem_path.relative_to(base_output_dir)),
            "coarse_90m_max": str((region_dir / "coarse_90m_max.tif").relative_to(base_output_dir)),
            "coarse_90m_min": str((region_dir / "coarse_90m_min.tif").relative_to(base_output_dir)),
            "coarse_90m_mean": str((region_dir / "coarse_90m_mean.tif").relative_to(base_output_dir)),
            "coarse_90m_relief": str((region_dir / "coarse_90m_relief.tif").relative_to(base_output_dir)),
            "terrain_cache": str(cache_dir.relative_to(base_output_dir)),
        },
    }

    with open(region_dir / "region_info.json", "w", encoding="utf-8") as f:
        json.dump(info, f, indent=2, ensure_ascii=False)

    return info


def main() -> None:
    t0 = time.time()
    regions_root = Path("regions")
    working_dem_root = Path("working_dem")

    print("======================================================================")
    print("TURKEY REGIONAL DEM BUILDER (30x30 km ROI + 5 km buffer = 40x40 km)")
    print("======================================================================")

    all_infos = {}
    for spec in REGIONS:
        info = process_region(spec, regions_root)
        all_infos[spec.id] = info

        # Also mirror into working_dem/<region>/
        target_wd = working_dem_root / spec.id
        target_wd.mkdir(parents=True, exist_ok=True)
        # Copy working_dem.tif and coarse files for direct compatibility
        source_dir = regions_root / spec.id
        for f in source_dir.glob("*.tif"):
            shutil.copy2(f, target_wd / f.name)
        shutil.copytree(source_dir / "terrain_cache", target_wd / "terrain_cache", dirs_exist_ok=True)
        shutil.copy2(source_dir / "region_info.json", target_wd / "region_info.json")
        print(f"  Mirrored to {target_wd}")

    # Write overall manifest
    with open(regions_root / "regions_manifest.json", "w", encoding="utf-8") as f:
        json.dump(all_infos, f, indent=2, ensure_ascii=False)

    elapsed = time.time() - t0
    print(f"\n{'='*70}")
    print(f"ALL 3 REGIONS BUILT SUCCESSFULLY in {elapsed:.2f} seconds!")
    print(f"Output directories:")
    print(f"  - regions/bilecik/ and working_dem/bilecik/")
    print(f"  - regions/mugla/ and working_dem/mugla/")
    print(f"  - regions/ankara/ and working_dem/ankara/")
    print(f"{'='*70}")


if __name__ == "__main__":
    main()
