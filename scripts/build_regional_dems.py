"""Build 90x90 km regional working DEMs with 5 km buffer margins (100x100 km working DEM)
for Bilecik, Mugla, Ankara, and Aladaglar.

Replicates and scales the exact Copernicus GLO-30 processing pipeline:
  1. Identifies and mosaics all required Copernicus GLO-30 source tiles (EPSG:4326).
  2. Reprojects to UTM at pinned 30.0 m pixel resolution using block-maximum resampling (Resampling.max)
     to strictly preserve peaks for terrain avoidance safety.
  3. Working DEM covers 100x100 km box: 90x90 km ROI + 5 km buffer margin on every side (HALF_EXTENT_M = 50_000.0).
  4. Extracts 90x90 km fine ROI raster (3000x3000 px).
  5. Computes conservative coarse 90m stats (factor=3, 1000x1000 px: max, min, mean, relief) via planner.coarse.
  6. Generates persistent terrain cache (manifest.json + arrays.npz for factors [2, 3]) via planner.terrain_cache.
  7. Stores outputs in separate folders: regions/<region>/ and working_dem/<region>/.
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
from rasterio.io import MemoryFile
from rasterio.merge import merge
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
    Path("copernicus_glo30_turkey"),
    Path("C:/Users/PC_10004_YD26/Desktop/uav_pathfinder/copernicus_glo30_turkey"),
]

# Extents for 90x90 km ROI
ROI_SIZE_M = 90_000.0
BUFFER_MARGIN_M = 5_000.0
WORKING_EXTENT_M = ROI_SIZE_M + 2 * BUFFER_MARGIN_M  # 100_000.0 m (100x100 km)
HALF_EXTENT_M = WORKING_EXTENT_M / 2.0  # 50_000.0 m
ROI_HALF_M = ROI_SIZE_M / 2.0  # 45_000.0 m

DST_NODATA = -9999.0
PIXEL_RES_M = 30.0


@dataclass(frozen=True)
class RegionSpec:
    id: str
    name: str
    center_lonlat: Tuple[float, float]
    target_crs: str
    description: str
    canyon_features: str


REGIONS = [
    RegionSpec(
        id="bilecik",
        name="Bilecik - Sakarya Vadisi ve Kanyonları",
        center_lonlat=(30.30, 40.25),
        target_crs="EPSG:32636",
        description="Sakarya Nehri kanyon geçişi, Vezirhan, Osmaneli, Gölpazarı ve İnhisar vadileri. Taban rakımı 34m, çevre tepeler 600-1683m.",
        canyon_features="Genişletilmiş 90x90 km alan: Sakarya Kanyonu, Bilecik vadileri, güneyde Bozüyük/Eskişehir platosu ve kuzeyde İznik/Adapazarı havzası geçişleri.",
    ),
    RegionSpec(
        id="mugla",
        name="Muğla - Gökova Körfezi, Ula Kanyonu ve Sakar Geçidi",
        center_lonlat=(28.35, 37.22),
        target_crs="EPSG:32636",
        description="Gökova Körfezi, Datça-Marmaris koridoru, Sakar Geçidi, Ula kanyonu, Milas ve Yatağan havzası. Taban 0m (deniz), zirveler 1891m.",
        canyon_features="Deniz seviyesinden 1800m+ plato ve dağ zirvelerine dik falezler, karmaşık kıyı-dağ topografyası ve derin kanyon koridorları.",
    ),
    RegionSpec(
        id="ankara",
        name="Ankara - Güdül, Kirmir Çayı Kanyonu ve Ayaş",
        center_lonlat=(32.25, 40.25),
        target_crs="EPSG:32636",
        description="Kirmir Çayı Kanyonu, Güdül, Ayaş, Beypazarı, Kızılcahamam ve Çamlıdere volkanik plato ve dağlık arazisi. Taban 460m, tepeler 2000m+.",
        canyon_features="Kirmir Çayı boyunca uzanan dar kanyon tabanı, dik volkanik tepeler, derin vadiler ve tepe aşma (ridge hopping) rotaları.",
    ),
    RegionSpec(
        id="aladaglar",
        name="Aladağlar - Demirkazık Zirvesi ve Çamardı Kanyonları",
        center_lonlat=(35.15, 37.81),
        target_crs="EPSG:32636",
        description="Toros Dağları'nın en sarp ve yüksek silsilesi, Demirkazık (3756m), Bolkar Dağları, Ecemiş Fayı koridoru, Çamardı ve Pozantı kanyonları.",
        canyon_features="1000m-3756m arası sarp duvarlar, derin kanyon yarıkları, aşırı irtifa değişimleri ve yüksek irtifa dağ aşma test alanı.",
    ),
]


def find_available_tiles() -> List[Path]:
    for d in COPERNICUS_DIRS:
        if d.exists():
            tiles = list(d.glob("*.tif"))
            if tiles:
                return tiles
    raise FileNotFoundError(f"No DEM tiles found in {[str(d) for d in COPERNICUS_DIRS]}")


def get_intersecting_tiles(spec: RegionSpec, all_tiles: List[Path]) -> List[Path]:
    to_utm = Transformer.from_crs("EPSG:4326", spec.target_crs, always_xy=True)
    to_wgs = Transformer.from_crs(spec.target_crs, "EPSG:4326", always_xy=True)
    cx, cy = to_utm.transform(spec.center_lonlat[0], spec.center_lonlat[1])

    corners_utm = [
        (cx - HALF_EXTENT_M, cy - HALF_EXTENT_M),
        (cx + HALF_EXTENT_M, cy - HALF_EXTENT_M),
        (cx + HALF_EXTENT_M, cy + HALF_EXTENT_M),
        (cx - HALF_EXTENT_M, cy + HALF_EXTENT_M),
    ]
    corners_wgs = [to_wgs.transform(x, y) for x, y in corners_utm]
    min_lon = min(c[0] for c in corners_wgs)
    max_lon = max(c[0] for c in corners_wgs)
    min_lat = min(c[1] for c in corners_wgs)
    max_lat = max(c[1] for c in corners_wgs)

    matching = []
    for t in all_tiles:
        with rasterio.open(t) as src:
            b = src.bounds
            if not (b.right <= min_lon or b.left >= max_lon or b.top <= min_lat or b.bottom >= max_lat):
                matching.append(t)
    return matching


def clean_old_30km_files(region_dir: Path) -> None:
    """Remove old 30x30 km files from region directory."""
    old_files = [
        "roi_30km_dem.tif",
        "roi_dem.tif",
    ]
    for fname in old_files:
        p = region_dir / fname
        if p.exists():
            p.unlink()
            print(f"    Removed obsolete file: {p.name}")


def process_region(spec: RegionSpec, all_tiles: List[Path], base_output_dir: Path) -> Dict:
    print(f"\n{'='*70}")
    print(f"Processing 90x90 km Region: {spec.name} ({spec.id})")
    print(f"{'='*70}")

    matching_tiles = get_intersecting_tiles(spec, all_tiles)
    print(f"  Center (lon, lat): {spec.center_lonlat}")
    print(f"  Target CRS: {spec.target_crs}")
    print(f"  Intersecting Copernicus tiles ({len(matching_tiles)}):")
    for t in sorted(matching_tiles):
        print(f"    - {t.name}")

    region_dir = base_output_dir / spec.id
    region_dir.mkdir(parents=True, exist_ok=True)
    clean_old_30km_files(region_dir)

    working_dem_path = region_dir / "working_dem.tif"
    roi_dem_path = region_dir / "roi_90km_dem.tif"
    cache_dir = region_dir / "terrain_cache"

    # Step 1: Merge source tiles in EPSG:4326
    t0 = time.time()
    src_datasets = [rasterio.open(t) for t in matching_tiles]
    mosaic_arr, mosaic_transform = merge(src_datasets, method="max")
    mosaic_profile = src_datasets[0].profile.copy()
    mosaic_profile.update(
        height=mosaic_arr.shape[1],
        width=mosaic_arr.shape[2],
        transform=mosaic_transform,
        nodata=DST_NODATA,
    )
    for s in src_datasets:
        s.close()
    print(f"  Mosaicked {len(matching_tiles)} tiles in {time.time() - t0:.2f}s (EPSG:4326 shape: {mosaic_arr.shape})")

    # Step 2: Reproject to UTM with Resampling.max and crop working & ROI windows
    with MemoryFile() as memfile:
        with memfile.open(**mosaic_profile) as mem_src:
            mem_src.write(mosaic_arr)

            dst_transform, dst_width, dst_height = calculate_default_transform(
                mem_src.crs,
                spec.target_crs,
                mem_src.width,
                mem_src.height,
                *mem_src.bounds,
                resolution=(PIXEL_RES_M, PIXEL_RES_M),
            )

            with WarpedVRT(
                mem_src,
                crs=spec.target_crs,
                transform=dst_transform,
                width=dst_width,
                height=dst_height,
                resampling=Resampling.max,
                nodata=DST_NODATA,
            ) as vrt:
                to_working = Transformer.from_crs("EPSG:4326", spec.target_crs, always_xy=True)
                cx, cy = to_working.transform(spec.center_lonlat[0], spec.center_lonlat[1])

                # 100x100 km working DEM window (90 km ROI + 5 km margin each side)
                window_100km = from_bounds(
                    cx - HALF_EXTENT_M,
                    cy - HALF_EXTENT_M,
                    cx + HALF_EXTENT_M,
                    cy + HALF_EXTENT_M,
                    transform=vrt.transform,
                ).round_lengths().round_offsets()

                data_100km = vrt.read(1, window=window_100km)
                transform_100km = vrt.window_transform(window_100km)

                profile_100km = vrt.profile.copy()
                profile_100km.update(
                    count=1,
                    height=data_100km.shape[0],
                    width=data_100km.shape[1],
                    transform=transform_100km,
                    nodata=DST_NODATA,
                    dtype=data_100km.dtype,
                    driver="GTiff",
                    compress="deflate",
                )

                # Write working_dem.tif (100x100 km)
                with rasterio.open(working_dem_path, "w", **profile_100km) as dst:
                    dst.write(data_100km, 1)

                # Extract exact 90x90 km ROI
                window_90km = from_bounds(
                    cx - ROI_HALF_M,
                    cy - ROI_HALF_M,
                    cx + ROI_HALF_M,
                    cy + ROI_HALF_M,
                    transform=vrt.transform,
                ).round_lengths().round_offsets()

                data_90km = vrt.read(1, window=window_90km)
                transform_90km = vrt.window_transform(window_90km)

                profile_90km = vrt.profile.copy()
                profile_90km.update(
                    count=1,
                    height=data_90km.shape[0],
                    width=data_90km.shape[1],
                    transform=transform_90km,
                    nodata=DST_NODATA,
                    dtype=data_90km.dtype,
                    driver="GTiff",
                    compress="deflate",
                )

                with rasterio.open(roi_dem_path, "w", **profile_90km) as dst:
                    dst.write(data_90km, 1)

    # Validate 100km & 90km rasters
    valid_100km = data_100km[data_100km != DST_NODATA]
    valid_90km = data_90km[data_90km != DST_NODATA]
    nodata_count_100km = int(np.sum(data_100km == DST_NODATA))
    nodata_count_90km = int(np.sum(data_90km == DST_NODATA))

    print(f"  Working DEM (100x100 km with 5 km buffer): {working_dem_path}")
    print(f"    Dimensions: {data_100km.shape[1]}x{data_100km.shape[0]} px ({data_100km.shape[1]*PIXEL_RES_M/1000:.1f}x{data_100km.shape[0]*PIXEL_RES_M/1000:.1f} km)")
    print(f"    Elevation (valid px): {valid_100km.min():.1f} - {valid_100km.max():.1f} m (median {np.median(valid_100km):.1f} m)")
    print(f"    NoData px: {nodata_count_100km} / {data_100km.size}")

    print(f"  ROI DEM (90x90 km): {roi_dem_path}")
    print(f"    Dimensions: {data_90km.shape[1]}x{data_90km.shape[0]} px ({data_90km.shape[1]*PIXEL_RES_M/1000:.1f}x{data_90km.shape[0]*PIXEL_RES_M/1000:.1f} km)")
    print(f"    Elevation (valid px): {valid_90km.min():.1f} - {valid_90km.max():.1f} m (median {np.median(valid_90km):.1f} m)")
    print(f"    NoData px: {nodata_count_90km} / {data_90km.size}")

    # Step 3: Build ROIData object for 90x90 km ROI
    roi_data = ROIData(
        elevation=data_90km,
        transform=transform_90km,
        crs=spec.target_crs,
        width=data_90km.shape[1],
        height=data_90km.shape[0],
        bounds=window_bounds(window_90km, transform_90km),
        resolution=(abs(transform_90km.a), abs(transform_90km.e)),
        nodata=DST_NODATA,
    )

    # Step 4: Compute coarse 90m rasters (factor=3) -> 1000x1000 px
    print("  Generating coarse 90m terrain stats (factor=3, 1000x1000 px)...")
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
        "source_tiles": [t.name for t in sorted(matching_tiles)],
        "center_lonlat": spec.center_lonlat,
        "center_utm_xy": [cx, cy],
        "target_crs": spec.target_crs,
        "roi_size_m": ROI_SIZE_M,
        "buffer_margin_m": BUFFER_MARGIN_M,
        "working_extent_m": WORKING_EXTENT_M,
        "elevation_stats": {
            "working_dem_min_m": float(valid_100km.min()),
            "working_dem_max_m": float(valid_100km.max()),
            "working_dem_median_m": float(np.median(valid_100km)),
            "roi_min_m": float(valid_90km.min()),
            "roi_max_m": float(valid_90km.max()),
            "roi_median_m": float(np.median(valid_90km)),
        },
        "working_dem_shape": list(data_100km.shape),
        "roi_dem_shape": list(data_90km.shape),
        "coarse_90m_shape": list(coarse_stats.max_elevation.shape),
        "files": {
            "working_dem": str(working_dem_path.relative_to(base_output_dir)),
            "roi_90km_dem": str(roi_dem_path.relative_to(base_output_dir)),
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
    print("TURKEY REGIONAL 90x90 KM DEM BUILDER (100x100 km working DEM)")
    print("======================================================================")

    all_tiles = find_available_tiles()
    print(f"Total available Copernicus tiles: {len(all_tiles)}")

    all_infos = {}
    for spec in REGIONS:
        info = process_region(spec, all_tiles, regions_root)
        all_infos[spec.id] = info

        # Mirror into working_dem/<region>/
        target_wd = working_dem_root / spec.id
        target_wd.mkdir(parents=True, exist_ok=True)
        clean_old_30km_files(target_wd)

        source_dir = regions_root / spec.id
        for f in source_dir.glob("*.tif"):
            shutil.copy2(f, target_wd / f.name)
        shutil.copytree(source_dir / "terrain_cache", target_wd / "terrain_cache", dirs_exist_ok=True)
        # Update manifest.json in mirrored terrain_cache to point to its own working_dem.tif
        wd_manifest_path = target_wd / "terrain_cache" / "manifest.json"
        if wd_manifest_path.exists():
            with open(wd_manifest_path, "r", encoding="utf-8") as mf:
                mdata = json.load(mf)
            mdata["source_dem_path"] = str(target_wd / "working_dem.tif")
            with open(wd_manifest_path, "w", encoding="utf-8") as mf:
                json.dump(mdata, mf, indent=2)
        shutil.copy2(source_dir / "region_info.json", target_wd / "region_info.json")
        print(f"  Mirrored 90x90 km outputs to {target_wd}")

    # Write overall manifest
    with open(regions_root / "regions_manifest.json", "w", encoding="utf-8") as f:
        json.dump(all_infos, f, indent=2, ensure_ascii=False)

    elapsed = time.time() - t0
    print(f"\n{'='*70}")
    print(f"ALL 4 REGIONS (90x90 km) BUILT SUCCESSFULLY in {elapsed:.2f} seconds!")
    print(f"Output directories:")
    for spec in REGIONS:
        print(f"  - regions/{spec.id}/ and working_dem/{spec.id}/")
    print(f"{'='*70}")


if __name__ == "__main__":
    main()
