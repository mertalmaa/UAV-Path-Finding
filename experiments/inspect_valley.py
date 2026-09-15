import math
import sys
import numpy as np
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from planner.roi import load_roi
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from scripts.benchmark_missions import CACHE_DIR, CONFIG, FACTOR, SOURCE_DEM_PATH

roi = load_roi(CONFIG)
cache = load_terrain_cache(CACHE_DIR, roi, SOURCE_DEM_PATH)
terrain = build_terrain_query_from_cache(cache, roi, FACTOR)

elev = terrain.roi.elevation
print("Elev grid shape:", elev.shape)

for r in range(15, 80, 5):
    row_elevs = [f"{c}:{elev[r, c]:.0f}" for c in range(3, 20)]
    print(f"Row {r:2d} -> " + " ".join(row_elevs))
