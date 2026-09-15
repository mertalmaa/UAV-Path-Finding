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
print("Shape:", elev.shape)

# Let's inspect where the natural wide valley corridor is in the Western half of the DEM (cols 0 to 50)
print("Finding valley bottom (minimum elevation) for rows 10 to 120:")
for r in range(10, 130, 10):
    sub = elev[r, 5:60]
    min_idx = int(np.argmin(sub))
    best_c = 5 + min_idx
    min_elev = sub[min_idx]
    x, y = terrain.rowcol_to_xy(r, best_c)
    print(f"Row {r:3d}: best_c={best_c:2d}, elev={min_elev:.1f}m, xy=({x:.0f}, {y:.0f})")
