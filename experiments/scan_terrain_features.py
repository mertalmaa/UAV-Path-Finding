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
print("DEM Shape:", elev.shape)
print("Min elev:", np.nanmin(elev), "Max elev:", np.nanmax(elev))

# Let's inspect the Eastern sector (cols 100 to 160) for a North-South or East-West valley corridor
print("\n--- Eastern Corridor Elevation Scan (cols 110-155) ---")
for r in range(10, 160, 15):
    sub = elev[r, 110:155]
    best_c = 110 + int(np.nanargmin(sub))
    min_e = sub[best_c - 110]
    print(f"Row {r:3d}: best_c={best_c:3d}, min_elev={min_e:.1f} m MSL")

# Let's inspect the Southern sector (rows 110 to 160)
print("\n--- Southern Corridor Elevation Scan (rows 120-155) ---")
for c in range(10, 160, 15):
    sub = elev[120:155, c]
    best_r = 120 + int(np.nanargmin(sub))
    min_e = sub[best_r - 120]
    print(f"Col {c:3d}: best_r={best_r:3d}, min_elev={min_e:.1f} m MSL")
