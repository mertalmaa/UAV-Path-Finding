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
cell_m = abs(float(terrain.roi.transform.a))

# Candidate 1 for Second Valley:
# South-West Valley (East-West corridor along rows 140-150, cols 10 to 65) -> 3.3 km Eastbound
print("Candidate Second Valley (Southwest East-West Valley):")
for c in range(10, 65, 5):
    sub = elev[135:155, c]
    best_r = 135 + int(np.argmin(sub))
    min_e = sub[best_r - 135]
    print(f"Col {c:2d}: best_r={best_r:3d}, elev={min_e:.1f}m MSL")

# Candidate for Lateral Detour:
# Start at (120, 20) (elev 2050m), Goal at (90, 60) (elev 2100m).
# Direct straight line passes through a high ridge peak at (105, 40) where elev reaches 2700m+ MSL!
# A direct low-altitude flight would crash, but detouring via the southern valley (row 135) or western valley bypasses the peak!
print("\nCandidate Lateral Detour / Ridge Test:")
print("Start: (120, 20), Goal: (90, 60)")
for alpha in np.linspace(0, 1, 11):
    r = int(round(120 + alpha * (90 - 120)))
    c = int(round(20 + alpha * (60 - 20)))
    print(f"Direct Line {alpha*100:3.0f}%: (r={r:2d}, c={c:2d}) -> Elev = {elev[r, c]:.1f} m MSL")
