import math
import sys
import numpy as np
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from planner.roi import load_roi
from planner.terrain_cache import build_terrain_query_from_cache, load_terrain_cache
from planner.trajectory_safety import TerrainInfluenceCache
from scripts.benchmark_missions import CACHE_DIR, CONFIG, FACTOR, SOURCE_DEM_PATH

roi = load_roi(CONFIG)
cache = load_terrain_cache(CACHE_DIR, roi, SOURCE_DEM_PATH)
terrain = build_terrain_query_from_cache(cache, roi, FACTOR)
infl_cache = TerrainInfluenceCache(terrain)
field = infl_cache.field(CONFIG.lateral_buffer_m)

start_x, start_y = 684440.8, 4187608.1
goal_x, goal_y = 684440.8, 4190248.1

for y in np.arange(start_y, goal_y + 100, 100):
    r, c = terrain.xy_to_rowcol(start_x, y)
    q = terrain.query(start_x, y)
    buf_elev = field.elevation_msl[r, c]
    valid = field.valid[r, c]
    print(f"y={y:.0f} (r={r:2d}, c={c:2d}): Elev={q.elevation:.1f}m, BufferedElev(+20m)={buf_elev:.1f}m, Valid={valid}")
