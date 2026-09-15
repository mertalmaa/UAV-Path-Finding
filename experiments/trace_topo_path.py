import math
import sys
import numpy as np
import heapq
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

elev_grid = terrain.roi.elevation.copy()
valid = np.isfinite(elev_grid)
min_e, max_e = float(np.min(elev_grid[valid])), float(np.max(elev_grid[valid]))
elev_grid[~valid] = min_e
cell_m = abs(float(terrain.roi.transform.a))
rows, cols = elev_grid.shape

dy, dx = np.gradient(elev_grid, cell_m)
slope = np.sqrt(dx*dx + dy*dy)
max_s = float(np.percentile(slope, 95))
r_norm = (elev_grid - min_e) / max(1.0, max_e - min_e)
s_norm = np.clip(slope / max(0.1, max_s), 0.0, 2.0)
cost_surf = 1.0 + 3.0 * (r_norm**2) + 2.0 * (s_norm**2)

start_rc = (70, 5)
goal_rc = (20, 5)

gr, gc = goal_rc
dist_grid = np.full((rows, cols), np.inf, dtype=np.float64)
parent = {}
dist_grid[gr, gc] = 0.0
pq = [(0.0, gr, gc)]
visited = set()
diag_m = cell_m * math.sqrt(2.0)
nbrs = [
    (-1, 0, cell_m), (1, 0, cell_m), (0, -1, cell_m), (0, 1, cell_m),
    (-1, -1, diag_m), (-1, 1, diag_m), (1, -1, diag_m), (1, 1, diag_m),
]
while pq:
    d, r, c = heapq.heappop(pq)
    if (r, c) in visited:
        continue
    visited.add((r, c))
    for dr, dc, step in nbrs:
        nr, nc = r + dr, c + dc
        if 0 <= nr < rows and 0 <= nc < cols:
            mu = 0.5 * (cost_surf[r, c] + cost_surf[nr, nc])
            nd = d + step * mu
            if nd < dist_grid[nr, nc]:
                dist_grid[nr, nc] = nd
                parent[(nr, nc)] = (r, c)
                heapq.heappush(pq, (nd, nr, nc))

# Trace optimal topographic 2D path from start to goal
curr = start_rc
topo_path = [curr]
while curr in parent and curr != goal_rc:
    curr = parent[curr]
    topo_path.append(curr)

print("Topographic Geodesic Path (Start to Goal):")
for r, c in topo_path[::2]:
    print(f"Row {r:2d}, Col {c:2d} -> Elev = {elev_grid[r, c]:.1f} m MSL, Cost = {cost_surf[r, c]:.2f}")
