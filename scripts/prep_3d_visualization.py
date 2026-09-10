"""One-off data export for a 3D visualization artifact (terrain surface +
translucent red coarse corridor tube + solid blue found path). Not part of
any numbered stage -- pure visualization prep, does not touch planner/
astar.py or any search logic. Writes a single compact JSON the HTML
artifact embeds directly (no server, no fetch).
"""
import csv
import json

import numpy as np

from planner.config import DEFAULT_CONFIG
from planner.corridor import build_z_guide_grid
from planner.roi import load_roi
from planner.terrain import TerrainQuery

START_ROW, START_COL = 48, 276
GOAL_ROW, GOAL_COL = 264, 276
AIRCRAFT_MSL = 3760.0
Z_TOLERANCE_M = 200.0
XY_MASK_NPY = "outputs/stage36_corridor_mask.npy"
COARSE_PATH_CSV = "outputs/stage36_coarse_path_eps1.5.csv"
FINAL_PATH_CSV = "outputs/stage38_1_ara_final_path.csv"
OUT_JSON = "scratch_3d_viz_data.json"

TERRAIN_DOWNSAMPLE = 3      # block-mean factor for the surface mesh
CORRIDOR_CELL_STRIDE = 2    # keep every Nth corridor cell (both dims combined via flat stride)
CORRIDOR_Z_LEVELS = 4       # samples through the +/-200m tube per kept cell


def main():
    cfg = DEFAULT_CONFIG
    roi = load_roi(cfg)
    tq = TerrainQuery(roi)
    corridor_mask = np.load(XY_MASK_NPY)

    # ---- terrain surface (block-mean downsample for a smooth renderable mesh) ----
    elev = roi.elevation.astype(np.float64)
    h, w = elev.shape
    f = TERRAIN_DOWNSAMPLE
    h2, w2 = h // f, w // f
    cropped = elev[: h2 * f, : w2 * f]
    blocks = cropped.reshape(h2, f, w2, f)
    surf_z = blocks.mean(axis=(1, 3))

    xs = []
    ys = []
    for c in range(w2):
        x, _ = tq.rowcol_to_xy(0, c * f + f // 2)
        xs.append(x)
    for r in range(h2):
        _, y = tq.rowcol_to_xy(r * f + f // 2, 0)
        ys.append(y)

    # ---- coarse corridor "tube": translucent point cloud over corridor XY x z_guide+/-200m ----
    coarse_path_xyz = []
    with open(COARSE_PATH_CSV) as fh:
        for row in csv.DictReader(fh):
            coarse_path_xyz.append((float(row["x"]), float(row["y"]), float(row["z_msl"])))
    z_guide_grid = build_z_guide_grid(roi, coarse_path_xyz)

    rows_idx, cols_idx = np.nonzero(corridor_mask)
    order = np.arange(len(rows_idx))
    keep = order[:: CORRIDOR_CELL_STRIDE]

    tube_x, tube_y, tube_z = [], [], []
    for k in keep:
        r, c = int(rows_idx[k]), int(cols_idx[k])
        x, y = tq.rowcol_to_xy(r, c)
        z_center = float(z_guide_grid[r, c])
        for lvl in np.linspace(z_center - Z_TOLERANCE_M, z_center + Z_TOLERANCE_M, CORRIDOR_Z_LEVELS):
            tube_x.append(x)
            tube_y.append(y)
            tube_z.append(float(lvl))

    # ---- found fine path ----
    path_x, path_y, path_z = [], [], []
    with open(FINAL_PATH_CSV) as fh:
        for row in csv.DictReader(fh):
            path_x.append(float(row["x"]))
            path_y.append(float(row["y"]))
            path_z.append(float(row["z_msl"]))

    # ---- start / goal markers ----
    sx, sy = tq.rowcol_to_xy(START_ROW, START_COL)
    gx, gy = tq.rowcol_to_xy(GOAL_ROW, GOAL_COL)

    data = {
        "terrain": {"x": xs, "y": ys, "z": surf_z.tolist()},
        "tube": {"x": tube_x, "y": tube_y, "z": tube_z},
        "path": {"x": path_x, "y": path_y, "z": path_z},
        "start": {"x": sx, "y": sy, "z": AIRCRAFT_MSL},
        "goal": {"x": gx, "y": gy, "z": AIRCRAFT_MSL},
    }
    with open(OUT_JSON, "w") as f_out:
        json.dump(data, f_out)

    print(f"terrain grid: {h2}x{w2}  tube points: {len(tube_x)}  path points: {len(path_x)}")
    print(f"written to {OUT_JSON}")


if __name__ == "__main__":
    main()
