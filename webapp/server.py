"""Interactive local tool: pick start/goal on the real Aladağlar DEM, get
the minimum safe altitude at each point, then run the SAME pipeline the
offline stages use (coarse guide -> fine XY+Z corridor -> fine primitive
safety precompute -> genuine ARA*) for that specific start/goal/altitude.

Not a numbered project stage -- a demo/tool built on top of the existing,
already-validated planner modules (planner.coarse_astar, planner.corridor,
planner.fine_precompute, planner.astar.ara_star_search). Nothing in those
modules is modified; this file only calls them with request-supplied
start/goal/altitude instead of the fixed constants the stage scripts used.

For interactive response time, the ARA* schedule here is shortened to
(1.7, 1.5) with a smaller cumulative expansion cap than the offline Stage
38 runs (30,000) -- this is a deliberate, disclosed scope reduction for a
"click and wait" tool (measured ~15-90s depending on corridor size, the
fine-grid precompute step dominates), not a claim that it reproduces Stage 38's
own recorded numbers.
"""
import dataclasses
import math
import time
from typing import List, Optional, Tuple

import numpy as np
from fastapi import FastAPI
from fastapi.responses import FileResponse, JSONResponse
from pydantic import BaseModel

from planner.astar import (
    _path_min_observed_agl, ara_star_search, msl_to_z_index, state_to_xyz,
)
from planner.coarse import build_coarse_dem
from planner.coarse_astar import (
    coarse_astar_search, compute_coarse_distance_reference, lift_endpoint_if_unsafe,
)
from planner.config import DEFAULT_CONFIG
from planner.corridor import build_xy_corridor_mask, build_z_guide_grid, fine_grid_centers
from planner.fine_precompute import precompute_fine_corridor_primitive_safety
from planner.primitives import build_primitive_set, evaluate_primitive
from planner.roi import load_roi
from planner.terrain import TerrainQuery

# ---------------------------------------------------------------------------
# Fixed pipeline parameters -- SAME values Stage 36-38.1 used everywhere.
# ---------------------------------------------------------------------------
ALTITUDE_REFERENCE_MSL = 3240.0
NORMALIZED_ALTITUDE_SCALE_M = 1000.0
NORMALIZED_W_ALTITUDE = 1.25
NORMALIZED_W_DISTANCE = 1.0
NORMALIZED_W_REVERSAL = 1.0
GOAL_TOLERANCE_XY_M = 35.0
GOAL_TOLERANCE_Z_M = 25.0
CORRIDOR_XY_HALF_WIDTH_M = 300.0
CORRIDOR_Z_HALF_WIDTH_M = 200.0
COARSE_FACTOR = 3
COARSE_EPSILON = 1.5
COARSE_MAX_EXPANSIONS = 25000

# Shortened for interactive latency -- see module docstring.
FINE_EPSILON_SCHEDULE = (1.7, 1.5)
FINE_MAX_EXPANSIONS_CUMULATIVE = 15000

FINE_CFG = dataclasses.replace(
    DEFAULT_CONFIG,
    cost_mode="normalized",
    altitude_reference_msl=ALTITUDE_REFERENCE_MSL,
    normalized_altitude_scale_m=NORMALIZED_ALTITUDE_SCALE_M,
    normalized_w_distance=NORMALIZED_W_DISTANCE,
    normalized_w_altitude=NORMALIZED_W_ALTITUDE,
    normalized_w_reversal=NORMALIZED_W_REVERSAL,
    goal_tolerance_xy_m=GOAL_TOLERANCE_XY_M,
    goal_tolerance_z_m=GOAL_TOLERANCE_Z_M,
)
COARSE_CFG = dataclasses.replace(
    DEFAULT_CONFIG, xy_resolution_m=90.0, z_step_m=40.0, primitive_sample_spacing_m=30.0,
    altitude_reference_msl=ALTITUDE_REFERENCE_MSL,
    normalized_altitude_scale_m=NORMALIZED_ALTITUDE_SCALE_M,
    normalized_w_distance=NORMALIZED_W_DISTANCE,
    normalized_w_altitude=NORMALIZED_W_ALTITUDE,
)

TERRAIN_SURFACE_DOWNSAMPLE = 3  # block-mean factor, matches scripts/prep_3d_visualization.py

# ---------------------------------------------------------------------------
# Loaded once at process startup -- never rebuilt per request.
# ---------------------------------------------------------------------------
print("loading ROI + primitives...")
_roi = load_roi(DEFAULT_CONFIG)
_tq = TerrainQuery(_roi)
_fine_primitives = build_primitive_set(FINE_CFG)
_coarse_primitives = build_primitive_set(COARSE_CFG)
_coarse_dem = build_coarse_dem(_roi, factor=COARSE_FACTOR)
_coarse_tq = TerrainQuery(_coarse_dem.roi)
print(f"ready: fine ROI {_roi.height}x{_roi.width}, coarse ROI {_coarse_dem.roi.height}x{_coarse_dem.roi.width}")

app = FastAPI()


class PlanRequest(BaseModel):
    start_row: int
    start_col: int
    goal_row: int
    goal_col: int
    start_altitude_msl: float
    goal_altitude_msl: float


def _min_safe_altitude(row: int, col: int) -> Optional[float]:
    cell = _tq.elevation_at_rowcol(row, col)
    if not cell.valid:
        return None
    required = cell.elevation + FINE_CFG.min_agl_m
    return math.ceil(required / FINE_CFG.z_step_m) * FINE_CFG.z_step_m


def _fine_replay(path: List[Tuple[int, int, int]]):
    violations = []
    min_agl = math.inf
    max_angle = 0.0
    by_delta = {(p.drow, p.dcol, round(p.dz_m / FINE_CFG.z_step_m)): p for p in _fine_primitives}
    for (r1, c1, z1), (r2, c2, z2) in zip(path, path[1:]):
        prim = by_delta.get((r2 - r1, c2 - c1, z2 - z1))
        if prim is None:
            violations.append({"reason": "no_matching_primitive"})
            continue
        start_xyz = state_to_xyz((r1, c1, z1), _tq, FINE_CFG)
        result = evaluate_primitive(start_xyz, prim, _tq, FINE_CFG)
        if not result.valid:
            violations.append({"reason": result.reason})
        else:
            min_agl = min(min_agl, result.min_agl_m)
        angle = math.degrees(math.atan2(abs(prim.dz_m), prim.horizontal_distance_m)) if prim.horizontal_distance_m else 0.0
        max_angle = max(max_angle, angle)
    return {"violations": violations, "min_agl": min_agl if min_agl < math.inf else None, "max_angle": max_angle}


@app.get("/")
def index():
    return FileResponse("webapp/static/index.html")


@app.get("/api/terrain")
def terrain():
    elev = _roi.elevation.astype(np.float64)
    h, w = elev.shape
    f = TERRAIN_SURFACE_DOWNSAMPLE
    h2, w2 = h // f, w // f
    cropped = elev[: h2 * f, : w2 * f]
    surf_z = cropped.reshape(h2, f, w2, f).mean(axis=(1, 3))

    surf_x = [_tq.rowcol_to_xy(0, c * f + f // 2)[0] for c in range(w2)]
    surf_y = [_tq.rowcol_to_xy(r * f + f // 2, 0)[1] for r in range(h2)]

    x2d, y2d = fine_grid_centers(_roi)
    x_coords = x2d[0, :].tolist()
    y_coords = y2d[:, 0].tolist()

    nodata_mask = np.zeros(elev.shape, dtype=bool)
    if _roi.nodata is not None:
        nodata_mask = elev == _roi.nodata

    return JSONResponse({
        "height": h, "width": w,
        "x_coords": x_coords, "y_coords": y_coords,
        "elevation": elev.tolist(),
        "nodata_mask": nodata_mask.tolist(),
        "min_agl_m": FINE_CFG.min_agl_m, "z_step_m": FINE_CFG.z_step_m,
        "surface": {"x": surf_x, "y": surf_y, "z": surf_z.tolist()},
    })


@app.get("/api/min_altitude")
def min_altitude(row: int, col: int):
    if not _tq.in_bounds_rowcol(row, col):
        return JSONResponse({"ok": False, "message": "sınırlar dışında"}, status_code=400)
    val = _min_safe_altitude(row, col)
    if val is None:
        return JSONResponse({"ok": False, "message": "bu hücrede arazi verisi yok (NoData)"}, status_code=400)
    return {"ok": True, "min_safe_altitude_msl": val}


@app.post("/api/plan")
def plan(req: PlanRequest):
    t_total0 = time.perf_counter()
    start_rc = (req.start_row, req.start_col)
    goal_rc = (req.goal_row, req.goal_col)

    if start_rc == goal_rc:
        return JSONResponse({"ok": False, "stage": "validation", "message": "start ve goal aynı nokta olamaz"}, status_code=400)
    for name, (r, c), alt in [("start", start_rc, req.start_altitude_msl), ("goal", goal_rc, req.goal_altitude_msl)]:
        if not _tq.in_bounds_rowcol(r, c):
            return JSONResponse({"ok": False, "stage": "validation", "message": f"{name} sınırlar dışında"}, status_code=400)
        req_min = _min_safe_altitude(r, c)
        if req_min is None:
            return JSONResponse({"ok": False, "stage": "validation", "message": f"{name} noktasında arazi verisi yok (NoData)"}, status_code=400)
        if alt < req_min:
            return JSONResponse({
                "ok": False, "stage": "validation",
                "message": f"{name} noktasının KENDİSİ {alt:.0f}m'de güvenli değil "
                           f"(o hücrede {FINE_CFG.min_agl_m:.0f}m zemin marjı için en az {req_min:.0f}m MSL gerekiyor). "
                           f"Start ve goal FARKLI irtifalarda olabilir -- bu sadece {name} noktasının kendi tabanı.",
                "min_safe_altitude_msl": req_min,
            }, status_code=400)

    z0_start = msl_to_z_index(req.start_altitude_msl, FINE_CFG)
    z0_goal = msl_to_z_index(req.goal_altitude_msl, FINE_CFG)
    start = (req.start_row, req.start_col, z0_start)
    goal = (req.goal_row, req.goal_col, z0_goal)

    # ---- 1. coarse guide path ----
    t0 = time.perf_counter()
    coarse_start_rc = (req.start_row // COARSE_FACTOR, req.start_col // COARSE_FACTOR)
    coarse_goal_rc = (req.goal_row // COARSE_FACTOR, req.goal_col // COARSE_FACTOR)
    start_lift = lift_endpoint_if_unsafe(coarse_start_rc[0], coarse_start_rc[1], req.start_altitude_msl, _coarse_tq, COARSE_CFG)
    goal_lift = lift_endpoint_if_unsafe(coarse_goal_rc[0], coarse_goal_rc[1], req.goal_altitude_msl, _coarse_tq, COARSE_CFG)
    coarse_start = (start_lift.row, start_lift.col, start_lift.z_index)
    coarse_goal = (goal_lift.row, goal_lift.col, goal_lift.z_index)

    coarse_seg_min = float(min(
        _coarse_dem.roi.elevation[min(coarse_start_rc[0], coarse_goal_rc[0]):max(coarse_start_rc[0], coarse_goal_rc[0]) + 1,
                                   min(coarse_start_rc[1], coarse_goal_rc[1]):max(coarse_start_rc[1], coarse_goal_rc[1]) + 1].min(),
        min(req.start_altitude_msl, req.goal_altitude_msl),
    ))
    coarse_min_search = math.ceil((coarse_seg_min + COARSE_CFG.min_agl_m) / COARSE_CFG.z_step_m) * COARSE_CFG.z_step_m
    coarse_max_search = max(start_lift.msl, goal_lift.msl) + 2 * COARSE_CFG.z_step_m
    coarse_d_ref = compute_coarse_distance_reference(coarse_start, coarse_goal, _coarse_tq, COARSE_CFG)

    coarse_result = coarse_astar_search(
        coarse_start, coarse_goal, _coarse_tq, coarse_min_search, coarse_max_search, COARSE_CFG,
        primitives=_coarse_primitives, max_expansions=COARSE_MAX_EXPANSIONS,
        distance_reference_m=coarse_d_ref, altitude_reference_msl=ALTITUDE_REFERENCE_MSL,
        altitude_scale_m=NORMALIZED_ALTITUDE_SCALE_M, w_distance=NORMALIZED_W_DISTANCE, w_altitude=NORMALIZED_W_ALTITUDE,
        epsilon_search=COARSE_EPSILON,
    )
    coarse_time = time.perf_counter() - t0
    if not coarse_result.success:
        return JSONResponse({
            "ok": False, "stage": "coarse",
            "message": f"coarse rota {COARSE_MAX_EXPANSIONS} expansion içinde bulunamadı "
                       f"(en yakın mesafe: {coarse_result.closest_distance_to_goal_m:.0f}m)",
        }, status_code=200)

    coarse_path_xyz = [state_to_xyz(s, _coarse_tq, COARSE_CFG) for s in coarse_result.path]
    coarse_path_xy = [(p[0], p[1]) for p in coarse_path_xyz]

    # ---- 2. fine XY corridor + Z guide tube ----
    t0 = time.perf_counter()
    corridor_mask, _ = build_xy_corridor_mask(_roi, coarse_path_xy, CORRIDOR_XY_HALF_WIDTH_M)
    z_guide_grid = build_z_guide_grid(_roi, coarse_path_xyz)
    for (r, c), alt in ((start_rc, req.start_altitude_msl), (goal_rc, req.goal_altitude_msl)):
        diff = abs(alt - float(z_guide_grid[r, c]))
        if not (bool(corridor_mask[r, c]) and diff <= CORRIDOR_Z_HALF_WIDTH_M):
            z_guide_grid[r, c] = alt
            corridor_mask[r, c] = True
    corridor_time = time.perf_counter() - t0
    corridor_cell_count = int(corridor_mask.sum())

    # ---- 3. fine primitive safety precompute (this request's corridor only) ----
    t0 = time.perf_counter()
    precompute = precompute_fine_corridor_primitive_safety(_tq, corridor_mask, _fine_primitives, FINE_CFG)
    precompute_time = time.perf_counter() - t0

    # ---- 4. fine search bounds + ARA* ----
    corridor_elev = _roi.elevation[corridor_mask]
    if _roi.nodata is not None:
        corridor_elev = corridor_elev[corridor_elev != _roi.nodata]
    fine_seg_min = float(corridor_elev.min()) if corridor_elev.size else min(req.start_altitude_msl, req.goal_altitude_msl)
    fine_min_search = math.ceil((fine_seg_min + FINE_CFG.min_agl_m) / FINE_CFG.z_step_m) * FINE_CFG.z_step_m
    fine_max_search = max(req.start_altitude_msl, req.goal_altitude_msl) + 20.0

    t0 = time.perf_counter()
    ara_result = ara_star_search(
        start, goal, _tq, min_search_altitude_msl=fine_min_search, max_search_altitude_msl=fine_max_search,
        config=FINE_CFG, primitives=_fine_primitives, epsilon_schedule=FINE_EPSILON_SCHEDULE,
        max_expansions_cumulative=FINE_MAX_EXPANSIONS_CUMULATIVE,
        corridor_mask=corridor_mask, z_guide_grid=z_guide_grid, z_guide_tolerance_m=CORRIDOR_Z_HALF_WIDTH_M,
        fine_precompute=precompute,
    )
    ara_time = time.perf_counter() - t0

    # ---- corridor tube point cloud (for the 3D scene) ----
    rows_idx, cols_idx = np.nonzero(corridor_mask)
    stride = max(1, len(rows_idx) // 2500)
    tube_x, tube_y, tube_z = [], [], []
    for k in range(0, len(rows_idx), stride):
        r, c = int(rows_idx[k]), int(cols_idx[k])
        x, y = _tq.rowcol_to_xy(r, c)
        z_center = float(z_guide_grid[r, c])
        for lvl in np.linspace(z_center - CORRIDOR_Z_HALF_WIDTH_M, z_center + CORRIDOR_Z_HALF_WIDTH_M, 3):
            tube_x.append(x)
            tube_y.append(y)
            tube_z.append(float(lvl))

    total_time = time.perf_counter() - t_total0

    if not ara_result.path_found:
        return JSONResponse({
            "ok": False, "stage": "ara",
            "message": f"fine rota {FINE_MAX_EXPANSIONS_CUMULATIVE} expansion içinde bulunamadı -- "
                       f"start/goal irtifa farkı ({abs(req.start_altitude_msl - req.goal_altitude_msl):.0f}m) "
                       f"veya XY mesafesi bu kısaltılmış interaktif bütçe için fazla zorlayıcı olabilir; "
                       f"daha küçük bir irtifa farkı veya daha yakın iki nokta deneyin.",
            "tube": {"x": tube_x, "y": tube_y, "z": tube_z},
            "timing": {"coarse_s": coarse_time, "corridor_s": corridor_time, "precompute_s": precompute_time,
                       "ara_s": ara_time, "total_s": total_time},
        }, status_code=200)

    path = ara_result.final_incumbent_path
    xyz = [state_to_xyz(s, _tq, FINE_CFG) for s in path]
    xy_length = sum(math.hypot(xyz[i + 1][0] - xyz[i][0], xyz[i + 1][1] - xyz[i][1]) for i in range(len(xyz) - 1))
    length_3d = sum(math.sqrt((xyz[i + 1][0] - xyz[i][0]) ** 2 + (xyz[i + 1][1] - xyz[i][1]) ** 2
                               + (xyz[i + 1][2] - xyz[i][2]) ** 2) for i in range(len(xyz) - 1))
    altitudes = [p[2] for p in xyz]
    replay = _fine_replay(path)
    min_agl_direct = _path_min_observed_agl(path, _fine_primitives, _tq, FINE_CFG)
    safety_pass = (len(replay["violations"]) == 0 and min_agl_direct >= FINE_CFG.min_agl_m
                   and replay["max_angle"] <= FINE_CFG.max_climb_angle_deg)

    return {
        "ok": True,
        "start": {"row": req.start_row, "col": req.start_col, "x": xyz[0][0], "y": xyz[0][1], "z_msl": req.start_altitude_msl},
        "goal": {"row": req.goal_row, "col": req.goal_col, "x": xyz[-1][0], "y": xyz[-1][1], "z_msl": req.goal_altitude_msl},
        "coarse": {"expanded": coarse_result.expanded_nodes, "runtime_s": coarse_time, "path_nodes": len(coarse_result.path)},
        "corridor": {"cell_count": corridor_cell_count, "xy_half_width_m": CORRIDOR_XY_HALF_WIDTH_M,
                     "z_half_width_m": CORRIDOR_Z_HALF_WIDTH_M},
        "precompute": {"entry_count": precompute.entry_count, "preprocessing_time_s": precompute_time,
                       "approx_memory_mb": precompute.approx_memory_mb},
        "ara": {"epsilon_schedule": list(FINE_EPSILON_SCHEDULE), "total_expanded": ara_result.total_expanded,
                "runtime_s": ara_time, "refinement_limit_reached": ara_result.refinement_limit_reached,
                "first_incumbent_cost": ara_result.first_incumbent_cost,
                "first_incumbent_expanded": ara_result.first_incumbent_expanded,
                "final_incumbent_cost": ara_result.final_incumbent_cost},
        "tube": {"x": tube_x, "y": tube_y, "z": tube_z},
        "path": {"x": [p[0] for p in xyz], "y": [p[1] for p in xyz], "z": [p[2] for p in xyz]},
        "safety": {"min_agl": min_agl_direct, "max_angle": replay["max_angle"],
                   "violations": len(replay["violations"]), "pass": safety_pass},
        "stats": {"cost": ara_result.final_incumbent_cost, "length_3d": length_3d, "xy_length": xy_length,
                  "min_msl": min(altitudes), "max_msl": max(altitudes), "node_count": len(path)},
        "timing": {"coarse_s": coarse_time, "corridor_s": corridor_time, "precompute_s": precompute_time,
                   "ara_s": ara_time, "total_s": total_time},
    }
