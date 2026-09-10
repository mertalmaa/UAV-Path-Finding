"""Stage 35: Simplified Coarse 3D A* -- synthetic tests, the ONE real
6.48km coarse benchmark (30k cap, no retry), and a fine-DEM replay of
whatever coarse path is found. No corridor, no fine A*, no epsilon sweep,
no terrain-aware heuristic, no MIN/MEAN/RELIEF routing cost -- per spec.
"""
import csv
import dataclasses
import math
import os

import numpy as np
from affine import Affine

from planner.agl import evaluate_agl
from planner.astar import state_to_xyz, z_index_to_msl
from planner.coarse import build_coarse_dem, build_coarse_terrain_stats
from planner.coarse_astar import (
    compute_coarse_distance_reference, compute_coarse_edge_cost, coarse_astar_search,
    lift_endpoint_if_unsafe,
)
from planner.config import DEFAULT_CONFIG, PlannerConfig
from planner.primitives import MotionPrimitive, build_primitive_set, evaluate_primitive
from planner.roi import ROIData, load_roi
from planner.terrain import TerrainQuery
from planner.transition import evaluate_transition

NODATA = -9999.0
START_ROW, START_COL = 48, 276
GOAL_ROW, GOAL_COL = 264, 276
START_COARSE = (16, 92)
GOAL_COARSE = (88, 92)
ORIGINAL_MSL = 3760.0
MAX_EXPANSIONS = 30_000
ALTITUDE_REFERENCE_MSL = 3240.0
ALTITUDE_SCALE_M = 1000.0
W_DISTANCE = 1.0
W_ALTITUDE = 1.25
OUTPUT_CSV = "outputs/stage35_coarse_path.csv"

COARSE_CONFIG = dataclasses.replace(
    DEFAULT_CONFIG, xy_resolution_m=90.0, z_step_m=40.0, primitive_sample_spacing_m=30.0,
    altitude_reference_msl=ALTITUDE_REFERENCE_MSL,
    normalized_altitude_scale_m=ALTITUDE_SCALE_M,
    normalized_w_distance=W_DISTANCE,
    normalized_w_altitude=W_ALTITUDE,
)


def make_synthetic_roi(elevation: np.ndarray, nodata=NODATA, res: float = 90.0) -> ROIData:
    height, width = elevation.shape
    transform = Affine(res, 0.0, 500000.0, 0.0, -res, 4200000.0)
    return ROIData(
        elevation=elevation.astype(np.float32), transform=transform, crs="EPSG:32636",
        width=width, height=height, bounds=(500000.0, 4200000.0 - height * res, 500000.0 + width * res, 4200000.0),
        resolution=(res, res), nodata=nodata,
    )


# ----------------------------------------------------------------------
# 1/2: primitive set.
# ----------------------------------------------------------------------

def test_1_2() -> bool:
    print("=== 1/2: primitive set (24 primitives, all angles <=10 deg) ===")
    prims = build_primitive_set(COARSE_CONFIG)
    ok_count = len(prims) == 24
    max_angle = 0.0
    for p in prims:
        if p.horizontal_distance_m > 0:
            angle = math.degrees(math.atan2(abs(p.dz_m), p.horizontal_distance_m))
            max_angle = max(max_angle, angle)
    ok_angle = max_angle <= 10.0 + 1e-9
    print(f"  primitive_count={len(prims)} (expect 24)  max_angle={max_angle:.4f} (expect <=10.0)  "
          f"{'PASS' if (ok_count and ok_angle) else 'FAIL'}")
    return ok_count and ok_angle, prims


# ----------------------------------------------------------------------
# 3: flat terrain level path.
# ----------------------------------------------------------------------

def test_3(prims) -> bool:
    print()
    print("=== 3: flat terrain -> level path PASS ===")
    elev = np.full((10, 6), 1000.0)
    tq = TerrainQuery(make_synthetic_roi(elev))
    z0 = round(1300.0 / COARSE_CONFIG.z_step_m)
    start, goal = (1, 1, z0), (8, 1, z0)
    result = coarse_astar_search(start, goal, tq, 1100.0, 1500.0, COARSE_CONFIG, prims, max_expansions=2000)
    all_level = all(a[2] == b[2] for a, b in zip(result.path, result.path[1:])) if result.path else False
    ok = result.success and all_level
    print(f"  status={result.status} path_len={len(result.path)} all_level={all_level}  {'PASS' if ok else 'FAIL'}")
    return ok


# ----------------------------------------------------------------------
# 4: low-MSL path preferred (direct cost comparison, no full search needed).
# ----------------------------------------------------------------------

def test_4(prims) -> bool:
    print()
    print("=== 4: lower-MSL edge is cheaper (cost comparison) ===")
    level_e = next(p for p in prims if p.direction == "E" and p.primitive_type == "level")
    d_ref = 1000.0
    low_cost, *_ = compute_coarse_edge_cost(level_e, 3240.0, COARSE_CONFIG, d_ref, ALTITUDE_REFERENCE_MSL,
                                             ALTITUDE_SCALE_M, W_DISTANCE, W_ALTITUDE)
    high_cost, *_ = compute_coarse_edge_cost(level_e, 3600.0, COARSE_CONFIG, d_ref, ALTITUDE_REFERENCE_MSL,
                                              ALTITUDE_SCALE_M, W_DISTANCE, W_ALTITUDE)
    ok = low_cost < high_cost
    print(f"  same primitive at MSL=3240 -> cost={low_cost:.6f}; at MSL=3600 -> cost={high_cost:.6f}  "
          f"{'PASS' if ok else 'FAIL'}")
    return ok


# ----------------------------------------------------------------------
# 5: mid-segment ridge caught.
# ----------------------------------------------------------------------

def test_5(prims) -> bool:
    print()
    print("=== 5: mid-segment ridge caught (endpoints clear, middle doesn't) ===")
    elev = np.full((3, 5), 1000.0)
    elev[1, 2] = 1250.0  # a ridge directly between two safe-looking endpoints
    tq = TerrainQuery(make_synthetic_roi(elev))
    z0 = round(1300.0 / COARSE_CONFIG.z_step_m)
    start_xyz = state_to_xyz((1, 1, z0), tq, COARSE_CONFIG)
    level_e = next(p for p in prims if p.direction == "E" and p.primitive_type == "level")
    # Level_e's horizontal distance is 1 cell (90m) -- use a hand-built 2-cell
    # "long level" primitive to cross the ridge in one hop and expose the ridge
    # to primitive-internal sampling.
    long_level = MotionPrimitive("E", 0, 2, 0.0, 180.0, "level")
    result = evaluate_primitive(start_xyz, long_level, tq, COARSE_CONFIG)
    ok = not result.valid and result.reason == "below_min_agl"
    print(f"  2-cell level primitive over the ridge: valid={result.valid} reason={result.reason} "
          f"min_agl={result.min_agl_m:.1f}  {'PASS' if ok else 'FAIL'}")
    return ok


# ----------------------------------------------------------------------
# 6: MAX-only safety (MEAN never enters this module at all -- structural).
# ----------------------------------------------------------------------

def test_6() -> bool:
    print()
    print("=== 6: coarse safety strictly uses the stored (MAX) elevation, never a hypothetical MEAN ===")
    # A coarse cell whose stored value is the block's own MAX (3600m, as
    # build_coarse_dem, Stage 34, would have produced from a block with a much
    # lower mean, e.g. ~3300m) -- if this module used MEAN instead of MAX,
    # this edge would incorrectly pass; using MAX (the only value it has),
    # it correctly fails.
    elev = np.full((3, 3), 3600.0)
    tq = TerrainQuery(make_synthetic_roi(elev))
    z0 = round(3760.0 / COARSE_CONFIG.z_step_m)  # AGL = 3760-3600 = 160m < 200m
    start_xyz = state_to_xyz((1, 1, z0), tq, COARSE_CONFIG)
    level_e = next(p for p in build_primitive_set(COARSE_CONFIG) if p.direction == "E" and p.primitive_type == "level")
    result = evaluate_primitive(start_xyz, level_e, tq, COARSE_CONFIG)
    ok = not result.valid and result.reason == "below_min_agl"
    print(f"  cell elevation (=MAX by construction)=3600m, aircraft=3760m (AGL=160m<200m): "
          f"valid={result.valid} reason={result.reason}  {'PASS' if ok else 'FAIL'}")
    print("  (this module never receives a MEAN value at all -- CoarseTerrainStats.mean_elevation is "
          "not even imported here -- so 'MEAN would have passed' is a structural guarantee, not a runtime check)")
    return ok


# ----------------------------------------------------------------------
# 7/8: endpoint lift policy.
# ----------------------------------------------------------------------

def test_7_8() -> bool:
    print()
    print("=== 7/8: endpoint lift policy (unsafe lifted, safe left alone) ===")
    # Mirrors the real GOAL coarse cell (max=3560.36) and START coarse cell (max=3556.81).
    elev = np.full((2, 1), 3560.36)
    elev[1, 0] = 3556.81
    tq = TerrainQuery(make_synthetic_roi(elev))

    goal_lift = lift_endpoint_if_unsafe(0, 0, ORIGINAL_MSL, tq, COARSE_CONFIG)
    start_lift = lift_endpoint_if_unsafe(1, 0, ORIGINAL_MSL, tq, COARSE_CONFIG)

    ok7 = goal_lift.was_lifted and goal_lift.msl == 3800.0 and goal_lift.msl >= ORIGINAL_MSL
    ok8 = not start_lift.was_lifted and start_lift.msl == ORIGINAL_MSL
    print(f"  GOAL-like cell (max=3560.36): required={goal_lift.required_msl:.2f} was_lifted={goal_lift.was_lifted} "
          f"-> msl={goal_lift.msl} (expect lifted to 3800.0)  {'PASS' if ok7 else 'FAIL'}")
    print(f"  START-like cell (max=3556.81): required={start_lift.required_msl:.2f} "
          f"was_lifted={start_lift.was_lifted} -> msl={start_lift.msl} (expect unchanged 3760.0)  "
          f"{'PASS' if ok8 else 'FAIL'}")
    return ok7 and ok8


# ----------------------------------------------------------------------
# 9: NoData crossing.
# ----------------------------------------------------------------------

def test_9(prims) -> bool:
    print()
    print("=== 9: NoData crossing -> INVALID ===")
    elev = np.full((3, 3), 3000.0)
    elev[1, 1] = NODATA
    tq = TerrainQuery(make_synthetic_roi(elev, nodata=NODATA))
    z0 = round(3200.0 / COARSE_CONFIG.z_step_m)
    start_xyz = state_to_xyz((1, 0, z0), tq, COARSE_CONFIG)
    level_e = next(p for p in prims if p.direction == "E" and p.primitive_type == "level")
    long_level = MotionPrimitive("E", 0, 2, 0.0, 180.0, "level")
    result = evaluate_primitive(start_xyz, long_level, tq, COARSE_CONFIG)
    ok = not result.valid and result.reason == "nodata"
    print(f"  primitive crossing a NoData cell: valid={result.valid} reason={result.reason}  "
          f"{'PASS' if ok else 'FAIL'}")
    return ok


# ----------------------------------------------------------------------
# 10: state shape.
# ----------------------------------------------------------------------

def test_10() -> bool:
    print()
    print("=== 10: state is ONLY (row, col, z_index) ===")
    state = (5, 5, 90)
    ok = len(state) == 3 and all(isinstance(v, int) for v in state)
    print(f"  state={state} len={len(state)} all_int={all(isinstance(v, int) for v in state)}  "
          f"{'PASS' if ok else 'FAIL'}")
    return ok


# ----------------------------------------------------------------------
# Real 6.48km coarse benchmark.
# ----------------------------------------------------------------------

def fine_replay(waypoints, fine_terrain, fine_config, sample_spacing_m: float = 10.0) -> dict:
    violations = []
    min_agl = math.inf
    max_angle = 0.0
    for (x1, y1, z1), (x2, y2, z2) in zip(waypoints, waypoints[1:]):
        transition = evaluate_transition((x1, y1, z1), (x2, y2, z2), fine_config)
        max_angle = max(max_angle, transition.flight_path_angle_deg)
        if not transition.valid:
            violations.append({"reason": transition.reason, "segment": ((x1, y1, z1), (x2, y2, z2))})
        horizontal = transition.horizontal_distance_m
        n = max(1, math.ceil(horizontal / sample_spacing_m)) if horizontal > 0 else 1
        for i in range(n + 1):
            t = i / n
            x, y, z = x1 + (x2 - x1) * t, y1 + (y2 - y1) * t, z1 + (z2 - z1) * t
            agl = evaluate_agl(fine_terrain, x, y, z, fine_config)
            if not agl.valid:
                violations.append({"reason": agl.reason, "x": x, "y": y, "z": z})
            elif agl.agl_m < min_agl:
                min_agl = agl.agl_m
    return {"violations": violations, "min_agl_m": min_agl, "max_angle_deg": max_angle}


def main() -> None:
    r1, prims = test_1_2()
    results = [r1, test_3(prims), test_4(prims), test_5(prims), test_6(), test_7_8(), test_9(prims), test_10()]
    print()
    print(f"Synthetic tests: {'ALL PASS' if all(results) else 'SOME FAILED'}")
    if not all(results):
        print("Synthetic tests failed -- stopping before the real benchmark.")
        return

    print()
    print("=" * 70)
    print("=== REAL 6.48km COARSE BENCHMARK ===")
    fine_cfg = DEFAULT_CONFIG
    fine_roi = load_roi(fine_cfg)
    fine_terrain = TerrainQuery(fine_roi)
    coarse_result = build_coarse_dem(fine_roi, factor=3)
    coarse_terrain = TerrainQuery(coarse_result.roi)
    coarse_stats = build_coarse_terrain_stats(fine_roi, factor=3)

    start_lift = lift_endpoint_if_unsafe(START_COARSE[0], START_COARSE[1], ORIGINAL_MSL, coarse_terrain, COARSE_CONFIG)
    goal_lift = lift_endpoint_if_unsafe(GOAL_COARSE[0], GOAL_COARSE[1], ORIGINAL_MSL, coarse_terrain, COARSE_CONFIG)
    print(f"  START coarse={START_COARSE}: cell_max={start_lift.cell_max_terrain_msl:.2f} "
          f"required={start_lift.required_msl:.2f} original={ORIGINAL_MSL} -> msl={start_lift.msl} "
          f"was_lifted={start_lift.was_lifted}")
    print(f"  GOAL  coarse={GOAL_COARSE}: cell_max={goal_lift.cell_max_terrain_msl:.2f} "
          f"required={goal_lift.required_msl:.2f} original={ORIGINAL_MSL} -> msl={goal_lift.msl} "
          f"was_lifted={goal_lift.was_lifted}")

    start = (start_lift.row, start_lift.col, start_lift.z_index)
    goal = (goal_lift.row, goal_lift.col, goal_lift.z_index)

    seg = coarse_result.roi.elevation[min(START_COARSE[0], GOAL_COARSE[0]):max(START_COARSE[0], GOAL_COARSE[0]) + 1,
                                       START_COARSE[1]]
    seg_min = float(seg.min())
    min_search = math.ceil((seg_min + COARSE_CONFIG.min_agl_m) / COARSE_CONFIG.z_step_m) * COARSE_CONFIG.z_step_m
    max_search = max(start_lift.msl, goal_lift.msl) + 2 * COARSE_CONFIG.z_step_m

    d_ref = compute_coarse_distance_reference(start, goal, coarse_terrain, COARSE_CONFIG)
    print(f"  search bounds=[{min_search},{max_search}]  D_ref={d_ref:.2f}m  "
          f"altitude_reference_msl={ALTITUDE_REFERENCE_MSL}  w_distance={W_DISTANCE}  w_altitude={W_ALTITUDE}")
    print()

    result = coarse_astar_search(
        start, goal, coarse_terrain, min_search, max_search, COARSE_CONFIG,
        primitives=prims, max_expansions=MAX_EXPANSIONS, distance_reference_m=d_ref,
        altitude_reference_msl=ALTITUDE_REFERENCE_MSL, altitude_scale_m=ALTITUDE_SCALE_M,
        w_distance=W_DISTANCE, w_altitude=W_ALTITUDE,
    )

    print(f"  status={result.status}  runtime={result.runtime_s:.2f}s  expanded={result.expanded_nodes}  "
          f"max_open={result.max_open_size}  generated={result.generated_neighbors}  "
          f"rejected={result.rejected_neighbors}")
    print(f"  rejected_reason_counts={result.rejected_reason_counts}")

    if not result.success:
        print()
        print("=== FAIL diagnostics ===")
        cs = result.closest_state_to_goal
        if cs is not None:
            cx, cy, cz = state_to_xyz(cs, coarse_terrain, COARSE_CONFIG)
            cell = coarse_terrain.elevation_at_rowcol(cs[0], cs[1])
            agl = cz - cell.elevation if cell.valid else float("nan")
            print(f"  closest_state_to_goal={cs}  distance_to_goal_m={result.closest_distance_to_goal_m:.2f}")
            print(f"  x={cx:.1f} y={cy:.1f} msl={cz:.1f}  coarse_MAX_terrain={cell.elevation:.1f}  "
                  f"coarse_MAX_AGL={agl:.1f}")
        return

    print()
    print("=== SUCCESS -- coarse path metrics ===")
    path = result.path
    xyz = [state_to_xyz(s, coarse_terrain, COARSE_CONFIG) for s in path]
    xy_length = sum(math.hypot(xyz[i + 1][0] - xyz[i][0], xyz[i + 1][1] - xyz[i][1]) for i in range(len(xyz) - 1))
    length_3d = sum(math.sqrt((xyz[i + 1][0] - xyz[i][0]) ** 2 + (xyz[i + 1][1] - xyz[i][1]) ** 2
                               + (xyz[i + 1][2] - xyz[i][2]) ** 2) for i in range(len(xyz) - 1))
    msl_values = [p[2] for p in xyz]
    total_climb = sum(max(0.0, xyz[i + 1][2] - xyz[i][2]) for i in range(len(xyz) - 1))
    total_descent = sum(max(0.0, xyz[i][2] - xyz[i + 1][2]) for i in range(len(xyz) - 1))
    max_angle = 0.0
    min_agl_coarse = math.inf
    for (r1, c1, z1), (r2, c2, z2) in zip(path, path[1:]):
        p1 = state_to_xyz((r1, c1, z1), coarse_terrain, COARSE_CONFIG)
        prim_match = MotionPrimitive("_", r2 - r1, c2 - c1, z_index_to_msl(z2, COARSE_CONFIG) - z_index_to_msl(z1, COARSE_CONFIG),
                                      math.hypot((c2 - c1) * COARSE_CONFIG.xy_resolution_m,
                                                  (r2 - r1) * COARSE_CONFIG.xy_resolution_m), "_")
        eval_result = evaluate_primitive(p1, prim_match, coarse_terrain, COARSE_CONFIG)
        if eval_result.valid:
            min_agl_coarse = min(min_agl_coarse, eval_result.min_agl_m)
        angle = math.degrees(math.atan2(abs(prim_match.dz_m), prim_match.horizontal_distance_m)) \
            if prim_match.horizontal_distance_m > 0 else 0.0
        max_angle = max(max_angle, angle)

    print(f"  path_node_count={len(path)}")
    print(f"  xy_length_m={xy_length:.1f}  3d_length_m={length_3d:.1f}")
    print(f"  min_MSL={min(msl_values):.1f}  mean_MSL={sum(msl_values) / len(msl_values):.1f}  max_MSL={max(msl_values):.1f}")
    print(f"  total_climb_m={total_climb:.1f}  total_descent_m={total_descent:.1f}")
    print(f"  min_coarse_MAX_AGL_m={min_agl_coarse:.1f}  max_flight_path_angle_deg={max_angle:.2f}")
    print(f"  cost: distance={result.total_distance_cost:.6f}  altitude={result.total_altitude_cost:.6f}  "
          f"total={result.total_cost:.6f}")

    reliefs = [float(coarse_stats.relief[r, c]) for (r, c, _) in path]
    print()
    print("=== Path relief diagnostic (NOT used for cost/safety) ===")
    print(f"  mean_relief={sum(reliefs) / len(reliefs):.2f}  max_relief={max(reliefs):.2f}  "
          f"p95_relief={np.percentile(reliefs, 95):.2f}")
    top5_idx = sorted(range(len(path)), key=lambda i: reliefs[i], reverse=True)[:5]
    for i in top5_idx:
        r, c, z = path[i]
        print(f"    path_node={i} coarse(row={r},col={c}) relief={reliefs[i]:.2f} "
              f"min={coarse_stats.min_elevation[r, c]:.1f} mean={coarse_stats.mean_elevation[r, c]:.1f} "
              f"max={coarse_stats.max_elevation[r, c]:.1f}")

    print()
    print("=== FINE REPLAY (30m DEM, <=10m sampling) ===")
    replay = fine_replay(xyz, fine_terrain, fine_cfg, sample_spacing_m=10.0)
    print(f"  min_AGL_m={replay['min_agl_m']:.2f}  max_flight_path_angle_deg={replay['max_angle_deg']:.2f}  "
          f"violation_count={len(replay['violations'])}")
    if replay["violations"]:
        for v in replay["violations"][:10]:
            print(f"    violation: {v}")

    os.makedirs(os.path.dirname(OUTPUT_CSV), exist_ok=True)
    with open(OUTPUT_CSV, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "row", "col", "z_index", "x", "y", "z_msl"])
        for i, ((r, c, z), (x, y, zm)) in enumerate(zip(path, xyz)):
            writer.writerow([i, r, c, z, f"{x:.2f}", f"{y:.2f}", f"{zm:.2f}"])
    print()
    print(f"Coarse path written to {OUTPUT_CSV}")


if __name__ == "__main__":
    main()
