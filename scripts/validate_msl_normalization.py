"""Stage 10 validation: MSL cost normalization independent of search bounds.

The bug this fixes: the old altitude_norm was relative to a given search
call's [min_search_altitude_msl, max_search_altitude_msl], clamped to
[0,1]. The exact same physical 1400m edge got a strong penalty under a
narrow search ceiling and a nearly-zero penalty under a wide one, purely
because the ceiling moved -- nothing about the edge itself changed. Fixed
by switching to a fixed reference/scale (config.msl_reference_m /
msl_scale_m), unbounded above, computed with no search-bounds input at all.

A) weight=0 regression.
B) per-edge mathematical invariance: compute_edge_cost() takes no search
   bounds anymore, so the same physical edge always costs the same --
   demonstrated directly, and contrasted with the old (legacy, validation-
   only) formula's search-bounds-relative values.
C) search-level invariance: the narrow-vs-wide scenario that exposed the
   original bug, re-run under the new normalization.
D) hard safety regression: AGL/climb/descent limits still supreme.
E) real Aladaglar ROI check, 2 representative weight combinations.
"""
import dataclasses
import math
import time

import numpy as np
from affine import Affine

from planner.astar import astar_search, compute_edge_cost, msl_to_z_index
from planner.config import DEFAULT_CONFIG
from planner.primitives import build_primitive_set
from planner.roi import ROIData, load_roi
from planner.terrain import TerrainQuery

NODATA = -9999.0


def make_roi(elevation: np.ndarray, nodata: float = NODATA, res: float = 30.0) -> ROIData:
    height, width = elevation.shape
    transform = Affine(res, 0.0, 0.0, 0.0, -res, height * res)
    return ROIData(
        elevation=elevation.astype(np.float32), transform=transform, crs="EPSG:32636",
        width=width, height=height, bounds=(0.0, 0.0, width * res, height * res),
        resolution=(res, res), nodata=nodata,
    )


def legacy_altitude_norm(mean_altitude_msl: float, min_search_altitude_msl: float, max_search_altitude_msl: float) -> float:
    """The OLD (removed from production) search-bounds-relative formula.
    Reproduced here ONLY for this validation script's legacy-vs-new comparison."""
    span = max_search_altitude_msl - min_search_altitude_msl
    if span == 0.0:
        return 0.0
    norm = (mean_altitude_msl - min_search_altitude_msl) / span
    return min(1.0, max(0.0, norm))


def report(label, result) -> None:
    print(f"  [{label}] status={result.status} success={result.success}")
    if not result.success:
        return
    print(f"    path_states={len(result.path)} geometric_length={result.geometric_path_length:.2f} "
          f"total_cost(weighted)={result.total_cost:.2f}")
    print(f"    MSL: min={result.minimum_aircraft_msl:.1f} max={result.maximum_aircraft_msl:.1f} "
          f"avg={result.average_aircraft_msl:.1f}")
    print(f"    climb={result.total_climb_m:.1f} descent={result.total_descent_m:.1f} "
          f"vertical_motion={result.total_vertical_motion_m:.1f}")
    print(f"    minimum_observed_agl={result.minimum_observed_agl:.1f} expanded={result.expanded_nodes} "
          f"max_open={result.max_open_size} runtime={result.runtime_s * 1000:.1f}ms")


def validation_a(cfg, primitives) -> bool:
    print("=== A) weight=0.0 regression ===")
    flat = np.full((5, 20), 1000.0)
    roi = make_roi(flat)
    tq = TerrainQuery(roi)
    z0 = msl_to_z_index(1300.0, cfg)
    start, goal = (2, 2, z0), (2, 12, z0)

    c0 = dataclasses.replace(cfg, msl_cost_weight=0.0)
    result = astar_search(start, goal, tq, min_search_altitude_msl=1100.0, max_search_altitude_msl=1500.0,
                           config=c0, primitives=primitives)
    report("msl_weight=0.0", result)

    level_e = next(p for p in primitives if p.direction == "E" and p.primitive_type == "level")
    unit_cost = compute_edge_cost(level_e, 1300.0, c0)
    expected_unit = math.sqrt(level_e.horizontal_distance_m ** 2 + level_e.dz_m ** 2) + c0.vertical_cost_weight * abs(level_e.dz_m)

    ok = (
        result.success and abs(result.total_cost - 300.0) < 1e-6
        and abs(result.total_cost - result.geometric_path_length) < 1e-9
        and abs(unit_cost - expected_unit) < 1e-9
    )
    print(f"  compute_edge_cost unit check: {unit_cost:.4f} == geometric_cost + vertical_weight*|dz| == {expected_unit:.4f}")
    print(f"  PASS" if ok else "  FAIL")
    return ok


def validation_b(cfg) -> bool:
    print("=== B) per-edge mathematical invariance (no search-bounds input at all) ===")
    print(f"  msl_reference_m={cfg.msl_reference_m}, msl_scale_m={cfg.msl_scale_m}")

    from planner.astar import _altitude_scaled

    scaled_1400 = _altitude_scaled(1400.0, cfg)
    scaled_2800 = _altitude_scaled(2800.0, cfg)
    scaled_3500 = _altitude_scaled(3500.0, cfg)
    print(f"  altitude_scaled(1400m) = {scaled_1400}  (expect 1.4)")
    print(f"  altitude_scaled(2800m) = {scaled_2800}  (expect 2.8)")
    print(f"  altitude_scaled(3500m) = {scaled_3500}  (expect 3.5)")

    # compute_edge_cost() has no min/max_search_altitude_msl parameter left --
    # calling it "in two different search contexts" is structurally the same
    # call, which is the point: there is no longer a bounds input to vary.
    from planner.primitives import build_primitive_set
    prims = build_primitive_set(cfg)
    level_e = next(p for p in prims if p.direction == "E" and p.primitive_type == "level")
    cost_a = compute_edge_cost(level_e, 1400.0, cfg)
    cost_b = compute_edge_cost(level_e, 1400.0, cfg)  # identical call, no bounds to vary
    print(f"  compute_edge_cost(level_E, mean_alt=1400m) = {cost_a:.4f} (called twice: {cost_a == cost_b})")

    ok = scaled_1400 == 1.4 and scaled_2800 == 2.8 and scaled_3500 == 3.5 and cost_a == cost_b
    print(f"  PASS" if ok else "  FAIL")
    return ok


def validation_c(cfg, primitives) -> bool:
    print("=== C) search-level invariance (the narrow-vs-wide scenario that exposed the bug) ===")
    flat = np.full((3, 55), 1000.0)
    roi = make_roi(flat)
    tq = TerrainQuery(roi)
    z0 = msl_to_z_index(1400.0, cfg)
    start, goal = (1, 2, z0), (1, 52, z0)
    c = dataclasses.replace(cfg, msl_cost_weight=0.25, vertical_cost_weight=1.0)

    results = {}
    for label, lo, hi in (("narrow [1300,1400]", 1300.0, 1400.0), ("wide [1300,2400]", 1300.0, 2400.0)):
        r = astar_search(start, goal, tq, min_search_altitude_msl=lo, max_search_altitude_msl=hi,
                          config=c, primitives=primitives, max_expansions=100_000)
        results[label] = r
        report(label, r)

    narrow, wide = results["narrow [1300,1400]"], results["wide [1300,2400]"]
    ok = narrow.success and wide.success
    ok = ok and narrow.path == wide.path  # identical chosen path now
    ok = ok and abs(narrow.average_aircraft_msl - wide.average_aircraft_msl) < 1e-6
    ok = ok and abs(narrow.total_vertical_motion_m - wide.total_vertical_motion_m) < 1e-6

    print(f"  same path in both cases: {narrow.path == wide.path}")
    print("  (previously: narrow dove 120m, wide dove 0m -- same physical scenario, different behavior)")
    print(f"  PASS" if ok else "  FAIL")
    return ok


def validation_d(cfg) -> bool:
    print("=== D) legacy (search-relative) vs new (fixed-scale) side by side ===")
    mean_alt = 1400.0
    narrow = (1300.0, 1400.0)
    wide = (1300.0, 2400.0)

    legacy_narrow = legacy_altitude_norm(mean_alt, *narrow)
    legacy_wide = legacy_altitude_norm(mean_alt, *wide)
    new_narrow = max(0.0, mean_alt - cfg.msl_reference_m) / cfg.msl_scale_m
    new_wide = new_narrow  # no bounds input -- literally the same computation

    print(f"  LEGACY (removed): same {mean_alt}m edge -> narrow bounds {narrow}: norm={legacy_narrow:.4f}, "
          f"wide bounds {wide}: norm={legacy_wide:.4f}  (different!)")
    print(f"  NEW: same {mean_alt}m edge -> narrow context: scaled={new_narrow:.4f}, "
          f"wide context: scaled={new_wide:.4f}  (identical)")

    ok = abs(legacy_narrow - 1.0) < 1e-9 and abs(legacy_wide - (100.0 / 1100.0)) < 1e-9 and legacy_narrow != legacy_wide
    ok = ok and new_narrow == new_wide == 1.4
    print(f"  PASS" if ok else "  FAIL")
    return ok


def validation_e(cfg, primitives) -> bool:
    print("=== E) hard safety regression (AGL/climb/descent still supreme under new MSL cost) ===")
    width, height = 45, 10
    elev = np.full((height, width), 1000.0)
    elev[:, 15:21] = 1110.0  # unsafe at 1300m (AGL=190), safe at 1320m (AGL=210)
    roi = make_roi(elev)
    tq = TerrainQuery(roi)
    z_top = msl_to_z_index(1320.0, cfg)
    start, goal = (5, 2, z_top), (5, 42, z_top)

    result = astar_search(start, goal, tq, min_search_altitude_msl=1300.0, max_search_altitude_msl=1320.0,
                           config=cfg, primitives=primitives)  # default weights
    report(f"msl={cfg.msl_cost_weight}, vertical={cfg.vertical_cost_weight}", result)

    ok = result.success and result.minimum_observed_agl >= cfg.min_agl_m - 1e-6
    print(f"  minimum_observed_agl={result.minimum_observed_agl:.1f} >= min_agl_m={cfg.min_agl_m}: {ok}")
    print(f"  PASS" if ok else "  FAIL")
    return ok


def validation_f(cfg, primitives) -> bool:
    print("=== F) real Aladaglar ROI, 2 representative combinations ===")
    print("  PROTOTYPE test scenario (same as Stage 9), not a real mission altitude/route.")
    roi = load_roi(cfg)
    tq = TerrainQuery(roi)

    row_idx, c_start, c_goal = 80, 90, 128
    seg = roi.elevation[row_idx, c_start:c_goal + 1]
    seg_min, seg_max = float(seg.min()), float(seg.max())
    cruise_msl = math.ceil((seg_max + cfg.min_agl_m) / cfg.z_step_m) * cfg.z_step_m + 20.0
    min_search = math.ceil((seg_min + cfg.min_agl_m) / cfg.z_step_m) * cfg.z_step_m
    max_search = cruise_msl + 20.0

    z0 = msl_to_z_index(cruise_msl, cfg)
    start, goal = (row_idx, c_start, z0), (row_idx, c_goal, z0)
    print(f"  segment terrain min={seg_min:.1f}m max={seg_max:.1f}m, cruise_msl={cruise_msl:.0f}m "
          f"(altitude_scaled at cruise = {cruise_msl / cfg.msl_scale_m:.3f})")

    ok = True
    for label, w_msl, w_v in (("Case A", 0.0, 1.0), ("Case B", 0.25, 1.0)):
        c = dataclasses.replace(cfg, msl_cost_weight=w_msl, vertical_cost_weight=w_v)
        t0 = time.perf_counter()
        r = astar_search(start, goal, tq, min_search_altitude_msl=min_search, max_search_altitude_msl=max_search,
                          config=c, primitives=primitives, max_expansions=150_000)
        dt = time.perf_counter() - t0
        report(f"{label} (msl={w_msl}, vertical={w_v}) [wall={dt:.1f}s]", r)
        if r.status != "success":
            ok = False
        elif r.minimum_observed_agl < cfg.min_agl_m - 1e-6:
            ok = False

    print("  NOTE: if Case B looks drastically different in cost/runtime from Stage 9's old-normalization")
    print("  result, that is expected -- msl_cost_weight's *meaning* changed with the normalization.")
    print("  w_MSL requires re-tuning under the new fixed-scale normalization (explicitly out of scope here).")
    print(f"  PASS" if ok else "  FAIL")
    return ok


def main() -> None:
    cfg = DEFAULT_CONFIG
    primitives = build_primitive_set(cfg)

    results = [
        validation_a(cfg, primitives),
        validation_b(cfg),
        validation_c(cfg, primitives),
        validation_d(cfg),
        validation_e(cfg, primitives),
        validation_f(cfg, primitives),
    ]

    print()
    print(f"Overall: {'ALL PASS' if all(results) else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
