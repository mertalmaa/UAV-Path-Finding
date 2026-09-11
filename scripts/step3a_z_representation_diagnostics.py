"""Roadmap Step 3A: sparse/adaptive Z representation -- controlled
diagnostics (section 6) + state-count estimate (section 10).

No production graph, no final Z spacing chosen, no aircraft safety bubble,
no heading/turn-radius. planner/*.py untouched. Reuses evaluate_agl,
evaluate_transition, evaluate_primitive, MotionPrimitive, CoarseTerrainStats
verbatim -- nothing under planner/ reimplemented.
"""
import dataclasses
import math

import numpy as np
from affine import Affine

from planner.coarse import build_coarse_terrain_stats
from planner.config import DEFAULT_CONFIG
from planner.primitives import MotionPrimitive, evaluate_primitive
from planner.roi import ROIData, load_roi
from planner.terrain import TerrainQuery

STEP2B_CONFIG = dataclasses.replace(DEFAULT_CONFIG, min_agl_m=100.0)
CEILING = 6000.0
DENSE_STEP = 20.0          # existing production z_step_m -- reference/baseline only
SPARSE_TEST_STEP = 100.0   # illustrative diagnostic instrument only, NOT a proposal


def make_synthetic_roi(elevation, res=30.0, nodata=-9999.0):
    height, width = elevation.shape
    transform = Affine(res, 0.0, 500000.0, 0.0, -res, 4200000.0)
    return ROIData(elevation=elevation.astype(np.float32), transform=transform, crs="EPSG:32636",
                    width=width, height=height,
                    bounds=(500000.0, 4200000.0 - height * res, 500000.0 + width * res, 4200000.0),
                    resolution=(res, res), nodata=nodata)


def lowest_feasible_layer(terrain_elev, min_agl, z_step, z_lo, z_hi):
    true_floor = terrain_elev + min_agl
    candidate = math.ceil(max(true_floor, z_lo) / z_step) * z_step
    return None if candidate > z_hi else candidate


def candidate_count(z_lo, z_hi, z_step):
    if z_hi < z_lo:
        return 0
    lo = math.ceil(z_lo / z_step) * z_step
    return max(0, int((z_hi - lo) // z_step) + 1)


# ===========================================================================
# Section 6: controlled diagnostics, cases A-E
# ===========================================================================

def case_A_wide():
    terrain_elev = 3000.0
    z_lo, z_hi = terrain_elev + STEP2B_CONFIG.min_agl_m, CEILING
    dense_n = candidate_count(z_lo, z_hi, DENSE_STEP)
    sparse_n = candidate_count(z_lo, z_hi, SPARSE_TEST_STEP)
    fine_z = lowest_feasible_layer(terrain_elev, STEP2B_CONFIG.min_agl_m, DENSE_STEP, z_lo, z_hi)
    sparse_z = lowest_feasible_layer(terrain_elev, STEP2B_CONFIG.min_agl_m, SPARSE_TEST_STEP, z_lo, z_hi)
    return {"case": "A_wide_feasible_interval", "terrain_elev": terrain_elev,
            "dense_candidate_count": dense_n, "sparse_candidate_count": sparse_n,
            "P3_holds": sparse_z is not None, "delta_z_loss": (sparse_z - fine_z) if sparse_z and fine_z else None}


def case_B_narrow():
    terrain_elev = 3540.0
    z_lo, z_hi = terrain_elev + STEP2B_CONFIG.min_agl_m, 3660.0  # narrow ledge band, same as Step 2D case 3
    dense_n = candidate_count(z_lo, z_hi, DENSE_STEP)
    sparse_n = candidate_count(z_lo, z_hi, SPARSE_TEST_STEP)
    fine_z = lowest_feasible_layer(terrain_elev, STEP2B_CONFIG.min_agl_m, DENSE_STEP, z_lo, z_hi)
    sparse_z = lowest_feasible_layer(terrain_elev, STEP2B_CONFIG.min_agl_m, SPARSE_TEST_STEP, z_lo, z_hi)
    return {"case": "B_narrow_feasible_interval", "terrain_elev": terrain_elev, "band": (z_lo, z_hi),
            "dense_candidate_count": dense_n, "sparse_candidate_count": sparse_n,
            "P3_holds_dense": fine_z is not None, "P3_holds_sparse": sparse_z is not None,
            "note": "sparse loses the ONLY usable state entirely (P3 fails), dense keeps exactly 1"}


def case_C_P3_pass_delta_meaningful():
    terrain_elev = 3512.0  # Step 2D's own Case 2
    z_lo, z_hi = terrain_elev, terrain_elev + 300.0
    fine_z = lowest_feasible_layer(terrain_elev, STEP2B_CONFIG.min_agl_m, DENSE_STEP, z_lo, z_hi)
    sparse_z = lowest_feasible_layer(terrain_elev, STEP2B_CONFIG.min_agl_m, SPARSE_TEST_STEP, z_lo, z_hi)
    return {"case": "C_P3_pass_but_delta_z_loss", "terrain_elev": terrain_elev,
            "P3_holds": sparse_z is not None, "dense_z": fine_z, "sparse_z": sparse_z,
            "delta_z_loss": (sparse_z - fine_z) if (sparse_z and fine_z) else None}


def case_D_P3_fail():
    terrain_elev = 3540.0
    z_lo, z_hi = terrain_elev + STEP2B_CONFIG.min_agl_m, 3660.0
    fine_z = lowest_feasible_layer(terrain_elev, STEP2B_CONFIG.min_agl_m, DENSE_STEP, z_lo, z_hi)
    sparse_z = lowest_feasible_layer(terrain_elev, STEP2B_CONFIG.min_agl_m, SPARSE_TEST_STEP, z_lo, z_hi)
    return {"case": "D_P3_fail", "terrain_elev": terrain_elev, "band": (z_lo, z_hi),
            "P3_holds_dense": fine_z is not None, "P3_holds_sparse": sparse_z is not None,
            "delta_z_loss": "undefined (sparse has no representable state at all)"}


def case_E_motion_edge_lost():
    """Endpoints individually valid at BOTH resolutions -- but the DIRECT
    motion edge between them clips a localized hump that only an
    intermediate DENSE-Z landing state (unavailable under sparse Z) can
    route around. Verified against real evaluate_primitive/evaluate_transition,
    not asserted by hand."""
    elev = np.full((1, 21), 3500.0)
    elev[0, 9:12] = 3551.0  # hump near the midpoint (cols 9-11 -> 270-330m)
    roi = make_synthetic_roi(elev)
    tq = TerrainQuery(roi)
    cfg = STEP2B_CONFIG

    direct = MotionPrimitive("E", drow=0, dcol=20, dz_m=60.0, horizontal_distance_m=600.0, primitive_type="climb")
    hop1 = MotionPrimitive("E", drow=0, dcol=10, dz_m=40.0, horizontal_distance_m=300.0, primitive_type="climb")
    hop2 = MotionPrimitive("E", drow=0, dcol=10, dz_m=20.0, horizontal_distance_m=300.0, primitive_type="climb")

    x0, y0 = tq.rowcol_to_xy(0, 0)
    x_mid, y_mid = tq.rowcol_to_xy(0, 10)

    r_direct = evaluate_primitive((x0, y0, 3620.0), direct, tq, cfg)
    r_hop1 = evaluate_primitive((x0, y0, 3620.0), hop1, tq, cfg)
    r_hop2 = evaluate_primitive((x_mid, y_mid, 3660.0), hop2, tq, cfg)

    return {
        "case": "E_endpoints_valid_motion_edge_lost",
        "start_z": 3620.0, "target_z": 3680.0, "intermediate_dense_z": 3660.0,
        "direct_single_edge_valid": r_direct.valid, "direct_min_agl": r_direct.min_agl_m,
        "hop1_valid": r_hop1.valid, "hop1_min_agl": r_hop1.min_agl_m,
        "hop2_valid": r_hop2.valid, "hop2_min_agl": r_hop2.min_agl_m,
        "note": "direct sparse-equivalent single climb clips the hump (min_agl<100); "
                "the SAME total climb via a dense-Z (20m-grid) intermediate landing state at 3660m succeeds both hops",
    }


# ===========================================================================
# Section 10: state-count estimate (60m XY grid, dense vs sparse/event-aware Z)
# ===========================================================================

def section_10():
    fine_roi = load_roi(DEFAULT_CONFIG)
    stats60 = build_coarse_terrain_stats(fine_roi, factor=2)
    max_e = stats60.max_elevation
    nodata_mask = (max_e == stats60.nodata) if stats60.nodata is not None else np.zeros_like(max_e, dtype=bool)
    valid = ~nodata_mask
    total_cells = int(valid.sum())

    dense_counts = []
    for e in max_e[valid].flatten():
        floor = math.ceil((float(e) + STEP2B_CONFIG.min_agl_m) / DENSE_STEP) * DENSE_STEP
        dense_counts.append(max(0, int((CEILING - floor) // DENSE_STEP) + 1))
    dense_counts = np.array(dense_counts)

    # illustrative event-aware candidate count per cell: floor itself, + a
    # small fixed number of OTHER mission events that would apply map-wide
    # (start altitude, goal altitude, mission ceiling, one or two validity-
    # transition levels) -- NOT a chosen production number, purely for the
    # order-of-magnitude comparison requested.
    illustrative_events_per_cell = 6

    return {
        "xy_cells": total_cells,
        "dense_total_possible_states": int(dense_counts.sum()),
        "dense_mean_per_cell": float(dense_counts.mean()),
        "dense_max_per_cell": int(dense_counts.max()),
        "sparse_event_aware_per_cell_illustrative": illustrative_events_per_cell,
        "sparse_event_aware_total_illustrative": total_cells * illustrative_events_per_cell,
        "reduction_factor_illustrative": float(dense_counts.sum()) / (total_cells * illustrative_events_per_cell),
    }


def main():
    print("=" * 70)
    print("STEP 3A -- sparse/adaptive Z representation: controlled diagnostics")
    print("=" * 70)

    print("\n--- SECTION 6: controlled diagnostics (dense 20m vs illustrative sparse 100m) ---")
    for fn in (case_A_wide, case_B_narrow, case_C_P3_pass_delta_meaningful, case_D_P3_fail, case_E_motion_edge_lost):
        print(f"  {fn()}")

    print("\n--- SECTION 10: state-count estimate (60m XY grid) ---")
    print(f"  {section_10()}")

    print("\n" + "=" * 70)
    print("Done. No final Z spacing chosen, no production graph.")


if __name__ == "__main__":
    main()
