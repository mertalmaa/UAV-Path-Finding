"""Step REP-1.2B.1 validation: multi-cell vertical-motion horizon bridge.

Synthetic test matrix (A-K) only -- no Mission C, see
scripts/rep12b1_mission_ab.py for the real-terrain Mission A/B run.
"""
import dataclasses
import math

import numpy as np
from affine import Affine

from planner.aircraft_profile import load_aircraft_profile
from planner.astar import (
    _generate_candidate_z_neighbors,
    decode_candidate_altitude,
    encode_candidate_altitude,
)
from planner.candidate_z import CandidateZGenerator, MissionContext, TerrainMetadataStore
from planner.config import DEFAULT_CONFIG
from planner.primitives import primitive_for_target_altitude_over_horizon
from planner.roi import ROIData
from planner.terrain import TerrainQuery
from planner.vertical_motion import derive_minimum_horizontal_distance_m, evaluate_vertical_motion

RES = 60.0  # matches MISSION_CONFIG's real xy_resolution_m -- config below must match it
NODATA = -9999.0
AIRCRAFT_PROFILE_PATH = "jsbsim/results/c172p_aircraft_profile_planner_safe_v3.json"


def make_terrain(elevation: np.ndarray) -> TerrainQuery:
    height, width = elevation.shape
    transform = Affine(RES, 0.0, 0.0, 0.0, -RES, height * RES)
    roi = ROIData(
        elevation=elevation.astype(np.float32), transform=transform, crs="EPSG:32636",
        width=width, height=height, bounds=(0.0, 0.0, width * RES, height * RES),
        resolution=(RES, RES), nodata=NODATA,
    )
    return TerrainQuery(roi)


def main() -> None:
    all_pass = True

    def check(label: str, ok: bool) -> None:
        nonlocal all_pass
        all_pass = all_pass and ok
        print(f"  {label:<78} {'PASS' if ok else 'FAIL'}")

    cfg = dataclasses.replace(DEFAULT_CONFIG, xy_resolution_m=RES)
    profile = load_aircraft_profile(AIRCRAFT_PROFILE_PATH)
    min_agl = cfg.min_agl_m if cfg.min_agl_m is not None else 200.0

    print("=== A: 60m + large delta_z -> single-step rejected (unchanged REP-1.2B behavior) ===")
    flat_elev = np.full((80, 80), 1000.0, dtype=np.float32)
    terrain = make_terrain(flat_elev)
    store = TerrainMetadataStore(terrain)
    start_z = math.ceil((1000.0 + min_agl) / cfg.z_step_m) * cfg.z_step_m  # on the safe floor
    # goal_z == ceiling_msl: a large, deliberately single-step-infeasible climb, sourced from
    # the START cell's own generate() (CLASS A ceiling event) -- not an invented arbitrary value.
    goal_z = start_z + 100.0
    mission = MissionContext(
        start_rowcol=(40, 40), start_z_msl=start_z, goal_rowcol=(40, 60), goal_z_msl=start_z,
        ceiling_msl=goal_z, min_agl_m=min_agl,
    )
    gen = CandidateZGenerator(store, mission)
    state = (40, 40, encode_candidate_altitude(start_z))
    args = (state, terrain, cfg, 0.0, 10000.0, None, {"hits": 0, "misses": 0, "actual_calls": 0}, None, None, None, None, gen)
    # No aircraft_profile -- REP-1.2B behavior only.
    r_no_profile = _generate_candidate_z_neighbors(*args)
    single_step_only_reaches_100m = any(
        abs(decode_candidate_altitude(ns[2]) - goal_z) < 1e-6 for ns, _ in r_no_profile[0]
    )
    check("without aircraft_profile, a 100m climb never appears as a successor", not single_step_only_reaches_100m)

    print("=== B: same delta_z over a sufficient multi-cell horizon -> accepted ===")
    r_with_profile = _generate_candidate_z_neighbors(*args, aircraft_profile=profile)
    reaches_via_horizon = [
        (ns, cost) for ns, cost in r_with_profile[0] if abs(decode_candidate_altitude(ns[2]) - goal_z) < 1e-6
    ]
    check("with aircraft_profile, the same 100m climb IS accepted via a longer horizon", len(reaches_via_horizon) >= 1)
    # East-direction successors only (row unchanged, col increases) -- isolates a single, predictable
    # direction for the numeric horizon checks below (C, H), since flat terrain lets every direction succeed.
    reaches_via_horizon_east = [(ns, c) for ns, c in reaches_via_horizon if ns[0] == 40 and ns[1] > 40]

    print("=== C: minimum horizon computed conservatively (ceil) ===")
    min_horizontal = derive_minimum_horizontal_distance_m(start_z, goal_z, profile)
    expected_n_cells = math.ceil(min_horizontal / RES - 1e-9)
    if reaches_via_horizon_east:
        ns, _ = reaches_via_horizon_east[0]
        actual_n_cells = ns[1] - 40  # east direction, dcol
        check(
            f"accepted successor uses the conservatively-rounded-up cell count "
            f"(min_horizontal={min_horizontal:.1f}m, expected>={expected_n_cells} cells, got {actual_n_cells})",
            actual_n_cells >= expected_n_cells,
        )
    else:
        check("accepted successor uses the conservatively-rounded-up cell count", False)

    print("=== D: one cell short of the minimum horizon -> rejected ===")
    short_prim = primitive_for_target_altitude_over_horizon("E", start_z, goal_z, expected_n_cells - 1, cfg)
    short_ok = short_prim is None
    if short_prim is not None:
        duration_s = short_prim.horizontal_distance_m / profile.manifest.nominal_ias_context_mps
        motion = evaluate_vertical_motion(start_z, goal_z, duration_s, profile)
        short_ok = motion.status != "FEASIBLE"
    check(f"n_cells={expected_n_cells - 1} (one short) is geometrically invalid or CLASS-C-infeasible", short_ok)

    print("=== E: exact minimum/conservative horizon -> accepted ===")
    exact_prim = primitive_for_target_altitude_over_horizon("E", start_z, goal_z, expected_n_cells, cfg)
    exact_ok = exact_prim is not None
    if exact_prim is not None:
        duration_s = exact_prim.horizontal_distance_m / profile.manifest.nominal_ias_context_mps
        motion = evaluate_vertical_motion(start_z, goal_z, duration_s, profile)
        exact_ok = motion.status == "FEASIBLE"
    check(f"n_cells={expected_n_cells} (conservative minimum) is geometrically valid AND CLASS-C-feasible", exact_ok)

    print("=== F: 5000m positive climb -> unavailable/rejected regardless of horizon ===")
    unavailable_horizon = derive_minimum_horizontal_distance_m(5000.0, 5100.0, profile)
    check("derive_minimum_horizontal_distance_m returns None at 5000m CLIMB (profile UNAVAILABLE there)", unavailable_horizon is None)
    huge_prim = primitive_for_target_altitude_over_horizon("E", 5000.0, 5100.0, 60, cfg)
    if huge_prim is not None:
        duration_s = huge_prim.horizontal_distance_m / profile.manifest.nominal_ias_context_mps
        motion = evaluate_vertical_motion(5000.0, 5100.0, duration_s, profile)
        check("even a 60-cell horizon is not FEASIBLE at 5000m (long horizon cannot rescue UNAVAILABLE)", motion.status != "FEASIBLE")
    else:
        check("even a 60-cell horizon is not FEASIBLE at 5000m (geometry itself already invalid)", True)

    print("=== G: descent uses altitude-local safe capability ===")
    descent_low = derive_minimum_horizontal_distance_m(1000.0, 900.0, profile)
    descent_high = derive_minimum_horizontal_distance_m(3500.0, 3400.0, profile)
    check(
        "descent horizon differs between two different altitudes (queried locally, not a global constant)",
        descent_low is not None and descent_high is not None and abs(descent_low - descent_high) > 1e-6,
    )

    print("=== H: terrain along intermediate cells can reject an otherwise kinematically valid transition ===")
    ridge_elev = np.full((80, 80), 1000.0, dtype=np.float32)
    ridge_row = 40
    ridge_col_mid = 40 + (expected_n_cells // 2 if expected_n_cells > 1 else 1)
    ridge_elev[ridge_row, ridge_col_mid] = 1000.0 + 400.0  # a tall ridge strictly between source and destination
    ridge_terrain = make_terrain(ridge_elev)
    ridge_store = TerrainMetadataStore(ridge_terrain)
    ridge_mission = MissionContext(
        start_rowcol=(40, 40), start_z_msl=start_z, goal_rowcol=(40, 60), goal_z_msl=start_z,
        ceiling_msl=goal_z, min_agl_m=min_agl,
    )
    ridge_gen = CandidateZGenerator(ridge_store, ridge_mission)
    ridge_state = (40, 40, encode_candidate_altitude(start_z))
    ridge_args = (ridge_state, ridge_terrain, cfg, 0.0, 10000.0, None, {"hits": 0, "misses": 0, "actual_calls": 0}, None, None, None, None, ridge_gen)
    r_ridge = _generate_candidate_z_neighbors(*ridge_args, aircraft_profile=profile)
    # Only the EAST direction crosses the ridge (placed at row 40); N/S/W climbs are untouched by
    # it, so this checks specifically that EAST's otherwise-kinematically-valid climb is blocked.
    ridge_blocks_east_climb = not any(
        ns[0] == 40 and ns[1] > 40 and abs(decode_candidate_altitude(ns[2]) - goal_z) < 1e-6
        for ns, _ in r_ridge[0]
    )
    check("a mid-path ridge rejects the multi-cell EAST climb even though the kinematics alone would allow it", ridge_blocks_east_climb)

    print("=== I: no intermediate Z search states created ===")
    z_values_in_result = {decode_candidate_altitude(ns[2]) for ns, _ in r_with_profile[0]}
    legal_targets = {start_z} | set(gen.generate(40, 40)) | set(gen.generate(40, 41))
    no_intermediate_states = z_values_in_result.issubset({v for v in legal_targets} | {goal_z})
    check(
        "every successor altitude is start_z, a generate() event, or the exact multi-cell target -- no ad-hoc intermediate Z",
        z_values_in_result.issubset(legal_targets | {goal_z}),
    )

    print("=== J: CandidateZ target altitude preserved exactly ===")
    if reaches_via_horizon:
        ns, _ = reaches_via_horizon[0]
        check("accepted successor's decoded altitude equals goal_z to float precision", abs(decode_candidate_altitude(ns[2]) - goal_z) < 1e-9)
    else:
        check("accepted successor's decoded altitude equals goal_z to float precision", False)

    print("=== K: no regular z_step successor path used ===")
    import inspect
    from planner import astar as astar_mod
    src = inspect.getsource(astar_mod._generate_candidate_z_neighbors)
    code_only = src.split('"""', 2)[-1] if src.count('"""') >= 2 else src
    check("no 'z_step_m' token in _generate_candidate_z_neighbors' CODE", "z_step_m" not in code_only)
    check("no 'z_index' token in _generate_candidate_z_neighbors' CODE", "z_index" not in code_only)

    print()
    print(f"Overall: {'ALL PASS' if all_pass else 'SOME FAILED'}")


if __name__ == "__main__":
    main()
