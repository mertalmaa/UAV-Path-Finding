"""Central configuration for the terrain-aware fixed-wing path planner.

Single source of truth for parameters shared across planner modules (DEM
paths, working CRS, ROI geometry). Fields marked PLACEHOLDER are not
physically validated -- they exist so later modules have a stable place to
read from, not because a real value has been decided. Do not use them for
actual planning until they are replaced with sourced numbers.
"""
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple


@dataclass(frozen=True)
class PlannerConfig:
    # --- DEM / CRS ---
    working_dem_path: Path = Path("working_dem/aladaglar_N37_E035_utm36n_max.tif")
    target_crs: str = "EPSG:32636"  # UTM 36N; covers the Aladaglar tile with <0.01% distortion
    nodata_value: float = -9999.0

    # --- ROI geometry ---
    roi_center_lonlat: Tuple[float, float] = (35.14833, 37.80611)  # source DEM max-elevation pixel, near Demirkazik
    roi_size_m: float = 10_000.0  # 10x10 km, this stage's scope
    xy_resolution_m: float = 30.0  # working DEM pixel size

    # --- Vertical (Z) lattice step -- ACTIVE production parameter ---
    # CORRECTION (2026-09-13): the old comment here ("Reserved for future
    # 3D state-space work; unused at this stage") predates the actual 3D
    # integration and is no longer true. z_step_m is now load-bearing:
    # planner/primitives.py's build_primitive_set() derives climb/descent
    # dz from it, both search layers (astar.py, coarse_astar.py) anchor
    # CanonicalState.z_index to it, and CandidateZGenerator.is_representable()
    # (planner/candidate_z.py, Step REP-1) checks against it directly. See
    # project.md "Step REP-1"/"Step REP-1.1" for why the regular z_step_m
    # lattice could not yet be fully replaced by sparse/lazy CandidateZ.
    z_step_m: float = 20.0

    # --- PLACEHOLDERS turned TEST PARAMETERS ---
    # None of these are sourced from a real aircraft/mission requirement.
    # They exist so the feasibility gates below (planner/agl.py,
    # planner/transition.py) have something concrete to validate against.
    # Replace with real numbers before any of this is used for actual planning.
    min_agl_m: Optional[float] = 200.0  # TEST PARAMETER, not a real requirement
    max_climb_angle_deg: Optional[float] = 10.0  # TEST PARAMETER, not a real requirement
    max_descent_angle_deg: Optional[float] = 10.0  # TEST PARAMETER, not a real requirement

    # --- Numerical resolution parameter (not an aircraft performance limit) ---
    # Max spacing between terrain samples along a motion primitive. This is
    # a prototype value for validating primitive feasibility, not something
    # derived from aircraft capability.
    primitive_sample_spacing_m: float = 10.0

    # --- A* cost tuning (not an aircraft/mission requirement) ---
    # How strongly the A* edge cost prefers lower absolute MSL altitude
    # among otherwise-safe routes. 0.0 reproduces the plain-geometric
    # baseline A* exactly. This is a TEST/TUNING PARAMETER for prototyping
    # the cost shape, not a sourced physical or mission value.
    msl_cost_weight: float = 0.25  # TEST/TUNING PARAMETER, not a real requirement -- re-tuning pending under the new fixed-scale normalization below

    # Fixed-scale MSL normalization: altitude_scaled = max(0, mean_altitude_msl
    # - msl_reference_m) / msl_scale_m, NOT clamped to [0,1] and NOT relative
    # to a search call's min/max_search_altitude_msl (that produced the same
    # physical altitude getting a different cost depending on search ceiling
    # -- see project.md "Stage 10"). msl_reference_m=0.0 is sea level;
    # msl_scale_m=1000.0 is purely a cost-scaling constant. Both are
    # TEST/TUNING PARAMETERS, not aircraft/mission values.
    msl_reference_m: float = 0.0  # TEST/TUNING PARAMETER, not a real requirement
    msl_scale_m: float = 1000.0  # TEST/TUNING PARAMETER, not a real requirement

    # --- Cost model selection (Stage 32) ---
    # "legacy": the Stage 1-31 production formula, kept bit-for-bit unchanged
    # (compute_edge_cost's legacy branch). "normalized": the dimensionless
    # distance/altitude/reversal cost from project.md "Stage 32" (see
    # planner/astar.py compute_edge_cost / _compute_edge_cost_normalized).
    # "legacy" remains the default -- normalized is opt-in only via an
    # explicit dataclasses.replace(), not yet validated as a production
    # default.
    cost_mode: str = "legacy"

    # --- Normalized cost mode parameters (Stage 32) -- read ONLY when
    # cost_mode == "normalized"; inert (never referenced) under "legacy". ---
    # Explicit mission/cost altitude reference for the normalized excess-MSL
    # term. Unlike the legacy msl_reference_m (fixed at sea level) or Stage
    # 30/31's diagnostic H_FLOOR (which was DERIVED from a search call's own
    # min_search_altitude_msl -- a stability risk flagged in project.md
    # "Stage 31"/"Stage 32"), this must be set explicitly by the caller; it
    # is never derived from search bounds and never changes with them.
    # None means "not configured" -- normalized mode requires a real value,
    # checked at cost-computation time (planner/astar.py raises if unset).
    altitude_reference_msl: Optional[float] = None
    # H_scale: normalizes excess-altitude (MSL above altitude_reference_msl)
    # into a dimensionless quantity, matched in scale to the dimensionless
    # distance ratio (ds/D_ref). TEST/TUNING PARAMETER -- Stage 32
    # calibration candidate, not a sourced physical value.
    normalized_altitude_scale_m: float = 1000.0
    # Component weights for the normalized cost: dC_total = w_distance*
    # dC_distance + w_altitude*dC_altitude. Both TEST/TUNING PARAMETERS,
    # not sourced physical/mission values.
    normalized_w_distance: float = 1.0
    # Production mission-policy candidate. Normalized mode remains explicit;
    # this does not alter legacy-mode callers.
    normalized_w_altitude: float = 1.25

    # --- Safe goal region (Stage 33) -- production feature, opt-in. ---
    # A state counts as "at goal" if its physical position is within this
    # axis-aligned box (+/- goal_tolerance_xy_m in X and Y, +/- goal_tolerance_z_m
    # in Z) around the goal center, rather than requiring the exact goal state.
    # Both default to 0.0, which reproduces the pre-Stage-33 EXACT goal condition
    # bit-for-bit (see planner/astar.py _state_in_goal_region) -- this loosens
    # ONLY target-location precision, never any safety constraint: AGL, terrain
    # collision, NoData, bounds, and max climb/descent angle are enforced by
    # evaluate_primitive() before a state ever becomes a candidate at all, so a
    # state inside the box that fails any of those is never reachable as "goal"
    # regardless of tolerance.
    goal_tolerance_xy_m: float = 0.0
    goal_tolerance_z_m: float = 0.0

    # Note (Step CLEAN-1): the sparse/lazy Z representation (Roadmap Step 3E,
    # planner/candidate_z.py) has no config flag -- astar_search() simply
    # accepts an optional candidate_z_generator argument; passing one IS the
    # "sparse_lazy" behavior, passing None IS the old "legacy" behavior. There
    # used to be a representation_mode enum selecting between the two; it was
    # removed because it only ever gated that same single boolean.


DEFAULT_CONFIG = PlannerConfig()
