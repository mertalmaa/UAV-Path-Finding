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
    working_dem_path: Path = Path("regions/bilecik/working_dem.tif")
    target_crs: str = "EPSG:32636"  # Bilecik working DEM, UTM 36N
    nodata_value: float = -9999.0

    # --- ROI geometry ---
    roi_center_lonlat: Tuple[float, float] = (30.30, 40.25)
    roi_size_m: float = 30_000.0  # Bilecik mission area, 30x30 km
    xy_resolution_m: float = 30.0  # working DEM pixel size

    # CandidateZ terrain-floor event quantization. This rounds the
    # conservative clearance floor upward; it is not a state lattice or
    # successor delta.
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

    # --- Pose-aware fixed-wing search representation (first baseline) ---
    # These are dominance/hash buckets only.  They never alter a
    # PhysicalPose or a validated continuous trajectory.
    search_xy_bin_m: float = 60.0
    search_z_bin_m: float = 5.0
    search_heading_bin_deg: float = 15.0
    # Climbing/descending turns use the same continuous safety pipeline.
    # BASIC remains available for controlled comparisons.
    enable_combined_turns: bool = True
    # Weighted search trades exact optimality for bounded practical search.
    # 1.3: validated on Bilecik 90 km missions (bounded greediness at long range).
    search_heuristic_weight: float = 1.3
    # Deprecated compatibility field; no effect even when True. Cross-altitude
    # goal/terrain-proximity pruning was unsound and has been removed.
    # Each full (x, y, z, heading) key still keeps one approximate representative.
    enable_pareto_z_pruning: bool = False
    # Optional terrain-relative soft objective.  It is deliberately
    # opt-in: safety is still governed solely by min_agl_m and continuous
    # trajectory validation.  When disabled, pose-aware A* uses its original
    # geometric 3D edge length without terrain queries in the cost path.
    enable_low_altitude_cost: bool = False
    # Optional topographic Dijkstra ordering with climb/descent lookahead.
    # Works independently of low-altitude cost. Approximate guidance, not an
    # admissible heuristic claim; never bypasses full trajectory safety.
    enable_terrain_guidance: bool = True
    # Soft ordering only: sample the terrain guide at this pixel stride.
    # Full-resolution trajectory safety remains unchanged.
    terrain_guidance_stride: int = 3
    # Dual-queue multi-heuristic search (RR-MHA*): interleave inadmissible
    # topographic guidance queue with the admissible anchor queue to guarantee
    # completeness while preventing heuristic deception traps.
    # guidance_queue_ratio = K means K pops from guided queue for every 1 pop from anchor queue.
    guidance_queue_ratio: int = 3
    # Guidance cost field shape.
    #   "absolute_quadratic": historical 1 + 2*((elev-lo)/(hi-lo))^2, normalised by
    #       the WHOLE ROI relief. Works on 30 km maps where the target valley is the
    #       global minimum, but its contrast collapses on large maps (90 km).
    #   "valley_relative": 1 + alpha*min(HAND/scale, cap) where HAND is the height
    #       above the local terrain floor (moving minimum over valley_window_m).
    #       Scale-invariant: a valley at 500 m MSL is as attractive as one at 50 m.
    guidance_cost_mode: str = "valley_relative"
    valley_window_m: float = 5000.0
    valley_height_scale_m: float = 300.0
    # 2.0: 2.5 without an edge margin sent Bilecik M90_02 on a 2x detour.
    valley_cost_alpha: float = 2.0
    valley_cost_cap: float = 3.0
    # Soft ROI edge repulsion for guidance only (0 disables). Cells closer than
    # this distance to the ROI border get up to +guidance_edge_cost.
    # Default 0: Bilecik 31 km canyon runs ~300 m from the ROI edge; a 3 km
    # margin pushed that route onto the ridges (+10 km, +250 m max MSL).
    guidance_edge_margin_m: float = 0.0
    guidance_edge_cost: float = 2.0
    # When True, the A* g-cost integrates the guidance cost multiplier even with
    # enable_low_altitude_cost=False. This keeps g and the guided heuristic in
    # the same units (a real valley preference instead of greedy following)
    # without adding the z-dependent AGL term that floods z bins.
    guidance_multiplier_in_g: bool = True
    desired_agl_m: float = 120.0
    agl_cost_scale_m: float = 100.0
    lambda_agl: float = 0.0
    # linear: non-saturating excess-AGL cost; quadratic/capped_linear retain
    # their historical behavior for controlled comparisons.
    low_altitude_cost_shape: str = "capped_linear"
    full_penalty_agl_m: float = 500.0
    max_agl_cost_multiplier: float = 1.0
    # Centreline terrain semantics are the frozen first-baseline default.
    # A non-zero value is an explicit safety-model choice, not an aircraft
    # dimension and not a substitute for curve-to-chord coverage.
    lateral_buffer_m: float = 0.0

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

    # CandidateZ is the single production altitude representation and has no
    # mode flag. astar_search() requires a CandidateZGenerator.


DEFAULT_CONFIG = PlannerConfig()
