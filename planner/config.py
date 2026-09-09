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

    # --- Reserved for future 3D state-space work; unused at this stage ---
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

    # How strongly the A* edge cost penalizes a vertical-DIRECTION REVERSAL
    # (climb->descent or descent->climb), SCALED by how long the standing
    # trend had been running (see reversal_relax_distance_m below and
    # planner/astar.py _next_trend_and_age). A continuous climb or
    # continuous descent pays nothing here, no matter how long -- only
    # flip-flopping does, and a reversal after a long, stable trend costs
    # little to nothing. Replaces the earlier abs(delta_z)-per-primitive
    # penalty (and the even earlier "first reversal is free" special case)
    # -- neither is part of the production cost anymore. This is NOT a
    # real aircraft energy model -- just a TEST/TUNING PARAMETER for a
    # zigzag/smoothness preference.
    vertical_reversal_cost_weight: float = 1.0  # TEST/TUNING PARAMETER, not a real requirement

    # Horizontal distance a vertical trend (continuous climb or descent,
    # level moves included) must have run for before a reversal away from
    # it is completely free. Below this, the reversal cost is scaled down
    # linearly (see _next_trend_and_age). A prototype smoothness knob, not
    # a real aircraft/mission requirement.
    reversal_relax_distance_m: float = 300.0  # TEST/TUNING PARAMETER, not a real requirement

    # Discretization granularity for the trend-age carried in the search
    # state (row,col,z_index,vertical_trend,trend_age_units) -- an
    # implementation/state-space detail, not a behavior preference to tune.
    # trend_age_units is capped at ceil(reversal_relax_distance_m /
    # trend_age_unit_m) so the augmented state stays a small bounded integer.
    trend_age_unit_m: float = 30.0


DEFAULT_CONFIG = PlannerConfig()
