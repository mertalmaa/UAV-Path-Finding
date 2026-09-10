"""Central mission-objective contract shared by coarse and fine planners.

Hard feasibility is deliberately absent from this module. Terrain collision,
minimum AGL, bounds/NoData, and climb/descent limits are decided before an
edge reaches the objective.  This module only ranks already-safe motion.

"Low altitude" always means low *aircraft absolute MSL*.  It does not mean
low AGL, terrain following, low terrain elevation, or a reduced safety margin.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class CostComponents:
    distance: float
    altitude: float
    smoothness: float = 0.0

    @property
    def total(self) -> float:
        return self.distance + self.altitude + self.smoothness


@dataclass(frozen=True)
class MissionPolicy:
    """Normalized soft objective for safe paths.

    ``altitude_reference_msl`` is an explicit mission datum, not a terrain
    lookup and not a search-bound-derived value.  Distance remains a strong
    regularizer, so this is additive rather than lexicographic optimization.
    """

    altitude_reference_msl: float
    altitude_scale_m: float = 1000.0
    w_distance: float = 1.0
    w_altitude: float = 1.25
    w_smoothness: float = 1.0

    def __post_init__(self) -> None:
        if self.altitude_scale_m <= 0.0:
            raise ValueError("altitude_scale_m must be positive")
        if min(self.w_distance, self.w_altitude, self.w_smoothness) < 0.0:
            raise ValueError("mission objective weights must be non-negative")

    def edge_components(
        self,
        geometric_length_m: float,
        start_aircraft_msl: float,
        end_aircraft_msl: float,
        distance_reference_m: float,
        smoothness_raw_m: float = 0.0,
    ) -> CostComponents:
        if geometric_length_m < 0.0:
            raise ValueError("geometric_length_m must be non-negative")
        if distance_reference_m <= 0.0:
            raise ValueError("distance_reference_m must be positive")
        if smoothness_raw_m < 0.0:
            raise ValueError("smoothness_raw_m must be non-negative")

        d_distance = geometric_length_m / distance_reference_m
        mean_msl = (start_aircraft_msl + end_aircraft_msl) / 2.0
        excess_msl = max(0.0, mean_msl - self.altitude_reference_msl)
        d_altitude = d_distance * excess_msl / self.altitude_scale_m
        d_smoothness = smoothness_raw_m / distance_reference_m
        return CostComponents(
            distance=self.w_distance * d_distance,
            altitude=self.w_altitude * d_altitude,
            smoothness=self.w_smoothness * d_smoothness,
        )


PRODUCTION_W_DISTANCE = 1.0
PRODUCTION_W_ALTITUDE = 1.25
PRODUCTION_ALTITUDE_SCALE_M = 1000.0


def production_mission_policy(altitude_reference_msl: float) -> MissionPolicy:
    """Production candidate weights; the reference remains mission-specific."""
    return MissionPolicy(
        altitude_reference_msl=altitude_reference_msl,
        altitude_scale_m=PRODUCTION_ALTITUDE_SCALE_M,
        w_distance=PRODUCTION_W_DISTANCE,
        w_altitude=PRODUCTION_W_ALTITUDE,
    )


def mission_policy_from_config(config) -> MissionPolicy:
    """Adapter for existing PlannerConfig callers in normalized mode."""
    if config.altitude_reference_msl is None:
        raise ValueError("normalized mission objective requires altitude_reference_msl")
    return MissionPolicy(
        altitude_reference_msl=config.altitude_reference_msl,
        altitude_scale_m=config.normalized_altitude_scale_m,
        w_distance=config.normalized_w_distance,
        w_altitude=config.normalized_w_altitude,
        w_smoothness=config.normalized_w_reversal,
    )
