"""Step CLASS-C: the physical vertical-motion feasibility BRIDGE between
Z representation (planner/candidate_z.py) and real aircraft vertical
capability (planner/aircraft_profile.py). Search-independent, not wired
into planner/astar.py's successor generation -- see module docstring in
planner/candidate_z.py and project.md "Step CLASS-C" for why.

Audit finding this module exists to close (project.md "Step CLASS-C"):
production search's Z state is STILL the plain regular z_step_m lattice
(`z_index_to_msl(z_index) = z_index * config.z_step_m`, planner/astar.py)
-- CandidateZGenerator.floor_for() is only ever consulted as an efficiency
PREFILTER on top of that lattice (Step 3E), never as a replacement state
space; CandidateZGenerator.generate() (the CLASS A/B/C sparse candidate
SET Step 3A/3B designed) is not called anywhere in production. Nothing in
the codebase, until this module, ever compared a Z transition's implied
vertical rate against real aircraft capability -- evaluate_primitive()
checks terrain/AGL/climb-angle only.

Three distinct concepts this module keeps separate (see also
planner/candidate_z.py's own module docstring, and this repo's Step
CLASS-C decision record):

  REPRESENTABILITY -- which altitude values CAN exist as planner state at
    all. Decided entirely by planner/candidate_z.py (the z_step_m lattice
    today; CLASS A/B/C events if/when generate() is ever wired in) and
    planner/astar.py's search bounds. This module has NO opinion on it
    and never proposes a new representable altitude.

  PHYSICAL REACHABILITY -- whether the aircraft can actually fly from one
    representable altitude to another representable altitude within a
    given motion duration/path length, per planner-safe capability. This
    is exactly what evaluate_vertical_motion() below answers.

  INSTANTIATION -- whether a representable state was actually created
    during a particular search run. Entirely planner/astar.py's concern
    (open/closed sets, lazy generation); this module has no visibility
    into it at all.

REPRESENTABLE ENDPOINT != FEASIBLE EDGE: a candidate altitude existing
(CandidateZ said so, or it's on the lattice) never implies the aircraft
can physically fly there from wherever it currently is in the time/
distance a primitive would allow. evaluate_vertical_motion() is the
missing check for exactly that gap.

NO RESIDUAL/HISTORY STATE: this module takes a single (source_altitude_m,
target_altitude_m, motion_duration_s) transition and answers it in
isolation -- it never accumulates "progress" across calls, never reads or
writes anything resembling search history, and returns the identical
result for the identical inputs every time (referentially transparent).
The "lost partial progress" problem this stage was asked to analyze
(project.md "Step CLASS-C") is resolved by NOT tracking partial vertical
progress at all: the permanent design principle carried forward is that a
future vertical-motion primitive must itself span a COMPLETE transition
between two representable endpoints (whatever horizontal distance/time
that requires), never a partial climb needing a later primitive to
"remember" how far it got. This module doesn't implement that primitive
(Step 19: no primitives here) -- it only defines the contract such a
primitive would call once its full duration/path-length is known.

Units: SI throughout (meters, seconds, meters/second), matching planner.
aircraft_profile.
"""
from dataclasses import dataclass, field
from typing import Any, Mapping, Optional

VALID_STATUS_VALUES = frozenset({"FEASIBLE", "PHYSICALLY_UNAVAILABLE", "OUT_OF_PROFILE_DOMAIN", "INVALID_DURATION"})

_VZ_TOLERANCE_MPS = 1e-6  # float-noise tolerance on the required-vs-safe comparison, not a safety margin


@dataclass(frozen=True)
class VerticalMotionResult:
    """Response from evaluate_vertical_motion(). status is the actionable
    field; the rest is diagnostic context for callers/logging/tests.

    status:
      FEASIBLE -- required_vz_mps is within the profile's local planner-
        safe capability (or delta_z_m is ~0, the trivial level case).
      PHYSICALLY_UNAVAILABLE -- the profile reports this altitude/mode as
        UNAVAILABLE outright, or the required rate exceeds the local safe
        capability even though the maneuver itself is AVAILABLE there.
      OUT_OF_PROFILE_DOMAIN -- source_altitude_m is outside the aircraft
        profile's declared altitude domain -- distinct from
        PHYSICALLY_UNAVAILABLE (see planner.aircraft_profile: OUT_OF_
        DOMAIN and UNAVAILABLE are never conflated).
      INVALID_DURATION -- motion_duration_s was <= 0 (or None) for a
        non-zero delta_z_m -- not a physical-capability question at all,
        a malformed-call question.
    """
    status: str
    required_vz_mps: Optional[float]
    safe_vz_mps: Optional[float]
    delta_z_m: float
    duration_s: Optional[float]
    availability: str  # "AVAILABLE" | "UNAVAILABLE" | "OUT_OF_DOMAIN" -- passed through from the profile query
    query_altitude_m: float
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class SafeVerticalRateResult:
    """Response from query_safe_vertical_rate() -- the single place that
    reads a mode's planner-safe rate off an AircraftProfile, shared by
    evaluate_vertical_motion() and derive_minimum_horizontal_distance_m()
    (Step REP-1.2B.1) so there is exactly one lookup, not two drifting
    copies of the same `f"{mode.lower()}_vz_mps"` key access."""
    safe_vz_mps: Optional[float]  # None iff availability != "AVAILABLE"
    availability: str
    reason_unavailable: Optional[str]


def query_safe_vertical_rate(aircraft_profile, altitude_m: float, mode: str) -> SafeVerticalRateResult:
    """AVAILABLE iff aircraft_profile reports this mode as AVAILABLE at
    altitude_m; PHYSICALLY_UNAVAILABLE/OUT_OF_DOMAIN otherwise, with
    safe_vz_mps=None -- callers must not treat a longer motion duration as
    a way to make an unavailable maneuver available (see module docstring
    and project.md "Step REP-1.2B.1")."""
    q = aircraft_profile.vertical_query(altitude_m, mode)
    if q.availability != "AVAILABLE":
        return SafeVerticalRateResult(safe_vz_mps=None, availability=q.availability, reason_unavailable=q.reason_unavailable)
    key = f"{mode.lower()}_vz_mps"
    if key not in q.planner_safe:
        raise ValueError(
            f"AVAILABLE {mode} row at altitude_m={altitude_m} has no {key!r} in planner_safe "
            f"({sorted(q.planner_safe)}) -- schema/profile mismatch, not a physical-unavailability result"
        )
    return SafeVerticalRateResult(safe_vz_mps=q.planner_safe[key], availability="AVAILABLE", reason_unavailable=None)


def derive_minimum_horizontal_distance_m(
    source_altitude_m: float,
    target_altitude_m: float,
    aircraft_profile,
) -> Optional[float]:
    """Step REP-1.2B.1: the minimum horizontal distance, flown at the
    profile's own domain-declared nominal_ias_context_mps, an aircraft
    would need to complete this altitude change at its LOCAL planner-safe
    vertical rate (queried at source_altitude_m, same policy as
    evaluate_vertical_motion). Returns 0.0 for a ~zero delta_z (no vertical
    motion needed), and None if the climb/descent is PHYSICALLY_UNAVAILABLE
    (or OUT_OF_DOMAIN) at source_altitude_m -- per Step CLASS-C's contract,
    a longer horizon/duration can never make an unavailable maneuver
    available, so this deliberately does not return a distance in that
    case rather than one that would silently be infinite.

    This is a SIZING helper only (deterministic, no search/history) --
    the actual FEASIBILITY verdict for whatever horizon a caller ultimately
    picks (e.g. after rounding up to a whole number of grid cells) must
    still come from calling evaluate_vertical_motion() on that concrete
    (distance, duration) pair -- this function does not replace it, and
    the two share their rate lookup via query_safe_vertical_rate() so
    there is exactly one place that reads a profile's safe rate.
    """
    delta_z = target_altitude_m - source_altitude_m
    if abs(delta_z) < 1e-9:
        return 0.0
    mode = "CLIMB" if delta_z > 0.0 else "DESCENT"
    rate = query_safe_vertical_rate(aircraft_profile, source_altitude_m, mode)
    # DESCENT's planner_safe rate is stored as a NEGATIVE number (e.g. -3.2 m/s, matching
    # descent_vz_mps's own sign convention in the V3 schema) -- compare/divide by MAGNITUDE,
    # never assume a positive safe_vz_mps regardless of mode.
    if rate.safe_vz_mps is None or abs(rate.safe_vz_mps) <= 0.0:
        return None
    min_duration_s = abs(delta_z) / abs(rate.safe_vz_mps)
    return min_duration_s * aircraft_profile.manifest.nominal_ias_context_mps


def evaluate_vertical_motion(
    source_altitude_m: float,
    target_altitude_m: float,
    motion_duration_s: Optional[float],
    aircraft_profile,
) -> VerticalMotionResult:
    """The core physical-transition contract (project.md "Step CLASS-C"):

        required_vz = (target_altitude_m - source_altitude_m) / motion_duration_s

    CLIMB (target > source): FEASIBLE iff required_vz <= local safe climb
    capability. DESCENT (target < source): FEASIBLE iff |required_vz| <=
    |local safe descent capability|. Availability UNAVAILABLE/OUT_OF_DOMAIN
    at the query altitude rejects outright, before any rate comparison.

    Queried at source_altitude_m (the state being left FROM) -- a
    deliberate, documented policy choice, not an arbitrary one: "can this
    aircraft begin this climb/descent from where it currently is" is the
    question a search edge actually needs answered. This module does not
    invent a mid-transition or target-altitude query; a future primitive
    that wants stricter multi-point verification can call this function
    more than once.

    Reads planner_safe.<mode>_vz_mps (e.g. "climb_vz_mps"/"descent_vz_mps")
    -- the V3 schema's own naming convention for "the achievable safe rate"
    for this maneuver family (planner.aircraft_profile's straight_climb/
    straight_descent), never a hard-coded number. Raises ValueError (a
    genuine schema-contract violation, not a normal result) if an
    AVAILABLE row is missing that key -- this should never happen against
    a schema-valid profile and indicates a profile/schema mismatch, not a
    physically-unavailable maneuver.

    Combined (turning) vertical motion is out of scope here -- this
    contract covers planner.aircraft_profile.vertical_query() (straight
    climb/descent) only, not combined_query(); a future heading-aware
    primitive needing turning-vertical feasibility calls that separately.
    """
    delta_z = target_altitude_m - source_altitude_m

    if abs(delta_z) < 1e-9:
        return VerticalMotionResult(
            status="FEASIBLE", required_vz_mps=0.0, safe_vz_mps=0.0, delta_z_m=0.0,
            duration_s=motion_duration_s, availability="AVAILABLE", query_altitude_m=source_altitude_m,
        )

    if motion_duration_s is None or motion_duration_s <= 0.0:
        return VerticalMotionResult(
            status="INVALID_DURATION", required_vz_mps=None, safe_vz_mps=None, delta_z_m=delta_z,
            duration_s=motion_duration_s, availability="UNAVAILABLE", query_altitude_m=source_altitude_m,
            metadata={"reason": "motion_duration_s must be positive for a non-zero altitude change"},
        )

    required_vz = delta_z / motion_duration_s
    mode = "CLIMB" if delta_z > 0.0 else "DESCENT"

    rate = query_safe_vertical_rate(aircraft_profile, source_altitude_m, mode)

    if rate.availability == "OUT_OF_DOMAIN":
        return VerticalMotionResult(
            status="OUT_OF_PROFILE_DOMAIN", required_vz_mps=required_vz, safe_vz_mps=None, delta_z_m=delta_z,
            duration_s=motion_duration_s, availability="OUT_OF_DOMAIN", query_altitude_m=source_altitude_m,
        )
    if rate.safe_vz_mps is None:
        return VerticalMotionResult(
            status="PHYSICALLY_UNAVAILABLE", required_vz_mps=required_vz, safe_vz_mps=None, delta_z_m=delta_z,
            duration_s=motion_duration_s, availability=rate.availability, query_altitude_m=source_altitude_m,
            metadata={"reason_unavailable": rate.reason_unavailable},
        )
    safe_vz = rate.safe_vz_mps

    feasible = (required_vz <= safe_vz + _VZ_TOLERANCE_MPS) if mode == "CLIMB" \
        else (abs(required_vz) <= abs(safe_vz) + _VZ_TOLERANCE_MPS)

    return VerticalMotionResult(
        status="FEASIBLE" if feasible else "PHYSICALLY_UNAVAILABLE",
        required_vz_mps=required_vz, safe_vz_mps=safe_vz, delta_z_m=delta_z,
        duration_s=motion_duration_s, availability=rate.availability, query_altitude_m=source_altitude_m,
    )
