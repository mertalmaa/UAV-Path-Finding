"""Lowest feasible altitude envelope along a fixed, sampled horizontal path.

This is a difference-constraint solver, independent of terrain and aircraft
models. Its guarantee applies only to supplied stations and interval budgets:
callers must establish terrain clearance between stations and translate real
aircraft limits into conservative climb/descent budgets themselves. It does
not account for pitch transitions, acceleration, or changing horizontal paths.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


_ROUNDOFF_TOLERANCE_M = 1e-9


@dataclass(frozen=True)
class AltitudeProfileResult:
    """Solver outcome; infeasible outcomes contain no usable altitudes."""

    altitudes_msl_m: tuple[float, ...]
    feasible: bool
    status: str
    reason: str
    failure_index: int | None = None


def _finite_number(value: float, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must be a finite number")
    return number


def _finite_sequence(values: Sequence[float], name: str) -> tuple[float, ...]:
    try:
        return tuple(_finite_number(value, f"{name}[{index}]")
                     for index, value in enumerate(values))
    except TypeError as exc:
        raise ValueError(f"{name} must be a sequence of finite numbers") from exc


def solve_lowest_altitude_profile(
    floor_msl_m: Sequence[float],
    max_climb_m: Sequence[float],
    max_descent_m: Sequence[float],
    *,
    start_altitude_msl_m: float,
    end_altitude_msl_m: float,
    ceiling_msl_m: Sequence[float] | None = None,
) -> AltitudeProfileResult:
    """Return the componentwise lowest feasible station altitude profile.

    For N stations, floor and optional ceiling contain N finite MSL altitudes.
    Climb/descent arrays contain N-1 finite, nonnegative altitude budgets in
    metres. At interval i the constraints are::

        -max_descent_m[i] <= z[i+1] - z[i] <= max_climb_m[i]

    Endpoints must equal the requested altitudes exactly. Propagated endpoint
    and interval comparisons allow 1e-9 m of floating-point roundoff; this is
    not a terrain clearance margin. Floors and ceilings remain hard bounds.
    Invalid inputs raise
    ValueError; valid but incompatible constraints return an infeasible result.
    A one-station path is valid only if both endpoint altitudes are identical.

    Proof of minimality: seed each station with its floor, additionally seeding
    endpoints with their requested altitudes. In a backward pass enforce
    ``z[i] >= z[i+1] - climb[i]``. Every feasible profile must dominate these
    propagated lower bounds. A forward pass then enforces
    ``z[i+1] >= z[i] - descent[i]``, again producing necessary lower bounds.
    This second pass preserves the first pass's climb inequalities: either a
    station retains its old value, whose predecessor only increased, or it is
    raised to its predecessor minus a nonnegative descent budget. Thus the
    result satisfies both interval inequalities and every feasible solution
    dominates it. If it exceeds an endpoint or ceiling, no solution exists;
    otherwise it is the componentwise minimum. Time and storage are O(N).
    """
    floor = _finite_sequence(floor_msl_m, "floor_msl_m")
    climb = _finite_sequence(max_climb_m, "max_climb_m")
    descent = _finite_sequence(max_descent_m, "max_descent_m")
    start = _finite_number(start_altitude_msl_m, "start_altitude_msl_m")
    end = _finite_number(end_altitude_msl_m, "end_altitude_msl_m")
    count = len(floor)
    if count == 0:
        raise ValueError("floor_msl_m must contain at least one station")
    if len(climb) != count - 1 or len(descent) != count - 1:
        raise ValueError("climb and descent budgets must have N-1 entries")
    if any(value < 0.0 for value in climb + descent):
        raise ValueError("climb and descent budgets must be nonnegative")
    ceiling = None
    if ceiling_msl_m is not None:
        ceiling = _finite_sequence(ceiling_msl_m, "ceiling_msl_m")
        if len(ceiling) != count:
            raise ValueError("ceiling_msl_m must have N entries")

    if count == 1 and start != end:
        return AltitudeProfileResult(
            (), False, "ENDPOINT_CONFLICT",
            "a single station requires identical start and end altitudes", 0,
        )
    for index, requested, label in ((0, start, "start"), (count - 1, end, "end")):
        if requested < floor[index]:
            return AltitudeProfileResult(
                (), False, "ENDPOINT_CONFLICT",
                f"{label} altitude {requested:g} m is below the floor "
                f"{floor[index]:g} m at station {index}", index,
            )

    altitude = list(floor)
    altitude[0] = max(altitude[0], start)
    altitude[-1] = max(altitude[-1], end)
    for index in range(count - 2, -1, -1):
        altitude[index] = max(altitude[index], altitude[index + 1] - climb[index])
    for index in range(count - 1):
        altitude[index + 1] = max(altitude[index + 1], altitude[index] - descent[index])

    for index, requested, label in ((0, start, "start"), (count - 1, end, "end")):
        if altitude[index] - requested > _ROUNDOFF_TOLERANCE_M:
            return AltitudeProfileResult(
                (), False, "ENDPOINT_CONFLICT",
                f"{label} altitude {requested:g} m is below the required "
                f"{altitude[index]:g} m at station {index}", index,
            )
        altitude[index] = requested

    # Pin exact endpoints after tolerating arithmetic noise, and independently
    # check the returned profile against the same interval roundoff allowance.
    for index in range(count - 1):
        change = altitude[index + 1] - altitude[index]
        if (change - climb[index] > _ROUNDOFF_TOLERANCE_M
                or -change - descent[index] > _ROUNDOFF_TOLERANCE_M):
            return AltitudeProfileResult(
                (), False, "INTERVAL_CONFLICT",
                f"altitude change at interval {index} exceeds its budget", index,
            )
    if ceiling is not None:
        for index, (value, limit) in enumerate(zip(altitude, ceiling)):
            if value > limit:
                return AltitudeProfileResult(
                    (), False, "CEILING_CONFLICT",
                    f"required altitude {value:g} m exceeds ceiling "
                    f"{limit:g} m at station {index}", index,
                )
    return AltitudeProfileResult(tuple(altitude), True, "FEASIBLE", "")
