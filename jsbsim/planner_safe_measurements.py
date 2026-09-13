"""Measurement helpers used only by derived/planner-safe processing.

Historical RAW artifacts are immutable.  These helpers implement the canonical
shortest-angle convention discovered during U6.1 without rewriting the runner
or any previously generated measurement.
"""

from __future__ import annotations

import math
from collections.abc import Iterable


def shortest_angle_deg(angle_deg: float) -> float:
    """Return the equivalent angle in [-180, 180)."""
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def unwrap_degrees(samples_deg: Iterable[float]) -> list[float]:
    """Unwrap a sequence using canonical shortest deltas."""
    samples = [float(value) for value in samples_deg]
    if not samples:
        return []
    output = [samples[0]]
    for sample in samples[1:]:
        output.append(output[-1] + shortest_angle_deg(sample - output[-1]))
    return output


def radius_from_arc_and_course_change(
    arc_length_m: float, course_change_deg: float
) -> tuple[float, float]:
    """Return radius and the normalized course change used to compute it."""
    wrapped = shortest_angle_deg(course_change_deg)
    if not math.isfinite(arc_length_m) or arc_length_m <= 0.0:
        raise ValueError("arc_length_m must be finite and positive")
    if abs(wrapped) <= 1.0e-12:
        raise ValueError("course change is zero after shortest-angle normalization")
    return arc_length_m / abs(math.radians(wrapped)), wrapped
