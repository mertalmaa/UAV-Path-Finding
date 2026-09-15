"""Disposable 2B screening for future pose-aware SearchKey quantization.

Run from repository root:
    python -B experiments/search_key_quantization.py

This intentionally does not import planner.astar or modify planner state.  It
screens a deterministic corpus of real passive-geometry endpoints; it is not a
runtime, completeness, or future-frontier-distribution proof.
"""
from __future__ import annotations

import json
import math
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from planner.aircraft_profile import load_aircraft_profile
from planner.fixed_wing_envelope import FixedWingKinematicEnvelope
from planner.physical import (
    PhysicalPose,
    angular_distance_deg,
    build_helical_turn_trajectory,
    build_level_turn_trajectory,
    build_straight_level_trajectory,
    build_straight_vertical_trajectory,
    normalize_heading_deg,
)


PROFILE = ROOT / "jsbsim" / "results" / "c172p_aircraft_profile_planner_safe_v3.json"
XY_CANDIDATES_M = (30.0, 60.0, 90.0)
Z_CANDIDATES_M = (5.0, 10.0, 20.0, 30.0, 40.0, 50.0)
HEADING_CANDIDATES_DEG = (30.0, 22.5, 15.0)
_BOUNDARY_EPSILON = 1e-9


def _bin(value: float, width: float, origin: float = 0.0) -> int:
    """Deterministic half-open floor bin with float-noise boundary stability."""
    return math.floor((value - origin) / width + _BOUNDARY_EPSILON)


@dataclass(frozen=True)
class Endpoint:
    label: str
    family: str
    chain: str
    sequence: int
    pose: PhysicalPose


@dataclass(frozen=True)
class KeySpec:
    xy_bin_m: float
    z_bin_m: float
    heading_bin_deg: float
    x_origin_m: float = 0.0
    y_origin_m: float = 0.0
    z_origin_m: float = 0.0

    def key(self, pose: PhysicalPose) -> Tuple[int, int, int, int]:
        return (
            _bin(pose.x_m, self.xy_bin_m, self.x_origin_m),
            _bin(pose.y_m, self.xy_bin_m, self.y_origin_m),
            _bin(pose.z_msl_m, self.z_bin_m, self.z_origin_m),
            _bin(normalize_heading_deg(pose.heading_deg), self.heading_bin_deg),
        )


def _base_pose(altitude_m: float, heading_deg: float = 0.0) -> PhysicalPose:
    # Deliberately non-grid-centre continuous UTM coordinate.
    return PhysicalPose(500_015.25, 4_200_015.75, altitude_m, heading_deg)


def _append(result: List[Endpoint], label: str, family: str, chain: str, sequence: int, pose: PhysicalPose) -> None:
    result.append(Endpoint(label, family, chain, sequence, pose))


def build_endpoint_corpus() -> List[Endpoint]:
    """Generate deterministic endpoints from actual current primitive geometry."""
    envelope = FixedWingKinematicEnvelope(load_aircraft_profile(PROFILE))
    endpoints: List[Endpoint] = []
    altitudes = (0.0, 1500.0, 4500.0)
    for altitude in altitudes:
        start = _base_pose(altitude)
        for distance in (30.0, 60.0, 120.0, 240.0):
            _append(endpoints, f"straight-{altitude:g}-{distance:g}", "STRAIGHT", "independent", 0,
                    build_straight_level_trajectory(start, distance, 10.0).end_pose)
        for direction in ("LEFT", "RIGHT"):
            turn = envelope.level_turn(altitude, direction)
            if turn.level_turn is not None:
                for change in (15.0, 30.0, 45.0, 90.0, 180.0):
                    _append(endpoints, f"turn-{direction}-{altitude:g}-{change:g}", "LEVEL_TURN", "independent", 0,
                            build_level_turn_trajectory(start, turn.level_turn, change, 10.0).end_pose)
        for mode in ("CLIMB", "DESCENT"):
            vertical = envelope.straight_vertical(altitude, mode)
            if vertical.signed_vertical_rate_mps is not None:
                for distance in (60.0, 120.0, 240.0, 480.0):
                    _append(endpoints, f"vertical-{mode}-{altitude:g}-{distance:g}", f"STRAIGHT_{mode}", "independent", 0,
                            build_straight_vertical_trajectory(start, distance, vertical.signed_vertical_rate_mps, 10.0).end_pose)
        spiral = envelope.spiral_up(altitude, "RIGHT")
        if spiral.combined_turn is not None:
            for change in (15.0, 30.0, 45.0, 90.0, 180.0, 360.0):
                _append(endpoints, f"spiral-{altitude:g}-{change:g}", "SPIRAL_UP", "independent", 0,
                        build_helical_turn_trajectory(start, spiral.combined_turn, change, 10.0).end_pose)

    # Consecutive altitude-progress chains at 60 m: deliberately expose Z-bin
    # collapse risk under a one-representative key policy.
    for mode, start_altitude in (("CLIMB", 1500.0), ("DESCENT", 3000.0)):
        pose = _base_pose(start_altitude, 0.0)
        for sequence in range(1, 13):
            vertical = envelope.straight_vertical(pose.z_msl_m, mode)
            if vertical.signed_vertical_rate_mps is None:
                break
            pose = build_straight_vertical_trajectory(pose, 60.0, vertical.signed_vertical_rate_mps, 10.0).end_pose
            _append(endpoints, f"chain-{mode}-{sequence}", f"CHAIN_{mode}", f"vertical-{mode}", sequence, pose)

    # Consecutive 15-degree turns, including wrapped 359 -> 0 behaviour.
    for direction, initial_heading in (("RIGHT", 344.0), ("LEFT", 16.0)):
        pose = _base_pose(1500.0, initial_heading)
        turn = envelope.level_turn(pose.z_msl_m, direction).level_turn
        assert turn is not None
        for sequence in range(1, 13):
            pose = build_level_turn_trajectory(pose, turn, 15.0, 10.0).end_pose
            _append(endpoints, f"chain-{direction}-{sequence}", f"CHAIN_{direction}", f"turn-{direction}", sequence, pose)

    # Physical poses deliberately straddling global bin boundaries.  These are
    # passive straight endpoints from continuous, non-grid starts, not snapped
    # synthetic search states.
    for coordinate_offset in (29.9, 30.1, 59.9, 60.1, 89.9, 90.1):
        start = PhysicalPose(500_000.0 + coordinate_offset, 4_200_000.0 + coordinate_offset, 1500.0, 90.0)
        _append(endpoints, f"boundary-{coordinate_offset:g}", "BOUNDARY_STRAIGHT", "boundary", 0,
                build_straight_level_trajectory(start, 0.01, 0.01).end_pose)
    return endpoints


def _span(poses: Sequence[Endpoint]) -> Tuple[float, float, float]:
    max_xy = max_z = max_heading = 0.0
    for index, first in enumerate(poses):
        for second in poses[index + 1:]:
            max_xy = max(max_xy, math.hypot(first.pose.x_m - second.pose.x_m, first.pose.y_m - second.pose.y_m))
            max_z = max(max_z, abs(first.pose.z_msl_m - second.pose.z_msl_m))
            max_heading = max(max_heading, angular_distance_deg(first.pose.heading_deg, second.pose.heading_deg))
    return max_xy, max_z, max_heading


def _dimension_stats(endpoints: Sequence[Endpoint], bins, selector) -> List[dict]:
    report = []
    for width in bins:
        groups = defaultdict(list)
        for endpoint in endpoints:
            groups[selector(endpoint.pose, width)].append(endpoint)
        collided = [group for group in groups.values() if len(group) > 1]
        spans = [_span(group) for group in collided]
        report.append({
            "bin_width": width,
            "unique_bins": len(groups),
            "merge_ratio": 1.0 - len(groups) / len(endpoints),
            "max_same_bin_xy_m": max((span[0] for span in spans), default=0.0),
            "max_same_bin_z_m": max((span[1] for span in spans), default=0.0),
            "max_same_bin_heading_deg": max((span[2] for span in spans), default=0.0),
        })
    return report


def _consecutive_collapse(endpoints: Sequence[Endpoint], selector, chain_prefix: str) -> dict:
    pairs = collapsed = 0
    chains = defaultdict(list)
    for endpoint in endpoints:
        if endpoint.chain.startswith(chain_prefix):
            chains[endpoint.chain].append(endpoint)
    for chain in chains.values():
        chain.sort(key=lambda endpoint: endpoint.sequence)
        for first, second in zip(chain, chain[1:]):
            pairs += 1
            collapsed += selector(first.pose) == selector(second.pose)
    return {"pairs": pairs, "collapsed": collapsed, "rate": collapsed / pairs if pairs else 0.0}


def analyze(endpoints: Sequence[Endpoint]) -> dict:
    xy = _dimension_stats(endpoints, XY_CANDIDATES_M,
                          lambda pose, width: (_bin(pose.x_m, width), _bin(pose.y_m, width)))
    z = _dimension_stats(endpoints, Z_CANDIDATES_M, lambda pose, width: _bin(pose.z_msl_m, width))
    heading = _dimension_stats(endpoints, HEADING_CANDIDATES_DEG,
                               lambda pose, width: _bin(normalize_heading_deg(pose.heading_deg), width))
    combinations = []
    for xy_width in XY_CANDIDATES_M:
        for z_width in Z_CANDIDATES_M:
            for heading_width in HEADING_CANDIDATES_DEG:
                spec = KeySpec(xy_width, z_width, heading_width)
                groups = defaultdict(list)
                for endpoint in endpoints:
                    groups[spec.key(endpoint.pose)].append(endpoint)
                collisions = [group for group in groups.values() if len(group) > 1]
                spans = [_span(group) for group in collisions]
                vertical = _consecutive_collapse(
                    endpoints, lambda pose: _bin(pose.z_msl_m, z_width), "vertical-"
                )
                turns = _consecutive_collapse(
                    endpoints, lambda pose: _bin(normalize_heading_deg(pose.heading_deg), heading_width), "turn-"
                )
                high = 0
                for group in collisions:
                    xy_span, z_span, heading_span = _span(group)
                    consecutive = any(first.chain == second.chain and abs(first.sequence - second.sequence) == 1
                                      for index, first in enumerate(group) for second in group[index + 1:])
                    if consecutive or xy_span > 60.0 or z_span > 5.0 or heading_span > 15.0:
                        high += 1
                high += int(vertical["rate"] > 0.60) + int(turns["rate"] > 0.60)
                high += int(xy_width > 60.0)
                combinations.append({
                    "xy_bin_m": xy_width, "z_bin_m": z_width, "heading_bin_deg": heading_width,
                    "unique_keys": len(groups), "collision_rate": sum(map(len, collisions)) / len(endpoints),
                    "max_same_key_xy_m": max((span[0] for span in spans), default=0.0),
                    "max_same_key_z_m": max((span[1] for span in spans), default=0.0),
                    "max_same_key_heading_deg": max((span[2] for span in spans), default=0.0),
                    "consecutive_vertical_collapse_rate": vertical["rate"],
                    "consecutive_turn_collapse_rate": turns["rate"],
                    "high_risk_groups": high,
                })
    # Conservative screening score: avoid materially diverse collisions and
    # consecutive-progress loss first; only then prefer useful merging. This
    # remains a corpus screen, never a runtime optimizer.
    recommended = min(combinations, key=lambda item: (
        item["high_risk_groups"], item["consecutive_vertical_collapse_rate"],
        item["consecutive_turn_collapse_rate"], -item["collision_rate"],
    ))
    return {
        "corpus_size": len(endpoints),
        "physical_altitudes": len({round(endpoint.pose.z_msl_m, 9) for endpoint in endpoints}),
        "physical_headings": len({round(endpoint.pose.heading_deg, 9) for endpoint in endpoints}),
        "quantization_policy": "floor bins with 1e-9 boundary epsilon; global x/y/z origin = 0; normalized heading in [0,360)",
        "xy_candidates": xy,
        "z_candidates": z,
        "heading_candidates": heading,
        "z_consecutive": {str(width): _consecutive_collapse(endpoints, lambda pose, w=width: _bin(pose.z_msl_m, w), "vertical-") for width in Z_CANDIDATES_M},
        "heading_consecutive": {str(width): _consecutive_collapse(endpoints, lambda pose, w=width: _bin(normalize_heading_deg(pose.heading_deg), w), "turn-") for width in HEADING_CANDIDATES_DEG},
        "combined_keys": combinations,
        "recommended_initial_key": recommended,
        "limitation": "screening corpus only; not A* runtime/completeness evidence",
    }


if __name__ == "__main__":
    print(json.dumps(analyze(build_endpoint_corpus()), indent=2, sort_keys=True))
