from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


class CapabilityProfileQueryError(ValueError):
    pass


@dataclass(frozen=True)
class CapabilityQueryResult:
    availability: str
    altitude_m: float
    row: dict[str, Any] | None
    interpolation: str


class AltitudeAwareAircraftCapabilityProfile:
    """Aircraft-neutral reader for the altitude-aware planner-safe schema."""

    def __init__(self, payload: dict[str, Any]) -> None:
        if payload.get("profile_schema_id") != "altitude_aware_aircraft_capability_profile_v2":
            raise CapabilityProfileQueryError("unsupported altitude-aware profile schema")
        self.payload = payload
        self.minimum = float(payload["domain"]["min_altitude_m"])
        self.maximum = float(payload["domain"]["max_altitude_m"])

    @classmethod
    def load(cls, path: str | Path) -> "AltitudeAwareAircraftCapabilityProfile":
        return cls(json.loads(Path(path).read_text(encoding="utf-8")))

    def _query(self, family: str, altitude_m: float, **context: Any) -> CapabilityQueryResult:
        altitude = float(altitude_m)
        if altitude < self.minimum or altitude > self.maximum:
            return CapabilityQueryResult("OUT_OF_DOMAIN", altitude, None, "NONE")
        capability = self.payload["capabilities"][family]
        rows = [
            row for row in capability["rows"]
            if all(row.get(key) == value for key, value in context.items())
        ]
        if not rows:
            raise CapabilityProfileQueryError(f"unsupported {family} context: {context}")
        exact = next((row for row in rows if float(row["altitude_m"]) == altitude), None)
        if exact is not None:
            return CapabilityQueryResult(exact["availability"], altitude, exact, "EXACT_ANCHOR")
        lower = max((row for row in rows if row["altitude_m"] < altitude), key=lambda row: row["altitude_m"], default=None)
        upper = min((row for row in rows if row["altitude_m"] > altitude), key=lambda row: row["altitude_m"], default=None)
        if lower is None or upper is None:
            return CapabilityQueryResult("UNAVAILABLE", altitude, None, "NO_BRACKET")
        policy = capability["query_policy"]
        measured = self._interpolate_mapping(lower["measured"], upper["measured"], altitude, lower["altitude_m"], upper["altitude_m"])
        availability = "AVAILABLE" if lower["availability"] == upper["availability"] == "AVAILABLE" else "UNAVAILABLE"
        if any(start <= altitude <= end for start, end in policy.get("availability_interpolation_blocked_ranges_m", [])):
            availability = "UNAVAILABLE"
        safe = None
        if availability == "AVAILABLE":
            if policy["safe_interpolation"] == "LINEAR":
                safe = self._interpolate_mapping(lower["planner_safe"], upper["planner_safe"], altitude, lower["altitude_m"], upper["altitude_m"])
            elif policy["safe_interpolation"] == "CONSERVATIVE_ENDPOINT":
                safe = self._conservative_mapping(lower["planner_safe"], upper["planner_safe"])
            else:
                availability = "UNAVAILABLE"
        row = {
            "altitude_m": altitude,
            "availability": availability,
            "measured": measured,
            "planner_safe": safe,
            "evidence": {
                "derived_from_altitude_m": [lower["altitude_m"], upper["altitude_m"]],
                "policy": policy,
                "canonical_anchor": False,
            },
        }
        row.update(context)
        return CapabilityQueryResult(availability, altitude, row, policy["safe_interpolation"])

    @classmethod
    def _interpolate_mapping(
        cls, lower: dict[str, Any] | None, upper: dict[str, Any] | None,
        altitude: float, lower_altitude: float, upper_altitude: float,
    ) -> dict[str, Any] | None:
        if lower is None or upper is None:
            return None
        weight = (altitude - lower_altitude) / (upper_altitude - lower_altitude)
        result: dict[str, Any] = {}
        for key in lower.keys() & upper.keys():
            lo, hi = lower[key], upper[key]
            if isinstance(lo, bool) or isinstance(hi, bool):
                result[key] = lo if lo == hi else None
            elif isinstance(lo, (int, float)) and isinstance(hi, (int, float)):
                result[key] = float(lo) + weight * (float(hi) - float(lo))
            elif isinstance(lo, dict) and isinstance(hi, dict):
                result[key] = cls._interpolate_mapping(lo, hi, altitude, lower_altitude, upper_altitude)
            elif lo == hi:
                result[key] = lo
            else:
                result[key] = None
        return result

    @classmethod
    def _conservative_mapping(
        cls, lower: dict[str, Any] | None, upper: dict[str, Any] | None,
    ) -> dict[str, Any] | None:
        if lower is None or upper is None:
            return None
        result: dict[str, Any] = {}
        for key in lower.keys() & upper.keys():
            lo, hi = lower[key], upper[key]
            if isinstance(lo, bool) or isinstance(hi, bool):
                result[key] = lo and hi
            elif isinstance(lo, (int, float)) and isinstance(hi, (int, float)):
                if "radius" in key:
                    result[key] = max(float(lo), float(hi))
                elif "rate" in key or "vz" in key:
                    result[key] = min((float(lo), float(hi)), key=abs)
                else:
                    result[key] = min(float(lo), float(hi))
            elif isinstance(lo, dict) and isinstance(hi, dict):
                result[key] = cls._conservative_mapping(lo, hi)
            elif lo == hi:
                result[key] = lo
            else:
                result[key] = None
        return result

    def straight_query(self, altitude_m: float) -> CapabilityQueryResult:
        return self._query("straight", altitude_m)

    def turn_query(self, altitude_m: float, direction: str, bank_deg: float) -> CapabilityQueryResult:
        return self._query("level_turn", altitude_m, direction=direction.upper(), command_bank_deg=float(bank_deg))

    def vertical_query(self, altitude_m: float, mode: str) -> CapabilityQueryResult:
        family = "straight_climb" if mode.upper() == "CLIMB" else "straight_descent" if mode.upper() == "DESCENT" else None
        if family is None:
            raise CapabilityProfileQueryError(f"unsupported vertical mode: {mode}")
        return self._query(family, altitude_m)

    def combined_query(self, altitude_m: float, direction: str, mode: str) -> CapabilityQueryResult:
        family = "climbing_turn" if mode.upper() == "CLIMB" else "descending_turn" if mode.upper() == "DESCENT" else None
        if family is None:
            raise CapabilityProfileQueryError(f"unsupported combined mode: {mode}")
        return self._query(family, altitude_m, direction=direction.upper())
