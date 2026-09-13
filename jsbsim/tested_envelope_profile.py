from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from aircraft_capability_profile import AltitudeAwareAircraftCapabilityProfile, CapabilityProfileQueryError


class TestedEnvelopeAircraftProfile(AltitudeAwareAircraftCapabilityProfile):
    """Aircraft-neutral query interface for tested-envelope schema v3."""

    def __init__(self, payload: dict[str, Any]) -> None:
        if payload.get("profile_schema_id") != "tested_safe_envelope_aircraft_capability_profile_v3":
            raise CapabilityProfileQueryError("unsupported tested-envelope profile schema")
        self.payload = payload
        self.minimum = float(payload["domain"]["min_altitude_m"])
        self.maximum = float(payload["domain"]["max_altitude_m"])

    @classmethod
    def load(cls, path: str | Path) -> "TestedEnvelopeAircraftProfile":
        return cls(json.loads(Path(path).read_text(encoding="utf-8")))
