"""Frozen production mixture policy shared by LUT generation and final replay."""

from __future__ import annotations

from typing import Any


POLICY_ID = "c172r-production-mixture-pressure-ratio-v1"
PRESSURE_PROPERTY = "atmosphere/P-psf"
REFERENCE_PRESSURE_PSF = 2117.0
COMMAND_PROPERTY = "fcs/mixture-cmd-norm"
POSITION_PROPERTY = "fcs/mixture-pos-norm"
LOWER_LIMIT_NORM = 0.0
UPPER_LIMIT_NORM = 1.0


def command_from_pressure_psf(pressure_psf: float) -> float:
    """Return the deterministic normalized mixture command for ambient pressure."""
    raw = float(pressure_psf) / REFERENCE_PRESSURE_PSF
    return min(UPPER_LIMIT_NORM, max(LOWER_LIMIT_NORM, raw))


def command_for_fdm(fdm: Any) -> float:
    """Evaluate the stateless policy from the current JSBSim environment state."""
    return command_from_pressure_psf(float(fdm[PRESSURE_PROPERTY]))


def apply(fdm: Any, *, set_position: bool = True) -> float:
    """Apply one policy update and return the command that was written.

    The c172r has no mixture actuator model in its stock aircraft XML.  The
    production contract therefore writes both command and position at engine
    initialization and on every simulation frame.  Final replay must use this
    same function and cadence.
    """
    command = command_for_fdm(fdm)
    fdm[COMMAND_PROPERTY] = command
    if set_position:
        fdm[POSITION_PROPERTY] = command
    return command


def metadata() -> dict[str, object]:
    return {
        "policy_id": POLICY_ID,
        "formula": "clip(atmosphere/P-psf / 2117.0, 0.0, 1.0)",
        "input_property": PRESSURE_PROPERTY,
        "input_quantity": "ambient static pressure",
        "input_units": "psf",
        "reference_pressure_psf": REFERENCE_PRESSURE_PSF,
        "output_units": "normalized",
        "clip_lower_norm": LOWER_LIMIT_NORM,
        "clip_upper_norm": UPPER_LIMIT_NORM,
        "command_property": COMMAND_PROPERTY,
        "position_property": POSITION_PROPERTY,
        "update_cadence": "after engine-start initialization and before every simulation frame",
        "stateful": False,
        "altitude_lookup_table_used": False,
        "history_dependent": False,
        "required_consumers": ["raw_lut_generation", "final_replay"],
    }
