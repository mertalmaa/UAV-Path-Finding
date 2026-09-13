from __future__ import annotations

import hashlib
import json
import math
import sys
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
sys.path.insert(0, str(HERE))

from aircraft_capability_profile import AltitudeAwareAircraftCapabilityProfile


def _load(name: str):
    return json.loads((RESULTS / name).read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    config = yaml.safe_load((HERE / "u6_2_1_altitude_profile_configuration.yaml").read_text(encoding="utf-8"))
    profile = _load("c172p_aircraft_profile_planner_safe_v2.json")
    audit = _load("u6_2_1_global_constant_audit.json")
    altitude_audit = _load("u6_2_1_altitude_audit.json")
    provenance = _load("u6_2_1_provenance.json")
    result = _load("u6_2_1_result.json")
    altitudes = [float(value) for value in range(0, 5501, 500)]

    assert profile["profile_schema_id"] == "altitude_aware_aircraft_capability_profile_v2"
    assert profile["domain"]["canonical_altitude_grid_m"] == altitudes
    assert profile["global_aircraft_capability_constants"] == []
    assert profile["measured_and_planner_safe_layers_separate"] is True
    assert profile["interface_contract"]["aircraft_specific_planner_branching_forbidden"] is True
    assert profile["production_planner_integrated"] is False
    assert set(profile["capabilities"]) == {
        "straight", "level_turn", "straight_climb", "straight_descent", "climbing_turn", "descending_turn"
    }

    straight = profile["capabilities"]["straight"]["rows"]
    assert len(straight) == 12 and {row["altitude_m"] for row in straight} == set(altitudes)
    for row in straight:
        assert row["measured"]["actual_ias_mps"]
        assert row["measured"]["tas_mps"]
        assert row["measured"]["power"]["rpm"]
        assert "pitch_deg" in row["measured"] and "aoa_deg" in row["measured"] and "beta_deg" in row["measured"]
        assert row["measured"]["quality"]["repeatable"] is True
        assert row["planner_safe"] is not row["measured"]
    assert len({round(row["measured"]["power"]["throttle"], 6) for row in straight}) == 12

    turns = profile["capabilities"]["level_turn"]["rows"]
    assert len(turns) == 48
    for altitude in altitudes:
        local = [row for row in turns if row["altitude_m"] == altitude]
        assert {(row["direction"], row["command_bank_deg"]) for row in local} == {
            ("LEFT", -20.0), ("RIGHT", 20.0), ("LEFT", -30.0), ("RIGHT", 30.0)
        }
        left = next(row for row in local if row["command_bank_deg"] == -20.0)
        right = next(row for row in local if row["command_bank_deg"] == 20.0)
        assert left["planner_safe"]["turn_radius_m"] >= left["measured"]["turn_radius_m"]
        assert right["planner_safe"]["turn_radius_m"] >= right["measured"]["turn_radius_m"]
        assert not math.isclose(left["planner_safe"]["turn_radius_m"], right["planner_safe"]["turn_radius_m"])
        for row in local:
            assert row["measured"]["actual_ias_mps"] and row["measured"]["power"]["throttle"] is not None
        for row in local:
            if abs(row["command_bank_deg"]) == 30.0:
                assert row["availability"] == "UNAVAILABLE" and row["planner_safe"] is None
                assert row["measured"]["turn_radius_m"] is not None

    climbs = profile["capabilities"]["straight_climb"]["rows"]
    descents = profile["capabilities"]["straight_descent"]["rows"]
    assert len(climbs) == len(descents) == 12
    assert len({round(row["measured"]["maximum_observed_stable_climb_vz_mps"], 6) for row in climbs}) > 6
    assert len({round(row["planner_safe"]["climb_vz_mps"], 6) for row in climbs if row["planner_safe"]}) > 6
    assert len({round(row["measured"]["maximum_observed_stable_descent_vz_mps"], 6) for row in descents}) > 6
    assert len({round(row["planner_safe"]["descent_vz_mps"], 6) for row in descents}) > 6
    at_4000 = next(row for row in climbs if row["altitude_m"] == 4000.0)
    assert 3.15 < at_4000["measured"]["maximum_observed_stable_climb_vz_mps"] < 3.17
    at_5000 = next(row for row in climbs if row["altitude_m"] == 5000.0)
    assert 2.85 < at_5000["measured"]["maximum_observed_stable_climb_vz_mps"] < 2.87
    assert at_5000["availability"] == "UNAVAILABLE" and at_5000["planner_safe"] is None
    at_3000_descent = next(row for row in descents if row["altitude_m"] == 3000.0)
    assert -4.23 < at_3000_descent["measured"]["maximum_observed_stable_descent_vz_mps"] < -4.21
    assert abs(at_3000_descent["planner_safe"]["descent_vz_mps"]) < abs(at_3000_descent["measured"]["safe_basis_actual_vz_mps"])

    combined_climb = profile["capabilities"]["climbing_turn"]["rows"]
    combined_descent = profile["capabilities"]["descending_turn"]["rows"]
    assert len(combined_climb) == len(combined_descent) == 24
    assert all(row["availability"] == "UNAVAILABLE" and row["planner_safe"] is None for row in combined_climb)
    assert all(row["measured"] is not None for row in combined_climb if row["altitude_m"] >= 1000.0)
    assert all(row["measured"] is not None for row in combined_descent if row["altitude_m"] >= 1000.0)
    assert all(row["planner_safe"] is not None for row in combined_descent if row["altitude_m"] >= 3500.0)
    assert len({round(row["planner_safe"]["turn_radius_m"], 5) for row in combined_descent if row["planner_safe"]}) == 10
    corrected = next(row for row in combined_climb if row["altitude_m"] == 1000.0 and row["direction"] == "LEFT")
    assert math.isclose(corrected["measured"]["turn_radius_m"], 427.36125497816295, abs_tol=1e-9)
    assert math.isclose(corrected["measured"]["measurement_correction"]["historical_raw_turn_radius_m"], 194.8156457998017, abs_tol=1e-9)

    assert len(altitude_audit["rows"]) == 12
    assert all("combined_descent_safe" in row and "main_limitation" in row for row in altitude_audit["rows"])
    invalid = [item for item in audit["findings"] if item["classification"] == "INVALID_GLOBAL_SIMPLIFICATION"]
    assert {item["item"] for item in invalid} == {"climb rate", "descent rate"}
    assert audit["remaining_global_aircraft_capability_constants"] == []

    query = AltitudeAwareAircraftCapabilityProfile.load(RESULTS / "c172p_aircraft_profile_planner_safe_v2.json")
    assert query.straight_query(275.0).availability == "AVAILABLE"
    assert query.turn_query(275.0, "LEFT", -20.0).availability == "AVAILABLE"
    assert query.turn_query(275.0, "RIGHT", 30.0).availability == "UNAVAILABLE"
    assert query.vertical_query(3750.0, "CLIMB").availability == "AVAILABLE"
    high_climb = query.vertical_query(4250.0, "CLIMB")
    assert high_climb.availability == "UNAVAILABLE" and high_climb.row["measured"] is not None and high_climb.row["planner_safe"] is None
    assert query.vertical_query(275.0, "DESCENT").availability == "AVAILABLE"
    assert query.combined_query(3750.0, "LEFT", "DESCENT").availability == "AVAILABLE"
    assert query.combined_query(2750.0, "LEFT", "DESCENT").availability == "UNAVAILABLE"
    assert query.combined_query(3750.0, "RIGHT", "CLIMB").availability == "UNAVAILABLE"
    assert query.straight_query(6000.0).availability == "OUT_OF_DOMAIN"

    assert provenance["source_artifacts_unchanged"] is True
    assert provenance["source_hashes_before"] == provenance["source_hashes_after"]
    for name, source in config["source_artifacts"].items():
        assert _sha256(HERE / source["path"]) == source["sha256"] == provenance["source_hashes_after"][name]
    for item in provenance["files"].values():
        assert _sha256(Path(item["path"])) == item["sha256"]

    assert result["step_status"] == "PASS"
    assert result["canonical_altitude_rows"] == 12
    assert result["new_jsbsim_runs"] == 0
    assert result["full_altitude_dependent_profile"] is True
    assert result["global_aircraft_capability_constants_removed"] is True
    assert result["measured_and_safe_values_preserved"] is True
    assert result["aircraft_swappable"] is True
    assert result["ready_for_aircraftprofile_integration"] is True
    assert result["production_planner_integrated"] is result["next_stage_started"] is False

    print("U6.2.1 full altitude-dependent aircraft capability profile checks passed")


if __name__ == "__main__":
    main()
