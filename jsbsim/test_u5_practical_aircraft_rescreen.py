from __future__ import annotations

import hashlib
import json
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"


def _load(name: str):
    return json.loads((RESULTS / name).read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    config = yaml.safe_load(
        (HERE / "u5_practical_aircraft_rescreen_configuration.yaml").read_text(
            encoding="utf-8"
        )
    )
    phase1 = _load("u5_phase1_speed_screen.json")
    high = _load("u5_high_altitude_screen.json")
    comparison = _load("u5_candidate_comparison.json")
    freeze = _load("u5_aircraft_freeze.json")
    provenance = _load("u5_provenance.json")
    points = _load("u5_points.json")
    runs = _load("u5_runs.json")
    result = _load("u5_result.json")
    pid = result["provenance_id"]

    required = {"c172r", "c172p", "c172x", "c182", "DHC6", "J3Cub"}
    assert required <= set(config["candidates"])
    assert set(config["candidates"]) == required | {"pa28", "c310"}
    assert config["phase1"]["altitudes_m"] == [0, 3000]
    assert config["phase1"]["initial_speed_probes_mps"] == [30, 40, 50, 60]
    assert config["phase1"]["maximum_selected_speeds_per_aircraft"] == 1
    assert config["phase2"]["altitudes_m"] == [5000, 5500, 6000]
    assert config["phase2"]["turn_bank_targets_deg"] == [-20, 20, -30, 30]
    assert config["phase2"]["vertical_speed_targets_mps"] == [-4.5, -2.5, 2.5, 4.5]
    assert config["selection_philosophy"]["mentor_values_are_hard_gates"] is False
    assert config["actual_stable_response_contract"]["strict_u1_status_is_only_one_input"] is True
    assert config["actual_stable_response_contract"]["unknown_is_infeasible"] is False
    for forbidden in (
        "full_lut_generation",
        "boundary_refinement",
        "maximum_bank_search",
        "service_ceiling_research",
        "controller_gain_tuning",
        "stock_xml_modification",
        "planner_modification",
        "interpolation",
        "derating",
        "heading_addition",
        "final_replay_implementation",
    ):
        assert config["scope"][forbidden] is False

    assert pid == phase1["provenance_id"] == high["provenance_id"]
    assert pid == comparison["provenance_id"] == freeze["provenance_id"]
    assert pid == provenance["provenance_id"] == points["provenance_id"]
    assert pid == runs["provenance_id"]
    for item in provenance["files"].values():
        path = Path(item["path"])
        assert path.is_file()
        assert _sha256(path) == item["sha256"]
    expected_historical = {
        name: item["sha256"] for name, item in config["historical_artifacts"].items()
    }
    assert provenance["historical_artifact_hashes_before"] == expected_historical
    assert provenance["historical_artifact_hashes_after"] == expected_historical
    assert provenance["historical_artifacts_unchanged"] is True

    choices = {row["model"]: row for row in phase1["speed_choices"]}
    assert len(phase1["rows"]) == 64
    assert {name: row["selected_speed_mps"] for name, row in choices.items()} == {
        "c172r": 40.0,
        "c172p": 40.0,
        "c172x": 30.0,
        "c182": 40.0,
        "DHC6": 60.0,
        "J3Cub": 30.0,
        "pa28": 50.0,
        "c310": None,
    }
    assert [name for name, row in choices.items() if row["early_rejected"]] == ["c310"]
    assert choices["DHC6"]["selected_speed_mps"] > 50.0
    assert choices["c172x"]["selected_speed_mps"] < 35.0
    assert choices["J3Cub"]["selected_speed_mps"] < 35.0

    ranking = comparison["ranking"]
    assert [row["model"] for row in ranking] == [
        "c172p", "DHC6", "c172r", "c172x", "c182", "J3Cub", "pa28", "c310"
    ]
    assert ranking[0]["selection"] == "PRIMARY"
    assert ranking[1]["selection"] == "BACKUP"
    usable = {
        row["model"]: [item["altitude_m"] for item in row["high_altitude"] if item["usable"]]
        for row in ranking
    }
    assert usable["c172p"] == [5000.0, 5500.0]
    assert usable["DHC6"] == [5000.0, 5500.0]
    assert usable["c172r"] == []
    for model in ("c172p", "DHC6"):
        at_6000 = next(x for x in next(r for r in ranking if r["model"] == model)["high_altitude"] if x["altitude_m"] == 6000.0)
        assert at_6000["usable"] is False
        assert at_6000["requirements"]["climb"] is False

    c172p_5000 = next(
        row for row in high["rows"]
        if row["model"] == "c172p"
        and row["altitude_m"] == 5000.0
        and row["maneuver"] == "vertical"
        and row["target_value"] == 2.5
    )
    assert c172p_5000["strict_status"] == "INFEASIBLE"
    assert c172p_5000["actual_stable_response"] is True

    assert len(points["points"]) == result["executed_points"] == 189
    assert len(runs["runs"]) == result["executed_cold_start_runs"] == 567
    assert len({row["run_id"] for row in runs["runs"]}) == 567
    assert all(len(point["run_ids"]) == 3 for point in points["points"])

    assert freeze["primary_aircraft"] == result["primary_aircraft"] == "c172p"
    assert freeze["backup_aircraft"] == result["backup_aircraft"] == "DHC6"
    assert result["selected_nominal_ias_mps"] == 40.0
    assert result["primary_high_altitude_usable_region_m"] == [5000.0, 5500.0]
    stack = freeze["selected_stack"]
    assert stack["aircraft"] == "c172p"
    assert stack["mixture_policy_id"] == "c172p-production-mixture-pressure-ratio-v1"
    assert stack["mixture_formula"] == "clip(atmosphere/P-psf / 2117.0, 0.0, 1.0)"
    assert stack["mixture_update_cadence"] == "after engine start and before every simulation frame"
    assert stack["gain_tuning_performed"] is False

    assert result["step_status"] == "PASS"
    assert result["aircraft_selection_frozen"] is True
    assert result["ready_to_return_to_path_planner_work"] is True
    assert result["historical_artifacts_unchanged"] is True
    for forbidden in (
        "full_lut_generated",
        "controller_gain_tuning_performed",
        "stock_xml_modified",
        "planner_modified",
        "interpolation_performed",
        "derating_performed",
        "heading_added",
        "next_stage_started",
    ):
        assert result[forbidden] is False
    print("U5 practical aircraft re-screen and final freeze checks passed")


if __name__ == "__main__":
    main()
