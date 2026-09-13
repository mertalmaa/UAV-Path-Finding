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
        (HERE / "u5_2_finalist_general_validation_configuration.yaml").read_text(
            encoding="utf-8"
        )
    )
    reuse = _load("u5_2_reuse_inventory.json")
    runs = _load("u5_2_new_runs.json")
    points = _load("u5_2_new_points.json")
    straight = _load("u5_2_new_straight_coverage.json")
    level_turn = _load("u5_2_new_level_turn_coverage.json")
    maps = _load("u5_2_altitude_speed_maps.json")
    combined = _load("u5_2_combined_3d_validation.json")
    comparison = _load("u5_2_final_comparison.json")
    provenance = _load("u5_2_provenance.json")
    result = _load("u5_2_result.json")
    u5_points = _load("u5_points.json")["points"]
    pid = result["provenance_id"]

    assert list(config["finalists"]) == ["c172p", "DHC6"]
    assert config["finalists"]["c172p"]["nominal_ias_mps"] == 40
    assert config["finalists"]["DHC6"]["nominal_ias_mps"] == 60
    assert config["finalists"]["c172p"]["reference_speed_probes_mps"] == [30, 40, 50, 60]
    assert config["finalists"]["DHC6"]["reference_speed_probes_mps"] == [40, 50, 60, 70]
    assert config["altitude_anchors_m"] == [0, 1000, 2000, 3000, 4000, 5000, 5500, 6000]
    assert config["combined_3d_validation"]["representative_regions"] == {
        "LOW": 1000, "MID": 4000, "HIGH": 5500, "EXTREME_HIGH": 6000
    }
    assert config["combined_3d_validation"]["bank_targets_deg"] == [-20, 20]
    assert config["combined_3d_validation"]["vertical_speed_targets_mps"] == [-2.5, 2.5]
    assert config["selection_philosophy"]["mentor_values_are_hard_requirements"] is False
    assert config["scope"]["finalist_models_only"] is True
    assert all(not value for key, value in config["scope"].items() if key != "finalist_models_only")

    artifacts = [reuse, runs, points, straight, level_turn, maps, combined, comparison, provenance]
    assert all(item["provenance_id"] == pid for item in artifacts)
    assert reuse["u5_total_points_available"] == 189
    assert reuse["u5_total_runs_available"] == 567
    assert reuse["finalist_points_reused"] == result["existing_finalist_points_reused"] == 70
    assert reuse["finalist_runs_reused"] == result["existing_finalist_runs_reused"] == 210
    assert reuse["duplicate_existing_points_executed"] == 0
    assert reuse["reused_maneuver_types"] == ["straight", "turn", "vertical"]

    assert len(points["points"]) == result["new_points_executed"] == 46
    assert len(runs["runs"]) == result["new_cold_start_runs_executed"] == 138
    assert len({row["run_id"] for row in runs["runs"]}) == 138
    assert all(len(point["run_ids"]) == 3 for point in points["points"])
    assert len(straight["rows"]) == result["new_straight_points"] == 6
    assert len(level_turn["rows"]) == result["new_level_turn_points"] == 8
    assert len(combined["rows"]) == result["new_combined_3d_points"] == 32
    assert all(row["repeatability"]["passed"] for row in straight["rows"])
    assert all(row["actual_stable_response"] for row in straight["rows"])
    assert all(row["strict_status"] == "VALID" for row in straight["rows"])

    existing_keys = {
        (
            p["aircraft_model"], p["altitude_msl_m"], p["requested"]["ias_mps"],
            p["maneuver"], p["requested"]["bank_deg"], p["requested"]["vertical_speed_mps"],
        )
        for p in u5_points
    }
    new_noncombined_keys = {
        (
            p["aircraft_model"], p["altitude_msl_m"], p["requested"]["ias_mps"],
            p["maneuver"], p["requested"]["bank_deg"], p["requested"]["vertical_speed_mps"],
        )
        for p in points["points"] if p["maneuver"] != "combined"
    }
    assert existing_keys.isdisjoint(new_noncombined_keys)

    map_by_model = {row["aircraft"]: row for row in maps["aircraft"]}
    assert set(map_by_model) == {"c172p", "DHC6"}
    for model, nominal in (("c172p", 40.0), ("DHC6", 60.0)):
        matrix = map_by_model[model]["matrix"]
        assert [row["altitude_m"] for row in matrix] == [0.0, 1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 5500.0, 6000.0]
        nominal_cells = [
            next(cell for cell in row["cells"] if cell["ias_mps"] == nominal)
            for row in matrix
        ]
        assert all(cell["actual_stable_response"] for cell in nominal_cells)
        assert all(cell["raw_strict_status"] == "VALID" for cell in nominal_cells)
    dhc6_6000 = next(
        cell for row in map_by_model["DHC6"]["matrix"] if row["altitude_m"] == 6000.0
        for cell in row["cells"] if cell["ias_mps"] == 60.0
    )
    assert dhc6_6000["usability"] == "MARGINAL"
    assert all(
        next(cell for cell in row["cells"] if cell["ias_mps"] == 70.0)["usability"] == "NOT_TESTED"
        for row in map_by_model["DHC6"]["matrix"]
    )

    combined_by_model = {
        model: [row for row in combined["rows"] if row["aircraft"] == model]
        for model in ("c172p", "DHC6")
    }
    for model, rows_for_model in combined_by_model.items():
        assert len(rows_for_model) == 16
        assert {row["representative_region"] for row in rows_for_model} == {
            "LOW", "MID", "HIGH", "EXTREME_HIGH"
        }
        assert all(row["repeatability"]["passed"] for row in rows_for_model)
        assert all(row["measured_turn_radius_m"] > 0.0 for row in rows_for_model)
        assert all(row["level_turn_radius_m"] > 0.0 for row in rows_for_model)
        assert all(row["combined_to_level_radius_ratio"] > 0.0 for row in rows_for_model)
        assert all(len(row["run_ids"]) == 3 for row in rows_for_model)

    rows = {row["aircraft"]: row for row in comparison["rows"]}
    assert rows["c172p"]["nominal_straight_all_altitude_anchors_actual_stable"] is True
    assert rows["DHC6"]["nominal_straight_all_altitude_anchors_actual_stable"] is True
    assert rows["c172p"]["combined_climbing_turn_actual_stable_count"] == 3
    assert rows["c172p"]["combined_descending_turn_actual_stable_count"] == 8
    assert rows["DHC6"]["combined_climbing_turn_actual_stable_count"] == 5
    assert rows["DHC6"]["combined_descending_turn_actual_stable_count"] == 7
    assert rows["c172p"]["combined_all_direction_actual_stable_altitudes_m"] == [1000]
    assert rows["DHC6"]["combined_all_direction_actual_stable_altitudes_m"] == [4000]
    assert rows["c172p"]["combined_all_direction_strict_usable_altitudes_m"] == []
    assert rows["DHC6"]["combined_all_direction_strict_usable_altitudes_m"] == []
    assert comparison["primary_aircraft"] == "c172p"
    assert comparison["backup_aircraft"] == "DHC6"
    assert comparison["nominal_ias_context_mps"] == 40.0
    assert comparison["tested_separate_maneuver_usable_altitude_region_m"] == [5000.0, 5500.0]
    assert comparison["combined_3d_planner_safe_altitude_region_m"] == []
    assert comparison["aircraft_selection_finally_closed"] is True

    assert provenance["source_hashes_before"] == provenance["source_hashes_after"]
    assert provenance["source_artifacts_unchanged"] is True
    assert _sha256(Path(provenance["configuration"]["path"])) == provenance["configuration"]["sha256"]
    assert _sha256(Path(provenance["harness"]["path"])) == provenance["harness"]["sha256"]
    for name, relative in config["source_artifacts"].items():
        assert _sha256(HERE / relative) == provenance["source_hashes_after"][name]

    assert result["step_status"] == "PASS"
    assert result["primary_aircraft"] == "c172p"
    assert result["backup_aircraft"] == "DHC6"
    assert result["tested_separate_maneuver_usable_altitude_region_m"] == [5000.0, 5500.0]
    assert result["combined_3d_planner_safe_altitude_region_m"] == []
    assert result["aircraft_selection_finally_closed"] is True
    assert result["ready_for_selected_aircraft_lut"] is True
    assert result["source_artifacts_unchanged"] is True
    for forbidden in (
        "full_u5_rerun_performed", "new_aircraft_added", "full_lut_generated",
        "controller_tuning_performed", "aircraft_xml_modified",
        "planner_or_search_modified", "next_stage_started",
    ):
        assert result[forbidden] is False
    print("U5.2 finalist general validation and final freeze checks passed")


if __name__ == "__main__":
    main()
