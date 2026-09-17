"""Mission presets taken verbatim from the repository's mission scripts.

Coordinates are the scripts' UTM (EPSG:32636) values; altitude offsets are the
scripts' AGL offsets above the planner's buffered terrain.
"""
PRESETS = [
    {"id": "M05_canyon_4km", "label": "Short · Deep canyon corridor (4 km)", "region_id": "bilecik",
     "source": "scripts/bilecik_missions_spec.py M05", "start_xy": (258000.0, 4450000.0),
     "goal_xy": (258000.0, 4454000.0), "alt_agl_m": 130.0, "heading_deg": 0.0},
    {"id": "M19_ridge_7km", "label": "Short · Double ridge hop (7 km)", "region_id": "bilecik",
     "source": "scripts/bilecik_missions_spec.py M19", "start_xy": (265000.0, 4455000.0),
     "goal_xy": (272000.0, 4455000.0), "alt_agl_m": 150.0, "heading_deg": 90.0},
    {"id": "canyon_31km", "label": "~30 km · Sakarya canyon single-shot (26 km direct)", "region_id": "bilecik",
     "source": "scripts/run_single_shot_31km.py", "start_xy": (261800.0, 4445000.0),
     "goal_xy": (262800.0, 4471200.0), "alt_agl_m": 130.0, "heading_deg": 319.09},
    {"id": "M90_01", "label": "90 km · M90_01 Sakarya corridor S→N", "region_id": "bilecik",
     "source": "scripts/run_bilecik_90km_5_missions.py", "start_xy": (261000.0, 4419000.0),
     "goal_xy": (255000.0, 4499500.0), "alt_agl_m": 150.0, "heading_deg": None},
    {"id": "M90_02", "label": "90 km · M90_02 Eastern ridges S→N", "region_id": "bilecik",
     "source": "scripts/run_bilecik_90km_5_missions.py", "start_xy": (300000.0, 4420000.0),
     "goal_xy": (292000.0, 4499000.0), "alt_agl_m": 150.0, "heading_deg": None},
    {"id": "M90_03", "label": "90 km · M90_03 West→East ridge crossing", "region_id": "bilecik",
     "source": "scripts/run_bilecik_90km_5_missions.py", "start_xy": (230500.0, 4463000.0),
     "goal_xy": (309500.0, 4455000.0), "alt_agl_m": 150.0, "heading_deg": None},
    {"id": "M90_04", "label": "90 km · M90_04 SW→NE diagonal", "region_id": "bilecik",
     "source": "scripts/run_bilecik_90km_5_missions.py", "start_xy": (231000.0, 4421000.0),
     "goal_xy": (289000.0, 4479000.0), "alt_agl_m": 150.0, "heading_deg": None},
    {"id": "M90_05", "label": "90 km · M90_05 NW→SE diagonal", "region_id": "bilecik",
     "source": "scripts/run_bilecik_90km_5_missions.py", "start_xy": (233000.0, 4497000.0),
     "goal_xy": (291000.0, 4439000.0), "alt_agl_m": 150.0, "heading_deg": None},
]
