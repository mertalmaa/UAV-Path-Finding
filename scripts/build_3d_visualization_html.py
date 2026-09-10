"""Assembles the standalone 3D visualization artifact HTML by embedding
scratch_3d_viz_data.json directly into a Plotly.js page. One-off tooling,
not part of any numbered stage."""
import json

DATA_JSON = "scratch_3d_viz_data.json"
TEMPLATE_PATH = "scripts/_viz_template.html"
OUT_PATH = "scratch_3d_viz_artifact.html"

STATS = {
    "start_row": 48, "start_col": 276, "goal_row": 264, "goal_col": 276,
    "aircraft_msl": 3760.0,
    "corridor_xy_m": 300, "corridor_z_m": 200,
    "corridor_cells": 4853,
    "path_nodes": 89, "path_cost": 1.483210,
    "path_length_3d": 6803.7, "path_min_msl": 3320.0, "path_max_msl": 3760.0,
    "min_agl": 200.76, "max_angle": 9.46,
}


def main():
    with open(DATA_JSON) as f:
        data = json.load(f)
    with open(TEMPLATE_PATH, encoding="utf-8") as f:
        template = f.read()

    html = template.replace("__VIZ_DATA__", json.dumps(data))
    html = html.replace("__STATS__", json.dumps(STATS))

    with open(OUT_PATH, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"written {OUT_PATH} ({len(html)/1024:.1f} KB)")


if __name__ == "__main__":
    main()
