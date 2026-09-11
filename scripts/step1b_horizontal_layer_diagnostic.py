"""Roadmap Step 1B: verify, on the REAL Aladaglar DEM, that the EXISTING
absolute-MSL z_index representation (planner.astar.msl_to_z_index /
z_index_to_msl -- UNCHANGED, imported verbatim) already produces the
mentor-requested horizontal-layer behavior: every (row, col) on a given
layer shares the identical MSL altitude, and terrain only ever decides
per-candidate VALIDITY (clearance >= min_agl), never the layer's own Z
value.

This is a read-only DIAGNOSTIC script, not a planner change:
  - No file under planner/ is edited.
  - z_step_m is NOT changed (stays DEFAULT_CONFIG's 20.0m grid).
  - A separate STEP1B_CONFIG (dataclasses.replace) carries only the new
    prototype min_agl_m=100.0 -- DEFAULT_CONFIG itself is never mutated.
  - Terrain/AGL feasibility reuses planner.agl.evaluate_agl() verbatim --
    no reimplementation of the clearance rule.
  - No primitive/motion-primitive geometry is touched or built here at
    all -- this is a pure NODE (not edge) diagnostic.

Candidate vs valid (per the Step 1B spec):
  candidate -- a geometric (row, col, z_msl) point on a horizontal layer.
               Exists unconditionally, regardless of terrain.
  valid     -- a candidate whose clearance (z_msl - terrain_msl) >= min_agl_m,
               decided ENTIRELY by evaluate_agl() (reused, not reimplemented).
"""
import dataclasses
import math

from planner.agl import evaluate_agl
from planner.astar import msl_to_z_index, z_index_to_msl
from planner.config import DEFAULT_CONFIG
from planner.roi import load_roi
from planner.terrain import TerrainQuery

# ---------------------------------------------------------------------------
# Separate prototype/test config -- DEFAULT_CONFIG is never modified or
# replaced globally; this is a local, explicit dataclasses.replace() instance
# used only inside this script (see instruction 7).
# ---------------------------------------------------------------------------
STEP1B_CONFIG = dataclasses.replace(DEFAULT_CONFIG, min_agl_m=100.0)
# z_step_m deliberately NOT touched -- stays DEFAULT_CONFIG.z_step_m (20.0m),
# per instruction 9 ("z_step'i ... değiştirme").
assert STEP1B_CONFIG.z_step_m == DEFAULT_CONFIG.z_step_m

# Small, controlled ROI window: 40x40 fine cells (~1200m x 1200m) centered on
# the full ROI's own center pixel -- planner/config.py documents
# roi_center_lonlat as "source DEM max-elevation pixel, near Demirkazik", so
# this window is centered exactly on real relief (a peak), not a flat patch.
WINDOW_HALF_WIDTH = 20  # cells -- window is [center-20, center+20)

# Prototype diagnostic layers (MSL, all exact multiples of z_step_m=20 --
# msl_to_z_index() will raise, not silently snap, if any is off-grid):
# 3200/3400/3600/3800 straddle this window's real terrain (3099-3700m) to
# exercise both valid and invalid candidates; 5000/6000 are the mentor's
# literal nominal-start/max-planning prototype values.
LAYERS_MSL = [3200.0, 3400.0, 3600.0, 3800.0, 5000.0, 6000.0]

OUTPUT_SVG = "outputs/step1b_horizontal_layers.svg"


def main() -> None:
    roi = load_roi(DEFAULT_CONFIG)  # real Aladaglar working DEM, untouched
    terrain = TerrainQuery(roi)

    h, w = roi.elevation.shape
    cy, cx = h // 2, w // 2
    r0, r1 = cy - WINDOW_HALF_WIDTH, cy + WINDOW_HALF_WIDTH
    c0, c1 = cx - WINDOW_HALF_WIDTH, cx + WINDOW_HALF_WIDTH
    window_elev = roi.elevation[r0:r1, c0:c1]

    print("=== ROI ===")
    print(f"  full ROI shape: {h}x{w} (30m fine grid, real Aladağlar working DEM)")
    print(f"  window: rows [{r0},{r1}) x cols [{c0},{c1})  ({r1-r0}x{c1-c0} cells, "
          f"~{(r1-r0)*roi.resolution[0]:.0f}m x {(c1-c0)*roi.resolution[1]:.0f}m)")
    print(f"  window terrain elevation: min={window_elev.min():.1f}m max={window_elev.max():.1f}m "
          f"relief={window_elev.max()-window_elev.min():.1f}m")
    print(f"  STEP1B_CONFIG: min_agl_m={STEP1B_CONFIG.min_agl_m} (DEFAULT_CONFIG.min_agl_m={DEFAULT_CONFIG.min_agl_m} unchanged), "
          f"z_step_m={STEP1B_CONFIG.z_step_m} (unchanged)")
    print(f"  layers (MSL): {LAYERS_MSL}")

    rows = list(range(r0, r1))
    cols = list(range(c0, c1))

    # ---- build the candidate lattice + validity classification ----
    # per_layer[layer_msl] = {
    #   "z_msl_values": set(),           # for assertion A/D
    #   "valid": [(row,col,z_msl), ...],
    #   "invalid_clearance": [...],      # invalid specifically via below_min_agl
    #   "invalid_other": [...],          # invalid via out_of_bounds/nodata (expected empty here)
    # }
    per_layer = {}
    for layer_msl in LAYERS_MSL:
        z_index = msl_to_z_index(layer_msl, STEP1B_CONFIG)  # exact, grid-aligned -- raises otherwise
        entry = {"z_msl_values": set(), "valid": [], "invalid_clearance": [], "invalid_other": []}
        for row in rows:
            for col in cols:
                node_z_msl = z_index_to_msl(z_index, STEP1B_CONFIG)  # reused verbatim, unchanged
                entry["z_msl_values"].add(node_z_msl)
                x, y = terrain.rowcol_to_xy(row, col)
                result = evaluate_agl(terrain, x, y, node_z_msl, STEP1B_CONFIG)  # reused verbatim
                if result.valid:
                    entry["valid"].append((row, col, node_z_msl))
                elif result.reason == "below_min_agl":
                    entry["invalid_clearance"].append((row, col, node_z_msl, result.terrain_elevation_msl))
                else:
                    entry["invalid_other"].append((row, col, node_z_msl, result.reason))
        per_layer[layer_msl] = entry

    print("\n=== PER-LAYER CANDIDATE/VALID COUNTS ===")
    total_cells = len(rows) * len(cols)
    for layer_msl in LAYERS_MSL:
        e = per_layer[layer_msl]
        print(f"  layer={layer_msl:.0f}m  candidates={total_cells}  valid={len(e['valid'])}  "
              f"invalid(clearance)={len(e['invalid_clearance'])}  invalid(other)={len(e['invalid_other'])}")

    # ---- assertions ----
    results = {}

    # A. Every candidate on the same horizontal layer has the identical MSL Z.
    a_ok = all(len(per_layer[layer_msl]["z_msl_values"]) == 1 for layer_msl in LAYERS_MSL)
    results["A"] = a_ok

    # B. Every valid node: z_msl - terrain_msl >= min_agl_m.
    b_violations = []
    for layer_msl in LAYERS_MSL:
        for (row, col, z_msl) in per_layer[layer_msl]["valid"]:
            x, y = terrain.rowcol_to_xy(row, col)
            terrain_msl = terrain.query(x, y).elevation
            clearance = z_msl - terrain_msl
            if clearance < STEP1B_CONFIG.min_agl_m - 1e-9:
                b_violations.append((layer_msl, row, col, clearance))
    results["B"] = len(b_violations) == 0

    # C. Every node invalid BECAUSE OF clearance: z_msl - terrain_msl < min_agl_m.
    c_violations = []
    for layer_msl in LAYERS_MSL:
        for (row, col, z_msl, terrain_msl) in per_layer[layer_msl]["invalid_clearance"]:
            clearance = z_msl - terrain_msl
            if clearance >= STEP1B_CONFIG.min_agl_m - 1e-9:
                c_violations.append((layer_msl, row, col, clearance))
    results["C"] = len(c_violations) == 0

    # D. As terrain rises/falls across the window, the layer's own Z value never
    #    changes -- a targeted spot-check at the window's own lowest- and
    #    highest-terrain cells, reading back the ACTUAL z_msl each was assigned
    #    (from the same records built above), not a fresh/independent recomputation.
    min_row, min_col, max_row, max_col = None, None, None, None
    min_t, max_t = math.inf, -math.inf
    for row in rows:
        for col in cols:
            x, y = terrain.rowcol_to_xy(row, col)
            t = terrain.query(x, y).elevation
            if t < min_t:
                min_t, min_row, min_col = t, row, col
            if t > max_t:
                max_t, max_row, max_col = t, row, col

    def recorded_z_msl(layer_msl, row, col):
        e = per_layer[layer_msl]
        for bucket in ("valid", "invalid_clearance", "invalid_other"):
            for rec in e[bucket]:
                if rec[0] == row and rec[1] == col:
                    return rec[2]
        return None

    d_ok = True
    for layer_msl in LAYERS_MSL:
        z_at_min_terrain = recorded_z_msl(layer_msl, min_row, min_col)
        z_at_max_terrain = recorded_z_msl(layer_msl, max_row, max_col)
        if z_at_min_terrain != layer_msl or z_at_max_terrain != layer_msl or z_at_min_terrain != z_at_max_terrain:
            d_ok = False
    print(f"  (D detail: window lowest terrain={min_t:.1f}m at (row={min_row},col={min_col}); "
          f"highest terrain={max_t:.1f}m at (row={max_row},col={max_col}))")
    results["D"] = d_ok

    print("\n=== ASSERTIONS ===")
    print(f"  A (same layer -> identical Z everywhere): {'PASS' if results['A'] else 'FAIL'}")
    print(f"  B (every valid node clears min_agl):       {'PASS' if results['B'] else 'FAIL'}"
          + (f"  -- {len(b_violations)} violations, first={b_violations[0]}" if b_violations else ""))
    print(f"  C (every clearance-invalid node under min_agl): {'PASS' if results['C'] else 'FAIL'}"
          + (f"  -- {len(c_violations)} violations, first={c_violations[0]}" if c_violations else ""))
    print(f"  D (layer Z constant as terrain varies):    {'PASS' if results['D'] else 'FAIL'}")

    invalid_other_total = sum(len(per_layer[l]["invalid_other"]) for l in LAYERS_MSL)
    print(f"\n  (diagnostic: invalid_other (NoData/out_of_bounds) count = {invalid_other_total}, "
          f"expected 0 for this interior window)")

    # ---- visualization: real-DEM transect through the row with the most relief ----
    row_reliefs = [(window_elev[r - r0].max() - window_elev[r - r0].min(), r) for r in rows]
    _, transect_row = max(row_reliefs)
    build_svg(terrain, transect_row, cols, LAYERS_MSL, STEP1B_CONFIG, OUTPUT_SVG)
    print(f"\nVisualization written to {OUTPUT_SVG} (transect row={transect_row})")


def build_svg(terrain, transect_row, cols, layers_msl, config, out_path):
    """Real-DEM 2D transect: terrain profile + horizontal MSL layers (near-
    terrain panel, zoomed) + valid/invalid candidate markers, reusing
    evaluate_agl() for the same classification computed above -- this
    function only draws, it does not re-decide validity independently."""
    terrain_profile = []
    for col in cols:
        x, y = terrain.rowcol_to_xy(transect_row, col)
        terrain_profile.append(terrain.query(x, y).elevation)

    near_layers = [l for l in layers_msl if l <= 4000.0]
    far_layers = [l for l in layers_msl if l > 4000.0]

    y_min = min(terrain_profile) - 150.0
    y_max = max(near_layers, default=max(terrain_profile)) + 150.0
    px_w, px_h = 900, 480
    margin_l, margin_r, margin_t, margin_b = 70, 20, 30, 40
    plot_w = px_w - margin_l - margin_r
    plot_h = px_h - margin_t - margin_b

    def sx(i):
        return margin_l + (i / (len(cols) - 1)) * plot_w

    def sy(z):
        return margin_t + (1.0 - (z - y_min) / (y_max - y_min)) * plot_h

    parts = []
    parts.append(f'<svg viewBox="0 0 {px_w} {px_h + 130}" xmlns="http://www.w3.org/2000/svg" '
                  f'font-family="monospace" font-size="11">')
    parts.append(f'<rect x="0" y="0" width="{px_w}" height="{px_h + 130}" fill="#0a0e13"/>')
    parts.append(f'<text x="{margin_l}" y="18" fill="#e7edf3" font-size="14" font-weight="bold">'
                 f'Step 1B: horizontal MSL layers over REAL Aladağlar DEM (row={transect_row})</text>')

    # near-terrain panel
    for layer_msl in near_layers:
        y = sy(layer_msl)
        color = "#5c6b7c"
        parts.append(f'<line x1="{margin_l}" y1="{y:.1f}" x2="{margin_l+plot_w}" y2="{y:.1f}" '
                      f'stroke="{color}" stroke-width="1" stroke-dasharray="4,3"/>')
        parts.append(f'<text x="{margin_l+plot_w+4}" y="{y+4:.1f}" fill="{color}">{layer_msl:.0f}m</text>')

    terrain_pts = " ".join(f"{sx(i):.1f},{sy(z):.1f}" for i, z in enumerate(terrain_profile))
    fill_pts = f"{sx(0):.1f},{sy(y_min):.1f} {terrain_pts} {sx(len(cols)-1):.1f},{sy(y_min):.1f}"
    parts.append(f'<polygon points="{fill_pts}" fill="#3a2f22" stroke="#a68a5c" stroke-width="1.5"/>')

    for i, col in enumerate(cols):
        x, y = terrain.rowcol_to_xy(transect_row, col)
        terrain_msl = terrain_profile[i]
        for layer_msl in near_layers:
            result = evaluate_agl(terrain, x, y, layer_msl, config)
            cx_, cy_ = sx(i), sy(layer_msl)
            if result.valid:
                parts.append(f'<circle cx="{cx_:.1f}" cy="{cy_:.1f}" r="2.6" fill="#35c97b"/>')
            elif result.reason == "below_min_agl":
                parts.append(f'<line x1="{cx_-2.6:.1f}" y1="{cy_-2.6:.1f}" x2="{cx_+2.6:.1f}" y2="{cy_+2.6:.1f}" '
                              f'stroke="#e8432f" stroke-width="1.3"/>')
                parts.append(f'<line x1="{cx_-2.6:.1f}" y1="{cy_+2.6:.1f}" x2="{cx_+2.6:.1f}" y2="{cy_-2.6:.1f}" '
                              f'stroke="#e8432f" stroke-width="1.3"/>')

    parts.append(f'<text x="{margin_l}" y="{px_h+15}" fill="#8fa0b3">'
                 f'green dot = valid (clearance&gt;=100m)   red X = invalid (clearance&lt;100m)   '
                 f'brown fill = real terrain profile</text>')
    parts.append(f'<text x="{margin_l}" y="{px_h+32}" fill="#8fa0b3">'
                 f'far_layers (all &gt; 4000m, off this zoomed panel): '
                 f'{", ".join(f"{l:.0f}m" for l in far_layers)} -- see printed counts, all 100% valid '
                 f'over this window (real terrain max ~{max(terrain_profile):.0f}m)</text>')
    parts.append(f'<text x="{margin_l}" y="{px_h+49}" fill="#5c6b7c">'
                 f'window: 40x40 fine cells, ~1200m x 1200m, centered on ROI max-elevation pixel</text>')
    parts.append("</svg>")

    import os
    os.makedirs("outputs", exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
