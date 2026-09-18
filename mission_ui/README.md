# Mission UI: terrain-aware fixed-wing mission planner (web)

This is a standalone web UI for the planner in `planner/`. It is isolated in
`mission_ui/`. Planner code is **not modified**, and the Python planner stays the
single source of truth.

```powershell
# from the repository root
python -m mission_ui.server --warm bilecik          # http://127.0.0.1:8765
python -m mission_ui.server --prebuild-zoom 8       # optional: pre-render the national terrain pyramid
python -m pytest -q mission_ui/tests                # 6 adapter/terrain contract tests
```

On Windows, `UAV_Pathfinder.bat` in the repository root starts the same server
from a menu and opens the browser once the port answers
(see [`../scripts/launcher/README.md`](../scripts/launcher/README.md)).

No new Python dependencies: the server uses the stdlib HTTP server plus
numpy / rasterio / pyproj / Pillow, which the planner already needs.
No Node or build step is required either: the frontend is plain ES modules
with vendored MapLibre GL JS, uPlot and Fira fonts under `web/vendor/`.
It works offline. The optional OpenStreetMap layer is the only thing that
needs internet.

All four planner regions — **bilecik, mugla, ankara, aladaglar** — are served;
the region is picked from the map position, not from a dropdown. Mission
presets exist for Bilecik only. See [`../regions/README.md`](../regions/README.md)
for the data contract behind them.

## Command line

| Flag | Default | Purpose |
|---|---|---|
| `--host` | `127.0.0.1` | bind address |
| `--port` | `8765` | API + static files |
| `--tile-port` | `--port + 1` | second port for terrain tiles; `0` serves them from the main port |
| `--warm REGION...` | none | preload region DEM + 60 m buffered field in the background |
| `--prebuild-zoom Z` | none | render and cache every display-terrain tile up to zoom Z (capped at 12) |
| `--cache-dir` | `mission_ui/.cache` | terrain tile cache location |
| `--terrain-source DIR` | `copernicus_glo30_turkey/` in the repo, else next to it | Copernicus GLO-30 COGs for *display* terrain; optional (see below) |
| `--planner-thread` | off | run the planner in-process instead of a worker process (debugging only) |

## Running without the Copernicus sources

The ~5 GB `copernicus_glo30_turkey/` COG set is **display-only and optional**.
Planning, AGL and the altitude profile never touch it — they use
`regions/<id>/working_dem.tif`.

The server picks its display terrain in this order:

1. **COGs present** (`source_mode: cogs`) — full detail up to zoom 12.
2. **No COGs, cached mosaic present** (`source_mode: cache`) — terrain is sampled
   from `.cache/terrain/v1/mosaic_8arcsec.npy` (≈65 MB, 8″ ≈ 240 m). Relief is
   correct everywhere the mosaic covers; above zoom 9 it is coarser than the COGs
   would be, and those tiles are deliberately *not* written to the cache so that a
   later run with the COGs present is not stuck with them. Any tiles already in
   the cache are served as-is, at their original detail.
3. **Neither** (`source_mode: none`) — flat tiles and a warning on startup. The
   map still works; it just has no relief.

So to run the UI on another machine, ship the code, at least one region under
`regions/`, and `mission_ui/.cache/terrain/` — not the COGs.
`scripts/make_demo_package.ps1` already packages exactly this.

Startup prints which mode is active next to `terrain source tiles: N`, and
`/api/meta` reports it as `source_mode` in the terrain TileJSON.

## Workflow

1. Pick a preset (taken verbatim from `scripts/`), or click the map to place
   **S** (start) and then **G** (goal). You can drag markers or type lat/lon.
2. Set altitude as **AGL** or **MSL**. The backend resolves AGL against the
   *planner terrain* (block-max 30 m DEM + lateral-buffer max filter), which is
   the same field the mission scripts use. Both values are always shown.
3. Set min AGL (hard floor), target AGL and lateral buffer. Budgets and smoothing
   are under *Search budget & smoothing*.
4. **Run planner**. The real pipeline runs as a background job.
5. Inspect the route in 2D, switch to **3D**, and read the altitude profile.
   Hovering the chart moves a marker on the map (2D and 3D). Hovering the route
   in 2D moves the chart cursor.
6. **Engineering** mode adds search, profile and smoothing diagnostics, the
   stage layers (A* track / profiled / final), vertical-rate and bank/roll charts,
   event fly-to, and the tile LOD grid.

## Architecture

```
browser (web/)                               python (server/)
─────────────────────────────                ────────────────────────────────────────────
main.js      state + panels     ──POST /api/plan──▶  planner_service.PlannerService (1 worker)
map-view.js  MapLibre 2D / 3D   ◀─GET /api/jobs/id─      plan_terrain_following (A* + profile + feedback)
route3d-layer.js WebGL2 route                            apply_corridor_safe_local_bspline_smoothing
charts.js    uPlot profile      ──GET /api/point───▶  regions.RegionRegistry (cached ROI, buffered field)
             hover sync         ──GET /api/terrain/z/x/y.png──▶ terrain_tiles.TerrainTileService
```

| File | Role |
|---|---|
| `server/app.py` | HTTP routes, static files, gzip, CLI |
| `server/planner_service.py` | request validation, AGL→MSL resolution, job queue, planner call, output packaging |
| `server/regions.py` | region registry (`regions/regions_manifest.json`) and cached `load_roi` / `TerrainQuery` / buffered field |
| `server/terrain_tiles.py` | display terrain pyramid (Terrarium PNG) from Copernicus GLO-30 COGs |
| `server/presets.py` | mission presets copied from `scripts/` |
| `web/js/map-view.js` | one MapLibre map: 2D (hillshade + tint) and 3D (GPU terrain) |
| `web/js/route3d-layer.js` | custom WebGL2 layer: route at true MSL, clearance curtain, masts, hover |
| `web/js/charts.js` | altitude profile, AGL, vertical rate, bank/roll (uPlot canvas) |
| `web/js/main.js` | mission inputs, validation, job polling, results and engineering tables |

## API

| Method | Path | Purpose |
|---|---|---|
| GET | `/api/meta` | defaults, aircraft limits, terrain TileJSON |
| GET | `/api/regions` | planner regions with ROI polygon (lon/lat), CRS, bounds |
| GET | `/api/presets` | script missions in lon/lat and UTM |
| GET | `/api/point?lon&lat&lateral_buffer_m` | region, UTM, DEM cell, **planner ground** (buffered) |
| GET | `/api/point?lon&lat&planner=0` | lightweight hover readout (display DEM only) |
| POST | `/api/plan` | start a job → `202 {id, state}` |
| GET | `/api/jobs/{id}` | `{state, stage, elapsed_s, stage_log, error, result}` |
| GET | `/api/terrain/{z}/{x}/{y}.png` | display terrain tile (Terrarium) |
| GET | `/api/terrain/stats` | tile render / cache counters |

### `POST /api/plan`

```json
{
  "start": {"lon": 30.158, "lat": 40.165, "alt": {"mode": "agl", "value": 130}, "heading_deg": null},
  "goal":  {"lon": 30.157, "lat": 40.201, "alt": {"mode": "msl", "value": 480}},
  "safety": {"min_agl_m": 100, "target_agl_m": 120, "lateral_buffer_m": 60},
  "tolerance": {"xy_m": 200, "alt_m": 30},
  "budget": {"max_expansions": 120000, "max_search_time_s": 120, "max_feedback_passes": 3},
  "smoothing": {"enabled": true, "max_corridor_deviation_m": 12}
}
```

`heading_deg: null` means the bearing to the goal (what the 90 km scripts use).

### Result (`job.result`)

* `mission`: resolved start/goal (lon, lat, UTM, planner ground, **MSL and AGL**, heading source).
* `status`: `verdict` (`SAFE`, `SAFE_WITH_ADVISORIES`, `UNSAFE` or `NO_ROUTE`), headline and checks.
  Hard checks are planner validation, final min AGL and corridor safety. Advisory checks are
  vertical rate, bank and roll rate. The roll-rate cap is a known open issue
  (see the "Bilinen sınırlar" section of the root [`README.md`](../README.md)).
* `search`, `profile`, `smoothing`, `flight`, `timing`: planner metrics.
* `stages`: only stages that exist in the pipeline. Each is columnar and decimated to ≤ 5000
  points, always keeping the endpoints and extreme samples.
  * `search`: A* ground track at search altitude, or `search_best_partial` when the budget
    ended (diagnostic only, never labelled a solution).
  * `profiled`: terrain-following profile.
  * `final`: smoothed trajectory telemetry (1 m source samples): `s, t, lon, lat, x, y, z, ground,
    agl, heading, bank, roll_rate, vz, load, dev, in_transition`.
* `events`: min AGL, max roll rate, max altitude, and the profile failure location.

The browser never converts coordinates for the planner and never computes
trajectory, safety or physics. `lon/lat` come from the backend (pyproj).
Web Mercator conversion is done only for rendering.

## Responsiveness

* The planner runs in a **separate worker process** (spawned once, reused). It is CPU-bound
  pure Python; in the server process it would hold the GIL and stall tile serving and job
  polling, which makes the page look frozen. If the worker dies, the job fails with a clear
  message and the next job restarts it. `--planner-thread` restores in-process mode for debugging.
* Terrain tiles are also served on **`--port + 1`** (for example 8766). Browsers allow only about 6
  connections per host:port, so a burst of 3D tile requests no longer queues the API calls.
  Use `--tile-port 0` to serve tiles from the main port only.
* Cold tile renders are limited to a few at a time. Low-zoom tiles never wait for the one-off
  national mosaic build; they are rendered from the COG overviews until the mosaic exists.
* Job polling tolerates slow responses (10 s timeout, shows "Waiting for server…") instead of failing.

## Terrain LOD

* **Display terrain** (visual only) is a Web Mercator XYZ pyramid of 256 px Terrarium PNGs:
  * **z ≤ 9:** sampled from a national mosaic built once from the COG 1/8 overviews
    (8″ ≈ 240 m) and cached as `.npy`.
  * **z 10–12:** windowed, decimated reads of only the intersecting 1° COGs (GDAL uses the overviews).
  * **z > 12:** MapLibre overzooms. GLO-30 has no finer detail.
* Display tiles are rendered from the `copernicus_glo30_turkey/` COGs, which are **not** part
  of the repository and are optional at runtime — see "Running without the Copernicus sources"
  above. Outside the covered box the service returns a flat zero tile.
* Tiles are rendered on first request and then cached on disk
  (`mission_ui/.cache/terrain/v1`, git-ignored). Zoom 0–8 — the levels the map opens on —
  are always warmed at startup (~175 tiles, ~2 s, all cheap mosaic samples), so
  country-scale panning never waits for a cold render. `--prebuild-zoom` extends that to
  the finer levels.
* Browser caching is per tile *kind*, because the alternative poisons clients: a real tile
  is `immutable` for a week, a coarse mosaic stand-in is revalidated hourly, and a flat
  placeholder (no terrain available) is `no-store` and never cached. The tile URL also
  carries a `?v=<source_mode><source_count>` epoch, so when the terrain situation changes —
  sources added or removed — clients request new URLs instead of reusing what they hold.
* MapLibre loads only the tiles covering the viewport at the current zoom
  (quadtree LOD) and draws 3D terrain on the GPU. Picking uses MapLibre's geometric
  unprojection against the terrain; individual DEM cells are never objects.
* The 3D terrain source declares 128 px tiles, so the 3D mesh uses DEM tiles one zoom
  level finer than 2D shading. This prevents narrow canyons from being filled in by
  coarse averaging at overview zooms.
* Planning, AGL and the altitude profile always use the **planner DEM**
  (`regions/<id>/working_dem.tif`), never display tiles.
* A new mission or route never reloads terrain. Only GeoJSON sources and the route
  WebGL buffers change.

## Altitude semantics

* **MSL**: Copernicus GLO-30 heights (EGM2008 geoid), which is what the planner uses as `z_msl_m`.
* **Planner terrain / ground**: block-max 30 m DEM, max-filtered by `lateral_buffer_m`.
* **AGL** = MSL − planner ground. The UI always shows both references, and the backend
  rejects start/goal points below `min_agl_m`.
* **Min AGL**: the hard floor enforced by the planner. **Target AGL**: the terrain-following
  goal (`desired_agl_m`).
