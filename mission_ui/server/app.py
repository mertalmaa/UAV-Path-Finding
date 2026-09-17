"""HTTP API + static file server for the mission planning UI (stdlib only).

Run from the repository root:

    python -m mission_ui.server            # http://127.0.0.1:8765
    python -m mission_ui.server --port 9000 --warm bilecik
"""
from __future__ import annotations

import argparse
import gzip
import json
import mimetypes
import re
import sys
import threading
import time
import traceback
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from .planner_service import DEFAULTS, PlannerService, RequestError  # noqa: E402
from .presets import PRESETS  # noqa: E402
from .regions import RegionRegistry  # noqa: E402
from .terrain_tiles import TerrainTileService  # noqa: E402

WEB_ROOT = Path(__file__).resolve().parents[1] / "web"
mimetypes.add_type("text/javascript", ".mjs")
mimetypes.add_type("text/javascript", ".js")
mimetypes.add_type("font/woff2", ".woff2")

TILE_RE = re.compile(r"^/api/terrain/(\d+)/(\d+)/(\d+)\.png$")
JOB_RE = re.compile(r"^/api/jobs/([\w-]+)$")


class App:
    def __init__(self, project_root: Path, cache_dir: Path, planner_mode: str = "process", warm_regions=(),
                 tile_port: int = None):
        self.registry = RegionRegistry(project_root)
        self.tiles = TerrainTileService(project_root / "copernicus_glo30_turkey", cache_dir)
        self.planner = PlannerService(self.registry, mode=planner_mode, warm_regions=warm_regions)
        self.tile_port = tile_port
        self.started = time.time()

    def regions_payload(self) -> dict:
        out = []
        for info in self.registry.regions.values():
            out.append({
                "region_id": info.region_id, "name": info.name, "description": info.description,
                "center_lonlat": info.center_lonlat, "crs": info.crs, "roi_size_m": info.roi_size_m,
                "roi_bounds_utm": info.roi_bounds_utm, "elevation_stats": info.elevation_stats,
                "roi_polygon": self.registry.roi_polygon_lonlat(info),
                "loaded": self.registry.is_loaded(info.region_id),
            })
        return {"regions": out}

    def presets_payload(self) -> dict:
        out = []
        for p in PRESETS:
            info = self.registry.regions.get(p["region_id"])
            if info is None:
                continue
            _, to_ll = self.registry.transformers(info.crs)
            s = to_ll.transform(*p["start_xy"])
            g = to_ll.transform(*p["goal_xy"])
            out.append({**{k: v for k, v in p.items() if k not in ("start_xy", "goal_xy")},
                        "start": {"lon": s[0], "lat": s[1], "x_m": p["start_xy"][0], "y_m": p["start_xy"][1]},
                        "goal": {"lon": g[0], "lat": g[1], "x_m": p["goal_xy"][0], "y_m": p["goal_xy"][1]}})
        return {"presets": out}


def make_handler(app: App):
    class Handler(BaseHTTPRequestHandler):
        server_version = "UAVMissionUI/1.0"

        def log_message(self, fmt, *args):  # quiet tile spam; keep API lines
            if "/api/terrain/" in (self.path or ""):
                return
            sys.stderr.write("[%s] %s\n" % (time.strftime("%H:%M:%S"), fmt % args))

        # ---------------------------------------------------------- helpers
        def _send(self, status: int, body: bytes, ctype: str, extra=None):
            accept = self.headers.get("Accept-Encoding", "")
            if len(body) > 2048 and "gzip" in accept and not ctype.startswith(("image/", "font/")):
                body = gzip.compress(body, compresslevel=5)
                extra = {**(extra or {}), "Content-Encoding": "gzip"}
            self.send_response(status)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            if self.command != "HEAD":
                self.wfile.write(body)

        def _json(self, status: int, payload, cache: str = "no-store"):
            body = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode("utf-8")
            self._send(status, body, "application/json; charset=utf-8", {"Cache-Control": cache})

        def _error(self, status: int, message: str):
            self._json(status, {"error": message})

        # ------------------------------------------------------------ routes
        def do_GET(self):
            try:
                self._route_get()
            except BrokenPipeError:
                pass
            except Exception as exc:
                traceback.print_exc()
                try:
                    self._error(500, f"{type(exc).__name__}: {exc}")
                except Exception:
                    pass

        do_HEAD = do_GET

        def _route_get(self):
            url = urlparse(self.path)
            path = url.path
            m = TILE_RE.match(path)
            if m:
                z, x, y = (int(v) for v in m.groups())
                try:
                    png, src = app.tiles.get_tile(z, x, y)
                except ValueError as exc:
                    return self._error(404, str(exc))
                return self._send(200, png, "image/png", {
                    "Cache-Control": "public, max-age=604800, immutable", "X-Tile-Source": src,
                    "Access-Control-Allow-Origin": "*", "Access-Control-Expose-Headers": "X-Tile-Source"})
            if path == "/api/health":
                return self._json(200, {"ok": True, "uptime_s": round(time.time() - app.started, 1)})
            if path == "/api/meta":
                return self._json(200, {
                    "defaults": DEFAULTS, "limits": app.planner.limits(),
                    "terrain": app.tiles.tilejson("/api/terrain/{z}/{x}/{y}.png"),
                    # Tiles are also served on a second port: browsers allow only ~6 connections
                    # per host:port, so a burst of terrain tiles can no longer stall API polling.
                    "tile_port": app.tile_port,
                })
            if path == "/api/regions":
                return self._json(200, app.regions_payload())
            if path == "/api/presets":
                return self._json(200, app.presets_payload())
            if path == "/api/point":
                q = parse_qs(url.query)
                try:
                    lon = float(q["lon"][0]); lat = float(q["lat"][0])
                    buf = float(q.get("lateral_buffer_m", [DEFAULTS["lateral_buffer_m"]])[0])
                except (KeyError, ValueError):
                    return self._error(400, "lon, lat (and optional lateral_buffer_m) are required numbers")
                if q.get("planner", ["1"])[0] == "0":
                    # Lightweight hover readout: display DEM only, never loads a region.
                    info = app.registry.find_region(lon, lat)
                    payload = {"lon": lon, "lat": lat, "region_id": info.region_id if info else None}
                else:
                    payload = app.planner.point_info(lon, lat, buf)
                payload["display_dem_m"] = app.tiles.sample_elevation(lon, lat)
                return self._json(200, payload)
            if path == "/api/terrain/stats":
                return self._json(200, {**app.tiles.stats, "mosaic_build_s": app.tiles.mosaic_build_s})
            m = JOB_RE.match(path)
            if m:
                job = app.planner.jobs.get(m.group(1))
                if job is None:
                    return self._error(404, "unknown job")
                return self._json(200, job.summary(include_result=True))
            return self._static(path)

        def _static(self, path: str):
            if path in ("", "/"):
                path = "/index.html"
            target = (WEB_ROOT / path.lstrip("/")).resolve()
            if WEB_ROOT not in target.parents and target != WEB_ROOT or not target.is_file():
                return self._error(404, "not found")
            ctype = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
            cache = "public, max-age=86400" if "/vendor/" in path else "no-cache"
            self._send(200, target.read_bytes(), ctype, {"Cache-Control": cache})

        def do_POST(self):
            try:
                url = urlparse(self.path)
                length = int(self.headers.get("Content-Length") or 0)
                if length > 1_000_000:
                    return self._error(413, "request too large")
                raw = self.rfile.read(length) if length else b""
                try:
                    body = json.loads(raw or b"{}")
                except json.JSONDecodeError:
                    return self._error(400, "invalid JSON")
                if url.path == "/api/plan":
                    try:
                        job = app.planner.submit(body)
                    except RequestError as exc:
                        return self._error(400, str(exc))
                    return self._json(202, job.summary(include_result=False))
                return self._error(404, "not found")
            except Exception as exc:
                traceback.print_exc()
                self._error(500, f"{type(exc).__name__}: {exc}")

    return Handler


def main(argv=None):
    ap = argparse.ArgumentParser(description="UAV mission planning UI server")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--cache-dir", default=str(PROJECT_ROOT / "mission_ui" / ".cache"))
    ap.add_argument("--warm", nargs="*", default=[], metavar="REGION",
                    help="preload region DEM + 60 m buffered field in the background")
    ap.add_argument("--prebuild-zoom", type=int, default=None, metavar="Z",
                    help="render and cache every display-terrain tile up to zoom Z in the background "
                         "(e.g. 8 for the whole country; tiles are otherwise rendered on first request)")
    ap.add_argument("--tile-port", type=int, default=None,
                    help="second port for terrain tiles (default: --port + 1; 0 disables)")
    ap.add_argument("--planner-thread", action="store_true",
                    help="run the planner in a thread instead of a separate process (debugging only)")
    args = ap.parse_args(argv)
    tile_port = args.port + 1 if args.tile_port is None else (args.tile_port or None)

    app = App(PROJECT_ROOT, Path(args.cache_dir), planner_mode="thread" if args.planner_thread else "process",
              warm_regions=args.warm, tile_port=tile_port)

    def warm():
        try:
            if app.tiles.coverage:
                app.tiles.ensure_mosaic()
            if args.prebuild_zoom is not None:
                t0 = time.perf_counter(); count = 0
                for z, x, y in app.tiles.iter_coverage_tiles(min(args.prebuild_zoom, 12)):
                    app.tiles.get_tile(z, x, y); count += 1
                print(f"[warm] terrain pyramid z0-{args.prebuild_zoom}: {count} tiles in {time.perf_counter() - t0:.1f} s")
            for rid in args.warm:
                if rid in app.registry.regions:
                    ctx = app.registry.context(rid)
                    ctx.field(DEFAULTS["lateral_buffer_m"])
                    print(f"[warm] region {rid} ready")
        except Exception:
            traceback.print_exc()

    threading.Thread(target=warm, name="warm", daemon=True).start()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(app))
    server.daemon_threads = True
    if tile_port:
        tile_server = ThreadingHTTPServer((args.host, tile_port), make_handler(app))
        tile_server.daemon_threads = True
        threading.Thread(target=tile_server.serve_forever, name="tile-server", daemon=True).start()
    print(f"UAV mission UI: http://{args.host}:{args.port}  (tiles on :{tile_port}; regions: {', '.join(app.registry.regions)}; "
          f"terrain source tiles: {len(app.tiles.sources)})")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
