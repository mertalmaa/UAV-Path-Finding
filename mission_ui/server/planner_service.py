"""Planner adapter: mission request -> real planner pipeline -> UI data contract.

Pipeline (identical to scripts/run_bilecik_90km_5_missions.py, plus the
closed-loop feedback of planner.terrain_following.plan_terrain_following):

  1. pose-aware A* ground-track search (planner.pose_search)
  2. terrain-following altitude profile (planner.terrain_following)
  3. corridor-safe local B-spline smoothing (planner.local_trajectory_smoothing)

Nothing here plans, validates or computes physics. The adapter only:
  * converts WGS84 <-> region CRS (pyproj),
  * resolves an AGL altitude *input* to MSL using the planner's own buffered
    terrain field (the same field the mission scripts use),
  * serialises planner outputs, decimating for transport while keeping the
    extreme samples (min AGL, max roll rate, max bank) that the UI highlights.
"""
from __future__ import annotations

import itertools
import math
import queue
import threading
import time
import traceback
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from planner.fixed_wing_envelope import (
    BANK_ANGLE_DEG,
    HORIZONTAL_SPEED_MPS,
    MAX_CLIMB_RATE_MPS,
    MAX_DESCENT_RATE_MPS,
    TURN_RADIUS_M,
    FixedWingKinematicEnvelope,
)
from planner.local_trajectory_smoothing import apply_corridor_safe_local_bspline_smoothing
from planner.physical import PhysicalPose, PhysicalTrajectory
from planner.pose_search import GoalPose, GoalTolerance, navigation_bearing_deg
from planner.terrain_following import plan_terrain_following

from .regions import RegionContext, RegionRegistry

# Roll-rate cap referenced by the smoothing stage and all mission scripts.
ROLL_RATE_CAP_DEG_S = 15.0
MAX_TRANSPORT_POINTS = 5000

DEFAULTS = {
    "start_alt": {"mode": "agl", "value": 150.0},
    "goal_alt": {"mode": "agl", "value": 150.0},
    "min_agl_m": 100.0,          # mission scripts (DEFAULT_CONFIG is 200 m)
    "target_agl_m": 120.0,
    "lateral_buffer_m": 60.0,
    "goal_tolerance_xy_m": 200.0,
    "goal_tolerance_alt_m": 30.0,
    "max_expansions": 120000,
    "max_search_time_s": 120.0,
    "max_feedback_passes": 3,
    "smoothing": True,
    "max_corridor_deviation_m": 12.0,
}


class RequestError(ValueError):
    """Invalid mission input (reported to the user, HTTP 400)."""


def _num(obj: dict, key: str, default=None, lo=None, hi=None) -> float:
    v = obj.get(key, default)
    if v is None:
        raise RequestError(f"'{key}' is required")
    try:
        v = float(v)
    except (TypeError, ValueError):
        raise RequestError(f"'{key}' must be a number")
    if not math.isfinite(v):
        raise RequestError(f"'{key}' must be finite")
    if lo is not None and v < lo:
        raise RequestError(f"'{key}' must be >= {lo}")
    if hi is not None and v > hi:
        raise RequestError(f"'{key}' must be <= {hi}")
    return v


@dataclass
class Job:
    id: str
    request: dict
    state: str = "queued"          # queued | running | done | failed
    stage: str = "queued"
    created: float = field(default_factory=time.time)
    started: Optional[float] = None
    finished: Optional[float] = None
    result: Optional[dict] = None
    error: Optional[str] = None
    stage_log: List[dict] = field(default_factory=list)

    def set_stage(self, stage: str):
        now = time.time()
        self.stage = stage
        self.stage_log.append({"stage": stage, "t": round(now - (self.started or now), 3)})

    def summary(self, include_result: bool = True) -> dict:
        now = time.time()
        out = {
            "id": self.id,
            "state": self.state,
            "stage": self.stage,
            "elapsed_s": round(((self.finished or now) - (self.started or now)) if self.started else 0.0, 2),
            "queued_s": round(((self.started or now) - self.created), 2),
            "stage_log": self.stage_log,
            "error": self.error,
        }
        if include_result and self.state == "done":
            out["result"] = self.result
        return out


class _JobShim:
    """What _execute needs from a Job, usable inside the worker process."""

    def __init__(self, job_id: str, request: dict, events):
        self.id, self.request, self._events = job_id, request, events

    def set_stage(self, stage: str):
        self._events.put(("stage", self.id, stage))


def _worker_main(project_root: str, tasks, events):
    """Planner worker process entry point (spawn-safe, top-level).

    The planner is CPU-bound pure Python. Running it in its own process keeps
    the HTTP server (terrain tiles, job polling, point queries) responsive:
    in a thread it competes for the GIL and the browser appears frozen.
    """
    import sys as _sys
    if project_root not in _sys.path:
        _sys.path.insert(0, project_root)
    from pathlib import Path as _Path
    svc = PlannerService(RegionRegistry(_Path(project_root)), mode="inline")
    events.put(("ready", None, None))
    while True:
        task = tasks.get()
        if task is None:
            return
        kind, job_id, payload = task
        if kind == "warm":
            try:
                svc.registry.context(payload).field(DEFAULTS["lateral_buffer_m"])
            except Exception:
                traceback.print_exc()
            continue
        shim = _JobShim(job_id, payload, events)
        try:
            events.put(("done", job_id, svc._execute(shim)))
        except RequestError as exc:
            events.put(("failed", job_id, str(exc)))
        except Exception as exc:
            traceback.print_exc()
            events.put(("failed", job_id, f"{type(exc).__name__}: {exc}"))


class PlannerService:
    """Job bookkeeping + validation in the server process.

    mode="process" (default): planner runs in a dedicated worker process.
    mode="thread": planner runs in a background thread (debug / tests).
    mode="inline": no worker; used inside the worker process itself.
    """

    def __init__(self, registry: RegionRegistry, mode: str = "process", warm_regions=()):
        self.registry = registry
        self.envelope = FixedWingKinematicEnvelope()
        self.jobs: Dict[str, Job] = {}
        self._ids = itertools.count(1)
        self._queue: "queue.Queue[Job]" = queue.Queue()
        self.mode = mode
        self._warm_regions = list(warm_regions)
        self.worker_restarts = 0
        if mode == "thread":
            threading.Thread(target=self._run_forever, name="planner-worker", daemon=True).start()
        elif mode == "process":
            threading.Thread(target=self._run_process_forever, name="planner-dispatch", daemon=True).start()

    # ------------------------------------------------------------ public API
    def limits(self) -> dict:
        return {
            "horizontal_speed_mps": HORIZONTAL_SPEED_MPS,
            "max_climb_rate_mps": MAX_CLIMB_RATE_MPS,
            "max_descent_rate_mps": MAX_DESCENT_RATE_MPS,
            "max_bank_deg": BANK_ANGLE_DEG,
            "turn_radius_m": TURN_RADIUS_M,
            "roll_rate_cap_deg_s": ROLL_RATE_CAP_DEG_S,
        }

    def point_info(self, lon: float, lat: float, lateral_buffer_m: float) -> dict:
        info = self.registry.find_region(lon, lat)
        if info is None:
            return {"lon": lon, "lat": lat, "region_id": None}
        ctx = self.registry.context(info.region_id)
        x, y = ctx.to_utm.transform(lon, lat)
        g = self.registry.ground_at(ctx, x, y, lateral_buffer_m)
        return {"lon": lon, "lat": lat, "region_id": info.region_id, "crs": info.crs,
                "x_m": x, "y_m": y, "lateral_buffer_m": lateral_buffer_m, **g}

    def submit(self, request: dict) -> Job:
        resolved = self._validate(request)  # raises RequestError before queueing
        job = Job(id=f"job-{next(self._ids)}", request=resolved)
        self.jobs[job.id] = job
        # keep the store bounded
        if len(self.jobs) > 30:
            for old in sorted(self.jobs.values(), key=lambda j: j.created)[:-30]:
                if old.state in ("done", "failed"):
                    self.jobs.pop(old.id, None)
        self._queue.put(job)
        return job

    # ------------------------------------------------------------ validation
    def _validate(self, req: dict) -> dict:
        if not isinstance(req, dict):
            raise RequestError("request body must be a JSON object")
        start = req.get("start") or {}
        goal = req.get("goal") or {}
        safety = req.get("safety") or {}
        tol = req.get("tolerance") or {}
        budget = req.get("budget") or {}
        smooth = req.get("smoothing") or {}

        def alt(point, name):
            a = point.get("alt") or {}
            mode = a.get("mode", DEFAULTS[f"{name}_alt"]["mode"])
            if mode not in ("agl", "msl"):
                raise RequestError(f"{name}.alt.mode must be 'agl' or 'msl'")
            return {"mode": mode, "value": _num(a, "value", DEFAULTS[f"{name}_alt"]["value"], -500, 9000)}

        out = {
            "start": {"lon": _num(start, "lon", None, -180, 180), "lat": _num(start, "lat", None, -85, 85),
                      "alt": alt(start, "start"),
                      "heading_deg": None if start.get("heading_deg") in (None, "") else
                      _num(start, "heading_deg", None, -360, 720)},
            "goal": {"lon": _num(goal, "lon", None, -180, 180), "lat": _num(goal, "lat", None, -85, 85),
                     "alt": alt(goal, "goal")},
            "min_agl_m": _num(safety, "min_agl_m", DEFAULTS["min_agl_m"], 0, 3000),
            "target_agl_m": _num(safety, "target_agl_m", DEFAULTS["target_agl_m"], 0, 3000),
            "lateral_buffer_m": _num(safety, "lateral_buffer_m", DEFAULTS["lateral_buffer_m"], 0, 1000),
            "goal_tolerance_xy_m": _num(tol, "xy_m", DEFAULTS["goal_tolerance_xy_m"], 1, 5000),
            "goal_tolerance_alt_m": _num(tol, "alt_m", DEFAULTS["goal_tolerance_alt_m"], 1, 2000),
            "max_expansions": int(_num(budget, "max_expansions", DEFAULTS["max_expansions"], 100, 5_000_000)),
            "max_search_time_s": _num(budget, "max_search_time_s", DEFAULTS["max_search_time_s"], 1, 3600),
            "max_feedback_passes": int(_num(budget, "max_feedback_passes", DEFAULTS["max_feedback_passes"], 1, 10)),
            "smoothing": bool(smooth.get("enabled", DEFAULTS["smoothing"])),
            "max_corridor_deviation_m": _num(smooth, "max_corridor_deviation_m",
                                             DEFAULTS["max_corridor_deviation_m"], 0.5, 200),
        }
        if out["target_agl_m"] < out["min_agl_m"]:
            raise RequestError("target AGL must be >= minimum AGL (planner contract)")
        r1 = self.registry.find_region(out["start"]["lon"], out["start"]["lat"])
        r2 = self.registry.find_region(out["goal"]["lon"], out["goal"]["lat"])
        if r1 is None:
            raise RequestError("start is outside every planner region ROI")
        if r2 is None:
            raise RequestError("goal is outside every planner region ROI")
        if r1.region_id != r2.region_id:
            raise RequestError(f"start ({r1.region_id}) and goal ({r2.region_id}) are in different regions")
        out["region_id"] = r1.region_id
        return out

    # ---------------------------------------------------------------- worker
    def _start_process(self):
        import multiprocessing as mp
        ctx = mp.get_context("spawn")  # identical behaviour on Windows and Linux
        self._tasks, self._events = ctx.Queue(), ctx.Queue()
        self._proc = ctx.Process(target=_worker_main, name="planner-process", daemon=True,
                                 args=(str(self.registry.root), self._tasks, self._events))
        self._proc.start()
        for rid in self._warm_regions:
            self._tasks.put(("warm", None, rid))

    def _run_process_forever(self):
        self._start_process()
        while True:
            job = self._queue.get()
            if not self._proc.is_alive():
                self.worker_restarts += 1
                self._start_process()
            job.state = "running"
            job.started = time.time()
            self._tasks.put(("plan", job.id, job.request))
            while True:
                try:
                    kind, job_id, payload = self._events.get(timeout=1.0)
                except queue.Empty:
                    if not self._proc.is_alive():
                        job.state = "failed"
                        job.error = f"planner worker process exited unexpectedly (exit code {self._proc.exitcode})"
                        break
                    continue
                if job_id != job.id:
                    continue  # 'ready' or stale events
                if kind == "stage":
                    job.set_stage(payload)
                elif kind == "done":
                    job.result = payload
                    job.set_stage("done")
                    job.state = "done"
                    break
                elif kind == "failed":
                    job.state, job.error = "failed", payload
                    break
            job.finished = time.time()

    def _run_forever(self):
        while True:
            job = self._queue.get()
            job.state = "running"
            job.started = time.time()
            try:
                job.result = self._execute(job)
                job.state = "done"
                job.set_stage("done")
            except RequestError as exc:
                job.state, job.error = "failed", str(exc)
            except Exception as exc:  # surfaced verbatim to the engineering view
                job.state = "failed"
                job.error = f"{type(exc).__name__}: {exc}"
                traceback.print_exc()
            finally:
                job.finished = time.time()

    def _resolve_point(self, ctx: RegionContext, p: dict, lateral_buffer_m: float, min_agl_m: float, name: str):
        x, y = ctx.to_utm.transform(p["lon"], p["lat"])
        g = self.registry.ground_at(ctx, x, y, lateral_buffer_m)
        ground = g["planner_ground_m"]
        if ground is None:
            raise RequestError(f"{name}: no valid planner terrain (edge of ROI or NoData) at this location")
        if p["alt"]["mode"] == "agl":
            z = ground + p["alt"]["value"]
        else:
            z = p["alt"]["value"]
        agl = z - ground
        if agl < min_agl_m:
            raise RequestError(
                f"{name} altitude {z:.1f} m MSL is {agl:.1f} m above planner terrain "
                f"({ground:.1f} m, incl. {lateral_buffer_m:.0f} m lateral buffer); minimum AGL is {min_agl_m:.0f} m")
        return {"lon": p["lon"], "lat": p["lat"], "x_m": x, "y_m": y, "ground_m": ground,
                "z_msl_m": z, "agl_m": agl, "alt_input": p["alt"]}

    def _execute(self, job: Job) -> dict:
        r = job.request
        t_total = time.perf_counter()
        job.set_stage("loading_terrain")
        ctx = self.registry.context(r["region_id"])
        t0 = time.perf_counter()
        ctx.field(r["lateral_buffer_m"])  # warm the buffered field used for AGL resolution
        field_s = time.perf_counter() - t0

        start = self._resolve_point(ctx, r["start"], r["lateral_buffer_m"], r["min_agl_m"], "start")
        goal = self._resolve_point(ctx, r["goal"], r["lateral_buffer_m"], r["min_agl_m"], "goal")
        straight = math.hypot(goal["x_m"] - start["x_m"], goal["y_m"] - start["y_m"])
        if straight < 1.0:
            raise RequestError("start and goal are at the same location")
        if r["start"]["heading_deg"] is None:
            heading = navigation_bearing_deg(start["x_m"], start["y_m"], goal["x_m"], goal["y_m"])
            heading_source = "bearing_to_goal"
        else:
            heading = r["start"]["heading_deg"] % 360.0
            heading_source = "user"
        start["heading_deg"] = heading
        start["heading_source"] = heading_source

        cfg = ctx.info.config(min_agl_m=r["min_agl_m"], desired_agl_m=r["target_agl_m"],
                              lateral_buffer_m=r["lateral_buffer_m"], enable_combined_turns=True)
        start_pose = PhysicalPose(start["x_m"], start["y_m"], start["z_msl_m"], heading)
        goal_pose = GoalPose(goal["x_m"], goal["y_m"], goal["z_msl_m"])
        tolerance = GoalTolerance(xy_m=r["goal_tolerance_xy_m"], altitude_m=r["goal_tolerance_alt_m"])

        job.set_stage("search_and_profile")
        t0 = time.perf_counter()
        plan = plan_terrain_following(
            start_pose, goal_pose, ctx.terrain, goal_tolerance=tolerance, config=cfg,
            target_agl_m=r["target_agl_m"], max_expansions=r["max_expansions"],
            max_search_time_s=r["max_search_time_s"], envelope=self.envelope,
            max_feedback_passes=r["max_feedback_passes"],
        )
        plan_s = time.perf_counter() - t0
        search = plan.search_result
        profile = plan.profile_result
        fld = ctx.field(r["lateral_buffer_m"])

        result: Dict[str, Any] = {
            "mission": {
                "region_id": ctx.info.region_id, "region_name": ctx.info.name, "crs": ctx.info.crs,
                "start": start, "goal": goal, "straight_distance_m": straight,
                "min_agl_m": r["min_agl_m"], "target_agl_m": r["target_agl_m"],
                "lateral_buffer_m": r["lateral_buffer_m"],
                "goal_tolerance": {"xy_m": r["goal_tolerance_xy_m"], "alt_m": r["goal_tolerance_alt_m"]},
                "budget": {"max_expansions": r["max_expansions"], "max_search_time_s": r["max_search_time_s"],
                           "max_feedback_passes": r["max_feedback_passes"]},
                "altitude_reference": "MSL = Copernicus GLO-30 height (EGM2008 geoid). AGL = MSL - planner "
                                      "ground (block-max DEM + lateral-buffer max filter)",
            },
            "limits": self.limits(),
            "search": self._search_metrics(search),
            "profile": None,
            "smoothing": None,
            "flight": None,
            "stages": {},
            "events": [],
            "status": {},
            "timing": {"region_load_s": round(ctx.load_s, 3), "buffered_field_s": round(field_s, 3),
                       "search_and_profile_s": round(plan_s, 3)},
        }

        if search.success and search.trajectories:
            result["stages"]["search"] = self._trajectory_stage(
                ctx, fld, search.trajectories, label="A* ground track (search altitude)")
        elif not search.success:
            partial = tuple(n.incoming_trajectory for n in search.best_nodes[1:] if n.incoming_trajectory is not None)
            if partial:
                result["stages"]["search_best_partial"] = self._trajectory_stage(
                    ctx, fld, partial, label="Best partial chain when the budget ended (diagnostic only, NOT a solution)")

        if profile is not None:
            result["profile"] = {
                "success": profile.success, "status": profile.status,
                "min_agl_m": _f(profile.minimum_agl_m), "runtime_s": round(profile.runtime_s, 4),
                "refinement_passes": profile.refinement_passes, "target_agl_m": profile.target_agl_m,
                "failure_reason_detail": profile.failure_reason_detail,
                "failure_location": self._lonlat_xy(ctx, profile.failure_location),
            }
            if profile.failure_location is not None:
                loc = self._lonlat_xy(ctx, profile.failure_location)
                result["events"].append({"kind": "profile_failure", **loc, "detail": profile.status})

        if plan.success:
            result["stages"]["profiled"] = self._trajectory_stage(
                ctx, fld, plan.trajectories, label="Terrain-following profile (piecewise-linear altitude)")
            final_points = None
            if r["smoothing"]:
                job.set_stage("smoothing")
                t0 = time.perf_counter()
                sm = apply_corridor_safe_local_bspline_smoothing(
                    plan.trajectories, ctx.terrain, config=cfg, speed_mps=HORIZONTAL_SPEED_MPS,
                    sample_step_m=1.0, max_bank_rate_deg_s=ROLL_RATE_CAP_DEG_S,
                    max_corridor_deviation_m=r["max_corridor_deviation_m"],
                )
                result["timing"]["smoothing_s"] = round(time.perf_counter() - t0, 3)
                result["smoothing"] = {
                    "junctions_total": sm.num_junctions_total, "junctions_smoothed": sm.num_junctions_smoothed,
                    "raw_max_bank_rate_deg_s": sm.raw_max_bank_rate_deg_s,
                    "smoothed_max_bank_rate_deg_s": sm.smoothed_max_bank_rate_deg_s,
                    "raw_max_bank_angle_deg": sm.raw_max_bank_angle_deg,
                    "smoothed_max_bank_angle_deg": sm.smoothed_max_bank_angle_deg,
                    "max_corridor_deviation_m": sm.max_corridor_deviation_m,
                    "corridor_safe": sm.corridor_safe, "min_agl_m": sm.min_agl_m, "mean_agl_m": sm.mean_agl_m,
                    "sample_step_m": 1.0,
                }
                final_points = sm.smoothed_points
                final_label = "Final: corridor-safe B-spline smoothed trajectory"
            else:
                # Smoothing disabled: use the smoothing module's own telemetry of
                # the unsmoothed profiled chain ('original_points'); nothing is
                # recomputed here.
                sm = apply_corridor_safe_local_bspline_smoothing(plan.trajectories, ctx.terrain, config=cfg, sample_step_m=1.0,
                        max_corridor_deviation_m=0.0)
                final_points = sm.original_points
                final_label = "Final: terrain-following profile (smoothing disabled)"
            job.set_stage("packaging")
            result["stages"]["final"] = self._telemetry_stage(ctx, final_points, final_label, r)
            result["flight"] = self._flight_metrics(final_points, straight, plan.trajectories)
            result["events"].extend(self._events(ctx, final_points))

        result["status"] = self._status(search, profile, result, r)
        result["timing"]["total_s"] = round(time.perf_counter() - t_total, 3)
        return result

    # ------------------------------------------------------------- packaging
    @staticmethod
    def _search_metrics(s) -> dict:
        return {
            "success": s.success, "status": s.status, "termination_reason": s.termination_reason,
            "expanded_nodes": s.expanded_nodes, "generated_neighbors": s.generated_neighbors,
            "rejected_neighbors": s.rejected_neighbors, "rejected_reason_counts": dict(s.rejected_reason_counts),
            "max_open_size": s.max_open_size, "runtime_s": round(s.runtime_s, 3),
            "unique_search_keys": s.unique_search_keys, "total_cost": _f(s.total_cost),
            "goal_xy_error_m": _f(s.goal_xy_error_m), "goal_z_error_m": _f(s.goal_z_error_m),
            "closest_xy_distance_to_goal_m": _f(s.closest_xy_distance_to_goal_m),
            "maximum_altitude_msl_m": _f(s.maximum_altitude_msl_m),
            "path_primitive_counts": s.path_primitive_counts if s.success else {},
            "primitive_count": len(s.trajectories),
            "continuous_path_length_m": _f(s.continuous_path_length_m) if s.trajectories else None,
        }

    def _lonlat_xy(self, ctx: RegionContext, xy) -> Optional[dict]:
        if xy is None:
            return None
        lon, lat = ctx.to_lonlat.transform(xy[0], xy[1])
        return {"lon": lon, "lat": lat, "x_m": xy[0], "y_m": xy[1]}

    def _trajectory_stage(self, ctx: RegionContext, fld, trajectories: Sequence[PhysicalTrajectory], label: str) -> dict:
        xs, ys, zs, ss = [], [], [], []
        cum = 0.0
        for traj in trajectories:
            samples = traj.samples if not xs else traj.samples[1:]
            for smp in samples:
                xs.append(smp.x_m); ys.append(smp.y_m); zs.append(smp.z_msl_m)
                ss.append(cum + smp.horizontal_distance_along_path_m)
            cum += traj.horizontal_arc_length_m
        x = np.asarray(xs); y = np.asarray(ys); z = np.asarray(zs); s = np.asarray(ss)
        tq = ctx.terrain
        ground = np.full(len(x), np.nan)
        for i in range(len(x)):
            rr, cc = tq.xy_to_rowcol(x[i], y[i])
            if tq.in_bounds_rowcol(rr, cc) and fld.valid[rr, cc]:
                ground[i] = fld.elevation_msl[rr, cc]
        idx = _decimate_indices(len(x), MAX_TRANSPORT_POINTS, [int(np.nanargmin(z - ground)) if np.isfinite(ground).any() else 0])
        lon, lat = ctx.to_lonlat.transform(x[idx], y[idx])
        g = ground[idx]
        return {
            "label": label, "n_source_samples": int(len(x)), "n": int(len(idx)),
            "columns": {
                "s": _r(s[idx], 1), "lon": _r(lon, 7), "lat": _r(lat, 7), "x": _r(x[idx], 2), "y": _r(y[idx], 2),
                "z": _r(z[idx], 2), "ground": _r(g, 2), "agl": _r(z[idx] - g, 2),
            },
        }

    def _telemetry_stage(self, ctx: RegionContext, pts, label: str, r: dict) -> dict:
        n = len(pts)
        cols = {k: np.fromiter((getattr(p, a) for p in pts), dtype=np.float64, count=n) for k, a in (
            ("s", "s_m"), ("t", "t_s"), ("x", "x_m"), ("y", "y_m"), ("z", "z_msl_m"), ("ground", "ground_m"),
            ("agl", "agl_m"), ("heading", "heading_deg"), ("bank", "bank_angle_deg"),
            ("roll_rate", "bank_rate_deg_s"), ("vz", "vz_mps"), ("load", "load_factor_g"),
            ("dev", "deviation_from_orig_m"))}
        trans = np.fromiter((p.is_in_transition for p in pts), dtype=bool, count=n)
        keep = [int(np.argmin(cols["agl"])), int(np.argmax(np.abs(cols["roll_rate"]))),
                int(np.argmax(np.abs(cols["bank"]))), int(np.argmax(cols["z"])), int(np.argmax(np.abs(cols["vz"])))]
        idx = _decimate_indices(n, MAX_TRANSPORT_POINTS, keep)
        lon, lat = ctx.to_lonlat.transform(cols["x"][idx], cols["y"][idx])
        dec = {"lon": 7, "lat": 7, "s": 1, "t": 2, "x": 2, "y": 2, "z": 2, "ground": 2, "agl": 2, "heading": 2,
               "bank": 2, "roll_rate": 2, "vz": 3, "load": 3, "dev": 2}
        out_cols = {"lon": _r(lon, 7), "lat": _r(lat, 7)}
        for k, v in cols.items():
            out_cols[k] = _r(v[idx], dec[k])
        out_cols["in_transition"] = [bool(b) for b in trans[idx]]
        return {"label": label, "n_source_samples": n, "source_sample_step_m": 1.0, "n": int(len(idx)),
                "columns": out_cols}

    @staticmethod
    def _flight_metrics(pts, straight_m: float, trajectories) -> dict:
        z = np.array([p.z_msl_m for p in pts]); g = np.array([p.ground_m for p in pts])
        agl = z - g
        vz = np.array([p.vz_mps for p in pts]); bank = np.array([p.bank_angle_deg for p in pts])
        rr = np.array([p.bank_rate_deg_s for p in pts]); load = np.array([p.load_factor_g for p in pts])
        dist = float(pts[-1].s_m)
        return {
            "distance_m": dist, "flight_time_s": float(pts[-1].t_s), "straight_distance_m": straight_m,
            "route_to_straight_ratio": dist / straight_m if straight_m > 0 else None,
            "min_agl_m": float(agl.min()), "mean_agl_m": float(agl.mean()), "max_agl_m": float(agl.max()),
            "min_msl_m": float(z.min()), "max_msl_m": float(z.max()), "mean_msl_m": float(z.mean()),
            "mean_ground_m": float(g.mean()), "max_ground_m": float(g.max()),
            "max_climb_rate_mps": float(vz.max()), "max_descent_rate_mps": float(vz.min()),
            "max_abs_bank_deg": float(np.abs(bank).max()), "max_abs_roll_rate_deg_s": float(np.abs(rr).max()),
            "max_load_factor_g": float(load.max()), "primitive_count": len(trajectories),
            "note": "Derived by planner.local_trajectory_smoothing telemetry (1 m samples)",
        }

    def _events(self, ctx: RegionContext, pts) -> List[dict]:
        def ev(kind, i, value, unit):
            p = pts[i]
            lon, lat = ctx.to_lonlat.transform(p.x_m, p.y_m)
            return {"kind": kind, "s": p.s_m, "lon": lon, "lat": lat, "x_m": p.x_m, "y_m": p.y_m,
                    "z_msl_m": p.z_msl_m, "agl_m": p.agl_m, "value": value, "unit": unit}
        agl = [p.agl_m for p in pts]
        rr = [abs(p.bank_rate_deg_s) for p in pts]
        zz = [p.z_msl_m for p in pts]
        i_agl = int(np.argmin(agl)); i_rr = int(np.argmax(rr)); i_z = int(np.argmax(zz))
        return [ev("min_agl", i_agl, agl[i_agl], "m"), ev("max_roll_rate", i_rr, rr[i_rr], "deg/s"),
                ev("max_altitude", i_z, zz[i_z], "m MSL")]

    def _status(self, search, profile, result: dict, r: dict) -> dict:
        checks = []
        planner_ok = bool(profile is not None and profile.success)
        if not search.success:
            verdict = "NO_ROUTE"
            headline = {
                "timeout": "Search budget ended (timeout) — not a proof that no route exists",
                "search_limit_reached": "Expansion limit reached — not a proof that no route exists",
                "no_path": "Search frontier exhausted inside the modelled space",
            }.get(search.status, f"Search ended: {search.termination_reason}")
        elif not planner_ok:
            verdict = "NO_ROUTE"
            headline = f"Ground track found but altitude profile failed ({profile.status if profile else 'n/a'})"
        else:
            fl = result["flight"]
            sm = result["smoothing"]
            checks.append({"id": "planner_validation", "label": "Planner continuous safety validation (profile stage)",
                           "ok": True, "value": f"min AGL {profile.minimum_agl_m:.1f} m"})
            checks.append({"id": "min_agl", "label": f"Final trajectory ≥ {r['min_agl_m']:.0f} m AGL",
                           "ok": fl["min_agl_m"] >= r["min_agl_m"] - 1e-6, "value": f"{fl['min_agl_m']:.1f} m"})
            if sm is not None:
                checks.append({"id": "corridor", "label": "Smoothing corridor safe",
                               "ok": bool(sm["corridor_safe"]),
                               "value": f"max deviation {sm['max_corridor_deviation_m']:.2f} m"})
            lim = self.limits()
            checks.append({"id": "vz", "label": "Vertical rate within ±5 m/s",
                           "ok": fl["max_climb_rate_mps"] <= lim["max_climb_rate_mps"] + 0.05
                           and -fl["max_descent_rate_mps"] <= lim["max_descent_rate_mps"] + 0.05,
                           "value": f"{fl['max_climb_rate_mps']:+.2f} / {fl['max_descent_rate_mps']:+.2f} m/s",
                           "advisory": True})
            checks.append({"id": "bank", "label": f"Bank ≤ {lim['max_bank_deg']:.0f}°",
                           "ok": fl["max_abs_bank_deg"] <= lim["max_bank_deg"] + 0.1,
                           "value": f"{fl['max_abs_bank_deg']:.1f}°", "advisory": True})
            checks.append({"id": "roll_rate", "label": f"Roll rate ≤ {lim['roll_rate_cap_deg_s']:.0f}°/s",
                           "ok": fl["max_abs_roll_rate_deg_s"] <= lim["roll_rate_cap_deg_s"] + 0.1,
                           "value": f"{fl['max_abs_roll_rate_deg_s']:.1f}°/s", "advisory": True,
                           "note": "Known open issue (project.md #1): smoothed roll rate exceeds the cap"})
            hard = [c for c in checks if not c.get("advisory")]
            if all(c["ok"] for c in hard):
                verdict = "SAFE" if all(c["ok"] for c in checks) else "SAFE_WITH_ADVISORIES"
                headline = "Terrain-safe route" + ("" if verdict == "SAFE" else " — dynamics advisories")
            else:
                verdict = "UNSAFE"
                headline = "Final trajectory violates a hard terrain constraint"
        return {"verdict": verdict, "headline": headline, "planner_success": planner_ok,
                "search_status": search.status, "termination_reason": search.termination_reason,
                "profile_status": profile.status if profile else None, "checks": checks}


def _f(v) -> Optional[float]:
    try:
        v = float(v)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def _r(arr, nd: int) -> list:
    a = np.round(np.asarray(arr, dtype=np.float64), nd)
    return [None if not math.isfinite(v) else (int(v) if nd == 0 else float(v)) for v in a.tolist()]


def _decimate_indices(n: int, max_points: int, keep: Sequence[int]) -> np.ndarray:
    if n <= max_points:
        return np.arange(n)
    stride = math.ceil(n / max_points)
    idx = set(range(0, n, stride))
    idx.add(n - 1)
    idx.update(int(k) for k in keep if 0 <= k < n)
    return np.array(sorted(idx), dtype=np.int64)
