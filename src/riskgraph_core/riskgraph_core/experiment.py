"""The canonical two-corridor experiment: geometry, map, and verdict logic.

One physical course, laid out on the lab floor with tape:

* start marker **A** and goal marker **B** on the x axis of the ``map`` frame;
* one physical obstacle (a box) between them, slightly off the axis, so the
  course has exactly two feasible corridors, one on each side;
* a rectangular cleared area; everything outside it is occupied in the map.

The ``map`` frame is *defined* by marker A: the robot is anchored there at
the start of the session (see ``riskgraph_nav.localization_node``). The
static map is generated from this file, so the map, the anchor, the goal and
the map identity all come from one source of truth.

Everything here is pure Python so the pass/fail logic that decides the lab
verdict is unit tested before the robot is involved.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import yaml

from .geometry import Pose2D, point_to_polyline_distance, polyline_length, polyline_separation
from .risk_field import GridInfo, RiskField, RiskFieldParams

XY = Tuple[float, float]


class ExperimentError(ValueError):
    """The experiment file is missing or inconsistent."""


@dataclass(frozen=True)
class Rect:
    cx: float
    cy: float
    size_x: float
    size_y: float

    @property
    def x_min(self) -> float:
        return self.cx - self.size_x / 2.0

    @property
    def x_max(self) -> float:
        return self.cx + self.size_x / 2.0

    @property
    def y_min(self) -> float:
        return self.cy - self.size_y / 2.0

    @property
    def y_max(self) -> float:
        return self.cy + self.size_y / 2.0

    def contains(self, x: float, y: float) -> bool:
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max


@dataclass(frozen=True)
class Area:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def contains(self, x: float, y: float) -> bool:
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max


@dataclass(frozen=True)
class Experiment:
    name: str
    path: str                     # absolute path of the experiment file
    frame_id: str
    map_yaml: str                 # absolute path
    resolution: float
    border_m: float
    area: Area
    obstacle: Rect
    start: Pose2D
    start_marker: str
    goal: Pose2D
    goal_marker: str
    start_xy_tol_m: float
    start_yaw_tol_rad: float
    goal_xy_tol_m: float
    capture_radius_m: float
    injection_severity: float
    risk_params: RiskFieldParams
    min_route_separation_m: float
    max_cross_track_m: float
    max_speed_mps: float
    stationary_speed_mps: float
    max_execution_s: float
    max_anchor_drift_m: float
    max_anchor_drift_yaw_rad: float
    raw: Dict = field(default_factory=dict, compare=False, repr=False)

    @property
    def anchor(self) -> Dict[str, object]:
        return {"marker": self.start_marker, "x": self.start.x, "y": self.start.y,
                "yaw": self.start.yaw}

    @property
    def gate_x(self) -> float:
        """x where the two corridors are furthest apart: the obstacle centre."""
        return self.obstacle.cx

    def corridor_points(self) -> Dict[str, XY]:
        """Mid-corridor reference points beside the obstacle (for reports)."""
        left_y = (self.obstacle.y_max + self.area.y_max) / 2.0
        right_y = (self.obstacle.y_min + self.area.y_min) / 2.0
        return {"left": (self.gate_x, left_y), "right": (self.gate_x, right_y)}


def _f(d: Dict, key: str, where: str) -> float:
    try:
        v = float(d[key])
    except (KeyError, TypeError, ValueError) as exc:
        raise ExperimentError(f"{where}.{key}: missing or not a number") from exc
    if not math.isfinite(v):
        raise ExperimentError(f"{where}.{key}: not finite")
    return v


def load_experiment(path: str) -> Experiment:
    if not path or not os.path.isfile(path):
        raise ExperimentError(f"experiment file not found: {path!r}")
    with open(path, "r", encoding="utf-8") as fh:
        raw = yaml.safe_load(fh) or {}
    e = raw.get("experiment")
    if not isinstance(e, dict):
        raise ExperimentError(f"{path}: top-level 'experiment' mapping missing")
    base = os.path.dirname(os.path.abspath(path))
    map_yaml = str((e.get("map") or {}).get("yaml", ""))
    if not map_yaml:
        raise ExperimentError(f"{path}: experiment.map.yaml missing")
    if not os.path.isabs(map_yaml):
        map_yaml = os.path.normpath(os.path.join(base, map_yaml))

    area_d = e.get("area") or {}
    area = Area(_f(area_d, "x_min", "area"), _f(area_d, "x_max", "area"),
                _f(area_d, "y_min", "area"), _f(area_d, "y_max", "area"))
    ob_d = e.get("obstacle") or {}
    obstacle = Rect(_f(ob_d, "cx", "obstacle"), _f(ob_d, "cy", "obstacle"),
                    _f(ob_d, "size_x", "obstacle"), _f(ob_d, "size_y", "obstacle"))
    s = e.get("start") or {}
    g = e.get("goal") or {}
    tol = e.get("tolerances") or {}
    inj = e.get("injection") or {}
    rf = e.get("risk_field") or {}
    acc = e.get("acceptance") or {}
    try:
        risk_params = RiskFieldParams(
            radius_m=float(rf.get("radius_m", 0.8)),
            decay_half_life_s=float(rf.get("decay_half_life_s", 0.0)),
            value_per_unit_risk=float(rf.get("value_per_unit_risk", 90.0)),
            max_cell_value=int(rf.get("max_cell_value", 90)),
        )
    except ValueError as exc:
        raise ExperimentError(f"{path}: risk_field: {exc}") from exc

    exp = Experiment(
        name=str(e.get("name", "unnamed")),
        path=os.path.abspath(path),
        frame_id=str(e.get("frame_id", "map")),
        map_yaml=map_yaml,
        resolution=float(e.get("resolution", 0.05)),
        border_m=float(e.get("border_m", 0.5)),
        area=area,
        obstacle=obstacle,
        start=Pose2D(_f(s, "x", "start"), _f(s, "y", "start"), _f(s, "yaw", "start")),
        start_marker=str(s.get("marker", "A")),
        goal=Pose2D(_f(g, "x", "goal"), _f(g, "y", "goal"), _f(g, "yaw", "goal")),
        goal_marker=str(g.get("marker", "B")),
        start_xy_tol_m=float(tol.get("start_xy_m", 0.30)),
        start_yaw_tol_rad=float(tol.get("start_yaw_rad", 0.35)),
        goal_xy_tol_m=float(tol.get("goal_xy_m", 0.35)),
        capture_radius_m=float(inj.get("capture_radius_m", 0.35)),
        injection_severity=float(inj.get("severity", 1.0)),
        risk_params=risk_params,
        min_route_separation_m=float(acc.get("min_route_separation_m", 0.30)),
        max_cross_track_m=float(acc.get("max_cross_track_m", 0.75)),
        max_speed_mps=float(acc.get("max_speed_mps", 0.35)),
        stationary_speed_mps=float(acc.get("stationary_speed_mps", 0.05)),
        max_execution_s=float(acc.get("max_execution_s", 120.0)),
        max_anchor_drift_m=float(acc.get("max_anchor_drift_m", 0.5)),
        max_anchor_drift_yaw_rad=float(acc.get("max_anchor_drift_yaw_rad", 0.35)),
        raw=raw,
    )
    problems = check_experiment(exp)
    if problems:
        raise ExperimentError(f"{path}: " + "; ".join(problems))
    return exp


def check_experiment(exp: Experiment) -> List[str]:
    """Geometric sanity: the course must actually have two corridors."""
    p: List[str] = []
    a, o = exp.area, exp.obstacle
    if a.x_max <= a.x_min or a.y_max <= a.y_min:
        p.append("area is empty")
    if not (a.contains(exp.start.x, exp.start.y) and a.contains(exp.goal.x, exp.goal.y)):
        p.append("start and goal must be inside the area")
    if o.contains(exp.start.x, exp.start.y) or o.contains(exp.goal.x, exp.goal.y):
        p.append("start/goal inside the obstacle")
    if not (min(exp.start.x, exp.goal.x) < o.x_min and o.x_max < max(exp.start.x, exp.goal.x)):
        p.append("obstacle must sit between start and goal along x")
    if not (a.y_min < o.y_min and o.y_max < a.y_max):
        p.append("obstacle must leave a corridor on both sides")
    for side, width in (("left", a.y_max - o.y_max), ("right", o.y_min - a.y_min)):
        if width < 0.9:
            p.append(f"{side} corridor is {width:.2f} m wide; need >= 0.9 m for a GO2 plus inflation")
    if exp.resolution <= 0 or exp.resolution > 0.2:
        p.append("resolution must be in (0, 0.2] m")
    if exp.frame_id != "map":
        p.append("frame_id must be 'map'")
    return p


# -- course map -----------------------------------------------------------

def course_grid(exp: Experiment) -> Tuple[GridInfo, List[int]]:
    """Occupancy (0 free / 100 occupied) of the course, row-major, origin bottom-left."""
    res = exp.resolution
    x0 = exp.area.x_min - exp.border_m
    y0 = exp.area.y_min - exp.border_m
    w = int(round((exp.area.x_max - exp.area.x_min + 2 * exp.border_m) / res))
    h = int(round((exp.area.y_max - exp.area.y_min + 2 * exp.border_m) / res))
    info = GridInfo(resolution=res, width=w, height=h, origin_x=x0, origin_y=y0)
    data = []
    for iy in range(h):
        for ix in range(w):
            cx, cy = info.cell_center(ix, iy)
            free = exp.area.contains(cx, cy) and not exp.obstacle.contains(cx, cy)
            data.append(0 if free else 100)
    return info, data


def write_course_map(exp: Experiment, yaml_path: Optional[str] = None) -> Tuple[str, str]:
    """Write the course as a map_server PGM + YAML. Returns (yaml, pgm) paths."""
    yaml_path = yaml_path or exp.map_yaml
    info, data = course_grid(exp)
    stem = os.path.splitext(yaml_path)[0]
    pgm_path = stem + ".pgm"
    os.makedirs(os.path.dirname(os.path.abspath(yaml_path)), exist_ok=True)
    rows = []
    for row in range(info.height):
        iy = info.height - 1 - row  # PGM row 0 is the top (max y)
        rows.append(bytes(254 if data[iy * info.width + ix] == 0 else 0
                          for ix in range(info.width)))
    with open(pgm_path, "wb") as fh:
        fh.write(f"P5\n# RiskGraph course {exp.name}\n{info.width} {info.height}\n255\n".encode())
        fh.write(b"".join(rows))
    meta = {
        "image": os.path.basename(pgm_path),
        "mode": "trinary",
        "resolution": info.resolution,
        "origin": [info.origin_x, info.origin_y, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    with open(yaml_path, "w", encoding="utf-8") as fh:
        fh.write(f"# Generated from {os.path.basename(exp.path)} by riskgraph_generate_course_map.\n")
        fh.write("# Regenerate instead of editing: the map id is a hash of this content.\n")
        yaml.safe_dump(meta, fh, sort_keys=True)
    return yaml_path, pgm_path


# -- route analysis ---------------------------------------------------------

def route_side(points: Sequence[XY], exp: Experiment) -> str:
    """Which corridor a route uses: 'left' (+y), 'right' (-y), or 'none'.

    Decided by the path samples whose x lies within the obstacle's x span.
    'mixed' means samples on both sides, which a sane route cannot produce.
    """
    o = exp.obstacle
    ys = [y for x, y in points if o.x_min <= x <= o.x_max]
    if not ys:
        return "none"
    left = sum(1 for y in ys if y > o.cy)
    right = len(ys) - left
    if left >= 0.9 * len(ys):
        return "left"
    if right >= 0.9 * len(ys):
        return "right"
    return "mixed"


def validate_path(points: Sequence[XY], frame_id: str, exp: Experiment,
                  robot_xy: Optional[XY], goal_xy: XY,
                  is_lethal: Optional[Callable[[float, float], bool]] = None,
                  start_tol_m: float = 0.5) -> List[str]:
    """Reasons a planned route is malformed; empty list means acceptable."""
    problems: List[str] = []
    if frame_id != exp.frame_id:
        problems.append(f"path frame {frame_id!r} != {exp.frame_id!r}")
    if len(points) < 2:
        problems.append(f"path has {len(points)} poses")
        return problems
    if not all(math.isfinite(v) for p in points for v in p):
        problems.append("path contains non-finite coordinates")
        return problems
    if robot_xy is not None:
        d0 = math.hypot(points[0][0] - robot_xy[0], points[0][1] - robot_xy[1])
        if d0 > start_tol_m:
            problems.append(f"path starts {d0:.2f} m from the robot")
    dg = math.hypot(points[-1][0] - goal_xy[0], points[-1][1] - goal_xy[1])
    if dg > exp.goal_xy_tol_m:
        problems.append(f"path ends {dg:.2f} m from the goal")
    gaps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:])]
    if gaps and max(gaps) > 0.5:
        problems.append(f"path has a {max(gaps):.2f} m jump between poses")
    border = exp.border_m
    outside = [p for p in points if not (exp.area.x_min - border <= p[0] <= exp.area.x_max + border
                                         and exp.area.y_min - border <= p[1] <= exp.area.y_max + border)]
    if outside:
        problems.append(f"{len(outside)} poses outside the map")
    straight = math.hypot(goal_xy[0] - points[0][0], goal_xy[1] - points[0][1])
    length = polyline_length(points)
    if straight > 0.5 and length > 3.0 * straight:
        problems.append(f"path length {length:.2f} m is > 3x the straight line {straight:.2f} m")
    if is_lethal is not None:
        lethal = [p for p in points if is_lethal(p[0], p[1])]
        if lethal:
            problems.append(f"{len(lethal)} poses in occupied map cells")
    return problems


def cross_track_error(pose_xy: XY, path: Sequence[XY]) -> float:
    return point_to_polyline_distance(pose_xy, path)


def capture_point_on_path(path: Sequence[XY], exp: Experiment) -> Optional[XY]:
    """Where on a route the risk observation is to be captured: the path
    sample closest to the obstacle centre line x = gate_x."""
    if not path:
        return None
    return min(path, key=lambda p: abs(p[0] - exp.gate_x))


def nearest_sample(samples: Sequence[Dict], target: XY) -> Optional[Dict]:
    """The trajectory sample (dict with x, y) closest to ``target``."""
    best, best_d = None, math.inf
    for s in samples:
        d = math.hypot(s["x"] - target[0], s["y"] - target[1])
        if d < best_d:
            best, best_d = s, d
    if best is None:
        return None
    out = dict(best)
    out["distance_to_target_m"] = best_d
    return out


def compare_routes(baseline: Sequence[XY], aware: Sequence[XY], field: RiskField,
                   exp: Experiment) -> Dict[str, object]:
    """Trial C verdict: did remembered risk change the route, and for the better?

    Both routes are evaluated against the SAME current risk field, so the
    comparison is "what the risk-aware planner avoided", not two numbers from
    two different fields.
    """
    b_side, a_side = route_side(baseline, exp), route_side(aware, exp)
    mean_sep, hausdorff = polyline_separation(aware, baseline)
    bm = field.path_metrics(baseline)
    am = field.path_metrics(aware)
    b_risk = float(bm["accumulated_risk"])
    a_risk = float(am["accumulated_risk"])
    risk_reduced = a_risk < b_risk - 1e-6
    geometry_changed = (b_side != a_side and a_side in ("left", "right")) or \
        mean_sep >= exp.min_route_separation_m
    return {
        "baseline_side": b_side,
        "aware_side": a_side,
        "side_changed": b_side != a_side,
        "mean_separation_m": mean_sep,
        "hausdorff_m": hausdorff,
        "baseline_metrics": bm,
        "aware_metrics": am,
        "baseline_accumulated_risk": b_risk,
        "aware_accumulated_risk": a_risk,
        "risk_reduced": risk_reduced,
        "geometry_changed": geometry_changed,
        "pass": bool(risk_reduced and geometry_changed),
    }


def fallback_checks(path: Sequence[XY], plans_repeat: Sequence[Sequence[XY]],
                    grid_data: Sequence[int], resolution: float) -> Dict[str, object]:
    """Trial E: risk everywhere must degrade to a valid, stable plan."""
    finite = all(math.isfinite(v) for p in path for v in p)
    max_dev = 0.0
    for other in plans_repeat:
        if len(other) != len(path):
            max_dev = math.inf
            break
        for p, q in zip(path, other):
            max_dev = max(max_dev, math.hypot(p[0] - q[0], p[1] - q[1]))
    lethal_cells = sum(1 for v in grid_data if v >= 100)
    return {
        "path_poses": len(path),
        "path_finite": finite,
        "repeat_plans": len(plans_repeat),
        "max_repeat_deviation_m": max_dev,
        "deterministic": max_dev <= resolution + 1e-9,
        "risk_lethal_cells": lethal_cells,
        "pass": bool(len(path) >= 2 and finite and max_dev <= resolution + 1e-9
                     and lethal_cells == 0),
    }
