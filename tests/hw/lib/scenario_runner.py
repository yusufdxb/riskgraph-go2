"""Shared helpers for hardware scenario scripts.

These helpers assume rclpy is already initialized by the caller. They wrap
the chunks of boilerplate that every hw scenario will need:

- `record_bag(out_dir, topics)` -> subprocess.Popen for `ros2 bag record`.
- `make_route_msg(...)`, `make_segment_msg(...)`, `make_event_msg(...)` -
  thin builders so individual scenarios stay focused on the assertion.
- `call_score_routes(...)` - synchronous wrapper around the `/riskgraph/score_routes`
  service.
- `write_verdict(out_dir, payload)` - dump a verdict.json blob.

Nothing here is RiskGraph-specific beyond the imports — the scenarios pick
the actual segment geometry and event positions.
"""
from __future__ import annotations

import json
import os
import shutil
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence


def make_run_dir(root: Path) -> Path:
    """Create tests/hw/runs/<timestamp>/ and return it."""
    ts = time.strftime("%Y%m%d_%H%M%S")
    run_dir = root / "runs" / ts
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "bag").mkdir(exist_ok=True)
    return run_dir


def record_bag(out_dir: Path, topics: Sequence[str]) -> Optional[subprocess.Popen]:
    """Start a ros2 bag record subprocess. Caller must terminate it.

    Returns None if `ros2` is not on PATH (so the script still runs in
    a no-bag dev mode without crashing).
    """
    if shutil.which("ros2") is None:
        return None
    bag_path = out_dir / "bag" / "run"
    cmd = ["ros2", "bag", "record", "-o", str(bag_path)] + list(topics)
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )
    # Give the recorder a beat to subscribe before the scenario starts.
    time.sleep(1.0)
    return proc


def stop_bag(proc: Optional[subprocess.Popen]) -> None:
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5.0)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass


def write_verdict(out_dir: Path, payload: dict) -> Path:
    p = out_dir / "verdict.json"
    p.write_text(json.dumps(payload, indent=2, sort_keys=True))
    return p


# ---------------------------------------------------------------------------
# ROS message builders. Importing rclpy/msgs lazily so this module is import-
# safe on machines without ROS for static linting / dry-run.
# ---------------------------------------------------------------------------

def _import_msgs():
    from std_msgs.msg import Header
    from geometry_msgs.msg import Point, Twist, Vector3
    from riskgraph_msgs.msg import (
        RiskEvent as RiskEventMsg,
        RiskFactor as RiskFactorMsg,
        Route as RouteMsg,
        RouteSegment as RouteSegmentMsg,
    )
    from riskgraph_msgs.srv import ScoreRoutes
    return {
        "Header": Header, "Point": Point, "Twist": Twist, "Vector3": Vector3,
        "RiskEventMsg": RiskEventMsg, "RiskFactorMsg": RiskFactorMsg,
        "RouteMsg": RouteMsg, "RouteSegmentMsg": RouteSegmentMsg,
        "ScoreRoutes": ScoreRoutes,
    }


@dataclass
class EventSpec:
    event_id: str
    x: float
    y: float
    severity: float = 0.9
    category: str = "SLIP"
    source: str = "tactile/slip_state"
    detail: str = "scripted hardware scenario"


@dataclass
class SegmentSpec:
    segment_id: str
    x0: float
    y0: float
    x1: float
    y1: float
    semantic_label: str = ""


def build_event_msg(spec: EventSpec, frame_id: str = "map"):
    M = _import_msgs()
    msg = M["RiskEventMsg"]()
    msg.header = M["Header"]()
    now = time.time()
    msg.header.stamp.sec = int(now)
    msg.header.stamp.nanosec = int((now - int(now)) * 1e9)
    msg.header.frame_id = frame_id
    msg.event_id = spec.event_id
    msg.position = M["Point"](x=float(spec.x), y=float(spec.y), z=0.0)
    f = M["RiskFactorMsg"]()
    f.category = spec.category
    f.severity = float(spec.severity)
    f.source = spec.source
    f.detail = spec.detail
    msg.factors = [f]
    msg.confidence = 1.0
    return msg


def build_segment_msg(spec: SegmentSpec):
    M = _import_msgs()
    s = M["RouteSegmentMsg"]()
    s.segment_id = spec.segment_id
    s.start = M["Point"](x=float(spec.x0), y=float(spec.y0), z=0.0)
    s.end = M["Point"](x=float(spec.x1), y=float(spec.y1), z=0.0)
    s.length_m = ((spec.x1 - spec.x0) ** 2 + (spec.y1 - spec.y0) ** 2) ** 0.5
    s.semantic_label = spec.semantic_label
    return s


def build_route_msg(route_id: str, segments: Sequence[SegmentSpec], frame_id: str = "map"):
    M = _import_msgs()
    r = M["RouteMsg"]()
    r.header = M["Header"]()
    r.header.frame_id = frame_id
    r.route_id = route_id
    r.segments = [build_segment_msg(s) for s in segments]
    return r


def build_twist(vx: float, wz: float = 0.0):
    M = _import_msgs()
    t = M["Twist"]()
    t.linear = M["Vector3"](x=float(vx), y=0.0, z=0.0)
    t.angular = M["Vector3"](x=0.0, y=0.0, z=float(wz))
    return t


def call_score_routes(client, candidates: List, semantic_objective: str = "",
                      timeout_s: float = 10.0):
    """Block-call the ScoreRoutes service. Returns (response_or_None, error_str)."""
    M = _import_msgs()
    if not client.wait_for_service(timeout_sec=3.0):
        return None, "service /riskgraph/score_routes did not appear within 3s"
    req = M["ScoreRoutes"].Request()
    req.candidates = candidates
    req.semantic_objective = semantic_objective
    fut = client.call_async(req)
    deadline = time.time() + timeout_s
    while not fut.done() and time.time() < deadline:
        time.sleep(0.05)
    if not fut.done():
        return None, f"service call did not complete in {timeout_s:.1f}s"
    return fut.result(), None


def assert_chosen(resp, expected_route_id: str, require_evidence: bool = True) -> tuple:
    """Returns (ok: bool, reason: str)."""
    if resp is None:
        return False, "no response"
    chosen = resp.result.chosen_route_id
    if chosen != expected_route_id:
        return False, f"chosen_route_id={chosen!r}, expected {expected_route_id!r}"
    if require_evidence and len(resp.explanation.evidence_event_ids) < 1:
        return False, "explanation has no evidence_event_ids"
    return True, "ok"
