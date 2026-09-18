"""Pure logic for anchored-odometry localization (no ROS imports).

A stock GO2 publishes ``/utlidar/robot_odom`` (``odom`` -> ``base_link``,
origin at the robot's boot pose) and nothing else: no ``/tf``, no ``map``.
RiskGraph needs a map frame that means the same physical place on every run,
so the map frame is defined by a floor marker: with the robot standing still
on marker A, ``T_map_odom`` is solved so the robot's map pose equals the
marker pose, and then held fixed.

This file decides three things, each of which invalidates the trial when it
goes wrong:

* is the odometry stream alive and sane (rate, age, finite values, frames);
* did odometry jump (a reset or a glitch moves every map pose at once, so a
  jump LATCHES localization invalid until the operator re-anchors);
* is the robot stationary enough to anchor.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Optional, Tuple

from riskgraph_core.geometry import Pose2D, anchor_map_to_odom, normalize_angle, quat_is_valid, yaw_from_quat

OK = "OK"
NO_DATA = "NO_DATA"
STALE = "STALE"
JUMP = "JUMP"
UNANCHORED = "UNANCHORED"


@dataclass(frozen=True)
class OdomSample:
    t: float            # receipt time on the local clock
    pose: Pose2D
    z: float
    robot_stamp: float  # header stamp from the robot, 0 if unset


class OdomMonitor:
    """Validates and tracks the odometry stream on the receiving clock."""

    def __init__(self, odom_frame: str = "odom", base_frame: str = "base_link",
                 max_jump_m: float = 0.25, max_jump_yaw: float = 0.5,
                 stale_timeout_s: float = 0.25, rate_window_s: float = 1.0) -> None:
        self.odom_frame = odom_frame
        self.base_frame = base_frame
        self.max_jump_m = max_jump_m
        self.max_jump_yaw = max_jump_yaw
        self.stale_timeout_s = stale_timeout_s
        self.rate_window_s = rate_window_s
        self.last: Optional[OdomSample] = None
        self._times: Deque[float] = deque()
        self.accepted = 0
        self.invalid = 0
        self.frame_mismatch = 0
        self.jumps = 0
        self.gaps = 0
        self.jump_latched = False
        self.last_jump: Optional[Dict[str, float]] = None
        self.robot_clock_skew_s: Optional[float] = None

    def update(self, t: float, x: float, y: float, z: float,
               q: Tuple[float, float, float, float], frame: str, child: str,
               robot_stamp: float = 0.0) -> Optional[str]:
        """Feed one sample. Returns None if accepted, else the rejection reason
        ('INVALID', 'FRAME', or 'JUMP'; a JUMP sample is accepted as the new
        reference but latches the monitor)."""
        if frame != self.odom_frame or child != self.base_frame:
            self.frame_mismatch += 1
            return "FRAME"
        vals = (t, x, y, z) + tuple(q)
        if not all(math.isfinite(v) for v in vals) or not quat_is_valid(q, tol=1e-2):
            self.invalid += 1
            return "INVALID"
        pose = Pose2D(x, y, yaw_from_quat(q))
        result = None
        if self.last is not None:
            dt = t - self.last.t
            if dt > self.stale_timeout_s:
                self.gaps += 1
            d = pose.distance_to(self.last.pose)
            dyaw = abs(normalize_angle(pose.yaw - self.last.pose.yaw))
            if d > self.max_jump_m or dyaw > self.max_jump_yaw:
                self.jumps += 1
                self.jump_latched = True
                self.last_jump = {"t": t, "dt": dt, "distance_m": d, "yaw_rad": dyaw}
                result = "JUMP"
        if robot_stamp > 0.0:
            self.robot_clock_skew_s = t - robot_stamp
        self.last = OdomSample(t, pose, z, robot_stamp)
        self.accepted += 1
        self._times.append(t)
        while self._times and self._times[0] < t - self.rate_window_s:
            self._times.popleft()
        return result

    def rate_hz(self, now: float) -> float:
        n = sum(1 for s in self._times if s >= now - self.rate_window_s)
        return n / self.rate_window_s

    def age_s(self, now: float) -> Optional[float]:
        return None if self.last is None else now - self.last.t

    def state(self, now: float) -> str:
        if self.last is None:
            return NO_DATA
        if self.jump_latched:
            return JUMP
        if now - self.last.t > self.stale_timeout_s:
            return STALE
        return OK

    def clear_jump(self) -> None:
        self.jump_latched = False


class StationaryDetector:
    """True once the robot has held still for ``window_s``."""

    def __init__(self, window_s: float = 1.0, max_disp_m: float = 0.02,
                 max_yaw_rad: float = 0.02) -> None:
        self.window_s = window_s
        self.max_disp_m = max_disp_m
        self.max_yaw_rad = max_yaw_rad
        self._buf: Deque[Tuple[float, Pose2D]] = deque()

    def update(self, t: float, pose: Pose2D) -> None:
        self._buf.append((t, pose))
        while self._buf and self._buf[0][0] < t - 3.0 * self.window_s:
            self._buf.popleft()

    def reset(self) -> None:
        self._buf.clear()

    def speed_mps(self, span_s: float = 0.2) -> Optional[float]:
        if len(self._buf) < 2:
            return None
        t1, p1 = self._buf[-1]
        for t0, p0 in reversed(self._buf):
            if t1 - t0 >= span_s:
                return p1.distance_to(p0) / (t1 - t0)
        t0, p0 = self._buf[0]
        return p1.distance_to(p0) / (t1 - t0) if t1 > t0 else None

    def is_stationary(self, now: float) -> bool:
        pts = [(t, p) for t, p in self._buf if t >= now - self.window_s]
        if len(pts) < 3 or pts[-1][0] - pts[0][0] < 0.8 * self.window_s:
            return False
        ref = pts[-1][1]
        return all(p.distance_to(ref) <= self.max_disp_m and
                   abs(normalize_angle(p.yaw - ref.yaw)) <= self.max_yaw_rad for _, p in pts)


@dataclass(frozen=True)
class AnchorRecord:
    epoch: int
    marker: str
    marker_in_map: Pose2D
    base_in_odom: Pose2D
    map_to_odom: Pose2D
    t: float

    def as_dict(self) -> Dict[str, object]:
        def p(v: Pose2D):
            return {"x": v.x, "y": v.y, "yaw": v.yaw}
        return {"epoch": self.epoch, "marker": self.marker, "marker_in_map": p(self.marker_in_map),
                "base_in_odom_at_anchor": p(self.base_in_odom),
                "map_to_odom": p(self.map_to_odom), "anchored_at": self.t}


def make_anchor(epoch: int, marker: str, marker_in_map: Pose2D, base_in_odom: Pose2D,
                t: float) -> AnchorRecord:
    return AnchorRecord(epoch, marker, marker_in_map, base_in_odom,
                        anchor_map_to_odom(marker_in_map, base_in_odom), t)
