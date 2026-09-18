"""Frame math used on both sides of the ROS boundary (no ROS imports).

Everything that turns a pose in one frame into a pose in another goes
through here, so the rotation and composition conventions are tested once
instead of being re-derived inside each node.

Conventions: quaternions are ``(x, y, z, w)``; a transform ``T_a_b`` maps a
point expressed in frame ``b`` into frame ``a`` (the same meaning as a TF
lookup of target ``a``, source ``b``).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

Point3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]


def normalize_angle(a: float) -> float:
    """Wrap to (-pi, pi]."""
    a = math.fmod(float(a) + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def yaw_from_quat(q: Quat) -> float:
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw: float) -> Quat:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_is_valid(q: Quat, tol: float = 1e-3) -> bool:
    if not all(math.isfinite(v) for v in q):
        return False
    n = math.sqrt(sum(v * v for v in q))
    return abs(n - 1.0) < tol


def rotate(q: Quat, p: Point3) -> Point3:
    """Rotate point ``p`` by unit quaternion ``q``."""
    x, y, z, w = q
    px, py, pz = p
    # t = 2 * cross(q.xyz, p)
    tx = 2.0 * (y * pz - z * py)
    ty = 2.0 * (z * px - x * pz)
    tz = 2.0 * (x * py - y * px)
    # p' = p + w * t + cross(q.xyz, t)
    return (
        px + w * tx + (y * tz - z * ty),
        py + w * ty + (z * tx - x * tz),
        pz + w * tz + (x * ty - y * tx),
    )


def apply_transform(translation: Point3, rotation: Quat, p: Point3) -> Point3:
    """``T * p`` for a transform given as (translation, rotation)."""
    r = rotate(rotation, p)
    return (r[0] + translation[0], r[1] + translation[1], r[2] + translation[2])


@dataclass(frozen=True)
class Pose2D:
    """Planar pose ``(x, y, yaw)``; also used as a planar transform."""

    x: float
    y: float
    yaw: float

    def compose(self, other: "Pose2D") -> "Pose2D":
        """``self * other``: apply ``other`` in the frame of ``self``."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(
            self.x + c * other.x - s * other.y,
            self.y + s * other.x + c * other.y,
            normalize_angle(self.yaw + other.yaw),
        )

    def inverse(self) -> "Pose2D":
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(-(c * self.x + s * self.y), -(-s * self.x + c * self.y),
                      normalize_angle(-self.yaw))

    def transform_point(self, px: float, py: float) -> Tuple[float, float]:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (self.x + c * px - s * py, self.y + s * px + c * py)

    def distance_to(self, other: "Pose2D") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def heading_error(self, other: "Pose2D") -> float:
        return abs(normalize_angle(self.yaw - other.yaw))

    def is_finite(self) -> bool:
        return all(math.isfinite(v) for v in (self.x, self.y, self.yaw))


def anchor_map_to_odom(marker_in_map: Pose2D, base_in_odom: Pose2D) -> Pose2D:
    """``T_map_odom`` given that the robot base currently sits on the marker.

    With the robot on the marker, ``T_map_base = marker_in_map`` and
    ``T_odom_base = base_in_odom``, so
    ``T_map_odom = T_map_base * inverse(T_odom_base)``.
    """
    return marker_in_map.compose(base_in_odom.inverse())


def polyline_length(points: Sequence[Tuple[float, float]]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


def resample_polyline(points: Sequence[Tuple[float, float]], step: float
                      ) -> List[Tuple[float, float]]:
    """Points every ``step`` metres along the polyline, endpoints included."""
    if step <= 0:
        raise ValueError("step must be positive")
    pts = [(float(x), float(y)) for x, y in points]
    if len(pts) < 2:
        return list(pts)
    out = [pts[0]]
    carry = 0.0
    for a, b in zip(pts, pts[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if seg <= 0.0:
            continue
        d = step - carry
        while d <= seg:
            t = d / seg
            out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
            d += step
        carry = seg - (d - step)
    if out[-1] != pts[-1]:
        out.append(pts[-1])
    return out


def point_to_polyline_distance(p: Tuple[float, float],
                               points: Sequence[Tuple[float, float]]) -> float:
    """Distance from ``p`` to the closest point of a polyline (inf if empty)."""
    if not points:
        return math.inf
    if len(points) == 1:
        return math.hypot(p[0] - points[0][0], p[1] - points[0][1])
    best = math.inf
    for a, b in zip(points, points[1:]):
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 <= 1e-18:
            d = math.hypot(p[0] - ax, p[1] - ay)
        else:
            t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
            d = math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))
        best = min(best, d)
    return best


def polyline_separation(a: Sequence[Tuple[float, float]], b: Sequence[Tuple[float, float]],
                        step: float = 0.05) -> Tuple[float, float]:
    """(mean, max) distance from points of ``a`` to polyline ``b``.

    The max over both directions is the discrete Hausdorff distance, returned
    as the second value; the first is the mean of the ``a -> b`` distances.
    """
    if not a or not b:
        return (math.inf, math.inf)
    ra = resample_polyline(a, step) if len(a) > 1 else list(a)
    rb = resample_polyline(b, step) if len(b) > 1 else list(b)
    d_ab = [point_to_polyline_distance(p, b) for p in ra]
    d_ba = [point_to_polyline_distance(p, a) for p in rb]
    return (sum(d_ab) / len(d_ab), max(max(d_ab), max(d_ba)))


def all_finite(values: Iterable[float]) -> bool:
    try:
        return all(math.isfinite(float(v)) for v in values)
    except (TypeError, ValueError):
        return False
