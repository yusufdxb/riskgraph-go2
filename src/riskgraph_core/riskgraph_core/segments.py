"""Route segment geometry and spatial assignment.

A segment is a directed line segment between two map-frame points. Routes
are ordered lists of segments. `segment_for_point` performs a nearest-segment
spatial join used by the memory ingestion path to associate an incoming
event's pose with a known segment id.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Tuple

Point3 = Tuple[float, float, float]


def _dist(a: Point3, b: Point3) -> float:
    dx, dy, dz = a[0] - b[0], a[1] - b[1], a[2] - b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _point_segment_distance(p: Point3, a: Point3, b: Point3) -> float:
    """Euclidean distance from point p to the closed line segment a→b."""
    ax, ay, az = a
    bx, by, bz = b
    px, py, pz = p
    abx, aby, abz = bx - ax, by - ay, bz - az
    apx, apy, apz = px - ax, py - ay, pz - az
    ab_len_sq = abx * abx + aby * aby + abz * abz
    if ab_len_sq <= 1e-12:
        return _dist(p, a)
    t = (apx * abx + apy * aby + apz * abz) / ab_len_sq
    t = max(0.0, min(1.0, t))
    cx = ax + abx * t
    cy = ay + aby * t
    cz = az + abz * t
    return _dist(p, (cx, cy, cz))


@dataclass
class RouteSegment:
    segment_id: str
    start: Point3
    end: Point3
    semantic_label: Optional[str] = None

    @property
    def length_m(self) -> float:
        return _dist(self.start, self.end)


@dataclass
class Route:
    route_id: str
    segments: List[RouteSegment] = field(default_factory=list)

    @property
    def total_length_m(self) -> float:
        return sum(s.length_m for s in self.segments)

    def labels(self) -> List[str]:
        return [s.semantic_label for s in self.segments if s.semantic_label]


def segment_for_point(segments: Iterable[RouteSegment], point: Point3) -> Optional[RouteSegment]:
    """Return the nearest segment to `point`. None if `segments` is empty."""
    best: Optional[RouteSegment] = None
    best_d = math.inf
    for s in segments:
        d = _point_segment_distance(point, s.start, s.end)
        if d < best_d:
            best_d = d
            best = s
    return best
