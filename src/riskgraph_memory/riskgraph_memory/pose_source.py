"""Bounded-age odometry cache and the unavailable-pose policy.

Adapters translate upstream events (safety alerts, faults, slip flags) into
``riskgraph_msgs/RiskEvent``. Those upstream messages do not carry a robot
pose, so until now every adapter published ``position = (0, 0, 0)`` stamped
``frame_id = "map"``. That is not a missing value, it is a *wrong* value: the
memory node's spatial join happily bound every unposed event to whichever
seeded segment happens to sit nearest the origin, so a slip in the far
hallway was recorded against the segment at (0, 0).

This module holds the pose side of the fix, deliberately free of any ROS
import so the policy is unit-testable without rclpy. The ROS subscription
that feeds it lives in ``adapters.pose_tagging``.

Policy, in one paragraph. A pose is usable only if it was received, carries a
non-empty frame, has finite coordinates, and is no older than ``max_age_s``
relative to the event being stamped. When no usable pose exists the adapter
does NOT invent one and does NOT drop the event: safety information still
reaches the store, but it is published with ``frame_id`` set to
``UNKNOWN_FRAME`` (the empty string), which the memory node reads as "this
position is not meaningful" and refuses to spatially join. An unbound event
is honest; a confidently mislocated one is not.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

Point3 = Tuple[float, float, float]

#: ``header.frame_id`` value meaning "the position field is not meaningful".
#: Chosen as the empty string because that is what an unset ROS Header
#: already carries, so a publisher that never learned a pose degrades into
#: the unknown case rather than into a false "map".
UNKNOWN_FRAME = ""

#: Default staleness bound. The Go2 publishes ``/utlidar/robot_odom`` at
#: ~150 Hz, so half a second is ~75 missed messages: comfortably past
#: "a dropped packet" and well inside "the robot has moved somewhere else".
DEFAULT_MAX_AGE_S = 0.5


@dataclass(frozen=True)
class Pose:
    """A robot position at a point in time, in a named frame."""

    position: Point3
    frame_id: str
    stamp_s: float


class OdometryCache:
    """Holds the most recent usable odometry sample, with an age bound.

    Single-writer, single-reader by construction: the ROS executor calls
    :meth:`update` from the odometry callback and :meth:`pose_at` from the
    event callback on the same thread, so no locking is needed.
    """

    def __init__(self, max_age_s: float = DEFAULT_MAX_AGE_S) -> None:
        if max_age_s <= 0.0:
            raise ValueError(f"max_age_s must be positive, got {max_age_s!r}")
        self._max_age_s = float(max_age_s)
        self._pose: Optional[Pose] = None
        self._accepted = 0
        self._rejected = 0

    @property
    def max_age_s(self) -> float:
        return self._max_age_s

    @property
    def accepted_count(self) -> int:
        """Odometry samples stored."""
        return self._accepted

    @property
    def rejected_count(self) -> int:
        """Odometry samples refused as malformed (blank frame, NaN, no stamp)."""
        return self._rejected

    @property
    def last_pose(self) -> Optional[Pose]:
        """The last accepted sample, regardless of age. Use :meth:`pose_at`
        to get one that is actually fresh enough to stamp an event with."""
        return self._pose

    def update(self, x: float, y: float, z: float, frame_id: str,
               stamp_s: float) -> bool:
        """Store an odometry sample. Returns True if it was accepted.

        A sample is refused, and the previous one kept, when the frame is
        blank, any coordinate is non-finite, or the stamp is non-positive.
        A driver that emits garbage should not be able to poison the cache
        into stamping events with a NaN position.
        """
        if not frame_id:
            self._rejected += 1
            return False
        try:
            xf, yf, zf, ts = float(x), float(y), float(z), float(stamp_s)
        except (TypeError, ValueError):
            self._rejected += 1
            return False
        if not all(math.isfinite(v) for v in (xf, yf, zf, ts)):
            self._rejected += 1
            return False
        if ts <= 0.0:
            self._rejected += 1
            return False
        self._pose = Pose(position=(xf, yf, zf), frame_id=str(frame_id), stamp_s=ts)
        self._accepted += 1
        return True

    def pose_at(self, event_time_s: float) -> Optional[Pose]:
        """The cached pose if it is within ``max_age_s`` of ``event_time_s``.

        The comparison is two-sided on purpose. A pose far in the *future*
        of the event is as untrustworthy as a stale one: it means the two
        streams are on different clocks (a common sim-versus-wall-clock
        mistake), and stamping across that gap would fabricate precision.
        """
        pose = self._pose
        if pose is None:
            return None
        try:
            t = float(event_time_s)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(t):
            return None
        if abs(t - pose.stamp_s) > self._max_age_s:
            return None
        return pose

    def is_fresh(self, event_time_s: float) -> bool:
        """True when :meth:`pose_at` would return a pose for this time."""
        return self.pose_at(event_time_s) is not None


__all__ = [
    "DEFAULT_MAX_AGE_S",
    "UNKNOWN_FRAME",
    "OdometryCache",
    "Pose",
    "Point3",
]
