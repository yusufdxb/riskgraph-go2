"""ROS glue that stamps adapter-emitted RiskEvents with a real robot pose.

Every adapter in this package converts an upstream message that knows *what*
happened but not *where*. This mixin gives them the "where": it subscribes to
the robot's odometry, keeps the last sample in a bounded-age
:class:`~riskgraph_memory.pose_source.OdometryCache`, and stamps outgoing
events with that pose, or explicitly marks them unposed when no fresh pose is
available. The policy itself lives in ``pose_source``; this file is only the
ROS wiring.

Topic contract (verified against the Go2 EDU on 2026-04-17, see
``docs/hardware_integration.md``): ``/utlidar/robot_odom``,
``nav_msgs/Odometry``, ~150 Hz, ``header.frame_id = "odom"``,
``child_frame_id = "base_link"``. The Go2 SDK publishes no ``map`` frame and
no ``/tf``, so events tagged from this source are in ``odom``, and the
segment seed must declare the same frame for the join to happen at all.

The subscription is BEST_EFFORT / KEEP_LAST / depth 1: this is a high-rate
sensor stream where only the newest sample matters, and a best-effort
subscriber is compatible with either a best-effort or a reliable publisher.
"""
from __future__ import annotations

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry

from riskgraph_memory.pose_source import (
    DEFAULT_MAX_AGE_S,
    UNKNOWN_FRAME,
    OdometryCache,
)

#: Default odometry source on the Go2 EDU.
DEFAULT_ODOM_TOPIC = "/utlidar/robot_odom"

#: How often to repeat the "events are being stored unposed" warning, in
#: events. Loud enough that an operator notices during a run, quiet enough
#: that a robot with no odom at all does not drown the log.
UNPOSED_WARN_EVERY = 50


def stamp_seconds(stamp) -> float:
    """Convert a builtin_interfaces/Time to float seconds. 0.0 if unset."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class PoseTaggingMixin:
    """Mix into an adapter ``Node`` to give it pose-aware event emission.

    The host node must call :meth:`init_pose_tagging` after
    ``super().__init__()`` and then call :meth:`stamp_pose` on every outgoing
    RiskEvent before publishing it.
    """

    def init_pose_tagging(self) -> None:
        """Declare pose parameters and subscribe to the odometry source."""
        self.declare_parameter("odom_topic", DEFAULT_ODOM_TOPIC)
        self.declare_parameter("pose_max_age_s", DEFAULT_MAX_AGE_S)

        odom_topic = self.get_parameter(
            "odom_topic").get_parameter_value().string_value
        max_age_s = float(self.get_parameter(
            "pose_max_age_s").get_parameter_value().double_value)
        if max_age_s <= 0.0:
            self.get_logger().warn(
                f"pose_max_age_s={max_age_s} is not positive; "
                f"falling back to {DEFAULT_MAX_AGE_S}s"
            )
            max_age_s = DEFAULT_MAX_AGE_S

        self._pose_cache = OdometryCache(max_age_s=max_age_s)
        self._unposed_events = 0
        self._posed_events = 0

        if odom_topic:
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
            self._odom_sub = self.create_subscription(
                Odometry, odom_topic, self._on_odom, qos)
            self.get_logger().info(
                f"pose source: {odom_topic} (max age {max_age_s:.2f}s)"
            )
        else:
            # An explicitly empty topic is a supported configuration: it means
            # "this deployment has no odometry", and every event will be
            # emitted unposed rather than silently mislocated.
            self._odom_sub = None
            self.get_logger().warn(
                "odom_topic is empty; every event will be published unposed "
                f"(frame_id={UNKNOWN_FRAME!r}) and will not be spatially joined"
            )

    @property
    def pose_cache(self) -> OdometryCache:
        return self._pose_cache

    @property
    def unposed_event_count(self) -> int:
        return self._unposed_events

    @property
    def posed_event_count(self) -> int:
        return self._posed_events

    def _on_odom(self, msg) -> None:
        """Cache one odometry sample.

        Some Go2 firmware revisions publish odometry with an unset header
        stamp. Rather than refuse those outright, fall back to the receive
        time from the node clock: the sample is genuinely "now" to within one
        callback, which is exactly what the age bound is measuring.
        """
        stamp_s = stamp_seconds(msg.header.stamp)
        if stamp_s <= 0.0:
            stamp_s = stamp_seconds(self.get_clock().now().to_msg())
        p = msg.pose.pose.position
        self._pose_cache.update(p.x, p.y, p.z, msg.header.frame_id, stamp_s)

    def stamp_pose(self, out, event_time_s: float) -> bool:
        """Stamp ``out.position`` / ``out.header.frame_id`` for an event.

        Returns True when a fresh pose was applied. When it returns False the
        event is marked unposed (``frame_id`` = :data:`UNKNOWN_FRAME`, position
        left at the origin) and the memory node will store it without a
        segment rather than guess one.
        """
        pose = self._pose_cache.pose_at(event_time_s)
        if pose is None:
            self._unposed_events += 1
            out.header.frame_id = UNKNOWN_FRAME
            if self._unposed_events % UNPOSED_WARN_EVERY == 1:
                self.get_logger().warn(
                    f"no odometry within {self._pose_cache.max_age_s:.2f}s of "
                    f"event t={event_time_s:.3f}; publishing unposed "
                    f"({self._unposed_events} so far this run)"
                )
            return False
        out.position.x, out.position.y, out.position.z = pose.position
        out.header.frame_id = pose.frame_id
        self._posed_events += 1
        return True


__all__ = [
    "DEFAULT_ODOM_TOPIC",
    "UNPOSED_WARN_EVERY",
    "PoseTaggingMixin",
    "stamp_seconds",
]
