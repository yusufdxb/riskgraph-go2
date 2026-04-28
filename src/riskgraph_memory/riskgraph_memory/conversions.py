"""Pure-Python helpers for converting between ROS messages and core dataclasses.

Kept separate from the node so the same conversions are unit-testable without
spinning a ROS node. ROS message imports are deferred so this module is
importable in environments where rclpy/riskgraph_msgs are not built.
"""
from __future__ import annotations

from typing import Any, List

from riskgraph_core.events import RiskEvent, RiskFactor, FactorCategory
from riskgraph_core.segments import RouteSegment, Route


def core_event_from_msg(msg: Any) -> RiskEvent:
    factors = [
        RiskFactor(
            category=FactorCategory.coerce(f.category),
            severity=float(f.severity),
            source=str(f.source),
            detail=str(f.detail),
        )
        for f in msg.factors
    ]
    return RiskEvent(
        event_id=str(msg.event_id) or RiskEvent.new_id(),
        position=(float(msg.position.x), float(msg.position.y), float(msg.position.z)),
        factors=factors,
        confidence=float(msg.confidence),
        timestamp=float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
        frame_id=str(msg.header.frame_id) or "map",
        segment_id=(str(msg.segment_id) if getattr(msg, "segment_id", "") else None),
    )


def msg_segment_to_core(msg: Any) -> RouteSegment:
    return RouteSegment(
        segment_id=str(msg.segment_id),
        start=(float(msg.start.x), float(msg.start.y), float(msg.start.z)),
        end=(float(msg.end.x), float(msg.end.y), float(msg.end.z)),
        semantic_label=str(msg.semantic_label) or None,
    )


def msg_route_to_core(msg: Any) -> Route:
    return Route(
        route_id=str(msg.route_id),
        segments=[msg_segment_to_core(s) for s in msg.segments],
    )
