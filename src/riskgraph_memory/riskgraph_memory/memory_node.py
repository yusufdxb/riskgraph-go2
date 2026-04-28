"""ROS 2 node: persistent risk memory.

Subscribes to /riskgraph/risk_events (riskgraph_msgs/RiskEvent) and writes
every event into a SQLite-backed RiskStore. Exposes a service to query the
cumulative risk for a list of segment ids, used by the planner.

The node is intentionally minimal: it does not transform poses, run TF
lookups, or do spatial joins itself — adapters and the planner handle that.
This keeps the memory node a single-responsibility durable log.
"""
from __future__ import annotations

import os
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from riskgraph_msgs.msg import RiskEvent as RiskEventMsg
from riskgraph_msgs.srv import QuerySegmentRisk

from riskgraph_core.store import RiskStore
from riskgraph_core.segments import segment_for_point

from .conversions import core_event_from_msg


def _reliable_qos(depth: int = 50) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class RiskMemoryNode(Node):
    def __init__(self) -> None:
        super().__init__("riskgraph_memory")
        self.declare_parameter("store_path", ":memory:")
        self.declare_parameter("decay_half_life_s", 0.0)
        store_path = self.get_parameter("store_path").get_parameter_value().string_value
        if store_path and store_path != ":memory:":
            os.makedirs(os.path.dirname(os.path.abspath(store_path)) or ".", exist_ok=True)
        self._store = RiskStore(store_path)
        self._known_segments = []  # set externally via segment_registry topic in future

        self._sub = self.create_subscription(
            RiskEventMsg, "/riskgraph/risk_events", self._on_event, _reliable_qos()
        )
        self._srv = self.create_service(
            QuerySegmentRisk, "/riskgraph/query_segment_risk", self._on_query
        )
        self.get_logger().info(f"riskgraph_memory ready, store_path={store_path}")

    @property
    def store(self) -> RiskStore:
        return self._store

    def register_segments(self, segments) -> None:
        """Used by the bringup launch to seed known segments for spatial join."""
        self._known_segments = list(segments)

    def _on_event(self, msg: RiskEventMsg) -> None:
        try:
            ev = core_event_from_msg(msg)
        except Exception as exc:  # malformed input: log and drop
            self.get_logger().warn(f"dropped malformed RiskEvent: {exc}")
            return
        # If the emitter did not stamp a segment, attempt a spatial join.
        if not ev.segment_id and self._known_segments:
            nearest = segment_for_point(self._known_segments, ev.position)
            if nearest is not None:
                ev.segment_id = nearest.segment_id
        if not ev.segment_id:
            self.get_logger().debug(
                f"event {ev.event_id} has no segment_id; storing unbound"
            )
        self._store.record_event(ev)

    def _on_query(self, request: QuerySegmentRisk.Request,
                  response: QuerySegmentRisk.Response) -> QuerySegmentRisk.Response:
        decay = float(request.decay_half_life_s)
        risks: List[float] = []
        counts: List[int] = []
        dominants: List[str] = []
        for seg_id in request.segment_ids:
            r, c, d = self._store.segment_risk(seg_id, decay_half_life_s=decay)
            risks.append(float(r))
            counts.append(int(c))
            dominants.append(d)
        response.risks = risks
        response.event_counts = counts
        response.dominant_factor_categories = dominants
        return response

    def destroy_node(self) -> bool:
        try:
            self._store.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RiskMemoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
