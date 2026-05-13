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
from riskgraph_core.seed import (
    SegmentSeedError,
    SegmentSeedResult,
    load_segment_seed,
)

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
        self.declare_parameter("segment_seed_path", "")
        store_path = self.get_parameter("store_path").get_parameter_value().string_value
        if store_path and store_path != ":memory:":
            os.makedirs(os.path.dirname(os.path.abspath(store_path)) or ".", exist_ok=True)
        self._store = RiskStore(store_path)
        self._known_segments = []  # populated by segment seed at startup
        self._segment_seed: SegmentSeedResult = SegmentSeedResult(segments=[])

        seed_path = self.get_parameter("segment_seed_path").get_parameter_value().string_value
        if seed_path:
            try:
                self._segment_seed = load_segment_seed(seed_path)
                self.register_segments(self._segment_seed.segments)
                if self._segment_seed.duplicate_ids:
                    self.get_logger().warn(
                        f"segment seed {seed_path} has duplicate ids "
                        f"(last-write-wins): {self._segment_seed.duplicate_ids}"
                    )
                self.get_logger().info(
                    f"segment seed loaded from {seed_path}: "
                    f"{len(self._segment_seed)} segments, frame_id={self._segment_seed.frame_id}"
                )
            except SegmentSeedError as exc:
                # Seed misconfig is loud but non-fatal: the node still
                # runs, just with no spatial-join fallback. Operators
                # who care will see the WARN at launch time.
                self.get_logger().error(
                    f"segment seed load FAILED for {seed_path!r}: {exc}; "
                    f"continuing with empty known-segments list"
                )

        self._sub = self.create_subscription(
            RiskEventMsg, "/riskgraph/risk_events", self._on_event, _reliable_qos()
        )
        self._srv = self.create_service(
            QuerySegmentRisk, "/riskgraph/query_segment_risk", self._on_query
        )
        self.get_logger().info(
            f"riskgraph_memory ready, store_path={store_path}, "
            f"known_segments={len(self._known_segments)}"
        )

    @property
    def store(self) -> RiskStore:
        return self._store

    def register_segments(self, segments) -> None:
        """Used by the bringup launch to seed known segments for spatial join."""
        self._known_segments = list(segments)

    @property
    def known_segments(self) -> list:
        """Read-only view of the segments used for spatial-join fallback."""
        return list(self._known_segments)

    @property
    def segment_seed(self) -> SegmentSeedResult:
        """The parsed seed (or an empty SegmentSeedResult if seeding was skipped)."""
        return self._segment_seed

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
        try:
            self._store.record_event(ev)
        except Exception as exc:  # SQLite errors must not crash the subscription callback
            self.get_logger().error(
                f"failed to persist RiskEvent {ev.event_id}: {exc}"
            )

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
