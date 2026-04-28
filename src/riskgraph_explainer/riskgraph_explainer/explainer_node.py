"""ROS 2 node: re-broadcast explanations on a dedicated topic.

The planner already includes an explanation in its service response. This
node provides a *streaming* path for consumers that prefer subscribing to a
topic (e.g. UI overlays, voice synthesis): subscribe to /riskgraph/route_scores
and publish a freshly-rendered explanation per array.

If the SQLite store is reachable, the explanation includes evidence event ids;
otherwise it falls back to the dominant_factor_categories already on the score.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header

from riskgraph_msgs.msg import (
    RouteScoreArray as RouteScoreArrayMsg,
    RouteExplanation as RouteExplanationMsg,
)

from riskgraph_core.store import RiskStore


def _factor_human(category: str, plural: bool) -> str:
    table = {
        "SLIP": "slip", "SAFETY": "safety alert", "DEPTH": "depth hazard",
        "AUDIO": "audio anomaly", "FAULT": "system fault",
        "HUMAN": "human-related hazard", "COLLISION": "near-collision",
        "OTHER": "risk event",
    }
    base = table.get(category, "risk event")
    return base + "s" if (plural and not base.endswith("s")) else base


class ExplainerNode(Node):
    def __init__(self) -> None:
        super().__init__("riskgraph_explainer")
        self.declare_parameter("store_path", ":memory:")
        self.declare_parameter("input_topic", "/riskgraph/route_scores")
        self.declare_parameter("output_topic", "/riskgraph/explanations")

        store_path = self.get_parameter("store_path").get_parameter_value().string_value
        in_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        try:
            self._store = RiskStore(store_path)
        except Exception as exc:
            self.get_logger().warn(f"RiskStore unavailable ({exc}); evidence ids will be empty")
            self._store = None

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self._pub = self.create_publisher(RouteExplanationMsg, out_topic, qos)
        self._sub = self.create_subscription(RouteScoreArrayMsg, in_topic, self._on_scores, qos)
        self.get_logger().info(f"riskgraph_explainer ready, {in_topic} → {out_topic}")

    def _on_scores(self, msg: RouteScoreArrayMsg) -> None:
        if not msg.scores:
            return
        chosen = next((s for s in msg.scores if s.route_id == msg.chosen_route_id), msg.scores[0])
        evidence_ids = []
        if chosen.dominant_segment_ids and chosen.risk_cost > 0:
            cat = chosen.dominant_factor_categories[0] if chosen.dominant_factor_categories else "OTHER"
            seg = chosen.dominant_segment_ids[0]
            text = (
                f"Recommended route {chosen.route_id}; the dominant remaining risk on this "
                f"path is {_factor_human(cat, plural=True)} on segment {seg}."
            )
            if self._store is not None:
                evidence = self._store.evidence_for_segment(seg, max_events=3)
                evidence_ids = [e.event_id for e in evidence]
        else:
            text = f"Recommended route {chosen.route_id}; no risk events recorded on its segments."

        out = RouteExplanationMsg()
        out.header = Header()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id or "map"
        out.route_id = chosen.route_id
        out.text = text
        out.evidence_event_ids = evidence_ids
        self._pub.publish(out)

    def destroy_node(self) -> bool:
        if self._store is not None:
            try:
                self._store.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExplainerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
