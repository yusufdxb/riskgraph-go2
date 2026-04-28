"""ROS 2 node: replay a scenario fixture as live RiskEvent messages.

Used in `demo_offline.launch.py` to drive the full ROS pipeline (memory →
planner → explainer) with deterministic synthetic data, no upstream Go2
stack required.
"""
from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header
from geometry_msgs.msg import Point

from riskgraph_msgs.msg import RiskEvent as RiskEventMsg, RiskFactor as RiskFactorMsg

from .scenario import load_scenario


_DEFAULT_FIXTURE = (
    Path(__file__).resolve().parents[1] / "fixtures" / "scenario_glossy_hallway.json"
)


class SyntheticPublisher(Node):
    def __init__(self) -> None:
        super().__init__("riskgraph_synthetic_publisher")
        self.declare_parameter("scenario_path", str(_DEFAULT_FIXTURE))
        self.declare_parameter("publish_period_s", 0.2)
        self.declare_parameter("output_topic", "/riskgraph/risk_events")

        scenario_path = self.get_parameter("scenario_path").get_parameter_value().string_value
        period = float(self.get_parameter("publish_period_s").get_parameter_value().double_value)
        out_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        self._scenario = load_scenario(scenario_path)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=50)
        self._pub = self.create_publisher(RiskEventMsg, out_topic, qos)
        self._idx = 0
        self._timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f"synthetic_publisher: replaying {len(self._scenario.events)} events "
            f"from {scenario_path} → {out_topic} every {period:.2f}s"
        )

    def _tick(self) -> None:
        if self._idx >= len(self._scenario.events):
            self._timer.cancel()
            self.get_logger().info("synthetic_publisher: all events replayed")
            return
        ev = self._scenario.events[self._idx]
        self._idx += 1
        msg = RiskEventMsg()
        msg.header = Header()
        sec = int(ev.timestamp)
        nsec = int((ev.timestamp - sec) * 1e9)
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = max(0, min(999_999_999, nsec))
        msg.header.frame_id = ev.frame_id
        msg.event_id = ev.event_id
        msg.segment_id = ev.segment_id or ""
        p = Point()
        p.x, p.y, p.z = ev.position
        msg.position = p
        msg.factors = []
        for f in ev.factors:
            fm = RiskFactorMsg()
            fm.category = f.category.value
            fm.severity = float(f.severity)
            fm.source = f.source
            fm.detail = f.detail
            msg.factors.append(fm)
        msg.confidence = float(ev.confidence)
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SyntheticPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
