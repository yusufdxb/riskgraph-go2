"""Adapter: std_msgs/Bool on /tactile/slip_state → riskgraph_msgs/RiskEvent.

Listens for slip flag transitions and emits a single RiskEvent per leading edge.
This avoids flooding the memory store with one event per ROS publication while
the slip flag is held high. Uses Bool rather than the full TactileStamped to
stay decoupled from neuroskin_msgs availability.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Header
from geometry_msgs.msg import Point

from riskgraph_msgs.msg import RiskEvent as RiskEventMsg, RiskFactor as RiskFactorMsg


class TactileAdapter(Node):
    def __init__(self) -> None:
        super().__init__("riskgraph_tactile_adapter")
        self.declare_parameter("input_topic", "/tactile/slip_state")
        self.declare_parameter("output_topic", "/riskgraph/risk_events")
        self.declare_parameter("severity", 0.7)
        in_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self._severity = float(self.get_parameter("severity").get_parameter_value().double_value)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=20)
        self._pub = self.create_publisher(RiskEventMsg, out_topic, qos)
        self._sub = self.create_subscription(Bool, in_topic, self._on_slip, qos)
        self._prev = False
        self.get_logger().info(f"tactile_adapter: {in_topic} → {out_topic}")

    def _on_slip(self, msg: Bool) -> None:
        if msg.data and not self._prev:
            now = self.get_clock().now().to_msg()
            out = RiskEventMsg()
            out.header = Header()
            out.header.stamp = now
            out.header.frame_id = "map"
            out.event_id = ""
            out.position = Point()
            f = RiskFactorMsg()
            f.category = "SLIP"
            f.severity = self._severity
            f.source = "tactile/slip_state"
            f.detail = "leading edge of slip flag"
            out.factors = [f]
            out.confidence = 1.0
            self._pub.publish(out)
        self._prev = bool(msg.data)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TactileAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
