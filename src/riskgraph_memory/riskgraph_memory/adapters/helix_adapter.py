"""Adapter: helix_msgs/FaultEvent → riskgraph_msgs/RiskEvent.

Soft dependency on helix_msgs.
"""
from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import Header

from riskgraph_msgs.msg import RiskEvent as RiskEventMsg, RiskFactor as RiskFactorMsg

try:
    from helix_msgs.msg import FaultEvent  # noqa: F401
    HAVE_HELIX_MSGS = True
except ImportError:
    HAVE_HELIX_MSGS = False


# severity 1=WARN 2=ERROR 3=CRITICAL → [0,1]
_SEVERITY_MAP = {1: 0.3, 2: 0.6, 3: 1.0}


class HelixAdapter(Node):
    def __init__(self) -> None:
        super().__init__("riskgraph_helix_adapter")
        self.declare_parameter("input_topic", "/helix/faults")
        self.declare_parameter("output_topic", "/riskgraph/risk_events")
        in_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=50)
        self._pub = self.create_publisher(RiskEventMsg, out_topic, qos)
        from helix_msgs.msg import FaultEvent as _FaultEvent
        self._sub = self.create_subscription(_FaultEvent, in_topic, self._on_fault, qos)
        self.get_logger().info(f"helix_adapter: {in_topic} → {out_topic}")

    def _on_fault(self, msg) -> None:
        sev = _SEVERITY_MAP.get(int(msg.severity), 0.5)
        out = RiskEventMsg()
        out.header = Header()
        # helix FaultEvent carries timestamp as float64 — synthesize stamp
        sec = int(msg.timestamp)
        nsec = int((msg.timestamp - sec) * 1e9)
        out.header.stamp.sec = sec
        out.header.stamp.nanosec = max(0, min(999_999_999, nsec))
        out.header.frame_id = "map"
        out.event_id = ""
        out.position = Point()
        f = RiskFactorMsg()
        f.category = "FAULT"
        f.severity = float(sev)
        f.source = "helix/faults"
        f.detail = f"{msg.fault_type} from {msg.node_name}: {msg.detail}"
        out.factors = [f]
        out.confidence = 1.0
        self._pub.publish(out)


def main(args=None) -> None:
    if not HAVE_HELIX_MSGS:
        print(
            "[riskgraph_helix_adapter] helix_msgs not found; this adapter is a no-op.",
            file=sys.stderr,
        )
        return
    rclpy.init(args=args)
    node = HelixAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
