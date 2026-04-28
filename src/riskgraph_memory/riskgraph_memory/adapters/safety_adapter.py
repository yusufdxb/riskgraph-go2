"""Adapter: go2_msgs/SafetyAlert → riskgraph_msgs/RiskEvent.

Soft dependency on `go2_msgs`: if the upstream message is not present in the
ament install tree, this node logs an error on import and exits cleanly so it
does not block the rest of the bringup.
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
    from go2_msgs.msg import SafetyAlert  # noqa: F401
    HAVE_GO2_MSGS = True
except ImportError:
    HAVE_GO2_MSGS = False


# Map upstream alert_type → (severity, category)
_ALERT_TABLE = {
    "EMERGENCY_STOP":   (1.0, "SAFETY"),
    "DROP_DETECTED":    (0.9, "DEPTH"),
    "STAIRS_DETECTED":  (0.7, "DEPTH"),
    "NARROW_PASSAGE":   (0.5, "DEPTH"),
    "SLOWDOWN":         (0.4, "SAFETY"),
}


def _alert_to_factor(alert_type: str) -> tuple:
    return _ALERT_TABLE.get(alert_type, (0.5, "SAFETY"))


class SafetyAdapter(Node):
    def __init__(self) -> None:
        super().__init__("riskgraph_safety_adapter")
        self.declare_parameter("input_topic", "/go2/safety_alert")
        self.declare_parameter("output_topic", "/riskgraph/risk_events")
        in_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=20)
        self._pub = self.create_publisher(RiskEventMsg, out_topic, qos)
        from go2_msgs.msg import SafetyAlert as _SafetyAlert
        self._sub = self.create_subscription(_SafetyAlert, in_topic, self._on_alert, qos)
        self.get_logger().info(
            f"safety_adapter: {in_topic} → {out_topic}"
        )

    def _on_alert(self, msg) -> None:
        sev, cat = _alert_to_factor(str(msg.alert_type))
        out = RiskEventMsg()
        out.header = Header()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id or "map"
        out.event_id = ""  # memory node will not regen; use new_id pattern in conversions
        out.position = Point()  # adapter does not know robot pose; planner/memory does spatial join
        f = RiskFactorMsg()
        f.category = cat
        f.severity = float(sev)
        f.source = "go2/safety_alert"
        f.detail = f"{msg.alert_type}: {msg.description} (d={msg.distance:.2f}m)"
        out.factors = [f]
        out.confidence = 1.0
        self._pub.publish(out)


def main(args=None) -> None:
    if not HAVE_GO2_MSGS:
        # Print to stderr; rclpy may not be initialized.
        print(
            "[riskgraph_safety_adapter] go2_msgs not found; this adapter is a no-op. "
            "Install go2_msgs from upstream GO2-seeing-eye-dog to enable.",
            file=sys.stderr,
        )
        return
    rclpy.init(args=args)
    node = SafetyAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
