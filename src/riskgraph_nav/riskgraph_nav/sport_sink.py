"""RiskGraph GO2 sport sink: /nav/cmd_vel -> /api/sport/request (unitree_api/Request).

Single input (Nav2 cmd_vel), single robot-facing publisher. Mode is fixed at
startup (read-only parameter) so the operator changes it only by restarting:

  dry_run   (default) nothing is sent to the robot; every decision is traced
  stop_only only StopMove (1003) is ever sent
  armed     Move (1008) allowed within the sink limits

Every decision, in every mode, is published as JSON on /riskgraph/sink/trace.
On SIGINT/SIGTERM a StopMove burst is sent before exit (not in dry_run).
"""
from __future__ import annotations

import json
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from riskgraph_nav.sport_sink_core import (
    INPUT_TOPIC,
    MODE_DRY_RUN,
    MODES,
    NODE_NAME,
    SinkLimits,
    SinkLogic,
    fill_unitree_request,
    stop_move,
    TRACE_TOPIC,
)

IN_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE)
TRACE_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE)


class RiskGraphSportSink(Node):

    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        ro = ParameterDescriptor(read_only=True)
        self.declare_parameter('mode', MODE_DRY_RUN, ro)
        self.declare_parameter('input_topic', INPUT_TOPIC, ro)
        self.declare_parameter('request_topic', '/api/sport/request', ro)
        self.declare_parameter('trace_topic', TRACE_TOPIC, ro)
        self.declare_parameter('max_vx', 0.25, ro)
        self.declare_parameter('max_vy', 0.20, ro)
        self.declare_parameter('max_wz', 0.50, ro)
        self.declare_parameter('input_timeout_sec', 0.25, ro)
        self.declare_parameter('stop_repeat_hz', 2.0, ro)
        self.declare_parameter('move_hz', 20.0, ro)
        mode = self.get_parameter('mode').value
        if mode not in MODES:
            raise ValueError(f'mode must be one of {MODES}')
        self.logic = SinkLogic(
            mode,
            SinkLimits(self.get_parameter('max_vx').value,
                       self.get_parameter('max_vy').value,
                       self.get_parameter('max_wz').value),
            input_timeout_sec=self.get_parameter('input_timeout_sec').value,
            stop_repeat_hz=self.get_parameter('stop_repeat_hz').value,
            move_hz=self.get_parameter('move_hz').value)
        self._req_cls = None
        self._pub_req = None
        if mode != MODE_DRY_RUN:
            # Only needed when talking to the robot; absent on dev machines.
            from unitree_api.msg import Request
            self._req_cls = Request
            self._pub_req = self.create_publisher(
                Request, self.get_parameter('request_topic').value, 10)
        self._pub_trace = self.create_publisher(
            String, self.get_parameter('trace_topic').value, TRACE_QOS)
        self._req_id = int(time.time() * 1000) % 1_000_000_000
        self.create_subscription(
            Twist, self.get_parameter('input_topic').value, self._on_cmd, IN_QOS)
        self.create_timer(0.05, self._on_tick)
        self.get_logger().warning(f'RiskGraph GO2 sport sink up in mode={mode}')

    def _on_cmd(self, msg: Twist) -> None:
        self._send(self.logic.on_command(
            msg.linear.x, msg.linear.y, msg.angular.z, time.monotonic()),
            [msg.linear.x, msg.linear.y, msg.angular.z])

    def _on_tick(self) -> None:
        self._send(self.logic.on_tick(time.monotonic()), None)

    def _send(self, req, cmd_in) -> None:
        if req is None:
            return
        self._req_id += 1
        sent = False
        if self._pub_req is not None:
            self._pub_req.publish(fill_unitree_request(self._req_cls(), req, self._req_id))
            sent = True
        self._pub_trace.publish(String(data=json.dumps({
            't_wall': time.time(), 't_mono': time.monotonic(), 'mode': self.logic.mode,
            'api_id': req.api_id, 'reason': req.reason, 'x': req.x, 'y': req.y,
            'z': req.z, 'input': cmd_in, 'request_id': self._req_id,
            'sent_to_robot': sent})))
        if req.reason != 'MOVE':
            self.get_logger().info(f'StopMove ({req.reason}) sent_to_robot={sent}',
                                   throttle_duration_sec=1.0)

    def stop_burst(self, n: int = 3) -> None:
        for _ in range(n):
            self._send(stop_move('SHUTDOWN'), None)


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = RiskGraphSportSink()
    stop = {'sig': None}
    signal.signal(signal.SIGINT, lambda s, f: stop.__setitem__('sig', s))
    signal.signal(signal.SIGTERM, lambda s, f: stop.__setitem__('sig', s))
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    try:
        while stop['sig'] is None and rclpy.ok():
            ex.spin_once(timeout_sec=0.05)
        node.stop_burst()
        end = time.monotonic() + 0.2
        while time.monotonic() < end:
            ex.spin_once(timeout_sec=0.02)
    finally:
        ex.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
