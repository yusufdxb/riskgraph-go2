"""Live ROS graph introspection shared by the preflight and the trial runner.

Everything here only READS the graph: node lists, endpoint lists, lifecycle
states, parameters, message rates, TF. Nothing publishes.
"""
from __future__ import annotations

import json
import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                     depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
SENSOR = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
                    depth=10)


def fq(ns: str, name: str) -> str:
    return ns.rstrip("/") + "/" + name


class GraphProbe:
    """A node spinning on a background executor, plus read-only helpers."""

    def __init__(self, name: str = "riskgraph_graph_probe") -> None:
        self.node = rclpy.create_node(name)
        self._ex = MultiThreadedExecutor(num_threads=4)
        self._ex.add_node(self.node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()
        self._tf_buffer = None
        self._tf_listener = None

    def _spin(self) -> None:
        try:
            self._ex.spin()
        except Exception:
            pass

    def close(self) -> None:
        if self._tf_listener is not None:
            try:
                self._tf_listener.executor.shutdown()
                self._tf_listener.dedicated_listener_thread.join(timeout=2.0)
            except Exception:
                pass
        self._ex.shutdown(timeout_sec=1.0)
        self.node.destroy_node()

    # -- graph --------------------------------------------------------------

    def nodes(self) -> List[str]:
        return sorted(fq(ns, n) for n, ns in self.node.get_node_names_and_namespaces())

    def duplicate_nodes(self) -> List[str]:
        names = [fq(ns, n) for n, ns in self.node.get_node_names_and_namespaces()]
        return sorted({n for n in names if names.count(n) > 1})

    def topics(self) -> Dict[str, List[str]]:
        return {t: list(ty) for t, ty in self.node.get_topic_names_and_types()}

    def publishers(self, topic: str) -> List[Dict[str, str]]:
        return [{"node": fq(e.node_namespace, e.node_name), "type": e.topic_type,
                 "reliability": e.qos_profile.reliability.name,
                 "durability": e.qos_profile.durability.name}
                for e in self.node.get_publishers_info_by_topic(topic)]

    def subscribers(self, topic: str) -> List[Dict[str, str]]:
        return [{"node": fq(e.node_namespace, e.node_name), "type": e.topic_type,
                 "reliability": e.qos_profile.reliability.name,
                 "durability": e.qos_profile.durability.name}
                for e in self.node.get_subscriptions_info_by_topic(topic)]

    def topics_published_by(self, node_fq: str) -> Dict[str, List[str]]:
        ns, _, name = node_fq.rpartition("/")
        try:
            return {t: list(ty) for t, ty in
                    self.node.get_publisher_names_and_types_by_node(name, ns or "/")}
        except Exception:
            return {}

    def snapshot(self) -> Dict[str, object]:
        topics = self.topics()
        return {
            "nodes": self.nodes(),
            "topics": topics,
            "endpoints": {t: {"publishers": self.publishers(t), "subscribers": self.subscribers(t)}
                          for t in topics},
        }

    # -- services -------------------------------------------------------------

    def _call(self, srv_type, name: str, req, timeout: float = 2.0):
        cli = self.node.create_client(srv_type, name)
        try:
            if not cli.wait_for_service(timeout_sec=timeout):
                return None
            fut = cli.call_async(req)
            ev = threading.Event()
            fut.add_done_callback(lambda _f: ev.set())
            ev.wait(timeout)
            return fut.result() if fut.done() else None
        finally:
            self.node.destroy_client(cli)

    def lifecycle_state(self, node_fq: str) -> Optional[str]:
        from lifecycle_msgs.srv import GetState
        r = self._call(GetState, f"{node_fq}/get_state", GetState.Request())
        return r.current_state.label if r is not None else None

    def params(self, node_fq: str, names: List[str]) -> Optional[Dict[str, object]]:
        from rcl_interfaces.srv import GetParameters
        from rclpy.parameter import parameter_value_to_python
        r = self._call(GetParameters, f"{node_fq}/get_parameters", GetParameters.Request(names=names))
        if r is None or len(r.values) != len(names):
            return None
        return {n: parameter_value_to_python(v) for n, v in zip(names, r.values)}

    # -- data -----------------------------------------------------------------------

    def sample(self, topic: str, type_str: str, duration_s: float, qos: QoSProfile = SENSOR
               ) -> Tuple[int, Optional[object]]:
        """(message count over duration, last message)."""
        from rosidl_runtime_py.utilities import get_message
        cls = get_message(type_str)
        box = {"n": 0, "last": None}

        def cb(m):
            box["n"] += 1
            box["last"] = m
        sub = self.node.create_subscription(cls, topic, cb, qos)
        time.sleep(duration_s)
        self.node.destroy_subscription(sub)
        return box["n"], box["last"]

    def latest_json(self, topic: str, timeout_s: float = 3.0) -> Optional[dict]:
        n, m = self._wait_one(topic, "std_msgs/msg/String", timeout_s, LATCHED)
        if m is None:
            return None
        try:
            return json.loads(m.data)
        except ValueError:
            return None

    def _wait_one(self, topic: str, type_str: str, timeout_s: float, qos: QoSProfile):
        from rosidl_runtime_py.utilities import get_message
        cls = get_message(type_str)
        box = {"m": None}
        ev = threading.Event()

        def cb(m):
            box["m"] = m
            ev.set()
        sub = self.node.create_subscription(cls, topic, cb, qos)
        ev.wait(timeout_s)
        # Give a fresher sample a moment if one is streaming.
        time.sleep(0.05)
        self.node.destroy_subscription(sub)
        return (1 if box["m"] is not None else 0), box["m"]

    def wait_one(self, topic: str, type_str: str, timeout_s: float = 3.0,
                 qos: QoSProfile = LATCHED):
        return self._wait_one(topic, type_str, timeout_s, qos)[1]

    # -- TF --------------------------------------------------------------------------

    def tf_buffer(self):
        if self._tf_buffer is None:
            import tf2_ros
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, None, spin_thread=True)
        return self._tf_buffer

    def lookup(self, target: str, source: str, timeout_s: float = 0.5):
        """Latest transform or None. Returns (TransformStamped, age_s)."""
        from rclpy.duration import Duration
        from rclpy.time import Time
        buf = self.tf_buffer()
        try:
            t = buf.lookup_transform(target, source, Time(), timeout=Duration(seconds=timeout_s))
        except Exception:
            return None, None
        stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        now = self.node.get_clock().now().nanoseconds * 1e-9
        return t, (now - stamp if stamp > 0 else None)
