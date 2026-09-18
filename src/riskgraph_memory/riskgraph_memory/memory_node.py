"""ROS 2 node: persistent spatial risk memory.

Subscribes ``/riskgraph/risk_events`` and writes each event, transformed into
the ``map`` frame with a timestamped TF lookup, into the SQLite store. From
the stored events it publishes the risk representation Nav2 consumes:

* ``/riskgraph/risk_costmap`` (nav_msgs/OccupancyGrid, reliable, transient
  local): same geometry as ``/map``, values 0..90, never lethal. Nav2's
  global costmap reads it through a second StaticLayer (see
  ``riskgraph_bringup/config/nav2_live.yaml``).
* ``/riskgraph/status`` (std_msgs/String, JSON, transient local): absolute DB
  path, schema version, map id, incident / quarantine counts, active risk
  entries, last observation time, process id and a per-start instance id.

The node never publishes a velocity or any robot command. On shutdown it
publishes an all-zero grid so Nav2 does not keep planning around risk from a
process that no longer exists (the restart-persistence trial relies on this
to prove the risk comes back from disk, not from Nav2's cache).

Ingestion rules live in :mod:`riskgraph_memory.memory_core`.
"""
from __future__ import annotations

import json
import os
import signal
import socket
import sys
import time
import uuid
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

from riskgraph_msgs.msg import RiskEvent as RiskEventMsg
from riskgraph_msgs.srv import QuerySegmentRisk

from riskgraph_core.experiment import ExperimentError, load_experiment
from riskgraph_core.map_identity import MapIdentityInputError, compute_map_id, describe_map
from riskgraph_core.risk_field import GridInfo, RiskFieldParams, grid_stats
from riskgraph_core.seed import SegmentSeedError, SegmentSeedResult, load_segment_seed
from riskgraph_core.store import MEMORY, RiskStore, StoreError

from .conversions import core_event_from_msg
from .memory_core import RUN_MODES, MemoryCore, TransformUnavailable

#: How often to repeat a quarantine warning, in quarantined events.
WARN_EVERY = 20


def reliable_qos(depth: int = 50) -> QoSProfile:
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST, depth=depth)


def latched_qos() -> QoSProfile:
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST, depth=1,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class StartupError(RuntimeError):
    """Configuration that must stop the node before it touches anything."""


class RiskMemoryNode(Node):
    def __init__(self) -> None:
        super().__init__("riskgraph_memory")
        p = self.declare_parameter
        p("store_path", "")
        p("run_mode", "")
        p("map_id", "")
        p("map_yaml", "")
        p("experiment_file", "")
        p("target_frame", "map")
        p("tf_timeout_s", 0.2)
        p("max_clock_skew_s", 300.0)
        p("segment_seed_path", "")
        p("decay_half_life_s", 0.0)          # QuerySegmentRisk default
        p("risk_radius_m", -1.0)             # <0: take from experiment file (or default)
        p("risk_value_per_unit", -1.0)
        p("risk_max_cell_value", -1)
        p("risk_decay_half_life_s", -1.0)
        p("publish_grid", True)
        p("map_topic", "/map")
        p("grid_topic", "/riskgraph/risk_costmap")
        p("status_topic", "/riskgraph/status")
        p("status_period_s", 1.0)
        p("blank_grid_on_shutdown", True)

        g = self._param
        self.run_mode = g("run_mode")
        if self.run_mode not in RUN_MODES:
            raise StartupError(
                f"run_mode must be one of {RUN_MODES} (set it explicitly), got {self.run_mode!r}")
        self.instance_id = str(uuid.uuid4())
        self.start_time = time.time()
        self.target_frame = g("target_frame")

        # -- map identity: computed from the same files map_server loads --
        self.experiment = None
        exp_file = g("experiment_file")
        map_yaml = g("map_yaml")
        field_params = RiskFieldParams()
        anchor = None
        if exp_file:
            try:
                self.experiment = load_experiment(exp_file)
            except ExperimentError as exc:
                raise StartupError(f"experiment file: {exc}") from exc
            map_yaml = map_yaml or self.experiment.map_yaml
            field_params = self.experiment.risk_params
            anchor = self.experiment.anchor
        field_params = self._override_field_params(field_params)
        self.map_yaml = map_yaml
        self.expected_grid: Optional[GridInfo] = None
        computed_id = ""
        if map_yaml:
            try:
                computed_id = compute_map_id(map_yaml, anchor)
                self.expected_grid = describe_map(map_yaml).grid_info()
            except MapIdentityInputError as exc:
                raise StartupError(f"map: {exc}") from exc
        given_id = g("map_id")
        if given_id and computed_id and given_id != computed_id:
            raise StartupError(
                f"map_id parameter {given_id!r} does not match the id computed from "
                f"{map_yaml} ({computed_id!r}); a stale launch argument or the wrong map")
        self.map_id = given_id or computed_id
        if not self.map_id:
            raise StartupError("no map identity: set experiment_file, map_yaml or map_id")

        # -- store --
        store_path = g("store_path")
        if store_path == MEMORY and self.run_mode != "test":
            raise StartupError(":memory: store is only allowed in run_mode=test")
        try:
            self.store = RiskStore(store_path, map_id=self.map_id, evidence_class=self.run_mode)
        except StoreError as exc:
            raise StartupError(f"store: {exc}") from exc

        # -- TF --
        self._tf_buffer = None
        self._tf_listener = None
        self._tf_node = None
        try:
            import tf2_ros
            self._tf_buffer = tf2_ros.Buffer()
            # Lookups with a timeout happen inside our subscription callback, so
            # TF must be received on another thread. TransformListener's
            # spin_thread adds the node it is GIVEN to its own executor; giving
            # it THIS node would be undone the moment main() adds this node to
            # its executor, and the lookup would then wait on callbacks it is
            # itself blocking, so it gets a dedicated node. That node is created
            # here: node=None only works on tf2_ros >= 0.25.23, and the GO2
            # payload ships 0.25.20, where it crashes at startup.
            self._tf_node = rclpy.create_node(f"{self.get_name()}_tf_listener")
            self._tf_listener = tf2_ros.TransformListener(
                self._tf_buffer, self._tf_node, spin_thread=True)
        except ImportError:
            self.get_logger().error("tf2_ros unavailable: every non-map event will be quarantined")

        # -- segments (legacy spatial join for the ScoreRoutes service) --
        self._segment_seed = SegmentSeedResult(segments=[], frame_id=self.target_frame)
        seed_path = g("segment_seed_path")
        if seed_path:
            try:
                self._segment_seed = load_segment_seed(seed_path)
            except SegmentSeedError as exc:
                self.get_logger().error(f"segment seed load FAILED for {seed_path!r}: {exc}")

        self.core = MemoryCore(
            self.store, map_id=self.map_id, run_mode=self.run_mode,
            target_frame=self.target_frame,
            transform_lookup=self._lookup if self._tf_buffer is not None else None,
            segments=self._segment_seed.segments, seed_frame=self._segment_seed.frame_id,
            max_clock_skew_s=float(g("max_clock_skew_s")), field_params=field_params,
            clock=self._now_s)
        if self._segment_seed.segments and self._segment_seed.frame_id != self.target_frame:
            self.get_logger().warn(
                f"segment seed frame {self._segment_seed.frame_id!r} != {self.target_frame!r}: "
                f"segment joins are disabled (events are stored in {self.target_frame!r})")

        # -- ROS interfaces --
        self._grid_info: Optional[GridInfo] = None
        self._grid_state = "WAITING_FOR_MAP" if g("publish_grid") else "DISABLED"
        self._grid_published = 0
        self._shutting_down = False
        self._last_grid: Optional[List[int]] = None
        self._last_grid_time: Optional[float] = None
        self._map_msg_info = None
        self._health: List[str] = []
        self._status_pub = self.create_publisher(String, g("status_topic"), latched_qos())
        self._grid_pub = None
        if g("publish_grid"):
            self._grid_pub = self.create_publisher(OccupancyGrid, g("grid_topic"), latched_qos())
            self._map_sub = self.create_subscription(
                OccupancyGrid, g("map_topic"), self._on_map, latched_qos())
        self._sub = self.create_subscription(
            RiskEventMsg, "/riskgraph/risk_events", self._on_event, reliable_qos())
        self._srv = self.create_service(
            QuerySegmentRisk, "/riskgraph/query_segment_risk", self._on_query)
        self.create_timer(0.5, self._maybe_publish_grid)
        self.create_timer(max(0.2, float(g("status_period_s"))), self._publish_status)

        st = self.store.status()
        self.get_logger().info(
            f"riskgraph_memory ready: run_mode={self.run_mode} db={st['db_path']} "
            f"schema=v{st['schema_version']} map_id={self.map_id} "
            f"incidents={st['incident_count']} quarantined={st['quarantined_count']} "
            f"active_risk_entries={self.core.active_risk_entries} "
            f"instance={self.instance_id} pid={os.getpid()}")
        self._publish_status()

    # -- helpers --------------------------------------------------------------

    def _param(self, name: str):
        return self.get_parameter(name).value

    def _override_field_params(self, base: RiskFieldParams) -> RiskFieldParams:
        def pick(name, cur, cast):
            v = self._param(name)
            return cast(v) if v is not None and float(v) >= 0 else cur
        return RiskFieldParams(
            radius_m=pick("risk_radius_m", base.radius_m, float),
            decay_half_life_s=pick("risk_decay_half_life_s", base.decay_half_life_s, float),
            value_per_unit_risk=pick("risk_value_per_unit", base.value_per_unit_risk, float),
            max_cell_value=pick("risk_max_cell_value", base.max_cell_value, int),
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _lookup(self, target: str, source: str, t: float):
        from rclpy.duration import Duration
        from rclpy.time import Time
        try:
            tf = self._tf_buffer.lookup_transform(
                target, source, Time(nanoseconds=int(t * 1e9)),
                timeout=Duration(seconds=float(self._param("tf_timeout_s"))))
        except Exception as exc:  # tf2 raises several unrelated exception types
            raise TransformUnavailable(f"{type(exc).__name__}: {exc}") from exc
        tr, q = tf.transform.translation, tf.transform.rotation
        return (tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w)

    # -- callbacks --------------------------------------------------------------

    def _on_event(self, msg: RiskEventMsg) -> None:
        try:
            ev = core_event_from_msg(msg)
        except Exception as exc:
            self.get_logger().warn(f"dropped malformed RiskEvent: {exc}")
            return
        res = self.core.ingest(ev)
        if res.status == "stored":
            e = res.event
            self.get_logger().info(
                f"stored {e.event_id} [{e.provenance.value}] at "
                f"({e.position[0]:.3f}, {e.position[1]:.3f}) in {e.frame_id} "
                f"(from {e.source_frame_id}){' clock:' + e.clock_note if e.clock_note else ''}")
        elif res.status == "quarantined":
            n = self.core.quarantine_reasons.get(res.reason, 0)
            if n % WARN_EVERY == 1:
                self.get_logger().warn(
                    f"quarantined {ev.event_id}: {res.reason} ({res.detail}); "
                    f"{n} with this reason so far")
        elif res.status == "duplicate":
            self.get_logger().info(f"duplicate event_id {ev.event_id} ignored")
        else:
            self.get_logger().error(f"failed to persist {ev.event_id}: {res.detail}")
        self._publish_status()

    def _on_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        q = info.origin.orientation
        problems = []
        if msg.header.frame_id != self.target_frame:
            problems.append(f"/map frame {msg.header.frame_id!r} != {self.target_frame!r}")
        if abs(q.z) > 1e-6 or abs(q.x) > 1e-6 or abs(q.y) > 1e-6:
            problems.append("/map origin is rotated")
        try:
            gi = GridInfo(resolution=float(info.resolution), width=int(info.width),
                          height=int(info.height), origin_x=float(info.origin.position.x),
                          origin_y=float(info.origin.position.y))
        except ValueError as exc:
            problems.append(f"/map geometry invalid: {exc}")
            gi = None
        if gi is not None and self.expected_grid is not None and not gi.same_geometry(self.expected_grid):
            problems.append(
                f"/map geometry {gi} does not match {self.map_yaml} ({self.expected_grid}): "
                f"map_server is serving a different map than this node's map_id describes")
        if problems:
            self._grid_state = "MAP_MISMATCH"
            self._grid_info = None
            self._health = problems
            for pr in problems:
                self.get_logger().error(pr)
            self._publish_status()
            return
        self._map_msg_info = info
        self._grid_info = gi
        self._grid_state = "OK"
        self._health = []
        self._last_grid = None  # force a publish for the new map
        self._maybe_publish_grid()

    def _grid_msg(self, data: List[int]) -> OccupancyGrid:
        m = OccupancyGrid()
        m.header.frame_id = self.target_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.info = self._map_msg_info
        m.data = data
        return m

    def _maybe_publish_grid(self) -> None:
        # After the shutdown blank, the grid must stay blank: without this latch
        # the timer would republish the stored risk during the flush window and
        # Nav2 would keep planning around risk from a process that is exiting.
        if self._grid_pub is None or self._grid_info is None or self._shutting_down:
            return
        data = self.core.grid(self._grid_info)
        if data == self._last_grid:
            return
        self._grid_pub.publish(self._grid_msg(data))
        self._last_grid = data
        self._last_grid_time = time.time()
        self._grid_published += 1
        st = grid_stats(data)
        self.get_logger().info(
            f"published risk grid #{self._grid_published}: {st['nonzero_cells']} nonzero cells, "
            f"max {st['max_value']}")
        self._publish_status()

    def publish_blank_grid(self) -> bool:
        if self._grid_pub is None or self._grid_info is None or self._map_msg_info is None:
            return False
        self._shutting_down = True
        blank = [0] * (self._grid_info.width * self._grid_info.height)
        self._grid_pub.publish(self._grid_msg(blank))
        self._last_grid = blank
        self._grid_state = "SHUTDOWN_BLANK"
        return True

    def status_dict(self) -> dict:
        st = self.core.status()
        st.update({
            "node": self.get_fully_qualified_name(),
            "instance_id": self.instance_id,
            "pid": os.getpid(),
            "host": socket.gethostname(),
            "start_time": self.start_time,
            "stamp": time.time(),
            "map_yaml": self.map_yaml,
            "experiment_file": self.experiment.path if self.experiment else "",
            "grid_state": self._grid_state,
            "grid_published_count": self._grid_published,
            "grid_last_publish_time": self._last_grid_time,
            "grid_stats": grid_stats(self._last_grid) if self._last_grid is not None else None,
            "health": list(self._health),
        })
        return st

    def _publish_status(self) -> None:
        try:
            self._status_pub.publish(String(data=json.dumps(self.status_dict(), default=str)))
        except Exception as exc:  # status must never take the node down
            self.get_logger().error(f"status publish failed: {exc}")

    def _on_query(self, request: QuerySegmentRisk.Request,
                  response: QuerySegmentRisk.Response) -> QuerySegmentRisk.Response:
        decay = float(request.decay_half_life_s)
        risks, counts, dominants = [], [], []
        for seg_id in request.segment_ids:
            r, c, d = self.store.segment_risk(seg_id, decay_half_life_s=decay)
            risks.append(float(r))
            counts.append(int(c))
            dominants.append(d)
        response.risks = risks
        response.event_counts = counts
        response.dominant_factor_categories = dominants
        return response

    def shutdown(self) -> None:
        if self._param("blank_grid_on_shutdown"):
            if self.publish_blank_grid():
                self.get_logger().info("published blank risk grid on shutdown")
        self._publish_status()

    def destroy_node(self) -> bool:
        try:
            self.store.close()
        except Exception:
            pass
        if self._tf_listener is not None:
            try:
                self._tf_listener.executor.shutdown()
                self._tf_listener.dedicated_listener_thread.join(timeout=2.0)
            except Exception:
                pass
        if self._tf_node is not None:
            self._tf_node.destroy_node()
        return super().destroy_node()


def run_until_signal(node: Node, on_stop=None, flush_s: float = 0.5) -> None:
    """Spin until SIGINT/SIGTERM, then call ``on_stop`` and keep spinning
    briefly so the messages it publishes are actually delivered.

    rclpy's default handlers shut the context down before user code runs,
    so a node that must publish on the way out installs its own.
    """
    from rclpy.executors import SingleThreadedExecutor
    stop = {"flag": False}

    def _handler(signum, _frame):
        stop["flag"] = True

    signal.signal(signal.SIGINT, _handler)
    signal.signal(signal.SIGTERM, _handler)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    try:
        while rclpy.ok() and not stop["flag"]:
            ex.spin_once(timeout_sec=0.1)
        if on_stop is not None:
            on_stop()
            end = time.monotonic() + flush_s
            while rclpy.ok() and time.monotonic() < end:
                ex.spin_once(timeout_sec=0.05)
    finally:
        ex.remove_node(node)


def main(args=None) -> None:
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = RiskMemoryNode()
    except StartupError as exc:
        print(f"[riskgraph_memory] FATAL: {exc}", file=sys.stderr, flush=True)
        rclpy.try_shutdown()
        sys.exit(2)
    try:
        run_until_signal(node, on_stop=node.shutdown)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
