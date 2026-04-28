#!/usr/bin/env python3
"""ROS 2 end-to-end smoke test for RiskGraph-Go2.

Spawns the memory + planner nodes in-process, publishes synthetic RiskEvent
messages onto /riskgraph/risk_events, then calls the /riskgraph/score_routes
service with two candidate routes and asserts the planner picks the safe one
and the explanation cites at least one event.

This is the runtime-side counterpart to the offline pytest regression: it
confirms the wiring (publisher → memory_node → store → planner_node → service
response → explanation) works against actual rclpy, not just the core lib.

Usage:
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 scripts/ros_end_to_end_check.py
"""
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header
from geometry_msgs.msg import Point

from riskgraph_msgs.msg import (
    RiskEvent as RiskEventMsg,
    RiskFactor as RiskFactorMsg,
    Route as RouteMsg,
    RouteSegment as RouteSegmentMsg,
)
from riskgraph_msgs.srv import ScoreRoutes

from riskgraph_memory.memory_node import RiskMemoryNode
from riskgraph_planner.planner_node import PlannerNode


SHARED_DB = "/tmp/riskgraph_e2e.sqlite"


def _ev(eid, x=2.0, y=0.0, severity=0.9, category="SLIP", source="tactile/slip_state"):
    msg = RiskEventMsg()
    msg.header = Header()
    now = time.time()
    msg.header.stamp.sec = int(now)
    msg.header.stamp.nanosec = 0
    msg.header.frame_id = "map"
    msg.event_id = eid
    msg.position = Point(x=float(x), y=float(y), z=0.0)
    f = RiskFactorMsg()
    f.category = category
    f.severity = float(severity)
    f.source = source
    f.detail = "smoke test"
    msg.factors = [f]
    msg.confidence = 1.0
    return msg


def _seg(seg_id, x0, y0, x1, y1):
    s = RouteSegmentMsg()
    s.segment_id = seg_id
    s.start = Point(x=float(x0), y=float(y0), z=0.0)
    s.end = Point(x=float(x1), y=float(y1), z=0.0)
    s.length_m = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
    s.semantic_label = ""
    return s


def _route(rid, segs):
    r = RouteMsg()
    r.header = Header()
    r.header.frame_id = "map"
    r.route_id = rid
    r.segments = segs
    return r


def main() -> int:
    # Reset shared store so the test is hermetic.
    p = Path(SHARED_DB)
    if p.exists():
        p.unlink()

    rclpy.init()
    executor = SingleThreadedExecutor()

    # Bring up memory + planner nodes pointed at the same SQLite file.
    import rclpy.parameter as rclparam
    memory = RiskMemoryNode()
    memory.set_parameters([rclparam.Parameter("store_path", rclparam.Parameter.Type.STRING, SHARED_DB)])
    # The store was opened with the default at __init__ — re-open against the shared file.
    from riskgraph_core.store import RiskStore
    memory._store.close()
    memory._store = RiskStore(SHARED_DB)
    # Pre-register a known segment so spatial join assigns events to "glossy".
    from riskgraph_core.segments import RouteSegment
    memory.register_segments([
        RouteSegment(segment_id="glossy",  start=(0.0, 0.0, 0.0), end=(4.0, 0.0, 0.0)),
        RouteSegment(segment_id="safe-a",  start=(0.0, 5.0, 0.0), end=(4.0, 5.0, 0.0)),
    ])

    planner = PlannerNode()
    planner._store.close()
    planner._store = RiskStore(SHARED_DB)

    executor.add_node(memory)
    executor.add_node(planner)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Publish 3 slip events into the memory node's subscription.
    helper = rclpy.create_node("e2e_publisher")
    qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST, depth=10)
    pub = helper.create_publisher(RiskEventMsg, "/riskgraph/risk_events", qos)
    helper_executor = SingleThreadedExecutor()
    helper_executor.add_node(helper)
    t_helper = threading.Thread(target=helper_executor.spin, daemon=True)
    t_helper.start()

    # Brief settling for discovery.
    time.sleep(0.5)
    for i in range(3):
        msg = _ev(f"e2e_slip_{i}", x=2.0, y=0.0)
        # The synthetic position (2, 0) lies on segment "glossy"
        pub.publish(msg)
        time.sleep(0.05)
    time.sleep(0.5)  # let the memory_node consume

    # Now call the score_routes service.
    short = _route("short", [_seg("glossy", 0.0, 0.0, 4.0, 0.0)])
    long_ = _route("long",  [_seg("safe-a", 0.0, 5.0, 4.0, 5.0)])

    cli = helper.create_client(ScoreRoutes, "/riskgraph/score_routes")
    if not cli.wait_for_service(timeout_sec=3.0):
        print("ERROR: /riskgraph/score_routes did not appear", file=sys.stderr)
        return 2
    req = ScoreRoutes.Request()
    req.candidates = [short, long_]
    req.semantic_objective = ""
    fut = cli.call_async(req)
    deadline = time.time() + 5.0
    while not fut.done() and time.time() < deadline:
        time.sleep(0.05)
    if not fut.done():
        print("ERROR: service call did not complete in 5s", file=sys.stderr)
        return 3
    resp = fut.result()
    chosen = resp.result.chosen_route_id
    print("chosen_route_id:", chosen)
    print("scores:")
    for s in resp.result.scores:
        print(f"  {s.route_id}: total={s.total_cost:.3f} risk={s.risk_cost:.3f} dom={list(s.dominant_segment_ids)}")
    print("explanation:", resp.explanation.text)
    print("evidence ids:", list(resp.explanation.evidence_event_ids))

    ok = chosen == "long" and len(resp.explanation.evidence_event_ids) >= 1
    print("VERDICT:", "PASS" if ok else "FAIL")

    helper_executor.shutdown()
    executor.shutdown()
    helper.destroy_node()
    planner.destroy_node()
    memory.destroy_node()
    rclpy.shutdown()

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
