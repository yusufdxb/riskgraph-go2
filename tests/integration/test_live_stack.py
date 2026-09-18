"""RiskGraph + anchored localization + Nav2 as real processes, no robot.

Proves off-robot, on the same code that runs on the payload:
  * the TF chain map->odom->base_link is built from skewed-clock odometry;
  * RiskGraph can start before Nav2 (waits for /map) and all nodes share ONE
    absolute database;
  * an event in the odom frame is TF-transformed into map coordinates;
  * an event in a frame with no TF is quarantined, not guessed;
  * remembered risk changes Nav2's plan to the other corridor;
  * SIGINT exits cleanly, blanks the risk layer, Nav2 reverts; the restarted
    process restores the same risk from disk and the plan changes back;
  * risk on both corridors still yields a valid, deterministic plan;
  * a database bound to another map makes the memory node refuse to start.
"""
from __future__ import annotations

import json
import math
import os
import random
import signal
import sqlite3
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
ODOM = (3.2, -1.7, 0.6)  # odom origin != marker A, with rotation


def _env():
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(env.get("RG_IT_DOMAIN", random.randint(120, 200)))
    env.pop("ROS_LOCALHOST_ONLY", None)
    try:
        from ament_index_python.packages import get_package_prefix
        get_package_prefix("rmw_cyclonedds_cpp")
        env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
        env["CYCLONEDDS_URI"] = f"file://{HERE / 'cyclonedds_loopback.xml'}"
    except Exception:
        pass
    return env


ENV = _env()
os.environ.update({k: ENV[k] for k in ("ROS_DOMAIN_ID",) if k in ENV})
for k in ("RMW_IMPLEMENTATION", "CYCLONEDDS_URI"):
    if k in ENV:
        os.environ[k] = ENV[k]


class Proc:
    def __init__(self, name, cmd, logdir):
        self.log = open(logdir / f"{name}.log", "w")
        self.p = subprocess.Popen(cmd, stdout=self.log, stderr=subprocess.STDOUT, env=ENV,
                                  start_new_session=True)

    def stop(self, sig=signal.SIGINT, timeout=20.0):
        if self.p.poll() is None:
            os.killpg(self.p.pid, sig)
            try:
                return self.p.wait(timeout)
            except subprocess.TimeoutExpired:
                os.killpg(self.p.pid, signal.SIGKILL)
                return self.p.wait()
        return self.p.returncode


class Ros:
    """Test-side node: status topics, grids, planner action, event publisher."""

    def __init__(self):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.executors import MultiThreadedExecutor
        from nav2_msgs.action import ComputePathToPose
        from nav_msgs.msg import OccupancyGrid
        from std_msgs.msg import String
        from riskgraph_msgs.msg import RiskEvent
        from riskgraph_nav.ros_graph import LATCHED
        import tf2_ros
        rclpy.init()
        self.n = rclpy.create_node("rg_integration_test")
        self.status = None
        self.loc = None
        self.risk = None
        self.glob = None
        self.n.create_subscription(String, "/riskgraph/status", lambda m: setattr(self, "status", json.loads(m.data)), LATCHED)
        self.n.create_subscription(String, "/riskgraph/localization/status", lambda m: setattr(self, "loc", json.loads(m.data)), LATCHED)
        self.n.create_subscription(OccupancyGrid, "/riskgraph/risk_costmap", lambda m: setattr(self, "risk", m), LATCHED)
        self.n.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: setattr(self, "glob", m), LATCHED)
        self.pub = self.n.create_publisher(RiskEvent, "/riskgraph/risk_events", 10)
        self.ac = ActionClient(self.n, ComputePathToPose, "/compute_path_to_pose")
        self.CP = ComputePathToPose
        self.RiskEvent = RiskEvent
        self.buf = tf2_ros.Buffer()
        self.tl_node = rclpy.create_node("riskgraph_live_test_tf_listener")
        self.tl = tf2_ros.TransformListener(self.buf, self.tl_node, spin_thread=True)
        self.ex = MultiThreadedExecutor()
        self.ex.add_node(self.n)
        threading.Thread(target=self.ex.spin, daemon=True).start()

    def close(self):
        import rclpy
        self.tl.executor.shutdown()
        self.ex.shutdown()
        rclpy.try_shutdown()

    def wait(self, fn, timeout=30.0):
        end = time.time() + timeout
        while time.time() < end:
            try:
                if fn():
                    return True
            except Exception:
                pass
            time.sleep(0.1)
        return False

    @staticmethod
    def cell(g, x, y):
        i = int(math.floor((x - g.info.origin.position.x) / g.info.resolution))
        j = int(math.floor((y - g.info.origin.position.y) / g.info.resolution))
        return g.data[j * g.info.width + i]

    def plan(self, gx=5.0, gy=0.0, start=None):
        assert self.ac.wait_for_server(timeout_sec=10)
        g = self.CP.Goal()
        g.planner_id = "GridBased"
        g.goal.header.frame_id = "map"
        g.goal.pose.position.x, g.goal.pose.position.y = gx, gy
        g.goal.pose.orientation.w = 1.0
        if start:
            g.use_start = True
            g.start.header.frame_id = "map"
            g.start.pose.position.x, g.start.pose.position.y = start
            g.start.pose.orientation.w = 1.0
        ev = threading.Event()
        box = {}
        fut = self.ac.send_goal_async(g)
        fut.add_done_callback(lambda f: (box.__setitem__("gh", f.result()), ev.set()))
        assert ev.wait(10)
        ev2 = threading.Event()
        rf = box["gh"].get_result_async()
        rf.add_done_callback(lambda f: ev2.set())
        assert ev2.wait(20)
        return [(p.pose.position.x, p.pose.position.y) for p in rf.result().result.path.poses]

    def publish(self, x, y, frame="map", prov="OPERATOR_INJECTED", eid=None):
        e = self.RiskEvent()
        e.header.frame_id = frame
        e.header.stamp = self.n.get_clock().now().to_msg()
        e.event_id = eid or str(uuid.uuid4())
        e.position.x, e.position.y = x, y
        from riskgraph_msgs.msg import RiskFactor
        f = RiskFactor()
        f.category, f.severity, f.source = "OTHER", 1.0, "integration_test"
        e.factors = [f]
        e.confidence = 1.0
        e.provenance = prov
        self.pub.publish(e)
        return e.event_id


def side(pts):
    ys = [y for x, y in pts if 2.0 <= x <= 3.0]
    return "left" if sum(ys) / len(ys) > -0.15 else "right"


@pytest.fixture(scope="module")
def stack(tmp_path_factory):
    logdir = tmp_path_factory.mktemp("logs")
    db_root = tmp_path_factory.mktemp("db")
    procs = {}
    ros = Ros()
    rg_cmd = ["ros2", "launch", "riskgraph_bringup", "riskgraph_live.launch.py", "run_mode:=test",
              f"db_root:={db_root}", "db_tag:=it"]
    procs["odom"] = Proc("odom", [sys.executable, str(HERE / "fake_odom.py"), *map(str, ODOM)], logdir)
    # RiskGraph BEFORE Nav2: it must wait for /map without failing.
    procs["rg"] = Proc("rg1", rg_cmd, logdir)
    assert ros.wait(lambda: ros.status is not None, 40), (logdir / "rg1.log").read_text()[-3000:]
    pre_nav_status = dict(ros.status)
    procs["nav"] = Proc("nav", ["ros2", "launch", "riskgraph_bringup", "riskgraph_nav_live.launch.py"], logdir)
    assert ros.wait(lambda: ros.loc and ros.loc.get("localization_valid"), 40), \
        (logdir / "nav.log").read_text()[-3000:]
    assert ros.wait(lambda: ros.status and ros.status.get("grid_state") == "OK", 40)
    assert ros.wait(lambda: ros.glob is not None, 40)
    yield dict(ros=ros, procs=procs, logdir=logdir, rg_cmd=rg_cmd, db_root=db_root,
               pre_nav_status=pre_nav_status)
    for p in reversed(list(procs.values())):
        p.stop()
    ros.close()


def test_riskgraph_started_before_nav2_waits_for_the_map(stack):
    st = stack["pre_nav_status"]
    assert st["grid_state"] == "WAITING_FOR_MAP"
    assert st["schema_version"] == 2 and os.path.isabs(st["db_path"])


def test_tf_chain_from_skewed_odometry_puts_robot_on_marker(stack):
    ros = stack["ros"]
    from rclpy.time import Time
    t = ros.buf.lookup_transform("map", "base_link", Time())
    assert (t.transform.translation.x, t.transform.translation.y) == pytest.approx((0.0, 0.0), abs=1e-6)
    now = ros.n.get_clock().now().nanoseconds * 1e-9
    stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
    assert abs(now - stamp) < 1.0, "odom->base_link must be stamped on the local clock"
    assert ros.loc["robot_clock_skew_s"] > 1e7


def test_all_nodes_share_one_absolute_database(stack):
    ros = stack["ros"]
    st = ros.status
    expected = str(stack["db_root"] / st["map_id"] / "it.sqlite")
    assert st["db_path"] == expected
    r = subprocess.run(["ros2", "param", "get", "/riskgraph_planner", "store_path"],
                       capture_output=True, text=True, env=ENV, timeout=20)
    assert expected in r.stdout


def test_odom_event_is_transformed_into_map(stack):
    ros = stack["ros"]
    # 1 m straight ahead of the robot, expressed in the ODOM frame.
    x, y, yaw = ODOM
    eid = ros.publish(x + math.cos(yaw), y + math.sin(yaw), frame="odom",
                      prov="HARDWARE_DERIVED")
    db = ros.status["db_path"]
    assert ros.wait(lambda: sqlite3.connect(db).execute(
        "SELECT COUNT(*) FROM risk_event WHERE event_id=?", (eid,)).fetchone()[0] == 1, 10)
    row = sqlite3.connect(db).execute(
        "SELECT position_x, position_y, frame_id, source_frame_id FROM risk_event WHERE event_id=?",
        (eid,)).fetchone()
    assert row[0] == pytest.approx(1.0, abs=1e-6) and row[1] == pytest.approx(0.0, abs=1e-6)
    assert row[2:] == ("map", "odom")
    sqlite3.connect(db).execute("DELETE FROM risk_event WHERE event_id=?", (eid,)).connection.commit()


def test_event_without_tf_is_quarantined(stack):
    ros = stack["ros"]
    eid = ros.publish(1.0, 1.0, frame="no_such_frame")
    db = ros.status["db_path"]
    assert ros.wait(lambda: sqlite3.connect(db).execute(
        "SELECT reason FROM quarantine WHERE event_id=?", (eid,)).fetchone() is not None, 10)
    reason = sqlite3.connect(db).execute("SELECT reason FROM quarantine WHERE event_id=?", (eid,)).fetchone()[0]
    assert reason == "TF_UNAVAILABLE"


def _restart_rg(stack, n):
    ros = stack["ros"]
    old = ros.status["instance_id"]
    stack["procs"]["rg"] = Proc(f"rg{n}", stack["rg_cmd"], stack["logdir"])
    assert ros.wait(lambda: ros.status["instance_id"] != old and ros.status["grid_state"] == "OK", 40)


def test_remembered_risk_changes_the_nav2_route_and_survives_restart(stack):
    ros = stack["ros"]
    # the odom-frame test event was deleted from the DB directly; restart so the
    # node's representation matches the file before the baseline
    code = stack["procs"]["rg"].stop()
    assert code == 0
    _restart_rg(stack, 2)
    assert ros.wait(lambda: max(ros.risk.data) == 0, 10)
    time.sleep(2.5)
    base = ros.plan()
    assert side(base) == "left"
    ros.publish(2.5, 1.05)
    assert ros.wait(lambda: ros.cell(ros.glob, 2.5, 1.05) > 60, 15), "Nav2 never saw the risk"
    aware = ros.plan()
    assert side(aware) == "right"
    n_before = ros.status["incident_count"]

    # clean SIGINT: exit 0, blank grid, Nav2 cost falls back, plan reverts
    code = stack["procs"]["rg"].stop()
    assert code == 0, (stack["logdir"] / "rg2.log").read_text()[-2000:]
    assert ros.wait(lambda: max(ros.risk.data) == 0, 10)
    assert ros.wait(lambda: ros.cell(ros.glob, 2.5, 1.05) < 60, 15)
    assert side(ros.plan()) == "left"

    # restart from disk: same incidents, risk and route come back
    _restart_rg(stack, 3)
    assert ros.status["incident_count"] == n_before
    assert ros.wait(lambda: ros.cell(ros.glob, 2.5, 1.05) > 60, 15)
    assert side(ros.plan()) == "right"


def test_fallback_when_both_corridors_are_risky(stack):
    ros = stack["ros"]
    ros.publish(2.5, -1.25)
    assert ros.wait(lambda: ros.cell(ros.glob, 2.5, -1.25) > 60, 15)
    plans = [ros.plan(start=(0.0, 0.0)) for _ in range(3)]
    assert all(len(p) > 10 for p in plans)
    assert all(math.isfinite(v) for p in plans for xy in p for v in xy)
    assert plans[0] == plans[1] == plans[2], "planning must be deterministic, no oscillation"
    assert max(ros.risk.data) <= 90
    into = ros.plan(2.5, 1.05, start=(0.0, 0.0))  # goal inside a risk region is still reachable
    assert len(into) > 5


def test_database_bound_to_another_map_refuses_to_start(stack, tmp_path):
    from riskgraph_core.store import RiskStore
    db = tmp_path / "other.sqlite"
    RiskStore(str(db), map_id="some-other-course-000000000000", evidence_class="test").close()
    r = subprocess.run(
        ["ros2", "run", "riskgraph_memory", "riskgraph_memory_node", "--ros-args",
         "-r", "__node:=riskgraph_memory_wrongmap",
         "-p", f"store_path:={db}", "-p", "run_mode:=test", "-p", "publish_grid:=false",
         "-p", f"experiment_file:={_experiment()}"],
        capture_output=True, text=True, env=ENV, timeout=30)
    assert r.returncode == 2
    assert "FATAL" in r.stderr and "some-other-course" in r.stderr


def _experiment():
    from riskgraph_nav.paths import default_experiment_file
    return default_experiment_file()
