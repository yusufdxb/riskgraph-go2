"""ROS side of the trial runner: observes the graph, talks to Nav2 actions,
publishes risk observations and trial markers. Publishes NO motion command.

The only things this class publishes:
  /riskgraph/risk_events    (OPERATOR_INJECTED observations, after typed confirmation)
  /riskgraph/trial/markers  (std_msgs/String JSON: trial boundaries, for the bag)
  /rehearsal/walk_to        (REHEARSAL ONLY: simulates the operator walking the robot)
"""
from __future__ import annotations

import json
import math
import threading
import time
from typing import Dict, List, Optional, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String

from riskgraph_core.geometry import Pose2D, quat_from_yaw, yaw_from_quat
from riskgraph_core.risk_field import RiskField
from riskgraph_msgs.msg import RiskEvent, RiskFactor

from .ros_graph import LATCHED, SENSOR, GraphProbe

_STATUS = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_ABORTED: "ABORTED",
           GoalStatus.STATUS_CANCELED: "CANCELED", GoalStatus.STATUS_UNKNOWN: "UNKNOWN"}


class TrialRos:
    def __init__(self, runner) -> None:
        from rclpy.action import ActionClient
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from nav2_msgs.action import ComputePathToPose, FollowPath
        self.r = runner
        self.probe = GraphProbe("riskgraph_live_trial")
        n = self.probe.node
        self._lock = threading.Lock()
        self.rg_status: Optional[dict] = None
        self.rg_status_t = 0.0
        self.loc_status: Optional[dict] = None
        self.loc_status_t = 0.0
        self.risk_grid: Optional[OccupancyGrid] = None
        self.map_grid: Optional[OccupancyGrid] = None
        self.global_grid: Optional[OccupancyGrid] = None
        self.plan_msgs = 0
        self.stop_requested = False
        self._last_nonzero_cmd = None
        self._arb = None
        self._arb_t = 0.0
        self._arb_counts = {"ticks": 0, "hold_ticks": 0, "nonzero_ticks": 0, "nonzero_while_hold": 0}
        n.create_subscription(String, "/riskgraph/status", self._on_rg, LATCHED)
        n.create_subscription(String, "/riskgraph/localization/status", self._on_loc, LATCHED)
        n.create_subscription(OccupancyGrid, "/riskgraph/risk_costmap", self._set("risk_grid"), LATCHED)
        n.create_subscription(OccupancyGrid, "/map", self._set("map_grid"), LATCHED)
        n.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._set("global_grid"), LATCHED)
        n.create_subscription(Path, "/plan", self._on_plan, 10)
        n.create_subscription(Twist, "/cmd_vel", self._on_cmd, SENSOR)
        try:
            from helix_msgs.msg import ArbiterStatus
            n.create_subscription(ArbiterStatus, "/helix/arbiter/status", self._on_arb, SENSOR)
            self.have_arbiter_type = True
        except ImportError:
            self.have_arbiter_type = False
        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self._event_pub = n.create_publisher(RiskEvent, "/riskgraph/risk_events", rel)
        self._marker_pub = n.create_publisher(String, "/riskgraph/trial/markers", rel)
        self._walk_pub = n.create_publisher(PoseStamped, "/rehearsal/walk_to", 10) \
            if runner.rehearsal else None
        self._ComputePath = ComputePathToPose
        self._FollowPath = FollowPath
        self._plan_ac = ActionClient(n, ComputePathToPose, "/compute_path_to_pose")
        self._follow_ac = ActionClient(n, FollowPath, "/follow_path")
        self.probe.tf_buffer()
        time.sleep(1.0)

    # -- callbacks ------------------------------------------------------------------

    def _set(self, attr):
        def cb(m):
            setattr(self, attr, m)
        return cb

    def _on_rg(self, m) -> None:
        try:
            self.rg_status = json.loads(m.data)
            self.rg_status_t = time.time()
        except ValueError:
            pass

    def _on_loc(self, m) -> None:
        try:
            self.loc_status = json.loads(m.data)
            self.loc_status_t = time.time()
        except ValueError:
            pass

    def _on_plan(self, _m) -> None:
        self.plan_msgs += 1

    def _on_cmd(self, m: Twist) -> None:
        if abs(m.linear.x) > 1e-6 or abs(m.linear.y) > 1e-6 or abs(m.angular.z) > 1e-6:
            self._last_nonzero_cmd = time.time()

    def _on_arb(self, m) -> None:
        self._arb = m
        self._arb_t = time.time()
        c = self._arb_counts
        c["ticks"] += 1
        nz = m.out_linear_x != 0.0 or m.out_linear_y != 0.0 or m.out_angular_z != 0.0
        if m.hold_active:
            c["hold_ticks"] += 1
            if nz:
                c["nonzero_while_hold"] += 1
        if nz:
            c["nonzero_ticks"] += 1

    def request_stop(self) -> None:
        self.stop_requested = True

    def close(self) -> None:
        self.probe.close()

    # -- waiting ------------------------------------------------------------------------

    def wait_until(self, fn, timeout: float, settle: float = 0.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            try:
                if fn():
                    if settle:
                        time.sleep(settle)
                        if not fn():
                            continue
                    return True
            except Exception:
                pass
            time.sleep(0.1)
        return False

    def wait_status(self, pred, timeout: float) -> Optional[dict]:
        ok = self.wait_until(lambda: self.rg_status is not None and pred(self.rg_status), timeout)
        return self.rg_status if ok else None

    # -- pose ---------------------------------------------------------------------------

    def robot_pose(self) -> Optional[dict]:
        t, age = self.probe.lookup("map", "base_link", 0.2)
        if t is None or (age is not None and age > 0.5):
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return {"x": tr.x, "y": tr.y, "yaw": yaw_from_quat((q.x, q.y, q.z, q.w)), "tf_age_s": age,
                "tf": {"frame": "map", "child": "base_link",
                       "translation": [tr.x, tr.y, tr.z], "rotation": [q.x, q.y, q.z, q.w],
                       "stamp": t.header.stamp.sec + t.header.stamp.nanosec * 1e-9}}

    def speed(self) -> Optional[float]:
        if self.loc_status is None or time.time() - self.loc_status_t > 1.0:
            return None
        return self.loc_status.get("speed_mps")

    def stationary(self) -> bool:
        s = self.speed()
        return s is not None and s < self.r.exp.stationary_speed_mps and \
            bool((self.loc_status or {}).get("stationary"))

    def map_to_odom_target(self, x: float, y: float, yaw: float) -> Tuple[float, float, float]:
        t, _ = self.probe.lookup("odom", "map", 0.5)
        if t is None:
            raise RuntimeError("no odom<-map transform")
        tr, q = t.transform.translation, t.transform.rotation
        T = Pose2D(tr.x, tr.y, yaw_from_quat((q.x, q.y, q.z, q.w)))
        p = T.compose(Pose2D(x, y, yaw))
        return p.x, p.y, p.yaw

    def rehearsal_walk_to(self, x: float, y: float, yaw: float) -> None:
        """REHEARSAL ONLY: stand-in for the operator walking the robot with the remote."""
        if self._walk_pub is None:
            raise RuntimeError("walk_to is only available in rehearsal mode")
        ox, oy, oyaw = self.map_to_odom_target(x, y, yaw)
        m = PoseStamped()
        m.header.frame_id = "odom"
        m.pose.position.x, m.pose.position.y = ox, oy
        q = quat_from_yaw(oyaw)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
        self._walk_pub.publish(m)
        self.r.log(f"[REHEARSAL] simulated operator walk to map ({x:.2f}, {y:.2f}, {yaw:.2f})")
        time.sleep(1.0)

    # -- Nav2 ------------------------------------------------------------------------------

    def _wait_future(self, fut, timeout: float):
        ev = threading.Event()
        fut.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout):
            return None
        return fut.result()

    def compute_path(self, goal: Pose2D, start: Optional[Pose2D] = None
                     ) -> Tuple[List[Tuple[float, float]], str, Optional[float]]:
        if not self._plan_ac.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("/compute_path_to_pose not available")
        g = self._ComputePath.Goal()
        g.planner_id = "GridBased"
        g.goal.header.frame_id = "map"
        g.goal.pose.position.x, g.goal.pose.position.y = goal.x, goal.y
        q = quat_from_yaw(goal.yaw)
        g.goal.pose.orientation.x, g.goal.pose.orientation.y, g.goal.pose.orientation.z, \
            g.goal.pose.orientation.w = q
        if start is not None:
            g.use_start = True
            g.start.header.frame_id = "map"
            g.start.pose.position.x, g.start.pose.position.y = start.x, start.y
            qs = quat_from_yaw(start.yaw)
            g.start.pose.orientation.x, g.start.pose.orientation.y, g.start.pose.orientation.z, \
                g.start.pose.orientation.w = qs
        gh = self._wait_future(self._plan_ac.send_goal_async(g), 10.0)
        if gh is None or not gh.accepted:
            raise RuntimeError("ComputePathToPose goal rejected")
        res = self._wait_future(gh.get_result_async(), 20.0)
        if res is None:
            raise RuntimeError("ComputePathToPose timed out (planning deadlock?)")
        if res.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"ComputePathToPose ended with status {res.status}")
        path = res.result.path
        pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        pt = res.result.planning_time
        return pts, path.header.frame_id, pt.sec + pt.nanosec * 1e-9

    def send_follow_path(self, points: List[Tuple[float, float]]):
        if not self._follow_ac.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("/follow_path not available")
        g = self._FollowPath.Goal()
        g.controller_id = "FollowPath"
        g.goal_checker_id = "goal_checker"
        g.path.header.frame_id = "map"
        g.path.header.stamp = self.probe.node.get_clock().now().to_msg()
        for i, (x, y) in enumerate(points):
            ps = PoseStamped()
            ps.header = g.path.header
            ps.pose.position.x, ps.pose.position.y = x, y
            nx, ny = points[min(i + 1, len(points) - 1)]
            px, py = points[max(i - 1, 0)]
            yaw = math.atan2(ny - py, nx - px) if (nx, ny) != (px, py) else 0.0
            q = quat_from_yaw(yaw)
            ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q
            g.path.poses.append(ps)
        gh = self._wait_future(self._follow_ac.send_goal_async(g), 10.0)
        if gh is None or not gh.accepted:
            raise RuntimeError("FollowPath goal rejected by controller_server")
        gh._rg_result = gh.get_result_async()
        return gh

    def follow_status(self, gh) -> Tuple[bool, Optional[str]]:
        fut = gh._rg_result
        if not fut.done():
            return False, None
        res = fut.result()
        return True, _STATUS.get(res.status, str(res.status))

    def cancel(self, gh) -> bool:
        try:
            r = self._wait_future(gh.cancel_goal_async(), 3.0)
            return bool(r is not None and len(r.goals_canceling) > 0)
        except Exception:
            return False

    def nav_states(self) -> Dict[str, Optional[str]]:
        return {n: self.probe.lifecycle_state(n) for n in
                ("/map_server", "/planner_server", "/controller_server", "/velocity_smoother")}

    def sink_params(self) -> Optional[dict]:
        return self.probe.params("/helix_go2_sport_sink", ["mode", "max_vx", "max_wz"])

    # -- grids ---------------------------------------------------------------------------------

    @staticmethod
    def _cell(g: Optional[OccupancyGrid], x: float, y: float) -> Optional[int]:
        if g is None:
            return None
        i = int(math.floor((x - g.info.origin.position.x) / g.info.resolution))
        j = int(math.floor((y - g.info.origin.position.y) / g.info.resolution))
        if 0 <= i < g.info.width and 0 <= j < g.info.height:
            return int(g.data[j * g.info.width + i])
        return None

    def global_cost_at(self, x, y):
        return self._cell(self.global_grid, x, y)

    def risk_value_at(self, x, y):
        return self._cell(self.risk_grid, x, y)

    def risk_grid_max(self) -> Optional[int]:
        return max(self.risk_grid.data) if self.risk_grid is not None else None

    def risk_grid_data(self) -> List[int]:
        return list(self.risk_grid.data) if self.risk_grid is not None else []

    def static_lethal(self, x: float, y: float) -> bool:
        v = self._cell(self.map_grid, x, y)
        return v is None or v >= 65 or v < 0

    def path_costs(self, pts) -> Dict[str, object]:
        g = [self.global_cost_at(x, y) for x, y in pts]
        r = [self.risk_value_at(x, y) for x, y in pts]
        g2 = [v for v in g if v is not None]
        r2 = [v for v in r if v is not None]
        return {"nav2_costmap_sum": sum(g2), "nav2_costmap_max": max(g2) if g2 else None,
                "riskgraph_grid_sum": sum(r2), "riskgraph_grid_max": max(r2) if r2 else None,
                "cells": len(pts), "note": "nav2 values are /global_costmap/costmap (0..100 scale)"}

    def current_field(self) -> RiskField:
        from riskgraph_core.store import RiskStore, StoreError
        try:
            with RiskStore(self.r.store_path, readonly=True, map_id=self.r.map_id) as s:
                events = s.all_events(frame_id="map")
        except StoreError:
            events = []
        return RiskField.from_events(events, self.r.field_params, now=time.time())

    # -- monitoring -----------------------------------------------------------------------------

    def arbiter_summary(self) -> Dict[str, object]:
        m = self._arb
        if m is None:
            return {"seen": False, "type_available": self.have_arbiter_type}
        return {"seen": True, "age_s": round(time.time() - self._arb_t, 3), "reason": m.reason,
                "hold_active": bool(m.hold_active), "selected": m.selected_source,
                "sink_subscribers": int(m.sink_subscribers)}

    def arbiter_reset_counters(self) -> None:
        for k in self._arb_counts:
            self._arb_counts[k] = 0

    def arbiter_counters(self) -> Dict[str, int]:
        return dict(self._arb_counts)

    def motion_sources(self) -> Dict[str, List[str]]:
        p = self.probe
        return {t: sorted({e["node"] for e in p.publishers(t)})
                for t in ("/cmd_vel", "/nav/cmd_vel", "/api/sport/request")}

    def last_cmd_vel_nonzero_age(self) -> Optional[float]:
        return None if self._last_nonzero_cmd is None else time.time() - self._last_nonzero_cmd

    def preexec_problems(self) -> List[str]:
        out = []
        loc = self.loc_status or {}
        if not loc.get("localization_valid") or time.time() - self.loc_status_t > 1.0:
            out.append(f"localization not valid/fresh ({loc.get('state')})")
        if not self.stationary():
            out.append("robot is moving")
        a = self.arbiter_summary()
        if not a.get("seen") or a.get("age_s", 99) > 0.5:
            out.append(f"motion arbiter status not fresh: {a}")
        elif a.get("hold_active"):
            out.append(f"HELIX hold active ({a.get('reason')})")
        st = self.rg_status or {}
        if st.get("health") or st.get("map_id") != self.r.map_id:
            out.append(f"RiskGraph unhealthy: {st.get('health')} map={st.get('map_id')}")
        return out

    def live_abort_condition(self, sources0: Dict[str, List[str]]) -> Optional[str]:
        now = time.time()
        loc = self.loc_status or {}
        if now - self.loc_status_t > 1.0:
            return "LOCALIZATION_STATUS_STALE: no localization status for > 1 s"
        if not loc.get("localization_valid"):
            return f"LOCALIZATION_INVALID: {loc.get('state')} (jumps={loc.get('odom_jumps')})"
        if (loc.get("odom_age_s") or 0) > 0.5:
            return f"ROBOT_STATE_LOST: odometry age {loc.get('odom_age_s')} s"
        a = self.arbiter_summary()
        if not a.get("seen") or a.get("age_s", 99) > 0.5:
            return f"ARBITER_LOST: {a}"
        if a.get("hold_active"):
            return f"HELIX_HOLD: {a.get('reason')} (trial confounded; the arbiter is forcing zero)"
        st = self.rg_status or {}
        if st.get("map_id") != self.r.map_id:
            return f"MAP_ID_MISMATCH: RiskGraph reports {st.get('map_id')}"
        if st.get("health"):
            return f"RISKGRAPH_UNHEALTHY: {st.get('health')}"
        gs = st.get("grid_stats") or {}
        if gs.get("out_of_range_cells"):
            return f"RISKGRAPH_INVALID_GRID: {gs}"
        if int(now * 10) % 10 == 0:  # graph queries are not free: once a second
            cur = self.motion_sources()
            if cur != sources0:
                return f"MOTION_SOURCE_CHANGED: {sources0} -> {cur}"
        return None

    # -- publishing (observations and markers only) -------------------------------------------------

    def publish_event(self, eid: str, x: float, y: float, t: float, severity: float, detail: str,
                      map_id: str) -> None:
        e = RiskEvent()
        e.header.frame_id = "map"
        e.header.stamp.sec = int(t)
        e.header.stamp.nanosec = int((t - int(t)) * 1e9)
        e.event_id = eid
        e.position.x, e.position.y = x, y
        f = RiskFactor()
        f.category = "OTHER"
        f.severity = float(severity)
        f.source = "operator_injected"
        f.detail = detail
        e.factors = [f]
        e.confidence = 1.0
        e.provenance = "OPERATOR_INJECTED"
        e.source_frame_id = "base_link"
        e.map_id = map_id
        self._event_pub.publish(e)

    def marker(self, d: dict) -> None:
        self._marker_pub.publish(String(data=json.dumps(dict(d, t=time.time()), default=str)))
