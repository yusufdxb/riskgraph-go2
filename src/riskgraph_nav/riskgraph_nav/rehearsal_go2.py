"""REHEARSAL robot for off-robot runs of the live trial. NOT a GO2 model.

Stands in for the GO2 at the two interfaces the trial touches:

* consumes ``unitree_api/msg/Request`` on ``/api/sport/request`` exactly as
  the HELIX sport sink publishes it (Move 1008, StopMove 1003; anything else
  is answered with code -1 and counted), answers on ``/api/sport/response``;
* publishes ``/utlidar/robot_odom`` (nav_msgs/Odometry, 150 Hz, ``odom`` ->
  ``base_link``) with header stamps offset by ``clock_skew_s``: by default
  the skew measured on the lab robot (robot stamps months behind the
  payload), so the rehearsal exercises the restamping that the real robot
  needs.

The odometry origin is deliberately NOT the start marker (``x0, y0, yaw0``),
so the map anchoring math is exercised with a real rotation and translation.

``/rehearsal/walk_to`` (geometry_msgs/PoseStamped in ``odom``) simulates the
operator walking the robot back to a marker with the handheld remote: the
robot moves there kinematically, ignoring sport commands until it arrives.
The live trial runner uses it ONLY in rehearsal mode.

Dynamics: first-order velocity lag toward the last Move target, Move older
than ``move_timeout_s`` decays to zero. Nothing here is measured on a GO2.
"""
from __future__ import annotations

import json
import math
import sys
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String

#: Measured on the lab GO2 (2026-09): robot stamps ~2025-10-17 during a
#: 2026-09 session. The exact value does not matter, only that it is huge.
MEASURED_SKEW_S = -28_300_000.0


class RehearsalGo2(Node):
    def __init__(self) -> None:
        super().__init__("rehearsal_go2")
        from unitree_api.msg import Request, Response  # noqa: F401 (fail early if absent)
        self._Response = Response
        p = self.declare_parameter
        p("tau_s", 0.25)
        p("move_timeout_s", 1.0)
        p("clock_skew_s", MEASURED_SKEW_S)
        p("x0", 3.2)
        p("y0", -1.7)
        p("yaw0", 0.6)
        p("walk_speed_mps", 0.5)
        p("walk_yaw_rate", 1.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._tau = float(g("tau_s"))
        self._move_timeout = float(g("move_timeout_s"))
        self._skew = float(g("clock_skew_s"))
        self._walk_v = float(g("walk_speed_mps"))
        self._walk_w = float(g("walk_yaw_rate"))
        self._pose = [float(g("x0")), float(g("y0")), float(g("yaw0"))]
        self._v = [0.0, 0.0, 0.0]
        self._target = (0.0, 0.0, 0.0)
        self._target_t = 0.0
        self._walk = None
        self._t = time.monotonic()
        self.counts = {"move": 0, "stop": 0, "bad_api": 0, "walks": 0}
        self.create_subscription(Request, "/api/sport/request", self._on_req, 50)
        self.create_subscription(PoseStamped, "/rehearsal/walk_to", self._on_walk, 10)
        self._pub_resp = self.create_publisher(Response, "/api/sport/response", 50)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self._pub_odom = self.create_publisher(Odometry, "/utlidar/robot_odom", qos)
        self._pub_state = self.create_publisher(String, "/rehearsal/state", 10)
        self.create_timer(1.0 / 150.0, self._step)
        self.create_timer(1.0, self._state_tick)
        # The other topics the HELIX monitors watch on a real GO2 (rates from
        # the GO2 field notes), so HELIX sees a healthy robot and does not hold.
        self._pub_imu = self.create_publisher(Imu, "/utlidar/imu", qos_profile_sensor_data)
        self._pub_cloud = self.create_publisher(PointCloud2, "/utlidar/cloud", qos_profile_sensor_data)
        self._pub_pose = self.create_publisher(PoseStamped, "/utlidar/robot_pose", 10)
        self._pub_gnss = self.create_publisher(String, "/gnss", 10)
        self._pub_multi = self.create_publisher(String, "/multiplestate", 10)
        self.create_timer(1.0 / 250.0, lambda: self._pub_imu.publish(self._stamped(Imu())))
        self.create_timer(1.0 / 15.0, lambda: self._pub_cloud.publish(self._stamped(PointCloud2())))
        self.create_timer(1.0 / 20.0, self._pose_tick)
        self.create_timer(1.0, self._json_tick)
        self.get_logger().warn("REHEARSAL robot: evidence produced against it is NOT hardware evidence")

    def _stamped(self, m):
        stamp = time.time() + self._skew
        m.header.stamp.sec = int(stamp)
        m.header.stamp.nanosec = int((stamp % 1) * 1e9)
        m.header.frame_id = "base_link"
        return m

    def _pose_tick(self) -> None:
        m = self._stamped(PoseStamped())
        m.header.frame_id = "odom"
        m.pose.position.x, m.pose.position.y = self._pose[0], self._pose[1]
        m.pose.orientation.z = math.sin(self._pose[2] / 2)
        m.pose.orientation.w = math.cos(self._pose[2] / 2)
        self._pub_pose.publish(m)

    def _json_tick(self) -> None:
        self._pub_gnss.publish(String(data=json.dumps(
            {"satellite_total": 0, "satellite_inuse": 0, "hdop": 0.0})))
        self._pub_multi.publish(String(data=json.dumps(
            {"volume": 5, "brightness": 0, "obstaclesAvoidSwitch": False, "uwbSwitch": False})))

    def _on_req(self, req) -> None:
        api = req.header.identity.api_id
        code = 0
        if api == 1008:
            p = json.loads(req.parameter)
            self._target = (float(p["x"]), float(p["y"]), float(p["z"]))
            self._target_t = time.monotonic()
            self.counts["move"] += 1
        elif api == 1003:
            self._target = (0.0, 0.0, 0.0)
            self.counts["stop"] += 1
        else:
            self.counts["bad_api"] += 1
            code = -1
            self.get_logger().error(f"rehearsal_go2: unexpected api_id {api}")
        r = self._Response()
        r.header.identity.id = req.header.identity.id
        r.header.identity.api_id = api
        r.header.status.code = code
        self._pub_resp.publish(r)

    def _on_walk(self, msg: PoseStamped) -> None:
        if msg.header.frame_id != "odom":
            self.get_logger().error("walk_to must be in the odom frame")
            return
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._walk = (msg.pose.position.x, msg.pose.position.y, yaw)
        self._v = [0.0, 0.0, 0.0]
        self.counts["walks"] += 1

    def _step(self) -> None:
        now = time.monotonic()
        dt, self._t = now - self._t, now
        if self._walk is not None:
            self._walk_step(dt)
        else:
            tgt = self._target if now - self._target_t <= self._move_timeout else (0.0, 0.0, 0.0)
            a = 1.0 - math.exp(-dt / self._tau)
            for i in range(3):
                self._v[i] += a * (tgt[i] - self._v[i])
                if abs(self._v[i]) < 1e-4 and tgt[i] == 0.0:
                    self._v[i] = 0.0
            yaw = self._pose[2]
            self._pose[0] += (self._v[0] * math.cos(yaw) - self._v[1] * math.sin(yaw)) * dt
            self._pose[1] += (self._v[0] * math.sin(yaw) + self._v[1] * math.cos(yaw)) * dt
            self._pose[2] = math.atan2(math.sin(yaw + self._v[2] * dt), math.cos(yaw + self._v[2] * dt))
        m = Odometry()
        stamp = time.time() + self._skew
        m.header.stamp.sec = int(stamp)
        m.header.stamp.nanosec = int((stamp % 1) * 1e9)
        m.header.frame_id, m.child_frame_id = "odom", "base_link"
        m.pose.pose.position.x, m.pose.pose.position.y = self._pose[0], self._pose[1]
        m.pose.pose.orientation.z = math.sin(self._pose[2] / 2)
        m.pose.pose.orientation.w = math.cos(self._pose[2] / 2)
        m.twist.twist.linear.x, m.twist.twist.linear.y = self._v[0], self._v[1]
        m.twist.twist.angular.z = self._v[2]
        self._pub_odom.publish(m)

    def _walk_step(self, dt: float) -> None:
        tx, ty, tyaw = self._walk
        dx, dy = tx - self._pose[0], ty - self._pose[1]
        d = math.hypot(dx, dy)
        if d > 0.005:
            step = min(d, self._walk_v * dt)
            self._pose[0] += dx / d * step
            self._pose[1] += dy / d * step
            return
        e = math.atan2(math.sin(tyaw - self._pose[2]), math.cos(tyaw - self._pose[2]))
        if abs(e) > 0.005:
            self._pose[2] += max(-self._walk_w * dt, min(self._walk_w * dt, e))
            return
        self._walk = None
        self._target = (0.0, 0.0, 0.0)

    def _state_tick(self) -> None:
        self._pub_state.publish(String(data=json.dumps(dict(
            self.counts, walking=self._walk is not None, pose_odom=self._pose,
            label="REHEARSAL"))))


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = RehearsalGo2()
    except ImportError as exc:
        print(f"[rehearsal_go2] needs unitree_api (source the unitree_ros2 workspace): {exc}",
              file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(2)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
