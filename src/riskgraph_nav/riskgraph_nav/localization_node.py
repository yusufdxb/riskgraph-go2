"""ROS 2 node: anchored-odometry localization for a stock GO2.

Publishes the two transforms a stock GO2 does not:

* ``odom -> base_link`` on ``/tf``, from ``/utlidar/robot_odom``, stamped with
  THIS host's clock at receipt. The robot's own stamps were measured months
  off the payload clock; TF stamped with them would be rejected by every
  consumer (Nav2 would see "extrapolation into the past" forever).
* ``map -> odom`` on ``/tf_static``, solved once so the robot's map pose
  equals the start marker's pose (from the experiment file) at the moment of
  anchoring. The robot must be standing still on the marker.

Localization quality is reported on ``/riskgraph/localization/status``
(std_msgs/String JSON, transient local): state (OK / NO_DATA / STALE / JUMP /
UNANCHORED), odometry rate and age on the local clock, jump and gap counts,
robot clock skew, the anchor record, the robot's current map pose and speed,
and ``localization_valid``. An odometry jump latches ``JUMP`` until the
operator re-anchors (``/riskgraph/localization/anchor``, std_srvs/Trigger).

This node publishes no velocity and no robot command.
"""
from __future__ import annotations

import json
import os
import sys
import time
from typing import Optional

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger

from riskgraph_core.experiment import ExperimentError, load_experiment
from riskgraph_core.geometry import Pose2D, quat_from_yaw
from riskgraph_core.map_identity import compute_map_id

from .localization_core import (
    JUMP,
    OK,
    UNANCHORED,
    AnchorRecord,
    OdomMonitor,
    StationaryDetector,
    make_anchor,
)


def _stamp_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class LocalizationNode(Node):
    def __init__(self) -> None:
        super().__init__("riskgraph_localization")
        p = self.declare_parameter
        p("experiment_file", "")
        p("odom_topic", "/utlidar/robot_odom")
        p("odom_frame", "odom")
        p("base_frame", "base_link")
        p("map_frame", "map")
        p("anchor_mode", "auto")          # auto: anchor the first time the robot is still
        p("stationary_window_s", 1.5)
        p("stationary_max_disp_m", 0.02)
        p("max_jump_m", 0.25)
        p("stale_timeout_s", 0.25)
        p("tf_rate_hz", 50.0)
        p("status_rate_hz", 5.0)
        p("anchor_dir", os.path.expanduser("~/.local/share/riskgraph/anchors"))

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        exp_file = g("experiment_file")
        try:
            self.exp = load_experiment(exp_file)
        except ExperimentError as exc:
            raise SystemExit(f"[riskgraph_localization] FATAL: {exc}")
        self.map_id = compute_map_id(self.exp.map_yaml, self.exp.anchor)
        self.odom_frame, self.base_frame, self.map_frame = (
            g("odom_frame"), g("base_frame"), g("map_frame"))
        self.anchor_mode = g("anchor_mode")
        if self.anchor_mode not in ("auto", "service"):
            raise SystemExit("[riskgraph_localization] FATAL: anchor_mode must be auto|service")
        self.monitor = OdomMonitor(self.odom_frame, self.base_frame,
                                   max_jump_m=float(g("max_jump_m")),
                                   stale_timeout_s=float(g("stale_timeout_s")))
        self.still = StationaryDetector(window_s=float(g("stationary_window_s")),
                                        max_disp_m=float(g("stationary_max_disp_m")))
        self.anchor: Optional[AnchorRecord] = None
        self.anchor_epoch = 0
        self.anchor_dir = g("anchor_dir")
        self._tf_period = 1.0 / max(1.0, float(g("tf_rate_hz")))
        self._last_tf_t = 0.0
        self.tf_published = 0

        import tf2_ros
        self._tf = tf2_ros.TransformBroadcaster(self)
        self._static = tf2_ros.StaticTransformBroadcaster(self)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(String, "/riskgraph/localization/status", latched)
        # BEST_EFFORT is compatible with both a reliable and a best-effort
        # publisher; only the newest sample matters.
        odom_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Odometry, g("odom_topic"), self._on_odom, odom_qos)
        self.create_service(Trigger, "/riskgraph/localization/anchor", self._on_anchor_srv)
        self.create_timer(1.0 / max(0.5, float(g("status_rate_hz"))), self._publish_status)
        self.get_logger().info(
            f"riskgraph_localization up: {g('odom_topic')} -> TF {self.odom_frame}->"
            f"{self.base_frame} (restamped on this host's clock); map anchored on marker "
            f"{self.exp.start_marker} at ({self.exp.start.x}, {self.exp.start.y}, "
            f"{self.exp.start.yaw}); anchor_mode={self.anchor_mode}; map_id={self.map_id}")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # -- odometry -------------------------------------------------------------

    def _on_odom(self, msg: Odometry) -> None:
        now = self._now()
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        res = self.monitor.update(now, p.x, p.y, p.z, (o.x, o.y, o.z, o.w),
                                  msg.header.frame_id, msg.child_frame_id,
                                  _stamp_s(msg.header.stamp))
        if res in ("FRAME", "INVALID"):
            if (self.monitor.invalid + self.monitor.frame_mismatch) % 150 == 1:
                self.get_logger().error(
                    f"rejected odometry sample ({res}): frame={msg.header.frame_id!r} "
                    f"child={msg.child_frame_id!r}")
            return
        if res == "JUMP":
            self.get_logger().error(
                f"odometry JUMP {self.monitor.last_jump}: localization INVALID until re-anchored "
                f"(ros2 service call /riskgraph/localization/anchor std_srvs/srv/Trigger)")
        pose = self.monitor.last.pose
        self.still.update(now, pose)
        if now - self._last_tf_t >= self._tf_period:
            self._publish_odom_tf(msg, now)
        if self.anchor is None and self.anchor_mode == "auto" and self.still.is_stationary(now):
            self._do_anchor(now)

    def _publish_odom_tf(self, msg: Odometry, now: float) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        p = msg.pose.pose.position
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
        t.transform.rotation = msg.pose.pose.orientation
        self._tf.sendTransform(t)
        self._last_tf_t = now
        self.tf_published += 1

    # -- anchoring -------------------------------------------------------------

    def _do_anchor(self, now: float) -> AnchorRecord:
        self.anchor_epoch += 1
        rec = make_anchor(self.anchor_epoch, self.exp.start_marker, self.exp.start,
                          self.monitor.last.pose, now)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = rec.map_to_odom.x
        t.transform.translation.y = rec.map_to_odom.y
        q = quat_from_yaw(rec.map_to_odom.yaw)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self._static.sendTransform(t)
        self.anchor = rec
        self.monitor.clear_jump()
        self._write_anchor_record(rec)
        self.get_logger().info(
            f"ANCHORED epoch {rec.epoch} on marker {rec.marker}: robot odom pose "
            f"({rec.base_in_odom.x:.3f}, {rec.base_in_odom.y:.3f}, {rec.base_in_odom.yaw:.3f}) "
            f"-> map ({rec.marker_in_map.x}, {rec.marker_in_map.y}, {rec.marker_in_map.yaw}); "
            f"T_map_odom=({rec.map_to_odom.x:.3f}, {rec.map_to_odom.y:.3f}, {rec.map_to_odom.yaw:.3f})")
        self._publish_status()
        return rec

    def _write_anchor_record(self, rec: AnchorRecord) -> None:
        try:
            os.makedirs(self.anchor_dir, exist_ok=True)
            path = os.path.join(self.anchor_dir, f"{self.map_id}_{int(rec.t)}_e{rec.epoch}.json")
            with open(path, "w") as fh:
                json.dump(dict(rec.as_dict(), map_id=self.map_id, host=os.uname().nodename), fh, indent=1)
        except OSError as exc:
            self.get_logger().warn(f"could not write anchor record: {exc}")

    def _on_anchor_srv(self, _req, resp):
        now = self._now()
        if self.monitor.last is None or self.monitor.state(now) not in (OK, JUMP):
            resp.success = False
            resp.message = f"odometry not usable: {self.monitor.state(now)}"
            return resp
        if not self.still.is_stationary(now):
            resp.success = False
            resp.message = (f"robot is not stationary (need {self.still.window_s}s still); "
                            f"release the sticks and wait")
            return resp
        rec = self._do_anchor(now)
        resp.success = True
        resp.message = json.dumps(rec.as_dict())
        return resp

    # -- status --------------------------------------------------------------------

    def status_dict(self) -> dict:
        now = self._now()
        st = self.monitor.state(now)
        state = st if st != OK else (OK if self.anchor is not None else UNANCHORED)
        robot_map = None
        if self.anchor is not None and self.monitor.last is not None:
            rp = self.anchor.map_to_odom.compose(self.monitor.last.pose)
            robot_map = {"x": rp.x, "y": rp.y, "yaw": rp.yaw}
        last = self.monitor.last
        return {
            "stamp": time.time(),
            "ros_time": now,
            "state": state,
            "localization_valid": state == OK,
            "anchored": self.anchor is not None,
            "anchor": self.anchor.as_dict() if self.anchor else None,
            "anchor_mode": self.anchor_mode,
            "map_id": self.map_id,
            "map_frame": self.map_frame,
            "odom_rate_hz": self.monitor.rate_hz(now),
            "odom_age_s": self.monitor.age_s(now),
            "odom_accepted": self.monitor.accepted,
            "odom_invalid": self.monitor.invalid,
            "odom_frame_mismatch": self.monitor.frame_mismatch,
            "odom_jumps": self.monitor.jumps,
            "odom_gaps": self.monitor.gaps,
            "last_jump": self.monitor.last_jump,
            "robot_clock_skew_s": self.monitor.robot_clock_skew_s,
            "robot_pose_odom": ({"x": last.pose.x, "y": last.pose.y, "yaw": last.pose.yaw}
                                if last else None),
            "robot_pose_map": robot_map,
            "speed_mps": self.still.speed_mps(),
            "stationary": self.still.is_stationary(now),
            "tf_published": self.tf_published,
            "pid": os.getpid(),
        }

    def _publish_status(self) -> None:
        self._status_pub.publish(String(data=json.dumps(self.status_dict(), default=str)))


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = LocalizationNode()
    except SystemExit as exc:
        print(str(exc), file=sys.stderr, flush=True)
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
