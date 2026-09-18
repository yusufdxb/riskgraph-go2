#!/usr/bin/env python3
"""Stationary fake GO2 odometry for integration tests: /utlidar/robot_odom at
150 Hz, odom -> base_link, origin deliberately NOT the start marker, header
stamps skewed like the lab robot's clock. NOT a robot model."""
import math
import sys
import time

import rclpy
import rclpy.executors
from nav_msgs.msg import Odometry

x0, y0, yaw0 = (float(v) for v in (sys.argv[1:4] if len(sys.argv) >= 4 else (3.2, -1.7, 0.6)))
skew = -28_300_000.0
rclpy.init()
n = rclpy.create_node("fake_odom")
pub = n.create_publisher(Odometry, "/utlidar/robot_odom", 10)


def tick():
    m = Odometry()
    s = time.time() + skew
    m.header.stamp.sec, m.header.stamp.nanosec = int(s), int((s % 1) * 1e9)
    m.header.frame_id, m.child_frame_id = "odom", "base_link"
    m.pose.pose.position.x, m.pose.pose.position.y = x0, y0
    m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw0 / 2), math.cos(yaw0 / 2)
    pub.publish(m)


n.create_timer(1.0 / 150.0, tick)
try:
    rclpy.spin(n)
except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    pass
