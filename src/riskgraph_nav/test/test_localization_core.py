"""Anchored-odometry localization logic: stream health, jumps, anchoring."""
import math

import pytest

from riskgraph_core.geometry import Pose2D, quat_from_yaw
from riskgraph_nav.localization_core import (
    JUMP, NO_DATA, OK, STALE, OdomMonitor, StationaryDetector, make_anchor)

Q0 = quat_from_yaw(0.0)


def feed(m, t0, n, dt=1 / 150, x0=0.0, vx=0.0, frame="odom", child="base_link"):
    for i in range(n):
        m.update(t0 + i * dt, x0 + vx * i * dt, 0.0, 0.0, Q0, frame, child, robot_stamp=1.0)


def test_no_data_then_ok_then_stale():
    m = OdomMonitor()
    assert m.state(0.0) == NO_DATA
    feed(m, 100.0, 150)
    assert m.state(101.0) == OK
    assert m.rate_hz(101.0) == pytest.approx(150, rel=0.05)
    assert m.state(101.5) == STALE


def test_robot_clock_skew_is_measured_not_used():
    m = OdomMonitor()
    m.update(1_789_000_000.0, 0, 0, 0, Q0, "odom", "base_link", robot_stamp=1_760_700_000.0)
    assert m.robot_clock_skew_s == pytest.approx(28_300_000.0)
    assert m.state(1_789_000_000.1) == OK


def test_wrong_frames_and_nonfinite_are_rejected():
    m = OdomMonitor()
    assert m.update(1.0, 0, 0, 0, Q0, "map", "base_link") == "FRAME"
    assert m.update(1.0, math.nan, 0, 0, Q0, "odom", "base_link") == "INVALID"
    assert m.update(1.0, 0, 0, 0, (0, 0, 0, 0), "odom", "base_link") == "INVALID"
    assert m.last is None and m.invalid == 2 and m.frame_mismatch == 1


def test_jump_latches_until_cleared():
    m = OdomMonitor(max_jump_m=0.25)
    feed(m, 0.0, 10)
    assert m.update(0.1, 1.0, 0, 0, Q0, "odom", "base_link") == "JUMP"
    feed(m, 0.11, 10, x0=1.0)
    assert m.state(0.2) == JUMP
    m.clear_jump()
    assert m.state(0.2) == OK


def test_yaw_jump_latches():
    m = OdomMonitor()
    feed(m, 0.0, 3)
    assert m.update(0.03, 0, 0, 0, quat_from_yaw(1.2), "odom", "base_link") == "JUMP"


def test_normal_walking_is_not_a_jump():
    m = OdomMonitor()
    feed(m, 0.0, 300, vx=0.25)
    assert m.jumps == 0


def test_stationary_detector():
    s = StationaryDetector(window_s=1.0, max_disp_m=0.02)
    for i in range(200):
        s.update(i * 0.01, Pose2D(0.001 * (i % 2), 0.0, 0.0))
    assert s.is_stationary(2.0)
    s2 = StationaryDetector(window_s=1.0)
    for i in range(200):
        s2.update(i * 0.01, Pose2D(0.2 * i * 0.01, 0.0, 0.0))
    assert not s2.is_stationary(2.0)
    assert s2.speed_mps() == pytest.approx(0.2, rel=0.05)
    s3 = StationaryDetector(window_s=1.0)
    s3.update(0.0, Pose2D(0, 0, 0))
    assert not s3.is_stationary(0.0)  # not enough history


def test_anchor_record_puts_robot_on_marker():
    rec = make_anchor(1, "A", Pose2D(0, 0, 0), Pose2D(3.2, -1.7, 0.6), 10.0)
    on = rec.map_to_odom.compose(Pose2D(3.2, -1.7, 0.6))
    assert (on.x, on.y, on.yaw) == pytest.approx((0, 0, 0), abs=1e-9)
    d = rec.as_dict()
    assert d["marker"] == "A" and d["epoch"] == 1
