"""Frame math: rotations, composition, anchoring, polyline helpers."""
from __future__ import annotations

import math

import pytest

from riskgraph_core.geometry import (
    Pose2D,
    anchor_map_to_odom,
    apply_transform,
    normalize_angle,
    point_to_polyline_distance,
    polyline_length,
    polyline_separation,
    quat_from_yaw,
    quat_is_valid,
    resample_polyline,
    rotate,
    yaw_from_quat,
)


@pytest.mark.parametrize("a,expected", [(0, 0), (math.pi, math.pi), (-math.pi, math.pi),
                                        (3 * math.pi, math.pi), (2 * math.pi + 0.1, 0.1),
                                        (-0.1, -0.1)])
def test_normalize_angle(a, expected):
    assert normalize_angle(a) == pytest.approx(expected)


@pytest.mark.parametrize("yaw", [0.0, 0.3, -1.2, math.pi / 2, 3.0])
def test_quat_yaw_round_trip(yaw):
    q = quat_from_yaw(yaw)
    assert quat_is_valid(q)
    assert yaw_from_quat(q) == pytest.approx(normalize_angle(yaw))


def test_quat_validity_rejects_nan_and_unnormalized():
    assert not quat_is_valid((0, 0, 0, 0))
    assert not quat_is_valid((math.nan, 0, 0, 1))


def test_rotate_90_deg_about_z():
    q = quat_from_yaw(math.pi / 2)
    assert rotate(q, (1.0, 0.0, 0.0)) == pytest.approx((0.0, 1.0, 0.0))
    assert rotate(q, (0.0, 1.0, 5.0)) == pytest.approx((-1.0, 0.0, 5.0))


def test_apply_transform_translation_and_rotation():
    # odom frame rotated +90 deg in map and shifted by (2, 3): a point 1 m ahead
    # along odom x lands 1 m along map y from (2, 3).
    p = apply_transform((2.0, 3.0, 0.0), quat_from_yaw(math.pi / 2), (1.0, 0.0, 0.0))
    assert p == pytest.approx((2.0, 4.0, 0.0))


def test_pose_compose_and_inverse_are_consistent():
    a = Pose2D(1.0, -2.0, 0.7)
    b = Pose2D(-0.4, 3.0, -2.1)
    ab = a.compose(b)
    back = a.inverse().compose(ab)
    assert back.x == pytest.approx(b.x)
    assert back.y == pytest.approx(b.y)
    assert back.yaw == pytest.approx(b.yaw)
    ident = a.compose(a.inverse())
    assert (ident.x, ident.y, ident.yaw) == pytest.approx((0, 0, 0), abs=1e-12)


def test_anchor_places_robot_on_marker():
    """The whole localization contract in one assertion: after anchoring,
    T_map_odom * T_odom_base == marker pose."""
    marker = Pose2D(0.0, 0.0, 0.0)
    base_in_odom = Pose2D(12.3, -4.5, 2.2)  # odom origin = wherever the robot booted
    m_o = anchor_map_to_odom(marker, base_in_odom)
    base_in_map = m_o.compose(base_in_odom)
    assert (base_in_map.x, base_in_map.y) == pytest.approx((0.0, 0.0), abs=1e-9)
    assert base_in_map.yaw == pytest.approx(0.0, abs=1e-9)
    # The robot then walks 1 m forward in its own frame: map pose follows.
    moved = base_in_odom.compose(Pose2D(1.0, 0.0, 0.0))
    assert (m_o.compose(moved).x, m_o.compose(moved).y) == pytest.approx((1.0, 0.0), abs=1e-9)


def test_anchor_with_nonzero_marker_pose():
    marker = Pose2D(2.0, 1.0, math.pi / 2)
    base_in_odom = Pose2D(-3.0, 0.5, -0.4)
    m_o = anchor_map_to_odom(marker, base_in_odom)
    ahead = base_in_odom.compose(Pose2D(1.0, 0.0, 0.0))
    p = m_o.compose(ahead)
    assert (p.x, p.y) == pytest.approx((2.0, 2.0), abs=1e-9)


def test_transform_point_matches_compose():
    t = Pose2D(1.0, 2.0, 0.5)
    x, y = t.transform_point(0.3, -0.7)
    c = t.compose(Pose2D(0.3, -0.7, 0.0))
    assert (x, y) == pytest.approx((c.x, c.y))


def test_polyline_helpers():
    pts = [(0, 0), (3, 0), (3, 4)]
    assert polyline_length(pts) == pytest.approx(7.0)
    rs = resample_polyline(pts, 0.5)
    assert rs[0] == (0.0, 0.0) and rs[-1] == (3.0, 4.0)
    gaps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(rs, rs[1:])]
    assert max(gaps) <= 0.5 + 1e-9
    assert point_to_polyline_distance((1.5, 1.0), pts) == pytest.approx(1.0)
    assert point_to_polyline_distance((0, 0), []) == math.inf


def test_resample_handles_zero_length_segments():
    rs = resample_polyline([(0, 0), (0, 0), (1, 0)], 0.25)
    assert rs[-1] == (1.0, 0.0) and len(rs) == 5


def test_polyline_separation_parallel_lines():
    a = [(0, 0), (4, 0)]
    b = [(0, 1), (4, 1)]
    mean, haus = polyline_separation(a, b)
    assert mean == pytest.approx(1.0)
    assert haus == pytest.approx(1.0)
    assert polyline_separation(a, a)[0] == pytest.approx(0.0)
