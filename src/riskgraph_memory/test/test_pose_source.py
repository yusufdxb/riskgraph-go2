"""Tests for the bounded-age odometry cache and the unavailable-pose policy.

`pose_source` is deliberately ROS-free, so these tests need no stubs: they
exercise the policy directly. The behaviors that matter are (a) a pose is
never handed out once it is older than the bound, (b) a malformed odometry
sample cannot poison the cache, and (c) "no pose" is reported as None rather
than as the origin.
"""
from __future__ import annotations

import math

import pytest

from riskgraph_memory.pose_source import (
    DEFAULT_MAX_AGE_S,
    UNKNOWN_FRAME,
    OdometryCache,
    Pose,
)


class TestConstruction:

    def test_default_max_age_matches_module_default(self):
        assert OdometryCache().max_age_s == DEFAULT_MAX_AGE_S

    def test_zero_max_age_rejected(self):
        with pytest.raises(ValueError, match="max_age_s"):
            OdometryCache(max_age_s=0.0)

    def test_negative_max_age_rejected(self):
        with pytest.raises(ValueError, match="max_age_s"):
            OdometryCache(max_age_s=-1.0)

    def test_empty_cache_has_no_pose(self):
        c = OdometryCache()
        assert c.last_pose is None
        assert c.pose_at(100.0) is None
        assert c.is_fresh(100.0) is False

    def test_unknown_frame_is_the_empty_string(self):
        # The memory node keys its refusal off this exact value.
        assert UNKNOWN_FRAME == ""


class TestUpdate:

    def test_accepts_a_well_formed_sample(self):
        c = OdometryCache()
        assert c.update(1.0, 2.0, 3.0, "odom", 100.0) is True
        assert c.last_pose == Pose(position=(1.0, 2.0, 3.0),
                                   frame_id="odom", stamp_s=100.0)
        assert c.accepted_count == 1
        assert c.rejected_count == 0

    def test_latest_sample_wins(self):
        c = OdometryCache()
        c.update(1.0, 0.0, 0.0, "odom", 100.0)
        c.update(2.0, 0.0, 0.0, "odom", 100.1)
        assert c.last_pose.position == (2.0, 0.0, 0.0)
        assert c.accepted_count == 2

    def test_blank_frame_rejected(self):
        c = OdometryCache()
        assert c.update(1.0, 2.0, 3.0, "", 100.0) is False
        assert c.last_pose is None
        assert c.rejected_count == 1

    @pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
    def test_non_finite_coordinate_rejected(self, bad):
        c = OdometryCache()
        assert c.update(bad, 0.0, 0.0, "odom", 100.0) is False
        assert c.last_pose is None
        assert c.rejected_count == 1

    def test_non_finite_stamp_rejected(self):
        c = OdometryCache()
        assert c.update(0.0, 0.0, 0.0, "odom", float("nan")) is False
        assert c.last_pose is None

    @pytest.mark.parametrize("stamp", [0.0, -1.0])
    def test_non_positive_stamp_rejected(self, stamp):
        c = OdometryCache()
        assert c.update(0.0, 0.0, 0.0, "odom", stamp) is False
        assert c.last_pose is None

    def test_non_numeric_coordinate_rejected(self):
        c = OdometryCache()
        assert c.update("west", 0.0, 0.0, "odom", 100.0) is False
        assert c.rejected_count == 1

    def test_bad_sample_does_not_evict_a_good_one(self):
        """A driver emitting garbage must not be able to blank the cache."""
        c = OdometryCache()
        c.update(5.0, 5.0, 0.0, "odom", 100.0)
        c.update(float("nan"), 0.0, 0.0, "odom", 100.1)
        assert c.last_pose.position == (5.0, 5.0, 0.0)
        assert c.accepted_count == 1
        assert c.rejected_count == 1


class TestAgeBound:

    def test_fresh_pose_returned(self):
        c = OdometryCache(max_age_s=0.5)
        c.update(1.0, 0.0, 0.0, "odom", 100.0)
        assert c.pose_at(100.2) is not None
        assert c.is_fresh(100.2)

    def test_pose_exactly_at_the_bound_is_still_fresh(self):
        c = OdometryCache(max_age_s=0.5)
        c.update(1.0, 0.0, 0.0, "odom", 100.0)
        assert c.pose_at(100.5) is not None

    def test_stale_pose_withheld(self):
        c = OdometryCache(max_age_s=0.5)
        c.update(1.0, 0.0, 0.0, "odom", 100.0)
        assert c.pose_at(100.51) is None
        assert c.is_fresh(100.51) is False

    def test_stale_pose_is_withheld_but_retained(self):
        """`last_pose` still shows it: withholding is a freshness judgment,
        not an eviction, so a later in-window event can still use it."""
        c = OdometryCache(max_age_s=0.5)
        c.update(1.0, 0.0, 0.0, "odom", 100.0)
        assert c.pose_at(200.0) is None
        assert c.last_pose is not None
        assert c.pose_at(100.1) is not None

    def test_pose_from_the_future_withheld(self):
        """A pose ahead of the event means the two streams are on different
        clocks; stamping across that gap would fabricate precision."""
        c = OdometryCache(max_age_s=0.5)
        c.update(1.0, 0.0, 0.0, "odom", 200.0)
        assert c.pose_at(100.0) is None

    def test_non_finite_event_time_yields_no_pose(self):
        c = OdometryCache()
        c.update(1.0, 0.0, 0.0, "odom", 100.0)
        assert c.pose_at(float("nan")) is None
        assert c.pose_at(math.inf) is None

    def test_non_numeric_event_time_yields_no_pose(self):
        c = OdometryCache()
        c.update(1.0, 0.0, 0.0, "odom", 100.0)
        assert c.pose_at("now") is None
