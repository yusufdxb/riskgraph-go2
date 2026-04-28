import pytest
from riskgraph_core.segments import RouteSegment, Route, segment_for_point


def _seg(seg_id, x0, y0, x1, y1):
    return RouteSegment(segment_id=seg_id, start=(x0, y0, 0.0), end=(x1, y1, 0.0),
                        semantic_label=None)


def test_segment_length_is_euclidean():
    s = _seg("a", 0.0, 0.0, 3.0, 4.0)
    assert s.length_m == pytest.approx(5.0)


def test_route_total_length_sums_segments():
    r = Route(route_id="r1", segments=[
        _seg("a", 0.0, 0.0, 3.0, 0.0),
        _seg("b", 3.0, 0.0, 3.0, 4.0),
    ])
    assert r.total_length_m == pytest.approx(7.0)


def test_segment_for_point_returns_nearest_segment():
    segments = [
        _seg("a", 0.0, 0.0, 10.0, 0.0),
        _seg("b", 0.0, 5.0, 10.0, 5.0),
    ]
    # Point right next to segment "a"
    nearest = segment_for_point(segments, (5.0, 0.1, 0.0))
    assert nearest.segment_id == "a"
    # Point right next to segment "b"
    nearest = segment_for_point(segments, (5.0, 4.9, 0.0))
    assert nearest.segment_id == "b"


def test_segment_for_point_handles_endpoint_proximity():
    segments = [_seg("a", 0.0, 0.0, 10.0, 0.0)]
    # Point past the end of the segment should still associate with it
    nearest = segment_for_point(segments, (15.0, 0.0, 0.0))
    assert nearest.segment_id == "a"


def test_segment_for_point_returns_none_for_empty():
    assert segment_for_point([], (1.0, 1.0, 0.0)) is None


def test_segment_for_point_handles_degenerate_zero_length_segment():
    # When a segment has start == end, the distance must collapse to dist-to-point
    # rather than dividing by zero.
    segments = [_seg("a", 5.0, 5.0, 5.0, 5.0)]
    nearest = segment_for_point(segments, (5.0, 5.0, 0.0))
    assert nearest.segment_id == "a"
    nearest = segment_for_point(segments, (10.0, 10.0, 0.0))
    assert nearest.segment_id == "a"


def test_segment_for_point_picks_correctly_with_near_collinear_segments():
    # Two parallel segments separated by a small gap — the nearest must be
    # selected unambiguously even when the offset is small.
    segments = [
        _seg("a", 0.0, 0.0, 10.0, 0.0),
        _seg("b", 0.0, 0.05, 10.0, 0.05),
    ]
    nearest = segment_for_point(segments, (5.0, 0.06, 0.0))
    assert nearest.segment_id == "b"
    nearest = segment_for_point(segments, (5.0, -0.01, 0.0))
    assert nearest.segment_id == "a"
