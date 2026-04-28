"""Test conversions.py without depending on rclpy/riskgraph_msgs.

We use lightweight mock objects with the same attribute shape as the ROS msgs.
"""
from types import SimpleNamespace

from riskgraph_memory.conversions import core_event_from_msg, msg_route_to_core
from riskgraph_core.events import FactorCategory


def _mk_factor(category="SLIP", severity=0.5, source="x", detail=""):
    return SimpleNamespace(category=category, severity=severity,
                           source=source, detail=detail)


def _mk_event(eid="e1", x=1.0, y=2.0, z=0.0, frame="map",
              factors=None, confidence=0.9, sec=12, nsec=345_000_000):
    factors = factors or [_mk_factor()]
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id=frame,
            stamp=SimpleNamespace(sec=sec, nanosec=nsec),
        ),
        event_id=eid,
        position=SimpleNamespace(x=x, y=y, z=z),
        factors=factors,
        confidence=confidence,
    )


def test_core_event_from_msg_round_trips_basic_fields():
    msg = _mk_event(eid="abc", x=4.0, y=5.0, factors=[
        _mk_factor("SAFETY", 0.8, "go2/safety_alert", "DROP"),
    ], confidence=0.7)
    ev = core_event_from_msg(msg)
    assert ev.event_id == "abc"
    assert ev.position == (4.0, 5.0, 0.0)
    assert ev.factors[0].category == FactorCategory.SAFETY
    assert ev.factors[0].source == "go2/safety_alert"
    assert ev.confidence == 0.7
    assert abs(ev.timestamp - (12 + 0.345)) < 1e-6


def test_core_event_handles_unknown_category_gracefully():
    msg = _mk_event(factors=[_mk_factor("WHAT", 0.4, "x")])
    ev = core_event_from_msg(msg)
    assert ev.factors[0].category == FactorCategory.OTHER


def test_msg_route_to_core_preserves_segment_order():
    s1 = SimpleNamespace(segment_id="a",
                         start=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                         end=SimpleNamespace(x=1.0, y=0.0, z=0.0),
                         semantic_label="hallway")
    s2 = SimpleNamespace(segment_id="b",
                         start=SimpleNamespace(x=1.0, y=0.0, z=0.0),
                         end=SimpleNamespace(x=2.0, y=0.0, z=0.0),
                         semantic_label="")
    route_msg = SimpleNamespace(route_id="R1", segments=[s1, s2])
    route = msg_route_to_core(route_msg)
    assert route.route_id == "R1"
    assert [s.segment_id for s in route.segments] == ["a", "b"]
    assert route.segments[0].semantic_label == "hallway"
    assert route.segments[1].semantic_label is None
