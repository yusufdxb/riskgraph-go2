"""Tests for memory_node segment-seed wiring (v0.1.1 phase-1 gap closure).

`memory_node.RiskMemoryNode.__init__` reads a `segment_seed_path` ROS param
and, if non-empty, loads it via `riskgraph_core.seed.load_segment_seed` and
populates `_known_segments`. These tests stub rclpy and friends (same
pattern as `test_adapters.py`) and assert that:

  * empty `segment_seed_path` leaves `_known_segments` empty
  * pointing at a valid seed file populates `_known_segments` AND drives the
    spatial-join fallback in `_on_event`
  * a broken seed file does not crash __init__ (loud WARN, empty list)
"""
from __future__ import annotations

import json
import sys
import types
from contextlib import contextmanager
from types import ModuleType, SimpleNamespace
from typing import List

import pytest

# Reuse the stub machinery from test_adapters.py — the FakeNode there knows
# how to surface declare_parameter / get_parameter etc. Importing the
# helpers directly keeps the wiring identical between adapter tests and
# memory-node tests, and avoids duplicating ~200 lines of stub code.
from test_adapters import (  # type: ignore[import-not-found]
    _FakeNode,
    _make_rclpy_stub,
    _make_geometry_msgs_stub,
    _make_std_msgs_stub,
)


# ---------------------------------------------------------------------------
# Local stubs for the memory-node-specific imports the adapter file does NOT
# touch: riskgraph_msgs.srv.QuerySegmentRisk and a few RiskEvent shape bits.
# ---------------------------------------------------------------------------

def _make_riskgraph_msgs_for_memory() -> ModuleType:
    pkg = types.ModuleType("riskgraph_msgs")
    msg = types.ModuleType("riskgraph_msgs.msg")

    class _RiskFactor:
        def __init__(self):
            self.category = ""
            self.severity = 0.0
            self.source = ""
            self.detail = ""

    class _RiskEvent:
        def __init__(self):
            self.header = SimpleNamespace(
                frame_id="map",
                stamp=SimpleNamespace(sec=0, nanosec=0),
            )
            self.event_id = ""
            self.position = SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.segment_id = ""
            self.factors: List[_RiskFactor] = []
            self.confidence = 1.0
    msg.RiskEvent = _RiskEvent
    msg.RiskFactor = _RiskFactor
    pkg.msg = msg

    srv = types.ModuleType("riskgraph_msgs.srv")

    class _QuerySegmentRisk:
        class Request:
            def __init__(self):
                self.segment_ids: List[str] = []
                self.decay_half_life_s = 0.0

        class Response:
            def __init__(self):
                self.risks: List[float] = []
                self.event_counts: List[int] = []
                self.dominant_factor_categories: List[str] = []
    srv.QuerySegmentRisk = _QuerySegmentRisk
    pkg.srv = srv
    return pkg


@contextmanager
def _memory_node_imports():
    saved = {
        name: sys.modules.get(name)
        for name in (
            "rclpy", "rclpy.node", "rclpy.qos",
            "geometry_msgs", "geometry_msgs.msg",
            "std_msgs", "std_msgs.msg",
            "riskgraph_msgs", "riskgraph_msgs.msg", "riskgraph_msgs.srv",
            "riskgraph_memory.memory_node",
        )
    }
    rclpy = _make_rclpy_stub()
    geom = _make_geometry_msgs_stub()
    stdm = _make_std_msgs_stub()
    rgm = _make_riskgraph_msgs_for_memory()
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy.node
    sys.modules["rclpy.qos"] = rclpy.qos
    sys.modules["geometry_msgs"] = geom
    sys.modules["geometry_msgs.msg"] = geom.msg
    sys.modules["std_msgs"] = stdm
    sys.modules["std_msgs.msg"] = stdm.msg
    sys.modules["riskgraph_msgs"] = rgm
    sys.modules["riskgraph_msgs.msg"] = rgm.msg
    sys.modules["riskgraph_msgs.srv"] = rgm.srv

    # Force a fresh import.
    sys.modules.pop("riskgraph_memory.memory_node", None)
    parent = sys.modules.get("riskgraph_memory")
    if parent is not None and hasattr(parent, "memory_node"):
        delattr(parent, "memory_node")
    try:
        yield
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v


# Patch FakeNode so `_FakeNode().get_parameter(name)` honors values we set
# explicitly in tests. The existing _FakeNode looks up declared defaults;
# we need to override them post-construction to model launch param overrides.

def _make_node_class(overrides: dict):
    """Subclass _FakeNode with a hook to inject param values + create_service.

    `overrides` is captured by closure so it survives the call to
    `super().__init__()` inside `RiskMemoryNode.__init__`, which would
    otherwise reset any instance-level dict we set up before invoking
    the real init.
    """
    class _SeedNode(_FakeNode):
        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            self._services: list = []

        def get_parameter(self, name):
            if name in overrides:
                v = overrides[name]
            else:
                v = self._params.get(name, "")

            class _PV:
                def __init__(self, val):
                    self.string_value = str(val) if not isinstance(val, bool) else ""
                    try:
                        self.double_value = float(val)
                    except (TypeError, ValueError):
                        self.double_value = 0.0

            class _P:
                def __init__(self, val):
                    self._val = val

                def get_parameter_value(self):
                    return _PV(self._val)
            return _P(v)

        def create_service(self, _srv_cls, _topic, callback):
            # memory_node calls this for /riskgraph/query_segment_risk.
            # We just record the (topic, callback) pair so the body of
            # __init__ doesn't blow up — no test currently exercises the
            # service from this file (covered elsewhere).
            self._services.append((_topic, callback))
            return SimpleNamespace(topic=_topic, callback=callback)
    return _SeedNode


def _patch_node_base(overrides: dict):
    """Swap rclpy.node.Node for our parameter-injectable subclass."""
    SeedNode = _make_node_class(overrides)
    sys.modules["rclpy.node"].Node = SeedNode
    sys.modules["rclpy"].node.Node = SeedNode  # mirror on the parent pkg too


def _write_seed(tmp_path, segments):
    p = tmp_path / "seed.json"
    p.write_text(json.dumps({"frame_id": "map", "segments": segments}))
    return str(p)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def _build_node(overrides: dict):
    """Common test helper: stub rclpy, patch Node base, build RiskMemoryNode."""
    _patch_node_base(overrides)
    from riskgraph_memory import memory_node as mn
    node = mn.RiskMemoryNode()
    return node


def test_empty_seed_path_leaves_known_segments_empty():
    with _memory_node_imports():
        node = _build_node({
            "store_path": ":memory:",
            "decay_half_life_s": 0.0,
            "segment_seed_path": "",
        })
        assert node.known_segments == []
        assert len(node.segment_seed) == 0


def test_seed_path_populates_known_segments(tmp_path):
    seed_path = _write_seed(tmp_path, [
        {"segment_id": "A", "start": [0.0, 0.0, 0.0], "end": [10.0, 0.0, 0.0],
         "semantic_label": "hallway-east"},
        {"segment_id": "B", "start": [0.0, 5.0, 0.0], "end": [10.0, 5.0, 0.0],
         "semantic_label": "hallway-north"},
    ])
    with _memory_node_imports():
        node = _build_node({
            "store_path": ":memory:",
            "decay_half_life_s": 0.0,
            "segment_seed_path": seed_path,
        })
        ids = [s.segment_id for s in node.known_segments]
        assert ids == ["A", "B"]
        assert node.segment_seed.source_path == seed_path
        assert node.segment_seed.frame_id == "map"


def test_seed_drives_spatial_join_on_event(tmp_path):
    """End-to-end: event arrives with no segment_id; the seeded segments
    let the memory node spatially-join it to 'A' before persisting."""
    seed_path = _write_seed(tmp_path, [
        {"segment_id": "A", "start": [0.0, 0.0, 0.0], "end": [10.0, 0.0, 0.0]},
        {"segment_id": "B", "start": [0.0, 5.0, 0.0], "end": [10.0, 5.0, 0.0]},
    ])
    with _memory_node_imports():
        node = _build_node({
            "store_path": ":memory:",
            "decay_half_life_s": 0.0,
            "segment_seed_path": seed_path,
        })

        # Build a ROS-shaped RiskEvent msg that lands near segment A and has
        # no segment_id set.
        from riskgraph_msgs.msg import RiskEvent as RiskEventMsg, RiskFactor
        msg = RiskEventMsg()
        msg.event_id = "ev_test"
        msg.position = SimpleNamespace(x=5.0, y=0.1, z=0.0)
        msg.confidence = 1.0
        f = RiskFactor()
        f.category = "SLIP"
        f.severity = 0.9
        f.source = "tactile/slip_state"
        f.detail = "test"
        msg.factors = [f]
        msg.header = SimpleNamespace(
            frame_id="map",
            stamp=SimpleNamespace(sec=1, nanosec=0),
        )
        msg.segment_id = ""

        node._on_event(msg)

        # Spatial join should have stamped segment_id = "A" before persist;
        # query the store to verify.
        events_a = node.store.events_for_segment("A")
        events_b = node.store.events_for_segment("B")
        assert len(events_a) == 1
        assert len(events_b) == 0
        assert events_a[0].event_id == "ev_test"
        assert events_a[0].segment_id == "A"


def test_broken_seed_path_logs_but_does_not_crash(tmp_path):
    bad = tmp_path / "bad.json"
    bad.write_text("{not valid json")
    with _memory_node_imports():
        node = _build_node({
            "store_path": ":memory:",
            "decay_half_life_s": 0.0,
            "segment_seed_path": str(bad),
        })
        assert node.known_segments == []
        assert len(node.segment_seed) == 0


def test_nonexistent_seed_path_logs_but_does_not_crash(tmp_path):
    with _memory_node_imports():
        node = _build_node({
            "store_path": ":memory:",
            "decay_half_life_s": 0.0,
            "segment_seed_path": str(tmp_path / "missing.json"),
        })
        assert node.known_segments == []
