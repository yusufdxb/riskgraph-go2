"""Soft-import adapter tests for SafetyAlert / FaultEvent / SlipState.

The three adapter modules (`safety_adapter`, `helix_adapter`, `tactile_adapter`)
import `rclpy`, `geometry_msgs`, `std_msgs`, and `riskgraph_msgs` at module
import time, plus they declare a soft dependency on upstream packages
(`go2_msgs`, `helix_msgs`) via try/except ImportError → HAVE_*_MSGS bool.

These tests stub the ROS-side modules in `sys.modules` BEFORE importing the
adapter modules so the soft-import code path can be exercised in both modes:

  * Mode A: upstream msgs available  → `HAVE_*_MSGS` is True, callbacks parse
    a representative SimpleNamespace-shaped msg into a RiskEvent emitted via
    a captured publisher.
  * Mode B: upstream msgs missing    → `HAVE_*_MSGS` is False, `main()` is a
    clean no-op (prints to stderr, returns) and never spins rclpy.

The tactile adapter has no soft upstream dep (it consumes std_msgs/Bool only),
so its tests cover normal/edge/missing-field/leading-edge-debounce instead of
import gating.
"""
from __future__ import annotations

import sys
import types
from contextlib import contextmanager
from types import ModuleType, SimpleNamespace
from typing import List

import pytest


# ---------- ROS module stubs ----------------------------------------------------
#
# We install minimal SimpleNamespace-shaped stubs for every ROS module the
# adapter modules touch at import time. These stubs are intentionally inert:
# create_subscription / create_publisher record their calls so the test can
# capture the published RiskEvent without spinning a real node.


class _FakePub:
    def __init__(self) -> None:
        self.published: List[object] = []

    def publish(self, msg) -> None:
        self.published.append(msg)


class _FakeSub:
    def __init__(self, callback) -> None:
        self.callback = callback


class _FakeNode:
    """Stand-in for rclpy.node.Node. Captures publishers/subscribers and
    parameter values, exposes a get_logger() that swallows messages."""

    def __init__(self, *_args, **_kwargs) -> None:
        self._params: dict = {}
        self._pubs: List[_FakePub] = []
        self._subs: List[_FakeSub] = []

    def declare_parameter(self, name: str, default):
        # Real rclpy returns a Parameter; we just stash the default.
        self._params[name] = default
        return SimpleNamespace(value=default)

    def get_parameter(self, name: str):
        v = self._params.get(name, "")

        def _string_value():
            return SimpleNamespace(string_value=str(v) if not isinstance(v, bool) else "")

        def _double_value():
            return SimpleNamespace(double_value=float(v))

        # Return an object whose .get_parameter_value() returns both flavors;
        # the adapter pulls .string_value or .double_value off of it.
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

    def create_publisher(self, _msg_cls, _topic, _qos):
        p = _FakePub()
        self._pubs.append(p)
        return p

    def create_subscription(self, _msg_cls, _topic, callback, _qos):
        s = _FakeSub(callback)
        self._subs.append(s)
        return s

    def get_logger(self):
        class _L:
            def info(self, _m): pass
            def warn(self, _m): pass
            def error(self, _m): pass
            def debug(self, _m): pass
        return _L()

    def get_clock(self):
        # Tactile adapter calls self.get_clock().now().to_msg().
        class _Now:
            def to_msg(self):
                return SimpleNamespace(sec=0, nanosec=0)

        class _Clock:
            def now(self):
                return _Now()
        return _Clock()


def _make_rclpy_stub() -> ModuleType:
    rclpy = types.ModuleType("rclpy")

    def _init(*_a, **_kw): pass
    def _spin(_node): pass
    def _shutdown(): pass
    rclpy.init = _init
    rclpy.spin = _spin
    rclpy.shutdown = _shutdown

    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = _FakeNode
    rclpy.node = rclpy_node

    rclpy_qos = types.ModuleType("rclpy.qos")

    class _QoSProfile:
        def __init__(self, **kw): self.kw = kw

    class _Reliability:
        RELIABLE = "reliable"
        BEST_EFFORT = "best_effort"

    class _History:
        KEEP_LAST = "keep_last"
    rclpy_qos.QoSProfile = _QoSProfile
    rclpy_qos.ReliabilityPolicy = _Reliability
    rclpy_qos.HistoryPolicy = _History
    rclpy.qos = rclpy_qos

    return rclpy


def _make_geometry_msgs_stub() -> ModuleType:
    pkg = types.ModuleType("geometry_msgs")
    msg = types.ModuleType("geometry_msgs.msg")

    class _Point:
        def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)
    msg.Point = _Point
    pkg.msg = msg
    return pkg


def _make_std_msgs_stub() -> ModuleType:
    pkg = types.ModuleType("std_msgs")
    msg = types.ModuleType("std_msgs.msg")

    class _Stamp:
        def __init__(self, sec: int = 0, nanosec: int = 0):
            self.sec = int(sec)
            self.nanosec = int(nanosec)

    class _Header:
        def __init__(self):
            self.frame_id = ""
            self.stamp = _Stamp()

    class _Bool:
        def __init__(self, data: bool = False):
            self.data = bool(data)
    msg.Header = _Header
    msg.Bool = _Bool
    pkg.msg = msg
    return pkg


def _make_riskgraph_msgs_stub() -> ModuleType:
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
            self.header = None
            self.event_id = ""
            self.position = None
            self.segment_id = ""
            self.factors: List[_RiskFactor] = []
            self.confidence = 0.0
    msg.RiskEvent = _RiskEvent
    msg.RiskFactor = _RiskFactor
    pkg.msg = msg

    srv = types.ModuleType("riskgraph_msgs.srv")
    pkg.srv = srv
    return pkg


# Soft-dep upstream packages.

def _make_go2_msgs_stub() -> ModuleType:
    pkg = types.ModuleType("go2_msgs")
    msg = types.ModuleType("go2_msgs.msg")

    class _SafetyAlert:
        def __init__(self):
            self.header = None
            self.alert_type = ""
            self.description = ""
            self.distance = 0.0
    msg.SafetyAlert = _SafetyAlert
    pkg.msg = msg
    return pkg


def _make_helix_msgs_stub() -> ModuleType:
    pkg = types.ModuleType("helix_msgs")
    msg = types.ModuleType("helix_msgs.msg")

    class _FaultEvent:
        def __init__(self):
            self.severity = 0
            self.timestamp = 0.0
            self.fault_type = ""
            self.node_name = ""
            self.detail = ""
    msg.FaultEvent = _FaultEvent
    pkg.msg = msg
    return pkg


# ---------- import context manager ----------


class _BlockingFinder:
    """A sys.meta_path finder that forces named packages to be unimportable.

    Popping a name out of sys.modules does NOT hide a package that is
    genuinely installed on sys.path (e.g. go2_msgs from a sourced ROS
    workspace). Without this, the `have_go2=False` / `have_helix=False`
    code paths only exercise the "missing" branch on machines where the
    upstream package happens not to be installed, which makes the suite
    pass or fail depending on the ambient environment. This finder makes
    the "missing soft dependency" case deterministic.
    """

    def __init__(self, blocked):
        # Block the package and any submodule under it (e.g. go2_msgs.msg).
        self._blocked = tuple(blocked)

    def find_spec(self, fullname, path=None, target=None):
        for name in self._blocked:
            if fullname == name or fullname.startswith(name + "."):
                raise ModuleNotFoundError(
                    f"{fullname} blocked by test fixture", name=fullname
                )
        return None


@contextmanager
def _stubbed_imports(have_go2: bool = True, have_helix: bool = True):
    """Install ROS / msg stubs into sys.modules for the duration of the block.

    `have_go2` / `have_helix` toggle whether the soft-dep upstream packages are
    findable on the import path. When False, we both drop any cached module
    entry AND install a `sys.meta_path` finder that raises
    `ModuleNotFoundError` for that name, so the adapter's
    `from go2_msgs.msg import SafetyAlert` fails exactly as it would on a
    Jetson without the package, regardless of whether the host has the
    upstream workspace sourced.
    """
    saved = {
        name: sys.modules.get(name)
        for name in (
            "rclpy", "rclpy.node", "rclpy.qos",
            "geometry_msgs", "geometry_msgs.msg",
            "std_msgs", "std_msgs.msg",
            "riskgraph_msgs", "riskgraph_msgs.msg", "riskgraph_msgs.srv",
            "go2_msgs", "go2_msgs.msg",
            "helix_msgs", "helix_msgs.msg",
            # Adapter modules — drop them so the next `import` re-runs module body.
            "riskgraph_memory.adapters.safety_adapter",
            "riskgraph_memory.adapters.helix_adapter",
            "riskgraph_memory.adapters.tactile_adapter",
        )
    }
    rclpy = _make_rclpy_stub()
    geom = _make_geometry_msgs_stub()
    stdm = _make_std_msgs_stub()
    rgm = _make_riskgraph_msgs_stub()

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

    blocked = []
    if have_go2:
        g = _make_go2_msgs_stub()
        sys.modules["go2_msgs"] = g
        sys.modules["go2_msgs.msg"] = g.msg
    else:
        sys.modules.pop("go2_msgs", None)
        sys.modules.pop("go2_msgs.msg", None)
        blocked.append("go2_msgs")
    if have_helix:
        h = _make_helix_msgs_stub()
        sys.modules["helix_msgs"] = h
        sys.modules["helix_msgs.msg"] = h.msg
    else:
        sys.modules.pop("helix_msgs", None)
        sys.modules.pop("helix_msgs.msg", None)
        blocked.append("helix_msgs")

    # Make the "missing soft dependency" case deterministic: a genuinely
    # installed go2_msgs/helix_msgs on sys.path would otherwise still import.
    finder = _BlockingFinder(blocked) if blocked else None
    if finder is not None:
        sys.meta_path.insert(0, finder)

    # Force re-import of adapter modules. Drop both the submodule entry in
    # sys.modules AND the attribute on the parent package, otherwise
    # `from riskgraph_memory.adapters import helix_adapter` resolves the
    # stale attribute and skips re-running the module body, leaving
    # HAVE_*_MSGS set from the previous test run.
    for name in (
        "riskgraph_memory.adapters.safety_adapter",
        "riskgraph_memory.adapters.helix_adapter",
        "riskgraph_memory.adapters.tactile_adapter",
    ):
        sys.modules.pop(name, None)
        leaf = name.rsplit(".", 1)[1]
        parent = sys.modules.get("riskgraph_memory.adapters")
        if parent is not None and hasattr(parent, leaf):
            delattr(parent, leaf)

    try:
        yield
    finally:
        # Remove the import-blocking finder first so restored modules import.
        if finder is not None and finder in sys.meta_path:
            sys.meta_path.remove(finder)
        # Restore.
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v


# ---------- helpers for callback msg synthesis ----------


def _safety_alert_msg(alert_type="DROP_DETECTED", description="cliff",
                      distance=1.25, sec=10, nsec=500_000_000, frame="map"):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=sec, nanosec=nsec),
            frame_id=frame,
        ),
        alert_type=alert_type,
        description=description,
        distance=distance,
    )


def _fault_event_msg(severity=2, timestamp=12.345,
                     fault_type="JOINT_LIMIT", node_name="locomotion",
                     detail="hip_q exceeded"):
    return SimpleNamespace(
        severity=severity,
        timestamp=timestamp,
        fault_type=fault_type,
        node_name=node_name,
        detail=detail,
    )


# ---------- safety adapter -----------------------------------------------------


class TestSafetyAdapter:

    def test_alert_table_lookup_known_types(self):
        with _stubbed_imports(have_go2=True):
            from riskgraph_memory.adapters import safety_adapter as sa
            assert sa._alert_to_factor("EMERGENCY_STOP") == (1.0, "SAFETY")
            assert sa._alert_to_factor("DROP_DETECTED") == (0.9, "DEPTH")
            assert sa._alert_to_factor("STAIRS_DETECTED") == (0.7, "DEPTH")
            assert sa._alert_to_factor("NARROW_PASSAGE") == (0.5, "DEPTH")
            assert sa._alert_to_factor("SLOWDOWN") == (0.4, "SAFETY")

    def test_alert_table_unknown_type_falls_back_to_safety_default(self):
        with _stubbed_imports(have_go2=True):
            from riskgraph_memory.adapters import safety_adapter as sa
            sev, cat = sa._alert_to_factor("WHO_KNOWS")
            assert sev == 0.5
            assert cat == "SAFETY"

    def test_have_go2_msgs_true_when_stub_present(self):
        with _stubbed_imports(have_go2=True):
            from riskgraph_memory.adapters import safety_adapter as sa
            assert sa.HAVE_GO2_MSGS is True

    def test_have_go2_msgs_false_when_stub_missing(self):
        with _stubbed_imports(have_go2=False):
            from riskgraph_memory.adapters import safety_adapter as sa
            assert sa.HAVE_GO2_MSGS is False

    def test_main_no_op_when_go2_msgs_missing(self, capsys):
        with _stubbed_imports(have_go2=False):
            from riskgraph_memory.adapters import safety_adapter as sa
            # main() must return cleanly without calling rclpy.init / .spin.
            calls = []
            sa.rclpy.init = lambda *a, **kw: calls.append("init")
            sa.rclpy.spin = lambda _n: calls.append("spin")
            sa.main()
            captured = capsys.readouterr()
            assert "go2_msgs not found" in captured.err
            assert calls == [], "main() must not init/spin when soft-dep is missing"

    def test_callback_publishes_riskevent_for_drop_detected(self):
        with _stubbed_imports(have_go2=True):
            from riskgraph_memory.adapters import safety_adapter as sa
            node = sa.SafetyAdapter()
            assert len(node._pubs) == 1
            assert len(node._subs) == 1
            msg = _safety_alert_msg(
                alert_type="DROP_DETECTED", description="cliff", distance=1.25,
            )
            node._on_alert(msg)
            published = node._pubs[0].published
            assert len(published) == 1
            ev = published[0]
            assert ev.confidence == 1.0
            assert ev.header.frame_id == "map"
            assert ev.header.stamp.sec == 10
            assert ev.header.stamp.nanosec == 500_000_000
            assert len(ev.factors) == 1
            f = ev.factors[0]
            assert f.category == "DEPTH"
            assert f.severity == pytest.approx(0.9)
            assert f.source == "go2/safety_alert"
            assert "DROP_DETECTED" in f.detail
            assert "1.25" in f.detail

    def test_callback_uses_emergency_stop_severity_for_estop(self):
        with _stubbed_imports(have_go2=True):
            from riskgraph_memory.adapters import safety_adapter as sa
            node = sa.SafetyAdapter()
            msg = _safety_alert_msg(alert_type="EMERGENCY_STOP", distance=0.0)
            node._on_alert(msg)
            ev = node._pubs[0].published[0]
            assert ev.factors[0].severity == pytest.approx(1.0)
            assert ev.factors[0].category == "SAFETY"

    def test_callback_default_frame_is_map_when_upstream_blank(self):
        with _stubbed_imports(have_go2=True):
            from riskgraph_memory.adapters import safety_adapter as sa
            node = sa.SafetyAdapter()
            msg = _safety_alert_msg(frame="")
            node._on_alert(msg)
            ev = node._pubs[0].published[0]
            assert ev.header.frame_id == "map"

    def test_callback_unknown_alert_type_uses_default_factor(self):
        with _stubbed_imports(have_go2=True):
            from riskgraph_memory.adapters import safety_adapter as sa
            node = sa.SafetyAdapter()
            msg = _safety_alert_msg(alert_type="UNRECOGNIZED")
            node._on_alert(msg)
            ev = node._pubs[0].published[0]
            assert ev.factors[0].severity == pytest.approx(0.5)
            assert ev.factors[0].category == "SAFETY"


# ---------- helix adapter ------------------------------------------------------


class TestHelixAdapter:

    def test_severity_map_three_levels(self):
        with _stubbed_imports(have_helix=True):
            from riskgraph_memory.adapters import helix_adapter as ha
            assert ha._SEVERITY_MAP[1] == 0.3
            assert ha._SEVERITY_MAP[2] == 0.6
            assert ha._SEVERITY_MAP[3] == 1.0

    def test_have_helix_msgs_true_when_stub_present(self):
        with _stubbed_imports(have_helix=True):
            from riskgraph_memory.adapters import helix_adapter as ha
            assert ha.HAVE_HELIX_MSGS is True

    def test_have_helix_msgs_false_when_stub_missing(self):
        with _stubbed_imports(have_helix=False):
            from riskgraph_memory.adapters import helix_adapter as ha
            assert ha.HAVE_HELIX_MSGS is False

    def test_main_no_op_when_helix_msgs_missing(self, capsys):
        with _stubbed_imports(have_helix=False):
            from riskgraph_memory.adapters import helix_adapter as ha
            calls = []
            ha.rclpy.init = lambda *a, **kw: calls.append("init")
            ha.rclpy.spin = lambda _n: calls.append("spin")
            ha.main()
            captured = capsys.readouterr()
            assert "helix_msgs not found" in captured.err
            assert calls == []

    def test_callback_publishes_riskevent_for_critical_fault(self):
        with _stubbed_imports(have_helix=True):
            from riskgraph_memory.adapters import helix_adapter as ha
            node = ha.HelixAdapter()
            msg = _fault_event_msg(severity=3, timestamp=42.500,
                                   fault_type="OVERHEAT", node_name="vision",
                                   detail="cpu thermal throttle")
            node._on_fault(msg)
            published = node._pubs[0].published
            assert len(published) == 1
            ev = published[0]
            assert ev.confidence == 1.0
            assert ev.header.stamp.sec == 42
            # 0.500 s -> 5e8 nsec
            assert ev.header.stamp.nanosec == 500_000_000
            assert len(ev.factors) == 1
            f = ev.factors[0]
            assert f.category == "FAULT"
            assert f.severity == pytest.approx(1.0)  # CRITICAL → 1.0
            assert f.source == "helix/faults"
            assert "OVERHEAT" in f.detail
            assert "vision" in f.detail

    def test_callback_unknown_severity_uses_default(self):
        with _stubbed_imports(have_helix=True):
            from riskgraph_memory.adapters import helix_adapter as ha
            node = ha.HelixAdapter()
            msg = _fault_event_msg(severity=99)  # not in map
            node._on_fault(msg)
            ev = node._pubs[0].published[0]
            assert ev.factors[0].severity == pytest.approx(0.5)

    def test_callback_warn_severity_maps_to_low_factor(self):
        with _stubbed_imports(have_helix=True):
            from riskgraph_memory.adapters import helix_adapter as ha
            node = ha.HelixAdapter()
            msg = _fault_event_msg(severity=1)
            node._on_fault(msg)
            assert node._pubs[0].published[0].factors[0].severity == pytest.approx(0.3)

    def test_callback_clamps_negative_nanosec_to_zero(self):
        """A timestamp like 12.0 has nsec exactly 0; verify the floor at 0
        path doesn't blow up on a near-integer value."""
        with _stubbed_imports(have_helix=True):
            from riskgraph_memory.adapters import helix_adapter as ha
            node = ha.HelixAdapter()
            msg = _fault_event_msg(severity=2, timestamp=7.0)
            node._on_fault(msg)
            ev = node._pubs[0].published[0]
            assert ev.header.stamp.sec == 7
            assert 0 <= ev.header.stamp.nanosec <= 999_999_999


# ---------- tactile adapter ----------------------------------------------------
#
# Tactile has no soft upstream dep; coverage is logical edge-cases of the
# leading-edge debouncer plus the standard normal/edge/missing-field shape.


class TestTactileAdapter:

    def test_module_imports_with_stubs(self):
        with _stubbed_imports():
            from riskgraph_memory.adapters import tactile_adapter as ta
            assert hasattr(ta, "TactileAdapter")

    def test_callback_emits_event_on_leading_edge_only(self):
        with _stubbed_imports():
            from riskgraph_memory.adapters import tactile_adapter as ta
            node = ta.TactileAdapter()
            true_msg = SimpleNamespace(data=True)
            false_msg = SimpleNamespace(data=False)
            # First True: leading edge → 1 event.
            node._on_slip(true_msg)
            # Held True: no new event.
            node._on_slip(true_msg)
            node._on_slip(true_msg)
            assert len(node._pubs[0].published) == 1
            # Drop and re-rise → second event.
            node._on_slip(false_msg)
            node._on_slip(true_msg)
            assert len(node._pubs[0].published) == 2

    def test_callback_default_severity_is_used(self):
        with _stubbed_imports():
            from riskgraph_memory.adapters import tactile_adapter as ta
            node = ta.TactileAdapter()
            node._on_slip(SimpleNamespace(data=True))
            ev = node._pubs[0].published[0]
            f = ev.factors[0]
            assert f.category == "SLIP"
            assert f.severity == pytest.approx(0.7)  # adapter default
            assert f.source == "tactile/slip_state"
            assert "leading edge" in f.detail
            assert ev.confidence == 1.0

    def test_callback_starting_with_false_is_a_no_op(self):
        with _stubbed_imports():
            from riskgraph_memory.adapters import tactile_adapter as ta
            node = ta.TactileAdapter()
            node._on_slip(SimpleNamespace(data=False))
            node._on_slip(SimpleNamespace(data=False))
            assert node._pubs[0].published == []

    def test_callback_handles_truthy_int_data(self):
        """std_msgs/Bool is bool-typed but adapter does bool(msg.data); a 1/0
        upstream value must still trigger the leading edge."""
        with _stubbed_imports():
            from riskgraph_memory.adapters import tactile_adapter as ta
            node = ta.TactileAdapter()
            node._on_slip(SimpleNamespace(data=1))
            assert len(node._pubs[0].published) == 1
            node._on_slip(SimpleNamespace(data=0))
            node._on_slip(SimpleNamespace(data=1))
            assert len(node._pubs[0].published) == 2
