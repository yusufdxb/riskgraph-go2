"""Static guards on the architectural boundary and the Nav2 configuration.

RiskGraph is a planning / risk-memory layer with exactly one motion exit:
``riskgraph_sport_sink`` (sport_sink.py), the only node allowed to publish
GO2 sport requests, and ``riskgraph_sink_stage`` (sink_stage.py), the only
code allowed to publish a velocity (bounded test commands into the sink).
Sport API ids exist as values only in sport_sink_core.py (and the stand-in
rehearsal robot that consumes them), so a hand-typed 1001 (Damp) cannot
appear anywhere else. Nav2's velocity limits must stay strictly inside the
sink's (the sink REJECTS over-limit commands with StopMove, never clamps).
"""
import ast
from pathlib import Path

import yaml

from riskgraph_nav.sport_sink_core import SinkLimits

SRC = Path(__file__).resolve().parents[2]
NAV2 = SRC / "riskgraph_bringup" / "config" / "nav2_live.yaml"
_L = SinkLimits()
SINK_LIMITS = {"vx": _L.max_vx, "vy": _L.max_vy, "wz": _L.max_wz}
SPORT_PUBLISHERS = {"sport_sink.py"}
VELOCITY_PUBLISHERS = {"sink_stage.py"}
API_ID_OWNERS = {"sport_sink_core.py", "rehearsal_go2.py"}
SPORT_API_IDS = {1001, 1002, 1003, 1008}


def _py_files():
    for pkg in ("riskgraph_core", "riskgraph_memory", "riskgraph_planner", "riskgraph_explainer",
                "riskgraph_nav", "riskgraph_demo", "riskgraph_bringup"):
        for p in (SRC / pkg).rglob("*.py"):
            if "/test/" not in str(p):
                yield p


def _publisher_calls(p):
    text = p.read_text()
    for node in ast.walk(ast.parse(text)):
        if isinstance(node, ast.Call) and getattr(node.func, "attr", "") == "create_publisher":
            yield ast.get_source_segment(text, node) or ""


def test_only_the_sink_publishes_sport_requests():
    offenders = [f"{p}: {src}" for p in _py_files() if p.name not in SPORT_PUBLISHERS
                 for src in _publisher_calls(p) if "sport/request" in src or "Request," in src]
    assert offenders == []


def test_only_the_sink_stage_publishes_velocity():
    offenders = [f"{p}: {src}" for p in _py_files() if p.name not in VELOCITY_PUBLISHERS
                 for src in _publisher_calls(p) if "Twist" in src or "cmd_vel" in src]
    assert offenders == []


def test_sport_sink_files_exist_and_publish():
    srcs = [src for p in _py_files() if p.name in SPORT_PUBLISHERS for src in _publisher_calls(p)]
    assert any("Request" in s for s in srcs)


def test_sport_api_ids_only_as_named_constants_in_the_sink_core():
    offenders = []
    for p in _py_files():
        if p.name in API_ID_OWNERS:
            continue
        for node in ast.walk(ast.parse(p.read_text())):
            if isinstance(node, ast.Constant) and type(node.value) is int and node.value in SPORT_API_IDS:
                offenders.append(f"{p}:{node.lineno}: {node.value}")
    assert offenders == []


def test_damp_is_never_a_value_anywhere():
    # 1001 on /api/sport/request is Damp: the robot drops. Not even the sink core holds it.
    for p in _py_files():
        if p.name == "rehearsal_go2.py":
            continue
        for node in ast.walk(ast.parse(p.read_text())):
            assert not (isinstance(node, ast.Constant) and node.value == 1001), f"{p}:{node.lineno}"



def test_controller_output_is_remapped_into_the_arbiter_source():
    launch = (SRC / "riskgraph_bringup" / "launch" / "riskgraph_nav_live.launch.py").read_text()
    # Topic names are built from sink_prefix, whose default "" gives exactly
    # /cmd_vel_nav -> /nav/cmd_vel (a non-empty prefix is the stationary check).
    assert 'DeclareLaunchArgument("sink_prefix", default_value="")' in launch
    assert 'cmd_vel_nav = f"{prefix}/cmd_vel_nav"' in launch
    assert 'nav_cmd_vel = f"{prefix}/nav/cmd_vel"' in launch
    assert '("cmd_vel", cmd_vel_nav)' in launch
    assert '("cmd_vel_smoothed", nav_cmd_vel)' in launch
    assert '/cmd_vel")' not in launch.replace('/cmd_vel_nav")', "").replace('/nav/cmd_vel")', "")


def test_nav2_velocity_limits_inside_sink_limits():
    cfg = yaml.safe_load(NAV2.read_text())
    vs = cfg["velocity_smoother"]["ros__parameters"]
    mx, mn = vs["max_velocity"], vs["min_velocity"]
    assert mx[0] < SINK_LIMITS["vx"] and mn[0] >= 0.0          # no reversing
    assert abs(mx[1]) < SINK_LIMITS["vy"] and abs(mn[1]) < SINK_LIMITS["vy"]
    assert mx[2] < SINK_LIMITS["wz"] and abs(mn[2]) < SINK_LIMITS["wz"]
    rpp = cfg["controller_server"]["ros__parameters"]["FollowPath"]
    assert rpp["desired_linear_vel"] < SINK_LIMITS["vx"]
    assert rpp["use_rotate_to_heading"] is False and rpp["allow_reversing"] is False


def test_no_recovery_behaviors_or_bt_navigator():
    cfg = yaml.safe_load(NAV2.read_text())
    assert "bt_navigator" not in cfg and "behavior_server" not in cfg
    names = cfg["lifecycle_manager_riskgraph_nav"]["ros__parameters"]["node_names"]
    assert set(names) == {"map_server", "planner_server", "controller_server", "velocity_smoother"}


def test_risk_layer_is_a_nonlethal_planning_cost():
    gc = yaml.safe_load(NAV2.read_text())["global_costmap"]["global_costmap"]["ros__parameters"]
    assert gc["plugins"] == ["static_layer", "riskgraph_layer", "inflation_layer"]
    assert gc["riskgraph_layer"]["map_topic"] == "/riskgraph/risk_costmap"
    assert gc["trinary_costmap"] is False and gc["use_maximum"] is True
    assert gc["lethal_cost_threshold"] == 100
    lc = yaml.safe_load(NAV2.read_text())["local_costmap"]["local_costmap"]["ros__parameters"]
    assert "riskgraph_layer" not in lc["plugins"]  # risk never enters collision checking
