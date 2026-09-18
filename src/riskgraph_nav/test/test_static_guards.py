"""Static guards on the architectural boundary and the Nav2 configuration.

RiskGraph is a planning / risk-memory layer. These tests fail the build if
any RiskGraph source creates a publisher for a velocity or a GO2 sport
command, or if Nav2's velocity limits could exceed the HELIX sport sink's
(the sink REJECTS over-limit commands with StopMove instead of clamping).
"""
import ast
from pathlib import Path

import yaml

SRC = Path(__file__).resolve().parents[2]
NAV2 = SRC / "riskgraph_bringup" / "config" / "nav2_live.yaml"
SINK_LIMITS = {"vx": 0.25, "vy": 0.20, "wz": 0.50}  # helix_arbiter sport_sink_core.SinkLimits


def _py_files():
    for pkg in ("riskgraph_core", "riskgraph_memory", "riskgraph_planner", "riskgraph_explainer",
                "riskgraph_nav", "riskgraph_demo", "riskgraph_bringup"):
        for p in (SRC / pkg).rglob("*.py"):
            if "/test/" not in str(p):
                yield p


def test_no_riskgraph_code_publishes_motion():
    offenders = []
    for p in _py_files():
        if p.name == "rehearsal_go2.py":  # the stand-in ROBOT consumes sport requests, never publishes them
            continue
        tree = ast.parse(p.read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and getattr(node.func, "attr", "") == "create_publisher":
                src = ast.get_source_segment(p.read_text(), node) or ""
                if any(k in src for k in ("Twist", "cmd_vel", "sport/request", "Request,")):
                    offenders.append(f"{p}: {src}")
    assert offenders == []


def test_no_riskgraph_code_mentions_sport_api_ids_as_commands():
    for p in _py_files():
        text = p.read_text()
        if p.name == "rehearsal_go2.py":
            continue
        assert "api_id" not in text, p


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
