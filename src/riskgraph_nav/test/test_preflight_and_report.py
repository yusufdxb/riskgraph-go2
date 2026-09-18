"""Preflight verdict logic and the evidence report's PASS rules."""
import copy
import json
import os
import time

import pytest

from riskgraph_nav.paths import resolve
from riskgraph_nav.preflight import (
    ARBITER, FAIL, GLOBAL_COSTMAP, LOC_NODE, LIVE_PKGS, NAV_NODES, REQUIRED_PKGS, RG_NODES, SINK,
    Config, evaluate, verdict)
from riskgraph_nav.report import criteria

R = resolve()


def good_facts():
    return {
        "git": {"sha": "abc", "branch": "main", "dirty": False, "porcelain": "", "head_commit_time": 1789000000},
        "env": {"ROS_DISTRO": "humble", "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp", "ROS_DOMAIN_ID": "0",
                "ROS_LOCALHOST_ONLY": None},
        "net": {"uri_ifaces": ["enP8p1s0"], "iface_up": True},
        "clock": {"now": time.time(), "head_commit_time": 1789000000},
        "packages": {p: True for p in REQUIRED_PKGS + LIVE_PKGS},
        "disk": {"free_gb": 50.0}, "processes": {"bag_play": [], "trial_runner": []},
        "clock_publishers": [], "use_sim_time": {n: False for n in RG_NODES}, "duplicate_nodes": [],
        "odom": {"type": "nav_msgs/msg/Odometry", "rate_hz": 150.0, "frame": "odom", "child": "base_link"},
        "sportmodestate": {"rate_hz": 295.0}, "sport_response_publishers": ["/robot"],
        "localization": {"localization_valid": True, "anchored": True, "map_id": R.map_id, "speed_mps": 0.0,
                         "state": "OK"},
        "tf": {"map_base_ok": True, "odom_base_age_s": 0.02},
        "tf_publishers": [LOC_NODE], "tf_static_publishers": [LOC_NODE],
        "lifecycle": {n: "active" for n in NAV_NODES},
        "actions": {"compute_path_to_pose": True, "follow_path": True},
        "nav_cmd_vel_publishers": ["/velocity_smoother"],
        "risk_costmap_subscribers": [GLOBAL_COSTMAP],
        "velocity_smoother_params": {"max_velocity": [0.2, 0.0, 0.4], "min_velocity": [0.0, 0.0, -0.4]},
        "sink_params": {"mode": "armed", "max_vx": 0.25, "max_wz": 0.5},
        "riskgraph_nodes": list(RG_NODES),
        "riskgraph_status": {"db_path": R.store_path, "stamp": time.time(), "schema_version": 2,
                             "map_id": R.map_id, "evidence_class": "live", "health": [], "grid_state": "OK",
                             "grid_stats": {"out_of_range_cells": 0}},
        "riskgraph_store_paths": {n: R.store_path for n in RG_NODES},
        "riskgraph_motion_topics": {n: [] for n in RG_NODES},
        "cmd_vel_publishers": [ARBITER], "cmd_vel_subscribers": [SINK],
        "sport_request_publishers": ["/stock_a", SINK], "twist_mux_nodes": [],
        "arbiter": {"fresh": True, "hold_active": False, "output_zero": True},
        "helix_evidence": {"D": {"verdict": "PASS", "rehearsal": False}, "E": {"verdict": "PASS", "rehearsal": False}},
    }


def cfg(**kw):
    c = Config(mode="live", riskgraph="expect-running", store_path=R.store_path, map_id=R.map_id,
               sport_baseline=["/stock_a"])
    for k, v in kw.items():
        setattr(c, k, v)
    return c


def test_good_facts_are_go():
    checks = evaluate(good_facts(), cfg())
    fails = [c for c in checks if c.result == FAIL]
    assert fails == [] and verdict(checks) == "GO"


@pytest.mark.parametrize("mutate,check_id", [
    (lambda f: f["git"].update(dirty=True), "P01"),
    (lambda f: f["env"].update(RMW_IMPLEMENTATION="rmw_fastrtps_cpp"), "P04"),
    (lambda f: f["net"].update(uri_ifaces=["enp0s31f6"]), "P06"),
    (lambda f: f["clock"].update(now=100.0), "P07"),
    (lambda f: f["packages"].update(nav2_planner=False), "P08"),
    (lambda f: f["disk"].update(free_gb=0.5), "P09"),
    (lambda f: f["processes"].update(bag_play=["123 ros2 bag play x"]), "P10"),
    (lambda f: f.update(clock_publishers=["/player"]), "P10"),
    (lambda f: f["use_sim_time"].update({"/riskgraph_memory": True}), "P11"),
    (lambda f: f.update(duplicate_nodes=["/riskgraph_memory"]), "P12"),
    (lambda f: f["odom"].update(rate_hz=0.0), "P14"),
    (lambda f: f["localization"].update(localization_valid=False), "P17"),
    (lambda f: f["localization"].update(map_id="other-map"), "P18"),
    (lambda f: f["tf"].update(map_base_ok=False), "P19"),
    (lambda f: f.update(tf_publishers=[LOC_NODE, "/odom_broadcaster"]), "P20"),
    (lambda f: f["lifecycle"].update({"/planner_server": "inactive"}), "P21"),
    (lambda f: f["actions"].update(follow_path=False), "P22"),
    (lambda f: f.update(nav_cmd_vel_publishers=["/velocity_smoother", "/teleop"]), "P23"),
    (lambda f: f.update(risk_costmap_subscribers=[]), "P24"),
    (lambda f: f["velocity_smoother_params"].update(max_velocity=[0.3, 0.0, 0.4]), "P25"),
    (lambda f: f["riskgraph_store_paths"].update({"/riskgraph_planner": "/tmp/other.sqlite"}), "P27"),
    (lambda f: f["riskgraph_status"].update(map_id="stale-map"), "P28"),
    (lambda f: f["riskgraph_status"].update(evidence_class="replay"), "P28"),
    (lambda f: f["riskgraph_motion_topics"].update({"/riskgraph_memory": ["/cmd_vel"]}), "P30"),
    (lambda f: f.update(cmd_vel_publishers=[ARBITER, "/rogue"]), "P31"),
    (lambda f: f.update(cmd_vel_subscribers=[]), "P32"),
    (lambda f: f.update(sport_request_publishers=["/stock_a", SINK, "/new_thing"]), "P33"),
    (lambda f: f.update(twist_mux_nodes=["/twist_mux"]), "P34"),
    (lambda f: f["arbiter"].update(hold_active=True), "P35"),
    (lambda f: f["sink_params"].update(mode="dry_run"), "P36"),
    (lambda f: f["helix_evidence"].update(E=None), "P37"),
    (lambda f: f["helix_evidence"].update(D={"verdict": "PASS", "rehearsal": True}), "P37"),
    (lambda f: f["localization"].update(speed_mps=0.3), "P38"),
])
def test_each_blocking_condition_is_no_go(mutate, check_id):
    f = copy.deepcopy(good_facts())
    mutate(f)
    checks = evaluate(f, cfg())
    failed = {c.id for c in checks if c.result == FAIL}
    assert check_id in failed
    assert verdict(checks) == "NO-GO"


def test_missing_sport_baseline_is_no_go_live_but_info_in_rehearsal():
    assert "P33" in {c.id for c in evaluate(good_facts(), cfg(sport_baseline=None)) if c.result == FAIL}
    f = good_facts()
    f["riskgraph_status"]["evidence_class"] = "rehearsal"
    f["helix_evidence"] = {"D": {"verdict": "PASS", "rehearsal": True},
                           "E": {"verdict": "PASS", "rehearsal": True}}
    checks = evaluate(f, cfg(mode="rehearsal", sport_baseline=None))
    assert verdict(checks) == "GO"


def test_expect_absent_rejects_running_riskgraph_and_foreign_db():
    f = good_facts()
    f["db"] = {"exists": True, "map_id": R.map_id, "evidence_class": "live", "dir_writable": True}
    assert verdict(evaluate(f, cfg(riskgraph="expect-absent"))) == "NO-GO"  # nodes still running
    f["riskgraph_nodes"] = []
    assert verdict(evaluate(f, cfg(riskgraph="expect-absent"))) == "GO"
    f["db"]["map_id"] = "other"
    assert "P27" in {c.id for c in evaluate(f, cfg(riskgraph="expect-absent")) if c.result == FAIL}


def test_paths_are_absolute_and_keyed_by_map_id(tmp_path):
    r = resolve(db_root=str(tmp_path), db_tag="t1")
    assert os.path.isabs(r.store_path)
    assert r.store_path == str(tmp_path / r.map_id / "t1.sqlite")
    with pytest.raises(ValueError):
        resolve(db_tag="../escape")


def _results(ok=True):
    return {"status": "COMPLETED", "preflight_verdict": "GO", "operator_attested": True,
            "bag": {"ok": True, "message_count": 1000}, "trials": {
        "A_baseline": {"plan": {"side": "left"}, "execution": {"succeeded": True, "arbiter": {"nonzero_while_hold": 0},
                                                              "post": {"stationary_after": True}}},
        "B_inject": {"row_ok": True, "position_error_m": 0.0, "db_path": "/db", "incidents_before": 0,
                     "incidents_after": 1, "nav2_received": True},
        "C_risk_aware": {"plan": {"side": "right"}, "comparison": {"risk_reduced": True, "pass": True},
                         "execution": {"succeeded": True, "executed_side": "right" if ok else "left",
                                       "arbiter": {"nonzero_while_hold": 0},
                                       "post": {"stationary_after": True}},
                         "executed_comparison": {"risk_reduced": True}},
        "D_restart": {"instance_before": "a", "instance_after": "b", "incidents_before": 1,
                      "incidents_after": 1, "db_path_after": "/db", "pass": True},
        "E_fallback": {"pass": True}}}


def _bundle(tmp_path):
    for f in ("manifest.json", "preflight.json", "db/riskgraph_after.sqlite", "logs/runner.log"):
        p = tmp_path / f
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text("x")
    (tmp_path / "bag").mkdir()


def test_report_all_criteria_pass(tmp_path):
    _bundle(tmp_path)
    crit = criteria(_results(), {"db_path": "/db"}, str(tmp_path))
    assert all(c["pass"] for c in crit), [c for c in crit if not c["pass"]]
    assert len(crit) == 14


def test_report_fails_when_robot_did_not_take_the_planned_corridor(tmp_path):
    _bundle(tmp_path)
    crit = {c["id"]: c["pass"] for c in criteria(_results(ok=False), {"db_path": "/db"}, str(tmp_path))}
    assert crit["8"] is False


def test_report_fails_when_robot_did_not_stop(tmp_path):
    _bundle(tmp_path)
    r = _results()
    r["trials"]["A_baseline"]["execution"]["post"]["stationary_after"] = False
    crit = {c["id"]: c["pass"] for c in criteria(r, {"db_path": "/db"}, str(tmp_path))}
    assert crit["2"] is False


def test_report_fails_when_bag_recorder_died(tmp_path):
    _bundle(tmp_path)
    r = _results()
    r["bag"] = {"ok": False, "died_before_stop": True}
    crit = {c["id"]: c["pass"] for c in criteria(r, {"db_path": "/db"}, str(tmp_path))}
    assert crit["13"] is False


def test_report_fails_without_bag(tmp_path):
    crit = {c["id"]: c["pass"] for c in criteria(_results(), {"db_path": "/db"}, str(tmp_path))}
    assert crit["13"] is False


def test_hardware_pass_is_impossible_for_rehearsal(tmp_path):
    from riskgraph_nav.report import build_report

    class _Ros:
        class r:
            exp = R.experiment

        def current_field(self):
            from riskgraph_core.risk_field import RiskField
            return RiskField.from_events([], R.experiment.risk_params, now=0)
    _bundle(tmp_path)
    info = None
    rep = build_report(str(tmp_path), {"evidence_class": "rehearsal", "db_path": "/db", "label": "REHEARSAL"},
                       _results(), _Ros(), info)
    assert rep["machine_pass"] is True and rep["hardware_pass"] is False
    s = json.load(open(tmp_path / "summary.json"))
    assert s["hardware_pass"] is False
    rep2 = build_report(str(tmp_path), {"evidence_class": "hardware", "db_path": "/db", "label": "HW"},
                        dict(_results(), operator_attested=False), _Ros(), info)
    assert rep2["hardware_pass"] is False
    rep3 = build_report(str(tmp_path), {"evidence_class": "hardware", "db_path": "/db", "label": "HW"},
                        _results(), _Ros(), info)
    assert rep3["hardware_pass"] is True
