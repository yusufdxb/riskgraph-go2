"""Sink stage runner logic on synthetic samples. No ROS, no robot."""
import json
import math
from pathlib import Path

import pytest

from riskgraph_nav import sink_stage_core as core

T0 = 100.0
SHA = "a" * 40


def make_odom(t_end, v_of_t, hz=150.0, t_from=T0 - 1.0):
    odom, x, t = [], 0.0, t_from
    while t <= t_end:
        odom.append([t, x, 0.0])
        x += v_of_t(t) / hz
        t += 1.0 / hz
    return odom


def lag_velocity(vx, t_on, t_off, tau):
    """First-order lag toward vx between t_on and t_off, then toward zero."""
    def v(t):
        if t < t_on:
            return 0.0
        if t < t_off:
            return vx * (1 - math.exp(-(t - t_on) / tau))
        v_off = vx * (1 - math.exp(-(t_off - t_on) / tau))
        return v_off * math.exp(-(t - t_off) / tau)
    return v


def samples_for(stage, *, vx=None, tau=0.25, stop_effect_delay=0.0, code=0, answer=True,
                late_move=False, deadman_latency=0.30, zero_reason="ZERO", moves=True,
                not_armed=True, stop_api=core.API_STOP_MOVE):
    """A good run by default; keyword args break one thing at a time."""
    plan = core.PLAN[stage]
    vx = plan["segments"][0][1] if vx is None else vx
    move_s = plan["segments"][0][0]
    t_ref = T0 + move_s                      # first zero (S0, S1) or last command (S2)
    t_last = T0 + move_s if stage == "S2" else T0 + sum(d for d, _ in plan["segments"]) - 0.05
    t_end = t_last + (plan["silent_s"] if stage == "S2" else 0.0) + plan["tail_s"]
    odom = make_odom(t_end, lag_velocity(vx, T0, t_ref + stop_effect_delay, tau)
                     if stage != "S0" else (lambda t: 0.0))
    trace, resp, rid = [], [], [1000]

    def add(api, reason, t, sent=True, answer_it=True, code_=0):
        rid[0] += 1
        trace.append({"api_id": api, "reason": reason, "request_id": rid[0], "sent_to_robot": sent,
                      "mode": "x", "t_rx": t})
        if sent and answer and answer_it:
            resp.append({"t_rx": t + 0.02, "id": rid[0], "api_id": api, "code": code_})

    t = T0
    while t < t_ref:
        if stage == "S0":
            if not_armed:
                add(core.API_STOP_MOVE, "NOT_ARMED", t + 0.005)
        elif moves:
            add(core.API_MOVE, "MOVE", t + 0.005)
        t += 0.05
    if late_move:
        add(core.API_MOVE, "MOVE", t_ref + 0.3)
    if stage == "S0":
        add(core.API_STOP_MOVE, "ZERO", t_ref + 0.005, code_=code)
        trace[-1]["t_rx"] = t_ref + 0.005
        # make the answered code the one under test
        for r in resp:
            r["code"] = code
    elif stage == "S1":
        add(stop_api, zero_reason, t_ref + 0.005, code_=code)
    else:
        add(stop_api, "DEADMAN", t_ref + deadman_latency, code_=code)
    ph = {"t_start": T0, "t_zero": t_ref if stage != "S2" else None,
          "t_last_cmd": t_last, "t_end": t_end}
    return {"phases": ph, "odom": odom, "trace": trace, "responses": resp, "cmds": []}


def run(stage, **kw):
    s = samples_for(stage, **kw)
    checks, m = core.evaluate_stage(stage, s, 0.25)
    return checks, m


def failed(checks):
    return {c.id for c in checks if not c.ok}


# -- speed estimate and safety monitor ----------------------------------------------

def test_speed_series_recovers_constant_velocity():
    odom = make_odom(T0 + 2.0, lambda t: 0.15)
    ser = core.speed_series(odom)
    assert ser and all(abs(v - 0.15) < 0.005 for _t, v in ser)
    assert ser[0][0] >= odom[0][0] + core.SPEED_WINDOW_S - 0.01


def test_first_sustained_below():
    ser = [(0.0, 0.2), (0.1, 0.03), (0.2, 0.2), (0.3, 0.03), (0.4, 0.03), (0.5, 0.03),
           (0.6, 0.03), (0.7, 0.03)]
    assert core.first_sustained_below(ser, 0.05, 0.3, 0.0) == pytest.approx(0.3)
    assert core.first_sustained_below(ser[:6], 0.05, 0.3, 0.0) is None
    assert core.first_sustained_below(ser, 0.05, 0.3, 0.5) is None


def test_safety_monitor_quiet_when_slow_and_trips_on_speed_or_distance():
    mon = core.SafetyMonitor()
    for t, x, y in make_odom(T0 + 1.0, lambda t: 0.15):
        assert mon.add(t, x, y) is None
    fast = core.SafetyMonitor()
    reasons = [fast.add(t, x, y) for t, x, y in make_odom(T0 + 1.0, lambda t: 0.5)]
    assert any(r and "speed" in r for r in reasons)
    far = core.SafetyMonitor()
    reasons = [far.add(t, x, y) for t, x, y in make_odom(T0 + 12.0, lambda t: 0.2)]
    assert any(r and "displacement" in r for r in reasons)


# -- stage evaluation: good runs ----------------------------------------------------

@pytest.mark.parametrize("stage", core.STAGES)
def test_good_samples_pass(stage):
    checks, m = run(stage)
    assert failed(checks) == set(), [c for c in checks if not c.ok]
    assert core.decide_verdict([], checks) == "PASS"


def test_good_measurements_are_populated():
    _c, m1 = run("S1")
    assert 0.14 < m1["peak_speed_mps"] < 0.17
    assert 0.0 < m1["stop_latency_s"] < core.STOP_BY_S
    assert m1["moves_after_stop"] == 0 and m1["response_codes"] == [0]
    _c, m2 = run("S2")
    assert m2["deadman_latency_s"] == pytest.approx(0.30, abs=0.01)
    _c, m0 = run("S0")
    assert m0["moves_in_window"] == 0 and m0["peak_speed_mps"] < core.STILL_MPS


# -- stage evaluation: failures -----------------------------------------------------

def test_s0_move_seen_fails():
    s = samples_for("S0")
    s["trace"].append({"api_id": core.API_MOVE, "reason": "MOVE", "request_id": 1, "t_rx": T0 + 0.5,
                       "sent_to_robot": True})
    checks, m = core.evaluate_stage("S0", s)
    assert "S0-no-move" in failed(checks) and m["moves_in_window"] == 1
    assert core.decide_verdict([], checks) == "FAIL"


def test_s0_no_not_armed_stop_fails():
    checks, _ = run("S0", not_armed=False)
    assert "S0-not-armed" in failed(checks)


def test_s0_robot_moving_fails():
    s = samples_for("S0")
    s["odom"] = make_odom(s["phases"]["t_end"], lag_velocity(0.15, T0, T0 + 5, 0.25))
    checks, _ = core.evaluate_stage("S0", s)
    assert "S0-still" in failed(checks)


@pytest.mark.parametrize("kw", [{"answer": False}, {"code": -1}])
def test_s0_unanswered_or_nonzero_code_fails(kw):
    checks, _ = run("S0", **kw)
    assert "S0-answered" in failed(checks)


def test_s1_stop_effect_delayed_fails():
    checks, _ = run("S1", stop_effect_delay=1.6)
    assert "S1-stopped" in failed(checks)


def test_s1_move_after_zero_fails():
    checks, m = run("S1", late_move=True)
    assert "S1-no-late-move" in failed(checks) and m["moves_after_stop"] == 1


@pytest.mark.parametrize("kw,cid", [({"answer": False}, "S1-answered"),
                                    ({"code": 3}, "S1-answered"),
                                    ({"zero_reason": "NOT_ARMED"}, "S1-zero-stop")])
def test_s1_response_and_reason_failures(kw, cid):
    checks, _ = run("S1", **kw)
    assert cid in failed(checks)


@pytest.mark.parametrize("vx", [0.40, 0.02])
def test_s1_peak_out_of_range_fails(vx):
    checks, m = run("S1", vx=vx)
    assert "S1-peak" in failed(checks)


def test_s1_sink_that_never_moved_fails():
    checks, _ = run("S1", moves=False)
    assert "S1-moved" in failed(checks)


def test_s2_deadman_late_fails():
    checks, m = run("S2", deadman_latency=0.60)
    assert "S2-deadman" in failed(checks)
    ok, _ = run("S2", deadman_latency=0.39)
    assert "S2-deadman" not in failed(ok)  # inside the 0.25 + 0.15 s limit


def test_s2_deadman_limit_follows_input_timeout():
    s = samples_for("S2", deadman_latency=0.60)
    checks, _ = core.evaluate_stage("S2", s, input_timeout_sec=0.50)
    assert "S2-deadman" not in failed(checks)


@pytest.mark.parametrize("kw,cid", [({"answer": False}, "S2-answered"),
                                    ({"code": 7}, "S2-answered"),
                                    ({"late_move": True}, "S2-no-late-move"),
                                    ({"stop_effect_delay": 1.6}, "S2-stopped")])
def test_s2_failures(kw, cid):
    checks, _ = run("S2", **kw)
    assert cid in failed(checks)


def test_s2_stop_is_not_deadman_reason_zero():
    checks, _ = run("S2")
    assert "S2-deadman" not in failed(checks)
    s = samples_for("S2")
    for e in s["trace"]:
        if e["reason"] == "DEADMAN":
            e["reason"] = "ZERO"
    checks, _ = core.evaluate_stage("S2", s)
    assert "S2-deadman" in failed(checks)


def test_abort_fails_the_stage_with_reason():
    s = samples_for("S1")
    checks, _ = core.evaluate_stage("S1", s, abort="speed 0.4 m/s > 0.35")
    assert "S1-abort" in failed(checks)
    assert "0.4" in [c for c in checks if c.id == "S1-abort"][0].detail
    assert core.decide_verdict([], checks) == "FAIL"


@pytest.mark.parametrize("stage", core.STAGES)
def test_empty_odometry_cannot_pass(stage):
    s = samples_for(stage)
    s["odom"] = []
    checks, _ = core.evaluate_stage(stage, s)
    assert f"{stage}-odom" in failed(checks)


def test_unknown_stage_rejected():
    with pytest.raises(ValueError):
        core.evaluate_stage("S9", samples_for("S1"))


# -- ordering and preconditions -----------------------------------------------------

def ev(stage, verdict="PASS", sha=SHA, rehearsal=False):
    return {"stage": stage, "verdict": verdict, "git_sha": sha, "rehearsal": rehearsal}


def test_s0_has_no_prerequisite():
    assert core.check_ordering("S0", {"S0": None, "S1": None, "S2": None}, SHA, False).ok


def test_s1_needs_s0_pass():
    none = {"S0": None, "S1": None, "S2": None}
    assert not core.check_ordering("S1", none, SHA, False).ok
    assert not core.check_ordering("S1", {**none, "S0": ev("S0", "FAIL")}, SHA, False).ok
    assert not core.check_ordering("S1", {**none, "S0": ev("S0", "INCOMPLETE")}, SHA, False).ok
    assert core.check_ordering("S1", {**none, "S0": ev("S0")}, SHA, False).ok


def test_s2_needs_s1_not_just_s0():
    e = {"S0": ev("S0"), "S1": None, "S2": None}
    assert not core.check_ordering("S2", e, SHA, False).ok
    e["S1"] = ev("S1")
    assert core.check_ordering("S2", e, SHA, False).ok


def test_sha_mismatch_refused():
    c = core.check_ordering("S1", {"S0": ev("S0", sha="b" * 40)}, SHA, False)
    assert not c.ok and "sha mismatch" in c.detail


def test_rehearsal_evidence_never_unlocks_live_and_vice_versa():
    assert not core.check_ordering("S1", {"S0": ev("S0", rehearsal=True)}, SHA, False).ok
    assert not core.check_ordering("S1", {"S0": ev("S0", rehearsal=False)}, SHA, True).ok
    assert core.check_ordering("S1", {"S0": ev("S0", rehearsal=True)}, SHA, True).ok


def good_facts(**over):
    f = {"nodes": [core.SINK_NODE, "/other"], "sink_mode": "stop_only", "input_timeout_sec": 0.25,
         "cmd_vel_publishers": [], "sink_subscribed": True, "odom_hz": 148.0,
         "odom_peak_speed": 0.004, "trace_age_s": 0.3,
         "git": {"sha": SHA, "dirty": False, "porcelain": ""}}
    f.update(over)
    return f


NO_EV = {"S0": None, "S1": None, "S2": None}


def test_preconditions_all_ok_for_s0():
    pre = core.precondition_checks("S0", good_facts(), NO_EV, False)
    assert all(c.ok for c in pre), [c for c in pre if not c.ok]


@pytest.mark.parametrize("over,cid", [
    ({"nodes": ["/other"]}, "sink-node"),
    ({"sink_mode": "armed"}, "sink-mode"),
    ({"sink_mode": None}, "sink-mode"),
    ({"cmd_vel_publishers": ["/velocity_smoother"]}, "cmd-vel-clear"),
    ({"cmd_vel_publishers": None}, "cmd-vel-clear"),
    ({"sink_subscribed": False}, "sink-subscribed"),
    ({"odom_hz": 20.0}, "odom-rate"),
    ({"odom_peak_speed": 0.08}, "robot-still"),
    ({"odom_peak_speed": None}, "robot-still"),
    ({"trace_age_s": 2.0}, "trace-fresh"),
    ({"trace_age_s": None}, "trace-fresh"),
    ({"git": {"sha": SHA, "dirty": True, "porcelain": " M x"}}, "git-clean"),
    ({"git": {"sha": "", "dirty": False}}, "git-clean"),
    ({"nodes": [core.SINK_NODE, core.REHEARSAL_NODE]}, "rehearsal-flag"),
])
def test_each_precondition_failure_is_incomplete(over, cid):
    pre = core.precondition_checks("S0", good_facts(**over), NO_EV, False)
    assert cid in failed(pre)
    assert core.decide_verdict(pre, []) == "INCOMPLETE"


def test_rehearsal_flag_needs_the_rehearsal_node():
    nodes = [core.SINK_NODE, core.REHEARSAL_NODE]
    assert all(c.ok for c in core.precondition_checks("S0", good_facts(nodes=nodes), NO_EV, True))
    assert "rehearsal-flag" in failed(core.precondition_checks("S0", good_facts(), NO_EV, True))


def test_required_sink_mode_per_stage():
    ok = core.precondition_checks("S1", good_facts(sink_mode="armed"), {"S0": ev("S0")}, False)
    assert all(c.ok for c in ok)
    bad = core.precondition_checks("S1", good_facts(sink_mode="stop_only"), {"S0": ev("S0")}, False)
    assert "sink-mode" in failed(bad)
    assert "ordering" in failed(core.precondition_checks("S2", good_facts(sink_mode="armed"),
                                                        {"S0": ev("S0")}, False))


def test_verdict_rules():
    ok, bad = [core.Check("a", "", True)], [core.Check("b", "", False)]
    assert core.decide_verdict(bad, ok) == "INCOMPLETE"
    assert core.decide_verdict(ok, ok) == "PASS"
    assert core.decide_verdict(ok, bad) == "FAIL"
    assert core.decide_verdict(ok, []) == "FAIL"


def test_confirmation_phrases_are_exact():
    assert core.phrase_ok("S0", "ROBOT STANDING STOP ONLY")
    assert core.phrase_ok("S1", "AREA CLEAR MOVE")
    assert core.phrase_ok("S2", "AREA CLEAR DEADMAN")
    assert not core.phrase_ok("S1", "ROBOT STANDING STOP ONLY")
    assert not core.phrase_ok("S1", "area clear move")
    assert not core.phrase_ok("S1", "AREA CLEAR MOVE ")
    assert not core.phrase_ok("S9", "AREA CLEAR MOVE")


def test_final_line():
    assert core.final_line("S1", "PASS", False) == "STAGE S1: PASS"
    assert core.final_line("S0", "INCOMPLETE", True) == \
        "STAGE S0: INCOMPLETE REHEARSAL: NOT HARDWARE EVIDENCE"


# -- evidence files -----------------------------------------------------------------

def test_read_sink_evidence_missing(tmp_path):
    assert core.read_sink_evidence(str(tmp_path)) == {"S0": None, "S1": None, "S2": None}
    assert core.read_sink_evidence(str(tmp_path / "nope")) == {"S0": None, "S1": None, "S2": None}
    assert core.read_sink_evidence(None) == {"S0": None, "S1": None, "S2": None}


def test_read_sink_evidence_rejects_corrupt_or_wrong_stage(tmp_path):
    (tmp_path / "stage_S0.json").write_text("{not json")
    (tmp_path / "stage_S1.json").write_text(json.dumps(ev("S0")))
    (tmp_path / "stage_S2.json").write_text("[1, 2]")
    assert core.read_sink_evidence(str(tmp_path)) == {"S0": None, "S1": None, "S2": None}


def test_evidence_roundtrip_and_prev_kept(tmp_path):
    checks, m = run("S1")
    e1 = core.build_evidence("S1", "PASS", False, SHA, 1.0, checks, m, {"sink_mode": "armed"})
    core.write_evidence(str(tmp_path), "S1", e1, {"odom": []})
    got = core.read_sink_evidence(str(tmp_path))
    assert got["S0"] is None and got["S2"] is None
    assert got["S1"]["verdict"] == "PASS" and got["S1"]["git_sha"] == SHA
    assert got["S1"]["rehearsal"] is False and got["S1"]["schema"] == core.SCHEMA
    assert {c["id"] for c in got["S1"]["checks"]} >= {"S1-peak", "S1-stopped"}
    for k in ("peak_speed_mps", "stop_latency_s", "deadman_latency_s", "displacement_m",
              "moves_after_stop", "response_codes"):
        assert k in got["S1"]["measurements"]
    assert (tmp_path / "stage_S1_samples.json").exists()
    e2 = core.build_evidence("S1", "FAIL", False, SHA, 2.0, checks, m)
    core.write_evidence(str(tmp_path), "S1", e2)
    assert core.read_sink_evidence(str(tmp_path))["S1"]["verdict"] == "FAIL"
    assert json.loads((tmp_path / "stage_S1.prev.json").read_text())["verdict"] == "PASS"


def test_baseline_excludes_the_sink(tmp_path):
    b = core.build_baseline([core.SINK_NODE, "/zzz", "/aaa"], 5.0, True, SHA)
    assert b == {"publishers": ["/aaa", "/zzz"], "t_wall": 5.0, "rehearsal": True, "git_sha": SHA}
    p = core.write_baseline(str(tmp_path), b)
    assert Path(p).name == "sport_baseline.json" and json.loads(Path(p).read_text()) == b


def test_stage_files_contain_no_em_dash():
    dash = chr(0x2014)
    here = Path(__file__).resolve()
    for name in ("sink_stage_core.py", "sink_stage.py"):
        assert dash not in (here.parents[1] / "riskgraph_nav" / name).read_text(), name
    assert dash not in here.read_text().replace("chr(0x2014)", "")
