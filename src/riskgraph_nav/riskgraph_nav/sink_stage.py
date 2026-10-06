"""On-robot stage runner: proves the RiskGraph sport sink stops the GO2.

    ros2 run riskgraph_nav riskgraph_sink_stage --stage S0 \\
        --session-dir ~/riskgraph_hw/<date>_sink --repo ~/riskgraph-go2 \\
        --confirm "ROBOT STANDING STOP ONLY"

Run stages in order S0, S1, S2 (each unlocks the next, see sink_stage_core).
The runner publishes a short Twist on /nav/cmd_vel, which only the RiskGraph
sport sink listens to, and records the sink's trace, the robot's responses
and the robot's odometry. It never publishes to the robot directly.
Nav2 must not be running. All pass/fail logic is in sink_stage_core.

Odometry header stamps are on a skewed robot clock: everything here uses local
receipt time and pose differencing.
"""
from __future__ import annotations

import argparse
import json
import signal
import sys
import threading
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from riskgraph_nav import sink_stage_core as core
from riskgraph_nav.preflight import git_info
from riskgraph_nav.ros_graph import SENSOR, GraphProbe

NODE_NAME = "riskgraph_sink_stage"
NODE_FQ = "/" + NODE_NAME
CMD_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                     reliability=ReliabilityPolicy.RELIABLE)
TRACE_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                       reliability=ReliabilityPolicy.RELIABLE)
# Best effort so it matches whatever the robot publishes responses with.
RESP_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50,
                      reliability=ReliabilityPolicy.BEST_EFFORT)


class Recorder:
    """Subscribes to odometry, sink trace and robot responses; stamps every
    message with local time.monotonic(). Also runs the live safety monitor."""

    def __init__(self, node, response_cls) -> None:
        self.lock = threading.Lock()
        self.odom: List[List[float]] = []
        self.trace: List[Dict] = []
        self.responses: List[Dict] = []
        self.monitor: Optional[core.SafetyMonitor] = None
        self.abort: Optional[str] = None
        node.create_subscription(Odometry, core.ODOM_TOPIC, self._on_odom, SENSOR)
        node.create_subscription(String, core.TRACE_TOPIC, self._on_trace, TRACE_QOS)
        node.create_subscription(response_cls, core.RESPONSE_TOPIC, self._on_resp, RESP_QOS)

    def _on_odom(self, m) -> None:
        t = time.monotonic()
        p = m.pose.pose.position
        with self.lock:
            self.odom.append([t, p.x, p.y])
            mon = self.monitor
            if mon is not None and self.abort is None:
                self.abort = mon.add(t, p.x, p.y)

    def _on_trace(self, m) -> None:
        t = time.monotonic()
        try:
            e = json.loads(m.data)
        except ValueError:
            return
        if isinstance(e, dict):
            e["t_rx"] = t
            with self.lock:
                self.trace.append(e)

    def _on_resp(self, m) -> None:
        t = time.monotonic()
        with self.lock:
            self.responses.append({"t_rx": t, "id": int(m.header.identity.id),
                                   "api_id": int(m.header.identity.api_id),
                                   "code": int(m.header.status.code)})

    def arm_monitor(self) -> None:
        with self.lock:
            self.abort = None
            self.monitor = core.SafetyMonitor()

    def snapshot(self) -> Dict:
        with self.lock:
            return {"odom": list(self.odom), "trace": list(self.trace),
                    "responses": list(self.responses)}


def gather_facts(probe, rec: Recorder, repo: str, listen_s: float = 1.3) -> Dict:
    """Read the graph and the recorded streams. Publishes nothing."""
    time.sleep(listen_s)
    f: Dict = {"nodes": probe.nodes()}
    params = probe.params(core.SINK_NODE, ["mode", "input_timeout_sec"]) or {}
    f["sink_mode"] = params.get("mode")
    f["input_timeout_sec"] = params.get("input_timeout_sec")
    f["cmd_vel_publishers"] = sorted(p["node"] for p in probe.publishers(core.CMD_TOPIC)
                                     if p["node"] != NODE_FQ)
    f["sink_subscribed"] = any(s["node"] == core.SINK_NODE
                               for s in probe.subscribers(core.CMD_TOPIC))
    f["sport_publishers"] = sorted(p["node"] for p in probe.publishers(core.REQUEST_TOPIC))
    now = time.monotonic()
    snap = rec.snapshot()
    recent = [o for o in snap["odom"] if o[0] >= now - 1.0]
    f["odom_hz"] = float(len(recent))
    ser = core.speed_series(recent)
    f["odom_peak_speed"] = max((v for _t, v in ser), default=None)
    f["trace_age_s"] = (now - snap["trace"][-1]["t_rx"]) if snap["trace"] else None
    f["git"] = git_info(repo)
    return f


def _sleep_until(t: float) -> None:
    d = t - time.monotonic()
    if d > 0:
        time.sleep(d)


def drive(stage: str, pub, rec: Recorder, interrupted: threading.Event) -> Dict:
    """Run the stage's command plan. On abort or Ctrl-C: zero Twist burst."""
    plan = core.PLAN[stage]
    cmds: List[List[float]] = []
    ph: Dict = {"t_start": None, "t_zero": None, "t_last_cmd": None, "t_end": None}
    abort: Optional[str] = None
    period = 1.0 / core.PUBLISH_HZ

    def publish(vx: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        pub.publish(msg)
        t = time.monotonic()
        cmds.append([t, float(vx)])
        ph["t_last_cmd"] = t
        if ph["t_start"] is None:
            ph["t_start"] = t
        if vx == 0.0 and ph["t_zero"] is None:
            ph["t_zero"] = t

    def tripped() -> Optional[str]:
        if interrupted.is_set():
            return "interrupted (Ctrl-C)"
        return rec.abort

    def zero_burst() -> None:
        burst["done"] = True
        for _ in range(5):
            publish(0.0)
            time.sleep(0.02)

    burst = {"done": False}
    rec.arm_monitor()
    try:
        nxt = time.monotonic()
        for dur, vx in plan["segments"]:
            for _ in range(int(round(dur * core.PUBLISH_HZ))):
                abort = tripped()
                if abort:
                    break
                publish(vx)
                nxt += period
                _sleep_until(nxt)
            if abort:
                break
        if not abort and plan["silent_s"] > 0:
            end = time.monotonic() + plan["silent_s"]
            while time.monotonic() < end and not abort:
                time.sleep(0.01)
                abort = tripped()
        if abort:
            zero_burst()
        end = time.monotonic() + plan["tail_s"]
        while time.monotonic() < end:
            time.sleep(0.02)
            abort = abort or tripped()
        abort = abort or tripped()
        if abort and not burst["done"]:
            zero_burst()
    except Exception as e:  # never leave a nonzero command standing
        abort = f"exception: {e!r}"
        zero_burst()
    ph["t_end"] = time.monotonic()
    return {"phases": ph, "cmds": cmds, "abort": abort}


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="RiskGraph sport sink stage runner (S0, S1, S2)")
    ap.add_argument("--stage", required=True, choices=core.STAGES)
    ap.add_argument("--session-dir", required=True)
    ap.add_argument("--repo", required=True, help="git repo whose HEAD is recorded; must be clean")
    ap.add_argument("--rehearsal", action="store_true",
                    help="off-robot run against rehearsal_go2; evidence is NOT hardware evidence")
    ap.add_argument("--confirm", required=True, help="the exact phrase for this stage")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    a = parse_args(argv)
    if not core.phrase_ok(a.stage, a.confirm):
        print(f"REFUSED: --confirm for {a.stage} must be exactly "
              f"\"{core.PHRASES[a.stage]}\". Nothing was published.", file=sys.stderr)
        return 2

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    interrupted = threading.Event()
    signal.signal(signal.SIGINT, lambda s, f: interrupted.set())
    signal.signal(signal.SIGTERM, lambda s, f: interrupted.set())
    probe = GraphProbe(NODE_NAME)
    rc = 2
    try:
        from unitree_api.msg import Response
        # The publisher is created first so the sink's subscription can be
        # checked, and so "other publishers" excludes only this runner.
        # Creating it publishes nothing.
        pub = probe.node.create_publisher(Twist, core.CMD_TOPIC, CMD_QOS)
        rec = Recorder(probe.node, Response)
        facts = gather_facts(probe, rec, a.repo)
        facts["sink_subscribed"] = facts["sink_subscribed"] and pub.get_subscription_count() >= 1
        evidence_in = core.read_sink_evidence(a.session_dir)
        pre = core.precondition_checks(a.stage, facts, evidence_in, a.rehearsal)
        if interrupted.is_set():
            pre.append(core.Check("interrupted", "no Ctrl-C before the stage started", False))
        sha = (facts.get("git") or {}).get("sha", "")
        params = {"sink_mode": facts.get("sink_mode"),
                  "input_timeout_sec": facts.get("input_timeout_sec")}
        samples: Optional[Dict] = None
        stage_checks: List[core.Check] = []
        meas: Dict = {}
        if all(c.ok for c in pre):
            if a.stage == "S0":  # taken before anything is published
                core.write_baseline(a.session_dir, core.build_baseline(
                    facts["sport_publishers"], time.time(), a.rehearsal, sha))
            print(f"STAGE {a.stage}: preconditions OK, publishing on {core.CMD_TOPIC}", flush=True)
            run = drive(a.stage, pub, rec, interrupted)
            samples = {**rec.snapshot(), "phases": run["phases"], "cmds": run["cmds"],
                       "abort": run["abort"]}
            stage_checks, meas = core.evaluate_stage(
                a.stage, samples, float(facts.get("input_timeout_sec") or 0.25), run["abort"])
        verdict = core.decide_verdict(pre, stage_checks)
        ev = core.build_evidence(a.stage, verdict, a.rehearsal, sha, time.time(),
                                 pre + stage_checks, meas, params)
        path = core.write_evidence(a.session_dir, a.stage, ev, samples)
        for c in pre + stage_checks:
            print(f"  [{'ok' if c.ok else 'FAIL'}] {c.id}: {c.desc} ({c.detail})")
        print(f"evidence: {path}")
        print(core.final_line(a.stage, verdict, a.rehearsal))
        rc = 0 if verdict == "PASS" else (1 if verdict == "FAIL" else 2)
    finally:
        probe.close()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
