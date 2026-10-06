"""Pure logic for the RiskGraph sport sink stage runner. No ROS imports.

The stage runner (sink_stage.py) proves, on the robot, that the RiskGraph sport
sink stops the GO2. It replaces HELIX HW_MOTION_TEST stages D/E as live-trial
preflight evidence. Three stages, each gated by the previous one:

  S0  sink in stop_only: a forward command must produce StopMove only, no Move
  S1  sink armed: a short 0.15 m/s Move, then a zero command, must stop the robot
  S2  sink armed: a short Move, then silence, must trip the deadman StopMove

This module holds everything that can be tested without a robot: the stage
plan, the speed estimate, the pass/fail thresholds applied to recorded
samples, the stage ordering rules, and the evidence files.

Recorded samples (all times are LOCAL time.monotonic() of the runner; the
robot's header stamps are on a skewed clock and are never used):

  odom       [[t_rx, x, y], ...]
  trace      [{... sink trace JSON ..., "t_rx": float}, ...]
  responses  [{"t_rx", "id", "api_id", "code"}, ...]
  cmds       [[t_pub, vx], ...]  every Twist the runner published
  phases     {"t_start", "t_zero" (None in S2), "t_last_cmd", "t_end"}
"""
from __future__ import annotations

import json
import math
import os
from collections import deque
from dataclasses import asdict, dataclass
from typing import Deque, Dict, List, Optional, Tuple

# The ids live only in the sink core (a static guard enforces it).
from riskgraph_nav.sport_sink_core import API_MOVE, API_STOP_MOVE  # noqa: F401

SCHEMA = 1
STAGES = ("S0", "S1", "S2")

SINK_NODE = "/riskgraph_sport_sink"
REHEARSAL_NODE = "/rehearsal_go2"
CMD_TOPIC = "/nav/cmd_vel"
TRACE_TOPIC = "/riskgraph/sink/trace"
REQUEST_TOPIC = "/api/sport/request"
RESPONSE_TOPIC = "/api/sport/response"
ODOM_TOPIC = "/utlidar/robot_odom"

PHRASES = {"S0": "ROBOT STANDING STOP ONLY",
           "S1": "AREA CLEAR MOVE",
           "S2": "AREA CLEAR DEADMAN"}
REQUIRED_MODE = {"S0": "stop_only", "S1": "armed", "S2": "armed"}
PREREQUISITE = {"S0": None, "S1": "S0", "S2": "S1"}

#: Per stage: segments of (duration_s, vx) published at PUBLISH_HZ, then
#: silent_s of publishing nothing, then tail_s of listening only. The tail lets
#: late responses and the 0.3 s stop-hold window complete.
PLAN = {
    "S0": {"segments": [(1.0, 0.10), (1.0, 0.0)], "silent_s": 0.0, "tail_s": 0.7},
    "S1": {"segments": [(2.0, 0.15), (2.0, 0.0)], "silent_s": 0.0, "tail_s": 0.5},
    "S2": {"segments": [(1.5, 0.15)], "silent_s": 2.0, "tail_s": 0.5},
}
PUBLISH_HZ = 20.0

# Thresholds. Field note: a straight 0.15 m/s Move for 2 s peaks at 0.159-0.161.
SPEED_WINDOW_S = 0.2        # pose differencing window for the speed estimate
STILL_MPS = 0.05            # below this the robot counts as stopped / still
HOLD_S = 0.3                # "stopped" means below STILL_MPS for this long
STOP_BY_S = 1.5             # stopped within this long of the zero / last command
PEAK_RANGE_MPS = (0.05, 0.25)
GRACE_S = 0.1               # Moves tolerated this long after zero / last command
DEADMAN_SLACK_S = 0.15      # deadman StopMove by input_timeout_sec + this
ABORT_SPEED_MPS = 0.35
ABORT_DISPLACEMENT_M = 1.0
MIN_ODOM_HZ = 50.0
TRACE_FRESH_S = 1.5


@dataclass
class Check:
    id: str
    desc: str
    ok: bool
    detail: str = ""

    def to_dict(self) -> Dict:
        return asdict(self)


class Checks:
    """Collector in the preflight style: add(id, desc, ok, detail)."""

    def __init__(self) -> None:
        self.items: List[Check] = []

    def add(self, i: str, desc: str, ok: bool, detail: str = "") -> bool:
        self.items.append(Check(i, desc, bool(ok), detail))
        return bool(ok)


# ---------------------------------------------------------------------------
# speed estimate and safety monitor
# ---------------------------------------------------------------------------

def speed_series(odom: List[List[float]], window_s: float = SPEED_WINDOW_S
                 ) -> List[Tuple[float, float]]:
    """(t, speed) by pose differencing over windows of at least ``window_s``.

    The robot's own twist and header stamps are not trusted; positions and the
    local receipt times are. The first ``window_s`` of samples yield no speed.
    """
    out: List[Tuple[float, float]] = []
    j = 0
    for i in range(len(odom)):
        t, x, y = odom[i]
        # newest sample that is at least window_s older than this one
        while j + 1 < i and odom[j + 1][0] <= t - window_s:
            j += 1
        if odom[j][0] <= t - window_s:
            dt = t - odom[j][0]
            out.append((t, math.hypot(x - odom[j][1], y - odom[j][2]) / dt))
    return out


def first_sustained_below(series: List[Tuple[float, float]], thresh: float, hold_s: float,
                          t_from: float) -> Optional[float]:
    """Start time of the first run at or after ``t_from`` that stays below
    ``thresh`` for at least ``hold_s``; None if there is none."""
    run_start: Optional[float] = None
    for t, v in series:
        if t < t_from:
            continue
        if v < thresh:
            if run_start is None:
                run_start = t
            if t - run_start >= hold_s:
                return run_start
        else:
            run_start = None
    return None


def peak_speed(series: List[Tuple[float, float]], t0: float, t1: float) -> Optional[float]:
    v = [s for t, s in series if t0 <= t <= t1]
    return max(v) if v else None


def displacement(odom: List[List[float]], t0: float, t1: float) -> Optional[float]:
    """Largest distance from the first pose inside [t0, t1]."""
    pts = [(x, y) for t, x, y in odom if t0 <= t <= t1]
    if not pts:
        return None
    x0, y0 = pts[0]
    return max(math.hypot(x - x0, y - y0) for x, y in pts)


class SafetyMonitor:
    """Live abort check on the odometry stream: speed or displacement."""

    def __init__(self, window_s: float = SPEED_WINDOW_S) -> None:
        self._w = window_s
        self._buf: Deque[Tuple[float, float, float]] = deque()
        self._origin: Optional[Tuple[float, float]] = None
        self.peak = 0.0
        self.max_disp = 0.0

    def add(self, t: float, x: float, y: float) -> Optional[str]:
        """Feed one pose; returns an abort reason or None."""
        if self._origin is None:
            self._origin = (x, y)
        self._buf.append((t, x, y))
        ref = None
        while len(self._buf) > 1 and self._buf[1][0] <= t - self._w:
            self._buf.popleft()
        if self._buf[0][0] <= t - self._w:
            ref = self._buf[0]
        d = math.hypot(x - self._origin[0], y - self._origin[1])
        self.max_disp = max(self.max_disp, d)
        if ref is not None:
            v = math.hypot(x - ref[1], y - ref[2]) / (t - ref[0])
            self.peak = max(self.peak, v)
            if v > ABORT_SPEED_MPS:
                return f"speed {v:.3f} m/s > {ABORT_SPEED_MPS}"
        if d > ABORT_DISPLACEMENT_M:
            return f"displacement {d:.3f} m > {ABORT_DISPLACEMENT_M}"
        return None


# ---------------------------------------------------------------------------
# trace and response helpers
# ---------------------------------------------------------------------------

def phrase_ok(stage: str, phrase: str) -> bool:
    return stage in PHRASES and phrase == PHRASES[stage]


def _entries(trace: List[Dict], api: int, t0: float, t1: float,
             reason: Optional[str] = None) -> List[Dict]:
    return [e for e in trace
            if e.get("api_id") == api and t0 <= e.get("t_rx", -1.0) <= t1
            and (reason is None or e.get("reason") == reason)]


def _answer(entry: Dict, responses: List[Dict]) -> Tuple[bool, Optional[int]]:
    """(answered with code 0 for this request, the code or None)."""
    if not entry.get("sent_to_robot"):
        return False, None
    for r in responses:
        if r.get("id") == entry.get("request_id") and r.get("api_id") == entry.get("api_id"):
            return r.get("code") == 0, r.get("code")
    return False, None


def _odom_coverage(c: Checks, stage: str, odom: List[List[float]], t0: float, t1: float) -> None:
    n = len([1 for t, _x, _y in odom if t0 <= t <= t1])
    hz = n / (t1 - t0) if t1 > t0 else 0.0
    c.add(f"{stage}-odom", f"odometry covered the stage window at >= {MIN_ODOM_HZ:.0f} Hz",
          hz >= MIN_ODOM_HZ, f"{n} samples over {t1 - t0:.2f} s = {hz:.1f} Hz")


# ---------------------------------------------------------------------------
# stage evaluation (recorded samples -> checks and measurements)
# ---------------------------------------------------------------------------

def evaluate_stage(stage: str, samples: Dict, input_timeout_sec: float = 0.25,
                   abort: Optional[str] = None) -> Tuple[List[Check], Dict]:
    """Apply the stage's pass criteria to recorded samples. Pure."""
    if stage not in STAGES:
        raise ValueError(f"unknown stage {stage!r}")
    ph = samples["phases"]
    odom, trace, resp = samples["odom"], samples["trace"], samples["responses"]
    t0, t_end = ph["t_start"], ph["t_end"]
    series = speed_series(odom)
    c = Checks()
    m: Dict = {"peak_speed_mps": peak_speed(series, t0, t_end),
               "displacement_m": displacement(odom, t0, t_end),
               "stop_latency_s": None, "deadman_latency_s": None,
               "moves_after_stop": None, "response_codes": []}
    c.add(f"{stage}-abort", "no safety abort (speed <= 0.35 m/s, displacement <= 1.0 m, no Ctrl-C)",
          abort is None, abort or "none")
    _odom_coverage(c, stage, odom, t0, t_end)

    if stage == "S0":
        moves = _entries(trace, API_MOVE, t0, t_end)
        m["moves_in_window"] = len(moves)
        m["moves_after_stop"] = len(moves)
        c.add("S0-no-move", "no Move (api 1008) in the trace during the window",
              not moves, f"{len(moves)} Move entries")
        na = _entries(trace, API_STOP_MOVE, t0, t_end, "NOT_ARMED")
        m["stop_not_armed"] = len(na)
        c.add("S0-not-armed", "at least one StopMove with reason NOT_ARMED",
              len(na) >= 1, f"{len(na)} entries")
        pk = m["peak_speed_mps"]
        c.add("S0-still", f"robot speed stays < {STILL_MPS} m/s",
              pk is not None and pk < STILL_MPS, f"peak={pk}")
        stops = [e for e in _entries(trace, API_STOP_MOVE, t0, t_end) if e.get("sent_to_robot")]
        got = [_answer(e, resp) for e in stops]
        m["response_codes"] = [code for _ok, code in got]
        c.add("S0-answered", "at least one StopMove answered with code 0",
              any(ok for ok, _ in got), f"{len(stops)} sent, codes={m['response_codes']}")
        return c.items, m

    t_zero, t_last = ph.get("t_zero"), ph["t_last_cmd"]
    if stage == "S1":
        t_ref = t_zero
        c.add("S1-zero-began", "the runner published a zero command", t_ref is not None)
        key, reason = "stop", "ZERO"
    else:
        t_ref = t_last
        key, reason = "deadman", "DEADMAN"
    if t_ref is None:
        return c.items, m

    pk = m["peak_speed_mps"]
    if stage == "S1":
        lo, hi = PEAK_RANGE_MPS
        c.add("S1-peak", f"peak speed in [{lo}, {hi}] m/s", pk is not None and lo <= pk <= hi,
              f"peak={pk}")
    else:
        m["peak_speed_note"] = "recorded only; S1 gates the peak"
    cmd_moves = _entries(trace, API_MOVE, t0, t_ref)
    c.add(f"{stage}-moved", "the sink sent Move while the command was nonzero",
          len(cmd_moves) >= 1, f"{len(cmd_moves)} Move entries")

    sm = _entries(trace, API_STOP_MOVE, t_ref, t_end, reason)
    first = min(sm, key=lambda e: e["t_rx"]) if sm else None
    if stage == "S1":
        c.add("S1-zero-stop", "trace StopMove reason ZERO after the zero began",
              first is not None, f"{len(sm)} entries")
    else:
        lat = (first["t_rx"] - t_ref) if first else None
        m["deadman_latency_s"] = lat
        lim = input_timeout_sec + DEADMAN_SLACK_S
        c.add("S2-deadman", f"DEADMAN StopMove within input_timeout_sec + {DEADMAN_SLACK_S} s "
              f"({lim:.2f} s) of the last command", lat is not None and lat <= lim,
              f"latency={lat}")

    stop_t = first_sustained_below(series, STILL_MPS, HOLD_S, t_ref)
    sl = (stop_t - t_ref) if stop_t is not None else None
    m["stop_latency_s"] = sl
    what = "first zero" if stage == "S1" else "last command"
    c.add(f"{stage}-stopped", f"speed < {STILL_MPS} m/s sustained {HOLD_S} s, reached within "
          f"{STOP_BY_S} s of the {what}", sl is not None and sl <= STOP_BY_S, f"latency={sl}")

    late = _entries(trace, API_MOVE, t_ref + GRACE_S, t_end)
    m["moves_after_stop"] = len(late)
    c.add(f"{stage}-no-late-move", f"no Move in the trace after the {what} + {GRACE_S} s",
          not late, f"{len(late)} Move entries")

    ok, code = _answer(first, resp) if first is not None else (False, None)
    m["response_codes"] = [code] if first is not None else []
    c.add(f"{stage}-answered", f"the {key} StopMove was answered with code 0", ok,
          f"code={code}" if first is not None else "no StopMove to answer")
    return c.items, m


# ---------------------------------------------------------------------------
# preconditions and ordering
# ---------------------------------------------------------------------------

def check_ordering(stage: str, evidence: Dict[str, Optional[Dict]], git_sha: str,
                   rehearsal: bool) -> Check:
    """A stage unlocks only on the previous stage's PASS, same sha, same
    rehearsal flag. A rehearsal session never unlocks a live stage and vice versa."""
    prev = PREREQUISITE[stage]
    if prev is None:
        return Check("ordering", f"{stage} has no prerequisite stage", True)
    desc = f"stage_{prev}.json is PASS at this git sha with the same rehearsal flag"
    ev = evidence.get(prev)
    if ev is None:
        return Check("ordering", desc, False, f"stage_{prev}.json missing or unreadable")
    if ev.get("verdict") != "PASS":
        return Check("ordering", desc, False, f"stage_{prev} verdict={ev.get('verdict')}")
    if not git_sha or ev.get("git_sha") != git_sha:
        return Check("ordering", desc, False,
                     f"git sha mismatch: evidence={ev.get('git_sha')} now={git_sha}")
    if bool(ev.get("rehearsal")) != bool(rehearsal):
        return Check("ordering", desc, False,
                     f"rehearsal flag mismatch: evidence={bool(ev.get('rehearsal'))} "
                     f"now={bool(rehearsal)}")
    return Check("ordering", desc, True, f"stage_{prev} PASS sha={git_sha[:10]}")


def precondition_checks(stage: str, facts: Dict, evidence: Dict[str, Optional[Dict]],
                        rehearsal: bool) -> List[Check]:
    """Every check must be ok before anything is published. Pure: facts in.

    facts: nodes, sink_mode, input_timeout_sec, cmd_vel_publishers (nodes other
    than the runner), sink_subscribed, odom_hz, odom_peak_speed, trace_age_s,
    git {sha, dirty, porcelain}.
    """
    c = Checks()
    nodes = facts.get("nodes") or []
    c.add("sink-node", f"{SINK_NODE} is present in the graph", SINK_NODE in nodes,
          f"{len(nodes)} nodes")
    want = REQUIRED_MODE[stage]
    c.add("sink-mode", f"sink mode parameter is {want} for {stage}",
          facts.get("sink_mode") == want, f"mode={facts.get('sink_mode')}")
    c.add("rehearsal-flag", "--rehearsal matches the graph (a rehearsal robot node is present "
          "iff --rehearsal)", (REHEARSAL_NODE in nodes) == bool(rehearsal),
          f"{REHEARSAL_NODE} present={REHEARSAL_NODE in nodes} --rehearsal={bool(rehearsal)}")
    pubs = facts.get("cmd_vel_publishers")
    c.add("cmd-vel-clear", f"no publisher on {CMD_TOPIC} other than this runner (Nav2 not running)",
          pubs is not None and not pubs, f"others={pubs}")
    c.add("sink-subscribed", f"the sink is subscribed to {CMD_TOPIC}",
          bool(facts.get("sink_subscribed")), "")
    hz = facts.get("odom_hz") or 0.0
    c.add("odom-rate", f"odometry >= {MIN_ODOM_HZ:.0f} Hz", hz >= MIN_ODOM_HZ, f"{hz:.1f} Hz")
    pk = facts.get("odom_peak_speed")
    c.add("robot-still", f"robot still (< {STILL_MPS} m/s)", pk is not None and pk < STILL_MPS,
          f"peak={pk}")
    age = facts.get("trace_age_s")
    c.add("trace-fresh", f"sink trace fresh (< {TRACE_FRESH_S} s)",
          age is not None and age < TRACE_FRESH_S, f"age={age}")
    g = facts.get("git") or {}
    c.add("git-clean", "git tree clean (evidence requires it)",
          bool(g.get("sha")) and not g.get("dirty", True),
          f"sha={g.get('sha')} dirty={g.get('dirty')} {str(g.get('porcelain', ''))[:200]}")
    c.items.append(check_ordering(stage, evidence, g.get("sha", ""), rehearsal))
    return c.items


def decide_verdict(pre: List[Check], stage_checks: List[Check]) -> str:
    if any(not x.ok for x in pre):
        return "INCOMPLETE"
    return "PASS" if stage_checks and all(x.ok for x in stage_checks) else "FAIL"


def final_line(stage: str, verdict: str, rehearsal: bool) -> str:
    s = f"STAGE {stage}: {verdict}"
    return s + " REHEARSAL: NOT HARDWARE EVIDENCE" if rehearsal else s


# ---------------------------------------------------------------------------
# evidence files
# ---------------------------------------------------------------------------

def _write_json(path: str, obj) -> None:
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f, indent=2, sort_keys=True)
        f.write("\n")
    os.replace(tmp, path)


def evidence_path(session_dir: str, stage: str) -> str:
    return os.path.join(session_dir, f"stage_{stage}.json")


def build_evidence(stage: str, verdict: str, rehearsal: bool, git_sha: str, t_wall: float,
                   checks: List[Check], measurements: Dict, params: Optional[Dict] = None) -> Dict:
    return {"schema": SCHEMA, "stage": stage, "verdict": verdict, "rehearsal": bool(rehearsal),
            "git_sha": git_sha, "t_wall": t_wall, "params": params or {},
            "checks": [x.to_dict() for x in checks], "measurements": measurements}


def write_evidence(session_dir: str, stage: str, evidence: Dict,
                   samples: Optional[Dict] = None) -> str:
    """Write stage_<S>.json (and stage_<S>_samples.json). The previous evidence
    for the stage is kept as stage_<S>.prev.json so a failed re-run does not
    silently destroy an earlier record."""
    os.makedirs(session_dir, exist_ok=True)
    p = evidence_path(session_dir, stage)
    if os.path.exists(p):
        os.replace(p, os.path.join(session_dir, f"stage_{stage}.prev.json"))
    _write_json(p, evidence)
    if samples is not None:
        _write_json(os.path.join(session_dir, f"stage_{stage}_samples.json"), samples)
    return p


def build_baseline(publishers: List[str], t_wall: float, rehearsal: bool, git_sha: str) -> Dict:
    """Nodes publishing /api/sport/request other than the RiskGraph sink, taken
    before the stage publishes anything."""
    return {"publishers": sorted(p for p in publishers if p != SINK_NODE), "t_wall": t_wall,
            "rehearsal": bool(rehearsal), "git_sha": git_sha}


def write_baseline(session_dir: str, baseline: Dict) -> str:
    os.makedirs(session_dir, exist_ok=True)
    p = os.path.join(session_dir, "sport_baseline.json")
    _write_json(p, baseline)
    return p


def read_sink_evidence(session_dir: Optional[str]) -> Dict[str, Optional[Dict]]:
    """{"S0": ev|None, "S1": ..., "S2": ...}. None when the file is missing,
    unreadable, not a JSON object, or names a different stage."""
    out: Dict[str, Optional[Dict]] = {}
    for s in STAGES:
        ev = None
        if session_dir:
            try:
                with open(evidence_path(session_dir, s)) as f:
                    ev = json.load(f)
            except (OSError, ValueError):
                ev = None
        out[s] = ev if isinstance(ev, dict) and ev.get("stage") == s else None
    return out
