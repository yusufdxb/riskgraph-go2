"""Hardware preflight for the live RiskGraph GO2 trial. Publishes nothing.

    ros2 run riskgraph_nav riskgraph_preflight --mode live \\
        --sink-session ~/riskgraph_sink/<date> [--riskgraph expect-absent]

Two halves:

* :func:`collect` reads the machine and the live ROS graph into a plain
  ``facts`` dict (git, environment, packages, disk, processes, topics and
  rates, TF, lifecycle states, parameters, RiskGraph / localization status,
  sport-sink state and its on-robot stage evidence).
* :func:`evaluate` is pure: facts in, checks out. Every check is PASS, WARN,
  FAIL or INFO. **Any FAIL is NO-GO.** It is unit tested on synthetic facts.

A GO does not mark anything as hardware verified. It says the setup is fit
to attempt the trial.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import time
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional

from .sport_sink_core import API_STOP_MOVE, NODE_NAME as SINK_NODE_NAME, TRACE_TOPIC as SINK_TRACE

PASS, WARN, FAIL, INFO = "PASS", "WARN", "FAIL", "INFO"
SINK_STAGES = ("S0", "S1", "S2")

ODOM_TOPIC = "/utlidar/robot_odom"
NAV_NODES = ["/map_server", "/planner_server", "/controller_server", "/velocity_smoother"]
RG_NODES = ["/riskgraph_memory", "/riskgraph_planner", "/riskgraph_explainer"]
LOC_NODE = "/riskgraph_localization"
SINK = "/" + SINK_NODE_NAME
GLOBAL_COSTMAP = "/global_costmap/global_costmap"
REQUIRED_PKGS = ["nav2_map_server", "nav2_planner", "nav2_navfn_planner", "nav2_controller",
                 "nav2_regulated_pure_pursuit_controller", "nav2_velocity_smoother",
                 "nav2_lifecycle_manager", "nav2_costmap_2d", "nav2_msgs", "rosbag2",
                 "tf2_ros", "riskgraph_msgs", "riskgraph_bringup", "riskgraph_nav"]
LIVE_PKGS = ["unitree_api"]
MOTION_TYPES = {"geometry_msgs/msg/Twist", "geometry_msgs/msg/TwistStamped",
                "unitree_api/msg/Request"}


@dataclass
class Config:
    mode: str = "live"                      # live | rehearsal
    riskgraph: str = "expect-running"       # expect-running | expect-absent
    expected_branch: Optional[str] = None
    expected_iface: str = "enP8p1s0"
    store_path: str = ""
    map_id: str = ""
    evidence_root: str = ""
    min_free_gb: float = 2.0
    min_odom_hz: float = 50.0
    max_odom_age_s: float = 0.25
    sink_limits: Dict[str, float] = field(default_factory=lambda: {"max_vx": 0.25, "max_wz": 0.50})
    sink_session: Optional[str] = None
    sport_baseline: Optional[List[str]] = None


@dataclass
class Check:
    id: str
    title: str
    result: str
    detail: str = ""


# ---------------------------------------------------------------------------
# evaluation (pure)
# ---------------------------------------------------------------------------

def _b(ok: bool, soft: bool = False) -> str:
    return PASS if ok else (WARN if soft else FAIL)


def evaluate(f: Dict, cfg: Config) -> List[Check]:
    live = cfg.mode == "live"
    out: List[Check] = []

    def add(i, title, result, detail=""):
        out.append(Check(i, title, result, detail))

    g = f.get("git", {})
    add("P01", "git tree clean (evidence requires it)",
        _b(not g.get("dirty", True), soft=not live),
        f"sha={g.get('sha')} dirty={g.get('dirty')} {g.get('porcelain', '')[:200]}")
    if cfg.expected_branch:
        add("P02", f"on expected branch {cfg.expected_branch}",
            _b(g.get("branch") == cfg.expected_branch), f"branch={g.get('branch')}")
    else:
        add("P02", "branch recorded", INFO, f"branch={g.get('branch')} (no --expected-branch)")

    env = f.get("env", {})
    add("P03", "ROS_DISTRO is humble", _b(env.get("ROS_DISTRO") == "humble"), str(env.get("ROS_DISTRO")))
    add("P04", "RMW is rmw_cyclonedds_cpp (the GO2 stack's RMW)",
        _b(env.get("RMW_IMPLEMENTATION") == "rmw_cyclonedds_cpp", soft=not live),
        str(env.get("RMW_IMPLEMENTATION")))
    add("P05", "ROS domain recorded; not localhost-only",
        _b(env.get("ROS_LOCALHOST_ONLY") not in ("1",), soft=not live),
        f"ROS_DOMAIN_ID={env.get('ROS_DOMAIN_ID')} ROS_LOCALHOST_ONLY={env.get('ROS_LOCALHOST_ONLY')}")
    net = f.get("net", {})
    if live:
        ok = cfg.expected_iface in net.get("uri_ifaces", []) and net.get("iface_up", False)
        add("P06", f"CycloneDDS bound to {cfg.expected_iface} and it is UP", _b(ok),
            f"uri ifaces={net.get('uri_ifaces')} {cfg.expected_iface} up={net.get('iface_up')}")
    else:
        add("P06", "DDS interface", INFO, f"uri ifaces={net.get('uri_ifaces')}")
    c = f.get("clock", {})
    add("P07", "local clock sane (after HEAD commit, year >= 2026)",
        _b(c.get("now", 0) >= max(c.get("head_commit_time", 0), 1767225600)),
        f"now={c.get('now')} head={c.get('head_commit_time')}")

    pk = f.get("packages", {})
    missing = [p for p in REQUIRED_PKGS + (LIVE_PKGS if live else []) if not pk.get(p)]
    add("P08", "required ROS packages installed", _b(not missing), f"missing={missing}")
    disk = f.get("disk", {})
    add("P09", f"disk free >= {cfg.min_free_gb} GB for bag + DB",
        _b(disk.get("free_gb", 0) >= cfg.min_free_gb), f"{disk}")
    pr = f.get("processes", {})
    add("P10", "no rosbag replay and no /clock (hardware data only)",
        _b(not pr.get("bag_play") and not f.get("clock_publishers")),
        f"bag play procs={pr.get('bag_play')} /clock pubs={f.get('clock_publishers')}")
    sim_nodes = [n for n, v in (f.get("use_sim_time") or {}).items() if v]
    add("P11", "use_sim_time false on RiskGraph, localization and Nav2 nodes", _b(not sim_nodes),
        f"sim time on: {sim_nodes}")
    add("P12", "no duplicate node names", _b(not f.get("duplicate_nodes")), str(f.get("duplicate_nodes")))
    add("P13", "no second trial runner running", _b(not pr.get("trial_runner")), str(pr.get("trial_runner")))

    o = f.get("odom", {})
    odom_ok = (o.get("type") == "nav_msgs/msg/Odometry" and o.get("rate_hz", 0) >= cfg.min_odom_hz
               and o.get("frame") == "odom" and o.get("child") == "base_link")
    add("P14", f"GO2 odometry flowing ({ODOM_TOPIC} >= {cfg.min_odom_hz} Hz, odom->base_link)",
        _b(odom_ok), f"{o}")
    if live:
        s = f.get("sportmodestate", {})
        add("P15", "GO2 /sportmodestate flowing (robot state)", _b(s.get("rate_hz", 0) >= 100.0,
            soft=s.get("type_unavailable", False)), f"{s}")
        add("P16", "/api/sport/response has a publisher (robot answers the sport API)",
            _b(bool(f.get("sport_response_publishers"))), str(f.get("sport_response_publishers")))

    loc = f.get("localization") or {}
    add("P17", "localization alive, anchored, valid",
        _b(bool(loc) and loc.get("localization_valid") is True and loc.get("anchored") is True),
        f"state={loc.get('state')} anchored={loc.get('anchored')} rate={loc.get('odom_rate_hz')} "
        f"jumps={loc.get('odom_jumps')} skew={loc.get('robot_clock_skew_s')}")
    add("P18", "localization map_id matches the experiment",
        _b(loc.get("map_id") == cfg.map_id), f"loc={loc.get('map_id')} expected={cfg.map_id}")
    tf = f.get("tf", {})
    add("P19", "TF map->odom->base_link resolves; odom->base_link fresh",
        _b(tf.get("map_base_ok") and tf.get("odom_base_age_s") is not None
           and tf["odom_base_age_s"] <= 0.5),
        f"{tf}")
    tf_pubs = set(f.get("tf_publishers", [])) | set(f.get("tf_static_publishers", []))
    add("P20", "only riskgraph_localization publishes TF (no conflicting odom/map source)",
        _b(tf_pubs <= {LOC_NODE}), f"/tf+/tf_static publishers={sorted(tf_pubs)}")

    ls = f.get("lifecycle", {})
    inactive = [n for n in NAV_NODES if ls.get(n) != "active"]
    add("P21", "Nav2 servers ACTIVE", _b(not inactive), f"{ls}")
    acts = f.get("actions", {})
    add("P22", "Nav2 actions available (compute_path_to_pose, follow_path)",
        _b(acts.get("compute_path_to_pose") and acts.get("follow_path")), f"{acts}")
    add("P23", "/nav/cmd_vel published only by velocity_smoother",
        _b(f.get("nav_cmd_vel_publishers") == ["/velocity_smoother"]),
        str(f.get("nav_cmd_vel_publishers")))
    add("P24", "global costmap subscribes the RiskGraph risk layer",
        _b(GLOBAL_COSTMAP in (f.get("risk_costmap_subscribers") or [])),
        f"subs of /riskgraph/risk_costmap={f.get('risk_costmap_subscribers')}")
    vs = f.get("velocity_smoother_params") or {}
    mx = vs.get("max_velocity") or [9, 9, 9]
    mn = vs.get("min_velocity") or [-9, -9, -9]
    sink = f.get("sink_params") or {}
    lim_vx = float(sink.get("max_vx", cfg.sink_limits["max_vx"]))
    lim_wz = float(sink.get("max_wz", cfg.sink_limits["max_wz"]))
    add("P25", "Nav2 velocity limits strictly inside the sport sink limits (no silent StopMove)",
        _b(mx[0] < lim_vx and abs(mn[0]) < lim_vx and mx[2] < lim_wz and abs(mn[2]) < lim_wz
           and mn[0] >= 0.0),
        f"smoother max={mx} min={mn} sink vx<{lim_vx} wz<{lim_wz}")

    rg = f.get("riskgraph_nodes", [])
    st = f.get("riskgraph_status") or {}
    if cfg.riskgraph == "expect-absent":
        add("P26", "no RiskGraph node running yet (the trial runner launches it)",
            _b(not rg), f"running={rg}")
        db = f.get("db") or {}
        ok = db.get("exists") is False or (db.get("map_id") in (None, cfg.map_id)
                                           and db.get("evidence_class") in (None, _ev_class(cfg)))
        add("P27", "database path writable; existing DB (if any) matches map and class",
            _b(ok and db.get("dir_writable", False)), f"{db}")
    else:
        add("P26", "exactly one of each RiskGraph node", _b(sorted(rg) == sorted(RG_NODES)), f"{rg}")
        paths = f.get("riskgraph_store_paths") or {}
        same = len(set(paths.values())) == 1 and cfg.store_path in paths.values()
        add("P27", "all RiskGraph nodes use the SAME absolute database",
            _b(same and bool(st) and st.get("db_path") == cfg.store_path
               and os.path.isabs(st.get("db_path", ""))),
            f"expected={cfg.store_path} params={paths} status={st.get('db_path')}")
        fresh = st.get("stamp") is not None and time.time() - float(st["stamp"]) < 5.0
        add("P28", "RiskGraph status fresh; schema v2; map id and evidence class match",
            _b(fresh and st.get("schema_version") == 2 and st.get("map_id") == cfg.map_id
               and st.get("evidence_class") == _ev_class(cfg) and not st.get("health")),
            f"map={st.get('map_id')} class={st.get('evidence_class')} health={st.get('health')} "
            f"incidents={st.get('incident_count')} grid={st.get('grid_state')}")
        add("P29", "risk grid published and valid (never lethal)",
            _b(st.get("grid_state") == "OK" and (st.get("grid_stats") or {}).get("out_of_range_cells", 1) == 0),
            f"{st.get('grid_stats')}")

    rg_motion = {n: t for n, t in (f.get("riskgraph_motion_topics") or {}).items() if t}
    add("P30", "RiskGraph/localization publish NO velocity or sport command", _b(not rg_motion),
        f"{rg_motion}")
    cmd_pubs = f.get("cmd_vel_publishers", [])
    foreign = f.get("foreign_sinks", [])
    add("P31", "/cmd_vel unused and no other sport sink running (one motion exit)",
        _b(not cmd_pubs and not foreign), f"/cmd_vel publishers={cmd_pubs} other sinks={foreign}")
    add("P32", "riskgraph_sport_sink consumes /nav/cmd_vel",
        _b(SINK in (f.get("nav_cmd_vel_subscribers") or [])), str(f.get("nav_cmd_vel_subscribers")))
    req = f.get("sport_request_publishers", [])
    if cfg.sport_baseline is not None:
        extra = [n for n in req if n not in cfg.sport_baseline and n != SINK]
        add("P33", "no /api/sport/request publisher beyond the sink stage S0 baseline + sink",
            _b(not extra), f"extra={extra}")
    else:
        add("P33", "/api/sport/request baseline", FAIL if live else INFO,
            "no S0 baseline (sink session sport_baseline.json); competing robot-side "
            "command sources are unverifiable" if live else f"publishers={req}")
    add("P34", "no twist_mux (a second arbiter)", _b(not f.get("twist_mux_nodes")), str(f.get("twist_mux_nodes")))
    sk = f.get("sink") or {}
    add("P35", "sport sink alive (trace fresh) and holding zero (last decision StopMove)",
        _b(sk.get("fresh") and sk.get("api_id") == API_STOP_MOVE), f"{sk}")
    add("P36", "sport sink ARMED (the only path Nav2 motion can take)",
        _b((f.get("sink_params") or {}).get("mode") == "armed"), f"{f.get('sink_params')}")
    ev = f.get("sink_evidence") or {}
    head = (f.get("git") or {}).get("sha")
    ok_ev = all((ev.get(s) or {}).get("verdict") == "PASS" and
                ((ev.get(s) or {}).get("rehearsal") is False or not live) and
                (ev.get(s) or {}).get("git_sha") == head for s in SINK_STAGES)
    add("P37", "sink stop proven on this robot at this SHA (stages S0, S1, S2 PASS)",
        _b(ok_ev), f"head={str(head)[:7]} " + str({s: {k: (ev.get(s) or {}).get(k) for k in
                                                   ('verdict', 'rehearsal', 'git_sha')} for s in SINK_STAGES}))
    spd = loc.get("speed_mps")
    add("P38", "robot stationary at preflight", _b(spd is not None and spd < 0.05), f"speed={spd}")
    return out


def _ev_class(cfg: Config) -> str:
    return "live" if cfg.mode == "live" else "rehearsal"


def verdict(checks: List[Check]) -> str:
    return "NO-GO" if any(c.result == FAIL for c in checks) else "GO"


# ---------------------------------------------------------------------------
# collection (reads the machine and the graph)
# ---------------------------------------------------------------------------

def git_info(repo: str) -> Dict[str, object]:
    def g(*a):
        r = subprocess.run(["git", "-C", repo, *a], capture_output=True, text=True)
        return r.stdout.strip() if r.returncode == 0 else ""
    por = g("status", "--porcelain")
    return {"repo": repo, "sha": g("rev-parse", "HEAD"), "branch": g("rev-parse", "--abbrev-ref", "HEAD"),
            "dirty": bool(por), "porcelain": por,
            "head_commit_time": int(g("log", "-1", "--format=%ct") or 0),
            "describe": g("describe", "--always", "--dirty")}


def cyclone_ifaces() -> List[str]:
    uri = os.environ.get("CYCLONEDDS_URI", "")
    text = uri
    if uri.startswith("file://"):
        try:
            text = open(uri[len("file://"):]).read()
        except OSError:
            return []
    return re.findall(r'NetworkInterface\s+name="([^"]+)"', text) + \
        re.findall(r"<NetworkInterfaceAddress>([^<]+)<", text)


def iface_up(name: str) -> bool:
    try:
        r = subprocess.run(["ip", "-brief", "link", "show", name], capture_output=True, text=True)
        return r.returncode == 0 and (" UP " in f" {r.stdout} " or "UNKNOWN" in r.stdout)
    except OSError:
        return False


def pkg_installed(name: str) -> bool:
    try:
        from ament_index_python.packages import get_package_prefix
        get_package_prefix(name)
        return True
    except Exception:
        return False


def processes() -> Dict[str, List[str]]:
    try:
        out = subprocess.run(["ps", "-eo", "pid,args"], capture_output=True, text=True).stdout
    except OSError:
        out = ""
    mine = {str(os.getpid()), str(os.getppid())}  # this process and its `ros2 run` parent
    lines = [ln.strip() for ln in out.splitlines()[1:]]
    return {
        "bag_play": [ln for ln in lines if ("bag play" in ln or "rosbag2_player" in ln)],
        "trial_runner": [ln for ln in lines if "riskgraph_live_trial" in ln
                         and ln.split(" ", 1)[0] not in mine and "preflight" not in ln],
    }


def read_sink_stage_evidence(session: Optional[str]) -> Dict[str, Optional[dict]]:
    from .sink_stage_core import read_sink_evidence
    ev = read_sink_evidence(session) if session else {}
    return {s: ({k: (ev.get(s) or {}).get(k) for k in ("verdict", "rehearsal", "git_sha")}
                if ev.get(s) else None) for s in SINK_STAGES}


def collect(probe, cfg: Config, repo: str, listen_s: float = 2.0) -> Dict:
    f: Dict = {}
    f["host"] = socket.gethostname()
    f["git"] = git_info(repo)
    f["env"] = {k: os.environ.get(k) for k in ("ROS_DISTRO", "RMW_IMPLEMENTATION", "ROS_DOMAIN_ID",
                                                "ROS_LOCALHOST_ONLY", "CYCLONEDDS_URI")}
    ifs = cyclone_ifaces()
    f["net"] = {"uri_ifaces": ifs, "iface_up": iface_up(cfg.expected_iface)}
    f["clock"] = {"now": time.time(), "head_commit_time": f["git"]["head_commit_time"]}
    f["packages"] = {p: pkg_installed(p) for p in REQUIRED_PKGS + LIVE_PKGS}
    root = cfg.evidence_root or os.path.expanduser("~")
    os.makedirs(root, exist_ok=True)
    f["disk"] = {"path": root, "free_gb": round(shutil.disk_usage(root).free / 1e9, 2)}
    f["processes"] = processes()

    time.sleep(1.0)  # discovery
    snap = probe.snapshot()
    f["graph"] = snap
    nodes = snap["nodes"]
    topics = snap["topics"]
    ep = snap["endpoints"]

    def pubs(t):
        return sorted({e["node"] for e in ep.get(t, {}).get("publishers", [])})

    def subs(t):
        return sorted({e["node"] for e in ep.get(t, {}).get("subscribers", [])})

    f["duplicate_nodes"] = probe.duplicate_nodes()
    f["clock_publishers"] = pubs("/clock")
    n, last = probe.sample(ODOM_TOPIC, "nav_msgs/msg/Odometry", listen_s) \
        if ODOM_TOPIC in topics else (0, None)
    f["odom"] = {"type": (topics.get(ODOM_TOPIC) or [None])[0], "rate_hz": n / listen_s,
                 "publishers": pubs(ODOM_TOPIC),
                 "frame": last.header.frame_id if last else None,
                 "child": last.child_frame_id if last else None}
    if cfg.mode == "live":
        try:
            n2, _ = probe.sample("/sportmodestate", "unitree_go/msg/SportModeState", 1.0)
            f["sportmodestate"] = {"rate_hz": n2 / 1.0}
        except Exception as exc:
            f["sportmodestate"] = {"rate_hz": 0.0, "type_unavailable": True, "error": str(exc)}
    f["sport_response_publishers"] = pubs("/api/sport/response")
    f["localization"] = probe.latest_json("/riskgraph/localization/status")
    t, _ = probe.lookup("map", "base_link", 1.0)
    t2, age = probe.lookup("odom", "base_link", 1.0)
    f["tf"] = {"map_base_ok": t is not None, "odom_base_ok": t2 is not None, "odom_base_age_s": age,
               "robot_map_xy": ([t.transform.translation.x, t.transform.translation.y] if t else None)}
    f["tf_publishers"] = [p for p in pubs("/tf") if not p.startswith("/transform_listener")]
    f["tf_static_publishers"] = pubs("/tf_static")
    f["lifecycle"] = {nn: (probe.lifecycle_state(nn) if nn in nodes else None) for nn in NAV_NODES}
    f["actions"] = {"compute_path_to_pose": bool(pubs("/compute_path_to_pose/_action/status")),
                    "follow_path": bool(pubs("/follow_path/_action/status"))}
    f["nav_cmd_vel_publishers"] = pubs("/nav/cmd_vel")
    f["risk_costmap_subscribers"] = subs("/riskgraph/risk_costmap")
    f["velocity_smoother_params"] = probe.params("/velocity_smoother", ["max_velocity", "min_velocity"]) \
        if "/velocity_smoother" in nodes else None
    f["sink_params"] = probe.params(SINK, ["mode", "max_vx", "max_vy", "max_wz"]) if SINK in nodes else None
    sim = {}
    for nn in RG_NODES + [LOC_NODE] + NAV_NODES:
        if nn in nodes:
            pv = probe.params(nn, ["use_sim_time"])
            sim[nn] = bool(pv and pv.get("use_sim_time"))
    f["use_sim_time"] = sim
    f["riskgraph_nodes"] = [nn for nn in nodes if nn in RG_NODES]
    f["riskgraph_status"] = probe.latest_json("/riskgraph/status") if f["riskgraph_nodes"] else None
    f["riskgraph_store_paths"] = {}
    for nn in f["riskgraph_nodes"]:
        pv = probe.params(nn, ["store_path"])
        if pv:
            f["riskgraph_store_paths"][nn] = pv["store_path"]
    motion = {}
    for nn in nodes:
        if (nn.startswith("/riskgraph") or nn == LOC_NODE) and nn != SINK:
            motion[nn] = sorted(t for t, types in probe.topics_published_by(nn).items()
                                if set(types) & MOTION_TYPES)
    f["riskgraph_motion_topics"] = motion
    f["cmd_vel_publishers"] = pubs("/cmd_vel")
    f["cmd_vel_subscribers"] = subs("/cmd_vel")
    f["sport_request_publishers"] = pubs("/api/sport/request")
    f["twist_mux_nodes"] = [nn for nn in nodes if "twist_mux" in nn]
    f["nav_cmd_vel_subscribers"] = subs("/nav/cmd_vel")
    f["foreign_sinks"] = [nn for nn in nodes if "sport_sink" in nn and nn != SINK]
    f["sink"] = _sink_facts(probe) if SINK in nodes else {"present": False}
    f["sink_evidence"] = read_sink_stage_evidence(cfg.sink_session)
    if cfg.riskgraph == "expect-absent":
        f["db"] = _db_facts(cfg.store_path)
    return f


def _sink_facts(probe) -> Dict:
    from .ros_graph import SENSOR
    try:
        m = probe.wait_one(SINK_TRACE, "std_msgs/msg/String", 2.0, qos=SENSOR)
    except Exception as exc:
        return {"present": True, "error": f"cannot read the sink trace: {exc}"}
    if m is None:
        return {"present": True, "fresh": False}
    try:
        d = json.loads(m.data)
    except ValueError:
        return {"present": True, "fresh": False, "error": "unparseable trace"}
    return {"present": True, "fresh": True, "mode": d.get("mode"), "api_id": d.get("api_id"),
            "reason": d.get("reason"), "sent_to_robot": d.get("sent_to_robot")}


def _db_facts(path: str) -> Dict:
    from riskgraph_core.store import RiskStore, StoreError
    d = os.path.dirname(path)
    while d and not os.path.isdir(d):  # nearest existing ancestor: the node creates the rest
        d = os.path.dirname(d)
    facts = {"path": path, "exists": os.path.exists(path),
             "dir_writable": bool(d) and os.access(d, os.W_OK), "writable_ancestor": d}
    if facts["exists"]:
        try:
            with RiskStore(path, readonly=True) as s:
                st = s.status()
            facts.update({k: st[k] for k in ("map_id", "evidence_class", "incident_count",
                                             "quarantined_count", "schema_version")})
        except StoreError as exc:
            facts["error"] = str(exc)
            facts["map_id"] = "UNREADABLE"
    return facts


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def print_report(checks: List[Check], v: str, stream=sys.stdout) -> None:
    width = max(len(c.title) for c in checks) + 2
    for c in checks:
        mark = {"PASS": " ok ", "WARN": "WARN", "FAIL": "FAIL", "INFO": "info"}[c.result]
        print(f"[{mark}] {c.id} {c.title.ljust(width)} {c.detail[:160]}", file=stream)
    nfail = sum(c.result == FAIL for c in checks)
    bar = "=" * 72
    print(bar, file=stream)
    if v == "GO":
        print("PREFLIGHT: GO  (fit to ATTEMPT the trial; this verifies nothing about the robot)",
              file=stream)
    else:
        print(f"PREFLIGHT: NO-GO  ({nfail} blocking failure(s)). DO NOT MOVE THE ROBOT.", file=stream)
        for c in checks:
            if c.result == FAIL:
                print(f"   FAIL {c.id}: {c.title}: {c.detail[:200]}", file=stream)
    print(bar, file=stream)


def repo_root() -> str:
    env = os.environ.get("RISKGRAPH_REPO")
    if env:
        return env
    here = os.path.dirname(os.path.realpath(__file__))
    r = subprocess.run(["git", "-C", here, "rev-parse", "--show-toplevel"], capture_output=True, text=True)
    return r.stdout.strip() or os.getcwd()


def build_config(a) -> Config:
    from .paths import resolve
    r = resolve(a.experiment, a.store_path, a.db_root, a.db_tag)
    baseline = None
    bpath = a.sport_baseline or (os.path.join(a.sink_session, "sport_baseline.json")
                                 if a.sink_session else None)
    if bpath and os.path.exists(bpath):
        baseline = json.load(open(bpath))
        if isinstance(baseline, dict):
            baseline = baseline.get("publishers")
    return Config(mode=a.mode, riskgraph=getattr(a, "riskgraph", "expect-absent"),
                  expected_branch=a.expected_branch,
                  expected_iface=a.iface, store_path=r.store_path, map_id=r.map_id,
                  evidence_root=os.path.expanduser(a.evidence_root),
                  sink_session=a.sink_session, sport_baseline=baseline)


def add_common_args(ap: argparse.ArgumentParser) -> None:
    from .paths import DEFAULT_DB_ROOT, DEFAULT_DB_TAG, DEFAULT_EVIDENCE_ROOT
    ap.add_argument("--mode", choices=["live", "rehearsal"], default="live")
    ap.add_argument("--experiment", default=None)
    ap.add_argument("--store-path", default=None)
    ap.add_argument("--db-root", default=DEFAULT_DB_ROOT)
    ap.add_argument("--db-tag", default=DEFAULT_DB_TAG)
    ap.add_argument("--evidence-root", default=DEFAULT_EVIDENCE_ROOT)
    ap.add_argument("--sink-session", default=None,
                    help="riskgraph_sink_stage session dir (stage_S0/S1/S2.json, sport_baseline.json)")
    ap.add_argument("--sport-baseline", default=None)
    ap.add_argument("--expected-branch", default=None)
    ap.add_argument("--iface", default="enP8p1s0")


def run(cfg: Config, repo: str) -> Dict:
    import rclpy
    from .ros_graph import GraphProbe
    started = not rclpy.ok()
    if started:
        rclpy.init()
    probe = GraphProbe("riskgraph_preflight")
    try:
        facts = collect(probe, cfg, repo)
    finally:
        probe.close()
        if started:
            rclpy.try_shutdown()
    checks = evaluate(facts, cfg)
    return {"verdict": verdict(checks), "mode": cfg.mode, "config": asdict(cfg),
            "checks": [asdict(c) for c in checks], "facts": facts, "time": time.time()}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    ap.add_argument("--riskgraph", choices=["expect-absent", "expect-running"], default="expect-absent",
                    help="expect-absent before the runner launches RiskGraph (default)")
    ap.add_argument("--out", default=None, help="write the full JSON report here")
    a = ap.parse_args(argv)
    cfg = build_config(a)
    rep = run(cfg, repo_root())
    print_report([Check(**c) for c in rep["checks"]], rep["verdict"])
    if a.out:
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        with open(a.out, "w") as fh:
            json.dump(rep, fh, indent=1, default=str)
        print(f"report: {a.out}")
    return 0 if rep["verdict"] == "GO" else 1


if __name__ == "__main__":
    sys.exit(main())
