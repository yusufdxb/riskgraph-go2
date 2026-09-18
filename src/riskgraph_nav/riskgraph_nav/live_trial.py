"""The canonical live RiskGraph GO2 experiment, end to end, with evidence.

    ros2 run riskgraph_nav riskgraph_live_trial --mode live \\
        --helix-session ~/helix_hw/<date>_motion

Runs Trials A to E of docs/HW_VERIFICATION.md against the navigation stack
the operator has already started (riskgraph_nav_live.launch.py), the HELIX
closed loop with its motion arbiter, and the HELIX sport sink in ``armed``
mode. This program launches (and, for Trial D, restarts) RiskGraph itself.

What this program never does:

* publish a velocity, a twist, or any robot command. Motion only ever
  results from sending ONE operator-approved path to Nav2's controller
  (``/follow_path``), whose output reaches the robot only through
  velocity_smoother -> /nav/cmd_vel -> helix_arbiter -> /cmd_vel ->
  helix_go2_sport_sink.
* send a goal without the typed arming phrase ``ARM LIVE GO2 RISKGRAPH TRIAL``
  (live mode refuses ``--auto-confirm``).
* keep going after an abort condition: it cancels the goal, verifies the
  command stops, and tells the operator to use the remote if it does not.

Evidence goes to ``<evidence_root>/<YYYYmmdd_HHMMSS>_<sha7>[_REHEARSAL]/``.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import traceback
import uuid
from typing import Dict, List, Optional, Tuple

ARM_PHRASE = "ARM LIVE GO2 RISKGRAPH TRIAL"
INJECT_PHRASE = "INJECT RISK"
REMOTE_PHRASE = "REMOTE IN HAND"
XY = Tuple[float, float]


class Abort(Exception):
    """An abort condition: stop the experiment, keep the evidence."""


def now_s() -> float:
    return time.time()


def _json_default(o):
    if hasattr(o, "__dict__"):
        return o.__dict__
    return str(o)


class Runner:
    def __init__(self, a) -> None:
        from riskgraph_core.map_identity import describe_map
        from riskgraph_core.risk_field import RiskFieldParams

        from .paths import resolve
        from .preflight import git_info, repo_root
        self.a = a
        self.mode = a.mode
        self.rehearsal = a.mode == "rehearsal"
        if a.auto_confirm and not self.rehearsal:
            raise SystemExit("--auto-confirm is refused in live mode: a human arms every motion")
        self.res = resolve(a.experiment, a.store_path, a.db_root, a.db_tag)
        self.exp = self.res.experiment
        self.map_id = self.res.map_id
        self.store_path = self.res.store_path
        self.grid_info = describe_map(self.exp.map_yaml).grid_info()
        self.field_params: RiskFieldParams = self.exp.risk_params
        self.repo = repo_root()
        self.git = git_info(self.repo)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        suffix = "_REHEARSAL" if self.rehearsal else ""
        root = os.path.abspath(os.path.expanduser(a.evidence_root))
        self.run_dir = os.path.join(root, f"{stamp}_{(self.git['sha'] or 'nogit')[:7]}{suffix}")
        for d in ("", "logs", "trials", "graph", "db", "config"):
            os.makedirs(os.path.join(self.run_dir, d), exist_ok=True)
        self._log = open(os.path.join(self.run_dir, "logs", "runner.log"), "a", buffering=1)
        self.rg_proc: Optional[subprocess.Popen] = None
        self.rg_launches = 0
        self.bag_proc: Optional[subprocess.Popen] = None
        self.ros = None
        self.results: Dict[str, object] = {"trials": {}}
        self.abort_reason: Optional[str] = None
        self._goal_handle = None

    # -- console / operator ------------------------------------------------------

    def log(self, msg: str) -> None:
        line = f"[{time.strftime('%H:%M:%S')}] {msg}"
        print(line, flush=True)
        self._log.write(line + "\n")

    def banner(self, msg: str) -> None:
        self.log("=" * 72)
        for ln in msg.splitlines():
            self.log(ln)
        self.log("=" * 72)

    def confirm(self, phrase: str, prompt: str) -> None:
        """Block until the operator types ``phrase`` exactly. Anything else aborts."""
        if self.a.auto_confirm:
            self.log(f"[REHEARSAL auto-confirm] {phrase}")
            return
        self.log(prompt)
        try:
            got = input(f"Type exactly '{phrase}' to continue (anything else aborts): ").strip()
        except EOFError:
            got = ""
        self._log.write(f"operator typed: {got!r}\n")
        if got != phrase:
            raise Abort(f"operator did not confirm ({got!r} != {phrase!r})")

    def ask(self, prompt: str, default: str = "") -> str:
        if self.a.auto_confirm:
            return default
        try:
            v = input(prompt).strip()
        except EOFError:
            v = ""
        self._log.write(f"operator answered {prompt!r}: {v!r}\n")
        return v or default

    def save(self, rel: str, obj) -> str:
        p = os.path.join(self.run_dir, rel)
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, "w") as fh:
            json.dump(obj, fh, indent=1, default=_json_default)
        return p

    # -- processes -------------------------------------------------------------------

    def launch_riskgraph(self) -> dict:
        self.rg_launches += 1
        logp = os.path.join(self.run_dir, "logs", f"riskgraph_launch_{self.rg_launches}.log")
        cmd = ["ros2", "launch", "riskgraph_bringup", "riskgraph_live.launch.py",
               f"run_mode:={self.mode}", f"experiment:={self.exp.path}",
               f"store_path:={self.store_path}"]
        self.log(f"launching RiskGraph #{self.rg_launches}: {' '.join(cmd)}")
        self.rg_proc = subprocess.Popen(cmd, stdout=open(logp, "w"), stderr=subprocess.STDOUT,
                                        start_new_session=True)
        prev = (self.ros.rg_status or {}).get("instance_id")
        st = self.ros.wait_status(lambda s: s.get("instance_id") != prev and
                                  s.get("grid_state") == "OK", timeout=45.0)
        if st is None:
            raise Abort(f"RiskGraph did not come up with a published grid (see {logp})")
        if st.get("db_path") != self.store_path or st.get("map_id") != self.map_id:
            raise Abort(f"RiskGraph opened {st.get('db_path')} / {st.get('map_id')}, expected "
                        f"{self.store_path} / {self.map_id}")
        self.log(f"RiskGraph up: instance={st['instance_id']} pid={st['pid']} db={st['db_path']} "
                 f"schema=v{st['schema_version']} incidents={st['incident_count']} "
                 f"active_risk_entries={st.get('active_risk_entries')}")
        return st

    def stop_riskgraph(self) -> Dict[str, object]:
        if self.rg_proc is None:
            return {"stopped": False}
        pgid = os.getpgid(self.rg_proc.pid)
        os.killpg(pgid, signal.SIGINT)
        try:
            rc = self.rg_proc.wait(timeout=20.0)
            how = "SIGINT"
        except subprocess.TimeoutExpired:
            os.killpg(pgid, signal.SIGTERM)
            try:
                rc = self.rg_proc.wait(timeout=10.0)
                how = "SIGTERM"
            except subprocess.TimeoutExpired:
                os.killpg(pgid, signal.SIGKILL)
                rc = self.rg_proc.wait()
                how = "SIGKILL"
        self.rg_proc = None
        gone = self.ros.wait_until(lambda: not [n for n in self.ros.probe.nodes()
                                                if n.startswith("/riskgraph_") and
                                                n != "/riskgraph_localization" and
                                                not n.startswith("/riskgraph_live_trial")],
                                   timeout=10.0)
        self.log(f"RiskGraph stopped via {how}, launch exit code {rc}, nodes gone={gone}")
        return {"stopped": True, "how": how, "exit_code": rc, "nodes_gone": gone}

    def start_bag(self) -> None:
        from ament_index_python.packages import get_package_share_directory
        tf = os.path.join(get_package_share_directory("riskgraph_bringup"), "config", "rosbag",
                          "live_trial_topics.txt")
        topics = [t.strip() for t in open(tf) if t.strip() and not t.startswith("#")]
        out = os.path.join(self.run_dir, "bag")
        cmd = ["ros2", "bag", "record", "-o", out, "--include-hidden-topics", *topics]
        self.bag_proc = subprocess.Popen(cmd, stdout=open(os.path.join(self.run_dir, "logs", "bag.log"), "w"),
                                         stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(2.0)
        if self.bag_proc.poll() is not None:
            raise Abort("rosbag recorder exited immediately (see logs/bag.log)")
        self.log(f"recording {len(topics)} topics to {out}")

    def stop_bag(self) -> None:
        if self.bag_proc is None:
            return
        os.killpg(os.getpgid(self.bag_proc.pid), signal.SIGINT)
        try:
            self.bag_proc.wait(timeout=20.0)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(self.bag_proc.pid), signal.SIGKILL)
        self.bag_proc = None
        self.log("bag closed")

    def snapshot_graph(self, tag: str) -> None:
        self.save(f"graph/{tag}_graph.json", self.ros.probe.snapshot())
        for node in ("/riskgraph_memory", "/riskgraph_planner", "/riskgraph_localization",
                     "/planner_server", "/controller_server", "/velocity_smoother",
                     "/helix_arbiter", "/helix_go2_sport_sink", "/global_costmap/global_costmap"):
            if node in self.ros.probe.nodes():
                r = subprocess.run(["ros2", "param", "dump", node], capture_output=True, text=True,
                                   timeout=20)
                with open(os.path.join(self.run_dir, "graph",
                                       f"{tag}_params{node.replace('/', '_')}.yaml"), "w") as fh:
                    fh.write(r.stdout or r.stderr)

    def copy_db(self, tag: str) -> Optional[str]:
        from riskgraph_core.store import RiskStore
        if not os.path.exists(self.store_path):
            return None
        with RiskStore(self.store_path, readonly=True) as s:
            return s.backup_to(os.path.join(self.run_dir, "db", f"riskgraph_{tag}.sqlite"))

    def db_status(self) -> dict:
        from riskgraph_core.store import RiskStore
        with RiskStore(self.store_path, readonly=True, map_id=self.map_id) as s:
            return s.status()

    # -- positioning -------------------------------------------------------------

    def robot_pose(self) -> Optional[dict]:
        return self.ros.robot_pose()

    def ensure_at_start(self, label: str) -> dict:
        exp = self.exp
        for attempt in range(3):
            p = self.robot_pose()
            ok = p is not None and math.hypot(p["x"] - exp.start.x, p["y"] - exp.start.y) <= exp.start_xy_tol_m \
                and abs(math.atan2(math.sin(p["yaw"] - exp.start.yaw), math.cos(p["yaw"] - exp.start.yaw))) \
                <= exp.start_yaw_tol_rad
            if ok and self.ros.stationary():
                self.log(f"{label}: robot on marker {exp.start_marker}: ({p['x']:.2f}, {p['y']:.2f}, "
                         f"yaw {p['yaw']:.2f}), stationary")
                return p
            if self.rehearsal:
                self.ros.rehearsal_walk_to(exp.start.x, exp.start.y, exp.start.yaw)
                self.ros.wait_until(self.ros.stationary, timeout=60.0, settle=2.0)
                continue
            self.banner(
                f"{label}: place the robot on marker {exp.start_marker} "
                f"({exp.start.x}, {exp.start.y}) facing marker {exp.goal_marker} (+x).\n"
                f"Walk it there with the handheld remote, release the sticks, keep the remote in hand.\n"
                f"Current map pose: {p}")
            self.ask("Press Enter when the robot is standing still on the marker: ")
        raise Abort(f"{label}: robot is not on marker {exp.start_marker} within tolerance "
                    f"({exp.start_xy_tol_m} m, {exp.start_yaw_tol_rad} rad)")

    # -- planning -----------------------------------------------------------------------

    def plan(self, label: str, start=None) -> dict:
        from riskgraph_core.experiment import route_side, validate_path
        from riskgraph_core.geometry import polyline_length
        t0 = now_s()
        path, frame, reported = self.ros.compute_path(self.exp.goal, start=start)
        latency = now_s() - t0
        pts = [(x, y) for x, y in path]
        rp = self.robot_pose()
        robot_xy = (start.x, start.y) if start is not None else ((rp["x"], rp["y"]) if rp else None)
        problems = validate_path(pts, frame, self.exp, robot_xy, (self.exp.goal.x, self.exp.goal.y),
                                 is_lethal=self.ros.static_lethal)
        field = self.ros.current_field()
        rec = {
            "label": label, "time": t0, "frame": frame, "n_poses": len(pts), "points": pts,
            "length_m": polyline_length(pts) if len(pts) > 1 else 0.0,
            "planning_latency_s": latency, "planner_reported_time_s": reported,
            "side": route_side(pts, self.exp), "validation_problems": problems,
            "risk_metrics": field.path_metrics(pts) if pts else None,
            "nav2_costs": self.ros.path_costs(pts),
            "incidents": (self.ros.rg_status or {}).get("incident_count"),
            "start_override": None if start is None else {"x": start.x, "y": start.y, "yaw": start.yaw},
        }
        self.log(f"{label}: plan {len(pts)} poses, {rec['length_m']:.2f} m, side={rec['side']}, "
                 f"accumulated risk={rec['risk_metrics']['accumulated_risk'] if pts else None}, "
                 f"latency={latency:.2f}s, problems={problems}")
        return rec

    def arming_screen(self, label: str, plan: dict) -> None:
        st = self.ros.rg_status or {}
        field = self.ros.current_field()
        arb = self.ros.arbiter_summary()
        affecting = plan["risk_metrics"]["contributing_events"] if plan.get("risk_metrics") else []
        entries = []
        for b in field.bumps:
            entries.append(f"  {b.event_id} at ({b.x:.2f}, {b.y:.2f}) weight {b.weight:.2f}"
                           f"{'  <-- ON THIS ROUTE' if b.event_id in affecting else ''}")
        rp = self.robot_pose()
        self.banner("\n".join([
            f"ARMING CHECK: {label}   [{self.mode.upper()}]",
            f"goal:           marker {self.exp.goal_marker} ({self.exp.goal.x}, {self.exp.goal.y}) in map",
            f"current pose:   {rp}",
            f"planned route:  {plan['n_poses']} poses, {plan['length_m']:.2f} m, corridor {plan['side']}",
            f"route risk:     accumulated {plan['risk_metrics']['accumulated_risk']:.3f}, "
            f"max {plan['risk_metrics']['max_risk']:.3f}",
            f"risk entries ({len(field.bumps)}):", *(entries or ["  none"]),
            f"motion path:    /nav/cmd_vel <- velocity_smoother; arbiter: {arb}",
            f"sport sink:     {self.ros.sink_params()}",
            f"Nav2 state:     {self.ros.nav_states()}",
            f"RiskGraph DB:   {st.get('db_path')} (schema v{st.get('schema_version')}, "
            f"{st.get('incident_count')} incidents)",
            f"git SHA:        {self.git['sha']} dirty={self.git['dirty']}",
            f"map id:         {self.map_id}",
            "Operator: physical remote in hand, spotter ready, course clear.",
        ]))

    # -- execution -----------------------------------------------------------------------

    def execute(self, label: str, plan: dict) -> dict:
        from riskgraph_core.experiment import cross_track_error, route_side
        from riskgraph_core.geometry import polyline_length
        exp = self.exp
        if plan["validation_problems"]:
            raise Abort(f"{label}: refusing to execute a malformed route: {plan['validation_problems']}")
        self.arming_screen(label, plan)
        self.confirm(ARM_PHRASE, f"{label}: arm the robot for this exact route.")
        pre = self.ros.preexec_problems()
        if pre:
            raise Abort(f"{label}: not safe to send the goal: {pre}")
        loc0 = dict(self.ros.loc_status or {})
        motion_sources = self.ros.motion_sources()
        plans_before = self.ros.plan_msgs
        samples: List[dict] = []
        tf_fail = 0
        consecutive_fail_t = None
        max_xtrack = 0.0
        max_speed = 0.0
        abort = None
        progress = []
        self.ros.arbiter_reset_counters()
        self.ros.marker({"event": "execute_start", "trial": label})
        t0 = now_s()
        gh = self.ros.send_follow_path(plan["points"])
        self._goal_handle = gh
        result = None
        try:
            while True:
                if self.ros.stop_requested:
                    abort = "OPERATOR_CANCEL (Ctrl-C)"
                    break
                done, status = self.ros.follow_status(gh)
                p = self.robot_pose()
                t = now_s()
                if p is None:
                    tf_fail += 1
                    consecutive_fail_t = consecutive_fail_t or t
                    if t - consecutive_fail_t > 0.5:
                        abort = "TF_LOST: map->base_link unavailable > 0.5 s"
                        break
                else:
                    consecutive_fail_t = None
                    samples.append(dict(p, t=t))
                    xt = cross_track_error((p["x"], p["y"]), plan["points"])
                    max_xtrack = max(max_xtrack, xt)
                    if xt > exp.max_cross_track_m:
                        abort = f"OFF_PATH: {xt:.2f} m from the approved route"
                        break
                    progress.append((t, math.hypot(exp.goal.x - p["x"], exp.goal.y - p["y"])))
                spd = self.ros.speed()
                if spd is not None:
                    max_speed = max(max_speed, spd)
                    if spd > exp.max_speed_mps:
                        abort = f"OVERSPEED: {spd:.2f} m/s > {exp.max_speed_mps}"
                        break
                if done:
                    result = status
                    break
                why = self.ros.live_abort_condition(motion_sources)
                if why:
                    abort = why
                    break
                if t - t0 > exp.max_execution_s:
                    abort = f"TIMEOUT: {exp.max_execution_s} s"
                    break
                old = [d for (tt, d) in progress if tt <= t - 15.0]
                if old and progress and old[-1] - progress[-1][1] < 0.2 and t - t0 > 20.0:
                    abort = "NO_PROGRESS: < 0.2 m closer to the goal in 15 s (oscillation or stall)"
                    break
                time.sleep(0.1)
        except KeyboardInterrupt:
            abort = "OPERATOR_CANCEL (Ctrl-C)"
        exec_time = now_s() - t0
        post = {}
        if abort:
            self.log(f"ABORT during {label}: {abort}. Cancelling the Nav2 goal.")
            post = self.cancel_and_verify_stop(gh)
        else:
            post = self.verify_still_after(label)
        self._goal_handle = None
        self.ros.marker({"event": "execute_end", "trial": label, "result": result, "abort": abort})
        pts = [(s["x"], s["y"]) for s in samples]
        loc1 = dict(self.ros.loc_status or {})
        field = self.ros.current_field()
        rec = {
            "label": label, "result": result, "succeeded": result == "SUCCEEDED" and not abort,
            "abort_reason": abort, "execution_time_s": exec_time, "samples": samples,
            "executed_length_m": polyline_length(pts) if len(pts) > 1 else 0.0,
            "executed_side": route_side(pts, exp), "max_cross_track_m": max_xtrack,
            "max_speed_mps": max_speed, "tf_failures": tf_fail,
            "replans_during_execution": self.ros.plan_msgs - plans_before,
            "localization_jumps_delta": (loc1.get("odom_jumps") or 0) - (loc0.get("odom_jumps") or 0),
            "localization_gaps_delta": (loc1.get("odom_gaps") or 0) - (loc0.get("odom_gaps") or 0),
            "executed_risk_metrics": field.path_metrics(pts) if len(pts) > 1 else None,
            "arbiter": self.ros.arbiter_counters(), "post": post,
            "final_pose": samples[-1] if samples else None,
        }
        self.log(f"{label}: result={result} abort={abort} time={exec_time:.1f}s "
                 f"executed={rec['executed_length_m']:.2f} m side={rec['executed_side']} "
                 f"max_xtrack={max_xtrack:.2f} max_speed={max_speed:.2f}")
        if abort:
            self.results["trials"][label] = {"execution": rec}
            raise Abort(f"{label}: {abort}")
        if not rec["succeeded"]:
            self.results["trials"][label] = {"execution": rec}
            raise Abort(f"{label}: Nav2 FollowPath ended {result}")
        return rec

    def cancel_and_verify_stop(self, gh) -> dict:
        cancelled = self.ros.cancel(gh)
        t0 = now_s()
        persisted = False
        nonzero_after = 0
        while now_s() - t0 < 3.0:
            cmd = self.ros.last_cmd_vel_nonzero_age()
            if cmd is not None and now_s() - t0 > 1.0 and cmd < 0.2:
                nonzero_after += 1
                persisted = True
            time.sleep(0.1)
        if persisted:
            self.banner("MOTION COMMAND PERSISTS AFTER GOAL CANCELLATION.\n"
                        "USE THE HANDHELD REMOTE / E-STOP NOW. The trial is aborted.")
        spd = self.ros.speed()
        return {"cancel_accepted": cancelled, "command_persisted_after_cancel": persisted,
                "nonzero_cmd_samples_after_1s": nonzero_after, "speed_after": spd}

    def verify_still_after(self, label: str) -> dict:
        ok = self.ros.wait_until(self.ros.stationary, timeout=5.0, settle=1.0)
        persisted = False
        age = self.ros.last_cmd_vel_nonzero_age()
        if age is not None and age < 0.2:
            persisted = True
        if not ok or persisted:
            self.banner(f"{label}: robot not still after the goal ended (still={ok}, "
                        f"command persists={persisted}). Use the remote if it is moving.")
        return {"stationary_after": ok, "command_persisted_after_goal": persisted}

    # -- trials ------------------------------------------------------------------------------

    def inject(self, label: str, capture: Optional[dict], target: XY) -> dict:
        """Publish an OPERATOR_INJECTED event at a real, TF-derived robot map pose."""
        from riskgraph_core.map_identity import compute_map_id  # noqa: F401 (identity recorded below)
        exp = self.exp
        mode = "pass-through"
        if capture is None or capture["distance_to_target_m"] > exp.capture_radius_m:
            mode = "stationary"
            self.log(f"{label}: no executed pose within {exp.capture_radius_m} m of the capture point "
                     f"{target}; falling back to a STATIONARY capture at the robot's live pose.")
            if self.rehearsal:
                tf = self.ros.map_to_odom_target(target[0], target[1], 0.0)
                self.ros.rehearsal_walk_to(*tf)
                self.ros.wait_until(self.ros.stationary, timeout=60.0, settle=2.0)
            else:
                self.banner(f"{label}: walk the robot to map ({target[0]:.2f}, {target[1]:.2f}) "
                            f"(the corridor beside the box) with the remote and stop there.")
                self.ask("Press Enter when the robot is standing still there: ")
            p = self.robot_pose()
            if p is None:
                raise Abort(f"{label}: no live pose for the injection")
            capture = dict(p, t=now_s(), distance_to_target_m=math.hypot(p["x"] - target[0], p["y"] - target[1]))
            if capture["distance_to_target_m"] > 2 * exp.capture_radius_m:
                raise Abort(f"{label}: robot is {capture['distance_to_target_m']:.2f} m from the "
                            f"capture point; refusing to inject elsewhere")
        self.confirm(INJECT_PHRASE, f"{label}: store an OPERATOR-INJECTED risk observation at the "
                     f"robot's live map pose ({capture['x']:.3f}, {capture['y']:.3f}), captured "
                     f"{mode} at t={capture['t']:.2f}.")
        before = self.db_status()
        grid_before = self.ros.global_cost_at(capture["x"], capture["y"])
        eid = str(uuid.uuid4())
        detail = {"capture_mode": mode, "tf_target": "map", "tf_source": "base_link",
                  "captured_at": capture["t"], "capture_tf": capture.get("tf"),
                  "distance_to_capture_point_m": capture["distance_to_target_m"],
                  "capture_point": list(target), "map_id": self.map_id, "trial": label,
                  "evidence_class": "live" if not self.rehearsal else "rehearsal"}
        self.ros.publish_event(eid, capture["x"], capture["y"], capture["t"], exp.injection_severity,
                               json.dumps(detail, default=str), self.map_id)
        st = self.ros.wait_status(lambda s: s.get("incident_count", 0) > before["incident_count"], 10.0)
        from riskgraph_core.store import RiskStore
        with RiskStore(self.store_path, readonly=True, map_id=self.map_id) as s:
            row = s.get_event(eid)
        if row is None:
            q = [x for x in RiskStore(self.store_path, readonly=True).quarantined() if x["event_id"] == eid]
            raise Abort(f"{label}: injected event not stored in {self.store_path} (quarantine: {q})")
        err = math.hypot(row.position[0] - capture["x"], row.position[1] - capture["y"])
        ok_row = (err < 1e-6 and row.frame_id == "map" and row.provenance.value == "OPERATOR_INJECTED"
                  and row.map_id == self.map_id and row.run_mode == self.mode)
        nav_ok = self.ros.wait_until(lambda: (self.ros.global_cost_at(capture["x"], capture["y"]) or 0)
                                     >= 60, timeout=10.0)
        rec = {
            "label": label, "event_id": eid, "capture": capture, "capture_mode": mode,
            "stored": {"position": list(row.position), "frame_id": row.frame_id,
                       "source_frame_id": row.source_frame_id, "timestamp": row.timestamp,
                       "severity": row.factors[0].severity, "provenance": row.provenance.value,
                       "map_id": row.map_id, "run_mode": row.run_mode, "ingest_time": row.ingest_time,
                       "clock_note": row.clock_note},
            "position_error_m": err, "row_ok": ok_row, "db_path": self.store_path,
            "incidents_before": before["incident_count"], "incidents_after": (st or {}).get("incident_count"),
            "risk_grid_value_at_event": self.ros.risk_value_at(capture["x"], capture["y"]),
            "nav2_global_cost_before": grid_before,
            "nav2_global_cost_after": self.ros.global_cost_at(capture["x"], capture["y"]),
            "nav2_received": nav_ok,
        }
        self.log(f"{label}: stored {eid} at ({row.position[0]:.3f}, {row.position[1]:.3f}) row_ok={ok_row} "
                 f"grid={rec['risk_grid_value_at_event']} nav2 cost {grid_before} -> "
                 f"{rec['nav2_global_cost_after']}")
        if not ok_row:
            raise Abort(f"{label}: stored row does not match the injected observation: {rec['stored']}")
        if not nav_ok:
            raise Abort(f"{label}: Nav2's global costmap did not pick up the risk within 10 s")
        return rec

    def run(self) -> int:
        from riskgraph_core.experiment import capture_point_on_path, compare_routes, nearest_sample
        from riskgraph_core.geometry import Pose2D
        from .preflight import Check, build_config, print_report, run as preflight_run, verdict
        exp = self.exp
        self.write_manifest()
        self.copy_db("before")
        from .ros_interface import TrialRos
        import rclpy
        rclpy.init()
        self.ros = TrialRos(self)
        signal.signal(signal.SIGINT, lambda *_: self.ros.request_stop())
        status = "INCOMPLETE"
        try:
            self.banner(f"RiskGraph live trial [{self.mode.upper()}]\nevidence: {self.run_dir}\n"
                        f"db: {self.store_path}\nmap_id: {self.map_id}")
            cfg = build_config(self.a)
            cfg.riskgraph = "expect-absent"
            rep = preflight_run(cfg, self.repo)
            self.save("preflight_before_launch.json", rep)
            print_report([Check(**c) for c in rep["checks"]], rep["verdict"])
            if rep["verdict"] != "GO":
                raise Abort("preflight (before launch) is NO-GO")
            st0 = self.launch_riskgraph()
            cfg.riskgraph = "expect-running"
            rep = preflight_run(cfg, self.repo)
            self.save("preflight.json", rep)
            print_report([Check(**c) for c in rep["checks"]], rep["verdict"])
            if rep["verdict"] != "GO":
                raise Abort("preflight (RiskGraph running) is NO-GO")
            self.results["preflight_verdict"] = rep["verdict"]
            if (st0.get("active_risk_entries") or 0) != 0:
                raise Abort(f"the database already holds {st0.get('active_risk_entries')} active risk "
                            f"entries; the baseline needs an empty field. Use a fresh --db-tag.")
            self.snapshot_graph("start")
            if not self.a.no_bag:
                self.start_bag()
            self.confirm(REMOTE_PHRASE, "Confirm the physical remote is in your hand, powered, and "
                         "you know how to stop the robot with it (stand lock / damp is NOT a stop).")

            # ---- Trial A: baseline ----
            self.ensure_at_start("Trial A")
            plan_a = self.plan("A_baseline")
            self.save("trials/A_baseline_plan.json", plan_a)
            ex_a = self.execute("A_baseline", plan_a)
            self.save("trials/A_baseline_execution.json", ex_a)
            self.results["trials"]["A_baseline"] = {"plan": _slim(plan_a), "execution": _slim(ex_a)}

            # ---- Trial B: risk observation on the baseline corridor ----
            target_b = capture_point_on_path(plan_a["points"], exp)
            cap_b = nearest_sample(ex_a["samples"], target_b)
            inj_b = self.inject("B_inject", cap_b, target_b)
            self.save("trials/B_injection.json", inj_b)
            self.results["trials"]["B_inject"] = inj_b

            # ---- Trial C: same goal, risk-aware ----
            self.ensure_at_start("Trial C")
            plan_c = self.plan("C_risk_aware")
            cmp_c = compare_routes(plan_a["points"], plan_c["points"], self.ros.current_field(), exp)
            self.save("trials/C_risk_aware_plan.json", dict(plan_c, comparison=cmp_c))
            self.log(f"C: baseline side={cmp_c['baseline_side']} aware side={cmp_c['aware_side']} "
                     f"risk {cmp_c['baseline_accumulated_risk']:.3f} -> {cmp_c['aware_accumulated_risk']:.3f} "
                     f"mean separation {cmp_c['mean_separation_m']:.2f} m  PASS={cmp_c['pass']}")
            if not cmp_c["pass"]:
                raise Abort("Trial C: remembered risk did not change the planned route "
                            "(see trials/C_risk_aware_plan.json)")
            ex_c = self.execute("C_risk_aware", plan_c)
            self.save("trials/C_risk_aware_execution.json", ex_c)
            exec_cmp = compare_routes([(s["x"], s["y"]) for s in ex_a["samples"]],
                                      [(s["x"], s["y"]) for s in ex_c["samples"]],
                                      self.ros.current_field(), exp)
            self.results["trials"]["C_risk_aware"] = {"plan": _slim(plan_c), "comparison": _slim(cmp_c),
                                                      "execution": _slim(ex_c),
                                                      "executed_comparison": _slim(exec_cmp)}

            # ---- Trial D: restart persistence ----
            self.results["trials"]["D_restart"] = self.trial_d(plan_a, plan_c)

            # ---- Trial E: fallback when avoidance is impossible ----
            target_e = capture_point_on_path(plan_c["points"], exp)
            cap_e = nearest_sample(ex_c["samples"], target_e)
            self.results["trials"]["E_fallback"] = self.trial_e(cap_e, target_e)
            status = "COMPLETED"
        except Abort as exc:
            self.abort_reason = str(exc)
            status = "ABORTED"
            self.banner(f"ABORTED: {exc}")
        except Exception as exc:  # anything unexpected is an abort, with a traceback
            self.abort_reason = f"{type(exc).__name__}: {exc}"
            status = "ERROR"
            self.log(traceback.format_exc())
        finally:
            try:
                if self._goal_handle is not None:
                    self.cancel_and_verify_stop(self._goal_handle)
            except Exception:
                pass
            try:
                self.snapshot_graph("end")
            except Exception as exc:
                self.log(f"end snapshot failed: {exc}")
            self.stop_bag()
            try:
                self.copy_db("after")
            except Exception as exc:
                self.log(f"db copy failed: {exc}")
            try:
                self.stop_riskgraph()
            except Exception as exc:
                self.log(f"stopping RiskGraph failed: {exc}")
            self.finish(status)
            self.ros.close()
            rclpy.try_shutdown()
        return 0 if status == "COMPLETED" and self.results.get("machine_pass") else 1

    def trial_d(self, plan_a: dict, plan_c: dict) -> dict:
        from riskgraph_core.experiment import compare_routes, route_side
        from riskgraph_core.geometry import polyline_separation
        exp = self.exp
        before = dict(self.ros.rg_status or {})
        ev_xy = _first_event_xy(self.ros.current_field())
        self.log("Trial D: stopping RiskGraph completely (memory, planner, explainer).")
        stop = self.stop_riskgraph()
        blank_seen = self.ros.wait_until(lambda: self.ros.risk_grid_max() == 0, timeout=10.0)
        cleared = self.ros.wait_until(lambda: (self.ros.global_cost_at(*ev_xy) or 0) < 60, timeout=10.0)
        self.ensure_at_start("Trial D")
        ablation = self.plan("D_riskgraph_down")
        abl_side_ok = ablation["side"] == plan_a["side"]
        self.log(f"D: with RiskGraph down Nav2 plans corridor {ablation['side']} (baseline "
                 f"{plan_a['side']}): " + ("the risk-aware choice came from RiskGraph, not a cache"
                                           if abl_side_ok else "NAV2 STILL AVOIDS THE RISK: stale layer"))
        st = self.launch_riskgraph()
        restored_costmap = self.ros.wait_until(lambda: (self.ros.global_cost_at(*ev_xy) or 0) >= 60,
                                               timeout=10.0)
        restored = self.plan("D_after_restart")
        sep = polyline_separation(restored["points"], plan_c["points"])[0]
        cmp_d = compare_routes(plan_a["points"], restored["points"], self.ros.current_field(), exp)
        rec = {
            "stop": stop, "blank_grid_seen": blank_seen, "nav2_cleared_after_stop": cleared,
            "ablation_plan": _slim(ablation), "ablation_side_matches_baseline": abl_side_ok,
            "instance_before": before.get("instance_id"), "instance_after": st.get("instance_id"),
            "pid_before": before.get("pid"), "pid_after": st.get("pid"),
            "incidents_before": before.get("incident_count"), "incidents_after": st.get("incident_count"),
            "db_path_after": st.get("db_path"), "nav2_restored": restored_costmap,
            "restored_plan": _slim(restored), "separation_from_trial_c_m": sep,
            "comparison_vs_baseline": _slim(cmp_d),
        }
        rec["pass"] = bool(
            stop.get("nodes_gone") and blank_seen and cleared and abl_side_ok
            and st.get("instance_id") != before.get("instance_id")
            and st.get("incident_count") == before.get("incident_count")
            and st.get("db_path") == self.store_path and restored_costmap
            and restored["side"] == plan_c["side"] and cmp_d["pass"])
        self.save("trials/D_restart.json", dict(rec, ablation_plan=ablation, restored_plan=restored))
        self.log(f"D: restart PASS={rec['pass']} (incidents {rec['incidents_before']} -> "
                 f"{rec['incidents_after']}, side {restored['side']})")
        do_exec = self.a.execute_d or self.ask("Execute the Trial D route physically? [y/N]: ", "n") \
            .lower().startswith("y")
        if do_exec:
            ex = self.execute("D_after_restart", restored)
            self.save("trials/D_execution.json", ex)
            rec["execution"] = _slim(ex)
            rec["executed_side"] = ex["executed_side"]
        if not rec["pass"]:
            raise Abort("Trial D: remembered risk did not survive the restart (see trials/D_restart.json)")
        return rec

    def trial_e(self, cap_e: Optional[dict], target_e: XY) -> dict:
        from riskgraph_core.experiment import fallback_checks, validate_path
        from riskgraph_core.geometry import Pose2D
        exp = self.exp
        inj = self.inject("E_second_corridor", cap_e, target_e)
        start = Pose2D(exp.start.x, exp.start.y, exp.start.yaw)
        plans = [self.plan(f"E_blocked_{i}", start=start) for i in range(5)]
        base = plans[0]
        chk = fallback_checks(base["points"], [p["points"] for p in plans[1:]],
                              self.ros.risk_grid_data(), exp.resolution)
        # a goal inside a risk region must still be plannable (risk is a cost, not an obstacle)
        ev_xy = (inj["stored"]["position"][0], inj["stored"]["position"][1])
        into = self.ros.compute_path(Pose2D(ev_xy[0], ev_xy[1], 0.0), start=start)
        states = self.ros.nav_states()
        rec = {
            "injection": inj, "plans": [_slim(p) for p in plans], "checks": chk,
            "all_valid": all(not p["validation_problems"] for p in plans),
            "latencies_s": [p["planning_latency_s"] for p in plans],
            "goal_in_risk_region_poses": len(into[0]),
            "nav2_states_after": states,
            "routes_through_risk": base["risk_metrics"]["accumulated_risk"] > 0,
        }
        rec["pass"] = bool(chk["pass"] and rec["all_valid"] and max(rec["latencies_s"]) < 5.0
                           and rec["goal_in_risk_region_poses"] >= 2
                           and all(v == "active" for v in states.values()))
        self.save("trials/E_fallback.json", rec)
        self.log(f"E: fallback PASS={rec['pass']} (deterministic={chk['deterministic']}, side "
                 f"{base['side']}, risk {base['risk_metrics']['accumulated_risk']:.3f})")
        return rec

    # -- bookkeeping -----------------------------------------------------------------------------

    def write_manifest(self) -> None:
        from .paths import default_experiment_file  # noqa: F401
        for src in (self.exp.path, self.exp.map_yaml,
                    os.path.splitext(self.exp.map_yaml)[0] + ".pgm"):
            shutil.copy(src, os.path.join(self.run_dir, "config", os.path.basename(src)))
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory("riskgraph_bringup")
            shutil.copy(os.path.join(share, "config", "nav2_live.yaml"), os.path.join(self.run_dir, "config"))
        except Exception:
            pass
        diff = subprocess.run(["git", "-C", self.repo, "diff", "HEAD"], capture_output=True, text=True).stdout
        with open(os.path.join(self.run_dir, "git_diff.patch"), "w") as fh:
            fh.write(diff)
        self.manifest = {
            "evidence_class": "hardware" if not self.rehearsal else "rehearsal",
            "mode": self.mode,
            "label": "HARDWARE TRIAL" if not self.rehearsal else "REHEARSAL: NOT HARDWARE EVIDENCE",
            "created": time.time(), "created_iso": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "host": socket.gethostname(), "user": os.environ.get("USER"),
            "git": self.git, "argv": sys.argv, "args": vars(self.a),
            "env": {k: os.environ.get(k) for k in ("ROS_DISTRO", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION",
                                                    "ROS_LOCALHOST_ONLY", "CYCLONEDDS_URI")},
            "map_id": self.map_id, "db_path": self.store_path, "experiment": self.exp.path,
            "map_yaml": self.exp.map_yaml, "run_dir": self.run_dir,
            "helix_session": self.a.helix_session,
        }
        self.save("manifest.json", self.manifest)

    def finish(self, status: str) -> None:
        from .report import build_report
        notes = ""
        if not self.a.auto_confirm:
            notes = self.ask("Operator notes for this run (one line, Enter to skip): ", "")
        attest = False
        if status == "COMPLETED" and not self.rehearsal:
            attest = self.ask("Did you SEE the robot walk both routes on the floor, with no manual "
                              "stick input during navigation? Type OBSERVED to attest: ", "") == "OBSERVED"
        with open(os.path.join(self.run_dir, "operator_notes.txt"), "a") as fh:
            fh.write(f"{time.strftime('%Y-%m-%dT%H:%M:%S')} status={status} attest={attest}\n{notes}\n")
        self.results.update(status=status, abort_reason=self.abort_reason, operator_attested=attest)
        rep = build_report(self.run_dir, self.manifest, self.results, self.ros, self.grid_info)
        self.results.update(machine_pass=rep["machine_pass"])
        self.banner(f"RESULT: {status}   machine checks: {'PASS' if rep['machine_pass'] else 'FAIL'}   "
                    f"hardware_pass: {rep['hardware_pass']}\n{rep['summary_path']}")


def _slim(d):
    """Drop bulky arrays from a record for the summary (full records are saved separately)."""
    if not isinstance(d, dict):
        return d
    return {k: v for k, v in d.items() if k not in ("points", "samples")}


def _first_event_xy(field) -> XY:
    if not field.bumps:
        raise Abort("no risk entry to test restart persistence with")
    b = field.bumps[0]
    return (b.x, b.y)


def main(argv=None) -> int:
    from .preflight import add_common_args
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    ap.add_argument("--auto-confirm", action="store_true",
                    help="REHEARSAL ONLY: answer every prompt automatically (refused in live mode)")
    ap.add_argument("--execute-d", action="store_true", help="execute the Trial D route without asking")
    ap.add_argument("--no-bag", action="store_true", help="do not record a rosbag (not for evidence)")
    a = ap.parse_args(argv)
    if a.mode == "live" and a.no_bag:
        raise SystemExit("--no-bag is refused in live mode: the bag is part of the evidence")
    code = Runner(a).run()
    # Every evidence file is written and closed at this point. rclpy's
    # interpreter-exit teardown with background executor threads has been
    # seen to segfault (exit 245 masking a PASS); skip it so the exit code is
    # the trial's.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)


if __name__ == "__main__":
    sys.exit(main())
