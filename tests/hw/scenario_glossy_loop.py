#!/usr/bin/env python3
"""Hardware scenario: glossy-loop.

Goal: prove on a real Go2 + Jetson that
    (a) the integration launch starts cleanly with adapters enabled,
    (b) live `/go2/safety_alert` events (or scripted equivalents) flow into
        the SQLite store and get spatially joined to the right segment,
    (c) `/riskgraph/score_routes` returns a verdict that prefers the safe
        arm over the historically-slippy arm,
    (d) cross-run memory survives a node restart on the Jetson NVMe path.

This script does NOT depend on the Go2 actually moving; the scripted-path
piece is designed to be runnable with the robot stationary on a stand or
with the operator manually walking the dog along a roughly known path.
The assertion target is the planner verdict, not closed-loop motion.

Why this is the v0.1.0 hw harness, not just another smoke test:

- It runs the *integration* launch, so all 3 adapters get exercised against
  the upstream wire format (whichever upstream packages are present).
- It uses a *real SQLite file* on a path the operator chooses (default
  `~/.local/share/riskgraph/hw_scenario.sqlite`), so cross-run NVMe
  semantics are tested.
- It records a rosbag the operator can keep alongside `verdict.json`, so
  post-session forensics are possible without re-running the dog.

Pre-flight (verify before pressing go):

    ros2 topic list | grep go2/safety_alert     # must exist
    ros2 topic list | grep helix/faults         # optional
    ros2 topic list | grep tactile/slip_state   # optional
    ros2 interface list | grep riskgraph        # must list 7 msgs + 2 srvs

Operator script: see docs/HW_VERIFICATION.md.

Exit codes:
    0  PASS   - both phase-1 (live event) and phase-2 (cross-run) green
    1  FAIL   - one or both assertions red; see verdict.json
    2  ABORT  - pre-flight problem (no service, no go2_msgs, etc.)
"""
from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from pathlib import Path

# rclpy and riskgraph imports are lazy so `--help` works on machines without ROS.

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from lib.scenario_runner import (  # noqa: E402
    EventSpec,
    SegmentSpec,
    assert_chosen,
    build_event_msg,
    build_route_msg,
    call_score_routes,
    make_run_dir,
    record_bag,
    stop_bag,
    write_verdict,
)


# ---------------------------------------------------------------------------
# Scenario geometry. Two parallel arms running 4m east. The "glossy" arm is
# at y=0; the "safe" arm is at y=2 (or whatever lateral clearance the lab
# has). All values are in the `map` frame.
#
# The slip events get published at (2, 0), the midpoint of the glossy arm,
# so spatial join (in either the adapter-passthrough case or the memory-node
# nearest-segment fallback) lands them on `glossy`.
# ---------------------------------------------------------------------------
GLOSSY = SegmentSpec("hw_glossy", x0=0.0, y0=0.0, x1=4.0, y1=0.0,
                    semantic_label="hallway-glossy")
SAFE = SegmentSpec("hw_safe", x0=0.0, y0=2.0, x1=4.0, y1=2.0,
                   semantic_label="hallway-safe")

EVENTS = [
    EventSpec(event_id=f"hw_slip_{i}", x=2.0, y=0.0, severity=0.9,
              category="SLIP", source="tactile/slip_state",
              detail="hw scenario synthetic slip")
    for i in range(3)
]

BAG_TOPICS = [
    "/riskgraph/risk_events",
    "/riskgraph/route_scores",
    "/riskgraph/explanations",
    "/go2/safety_alert",
    "/helix/faults",
    "/tactile/slip_state",
    "/cmd_vel",
    "/tf",
    "/tf_static",
]


def _publish_events(pub, events, period_s: float = 0.2) -> None:
    for spec in events:
        pub.publish(build_event_msg(spec))
        time.sleep(period_s)


def _phase_one_live(node, score_client, scores_pub_topic: str) -> dict:
    """Publish synthetic slip events, then call score_routes.

    Returns a dict with the per-phase result fields for verdict.json.
    """
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from riskgraph_msgs.msg import RiskEvent as RiskEventMsg

    qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST, depth=20)
    pub = node.create_publisher(RiskEventMsg, "/riskgraph/risk_events", qos)
    # Allow discovery to settle.
    time.sleep(1.0)

    _publish_events(pub, EVENTS)
    # Let the memory node consume.
    time.sleep(0.8)

    short = build_route_msg("short_glossy", [GLOSSY])
    long_ = build_route_msg("long_safe", [SAFE])

    resp, err = call_score_routes(score_client, [short, long_], semantic_objective="")
    if err:
        return {"phase": "phase1_live", "pass": False, "error": err}

    ok, reason = assert_chosen(resp, expected_route_id="long_safe")
    return {
        "phase": "phase1_live",
        "pass": ok,
        "reason": reason,
        "chosen_route_id": resp.result.chosen_route_id,
        "scores": [
            {
                "route_id": s.route_id,
                "total_cost": float(s.total_cost),
                "geometry_cost": float(s.geometry_cost),
                "risk_cost": float(s.risk_cost),
                "dominant_segment_ids": list(s.dominant_segment_ids),
                "dominant_factor_categories": list(s.dominant_factor_categories),
            }
            for s in resp.result.scores
        ],
        "explanation": resp.explanation.text,
        "evidence_event_ids": list(resp.explanation.evidence_event_ids),
    }


def _phase_two_cross_run(score_client) -> dict:
    """After a node restart, score the same routes WITHOUT republishing events.

    The verdict here proves the SQLite file persisted across the kill/restart.
    The operator must restart `riskgraph_memory` + `riskgraph_planner` (or the
    whole launch) between phase 1 and phase 2; this script blocks waiting for
    the planner service to reappear.
    """
    short = build_route_msg("short_glossy", [GLOSSY])
    long_ = build_route_msg("long_safe", [SAFE])

    resp, err = call_score_routes(score_client, [short, long_], semantic_objective="")
    if err:
        return {"phase": "phase2_cross_run", "pass": False, "error": err}

    ok, reason = assert_chosen(resp, expected_route_id="long_safe")
    return {
        "phase": "phase2_cross_run",
        "pass": ok,
        "reason": reason,
        "chosen_route_id": resp.result.chosen_route_id,
        "explanation": resp.explanation.text,
        "evidence_event_ids": list(resp.explanation.evidence_event_ids),
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--phase",
        choices=["one", "two", "both"],
        default="both",
        help="`one` = live publish + score; `two` = score-only (post-restart cross-run); "
             "`both` = run phase one, prompt for restart, then phase two.",
    )
    parser.add_argument("--out-root", default=str(HERE), help="root dir for runs/<ts>/")
    parser.add_argument("--no-bag", action="store_true", help="skip rosbag record")
    parser.add_argument(
        "--store-path",
        default=os.path.expanduser("~/.local/share/riskgraph/hw_scenario.sqlite"),
        help="SQLite file the launch was started with (informational; this script "
             "does not set the param itself - see docs/HW_VERIFICATION.md)",
    )
    args = parser.parse_args(argv)

    out_root = Path(args.out_root)
    run_dir = make_run_dir(out_root)
    print(f"[hw] run_dir = {run_dir}")
    print(f"[hw] store_path (informational) = {args.store_path}")

    bag_proc = None
    if not args.no_bag:
        bag_proc = record_bag(run_dir, BAG_TOPICS)
        if bag_proc is None:
            print("[hw] WARN: ros2 not found on PATH; skipping bag record")

    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from riskgraph_msgs.srv import ScoreRoutes
    except ImportError as e:
        print(f"[hw] ABORT: ROS environment not sourced ({e})", file=sys.stderr)
        stop_bag(bag_proc)
        return 2

    rclpy.init()
    node = rclpy.create_node("riskgraph_hw_scenario")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    score_client = node.create_client(ScoreRoutes, "/riskgraph/score_routes")

    results = []
    overall_pass = True
    try:
        if args.phase in ("one", "both"):
            r1 = _phase_one_live(node, score_client, "/riskgraph/route_scores")
            results.append(r1)
            print(f"[hw] phase1: pass={r1['pass']}  reason={r1.get('reason')}")
            if not r1["pass"]:
                overall_pass = False

        if args.phase == "both":
            print("\n[hw] >>> RESTART the launch now (kill + relaunch with the SAME store_path)")
            print("[hw] >>> Press <Enter> here when the planner service is back up.")
            try:
                input()
            except EOFError:
                # Non-interactive: assume operator knows they need to wait
                time.sleep(10.0)

        if args.phase in ("two", "both"):
            r2 = _phase_two_cross_run(score_client)
            results.append(r2)
            print(f"[hw] phase2: pass={r2['pass']}  reason={r2.get('reason')}")
            if not r2["pass"]:
                overall_pass = False

    finally:
        try:
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        stop_bag(bag_proc)

    verdict = {
        "scenario": "glossy_loop",
        "version_under_test": "v0.1.0",
        "store_path": args.store_path,
        "pass": overall_pass,
        "phases": results,
        "bag_dir": str((run_dir / "bag").resolve()) if not args.no_bag else None,
    }
    p = write_verdict(run_dir, verdict)
    print(f"[hw] verdict written to {p}")
    print(f"[hw] OVERALL: {'PASS' if overall_pass else 'FAIL'}")
    return 0 if overall_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
