"""Evidence report: PASS criteria, route comparison, metrics, SVG, summary.

The PASS list mirrors the lab acceptance criteria in docs/HW_VERIFICATION.md.
``machine_pass`` is what the software could check. ``hardware_pass`` is true
ONLY when the evidence class is ``hardware`` (live mode, never rehearsal or
replay), every machine criterion passed, AND the operator attested that they
saw the robot walk both routes. Nothing in this repository sets a
hardware-verified claim automatically; a human updates the docs from here.
"""
from __future__ import annotations

import json
import os
from typing import Dict, List, Tuple

from riskgraph_core.experiment import course_grid
from riskgraph_core.viz import render_routes_svg


def _get(d, *keys, default=None):
    for k in keys:
        if not isinstance(d, dict) or k not in d:
            return default
        d = d[k]
    return d


def criteria(results: Dict, manifest: Dict, run_dir: str) -> List[Dict]:
    t = results.get("trials", {})
    a, b, c, d, e = (t.get(k, {}) for k in ("A_baseline", "B_inject", "C_risk_aware",
                                            "D_restart", "E_fallback"))
    pre_ok = results.get("preflight_verdict") == "GO"
    files = ["manifest.json", "preflight.json", "db/riskgraph_after.sqlite", "logs/runner.log"]
    bag_ok = os.path.isdir(os.path.join(run_dir, "bag"))
    out = [
        ("1", "live localization fed the system (preflight GO: odometry, anchor, TF)", pre_ok),
        ("2", "baseline A->B executed to success and the robot stopped",
         _get(a, "execution", "succeeded") is True and
         _get(a, "execution", "post", "stationary_after") is True),
        ("3", "risk event stored at the captured live map pose",
         _get(b, "row_ok") is True and _get(b, "position_error_m") is not None
         and _get(b, "position_error_m") < 1e-6),
        ("4", "event present in the intended SQLite database",
         _get(b, "db_path") == manifest.get("db_path") and
         (_get(b, "incidents_after") or 0) > (_get(b, "incidents_before") or 0)),
        ("5", "route risk cost changed as intended (aware plan lower under the same field)",
         _get(c, "comparison", "risk_reduced") is True),
        ("6", "Nav2 received it (low-inflation cells at the event rose >= 30 in the global costmap)",
         _get(b, "nav2_received") is True),
        ("7", "a physically different, lower-risk route was planned",
         _get(c, "comparison", "pass") is True),
        ("8", "the GO2 executed that route (executed corridor = planned, lower executed risk)",
         _get(c, "execution", "succeeded") is True and
         _get(c, "execution", "post", "stationary_after") is True and
         _get(c, "execution", "executed_side") == _get(c, "plan", "side") and
         _get(c, "executed_comparison", "risk_reduced") is True),
        ("9", "restart did not erase the event (new process, same incident count, same DB)",
         _get(d, "instance_after") not in (None, _get(d, "instance_before")) and
         _get(d, "incidents_after") == _get(d, "incidents_before") and
         _get(d, "db_path_after") == manifest.get("db_path")),
        ("10", "persisted event still influences planning (ablation reverts, restart restores)",
         _get(d, "pass") is True),
        ("11", "no second motor-command authority (preflight P30/P31/P33/P34, no change during runs)",
         pre_ok and not str(results.get("abort_reason") or "").startswith("MOTION_SOURCE")),
        ("12", "motion only through arbiter + sink; no HELIX-held motion",
         pre_ok and all((_get(t.get(k, {}), "execution", "arbiter", "nonzero_while_hold") or 0) == 0
                        for k in ("A_baseline", "C_risk_aware"))),
        ("13", "evidence bundle complete (manifest, preflight, DB copies, logs, bag)",
         all(os.path.exists(os.path.join(run_dir, f)) for f in files) and bag_ok
         and _get(results, "bag", "ok") is True),
        ("E", "graceful fallback when avoidance is impossible", _get(e, "pass") is True),
    ]
    return [{"id": i, "criterion": txt, "pass": bool(ok)} for i, txt, ok in out]


def build_report(run_dir: str, manifest: Dict, results: Dict, ros, grid_info) -> Dict:
    crit = criteria(results, manifest, run_dir)
    machine_pass = results.get("status") == "COMPLETED" and all(c["pass"] for c in crit)
    hardware = manifest.get("evidence_class") == "hardware"
    hardware_pass = bool(hardware and machine_pass and results.get("operator_attested"))
    routes = _collect_routes(run_dir)
    comparison = {
        "baseline_plan_side": _get(results, "trials", "A_baseline", "plan", "side"),
        "risk_aware_plan_side": _get(results, "trials", "C_risk_aware", "plan", "side"),
        "plan_comparison": _get(results, "trials", "C_risk_aware", "comparison"),
        "executed_comparison": _get(results, "trials", "C_risk_aware", "executed_comparison"),
        "restart": {k: _get(results, "trials", "D_restart", k) for k in
                    ("ablation_side_matches_baseline", "separation_from_trial_c_m", "pass")},
    }
    with open(os.path.join(run_dir, "routes_comparison.json"), "w") as fh:
        json.dump(comparison, fh, indent=1, default=str)
    try:
        exp = ros.r.exp
        info, occ = course_grid(exp)
        field = ros.current_field()
        events = [{"id": b.event_id, "x": b.x, "y": b.y, "label": b.event_id[:8]} for b in field.bumps]
        svg = render_routes_svg(info, occ, field.rasterize(info), routes, events,
                                f"{manifest.get('label')} {os.path.basename(run_dir)}")
        with open(os.path.join(run_dir, "routes.svg"), "w") as fh:
            fh.write(svg)
    except Exception as exc:  # the picture is a convenience; the JSON is the evidence
        results["svg_error"] = str(exc)
    summary = {
        "evidence_class": manifest.get("evidence_class"), "label": manifest.get("label"),
        "status": results.get("status"), "abort_reason": results.get("abort_reason"),
        "machine_pass": machine_pass, "operator_attested": results.get("operator_attested"),
        "hardware_pass": hardware_pass,
        "hardware_pass_rule": "evidence_class == hardware AND all criteria AND operator attestation; "
                              "rehearsal and replay can never set it",
        "criteria": crit, "git": manifest.get("git", {}).get("sha"),
        "git_dirty": manifest.get("git", {}).get("dirty"), "map_id": manifest.get("map_id"),
        "db_path": manifest.get("db_path"), "results": results,
    }
    sp = os.path.join(run_dir, "summary.json")
    with open(sp, "w") as fh:
        json.dump(summary, fh, indent=1, default=str)
    _write_markdown(run_dir, summary)
    return {"machine_pass": machine_pass, "hardware_pass": hardware_pass, "summary_path": sp}


def _collect_routes(run_dir: str) -> List[Dict]:
    spec = [("trials/A_baseline_plan.json", "points", "A plan", "#1c7ed6", True),
            ("trials/A_baseline_execution.json", "samples", "A executed", "#1c7ed6", False),
            ("trials/C_risk_aware_plan.json", "points", "C plan", "#2b8a3e", True),
            ("trials/C_risk_aware_execution.json", "samples", "C executed", "#2b8a3e", False)]
    out = []
    for rel, key, name, color, dashed in spec:
        p = os.path.join(run_dir, rel)
        if not os.path.exists(p):
            continue
        d = json.load(open(p))
        pts = d.get(key) or []
        if key == "samples":
            pts = [(s["x"], s["y"]) for s in pts]
        out.append({"name": name, "points": [tuple(x) for x in pts], "color": color, "dashed": dashed})
    return out


def _write_markdown(run_dir: str, s: Dict) -> None:
    r = s["results"]
    t = r.get("trials", {})
    lines = [f"# RiskGraph live trial: {os.path.basename(run_dir)}", "",
             f"**{s['label']}**", "",
             f"- status: {s['status']}" + (f" (abort: {s['abort_reason']})" if s["abort_reason"] else ""),
             f"- machine checks: {'PASS' if s['machine_pass'] else 'FAIL'}",
             f"- operator attested: {s['operator_attested']}",
             f"- hardware_pass: {s['hardware_pass']}",
             f"- git: {s['git']} (dirty: {s['git_dirty']})", f"- map id: {s['map_id']}",
             f"- database: {s['db_path']}", "", "| # | criterion | result |", "|---|---|---|"]
    lines += [f"| {c['id']} | {c['criterion']} | {'PASS' if c['pass'] else 'FAIL'} |" for c in s["criteria"]]
    lines += ["", "## Routes", "", "| trial | planned corridor | plan length m | plan accumulated risk | "
              "executed corridor | executed m | time s | result |", "|---|---|---|---|---|---|---|---|"]
    for k in ("A_baseline", "C_risk_aware"):
        pl, ex = _get(t, k, "plan") or {}, _get(t, k, "execution") or {}
        lines.append(f"| {k} | {pl.get('side')} | {_f(pl.get('length_m'))} | "
                     f"{_f(_get(pl, 'risk_metrics', 'accumulated_risk'))} | {ex.get('executed_side')} | "
                     f"{_f(ex.get('executed_length_m'))} | {_f(ex.get('execution_time_s'))} | "
                     f"{ex.get('result')} |")
    lines += ["", "Files: manifest.json, preflight.json, trials/*.json, routes_comparison.json, "
              "routes.svg, db/riskgraph_{before,after}.sqlite, bag/, logs/, graph/, operator_notes.txt", ""]
    with open(os.path.join(run_dir, "summary.md"), "w") as fh:
        fh.write("\n".join(lines))


def _f(v) -> str:
    return f"{v:.3f}" if isinstance(v, (int, float)) else str(v)
