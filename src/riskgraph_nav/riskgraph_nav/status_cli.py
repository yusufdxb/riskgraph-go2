"""Print RiskGraph database status without ROS (read-only; never creates a file).

    ros2 run riskgraph_nav riskgraph_status [--db-tag trial] [--store-path /abs/file.sqlite] [--live]

Shows: absolute DB path, schema version, map identity (and whether it matches
the experiment), evidence class, incident / quarantine counts, active risk
entries, last observation time. ``--live`` also reads /riskgraph/status and
/riskgraph/localization/status from the running graph.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time


def db_report(store_path: str, expected_map_id: str, field_params) -> dict:
    from riskgraph_core.risk_field import RiskField
    from riskgraph_core.store import RiskStore, StoreError
    if not os.path.exists(store_path):
        return {"db_path": store_path, "exists": False}
    try:
        with RiskStore(store_path, readonly=True) as s:
            st = s.status()
            field = RiskField.from_events(s.all_events(frame_id="map"), field_params, now=time.time())
            st["events"] = [{"event_id": e.event_id, "x": round(e.position[0], 3),
                             "y": round(e.position[1], 3), "provenance": e.provenance.value,
                             "run_mode": e.run_mode, "timestamp": e.timestamp}
                            for e in s.all_events()]
    except StoreError as exc:
        return {"db_path": store_path, "exists": True, "error": str(exc)}
    st["exists"] = True
    st["active_risk_entries"] = len(field.bumps)
    st["map_id_matches_experiment"] = st.get("map_id") == expected_map_id
    st["expected_map_id"] = expected_map_id
    return st


def main(argv=None) -> int:
    from .paths import DEFAULT_DB_ROOT, DEFAULT_DB_TAG, resolve
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--experiment", default=None)
    ap.add_argument("--store-path", default=None)
    ap.add_argument("--db-root", default=DEFAULT_DB_ROOT)
    ap.add_argument("--db-tag", default=DEFAULT_DB_TAG)
    ap.add_argument("--live", action="store_true", help="also read the running graph's status topics")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    r = resolve(a.experiment, a.store_path, a.db_root, a.db_tag)
    rep = {"database": db_report(r.store_path, r.map_id, r.experiment.risk_params)}
    if a.live:
        import rclpy
        from .ros_graph import GraphProbe
        rclpy.init()
        p = GraphProbe("riskgraph_status_cli")
        try:
            rep["riskgraph_status"] = p.latest_json("/riskgraph/status")
            rep["localization_status"] = p.latest_json("/riskgraph/localization/status")
        finally:
            p.close()
            rclpy.try_shutdown()
    if a.json:
        print(json.dumps(rep, indent=1, default=str))
        return 0
    d = rep["database"]
    print(f"database:            {d.get('db_path')}")
    if not d.get("exists"):
        print("                     (does not exist yet; the memory node creates it on first start)")
    elif d.get("error"):
        print(f"ERROR:               {d['error']}")
    else:
        last = d.get("last_event_timestamp")
        print(f"schema version:      v{d.get('schema_version')}")
        print(f"map id:              {d.get('map_id')}  "
              f"({'matches' if d.get('map_id_matches_experiment') else 'DOES NOT MATCH'} experiment "
              f"{d.get('expected_map_id')})")
        print(f"evidence class:      {d.get('evidence_class')}")
        print(f"incidents stored:    {d.get('incident_count')}   provenance: {d.get('provenance_counts')}")
        print(f"quarantined:         {d.get('quarantined_count')}   reasons: {d.get('quarantine_reasons')}")
        print(f"active risk entries: {d.get('active_risk_entries')}")
        print(f"segments with events:{d.get('segments_with_events'):>3}")
        print(f"last observation:    {last} ({time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(last)) if last else '-'})")
        for e in d.get("events", []):
            print(f"  {e['event_id']}  ({e['x']}, {e['y']})  {e['provenance']}  {e['run_mode']}")
    for k in ("riskgraph_status", "localization_status"):
        if k in rep:
            v = rep[k]
            print(f"{k}: " + ("NOT RECEIVED" if v is None else json.dumps(
                {x: v.get(x) for x in ("db_path", "instance_id", "state", "incident_count",
                                       "grid_state", "localization_valid", "robot_pose_map") if x in v})))
    return 0 if d.get("exists") and not d.get("error") and d.get("map_id_matches_experiment") else 1


if __name__ == "__main__":
    sys.exit(main())
