"""Replay parity: does replaying a trial bag through the SAME RiskGraph code
reproduce the stored events and the risk grid Nav2 planned with?

    ros2 run riskgraph_nav riskgraph_replay_check \\
        --original <run_dir>/db/riskgraph_after.sqlite --replayed <replay db>

Compares event ids, map positions (to 1 mm) and provenance, and rasterizes
both databases with the experiment's risk parameters: the grids must be
identical. The replayed database is evidence class ``replay`` and can never
be opened as a live database. Exit 0 = parity.
"""
from __future__ import annotations

import argparse
import json
import math
import sys


def compare(original: str, replayed: str, experiment=None) -> dict:
    from riskgraph_core.map_identity import describe_map
    from riskgraph_core.risk_field import RiskField
    from riskgraph_core.store import RiskStore
    from .paths import resolve
    r = resolve(experiment)
    info = describe_map(r.experiment.map_yaml).grid_info()
    with RiskStore(original, readonly=True) as a, RiskStore(replayed, readonly=True) as b:
        ea = {e.event_id: e for e in a.all_events(frame_id="map")}
        eb = {e.event_id: e for e in b.all_events(frame_id="map")}
        sa, sb = a.status(), b.status()
    missing = sorted(set(ea) - set(eb))
    extra = sorted(set(eb) - set(ea))
    pos_err = {k: math.hypot(ea[k].position[0] - eb[k].position[0],
                             ea[k].position[1] - eb[k].position[1]) for k in set(ea) & set(eb)}
    prov = {k: (ea[k].provenance.value, eb[k].provenance.value) for k in set(ea) & set(eb)
            if ea[k].provenance != eb[k].provenance}
    ga = RiskField.from_events(ea.values(), r.experiment.risk_params, now=0).rasterize(info)
    gb = RiskField.from_events(eb.values(), r.experiment.risk_params, now=0).rasterize(info)
    diff = sum(1 for x, y in zip(ga, gb) if x != y)
    out = {"original": original, "replayed": replayed,
           "original_class": sa.get("evidence_class"), "replayed_class": sb.get("evidence_class"),
           "same_map_id": sa.get("map_id") == sb.get("map_id"),
           "events_original": len(ea), "events_replayed": len(eb),
           "missing_in_replay": missing, "extra_in_replay": extra,
           "max_position_error_m": max(pos_err.values()) if pos_err else 0.0,
           "provenance_mismatch": prov, "grid_cells_different": diff}
    out["parity"] = bool(not missing and not extra and out["max_position_error_m"] < 1e-3
                         and not prov and diff == 0 and out["same_map_id"]
                         and out["replayed_class"] == "replay")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--original", required=True)
    ap.add_argument("--replayed", required=True)
    ap.add_argument("--experiment", default=None)
    a = ap.parse_args(argv)
    out = compare(a.original, a.replayed, a.experiment)
    print(json.dumps(out, indent=1))
    print("REPLAY PARITY:", "PASS" if out["parity"] else "FAIL", "(replay evidence is never hardware evidence)")
    return 0 if out["parity"] else 1


if __name__ == "__main__":
    sys.exit(main())
