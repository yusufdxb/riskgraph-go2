#!/usr/bin/env python3
"""Read demo_results.json and emit a CSV row-per-route summary on stdout.

Usage:
    python3 scripts/export_demo_results.py [demo_results.json] [output.csv]
"""
import csv
import json
import sys
from pathlib import Path


def main() -> int:
    in_path = Path(sys.argv[1] if len(sys.argv) > 1 else "demo_results.json")
    out_path = Path(sys.argv[2]) if len(sys.argv) > 2 else None
    data = json.loads(in_path.read_text())

    rows = []
    for s in data["scores"]:
        rows.append({
            "scenario": data["scenario"],
            "route_id": s["route_id"],
            "total_cost": f"{s['total_cost']:.4f}",
            "geometry_cost": f"{s['geometry_cost']:.4f}",
            "semantic_cost": f"{s['semantic_cost']:.4f}",
            "risk_cost": f"{s['risk_cost']:.4f}",
            "dominant_segments": ";".join(s["dominant_segment_ids"]),
            "dominant_factors": ";".join(s["dominant_factor_categories"]),
            "chosen": "1" if s["route_id"] == data["chosen_route_id"] else "0",
        })

    fieldnames = list(rows[0].keys())
    if out_path:
        with out_path.open("w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=fieldnames)
            w.writeheader()
            w.writerows(rows)
        print(f"wrote {out_path}")
    else:
        w = csv.DictWriter(sys.stdout, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
