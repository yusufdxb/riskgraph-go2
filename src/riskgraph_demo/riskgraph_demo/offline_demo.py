"""Offline demo entry point — exercises the full risk model end-to-end without ROS.

Usage:
    python -m riskgraph_demo.offline_demo [SCENARIO_JSON] [OUTPUT_JSON]

Loads a scenario fixture, replays all events into a RiskStore, scores the
candidate routes, generates an explanation, prints a summary, and writes
machine-readable results to OUTPUT_JSON.

Returns nonzero exit if the chosen route does not match `expected_choice`,
making this also usable as a regression check.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Optional

from riskgraph_core.store import RiskStore
from riskgraph_core.scoring import score_routes
from riskgraph_core.explainer import explain_choice

from .scenario import load_scenario


_DEFAULT_FIXTURE = (
    Path(__file__).resolve().parents[1] / "fixtures" / "scenario_glossy_hallway.json"
)


def run_demo(fixture_path: str, output_path: Optional[str] = None) -> int:
    sc = load_scenario(fixture_path)
    print(f"=== RiskGraph-Go2 offline demo: {sc.name} ===")
    print(sc.description)
    print()

    store = RiskStore(":memory:")
    for ev in sc.events:
        store.record_event(ev)
    print(f"loaded {len(sc.events)} risk events into in-memory store")
    print(f"scoring {len(sc.routes)} candidate routes with weights={sc.weights}")
    print()

    result = score_routes(sc.routes, store, sc.weights, semantic_objective="")
    explanation = explain_choice(result, sc.routes, store)

    print("--- per-route scores (lower = better) ---")
    for s in result.scores:
        print(f"  {s.route_id}: total={s.total_cost:7.3f}  geom={s.geometry_cost:5.2f}  "
              f"sem={s.semantic_cost:5.2f}  risk={s.risk_cost:6.3f}  "
              f"dom_segs={s.dominant_segment_ids} dom_cats={s.dominant_factor_categories}")
    print()
    print(f"chosen: {result.chosen_route_id}  (expected: {sc.expected_choice})")
    print(f"explanation: {explanation.text}")
    print(f"evidence event ids: {explanation.evidence_event_ids}")
    print()

    ok_choice = result.chosen_route_id == sc.expected_choice
    ok_keywords = all(
        kw.lower() in explanation.text.lower()
        for kw in sc.expected_explanation_keywords
    )
    overall = ok_choice and ok_keywords
    print(f"choice match:      {'PASS' if ok_choice else 'FAIL'}")
    print(f"explanation match: {'PASS' if ok_keywords else 'FAIL'}")
    print(f"overall:           {'PASS' if overall else 'FAIL'}")

    if output_path:
        out = {
            "scenario": sc.name,
            "weights": vars(sc.weights),
            "scores": [vars(s) for s in result.scores],
            "chosen_route_id": result.chosen_route_id,
            "expected_choice": sc.expected_choice,
            "explanation": {
                "route_id": explanation.route_id,
                "text": explanation.text,
                "evidence_event_ids": explanation.evidence_event_ids,
            },
            "verdict": {
                "choice_match": ok_choice,
                "explanation_match": ok_keywords,
                "overall": overall,
            },
        }
        Path(output_path).write_text(json.dumps(out, indent=2))
        print(f"\nresults written to {output_path}")

    store.close()
    return 0 if overall else 1


def main() -> int:
    fixture = sys.argv[1] if len(sys.argv) > 1 else str(_DEFAULT_FIXTURE)
    output = sys.argv[2] if len(sys.argv) > 2 else "demo_results.json"
    return run_demo(fixture, output)


if __name__ == "__main__":
    raise SystemExit(main())
