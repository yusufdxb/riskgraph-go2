"""Scenario fixture loading — pure Python, no ROS coupling.

Used by both the offline demo (run as a plain script) and the synthetic
publisher (run as a ROS node) so the same scenarios are exercised in both paths.
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List

from riskgraph_core.events import RiskEvent, RiskFactor, FactorCategory
from riskgraph_core.segments import Route, RouteSegment
from riskgraph_core.scoring import ScoringWeights


@dataclass
class Scenario:
    name: str
    description: str
    segments: Dict[str, RouteSegment]
    routes: List[Route]
    events: List[RiskEvent]
    weights: ScoringWeights
    expected_choice: str
    expected_explanation_keywords: List[str] = field(default_factory=list)


def load_scenario(path: str) -> Scenario:
    raw = json.loads(Path(path).read_text())
    segs = {
        sid: RouteSegment(segment_id=sid,
                          start=tuple(v["start"]), end=tuple(v["end"]),
                          semantic_label=v.get("label"))
        for sid, v in raw["segments"].items()
    }
    routes = [
        Route(route_id=r["route_id"], segments=[segs[s] for s in r["segments"]])
        for r in raw["routes"]
    ]
    now = time.time()
    events = []
    for e in raw["events"]:
        events.append(RiskEvent(
            event_id=e["event_id"],
            position=tuple(segs[e["segment_id"]].start),
            factors=[RiskFactor(
                category=FactorCategory.coerce(e["category"]),
                severity=float(e["severity"]),
                source=e["source"],
                detail=e.get("detail", ""),
            )],
            confidence=float(e.get("confidence", 1.0)),
            timestamp=now - float(e["age_s"]),
            segment_id=e["segment_id"],
        ))
    w = raw.get("weights", {})
    weights = ScoringWeights(
        geometry=float(w.get("geometry", 1.0)),
        semantic=float(w.get("semantic", 1.0)),
        risk=float(w.get("risk", 2.0)),
        decay_half_life_s=float(w.get("decay_half_life_s", 0.0)),
    )
    return Scenario(
        name=raw["name"],
        description=raw["description"],
        segments=segs,
        routes=routes,
        events=events,
        weights=weights,
        expected_choice=raw["expected_choice"],
        expected_explanation_keywords=list(raw.get("expected_explanation_keywords", [])),
    )
