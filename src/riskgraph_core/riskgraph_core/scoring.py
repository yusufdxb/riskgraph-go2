"""Route scoring: geometry + semantic + risk + decay.

`score_routes` is the single entry point used by the planner node. It is
deterministic: same inputs → same outputs. Lower `total_cost` means preferred.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import List, Optional

from .segments import Route
from .store import RiskStore


@dataclass
class ScoringWeights:
    geometry: float = 1.0
    semantic: float = 1.0
    risk: float = 2.0
    decay_half_life_s: float = 0.0  # 0 = no decay


@dataclass
class _PerRouteScore:
    route_id: str
    total_cost: float
    geometry_cost: float
    semantic_cost: float
    risk_cost: float
    dominant_segment_ids: List[str] = field(default_factory=list)
    dominant_factor_categories: List[str] = field(default_factory=list)


@dataclass
class RouteScoreResult:
    scores: List[_PerRouteScore]
    chosen_route_id: str


def _semantic_penalty(route: Route, objective: str) -> float:
    """Lower is better. Returns 1.0 if no segment matches the objective, 0.0 if any does.

    Intentionally simple for the MVP: semantic objectives are short labels
    like "kitchen" or "exit". A future version can plug in a CLIP score over
    upstream SemanticDetection embeddings; the planner contract stays the same.
    """
    if not objective:
        return 0.0
    needle = objective.lower()
    for label in route.labels():
        if label and needle in label.lower():
            return 0.0
    return 1.0


def score_routes(
    candidates: List[Route],
    store: RiskStore,
    weights: ScoringWeights,
    semantic_objective: str = "",
    now: Optional[float] = None,
) -> RouteScoreResult:
    if not candidates:
        return RouteScoreResult(scores=[], chosen_route_id="")
    if now is None:
        now = time.time()

    out: List[_PerRouteScore] = []
    for route in candidates:
        geometry_cost = weights.geometry * route.total_length_m
        semantic_cost = weights.semantic * _semantic_penalty(route, semantic_objective)

        per_segment_risk: List[tuple] = []  # (segment_id, risk, dominant_category)
        for seg in route.segments:
            risk, _count, dominant = store.segment_risk(
                seg.segment_id, now=now,
                decay_half_life_s=weights.decay_half_life_s,
            )
            per_segment_risk.append((seg.segment_id, risk, dominant))
        risk_total = sum(r for _, r, _ in per_segment_risk)
        risk_cost = weights.risk * risk_total

        dominant_segments: List[str] = []
        dominant_cats: List[str] = []
        if risk_total > 0:
            ranked = sorted(per_segment_risk, key=lambda t: t[1], reverse=True)
            for seg_id, r, cat in ranked:
                if r <= 0:
                    break
                dominant_segments.append(seg_id)
                if cat and cat not in dominant_cats:
                    dominant_cats.append(cat)
                if len(dominant_segments) >= 3:
                    break

        total = geometry_cost + semantic_cost + risk_cost
        out.append(_PerRouteScore(
            route_id=route.route_id,
            total_cost=total,
            geometry_cost=geometry_cost,
            semantic_cost=semantic_cost,
            risk_cost=risk_cost,
            dominant_segment_ids=dominant_segments,
            dominant_factor_categories=dominant_cats,
        ))

    chosen = min(out, key=lambda s: s.total_cost)
    return RouteScoreResult(scores=out, chosen_route_id=chosen.route_id)
