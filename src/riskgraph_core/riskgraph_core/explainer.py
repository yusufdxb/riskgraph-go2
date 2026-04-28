"""Deterministic, evidence-grounded explanations for route choices.

The explainer does NOT call an LLM. It composes a short paragraph from
templates and cites the actual RiskEvent.event_id values that drove the
decision, so a reviewer can audit the explanation against the persistent log.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

from .scoring import RouteScoreResult
from .segments import Route
from .store import RiskStore


@dataclass
class ExplanationTemplate:
    avoid_risky: str = (
        "Chose route {chosen} because the alternative passed through "
        "{avoided_seg} where {n_events} prior {factor_human} {events_verb} "
        "been recorded. Going with the safer path."
    )
    shorter_clean: str = (
        "Chose route {chosen} — it is the shorter option ({chosen_length:.1f} m) "
        "and no prior risk events are recorded on its segments."
    )
    semantic_match: str = (
        "Chose route {chosen} because it passes through {label}, which matches "
        "the requested objective '{objective}'."
    )
    fallback: str = "Chose route {chosen} based on combined geometry, semantic, and risk cost."


@dataclass
class _Explanation:
    route_id: str
    text: str
    evidence_event_ids: List[str] = field(default_factory=list)


_FACTOR_HUMAN = {
    "SLIP": "slip",
    "SAFETY": "safety alert",
    "DEPTH": "depth hazard",
    "AUDIO": "audio anomaly",
    "FAULT": "system fault",
    "HUMAN": "human-related hazard",
    "COLLISION": "near-collision",
    "OTHER": "risk event",
}


def _factor_human(category: str, plural: bool) -> str:
    base = _FACTOR_HUMAN.get(category, "risk event")
    if plural and not base.endswith("s"):
        return base + "s"
    return base


def explain_choice(
    result: RouteScoreResult,
    candidates: List[Route],
    store: RiskStore,
    semantic_objective: str = "",
    template: Optional[ExplanationTemplate] = None,
) -> _Explanation:
    if not result.scores or not candidates:
        return _Explanation(route_id="", text="No candidate routes provided.")
    template = template or ExplanationTemplate()

    chosen = result.chosen_route_id
    chosen_score = next(s for s in result.scores if s.route_id == chosen)
    chosen_route = next(r for r in candidates if r.route_id == chosen)
    others = [s for s in result.scores if s.route_id != chosen]

    # Case 1: an alternative had material risk; the chosen one avoided it.
    if others:
        worst_other = max(others, key=lambda s: s.risk_cost)
        if (
            worst_other.risk_cost > 0
            and worst_other.risk_cost > chosen_score.risk_cost
            and worst_other.dominant_segment_ids
        ):
            seg_id = worst_other.dominant_segment_ids[0]
            evidence = store.evidence_for_segment(seg_id, max_events=3)
            n = len(evidence)
            cat = (worst_other.dominant_factor_categories[0]
                   if worst_other.dominant_factor_categories else "OTHER")
            text = template.avoid_risky.format(
                chosen=chosen,
                avoided_seg=seg_id,
                n_events=n if n > 0 else "several",
                factor_human=_factor_human(cat, plural=(n != 1)),
                events_verb="have" if n != 1 else "has",
            )
            return _Explanation(
                route_id=chosen,
                text=text,
                evidence_event_ids=[e.event_id for e in evidence],
            )

    # Case 2: semantic match drove the choice.
    if (
        semantic_objective
        and chosen_score.semantic_cost == 0.0
        and any(s.semantic_cost > 0.0 for s in others)
    ):
        labels = chosen_route.labels()
        label = next((lbl for lbl in labels if semantic_objective.lower() in lbl.lower()),
                     labels[0] if labels else semantic_objective)
        text = template.semantic_match.format(
            chosen=chosen, label=label, objective=semantic_objective
        )
        return _Explanation(route_id=chosen, text=text)

    # Case 3: geometry-only — chosen is shorter and no risk anywhere.
    if (
        chosen_score.risk_cost == 0.0
        and all(s.risk_cost == 0.0 for s in result.scores)
    ):
        text = template.shorter_clean.format(
            chosen=chosen,
            chosen_length=chosen_route.total_length_m,
        )
        return _Explanation(route_id=chosen, text=text)

    # Fallback.
    text = template.fallback.format(chosen=chosen)
    return _Explanation(route_id=chosen, text=text)
