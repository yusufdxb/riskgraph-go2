import time
import pytest

from riskgraph_core.events import RiskEvent, RiskFactor, FactorCategory
from riskgraph_core.segments import RouteSegment, Route
from riskgraph_core.store import RiskStore
from riskgraph_core.scoring import ScoringWeights, score_routes
from riskgraph_core.explainer import explain_choice


def _seg(sid, length, label=None):
    return RouteSegment(segment_id=sid, start=(0.0, 0.0, 0.0),
                        end=(length, 0.0, 0.0), semantic_label=label)


def _ev(eid, seg, sev=0.9, cat=FactorCategory.SLIP):
    return RiskEvent(
        event_id=eid, position=(0.0, 0.0, 0.0),
        factors=[RiskFactor(category=cat, severity=sev, source="test")],
        confidence=1.0, timestamp=time.time(), segment_id=seg,
    )


def test_explanation_cites_dominant_factor_when_avoiding():
    store = RiskStore(":memory:")
    for i in range(4):
        store.record_event(_ev(f"e{i}", "glossy", cat=FactorCategory.SLIP))
    short = Route(route_id="short", segments=[_seg("glossy", 4.0)])
    long_ = Route(route_id="long", segments=[_seg("safe1", 3.0), _seg("safe2", 3.0)])

    weights = ScoringWeights(geometry=1.0, semantic=0.0, risk=4.0)
    result = score_routes([short, long_], store, weights, "")
    explanation = explain_choice(result, [short, long_], store)
    assert explanation.route_id == "long"
    assert "slip" in explanation.text.lower()
    # The explanation must cite at least one event that actually drove the decision.
    assert len(explanation.evidence_event_ids) >= 1
    cited = set(explanation.evidence_event_ids)
    # All cited ids must come from the segment that was avoided.
    for cid in cited:
        assert cid.startswith("e")


def test_explanation_cites_geometry_when_no_risk():
    store = RiskStore(":memory:")
    short = Route(route_id="short", segments=[_seg("a", 2.0)])
    long_ = Route(route_id="long", segments=[_seg("b", 8.0)])
    weights = ScoringWeights(geometry=1.0, semantic=0.0, risk=1.0)
    result = score_routes([short, long_], store, weights, "")
    explanation = explain_choice(result, [short, long_], store)
    assert explanation.route_id == "short"
    assert "shorter" in explanation.text.lower() or "distance" in explanation.text.lower()
    assert explanation.evidence_event_ids == []


def test_explanation_mentions_semantic_objective_when_relevant():
    store = RiskStore(":memory:")
    plain = Route(route_id="plain", segments=[_seg("a", 5.0, label="hallway")])
    matched = Route(route_id="matched", segments=[_seg("b", 5.0, label="kitchen")])
    weights = ScoringWeights(geometry=1.0, semantic=10.0, risk=0.0)
    result = score_routes([plain, matched], store, weights, semantic_objective="kitchen")
    explanation = explain_choice(result, [plain, matched], store,
                                 semantic_objective="kitchen")
    assert explanation.route_id == "matched"
    assert "kitchen" in explanation.text.lower()
