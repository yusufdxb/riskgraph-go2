import time
import pytest

from riskgraph_core.events import RiskEvent, RiskFactor, FactorCategory
from riskgraph_core.segments import RouteSegment, Route
from riskgraph_core.store import RiskStore
from riskgraph_core.scoring import ScoringWeights, score_routes


def _seg(seg_id, length, label=None):
    # Build a unit-direction segment of the requested length so geometry is consistent.
    return RouteSegment(segment_id=seg_id, start=(0.0, 0.0, 0.0),
                        end=(length, 0.0, 0.0), semantic_label=label)


def _ev(eid, seg, sev, cat=FactorCategory.SLIP, t=None):
    return RiskEvent(
        event_id=eid, position=(0.0, 0.0, 0.0),
        factors=[RiskFactor(category=cat, severity=sev, source="test")],
        confidence=1.0,
        timestamp=t if t is not None else time.time(),
        segment_id=seg,
    )


def test_safer_longer_route_beats_shorter_risky_route():
    """The headline regression: when risk is high enough, the longer safe route wins."""
    store = RiskStore(":memory:")
    # Route SHORT goes through "glossy" with several slip events.
    # Route LONG is risk-free but 50% longer.
    for i in range(5):
        store.record_event(_ev(f"slip{i}", "glossy", 0.9, FactorCategory.SLIP))

    short = Route(route_id="short", segments=[_seg("glossy", 4.0)])
    long_ = Route(route_id="long", segments=[_seg("safe-a", 3.0), _seg("safe-b", 3.0)])

    weights = ScoringWeights(geometry=1.0, semantic=0.0, risk=4.0,
                             decay_half_life_s=0.0)
    result = score_routes([short, long_], store, weights, semantic_objective="")
    assert result.chosen_route_id == "long"
    short_score = next(s for s in result.scores if s.route_id == "short")
    long_score = next(s for s in result.scores if s.route_id == "long")
    assert short_score.total_cost > long_score.total_cost


def test_geometry_only_picks_shorter_when_no_risk():
    store = RiskStore(":memory:")
    short = Route(route_id="short", segments=[_seg("a", 2.0)])
    long_ = Route(route_id="long", segments=[_seg("b", 8.0)])
    w = ScoringWeights(geometry=1.0, semantic=0.0, risk=1.0)
    result = score_routes([short, long_], store, w, "")
    assert result.chosen_route_id == "short"


def test_risk_weight_zero_ignores_history_and_picks_shorter():
    store = RiskStore(":memory:")
    for i in range(10):
        store.record_event(_ev(f"e{i}", "shortpath", 1.0))
    short = Route(route_id="short", segments=[_seg("shortpath", 2.0)])
    long_ = Route(route_id="long", segments=[_seg("clean", 6.0)])
    w = ScoringWeights(geometry=1.0, semantic=0.0, risk=0.0)
    result = score_routes([short, long_], store, w, "")
    assert result.chosen_route_id == "short"


def test_semantic_objective_label_match_reduces_cost():
    store = RiskStore(":memory:")
    plain = Route(route_id="plain", segments=[_seg("a", 5.0, label="hallway")])
    matched = Route(route_id="matched", segments=[_seg("b", 5.0, label="kitchen")])
    w = ScoringWeights(geometry=1.0, semantic=10.0, risk=0.0)
    result = score_routes([plain, matched], store, w, semantic_objective="kitchen")
    assert result.chosen_route_id == "matched"


def test_dominant_segment_and_factor_are_populated_on_risky_route():
    store = RiskStore(":memory:")
    for i in range(3):
        store.record_event(_ev(f"e{i}", "trouble", 0.8, FactorCategory.SAFETY))
    r = Route(route_id="r", segments=[_seg("trouble", 2.0), _seg("clean", 2.0)])
    w = ScoringWeights(geometry=1.0, semantic=0.0, risk=2.0)
    result = score_routes([r], store, w, "")
    score = result.scores[0]
    assert "trouble" in score.dominant_segment_ids
    assert FactorCategory.SAFETY.value in score.dominant_factor_categories


def test_decay_reduces_old_event_influence():
    """Old events should weigh less than new events under exponential decay."""
    store = RiskStore(":memory:")
    now = time.time()
    # Old route: many events in the distant past
    for i in range(10):
        store.record_event(_ev(f"old{i}", "stale", 1.0, t=now - 10_000))
    # New route: one fresh event
    store.record_event(_ev("fresh", "fresh", 1.0, t=now))
    stale = Route(route_id="stale", segments=[_seg("stale", 1.0)])
    fresh = Route(route_id="fresh", segments=[_seg("fresh", 1.0)])
    w = ScoringWeights(geometry=1.0, semantic=0.0, risk=2.0, decay_half_life_s=100.0)
    result = score_routes([stale, fresh], store, w, "", now=now)
    # With aggressive decay, stale should look LESS risky than fresh.
    stale_s = next(s for s in result.scores if s.route_id == "stale")
    fresh_s = next(s for s in result.scores if s.route_id == "fresh")
    assert stale_s.risk_cost < fresh_s.risk_cost
