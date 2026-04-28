import pytest
from riskgraph_core.events import RiskFactor, RiskEvent, FactorCategory


def test_factor_severity_clamped():
    f = RiskFactor(category=FactorCategory.SLIP, severity=1.7, source="x")
    assert f.severity == 1.0
    f2 = RiskFactor(category=FactorCategory.SLIP, severity=-0.4, source="x")
    assert f2.severity == 0.0


def test_factor_category_normalized():
    f = RiskFactor(category="slip", severity=0.5, source="x")
    assert f.category == FactorCategory.SLIP


def test_factor_unknown_category_falls_back_to_other():
    f = RiskFactor(category="weirdthing", severity=0.5, source="x")
    assert f.category == FactorCategory.OTHER


def test_event_requires_at_least_one_factor():
    with pytest.raises(ValueError):
        RiskEvent(event_id="abc", position=(0.0, 0.0, 0.0), factors=[], confidence=0.9)


def test_event_dominant_category_is_max_severity():
    factors = [
        RiskFactor(category=FactorCategory.SLIP, severity=0.3, source="a"),
        RiskFactor(category=FactorCategory.SAFETY, severity=0.9, source="b"),
        RiskFactor(category=FactorCategory.AUDIO, severity=0.4, source="c"),
    ]
    ev = RiskEvent(event_id="abc", position=(1.0, 2.0, 0.0), factors=factors, confidence=1.0)
    assert ev.dominant_category() == FactorCategory.SAFETY


def test_event_aggregate_severity_is_max_factor_times_confidence():
    factors = [
        RiskFactor(category=FactorCategory.SLIP, severity=0.4, source="a"),
        RiskFactor(category=FactorCategory.SAFETY, severity=0.8, source="b"),
    ]
    ev = RiskEvent(event_id="x", position=(0, 0, 0), factors=factors, confidence=0.5)
    assert ev.aggregate_severity() == pytest.approx(0.4)
