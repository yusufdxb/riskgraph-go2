import time
import tempfile
from pathlib import Path

import pytest

from riskgraph_core.events import RiskEvent, RiskFactor, FactorCategory
from riskgraph_core.store import RiskStore


def _ev(eid, segment_id, severity, t=None, category=FactorCategory.SLIP):
    return RiskEvent(
        event_id=eid,
        position=(0.0, 0.0, 0.0),
        factors=[RiskFactor(category=category, severity=severity, source="test")],
        confidence=1.0,
        timestamp=t if t is not None else time.time(),
        segment_id=segment_id,
    )


def test_in_memory_store_round_trip():
    s = RiskStore(":memory:")
    s.record_event(_ev("e1", "seg-a", 0.5))
    s.record_event(_ev("e2", "seg-a", 0.7))
    s.record_event(_ev("e3", "seg-b", 0.2))
    events = list(s.events_for_segment("seg-a"))
    assert len(events) == 2
    ids = sorted(e.event_id for e in events)
    assert ids == ["e1", "e2"]


def test_persistence_across_handles_via_file(tmp_path):
    db_path = tmp_path / "rg.sqlite"
    s1 = RiskStore(str(db_path))
    s1.record_event(_ev("e1", "seg-a", 0.6))
    s1.close()

    s2 = RiskStore(str(db_path))
    events = list(s2.events_for_segment("seg-a"))
    assert len(events) == 1
    assert events[0].event_id == "e1"


def test_segment_risk_no_decay_sums_severities():
    s = RiskStore(":memory:")
    now = time.time()
    s.record_event(_ev("e1", "seg-a", 0.4, t=now))
    s.record_event(_ev("e2", "seg-a", 0.6, t=now))
    risk, count, dominant = s.segment_risk("seg-a", now=now, decay_half_life_s=0.0)
    assert risk == pytest.approx(1.0)
    assert count == 2
    assert dominant == FactorCategory.SLIP.value


def test_segment_risk_with_decay_drops_old_events():
    s = RiskStore(":memory:")
    now = time.time()
    s.record_event(_ev("e_old", "seg-a", 1.0, t=now - 1000.0))
    s.record_event(_ev("e_new", "seg-a", 1.0, t=now))
    half_life = 100.0  # 1000s = 10 half-lives -> ~1/1024 of severity
    risk, count, _ = s.segment_risk("seg-a", now=now, decay_half_life_s=half_life)
    # New event contributes ~1.0; old event contributes ~0.001
    assert 1.0 < risk < 1.01
    assert count == 2  # count is raw, decay applies only to severity


def test_dominant_factor_picks_highest_total():
    s = RiskStore(":memory:")
    now = time.time()
    s.record_event(_ev("e1", "seg-a", 0.6, t=now, category=FactorCategory.SLIP))
    s.record_event(_ev("e2", "seg-a", 0.9, t=now, category=FactorCategory.SAFETY))
    _, _, dominant = s.segment_risk("seg-a", now=now, decay_half_life_s=0.0)
    assert dominant == FactorCategory.SAFETY.value


def test_unknown_segment_yields_zero_risk():
    s = RiskStore(":memory:")
    risk, count, dominant = s.segment_risk("does-not-exist", now=time.time(), decay_half_life_s=0.0)
    assert risk == 0.0
    assert count == 0
    assert dominant == ""
