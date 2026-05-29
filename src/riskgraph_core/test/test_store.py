import time
import tempfile
from pathlib import Path

import pytest

from riskgraph_core.events import RiskEvent, RiskFactor, FactorCategory
from riskgraph_core.store import RiskStore


def _ev(eid, segment_id, severity, t=None, category=FactorCategory.SLIP, confidence=1.0):
    return RiskEvent(
        event_id=eid,
        position=(0.0, 0.0, 0.0),
        factors=[RiskFactor(category=category, severity=severity, source="test")],
        confidence=confidence,
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


def test_segment_risk_total_scales_with_confidence():
    """The cumulative-risk total is severity weighted by detection confidence.

    A half-confident detection of a 0.8-severity slip should contribute 0.8 * 0.5
    to the total, matching RiskEvent.aggregate_severity(). Every other store test
    pins confidence at 1.0; this locks the < 1.0 path that real (noisy) hardware
    detections will exercise.
    """
    s = RiskStore(":memory:")
    now = time.time()
    s.record_event(_ev("e1", "seg-a", 0.8, t=now, confidence=0.5))
    risk, count, _ = s.segment_risk("seg-a", now=now, decay_half_life_s=0.0)
    assert risk == pytest.approx(0.8 * 0.5)
    assert count == 1


def test_segment_risk_total_uses_max_factor_not_sum():
    """Multi-modal factors on one event do not sum: total uses the max severity.

    Two factors (0.9 SLIP, 0.7 SAFETY) on a single, fully-confident event describe
    one incident seen by two modalities. The total must be 0.9 (the max), never 1.6
    (the sum), per RiskEvent.aggregate_severity's anti-double-count contract.
    """
    s = RiskStore(":memory:")
    now = time.time()
    s.record_event(RiskEvent(
        event_id="e1", position=(0.0, 0.0, 0.0),
        factors=[
            RiskFactor(category=FactorCategory.SLIP, severity=0.9, source="t"),
            RiskFactor(category=FactorCategory.SAFETY, severity=0.7, source="t"),
        ],
        confidence=1.0, timestamp=now, segment_id="seg-a",
    ))
    risk, count, dominant = s.segment_risk("seg-a", now=now, decay_half_life_s=0.0)
    assert risk == pytest.approx(0.9)
    assert count == 1
    # Dominant category is the higher-severity factor.
    assert dominant == FactorCategory.SLIP.value


def test_segment_risk_confidence_and_decay_compound():
    """Confidence and exponential decay multiply together on the total.

    A 1.0-severity, 0.5-confidence event aged exactly one half-life contributes
    1.0 * 0.5 (confidence) * 0.5 (decay) = 0.25.
    """
    s = RiskStore(":memory:")
    now = time.time()
    half_life = 100.0
    s.record_event(_ev("e_old", "seg-a", 1.0, t=now - half_life, confidence=0.5))
    risk, count, _ = s.segment_risk("seg-a", now=now, decay_half_life_s=half_life)
    assert risk == pytest.approx(0.25, abs=1e-9)
    assert count == 1


def test_segment_risk_dominant_category_aggregates_across_events():
    """Dominant category is decided by summed, confidence-weighted severity across events.

    One high-severity SAFETY event plus two lower-severity SLIP events: SLIP wins
    only if its summed contribution exceeds SAFETY's. Here SAFETY=0.9 vs
    SLIP=0.4+0.4=0.8, so SAFETY remains dominant.
    """
    s = RiskStore(":memory:")
    now = time.time()
    s.record_event(_ev("safety", "seg-a", 0.9, t=now, category=FactorCategory.SAFETY))
    s.record_event(_ev("slip1", "seg-a", 0.4, t=now, category=FactorCategory.SLIP))
    s.record_event(_ev("slip2", "seg-a", 0.4, t=now, category=FactorCategory.SLIP))
    _, _, dominant = s.segment_risk("seg-a", now=now, decay_half_life_s=0.0)
    assert dominant == FactorCategory.SAFETY.value


def test_concurrent_reader_sees_atomic_event_writes(tmp_path):
    """A second handle to the same DB must observe an event with all factors at once,
    not a half-written row with zero factors."""
    db = tmp_path / "rg.sqlite"
    writer = RiskStore(str(db))
    reader = RiskStore(str(db))
    # Pre-write some baseline
    writer.record_event(_ev("e1", "seg-a", 0.4))
    # Reader must see the event with its factor
    risk, count, _ = reader.segment_risk("seg-a", now=time.time(), decay_half_life_s=0.0)
    assert risk > 0.0
    assert count == 1
    # And subsequent writes are also visible after commit
    writer.record_event(_ev("e2", "seg-a", 0.6))
    risk2, count2, _ = reader.segment_risk("seg-a", now=time.time(), decay_half_life_s=0.0)
    assert count2 == 2
    assert risk2 > risk
    writer.close()
    reader.close()
