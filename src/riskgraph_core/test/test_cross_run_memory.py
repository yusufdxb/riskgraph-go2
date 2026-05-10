"""Cross-run mock-pose memory tests.

These tests synthesize a sequence of pose updates along a known path with a
small number of high-risk patches, push the resulting events through the
SQLite-backed RiskStore, then re-open the store as a "second run" and assert:

  * the high-risk patches are still retrievable;
  * segment-keyed risk is elevated near them;
  * a cold start with no events yields zero risk;
  * an empty pose stream (no events recorded) is benign.

We deliberately do not run a ROS node here. The store is the source of truth
for cross-run persistence; pose -> segment_id is provided by
`segment_for_point`, which is the same call the memory_node makes when an
event arrives without a pre-assigned segment_id.
"""
from __future__ import annotations

import math
import time
from typing import List, Tuple

import pytest

from riskgraph_core.events import RiskEvent, RiskFactor, FactorCategory
from riskgraph_core.segments import RouteSegment, segment_for_point
from riskgraph_core.store import RiskStore


# ---------- fixtures and helpers ----------


@pytest.fixture
def hallway_segments() -> List[RouteSegment]:
    """Five 10 m segments laid end-to-end along +x. Mimics a hallway split into
    cells the planner can score independently."""
    return [
        RouteSegment(
            segment_id=f"seg-{i}",
            start=(float(i * 10), 0.0, 0.0),
            end=(float((i + 1) * 10), 0.0, 0.0),
            semantic_label=None,
        )
        for i in range(5)
    ]


def _synthesize_pose_path(
    n_poses: int = 100,
    x_start: float = 0.0,
    x_end: float = 50.0,
    y_jitter: float = 0.0,
) -> List[Tuple[float, float, float]]:
    """Sample n_poses (x, y, z) tuples linearly between x_start and x_end."""
    if n_poses <= 1:
        return [(x_start, 0.0, 0.0)]
    dx = (x_end - x_start) / (n_poses - 1)
    return [
        (x_start + i * dx, (y_jitter if (i % 7 == 0) else 0.0), 0.0)
        for i in range(n_poses)
    ]


def _risk_event_at(
    eid: str,
    pose: Tuple[float, float, float],
    segment_id: str,
    severity: float,
    t: float,
    category: FactorCategory = FactorCategory.SLIP,
) -> RiskEvent:
    return RiskEvent(
        event_id=eid,
        position=pose,
        factors=[RiskFactor(category=category, severity=severity, source="pose-mock")],
        confidence=1.0,
        timestamp=t,
        segment_id=segment_id,
    )


def _record_run(
    store: RiskStore,
    segments: List[RouteSegment],
    poses: List[Tuple[float, float, float]],
    risky_indices: List[int],
    severity: float,
    base_time: float,
    eid_prefix: str,
) -> int:
    """Walk poses, emit a RiskEvent for each pose in `risky_indices`, joined to
    the nearest segment. Returns the number of events written."""
    written = 0
    for i in risky_indices:
        if not (0 <= i < len(poses)):
            continue
        pose = poses[i]
        seg = segment_for_point(segments, pose)
        assert seg is not None, "pose path should always project to a segment"
        ev = _risk_event_at(
            eid=f"{eid_prefix}-{i:03d}",
            pose=pose,
            segment_id=seg.segment_id,
            severity=severity,
            t=base_time + i * 0.1,
        )
        store.record_event(ev)
        written += 1
    return written


# ---------- core mock-pose behaviour ----------


def test_mock_pose_path_assigns_events_to_correct_segments(hallway_segments):
    """100 poses spanning [0, 50] m → events at three risky indices land on
    the segments those poses geometrically belong to."""
    store = RiskStore(":memory:")
    poses = _synthesize_pose_path(n_poses=100, x_start=0.0, x_end=50.0)
    # Risky patches at x≈5, x≈25, x≈45 → seg-0, seg-2, seg-4.
    risky_indices = [10, 50, 90]
    base_t = 1_700_000_000.0
    n_written = _record_run(
        store, hallway_segments, poses, risky_indices,
        severity=0.8, base_time=base_t, eid_prefix="run1",
    )
    assert n_written == 3

    # Each of the three segments should now hold exactly one event.
    for seg_id in ("seg-0", "seg-2", "seg-4"):
        events = store.events_for_segment(seg_id)
        assert len(events) == 1, f"{seg_id} should have 1 event"
        assert events[0].factors[0].severity == pytest.approx(0.8)

    # Untouched segments stay clean.
    for seg_id in ("seg-1", "seg-3"):
        assert store.events_for_segment(seg_id) == []


def test_cold_start_yields_zero_risk_for_all_segments(hallway_segments):
    """A fresh in-memory store with no events records zero risk on every
    segment we ask about."""
    store = RiskStore(":memory:")
    for seg in hallway_segments:
        risk, count, dominant = store.segment_risk(
            seg.segment_id, now=time.time(), decay_half_life_s=0.0,
        )
        assert risk == 0.0
        assert count == 0
        assert dominant == ""


def test_empty_pose_stream_does_not_raise(hallway_segments):
    """Walking 0 poses must not crash and must leave the store empty."""
    store = RiskStore(":memory:")
    n_written = _record_run(
        store, hallway_segments, poses=[], risky_indices=[],
        severity=0.5, base_time=time.time(), eid_prefix="empty",
    )
    assert n_written == 0
    for seg in hallway_segments:
        assert store.events_for_segment(seg.segment_id) == []


def test_warm_start_second_run_observes_first_run_risk(tmp_path, hallway_segments):
    """Run 1 writes risky patches; run 2 opens the same DB and immediately sees
    the risk before recording anything new."""
    db = tmp_path / "warmstart.sqlite"
    base_t = 1_700_000_000.0

    # --- run 1 ---
    s1 = RiskStore(str(db))
    poses = _synthesize_pose_path(n_poses=100, x_start=0.0, x_end=50.0)
    _record_run(
        s1, hallway_segments, poses, risky_indices=[10, 50, 90],
        severity=0.8, base_time=base_t, eid_prefix="run1",
    )
    s1.close()

    # --- run 2: open as a fresh handle ---
    s2 = RiskStore(str(db))
    # Risky segments must come back elevated.
    for seg_id in ("seg-0", "seg-2", "seg-4"):
        risk, count, dominant = s2.segment_risk(
            seg_id, now=base_t + 100.0, decay_half_life_s=0.0,
        )
        assert count == 1, f"{seg_id} should retain its event from run 1"
        assert risk == pytest.approx(0.8)
        assert dominant == FactorCategory.SLIP.value
    # Quiet segments stay quiet.
    for seg_id in ("seg-1", "seg-3"):
        risk, count, _ = s2.segment_risk(
            seg_id, now=base_t + 100.0, decay_half_life_s=0.0,
        )
        assert count == 0
        assert risk == 0.0
    s2.close()


def test_persistence_across_three_consecutive_runs(tmp_path, hallway_segments):
    """Risk should accumulate across three consecutive write sessions.
    Same DB file, three open/close cycles, same risky segment."""
    db = tmp_path / "threeruns.sqlite"
    base_t = 1_700_000_000.0
    poses = _synthesize_pose_path(n_poses=100, x_start=0.0, x_end=50.0)

    for run_idx in range(3):
        s = RiskStore(str(db))
        _record_run(
            s, hallway_segments, poses,
            risky_indices=[50],  # always seg-2
            severity=0.6,
            base_time=base_t + run_idx * 1000.0,
            eid_prefix=f"run{run_idx}",
        )
        s.close()

    final = RiskStore(str(db))
    risk, count, dominant = final.segment_risk(
        "seg-2", now=base_t + 5000.0, decay_half_life_s=0.0,
    )
    # Each run contributed one event → 3 events on seg-2.
    assert count == 3
    # Aggregate severity = sum of per-event severities (no decay).
    assert risk == pytest.approx(0.6 + 0.6 + 0.6)
    assert dominant == FactorCategory.SLIP.value
    # Non-risky segments still empty.
    for seg_id in ("seg-0", "seg-1", "seg-3", "seg-4"):
        risk_q, count_q, _ = final.segment_risk(
            seg_id, now=base_t + 5000.0, decay_half_life_s=0.0,
        )
        assert count_q == 0
        assert risk_q == 0.0
    final.close()


def test_risk_only_elevated_near_recorded_patches(hallway_segments):
    """Only segments containing the risky-patch poses should have non-zero
    risk; geometrically-distant segments must remain clean."""
    store = RiskStore(":memory:")
    poses = _synthesize_pose_path(n_poses=200, x_start=0.0, x_end=50.0)
    # Two risky patches at x≈10 (seg-1) and x≈30 (seg-3).
    risky_indices = [40, 120]
    base_t = 1_700_000_000.0
    _record_run(
        store, hallway_segments, poses, risky_indices,
        severity=0.7, base_time=base_t, eid_prefix="patches",
    )
    elevated = []
    quiet = []
    for seg in hallway_segments:
        risk, _, _ = store.segment_risk(
            seg.segment_id, now=base_t + 10.0, decay_half_life_s=0.0,
        )
        (elevated if risk > 0 else quiet).append(seg.segment_id)
    assert sorted(elevated) == ["seg-1", "seg-3"]
    assert sorted(quiet) == ["seg-0", "seg-2", "seg-4"]


def test_warm_start_decay_drops_old_risk(tmp_path, hallway_segments):
    """After a long gap, decay-on-read should reduce a stale event's
    contribution well below 1.0 even though the event is still in the DB."""
    db = tmp_path / "decay.sqlite"
    poses = _synthesize_pose_path(n_poses=100, x_start=0.0, x_end=50.0)
    base_t = 1_700_000_000.0

    s1 = RiskStore(str(db))
    _record_run(
        s1, hallway_segments, poses, risky_indices=[50],  # seg-2
        severity=1.0, base_time=base_t, eid_prefix="old",
    )
    s1.close()

    s2 = RiskStore(str(db))
    # 10 half-lives later → decayed weight ≈ 1/1024
    half_life = 100.0
    later = base_t + 1000.0
    risk, count, _ = s2.segment_risk(
        "seg-2", now=later, decay_half_life_s=half_life,
    )
    assert count == 1
    assert risk < 0.01, f"stale event must decay well below 1.0, got {risk}"
    # Without decay, the same query returns the full severity.
    risk_no_decay, _, _ = s2.segment_risk(
        "seg-2", now=later, decay_half_life_s=0.0,
    )
    assert risk_no_decay == pytest.approx(1.0)
    s2.close()


def test_pose_path_with_lateral_jitter_still_joins_correctly(hallway_segments):
    """Real poses are noisy. A small lateral offset must not flip a pose to a
    different segment when the segments are end-to-end along x."""
    poses = _synthesize_pose_path(
        n_poses=50, x_start=0.0, x_end=50.0, y_jitter=0.2,
    )
    for pose in poses:
        seg = segment_for_point(hallway_segments, pose)
        assert seg is not None
        # x position must still bucket into the right seg-{i}
        expected_idx = min(int(pose[0] // 10), 4)
        assert seg.segment_id == f"seg-{expected_idx}", (
            f"pose {pose} expected seg-{expected_idx}, got {seg.segment_id}"
        )


def test_warm_start_dominant_category_survives_restart(tmp_path, hallway_segments):
    """The dominant factor reported for a segment must be stable across a
    restart (it's derived from the on-disk factor rows)."""
    db = tmp_path / "dom.sqlite"
    base_t = 1_700_000_000.0
    s1 = RiskStore(str(db))
    s1.record_event(_risk_event_at(
        eid="ev_safety", pose=(15.0, 0.0, 0.0), segment_id="seg-1",
        severity=0.9, t=base_t, category=FactorCategory.SAFETY,
    ))
    s1.record_event(_risk_event_at(
        eid="ev_slip", pose=(15.0, 0.0, 0.0), segment_id="seg-1",
        severity=0.4, t=base_t, category=FactorCategory.SLIP,
    ))
    s1.close()

    s2 = RiskStore(str(db))
    risk, count, dominant = s2.segment_risk(
        "seg-1", now=base_t + 1.0, decay_half_life_s=0.0,
    )
    assert count == 2
    assert dominant == FactorCategory.SAFETY.value
    assert risk > 0
    s2.close()


def test_evidence_for_segment_returns_top_severity_after_restart(tmp_path, hallway_segments):
    """Evidence ranking (severity-desc) must survive a restart so the
    explainer sees the same top-N events on a warm start."""
    db = tmp_path / "evidence.sqlite"
    base_t = 1_700_000_000.0
    severities = [0.2, 0.9, 0.4, 0.8, 0.1]

    s1 = RiskStore(str(db))
    for i, sev in enumerate(severities):
        s1.record_event(_risk_event_at(
            eid=f"ev{i}", pose=(25.0, 0.0, 0.0), segment_id="seg-2",
            severity=sev, t=base_t + i,
        ))
    s1.close()

    s2 = RiskStore(str(db))
    top = s2.evidence_for_segment(
        "seg-2", max_events=3, now=base_t + 100.0, decay_half_life_s=0.0,
    )
    assert [ev.event_id for ev in top] == ["ev1", "ev3", "ev2"]
    s2.close()


def test_long_pose_path_with_many_events_persists_atomically(tmp_path, hallway_segments):
    """Stress: 30 events spread across all 5 segments, then a restart must see
    every event with its factors intact."""
    db = tmp_path / "stress.sqlite"
    base_t = 1_700_000_000.0
    poses = _synthesize_pose_path(n_poses=300, x_start=0.0, x_end=50.0)
    risky_indices = list(range(0, 300, 10))  # 30 events, evenly spread

    s1 = RiskStore(str(db))
    n = _record_run(
        s1, hallway_segments, poses, risky_indices,
        severity=0.5, base_time=base_t, eid_prefix="stress",
    )
    assert n == 30
    s1.close()

    s2 = RiskStore(str(db))
    total_count = 0
    for seg in hallway_segments:
        events = s2.events_for_segment(seg.segment_id)
        for ev in events:
            assert len(ev.factors) == 1, f"event {ev.event_id} lost its factor"
            assert ev.factors[0].source == "pose-mock"
        total_count += len(events)
    assert total_count == 30
    s2.close()
