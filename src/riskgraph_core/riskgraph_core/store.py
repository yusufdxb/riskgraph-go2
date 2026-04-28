"""SQLite-backed persistent risk store.

The schema is intentionally narrow: one row per event, one row per
(event, factor) pair. Queries are segment-keyed and apply optional
exponential decay at read time, so the write path stays cheap.
"""
from __future__ import annotations

import json
import math
import sqlite3
import time
from collections import defaultdict
from typing import Iterable, List, Optional, Tuple

from .events import RiskEvent, RiskFactor, FactorCategory

_SCHEMA = """
CREATE TABLE IF NOT EXISTS risk_event (
    event_id    TEXT PRIMARY KEY,
    timestamp   REAL NOT NULL,
    position_x  REAL NOT NULL,
    position_y  REAL NOT NULL,
    position_z  REAL NOT NULL,
    frame_id    TEXT NOT NULL,
    segment_id  TEXT,
    confidence  REAL NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_risk_event_segment ON risk_event(segment_id);

CREATE TABLE IF NOT EXISTS risk_factor (
    event_id    TEXT NOT NULL,
    category    TEXT NOT NULL,
    severity    REAL NOT NULL,
    source      TEXT NOT NULL,
    detail      TEXT,
    FOREIGN KEY(event_id) REFERENCES risk_event(event_id) ON DELETE CASCADE
);
CREATE INDEX IF NOT EXISTS idx_risk_factor_event ON risk_factor(event_id);
"""


class RiskStore:
    """SQLite-backed risk event store. Pass `:memory:` for an ephemeral store.

    The connection is held open for the life of the instance; call `close()`
    when done. The schema is created on first use.
    """

    def __init__(self, path: str = ":memory:") -> None:
        self._path = path
        self._conn = sqlite3.connect(path, check_same_thread=False, timeout=5.0)
        # WAL + a busy_timeout let multiple processes (memory_node writes,
        # planner_node reads) share a single file safely. WAL is a no-op for
        # `:memory:` databases but the PRAGMA succeeds, so we leave it in.
        self._conn.execute("PRAGMA journal_mode=WAL")
        self._conn.execute("PRAGMA busy_timeout=2000")
        self._conn.execute("PRAGMA synchronous=NORMAL")
        self._conn.executescript(_SCHEMA)
        self._conn.commit()

    # -- write path -----------------------------------------------------------

    def record_event(self, event: RiskEvent) -> None:
        # Wrap the event + factor writes in a single transaction so a concurrent
        # reader can't observe an event row with no factors (which would have
        # been silently dropped by `events_for_segment`).
        cur = self._conn.cursor()
        try:
            cur.execute("BEGIN IMMEDIATE")
            cur.execute(
                "INSERT OR REPLACE INTO risk_event "
                "(event_id, timestamp, position_x, position_y, position_z, frame_id, segment_id, confidence) "
                "VALUES (?,?,?,?,?,?,?,?)",
                (
                    event.event_id,
                    event.timestamp,
                    event.position[0], event.position[1], event.position[2],
                    event.frame_id,
                    event.segment_id,
                    event.confidence,
                ),
            )
            cur.execute("DELETE FROM risk_factor WHERE event_id = ?", (event.event_id,))
            cur.executemany(
                "INSERT INTO risk_factor (event_id, category, severity, source, detail) "
                "VALUES (?,?,?,?,?)",
                [
                    (event.event_id, f.category.value, f.severity, f.source, f.detail)
                    for f in event.factors
                ],
            )
            self._conn.commit()
        except sqlite3.Error:
            try:
                self._conn.rollback()
            except sqlite3.Error:
                pass
            raise

    # -- read path ------------------------------------------------------------

    def events_for_segment(self, segment_id: str) -> List[RiskEvent]:
        cur = self._conn.cursor()
        rows = cur.execute(
            "SELECT event_id, timestamp, position_x, position_y, position_z, frame_id, segment_id, confidence "
            "FROM risk_event WHERE segment_id = ? ORDER BY timestamp",
            (segment_id,),
        ).fetchall()
        out: List[RiskEvent] = []
        for r in rows:
            event_id, ts, px, py, pz, frame, seg_id, conf = r
            f_rows = cur.execute(
                "SELECT category, severity, source, detail FROM risk_factor WHERE event_id = ?",
                (event_id,),
            ).fetchall()
            factors = [
                RiskFactor(
                    category=FactorCategory.coerce(c),
                    severity=sev, source=src, detail=det or ""
                )
                for c, sev, src, det in f_rows
            ]
            if not factors:
                continue
            out.append(RiskEvent(
                event_id=event_id, position=(px, py, pz),
                factors=factors, confidence=conf, timestamp=ts,
                frame_id=frame, segment_id=seg_id,
            ))
        return out

    def segment_risk(
        self,
        segment_id: str,
        now: Optional[float] = None,
        decay_half_life_s: float = 0.0,
    ) -> Tuple[float, int, str]:
        """Return (cumulative_risk, raw_event_count, dominant_factor_category).

        `cumulative_risk` is the sum of per-event aggregate severities, optionally
        decayed by an exponential with the given half life. `decay_half_life_s <= 0`
        disables decay. Dominant category is the factor category with the largest
        decayed severity contribution; empty string if there are no events.
        """
        events = self.events_for_segment(segment_id)
        if not events:
            return 0.0, 0, ""
        if now is None:
            now = time.time()
        decay_lambda = (math.log(2.0) / decay_half_life_s) if decay_half_life_s > 0 else 0.0
        per_category = defaultdict(float)
        total = 0.0
        for ev in events:
            age = max(0.0, now - ev.timestamp)
            weight = math.exp(-decay_lambda * age) if decay_lambda > 0 else 1.0
            ev_severity = ev.aggregate_severity() * weight
            total += ev_severity
            for f in ev.factors:
                per_category[f.category.value] += f.severity * weight * ev.confidence
        dominant = max(per_category.items(), key=lambda kv: kv[1])[0]
        return total, len(events), dominant

    def evidence_for_segment(
        self,
        segment_id: str,
        max_events: int = 3,
        now: Optional[float] = None,
        decay_half_life_s: float = 0.0,
    ) -> List[RiskEvent]:
        """Return the top-`max_events` events for a segment, ordered by decayed severity."""
        events = self.events_for_segment(segment_id)
        if not events:
            return []
        if now is None:
            now = time.time()
        decay_lambda = (math.log(2.0) / decay_half_life_s) if decay_half_life_s > 0 else 0.0

        def score(ev: RiskEvent) -> float:
            age = max(0.0, now - ev.timestamp)
            w = math.exp(-decay_lambda * age) if decay_lambda > 0 else 1.0
            return ev.aggregate_severity() * w

        events.sort(key=score, reverse=True)
        return events[:max_events]

    # -- lifecycle ------------------------------------------------------------

    def close(self) -> None:
        try:
            self._conn.commit()
        finally:
            self._conn.close()

    def __enter__(self) -> "RiskStore":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    @property
    def path(self) -> str:
        return self._path
