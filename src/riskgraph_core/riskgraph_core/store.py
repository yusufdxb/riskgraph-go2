"""SQLite-backed persistent risk store.

One row per event, one row per (event, factor) pair, plus two bookkeeping
tables: ``meta`` (schema version, map identity, evidence class) and
``quarantine`` (observations that arrived but could not be trusted as risk,
kept for the audit trail instead of being silently dropped).

Design rules, each of which exists because the opposite failed somewhere:

* **Absolute paths only.** A relative path resolves against whatever the
  process cwd happens to be, which is how one node ends up writing one file
  while another node reads a different one. ``":memory:"`` is the only
  non-absolute value accepted.
* **One database, one map.** The first writer binds the file to a ``map_id``.
  Opening it later against a different map raises :class:`MapIdentityError`
  instead of quietly mixing coordinates from two maps.
* **One database, one evidence class.** ``live``, ``rehearsal``, ``replay``
  and ``test`` databases cannot be opened as each other, so a replay can never
  write rows into the file a hardware trial reads.
* **Duplicates are ignored, not replaced.** The first write of an
  ``event_id`` wins; a re-delivered message cannot rewrite history.
* **Schema is versioned.** ``PRAGMA user_version`` carries the version and
  older files are migrated in a single transaction on writable open.

Queries are segment keyed or map keyed and apply optional exponential decay
at read time, so the write path stays a single transaction.
"""
from __future__ import annotations

import json
import math
import os
import socket
import sqlite3
import threading
import time
from collections import defaultdict
from typing import Dict, List, Optional, Tuple

from .events import FactorCategory, Provenance, RiskEvent, RiskFactor

#: Current on-disk schema version (``PRAGMA user_version``).
SCHEMA_VERSION = 2

#: Allowed evidence classes. The class is fixed when a database is created.
EVIDENCE_CLASSES = ("live", "rehearsal", "replay", "test")

MEMORY = ":memory:"


class StoreError(RuntimeError):
    """Base class for store problems that must stop the caller."""


class MapIdentityError(StoreError):
    """The database belongs to a different map than the caller expects."""


class EvidenceClassError(StoreError):
    """The database belongs to a different evidence class (live vs replay...)."""


class SchemaError(StoreError):
    """The database schema is unknown, newer than this code, or unmigrated."""


_TABLES_V2 = """
CREATE TABLE IF NOT EXISTS meta (
    key   TEXT PRIMARY KEY,
    value TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS risk_event (
    event_id        TEXT PRIMARY KEY,
    timestamp       REAL NOT NULL,
    position_x      REAL NOT NULL,
    position_y      REAL NOT NULL,
    position_z      REAL NOT NULL,
    frame_id        TEXT NOT NULL,
    segment_id      TEXT,
    confidence      REAL NOT NULL,
    provenance      TEXT NOT NULL DEFAULT 'UNKNOWN',
    source_frame_id TEXT NOT NULL DEFAULT '',
    map_id          TEXT NOT NULL DEFAULT '',
    run_mode        TEXT NOT NULL DEFAULT '',
    ingest_time     REAL NOT NULL DEFAULT 0,
    clock_note      TEXT NOT NULL DEFAULT ''
);
CREATE INDEX IF NOT EXISTS idx_risk_event_segment ON risk_event(segment_id);
CREATE INDEX IF NOT EXISTS idx_risk_event_map ON risk_event(map_id);

CREATE TABLE IF NOT EXISTS risk_factor (
    event_id    TEXT NOT NULL,
    category    TEXT NOT NULL,
    severity    REAL NOT NULL,
    source      TEXT NOT NULL,
    detail      TEXT,
    FOREIGN KEY(event_id) REFERENCES risk_event(event_id) ON DELETE CASCADE
);
CREATE INDEX IF NOT EXISTS idx_risk_factor_event ON risk_factor(event_id);

CREATE TABLE IF NOT EXISTS quarantine (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    event_id    TEXT NOT NULL,
    reason      TEXT NOT NULL,
    detail      TEXT NOT NULL DEFAULT '',
    ingest_time REAL NOT NULL,
    raw_json    TEXT NOT NULL DEFAULT ''
);
CREATE INDEX IF NOT EXISTS idx_quarantine_reason ON quarantine(reason);
"""

# Columns added in v2 to the v1 `risk_event` table, with their DDL.
_V2_EVENT_COLUMNS = (
    ("provenance", "TEXT NOT NULL DEFAULT 'UNKNOWN'"),
    ("source_frame_id", "TEXT NOT NULL DEFAULT ''"),
    ("map_id", "TEXT NOT NULL DEFAULT ''"),
    ("run_mode", "TEXT NOT NULL DEFAULT ''"),
    ("ingest_time", "REAL NOT NULL DEFAULT 0"),
    ("clock_note", "TEXT NOT NULL DEFAULT ''"),
)

_EVENT_COLS = (
    "event_id, timestamp, position_x, position_y, position_z, frame_id, "
    "segment_id, confidence, provenance, source_frame_id, map_id, run_mode, "
    "ingest_time, clock_note"
)


def resolve_store_path(path: str) -> str:
    """Validate a store path: ``":memory:"`` or an absolute filesystem path.

    ``~`` is expanded; anything still relative after that is refused.
    """
    if path == MEMORY:
        return path
    if not isinstance(path, str) or not path.strip():
        raise StoreError("store path is empty")
    expanded = os.path.expanduser(path.strip())
    if not os.path.isabs(expanded):
        raise StoreError(
            f"store path must be absolute, got {path!r}: a relative path resolves "
            f"against each process's cwd, so two nodes could open different files"
        )
    return os.path.normpath(expanded)


def _finite(*vals) -> bool:
    try:
        return all(math.isfinite(float(v)) for v in vals)
    except (TypeError, ValueError):
        return False


class RiskStore:
    """SQLite-backed risk event store.

    Args:
        path: absolute file path, or ``":memory:"``.
        map_id: bind (new file) or verify (existing file) the map identity.
            ``None`` skips the check; only tests and offline tools should.
        evidence_class: bind or verify the evidence class, one of
            :data:`EVIDENCE_CLASSES`. ``None`` skips the check.
        readonly: open without write access. Never creates, never migrates.
            Missing files raise :class:`StoreError`.

    The connection is shared across threads behind a lock; SQLite in WAL mode
    handles the separate-process reader case (planner / explainer / CLI).
    """

    def __init__(self, path: str = MEMORY, *, map_id: Optional[str] = None,
                 evidence_class: Optional[str] = None,
                 readonly: bool = False) -> None:
        self._path = resolve_store_path(path)
        self._readonly = bool(readonly)
        self._lock = threading.RLock()
        self.malformed_rows_skipped = 0
        if evidence_class is not None and evidence_class not in EVIDENCE_CLASSES:
            raise StoreError(
                f"evidence_class must be one of {EVIDENCE_CLASSES}, got {evidence_class!r}")

        if self._path == MEMORY:
            if self._readonly:
                raise StoreError("a :memory: store cannot be opened read-only")
            self._conn = sqlite3.connect(MEMORY, check_same_thread=False, timeout=5.0)
        elif self._readonly:
            if not os.path.exists(self._path):
                raise StoreError(f"database does not exist: {self._path}")
            self._conn = sqlite3.connect(
                f"file:{self._path}?mode=ro", uri=True,
                check_same_thread=False, timeout=5.0)
        else:
            parent = os.path.dirname(self._path)
            os.makedirs(parent, exist_ok=True)
            if not os.access(parent, os.W_OK):
                raise StoreError(f"database directory is not writable: {parent}")
            self._conn = sqlite3.connect(self._path, check_same_thread=False, timeout=5.0)

        self._conn.execute("PRAGMA busy_timeout=5000")
        self._conn.execute("PRAGMA foreign_keys=ON")
        if not self._readonly:
            # WAL: the memory node writes while planner/explainer/CLI read.
            # synchronous=FULL: a committed event survives power loss; the
            # write rate here is a handful of rows per minute, so the cost is
            # irrelevant and the durability is what a trial needs.
            self._conn.execute("PRAGMA journal_mode=WAL")
            self._conn.execute("PRAGMA synchronous=FULL")
            self._migrate()
        else:
            version = self.schema_version
            if version != SCHEMA_VERSION:
                raise SchemaError(
                    f"{self._path} has schema v{version}; this code reads v{SCHEMA_VERSION}. "
                    f"Open it once writable (start the memory node) to migrate.")

        if evidence_class is not None:
            self._bind_meta("evidence_class", evidence_class, EvidenceClassError)
        if map_id is not None:
            if not map_id:
                raise MapIdentityError("map_id must be a non-empty string when given")
            self._bind_meta("map_id", map_id, MapIdentityError)

    # -- schema -------------------------------------------------------------

    @property
    def schema_version(self) -> int:
        with self._lock:
            return int(self._conn.execute("PRAGMA user_version").fetchone()[0])

    def _table_exists(self, name: str) -> bool:
        row = self._conn.execute(
            "SELECT name FROM sqlite_master WHERE type='table' AND name=?", (name,)
        ).fetchone()
        return row is not None

    def _migrate(self) -> None:
        with self._lock:
            version = self.schema_version
            if version > SCHEMA_VERSION:
                raise SchemaError(
                    f"{self._path} has schema v{version}, newer than this code "
                    f"(v{SCHEMA_VERSION}); refusing to touch it")
            if version == SCHEMA_VERSION:
                return
            cur = self._conn.cursor()
            try:
                cur.execute("BEGIN IMMEDIATE")
                if self._table_exists("risk_event"):
                    # v1 file (user_version 0, pre-meta schema): add columns.
                    have = {r[1] for r in cur.execute("PRAGMA table_info(risk_event)")}
                    for col, ddl in _V2_EVENT_COLUMNS:
                        if col not in have:
                            cur.execute(f"ALTER TABLE risk_event ADD COLUMN {col} {ddl}")
                for stmt in _TABLES_V2.split(";"):
                    if stmt.strip():
                        cur.execute(stmt)
                now = time.time()
                cur.execute("INSERT OR IGNORE INTO meta(key, value) VALUES (?, ?)",
                            ("created_at", repr(now)))
                cur.execute("INSERT OR IGNORE INTO meta(key, value) VALUES (?, ?)",
                            ("created_by_host", socket.gethostname()))
                if version != 0 or self._has_legacy_rows(cur):
                    cur.execute("INSERT OR REPLACE INTO meta(key, value) VALUES (?, ?)",
                                ("migrated_from", str(version)))
                cur.execute(f"PRAGMA user_version={SCHEMA_VERSION}")
                self._conn.commit()
            except Exception:
                self._conn.rollback()
                raise

    @staticmethod
    def _has_legacy_rows(cur) -> bool:
        return cur.execute("SELECT COUNT(*) FROM risk_event").fetchone()[0] > 0

    def _meta_get(self, key: str) -> Optional[str]:
        with self._lock:
            if not self._table_exists("meta"):
                return None
            row = self._conn.execute("SELECT value FROM meta WHERE key=?", (key,)).fetchone()
            return row[0] if row else None

    def _bind_meta(self, key: str, value: str, err_cls) -> None:
        with self._lock:
            current = self._meta_get(key)
            if current is not None:
                if current != value:
                    raise err_cls(
                        f"{self._path} is bound to {key}={current!r}, "
                        f"caller expects {value!r}; refusing to mix them")
                return
            if self._readonly:
                raise err_cls(f"{self._path} has no {key} recorded (read-only, cannot bind)")
            n = self._conn.execute("SELECT COUNT(*) FROM risk_event").fetchone()[0]
            if n > 0:
                raise err_cls(
                    f"{self._path} already holds {n} events with no {key} recorded "
                    f"(legacy file); binding them to {value!r} would invent provenance. "
                    f"Use a fresh database path.")
            self._conn.execute("INSERT INTO meta(key, value) VALUES (?, ?)", (key, value))
            self._conn.commit()

    @property
    def map_id(self) -> Optional[str]:
        return self._meta_get("map_id")

    @property
    def evidence_class(self) -> Optional[str]:
        return self._meta_get("evidence_class")

    # -- write path ---------------------------------------------------------

    def record_event(self, event: RiskEvent) -> bool:
        """Persist one event atomically. Returns False if ``event_id`` exists.

        Raises ValueError for non-finite numbers; the caller quarantines.
        """
        if self._readonly:
            raise StoreError("store opened read-only")
        if not event.is_finite():
            raise ValueError(f"event {event.event_id!r} has non-finite position/time/confidence")
        for f in event.factors:
            if not _finite(f.severity):
                raise ValueError(f"event {event.event_id!r} has a non-finite severity")
        with self._lock:
            cur = self._conn.cursor()
            try:
                cur.execute("BEGIN IMMEDIATE")
                cur.execute(
                    f"INSERT OR IGNORE INTO risk_event ({_EVENT_COLS}) "
                    "VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
                    (
                        event.event_id,
                        float(event.timestamp),
                        float(event.position[0]), float(event.position[1]),
                        float(event.position[2]),
                        event.frame_id,
                        event.segment_id,
                        float(event.confidence),
                        event.provenance.value,
                        event.source_frame_id or "",
                        event.map_id or "",
                        event.run_mode or "",
                        float(event.ingest_time or 0.0),
                        event.clock_note or "",
                    ),
                )
                if cur.rowcount == 0:
                    self._conn.rollback()
                    return False
                cur.executemany(
                    "INSERT INTO risk_factor (event_id, category, severity, source, detail) "
                    "VALUES (?,?,?,?,?)",
                    [(event.event_id, f.category.value, float(f.severity), f.source, f.detail)
                     for f in event.factors],
                )
                self._conn.commit()
                return True
            except Exception:
                try:
                    self._conn.rollback()
                except sqlite3.Error:
                    pass
                raise

    def quarantine(self, event_id: str, reason: str, detail: str = "",
                   raw: Optional[dict] = None, ingest_time: Optional[float] = None) -> None:
        """Record an observation that must not count as risk, with the reason."""
        if self._readonly:
            raise StoreError("store opened read-only")
        with self._lock:
            self._conn.execute(
                "INSERT INTO quarantine (event_id, reason, detail, ingest_time, raw_json) "
                "VALUES (?,?,?,?,?)",
                (event_id or "", reason, detail,
                 float(ingest_time if ingest_time is not None else time.time()),
                 json.dumps(raw or {}, default=str, sort_keys=True)),
            )
            self._conn.commit()

    # -- read path ----------------------------------------------------------

    def _rows_to_events(self, rows) -> List[RiskEvent]:
        out: List[RiskEvent] = []
        cur = self._conn.cursor()
        for r in rows:
            (event_id, ts, px, py, pz, frame, seg_id, conf, prov, src_frame,
             map_id, run_mode, ingest, clock_note) = r
            if not _finite(ts, px, py, pz, conf):
                self.malformed_rows_skipped += 1
                continue
            f_rows = cur.execute(
                "SELECT category, severity, source, detail FROM risk_factor WHERE event_id = ?",
                (event_id,),
            ).fetchall()
            factors = [
                RiskFactor(category=FactorCategory.coerce(c), severity=sev,
                           source=src, detail=det or "")
                for c, sev, src, det in f_rows if _finite(sev)
            ]
            if not factors:
                self.malformed_rows_skipped += 1
                continue
            out.append(RiskEvent(
                event_id=event_id, position=(px, py, pz), factors=factors,
                confidence=conf, timestamp=ts, frame_id=frame, segment_id=seg_id,
                provenance=prov, source_frame_id=src_frame, map_id=map_id,
                run_mode=run_mode, ingest_time=ingest, clock_note=clock_note,
            ))
        return out

    def events_for_segment(self, segment_id: str) -> List[RiskEvent]:
        with self._lock:
            rows = self._conn.execute(
                f"SELECT {_EVENT_COLS} FROM risk_event WHERE segment_id = ? "
                "ORDER BY timestamp, event_id",
                (segment_id,),
            ).fetchall()
            return self._rows_to_events(rows)

    def all_events(self, frame_id: Optional[str] = None) -> List[RiskEvent]:
        """Every stored (non-quarantined) event, optionally only one frame."""
        with self._lock:
            if frame_id is None:
                rows = self._conn.execute(
                    f"SELECT {_EVENT_COLS} FROM risk_event ORDER BY timestamp, event_id"
                ).fetchall()
            else:
                rows = self._conn.execute(
                    f"SELECT {_EVENT_COLS} FROM risk_event WHERE frame_id = ? "
                    "ORDER BY timestamp, event_id", (frame_id,)
                ).fetchall()
            return self._rows_to_events(rows)

    def get_event(self, event_id: str) -> Optional[RiskEvent]:
        with self._lock:
            rows = self._conn.execute(
                f"SELECT {_EVENT_COLS} FROM risk_event WHERE event_id = ?", (event_id,)
            ).fetchall()
            events = self._rows_to_events(rows)
            return events[0] if events else None

    def quarantined(self) -> List[dict]:
        with self._lock:
            rows = self._conn.execute(
                "SELECT event_id, reason, detail, ingest_time FROM quarantine ORDER BY id"
            ).fetchall()
        return [dict(event_id=r[0], reason=r[1], detail=r[2], ingest_time=r[3]) for r in rows]

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

    def status(self) -> Dict[str, object]:
        """Machine-readable summary used by the status topic and CLI."""
        with self._lock:
            c = self._conn
            n_events = c.execute("SELECT COUNT(*) FROM risk_event").fetchone()[0]
            n_quar = c.execute("SELECT COUNT(*) FROM quarantine").fetchone()[0]
            n_segments = c.execute(
                "SELECT COUNT(DISTINCT segment_id) FROM risk_event WHERE segment_id IS NOT NULL"
            ).fetchone()[0]
            last_ts, last_ingest = c.execute(
                "SELECT MAX(timestamp), MAX(ingest_time) FROM risk_event").fetchone()
            prov = dict(c.execute(
                "SELECT provenance, COUNT(*) FROM risk_event GROUP BY provenance").fetchall())
            frames = dict(c.execute(
                "SELECT frame_id, COUNT(*) FROM risk_event GROUP BY frame_id").fetchall())
            reasons = dict(c.execute(
                "SELECT reason, COUNT(*) FROM quarantine GROUP BY reason").fetchall())
        return {
            "db_path": self._path,
            "readonly": self._readonly,
            "schema_version": self.schema_version,
            "map_id": self.map_id,
            "evidence_class": self.evidence_class,
            "incident_count": int(n_events),
            "quarantined_count": int(n_quar),
            "quarantine_reasons": reasons,
            "segments_with_events": int(n_segments),
            "provenance_counts": prov,
            "frame_counts": frames,
            "last_event_timestamp": last_ts,
            "last_ingest_time": last_ingest,
            "malformed_rows_skipped": self.malformed_rows_skipped,
        }

    # -- lifecycle ----------------------------------------------------------

    def backup_to(self, dest_path: str) -> str:
        """Consistent copy (SQLite online backup API), safe while WAL is active."""
        dest = resolve_store_path(dest_path)
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        with self._lock:
            target = sqlite3.connect(dest)
            try:
                self._conn.backup(target)
            finally:
                target.close()
        return dest

    def close(self) -> None:
        with self._lock:
            try:
                if not self._readonly:
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

    @property
    def readonly(self) -> bool:
        return self._readonly
