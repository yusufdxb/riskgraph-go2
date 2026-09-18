"""SQLite hardening: paths, schema versioning, identity binding, durability."""
from __future__ import annotations

import math
import os
import sqlite3
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

from riskgraph_core.events import FactorCategory, Provenance, RiskEvent, RiskFactor
from riskgraph_core.store import (
    SCHEMA_VERSION,
    EvidenceClassError,
    MapIdentityError,
    RiskStore,
    SchemaError,
    StoreError,
    resolve_store_path,
)

CORE_ROOT = Path(__file__).resolve().parents[1]


def _ev(eid, x=1.0, y=2.0, sev=0.8, seg=None, t=1000.0, prov=Provenance.OPERATOR_INJECTED):
    return RiskEvent(event_id=eid, position=(x, y, 0.0),
                     factors=[RiskFactor(FactorCategory.SLIP, sev, "test")],
                     timestamp=t, segment_id=seg, provenance=prov, map_id="m1")


# -- paths -------------------------------------------------------------------

def test_relative_path_is_refused():
    with pytest.raises(StoreError, match="absolute"):
        RiskStore("relative/db.sqlite")


def test_tilde_path_is_expanded_to_absolute(monkeypatch, tmp_path):
    monkeypatch.setenv("HOME", str(tmp_path))
    assert resolve_store_path("~/x/db.sqlite") == str(tmp_path / "x" / "db.sqlite")


def test_parent_directories_are_created(tmp_path):
    p = tmp_path / "a" / "b" / "c" / "rg.sqlite"
    s = RiskStore(str(p))
    s.close()
    assert p.exists()


def test_status_reports_absolute_path(tmp_path):
    p = tmp_path / "rg.sqlite"
    with RiskStore(str(p), map_id="m1", evidence_class="test") as s:
        st = s.status()
    assert st["db_path"] == str(p)
    assert os.path.isabs(st["db_path"])
    assert st["schema_version"] == SCHEMA_VERSION
    assert st["map_id"] == "m1"
    assert st["evidence_class"] == "test"


# -- identity binding ----------------------------------------------------------

def test_map_id_binds_on_first_open_and_rejects_mismatch(tmp_path):
    p = str(tmp_path / "rg.sqlite")
    RiskStore(p, map_id="course-aaaa").close()
    RiskStore(p, map_id="course-aaaa").close()  # same map: fine
    with pytest.raises(MapIdentityError, match="course-aaaa"):
        RiskStore(p, map_id="course-bbbb")


def test_evidence_class_rejects_replay_into_live(tmp_path):
    p = str(tmp_path / "rg.sqlite")
    RiskStore(p, map_id="m", evidence_class="live").close()
    with pytest.raises(EvidenceClassError):
        RiskStore(p, map_id="m", evidence_class="replay")


def test_unknown_evidence_class_rejected(tmp_path):
    with pytest.raises(StoreError):
        RiskStore(str(tmp_path / "x.sqlite"), evidence_class="hardware")


def test_empty_map_id_rejected(tmp_path):
    with pytest.raises(MapIdentityError):
        RiskStore(str(tmp_path / "x.sqlite"), map_id="")


# -- migration -----------------------------------------------------------------

_V1_SCHEMA = """
CREATE TABLE risk_event (event_id TEXT PRIMARY KEY, timestamp REAL NOT NULL,
  position_x REAL NOT NULL, position_y REAL NOT NULL, position_z REAL NOT NULL,
  frame_id TEXT NOT NULL, segment_id TEXT, confidence REAL NOT NULL);
CREATE TABLE risk_factor (event_id TEXT NOT NULL, category TEXT NOT NULL,
  severity REAL NOT NULL, source TEXT NOT NULL, detail TEXT);
"""


def _make_v1(path: Path, with_rows: bool) -> None:
    c = sqlite3.connect(str(path))
    c.executescript(_V1_SCHEMA)
    if with_rows:
        c.execute("INSERT INTO risk_event VALUES ('old1', 5.0, 1, 2, 0, 'odom', 'seg', 1.0)")
        c.execute("INSERT INTO risk_factor VALUES ('old1', 'SLIP', 0.9, 'legacy', '')")
    c.commit()
    c.close()


def test_v1_database_is_migrated_and_keeps_rows(tmp_path):
    p = tmp_path / "v1.sqlite"
    _make_v1(p, with_rows=True)
    s = RiskStore(str(p))
    assert s.schema_version == SCHEMA_VERSION
    ev = s.get_event("old1")
    assert ev is not None and ev.provenance == Provenance.UNKNOWN and ev.frame_id == "odom"
    assert s._meta_get("migrated_from") == "0"
    s.close()


def test_legacy_rows_cannot_be_bound_to_a_map(tmp_path):
    p = tmp_path / "v1.sqlite"
    _make_v1(p, with_rows=True)
    with pytest.raises(MapIdentityError, match="legacy"):
        RiskStore(str(p), map_id="course-x")


def test_readonly_open_refuses_unmigrated_file(tmp_path):
    p = tmp_path / "v1.sqlite"
    _make_v1(p, with_rows=False)
    with pytest.raises(SchemaError):
        RiskStore(str(p), readonly=True)


def test_newer_schema_is_refused(tmp_path):
    p = tmp_path / "future.sqlite"
    c = sqlite3.connect(str(p))
    c.execute(f"PRAGMA user_version={SCHEMA_VERSION + 1}")
    c.commit()
    c.close()
    with pytest.raises(SchemaError, match="newer"):
        RiskStore(str(p))


def test_readonly_never_creates_a_file(tmp_path):
    p = tmp_path / "missing.sqlite"
    with pytest.raises(StoreError, match="does not exist"):
        RiskStore(str(p), readonly=True)
    assert not p.exists()


def test_readonly_cannot_write(tmp_path):
    p = str(tmp_path / "rg.sqlite")
    RiskStore(p, map_id="m1").close()
    ro = RiskStore(p, readonly=True, map_id="m1")
    with pytest.raises(StoreError):
        ro.record_event(_ev("x"))


# -- duplicates, validation, quarantine ---------------------------------------

def test_duplicate_event_id_is_ignored_not_replaced(tmp_path):
    s = RiskStore(str(tmp_path / "rg.sqlite"))
    assert s.record_event(_ev("dup", x=1.0, sev=0.5)) is True
    assert s.record_event(_ev("dup", x=9.0, sev=1.0)) is False
    ev = s.get_event("dup")
    assert ev.position[0] == 1.0
    assert ev.factors[0].severity == 0.5
    assert len(ev.factors) == 1
    assert s.status()["incident_count"] == 1


@pytest.mark.parametrize("pos", [(math.nan, 0, 0), (0, math.inf, 0), (0, 0, -math.inf)])
def test_nonfinite_positions_are_rejected(tmp_path, pos):
    s = RiskStore(str(tmp_path / "rg.sqlite"))
    ev = RiskEvent("bad", pos, [RiskFactor("SLIP", 0.5, "t")], timestamp=1.0)
    with pytest.raises(ValueError):
        s.record_event(ev)
    assert s.status()["incident_count"] == 0


def test_nonfinite_timestamp_rejected(tmp_path):
    s = RiskStore(str(tmp_path / "rg.sqlite"))
    ev = RiskEvent("bad", (0, 0, 0), [RiskFactor("SLIP", 0.5, "t")], timestamp=math.nan)
    with pytest.raises(ValueError):
        s.record_event(ev)


def test_quarantine_is_persisted_and_counted(tmp_path):
    p = str(tmp_path / "rg.sqlite")
    s = RiskStore(p)
    s.quarantine("q1", "TF_UNAVAILABLE", "odom->map missing", raw={"x": 1})
    s.close()
    s2 = RiskStore(p)
    st = s2.status()
    assert st["quarantined_count"] == 1
    assert st["quarantine_reasons"] == {"TF_UNAVAILABLE": 1}
    assert s2.all_events() == []


def test_malformed_rows_are_skipped_not_raised(tmp_path):
    p = str(tmp_path / "rg.sqlite")
    s = RiskStore(p)
    s.record_event(_ev("good"))
    # A row written by something other than this code: NaN position, no factors.
    s._conn.execute(
        "INSERT INTO risk_event (event_id, timestamp, position_x, position_y, position_z, "
        "frame_id, confidence) VALUES ('nan_row', 1.0, 'NaN', 0, 0, 'map', 1.0)")
    s._conn.execute(
        "INSERT INTO risk_event (event_id, timestamp, position_x, position_y, position_z, "
        "frame_id, confidence) VALUES ('no_factors', 1.0, 0, 0, 0, 'map', 1.0)")
    s._conn.commit()
    ids = [e.event_id for e in s.all_events()]
    assert ids == ["good"]
    assert s.malformed_rows_skipped == 2


# -- atomicity and durability --------------------------------------------------

def test_interrupted_write_leaves_no_partial_event(tmp_path, monkeypatch):
    s = RiskStore(str(tmp_path / "rg.sqlite"))
    real_cursor = s._conn.cursor

    class _Boom:
        def __init__(self, cur):
            self._cur = cur

        def __getattr__(self, name):
            return getattr(self._cur, name)

        def executemany(self, *a, **k):
            raise sqlite3.OperationalError("disk I/O error (simulated)")

    monkeypatch.setattr(s, "_conn", _ConnProxy(s._conn, lambda: _Boom(real_cursor())))
    with pytest.raises(sqlite3.OperationalError):
        s.record_event(_ev("half"))
    monkeypatch.undo()
    assert s.get_event("half") is None
    assert s._conn.execute("SELECT COUNT(*) FROM risk_event").fetchone()[0] == 0


class _ConnProxy:
    def __init__(self, conn, cursor_factory):
        self._conn = conn
        self._factory = cursor_factory

    def cursor(self):
        return self._factory()

    def __getattr__(self, name):
        return getattr(self._conn, name)


def test_committed_event_survives_sigkill(tmp_path):
    """Write in a child process, SIGKILL it right after commit, reopen."""
    p = tmp_path / "rg.sqlite"
    code = (
        "import sys, time, os, signal\n"
        f"sys.path.insert(0, {str(CORE_ROOT)!r})\n"
        "from riskgraph_core.store import RiskStore\n"
        "from riskgraph_core.events import RiskEvent, RiskFactor\n"
        f"s = RiskStore({str(p)!r}, map_id='m1', evidence_class='test')\n"
        "s.record_event(RiskEvent('k1', (1.0, 2.0, 0.0), [RiskFactor('SLIP', 0.9, 't')], timestamp=5.0))\n"
        "print('committed', flush=True)\n"
        "os.kill(os.getpid(), signal.SIGKILL)\n"
    )
    r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=30)
    assert "committed" in r.stdout
    assert r.returncode == -9
    s = RiskStore(str(p), map_id="m1", evidence_class="test")
    ev = s.get_event("k1")
    assert ev is not None and ev.position == (1.0, 2.0, 0.0)


def test_concurrent_writers_and_a_second_reader(tmp_path):
    p = str(tmp_path / "rg.sqlite")
    writer = RiskStore(p, map_id="m1")
    errors = []

    def write(prefix):
        try:
            for i in range(50):
                writer.record_event(_ev(f"{prefix}{i}", x=float(i)))
        except Exception as exc:  # pragma: no cover - surfaced below
            errors.append(exc)

    threads = [threading.Thread(target=write, args=(f"t{k}_",)) for k in range(4)]
    for t in threads:
        t.start()
    reader = RiskStore(p, readonly=True, map_id="m1")
    seen = []
    while any(t.is_alive() for t in threads):
        for e in reader.all_events():
            assert e.factors, "reader observed an event without its factors"
        seen.append(reader.status()["incident_count"])
        time.sleep(0.005)
    for t in threads:
        t.join()
    assert not errors
    assert reader.status()["incident_count"] == 200
    assert seen == sorted(seen), "incident count must never go backwards"


def test_reopen_after_restart_reads_same_rows(tmp_path):
    p = str(tmp_path / "rg.sqlite")
    s = RiskStore(p, map_id="m1", evidence_class="live")
    s.record_event(_ev("r1"))
    s.close()
    s2 = RiskStore(p, map_id="m1", evidence_class="live")
    assert [e.event_id for e in s2.all_events()] == ["r1"]
    assert s2.status()["incident_count"] == 1


def test_backup_is_a_consistent_copy(tmp_path):
    p = str(tmp_path / "rg.sqlite")
    s = RiskStore(p, map_id="m1", evidence_class="test")
    s.record_event(_ev("b1"))
    dest = s.backup_to(str(tmp_path / "copy" / "rg_copy.sqlite"))
    c = RiskStore(dest, readonly=True, map_id="m1")
    assert [e.event_id for e in c.all_events()] == ["b1"]


def test_provenance_round_trips(tmp_path):
    s = RiskStore(str(tmp_path / "rg.sqlite"))
    e = _ev("p1", prov=Provenance.OPERATOR_INJECTED)
    e.source_frame_id = "base_link"
    e.run_mode = "live"
    e.ingest_time = 123.5
    s.record_event(e)
    back = s.get_event("p1")
    assert back.provenance == Provenance.OPERATOR_INJECTED
    assert back.source_frame_id == "base_link"
    assert back.map_id == "m1"
    assert back.run_mode == "live"
    assert back.ingest_time == 123.5
    assert s.status()["provenance_counts"] == {"OPERATOR_INJECTED": 1}
