"""Ingestion policy of the risk memory (riskgraph_memory.memory_core).

Replaces the earlier stub-based RiskMemoryNode tests: the node is now a thin
ROS shell, and every rule that decides whether an observation becomes
remembered risk lives in MemoryCore, tested here without a ROS graph. The
node wiring itself is exercised against real ROS processes in
tests/integration/.

Behavior change pinned here: an unposed event used to be stored unbound in
risk_event. It is now written to the quarantine table instead, so it can
never contribute to the spatial risk Nav2 plans with.
"""
from __future__ import annotations

import math
import sqlite3

import pytest

from riskgraph_core.events import Provenance, RiskEvent, RiskFactor
from riskgraph_core.geometry import quat_from_yaw
from riskgraph_core.risk_field import GridInfo, RiskFieldParams
from riskgraph_core.segments import RouteSegment
from riskgraph_core.store import RiskStore
from riskgraph_memory.memory_core import MemoryCore, TransformUnavailable

MAP = "course-test"
SEGS = [RouteSegment("A", (0.0, 0.0, 0.0), (10.0, 0.0, 0.0)),
        RouteSegment("B", (0.0, 5.0, 0.0), (10.0, 5.0, 0.0))]


def ev(eid="e1", frame="map", x=5.0, y=0.1, prov=Provenance.OPERATOR_INJECTED, t=1000.0,
       seg=None, map_id="", sev=0.9):
    return RiskEvent(eid, (x, y, 0.0), [RiskFactor("SLIP", sev, "test")], timestamp=t,
                     frame_id=frame, segment_id=seg, provenance=prov, map_id=map_id)


def core(tmp_path, run_mode="live", lookup=None, segments=(), seed_frame="map", clock=None):
    store = RiskStore(str(tmp_path / "rg.sqlite"), map_id=MAP, evidence_class=run_mode)
    return MemoryCore(store, map_id=MAP, run_mode=run_mode, transform_lookup=lookup,
                      segments=segments, seed_frame=seed_frame,
                      field_params=RiskFieldParams(radius_m=0.8),
                      clock=clock or (lambda: 1000.5))


def odom_to_map(tx=2.0, ty=3.0, yaw=math.pi / 2):
    def lookup(target, source, t):
        if (target, source) != ("map", "odom"):
            raise TransformUnavailable(f"no {source}->{target}")
        return (tx, ty, 0.0), quat_from_yaw(yaw)
    return lookup


# -- construction ------------------------------------------------------------

def test_rejects_unknown_run_mode_and_missing_map(tmp_path):
    store = RiskStore(str(tmp_path / "x.sqlite"))
    with pytest.raises(ValueError):
        MemoryCore(store, map_id=MAP, run_mode="hardware")
    with pytest.raises(ValueError):
        MemoryCore(store, map_id="", run_mode="live")


# -- frames and TF -----------------------------------------------------------

def test_map_frame_event_is_stored_as_is(tmp_path):
    c = core(tmp_path)
    r = c.ingest(ev(frame="map", x=1.0, y=2.0))
    assert r.status == "stored"
    stored = c.store.get_event("e1")
    assert stored.position == (1.0, 2.0, 0.0)
    assert stored.frame_id == "map" and stored.source_frame_id == "map"
    assert stored.map_id == MAP and stored.run_mode == "live"


def test_odom_event_is_transformed_with_translation_and_rotation(tmp_path):
    c = core(tmp_path, lookup=odom_to_map(tx=2.0, ty=3.0, yaw=math.pi / 2))
    r = c.ingest(ev(frame="odom", x=1.0, y=0.0, prov=Provenance.HARDWARE_DERIVED))
    assert r.status == "stored"
    stored = c.store.get_event("e1")
    assert stored.position == pytest.approx((2.0, 4.0, 0.0))
    assert stored.frame_id == "map" and stored.source_frame_id == "odom"
    assert c.counters["transformed"] == 1


def test_missing_tf_quarantines_instead_of_guessing(tmp_path):
    c = core(tmp_path, lookup=odom_to_map())
    r = c.ingest(ev(frame="base_link", prov=Provenance.HARDWARE_DERIVED))
    assert r.status == "quarantined" and r.reason == "TF_UNAVAILABLE"
    assert c.store.all_events() == []
    assert c.store.status()["quarantine_reasons"] == {"TF_UNAVAILABLE": 1}


def test_no_tf_provider_quarantines_non_map_frames(tmp_path):
    c = core(tmp_path, lookup=None)
    assert c.ingest(ev(frame="odom")).reason == "NO_TF"


def test_nonfinite_transform_is_quarantined(tmp_path):
    c = core(tmp_path, lookup=lambda *a: ((math.nan, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))
    assert c.ingest(ev(frame="odom")).reason == "TF_INVALID"
    c2 = core(tmp_path / "b", lookup=lambda *a: ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0)))
    assert c2.ingest(ev(frame="odom")).reason == "TF_INVALID"


def test_tf_lookup_uses_event_time_when_plausible(tmp_path):
    seen = []

    def lookup(target, source, t):
        seen.append(t)
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
    c = core(tmp_path, lookup=lookup, clock=lambda: 1000.2)
    c.ingest(ev(frame="odom", t=1000.0))
    assert seen == [1000.0]


def test_skewed_robot_clock_falls_back_to_receipt_time(tmp_path):
    seen = []

    def lookup(target, source, t):
        seen.append(t)
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
    c = core(tmp_path, lookup=lookup, clock=lambda: 1_789_000_000.0)
    c.ingest(ev(frame="odom", t=1_760_700_000.0))
    stored = c.store.get_event("e1")
    assert seen == [1_789_000_000.0]
    assert stored.timestamp == 1_789_000_000.0
    assert stored.clock_note.startswith("stamp_skew_")
    assert c.counters["clock_replaced"] == 1


def test_unposed_event_is_quarantined_not_placed(tmp_path):
    c = core(tmp_path, segments=SEGS)
    r = c.ingest(ev(frame=""))
    assert r.status == "quarantined" and r.reason == "UNPOSED"
    assert c.store.events_for_segment("A") == []
    assert c.counters["unposed"] == 1 and c.counters["joined"] == 0
    assert c.store.status()["quarantined_count"] == 1


# -- provenance and identity ------------------------------------------------------

@pytest.mark.parametrize("prov", [Provenance.SYNTHETIC, Provenance.REPLAY,
                                  Provenance.SIMULATION, Provenance.UNKNOWN])
def test_live_mode_quarantines_non_hardware_provenance(tmp_path, prov):
    c = core(tmp_path)
    r = c.ingest(ev(prov=prov))
    assert r.reason == "PROVENANCE_NOT_LIVE"
    assert c.store.all_events() == []


@pytest.mark.parametrize("mode", ["rehearsal", "replay", "test"])
def test_non_live_modes_accept_any_provenance_but_label_the_row(tmp_path, mode):
    c = core(tmp_path, run_mode=mode)
    assert c.ingest(ev(prov=Provenance.SYNTHETIC)).status == "stored"
    assert c.store.get_event("e1").run_mode == mode


def test_event_asserting_another_map_is_quarantined(tmp_path):
    c = core(tmp_path)
    assert c.ingest(ev(map_id="some-other-map")).reason == "MAP_ID_MISMATCH"
    assert c.ingest(ev(eid="e2", map_id=MAP)).status == "stored"


def test_nonfinite_event_is_quarantined(tmp_path):
    c = core(tmp_path)
    assert c.ingest(ev(x=math.nan)).reason == "NONFINITE"
    assert c.ingest(ev(eid="e2", sev=math.inf)).status == "stored"  # clamped to 1.0 by RiskFactor


# -- duplicates and persistence ------------------------------------------------------

def test_duplicate_event_id_is_reported_and_not_rewritten(tmp_path):
    c = core(tmp_path)
    assert c.ingest(ev(x=1.0)).status == "stored"
    r = c.ingest(ev(x=4.0))
    assert r.status == "duplicate"
    assert c.store.get_event("e1").position[0] == 1.0
    assert c.counters == {**c.counters, "stored": 1, "duplicate": 1}


def test_db_error_is_reported_not_raised(tmp_path):
    c = core(tmp_path)

    def boom(_ev):
        raise sqlite3.OperationalError("database is locked")
    c.store.record_event = boom
    r = c.ingest(ev())
    assert r.status == "error" and r.reason == "DB_ERROR"
    assert c.counters["db_errors"] == 1


def test_restart_sees_the_same_risk(tmp_path):
    c = core(tmp_path)
    c.ingest(ev(x=2.5, y=1.0))
    info = GridInfo(0.05, 100, 60, -1.0, -1.5)
    before = c.grid(info)
    c.store.close()
    c2 = core(tmp_path)
    assert c2.active_risk_entries == 1
    assert c2.grid(info) == before
    assert max(before) > 0


# -- segment join (legacy ScoreRoutes path) ---------------------------------------------

def test_seed_join_binds_map_events(tmp_path):
    c = core(tmp_path, segments=SEGS)
    c.ingest(ev(frame="map", x=5.0, y=0.1))
    assert [e.event_id for e in c.store.events_for_segment("A")] == ["e1"]
    assert c.counters["joined"] == 1


def test_seed_join_after_transform(tmp_path):
    c = core(tmp_path, lookup=odom_to_map(tx=0.0, ty=5.0, yaw=0.0), segments=SEGS)
    c.ingest(ev(frame="odom", x=5.0, y=0.0))  # lands at map (5, 5): segment B
    assert [e.event_id for e in c.store.events_for_segment("B")] == ["e1"]


def test_seed_in_another_frame_never_joins(tmp_path):
    c = core(tmp_path, segments=SEGS, seed_frame="odom")
    c.ingest(ev(frame="map"))
    assert c.store.events_for_segment("A") == []
    assert c.counters["seed_frame_mismatch"] == 1
    assert c.store.get_event("e1") is not None  # still remembered spatially


def test_emitter_stamped_segment_is_kept(tmp_path):
    c = core(tmp_path, segments=SEGS)
    c.ingest(ev(seg="B", x=5.0, y=0.1))
    assert [e.event_id for e in c.store.events_for_segment("B")] == ["e1"]


# -- representation --------------------------------------------------------------------

def test_grid_reflects_new_events_and_status_counts(tmp_path):
    c = core(tmp_path)
    info = GridInfo(0.05, 100, 60, -1.0, -1.5)
    assert max(c.grid(info)) == 0
    c.ingest(ev(x=1.0, y=0.0, sev=1.0))
    g = c.grid(info)
    cell = info.world_to_cell(1.0, 0.0)
    assert g[info.index(*cell)] > 80
    st = c.status()
    assert st["incident_count"] == 1
    assert st["active_risk_entries"] == 1
    assert st["node_map_id"] == MAP and st["map_id"] == MAP
    assert st["last_observation_time"] == 1000.0
    assert st["grid"]["nonzero_cells"] > 0
