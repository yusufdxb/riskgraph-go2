"""Ingestion policy for the risk memory, with no ROS imports.

The ROS node (``memory_node``) is a thin shell around :class:`MemoryCore`:
it converts messages, provides a TF lookup function, and publishes what this
class computes. Keeping the policy here means every rule that decides whether
an observation becomes remembered risk is unit tested without a ROS graph.

The order of checks for one incoming event, and what happens on failure:

1. non-finite position / time / confidence   -> quarantine ``NONFINITE``
2. provenance not allowed in this run mode   -> quarantine ``PROVENANCE_NOT_LIVE``
3. emitter asserts a different map id        -> quarantine ``MAP_ID_MISMATCH``
4. no pose (blank frame)                     -> quarantine ``UNPOSED``
5. frame != target and TF lookup fails       -> quarantine ``TF_UNAVAILABLE`` / ``NO_TF``
   (or the transform itself is not finite)   -> quarantine ``TF_INVALID``
6. optional spatial join to seeded segments (seed must be in the target frame)
7. persist; an already-known ``event_id``    -> ``duplicate`` (history is never rewritten)

Quarantined observations are written to the ``quarantine`` table with the
reason, so "nothing was risky" can always be told apart from "nothing could
be placed on the map".
"""
from __future__ import annotations

import math
import sqlite3
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from riskgraph_core.clock_policy import DEFAULT_MAX_SKEW_S, effective_event_time
from riskgraph_core.events import LIVE_PROVENANCES, RiskEvent
from riskgraph_core.geometry import apply_transform, quat_is_valid
from riskgraph_core.risk_field import GridInfo, RiskField, RiskFieldParams, grid_stats
from riskgraph_core.segments import RouteSegment, segment_for_point
from riskgraph_core.store import RiskStore

RUN_MODES = ("live", "rehearsal", "replay", "test")

#: ``header.frame_id`` meaning "position is not meaningful" (see pose_source).
UNKNOWN_FRAME = ""

Point3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]
#: (target_frame, source_frame, time_s) -> (translation, rotation) or raise.
TransformLookup = Callable[[str, str, float], Tuple[Point3, Quat]]


class TransformUnavailable(Exception):
    """A TF lookup failed (no transform, extrapolation, timeout...)."""


@dataclass
class IngestResult:
    status: str             # stored | duplicate | quarantined | error
    reason: str = ""
    detail: str = ""
    event: Optional[RiskEvent] = None


def _event_snapshot(ev: RiskEvent) -> Dict[str, object]:
    return {
        "event_id": ev.event_id, "position": list(ev.position), "frame_id": ev.frame_id,
        "timestamp": ev.timestamp, "confidence": ev.confidence,
        "provenance": ev.provenance.value, "map_id": ev.map_id,
        "source_frame_id": ev.source_frame_id, "segment_id": ev.segment_id,
        "factors": [{"category": f.category.value, "severity": f.severity,
                     "source": f.source, "detail": f.detail} for f in ev.factors],
    }


class MemoryCore:
    def __init__(self, store: RiskStore, *, map_id: str, run_mode: str,
                 target_frame: str = "map",
                 transform_lookup: Optional[TransformLookup] = None,
                 segments: Sequence[RouteSegment] = (),
                 seed_frame: str = "map",
                 max_clock_skew_s: float = DEFAULT_MAX_SKEW_S,
                 field_params: RiskFieldParams = RiskFieldParams(),
                 clock: Callable[[], float] = time.time) -> None:
        if run_mode not in RUN_MODES:
            raise ValueError(f"run_mode must be one of {RUN_MODES}, got {run_mode!r}")
        if not map_id:
            raise ValueError("map_id is required")
        if not target_frame:
            raise ValueError("target_frame is required")
        self.store = store
        self.map_id = map_id
        self.run_mode = run_mode
        self.target_frame = target_frame
        self.transform_lookup = transform_lookup
        self.segments: List[RouteSegment] = list(segments)
        self.seed_frame = seed_frame
        self.max_clock_skew_s = float(max_clock_skew_s)
        self.field_params = field_params
        self._clock = clock
        self.counters: Dict[str, int] = {
            "received": 0, "stored": 0, "duplicate": 0, "quarantined": 0,
            "transformed": 0, "joined": 0, "unposed": 0, "seed_frame_mismatch": 0,
            "clock_replaced": 0, "db_errors": 0,
        }
        self.quarantine_reasons: Dict[str, int] = {}
        self.last_observation_time: Optional[float] = None
        self.last_ingest_time: Optional[float] = None
        self._grid_cache: Optional[Tuple[GridInfo, List[int]]] = None
        self._dirty = True
        #: events currently contributing to the risk field (after decay/frame filters)
        self.active_risk_entries = len(self.risk_field().bumps)

    # -- ingestion ----------------------------------------------------------

    def _quarantine(self, ev: RiskEvent, reason: str, detail: str, raw: Dict,
                    now: float) -> IngestResult:
        self.counters["quarantined"] += 1
        self.quarantine_reasons[reason] = self.quarantine_reasons.get(reason, 0) + 1
        try:
            self.store.quarantine(ev.event_id, reason, detail, raw=raw, ingest_time=now)
        except (sqlite3.Error, Exception) as exc:  # the audit row is best effort
            self.counters["db_errors"] += 1
            detail = f"{detail}; quarantine write failed: {exc}"
        return IngestResult("quarantined", reason, detail, ev)

    def ingest(self, ev: RiskEvent) -> IngestResult:
        now = float(self._clock())
        self.counters["received"] += 1
        raw = _event_snapshot(ev)

        if not ev.is_finite():
            return self._quarantine(ev, "NONFINITE", "position/time/confidence not finite", raw, now)
        if any(not math.isfinite(f.severity) for f in ev.factors):
            return self._quarantine(ev, "NONFINITE", "factor severity not finite", raw, now)
        if self.run_mode == "live" and ev.provenance not in LIVE_PROVENANCES:
            return self._quarantine(
                ev, "PROVENANCE_NOT_LIVE",
                f"provenance {ev.provenance.value} is not accepted by a live-mode node", raw, now)
        if ev.map_id and ev.map_id != self.map_id:
            return self._quarantine(
                ev, "MAP_ID_MISMATCH", f"event asserts map {ev.map_id!r}, node has {self.map_id!r}",
                raw, now)

        t, note = effective_event_time(ev.timestamp, now, self.max_clock_skew_s)
        if note:
            self.counters["clock_replaced"] += 1

        src = ev.frame_id
        if src == UNKNOWN_FRAME:
            self.counters["unposed"] += 1
            return self._quarantine(ev, "UNPOSED", "event carries no pose (blank frame_id)", raw, now)
        if src != self.target_frame:
            if self.transform_lookup is None:
                return self._quarantine(
                    ev, "NO_TF", f"frame {src!r} != {self.target_frame!r} and no TF available",
                    raw, now)
            try:
                trans, rot = self.transform_lookup(self.target_frame, src, t)
            except TransformUnavailable as exc:
                return self._quarantine(ev, "TF_UNAVAILABLE", f"{src}->{self.target_frame}: {exc}",
                                        raw, now)
            if not (all(math.isfinite(v) for v in trans) and quat_is_valid(rot)):
                return self._quarantine(ev, "TF_INVALID", f"transform {trans} {rot}", raw, now)
            ev.position = apply_transform(trans, rot, ev.position)
            ev.source_frame_id = ev.source_frame_id or src
            ev.frame_id = self.target_frame
            self.counters["transformed"] += 1
        elif not ev.source_frame_id:
            ev.source_frame_id = src

        if not ev.segment_id and self.segments:
            if self.seed_frame != ev.frame_id:
                self.counters["seed_frame_mismatch"] += 1
            else:
                nearest = segment_for_point(self.segments, ev.position)
                if nearest is not None:
                    ev.segment_id = nearest.segment_id
                    self.counters["joined"] += 1

        ev.timestamp = t
        ev.clock_note = note
        ev.map_id = self.map_id
        ev.run_mode = self.run_mode
        ev.ingest_time = now

        try:
            inserted = self.store.record_event(ev)
        except ValueError as exc:
            return self._quarantine(ev, "NONFINITE", str(exc), raw, now)
        except sqlite3.Error as exc:
            self.counters["db_errors"] += 1
            return IngestResult("error", "DB_ERROR", str(exc), ev)
        if not inserted:
            self.counters["duplicate"] += 1
            return IngestResult("duplicate", "DUPLICATE_EVENT_ID", ev.event_id, ev)
        self.counters["stored"] += 1
        self.last_observation_time = t
        self.last_ingest_time = now
        self._dirty = True
        return IngestResult("stored", "", "", ev)

    # -- representation -----------------------------------------------------

    def risk_field(self, now: Optional[float] = None) -> RiskField:
        return RiskField.from_events(self.store.all_events(frame_id=self.target_frame),
                                     self.field_params,
                                     now=float(self._clock() if now is None else now),
                                     frame_id=self.target_frame)

    def grid(self, info: GridInfo, now: Optional[float] = None) -> List[int]:
        """Rasterized risk for ``info``; cached until the next stored event.

        With decay enabled the field changes with time, so the cache is only
        used when decay is off (the configuration the trial runs with).
        """
        cache_ok = (not self._dirty and self._grid_cache is not None
                    and self._grid_cache[0].same_geometry(info)
                    and self.field_params.decay_half_life_s <= 0.0)
        if cache_ok:
            return list(self._grid_cache[1])
        field = self.risk_field(now)
        data = field.rasterize(info)
        self.active_risk_entries = len(field.bumps)
        self._grid_cache = (info, data)
        self._dirty = False
        return list(data)

    def status(self) -> Dict[str, object]:
        st = dict(self.store.status())
        st.update({
            "node_map_id": self.map_id,
            "run_mode": self.run_mode,
            "target_frame": self.target_frame,
            "counters": dict(self.counters),
            "quarantine_reasons_this_run": dict(self.quarantine_reasons),
            "last_observation_time": self.last_observation_time,
            "last_ingest_time_this_run": self.last_ingest_time,
            "known_segments": len(self.segments),
            "active_risk_entries": self.active_risk_entries,
            "risk_field": {
                "radius_m": self.field_params.radius_m,
                "value_per_unit_risk": self.field_params.value_per_unit_risk,
                "max_cell_value": self.field_params.max_cell_value,
                "decay_half_life_s": self.field_params.decay_half_life_s,
            },
        })
        if self._grid_cache is not None:
            st["grid"] = grid_stats(self._grid_cache[1])
        return st
