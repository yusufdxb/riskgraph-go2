"""Segment seeding: load a fixed set of route segments from a config file.

Phase 1 of the planner pipeline needs `riskgraph_memory` to know which
named segments exist in the operating environment, so that incoming
`RiskEvent`s without a `segment_id` can be spatially joined to a segment
via `segment_for_point`. Without seeding, the in-memory `_known_segments`
list is empty, every spatial join falls through, and events get stored
unbound, which the planner cannot retrieve by segment_id.

This module is pure Python with NO ROS coupling. The memory node's
__init__ calls `load_segment_seed(path)` if a seed path is configured,
then hands the result to `register_segments`.

Seed file format (JSON):

    {
        "version": "1",
        "frame_id": "map",
        "segments": [
            {
                "segment_id": "hw_glossy",
                "start": [0.0, 0.0, 0.0],
                "end":   [4.0, 0.0, 0.0],
                "semantic_label": "hallway-glossy"
            },
            ...
        ]
    }

YAML is also accepted if PyYAML is available (it is, as a ROS 2 dep on
the Jetson). The format is identical, just YAML-encoded.

Overlap policy: if two segments share a `segment_id`, the LATER entry
wins. The duplicate ids are returned alongside the loaded list so the
caller can log them. This matches the way operators typically iterate:
they tweak a segment, paste it at the end of the file, and forget to
remove the old one. Last-write-wins keeps the iteration cheap; the
warning list keeps it honest.

Malformed input (missing required keys, wrong types, non-3-tuple points,
empty `segments`) raises `SegmentSeedError`. An empty file (or a seed
with `"segments": []`) is NOT an error: it yields an empty list and an
empty duplicates set, so launches with no seed configured still start.
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence, Set, Tuple

from .segments import RouteSegment

Point3 = Tuple[float, float, float]


class SegmentSeedError(ValueError):
    """Raised when a seed file is structurally invalid.

    Distinguished from a generic ValueError so callers (memory_node) can
    log a clear "your seed file is broken" message rather than swallowing
    a bare ValueError.
    """


@dataclass
class SegmentSeedResult:
    segments: List[RouteSegment]
    duplicate_ids: List[str] = field(default_factory=list)
    frame_id: str = "map"
    source_path: Optional[str] = None

    def __iter__(self):
        # Allow `for s in result:` to walk segments, the most common use.
        return iter(self.segments)

    def __len__(self) -> int:
        return len(self.segments)


def _coerce_point(raw, where: str) -> Point3:
    if not isinstance(raw, (list, tuple)):
        raise SegmentSeedError(f"{where}: expected a 3-element list, got {type(raw).__name__}")
    if len(raw) != 3:
        raise SegmentSeedError(f"{where}: expected 3 coordinates, got {len(raw)}")
    try:
        return (float(raw[0]), float(raw[1]), float(raw[2]))
    except (TypeError, ValueError) as exc:
        raise SegmentSeedError(f"{where}: non-numeric coordinate ({exc})") from exc


def _coerce_one(entry, idx: int) -> RouteSegment:
    if not isinstance(entry, dict):
        raise SegmentSeedError(f"segments[{idx}]: expected an object, got {type(entry).__name__}")
    seg_id = entry.get("segment_id")
    if not isinstance(seg_id, str) or not seg_id.strip():
        raise SegmentSeedError(f"segments[{idx}]: 'segment_id' must be a non-empty string")
    if "start" not in entry or "end" not in entry:
        raise SegmentSeedError(
            f"segments[{idx}] (id={seg_id!r}): both 'start' and 'end' are required"
        )
    start = _coerce_point(entry["start"], f"segments[{idx}] (id={seg_id!r}).start")
    end = _coerce_point(entry["end"], f"segments[{idx}] (id={seg_id!r}).end")
    label = entry.get("semantic_label")
    if label is not None and not isinstance(label, str):
        raise SegmentSeedError(
            f"segments[{idx}] (id={seg_id!r}): 'semantic_label' must be string or null"
        )
    if isinstance(label, str) and not label:
        label = None
    return RouteSegment(segment_id=seg_id, start=start, end=end, semantic_label=label)


def parse_segment_seed(raw: dict, source_path: Optional[str] = None) -> SegmentSeedResult:
    """Parse a previously-loaded dict (e.g. from json.loads or yaml.safe_load).

    Useful for tests and for callers that want to load the seed via a
    different transport, like a ROS topic carrying a JSON blob.
    """
    if not isinstance(raw, dict):
        raise SegmentSeedError(
            f"top level must be a JSON/YAML object, got {type(raw).__name__}"
        )
    if "segments" not in raw:
        raise SegmentSeedError("missing required key 'segments'")
    seg_list = raw["segments"]
    if not isinstance(seg_list, list):
        raise SegmentSeedError(
            f"'segments' must be a list, got {type(seg_list).__name__}"
        )

    frame_id = raw.get("frame_id", "map")
    if not isinstance(frame_id, str) or not frame_id:
        raise SegmentSeedError("'frame_id' must be a non-empty string if present")

    seen: dict = {}
    order: List[str] = []
    duplicates: List[str] = []
    for idx, entry in enumerate(seg_list):
        seg = _coerce_one(entry, idx)
        if seg.segment_id in seen:
            duplicates.append(seg.segment_id)
        else:
            order.append(seg.segment_id)
        # Last-write-wins: later entries replace earlier ones.
        seen[seg.segment_id] = seg

    segments = [seen[sid] for sid in order]
    # Preserve duplicate order without deduping: callers may want to know
    # how many redundant entries existed, not just which ids collided.
    return SegmentSeedResult(
        segments=segments,
        duplicate_ids=duplicates,
        frame_id=frame_id,
        source_path=source_path,
    )


def load_segment_seed(path: str) -> SegmentSeedResult:
    """Load and parse a segment seed file. Supports `.json`, `.yaml`, `.yml`.

    An empty path or a non-existent path raises `SegmentSeedError`. To
    skip seeding entirely, just do not call this function.
    """
    if not path:
        raise SegmentSeedError("seed path is empty")
    if not os.path.exists(path):
        raise SegmentSeedError(f"seed path does not exist: {path}")

    ext = os.path.splitext(path)[1].lower()
    try:
        text = open(path, "r", encoding="utf-8").read()
    except OSError as exc:
        raise SegmentSeedError(f"could not read seed file {path}: {exc}") from exc

    if not text.strip():
        # Empty file is a no-op seed, not an error. Matches "configured
        # but not populated yet" workflow.
        return SegmentSeedResult(segments=[], duplicate_ids=[], frame_id="map", source_path=path)

    if ext in (".yaml", ".yml"):
        try:
            import yaml  # type: ignore
        except ImportError as exc:
            raise SegmentSeedError(
                "YAML seed requested but PyYAML is not installed; "
                "use a .json file or `pip install pyyaml`"
            ) from exc
        try:
            raw = yaml.safe_load(text)
        except yaml.YAMLError as exc:
            raise SegmentSeedError(f"malformed YAML in {path}: {exc}") from exc
    else:
        try:
            raw = json.loads(text)
        except json.JSONDecodeError as exc:
            raise SegmentSeedError(f"malformed JSON in {path}: {exc}") from exc

    return parse_segment_seed(raw, source_path=path)


def merge_segment_seeds(seeds: Iterable[SegmentSeedResult]) -> SegmentSeedResult:
    """Concatenate multiple seeds, last-write-wins on id collisions.

    Used when an operator splits seeds across files (one per floor, one
    for outdoor laps, etc.) and bringup wants to combine them. The
    returned `duplicate_ids` carries collisions ACROSS files, in the
    order they were encountered.
    """
    seen: dict = {}
    order: List[str] = []
    duplicates: List[str] = []
    frame_id = "map"
    source_path: Optional[str] = None
    for seed in seeds:
        if seed.frame_id and seed.frame_id != "map":
            frame_id = seed.frame_id
        if seed.source_path:
            source_path = seed.source_path
        for seg in seed.segments:
            if seg.segment_id in seen:
                duplicates.append(seg.segment_id)
            else:
                order.append(seg.segment_id)
            seen[seg.segment_id] = seg
    return SegmentSeedResult(
        segments=[seen[sid] for sid in order],
        duplicate_ids=duplicates,
        frame_id=frame_id,
        source_path=source_path,
    )


__all__ = [
    "SegmentSeedError",
    "SegmentSeedResult",
    "parse_segment_seed",
    "load_segment_seed",
    "merge_segment_seeds",
]
