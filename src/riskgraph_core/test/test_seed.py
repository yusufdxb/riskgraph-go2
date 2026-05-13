"""Tests for segment-seeding (phase 1 gap closure).

Required scenarios from the v0.1.1 ticket:
- empty seed
- single segment
- multiple segments
- overlapping segments (same id, last-write-wins)
- malformed input (multiple flavors)

Plus a few extras that fall out naturally:
- file-not-found is a clean SegmentSeedError, not OSError
- merge_segment_seeds across files
- YAML loading when PyYAML is available (skipped otherwise)
- seed feeds segment_for_point correctly (integration with existing core path)
"""
from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path

import pytest

from riskgraph_core.seed import (
    SegmentSeedError,
    SegmentSeedResult,
    parse_segment_seed,
    load_segment_seed,
    merge_segment_seeds,
)
from riskgraph_core.segments import segment_for_point


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _seg(seg_id, x0=0.0, y0=0.0, x1=1.0, y1=0.0, label=None):
    entry = {
        "segment_id": seg_id,
        "start": [x0, y0, 0.0],
        "end": [x1, y1, 0.0],
    }
    if label is not None:
        entry["semantic_label"] = label
    return entry


def _write_json(tmp_path: Path, payload) -> Path:
    p = tmp_path / "seed.json"
    p.write_text(json.dumps(payload))
    return p


# ---------------------------------------------------------------------------
# Required: empty seed
# ---------------------------------------------------------------------------

def test_empty_seed_yields_empty_result(tmp_path):
    p = _write_json(tmp_path, {"segments": []})
    result = load_segment_seed(str(p))
    assert isinstance(result, SegmentSeedResult)
    assert result.segments == []
    assert result.duplicate_ids == []
    assert result.frame_id == "map"
    assert result.source_path == str(p)
    assert len(result) == 0


def test_empty_file_is_no_op(tmp_path):
    p = tmp_path / "empty.json"
    p.write_text("")
    result = load_segment_seed(str(p))
    assert result.segments == []
    assert result.duplicate_ids == []


def test_parse_empty_dict_segments_is_no_op():
    # Direct parse path, no file IO.
    result = parse_segment_seed({"segments": []})
    assert result.segments == []


# ---------------------------------------------------------------------------
# Required: single segment
# ---------------------------------------------------------------------------

def test_single_segment_loaded(tmp_path):
    p = _write_json(tmp_path, {"segments": [_seg("only_one", x1=3.0, y1=4.0, label="hallway")]})
    result = load_segment_seed(str(p))
    assert len(result) == 1
    s = result.segments[0]
    assert s.segment_id == "only_one"
    assert s.start == (0.0, 0.0, 0.0)
    assert s.end == (3.0, 4.0, 0.0)
    assert s.semantic_label == "hallway"
    assert s.length_m == pytest.approx(5.0)


def test_single_segment_without_label_has_none_label(tmp_path):
    p = _write_json(tmp_path, {"segments": [_seg("plain")]})
    result = load_segment_seed(str(p))
    assert result.segments[0].semantic_label is None


def test_single_segment_with_empty_label_becomes_none(tmp_path):
    p = _write_json(tmp_path, {"segments": [_seg("plain", label="")]})
    result = load_segment_seed(str(p))
    assert result.segments[0].semantic_label is None


# ---------------------------------------------------------------------------
# Required: multiple segments
# ---------------------------------------------------------------------------

def test_multiple_segments_preserve_order(tmp_path):
    p = _write_json(tmp_path, {"segments": [
        _seg("a", x1=1.0),
        _seg("b", x0=1.0, x1=2.0),
        _seg("c", x0=2.0, x1=3.0),
    ]})
    result = load_segment_seed(str(p))
    assert [s.segment_id for s in result.segments] == ["a", "b", "c"]
    assert result.duplicate_ids == []


def test_multiple_segments_feed_segment_for_point(tmp_path):
    # End-to-end: loaded seed plugs straight into the spatial-join call
    # used by the memory_node. This is the actual phase 1 path.
    p = _write_json(tmp_path, {"segments": [
        _seg("east_arm", x0=0.0, y0=0.0, x1=10.0, y1=0.0),
        _seg("north_arm", x0=0.0, y0=0.0, x1=0.0, y1=10.0),
    ]})
    result = load_segment_seed(str(p))
    nearest = segment_for_point(result.segments, (5.0, 0.1, 0.0))
    assert nearest is not None
    assert nearest.segment_id == "east_arm"
    nearest = segment_for_point(result.segments, (0.1, 5.0, 0.0))
    assert nearest.segment_id == "north_arm"


# ---------------------------------------------------------------------------
# Required: overlapping segments (same id)
# ---------------------------------------------------------------------------

def test_overlapping_segment_ids_last_write_wins(tmp_path):
    p = _write_json(tmp_path, {"segments": [
        _seg("dup", x1=1.0, label="first"),
        _seg("dup", x1=99.0, label="second"),
    ]})
    result = load_segment_seed(str(p))
    assert len(result.segments) == 1
    assert result.segments[0].segment_id == "dup"
    assert result.segments[0].end == (99.0, 0.0, 0.0)
    assert result.segments[0].semantic_label == "second"
    assert result.duplicate_ids == ["dup"]


def test_overlapping_ids_triple_collision_collected(tmp_path):
    p = _write_json(tmp_path, {"segments": [
        _seg("a", x1=1.0),
        _seg("a", x1=2.0),
        _seg("a", x1=3.0),
        _seg("b", x1=4.0),
    ]})
    result = load_segment_seed(str(p))
    # `a` collided twice (on the 2nd and 3rd entries)
    assert result.duplicate_ids == ["a", "a"]
    # Final value wins
    seg_by_id = {s.segment_id: s for s in result.segments}
    assert seg_by_id["a"].end == (3.0, 0.0, 0.0)
    assert seg_by_id["b"].end == (4.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# Required: malformed input
# ---------------------------------------------------------------------------

def test_malformed_top_level_not_object_raises():
    with pytest.raises(SegmentSeedError, match="top level"):
        parse_segment_seed([{"segment_id": "x"}])  # type: ignore[arg-type]


def test_malformed_missing_segments_key_raises():
    with pytest.raises(SegmentSeedError, match="segments"):
        parse_segment_seed({"version": "1"})


def test_malformed_segments_not_list_raises():
    with pytest.raises(SegmentSeedError, match="'segments' must be a list"):
        parse_segment_seed({"segments": {"a": {}}})


def test_malformed_segment_not_object_raises():
    with pytest.raises(SegmentSeedError, match=r"segments\[0\]"):
        parse_segment_seed({"segments": ["not an object"]})


def test_malformed_missing_segment_id_raises():
    with pytest.raises(SegmentSeedError, match="segment_id"):
        parse_segment_seed({"segments": [{"start": [0, 0, 0], "end": [1, 0, 0]}]})


def test_malformed_empty_segment_id_raises():
    with pytest.raises(SegmentSeedError, match="non-empty string"):
        parse_segment_seed({"segments": [{"segment_id": "  ", "start": [0, 0, 0], "end": [1, 0, 0]}]})


def test_malformed_missing_start_raises():
    with pytest.raises(SegmentSeedError, match="start.*end.*required"):
        parse_segment_seed({"segments": [{"segment_id": "x", "end": [1, 0, 0]}]})


def test_malformed_wrong_point_arity_raises():
    with pytest.raises(SegmentSeedError, match="3 coordinates"):
        parse_segment_seed({"segments": [{
            "segment_id": "x", "start": [0, 0], "end": [1, 0, 0],
        }]})


def test_malformed_non_numeric_coordinate_raises():
    with pytest.raises(SegmentSeedError, match="non-numeric"):
        parse_segment_seed({"segments": [{
            "segment_id": "x", "start": [0, "oops", 0], "end": [1, 0, 0],
        }]})


def test_malformed_label_type_raises():
    with pytest.raises(SegmentSeedError, match="semantic_label"):
        parse_segment_seed({"segments": [{
            "segment_id": "x",
            "start": [0, 0, 0],
            "end": [1, 0, 0],
            "semantic_label": 42,
        }]})


def test_malformed_json_file_raises(tmp_path):
    p = tmp_path / "broken.json"
    p.write_text("{not valid json")
    with pytest.raises(SegmentSeedError, match="malformed JSON"):
        load_segment_seed(str(p))


def test_load_nonexistent_path_raises(tmp_path):
    with pytest.raises(SegmentSeedError, match="does not exist"):
        load_segment_seed(str(tmp_path / "nope.json"))


def test_load_empty_path_raises():
    with pytest.raises(SegmentSeedError, match="empty"):
        load_segment_seed("")


# ---------------------------------------------------------------------------
# merge_segment_seeds
# ---------------------------------------------------------------------------

def test_merge_two_disjoint_seeds(tmp_path):
    a = parse_segment_seed({"segments": [_seg("a"), _seg("b")]})
    b = parse_segment_seed({"segments": [_seg("c"), _seg("d")]})
    merged = merge_segment_seeds([a, b])
    assert [s.segment_id for s in merged.segments] == ["a", "b", "c", "d"]
    assert merged.duplicate_ids == []


def test_merge_collision_across_files_last_wins():
    a = parse_segment_seed({"segments": [_seg("shared", x1=1.0)]})
    b = parse_segment_seed({"segments": [_seg("shared", x1=99.0)]})
    merged = merge_segment_seeds([a, b])
    assert len(merged.segments) == 1
    assert merged.segments[0].end == (99.0, 0.0, 0.0)
    assert merged.duplicate_ids == ["shared"]


def test_merge_empty_iterable_is_empty():
    merged = merge_segment_seeds([])
    assert merged.segments == []
    assert merged.duplicate_ids == []


# ---------------------------------------------------------------------------
# YAML (optional)
# ---------------------------------------------------------------------------

def test_yaml_seed_loads_when_pyyaml_available(tmp_path):
    pytest.importorskip("yaml")
    import yaml  # noqa: F401  (importorskip already verified)
    p = tmp_path / "seed.yaml"
    p.write_text(
        "version: '1'\n"
        "frame_id: map\n"
        "segments:\n"
        "  - segment_id: yaml_a\n"
        "    start: [0.0, 0.0, 0.0]\n"
        "    end:   [1.0, 0.0, 0.0]\n"
        "    semantic_label: yaml-hallway\n"
    )
    result = load_segment_seed(str(p))
    assert [s.segment_id for s in result.segments] == ["yaml_a"]
    assert result.segments[0].semantic_label == "yaml-hallway"


# ---------------------------------------------------------------------------
# frame_id handling
# ---------------------------------------------------------------------------

def test_custom_frame_id_preserved(tmp_path):
    p = _write_json(tmp_path, {
        "segments": [_seg("a")],
        "frame_id": "odom",
    })
    result = load_segment_seed(str(p))
    assert result.frame_id == "odom"


def test_empty_frame_id_raises():
    with pytest.raises(SegmentSeedError, match="frame_id"):
        parse_segment_seed({"segments": [_seg("a")], "frame_id": ""})


def test_non_string_frame_id_raises():
    with pytest.raises(SegmentSeedError, match="frame_id"):
        parse_segment_seed({"segments": [_seg("a")], "frame_id": 7})


# ---------------------------------------------------------------------------
# Iteration sugar
# ---------------------------------------------------------------------------

def test_result_iterable_walks_segments():
    result = parse_segment_seed({"segments": [_seg("a"), _seg("b")]})
    ids = [s.segment_id for s in result]
    assert ids == ["a", "b"]
