"""Map identity, the canonical experiment file, and the trial verdict logic."""
from __future__ import annotations

import math
import shutil
from pathlib import Path

import pytest

from riskgraph_core.clock_policy import effective_event_time
from riskgraph_core.events import RiskEvent, RiskFactor
from riskgraph_core.experiment import (
    ExperimentError,
    capture_point_on_path,
    check_experiment,
    compare_routes,
    course_grid,
    fallback_checks,
    load_experiment,
    nearest_sample,
    route_side,
    validate_path,
    write_course_map,
)
from riskgraph_core.map_identity import (
    MapIdentityInputError,
    compute_map_id,
    describe_map,
)
from riskgraph_core.risk_field import RiskField
from riskgraph_core.viz import render_routes_svg

REPO = Path(__file__).resolve().parents[3]
EXP_FILE = REPO / "src" / "riskgraph_bringup" / "config" / "experiment" / "two_corridor.yaml"


@pytest.fixture
def exp():
    return load_experiment(str(EXP_FILE))


@pytest.fixture
def exp_copy(tmp_path):
    """The canonical experiment, with its map regenerated into tmp_path."""
    d = tmp_path / "config" / "experiment"
    d.mkdir(parents=True)
    shutil.copy(EXP_FILE, d / "two_corridor.yaml")
    e = load_experiment(str(d / "two_corridor.yaml"))
    write_course_map(e)
    return e


# -- experiment file -----------------------------------------------------------

def test_canonical_experiment_loads(exp):
    assert exp.frame_id == "map"
    assert exp.start.x == 0.0 and exp.goal.x == 5.0
    assert exp.risk_params.max_cell_value < 100
    assert exp.risk_params.decay_half_life_s == 0.0
    assert check_experiment(exp) == []


def test_committed_map_matches_the_experiment_file(exp):
    """The .pgm in the repo must be exactly what the generator produces."""
    info, data = course_grid(exp)
    d = describe_map(exp.map_yaml)
    assert (d.width, d.height) == (info.width, info.height)
    assert d.resolution == pytest.approx(exp.resolution)
    raw = Path(d.image_path).read_bytes()
    pixels = raw[-info.width * info.height:]
    for row in range(info.height):
        iy = info.height - 1 - row
        for ix in range(0, info.width, 7):
            expect = 254 if data[iy * info.width + ix] == 0 else 0
            assert pixels[row * info.width + ix] == expect


def test_bad_experiment_is_rejected(tmp_path):
    p = tmp_path / "bad.yaml"
    p.write_text("experiment:\n  map: {yaml: x.yaml}\n  area: {x_min: 0, x_max: 1, y_min: 0, y_max: 1}\n"
                 "  obstacle: {cx: 5, cy: 0, size_x: 1, size_y: 1}\n"
                 "  start: {x: 0.5, y: 0.5, yaw: 0}\n  goal: {x: 0.9, y: 0.5, yaw: 0}\n")
    with pytest.raises(ExperimentError):
        load_experiment(str(p))
    with pytest.raises(ExperimentError):
        load_experiment(str(tmp_path / "missing.yaml"))


def test_course_has_free_corridors_and_occupied_obstacle(exp):
    info, data = course_grid(exp)

    def occ(x, y):
        return data[info.index(*info.world_to_cell(x, y))]

    assert occ(exp.start.x, exp.start.y) == 0
    assert occ(exp.goal.x, exp.goal.y) == 0
    assert occ(exp.obstacle.cx, exp.obstacle.cy) == 100
    left, right = exp.corridor_points()["left"], exp.corridor_points()["right"]
    assert occ(*left) == 0 and occ(*right) == 0
    assert occ(exp.area.x_max + 0.25, 0.0) == 100


# -- map identity ----------------------------------------------------------------

def test_map_id_is_stable_and_path_independent(exp_copy, tmp_path):
    a = compute_map_id(exp_copy.map_yaml, exp_copy.anchor)
    assert a == compute_map_id(exp_copy.map_yaml, exp_copy.anchor)
    moved = tmp_path / "elsewhere"
    moved.mkdir()
    for f in Path(exp_copy.map_yaml).parent.iterdir():
        shutil.copy(f, moved / f.name)
    assert compute_map_id(str(moved / Path(exp_copy.map_yaml).name), exp_copy.anchor) == a


def test_map_id_changes_with_image_resolution_origin_and_anchor(exp_copy):
    base = compute_map_id(exp_copy.map_yaml, exp_copy.anchor)
    assert compute_map_id(exp_copy.map_yaml, dict(exp_copy.anchor, x=0.1)) != base
    assert compute_map_id(exp_copy.map_yaml, None) != base
    y = Path(exp_copy.map_yaml)
    text = y.read_text()
    y.write_text(text.replace("resolution: 0.05", "resolution: 0.06"))
    assert compute_map_id(str(y), exp_copy.anchor) != base
    y.write_text(text)
    assert compute_map_id(str(y), exp_copy.anchor) == base
    pgm = Path(describe_map(str(y)).image_path)
    b = bytearray(pgm.read_bytes())
    b[-1] = 0 if b[-1] else 254
    pgm.write_bytes(bytes(b))
    assert compute_map_id(str(y), exp_copy.anchor) != base


def test_map_description_reads_pgm_size(exp_copy):
    d = describe_map(exp_copy.map_yaml)
    info, _ = course_grid(exp_copy)
    assert (d.width, d.height) == (info.width, info.height)
    assert d.grid_info().same_geometry(info)


def test_missing_map_inputs_raise(tmp_path):
    with pytest.raises(MapIdentityInputError):
        describe_map(str(tmp_path / "nope.yaml"))
    y = tmp_path / "m.yaml"
    y.write_text("image: missing.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n")
    with pytest.raises(MapIdentityInputError):
        describe_map(str(y))


# -- route analysis -----------------------------------------------------------

def _route(side, exp, y_off=1.0):
    y = exp.obstacle.cy + (y_off if side == "left" else -y_off)
    return [(0.0, 0.0), (1.5, y), (3.5, y), (5.0, 0.0)]


def test_route_side(exp):
    from riskgraph_core.geometry import resample_polyline
    assert route_side(resample_polyline(_route("left", exp), 0.05), exp) == "left"
    assert route_side(resample_polyline(_route("right", exp), 0.05), exp) == "right"
    assert route_side([(0, 0), (1, 0)], exp) == "none"


def test_validate_path_accepts_a_sane_route(exp):
    from riskgraph_core.geometry import resample_polyline
    pts = resample_polyline(_route("left", exp), 0.05)
    assert validate_path(pts, "map", exp, (0.0, 0.0), (5.0, 0.0)) == []


def test_validate_path_rejects_malformed_routes(exp):
    from riskgraph_core.geometry import resample_polyline
    pts = resample_polyline(_route("left", exp), 0.05)
    assert validate_path(pts, "odom", exp, (0, 0), (5, 0))
    assert validate_path([(0, 0)], "map", exp, (0, 0), (5, 0))
    assert validate_path([(0, 0), (math.nan, 1), (5, 0)], "map", exp, (0, 0), (5, 0))
    assert validate_path(pts, "map", exp, (2.0, 0.0), (5, 0))          # starts away from robot
    assert validate_path(pts[:-20], "map", exp, (0, 0), (5, 0))        # ends short of goal
    assert validate_path([(0, 0), (5, 0)], "map", exp, (0, 0), (5, 0))  # 5 m jump
    assert validate_path(pts, "map", exp, (0, 0), (5, 0), is_lethal=lambda x, y: True)
    loop = resample_polyline([(0, 0), (0, 1.7), (-0.9, 1.7), (-0.9, -1.7), (5.9, -1.7),
                              (5.9, 1.7), (5, 0)], 0.05)
    assert any("3x" in p for p in validate_path(loop, "map", exp, (0, 0), (5, 0)))


def _field_with_left_event(exp):
    x, y = exp.corridor_points()["left"]
    return RiskField.from_events(
        [RiskEvent("inj", (x, y, 0.0), [RiskFactor("OTHER", 1.0, "op")], frame_id="map")],
        exp.risk_params, now=0.0)


def test_compare_routes_passes_when_route_moves_away_from_risk(exp):
    from riskgraph_core.geometry import resample_polyline
    f = _field_with_left_event(exp)
    base = resample_polyline(_route("left", exp), 0.05)
    aware = resample_polyline(_route("right", exp), 0.05)
    v = compare_routes(base, aware, f, exp)
    assert v["baseline_side"] == "left" and v["aware_side"] == "right"
    assert v["risk_reduced"] and v["geometry_changed"] and v["pass"]
    assert v["aware_metrics"]["accumulated_risk"] == 0.0


def test_compare_routes_fails_when_route_did_not_change(exp):
    from riskgraph_core.geometry import resample_polyline
    f = _field_with_left_event(exp)
    base = resample_polyline(_route("left", exp), 0.05)
    v = compare_routes(base, base, f, exp)
    assert not v["pass"]
    assert not v["risk_reduced"] and not v["side_changed"]


def test_compare_routes_fails_on_number_change_without_geometry_change(exp):
    """A lower score alone is not success: the path must actually move."""
    from riskgraph_core.geometry import resample_polyline
    f = _field_with_left_event(exp)
    base = resample_polyline(_route("left", exp, y_off=1.0), 0.05)
    nudged = resample_polyline(_route("left", exp, y_off=1.1), 0.05)
    v = compare_routes(base, nudged, f, exp)
    assert v["mean_separation_m"] < exp.min_route_separation_m
    assert not v["pass"]


def test_fallback_checks(exp):
    p = [(0.0, 0.0), (1.0, 0.5), (5.0, 0.0)]
    ok = fallback_checks(p, [list(p), list(p)], [0, 50, 90], 0.05)
    assert ok["pass"] and ok["deterministic"]
    moved = fallback_checks(p, [[(0, 0), (1, 1.5), (5, 0)]], [0], 0.05)
    assert not moved["deterministic"] and not moved["pass"]
    lethal = fallback_checks(p, [p], [0, 100], 0.05)
    assert lethal["risk_lethal_cells"] == 1 and not lethal["pass"]
    nan = fallback_checks([(0, 0), (math.nan, 0)], [], [0], 0.05)
    assert not nan["pass"]


def test_capture_point_and_nearest_sample(exp):
    from riskgraph_core.geometry import resample_polyline
    pts = resample_polyline(_route("left", exp), 0.05)
    cp = capture_point_on_path(pts, exp)
    assert abs(cp[0] - exp.gate_x) < 0.05
    samples = [{"x": 2.0, "y": 0.8, "t": 1}, {"x": 2.49, "y": 0.86, "t": 2}]
    best = nearest_sample(samples, cp)
    assert best["t"] == 2 and best["distance_to_target_m"] < 0.05
    assert nearest_sample([], cp) is None


# -- clock policy -------------------------------------------------------------------

def test_clock_policy_keeps_plausible_stamps():
    assert effective_event_time(1000.0, 1000.4) == (1000.0, "")


def test_clock_policy_replaces_robot_clock_skew():
    t, note = effective_event_time(1760700000.0, 1789000000.0)  # months apart
    assert t == 1789000000.0 and note.startswith("stamp_skew_")


@pytest.mark.parametrize("stamp", [0.0, -1.0, math.nan, None])
def test_clock_policy_unset_stamp(stamp):
    t, note = effective_event_time(stamp, 50.0)
    assert t == 50.0 and note == "stamp_unset_used_receipt"


# -- visualization ---------------------------------------------------------------------

def test_svg_renders_routes_and_events(exp):
    info, occ = course_grid(exp)
    f = _field_with_left_event(exp)
    svg = render_routes_svg(info, occ, f.rasterize(info),
                            [{"name": "baseline", "points": _route("left", exp)},
                             {"name": "risk-aware", "points": _route("right", exp), "dashed": True}],
                            [{"id": "inj", "x": 2.5, "y": 1.0, "label": "inj"}], "test <&>")
    assert svg.startswith("<svg") and svg.rstrip().endswith("</svg>")
    assert "baseline" in svg and "risk-aware" in svg and "&lt;&amp;&gt;" in svg
