"""The spatial risk field Nav2 consumes: bounded, monotone, local, finite."""
from __future__ import annotations

import math
import random

import pytest

from riskgraph_core.events import RiskEvent, RiskFactor
from riskgraph_core.risk_field import (
    MAX_NONLETHAL_VALUE,
    GridInfo,
    RiskField,
    RiskFieldParams,
    grid_stats,
)

P = RiskFieldParams(radius_m=0.8, value_per_unit_risk=90.0, max_cell_value=90)
INFO = GridInfo(resolution=0.05, width=160, height=80, origin_x=-1.5, origin_y=-2.0)


def ev(eid, x, y, sev=1.0, conf=1.0, t=0.0, frame="map"):
    return RiskEvent(eid, (x, y, 0.0), [RiskFactor("SLIP", sev, "t")], confidence=conf,
                     timestamp=t, frame_id=frame)


def field(*events, params=P, now=0.0):
    return RiskField.from_events(list(events), params, now=now)


def test_empty_field_is_all_zero():
    f = field()
    assert f.risk_at(0, 0) == 0.0
    assert set(f.rasterize(INFO)) == {0}


def test_single_event_peaks_at_its_position():
    f = field(ev("a", 1.0, 1.0))
    assert f.risk_at(1.0, 1.0) == pytest.approx(1.0)
    assert f.value_for_risk(f.risk_at(1.0, 1.0)) == 90


def test_risk_is_zero_at_and_beyond_radius():
    f = field(ev("a", 0.0, 0.0))
    assert f.risk_at(0.8, 0.0) == 0.0
    assert f.risk_at(5.0, 0.0) == 0.0
    assert f.risk_at(0.79, 0.0) > 0.0


def test_distant_incident_does_not_touch_unrelated_route():
    f = field(ev("far", 0.0, 1.5))
    route = [(x / 10.0, -1.0) for x in range(0, 50)]
    m = f.path_metrics(route)
    assert m["accumulated_risk"] == 0.0
    assert m["contributing_events"] == []
    assert m["min_event_distance_m"] == pytest.approx(2.5, abs=0.01)


def test_severity_is_monotone():
    vals = [field(ev("a", 0, 0, sev=s)).risk_at(0.1, 0.1) for s in (0.1, 0.3, 0.6, 1.0)]
    assert vals == sorted(vals) and len(set(vals)) == 4


def test_kernel_decreases_with_distance():
    f = field(ev("a", 0, 0))
    vals = [f.risk_at(d, 0) for d in (0.0, 0.2, 0.4, 0.6, 0.79)]
    assert vals == sorted(vals, reverse=True)


def test_multiple_incidents_near_one_spot_add_up_but_stay_capped():
    one = field(ev("a", 0, 0, sev=0.3))
    three = field(ev("a", 0, 0, sev=0.3), ev("b", 0.05, 0, sev=0.3), ev("c", 0, 0.05, sev=0.3))
    assert three.risk_at(0, 0) > one.risk_at(0, 0)
    many = field(*[ev(f"e{i}", 0, 0) for i in range(50)])
    assert max(many.rasterize(INFO)) == 90


def test_confidence_scales_risk():
    assert field(ev("a", 0, 0, conf=0.5)).risk_at(0, 0) == pytest.approx(0.5)


def test_nan_and_inf_events_are_skipped_and_counted():
    bad = [ev("n1", math.nan, 0), ev("n2", 0, math.inf), ev("ok", 0, 0)]
    f = field(*bad)
    assert f.skipped_nonfinite == 2
    data = f.rasterize(INFO)
    assert all(isinstance(v, int) and 0 <= v <= 90 for v in data)
    assert f.risk_at(math.nan, 0.0) == 0.0


def test_nan_severity_contributes_nothing():
    f = field(ev("n", 0, 0, sev=math.nan))
    assert f.risk_at(0, 0) == 0.0


def test_extreme_severity_is_clamped():
    f = field(ev("x", 0, 0, sev=1e9))
    assert f.risk_at(0, 0) == pytest.approx(1.0)


def test_events_in_other_frames_are_excluded():
    f = field(ev("odom_ev", 0, 0, frame="odom"), ev("unposed", 0, 0, frame=""))
    assert f.skipped_frame == 2
    assert f.risk_at(0, 0) == 0.0


def test_decay_reduces_old_events():
    params = RiskFieldParams(radius_m=0.8, decay_half_life_s=100.0)
    f = RiskField.from_events([ev("old", 0, 0, t=0.0)], params, now=100.0)
    assert f.risk_at(0, 0) == pytest.approx(0.5)
    assert field(ev("new", 0, 0, t=100.0), params=params, now=100.0).risk_at(0, 0) == pytest.approx(1.0)


def test_rasterize_is_deterministic_regardless_of_event_order():
    es = [ev(f"e{i}", random.uniform(0, 4), random.uniform(-1, 1), sev=random.random())
          for i in range(30)]
    a = field(*es).rasterize(INFO)
    b = field(*reversed(es)).rasterize(INFO)
    assert a == b


def test_grid_never_contains_lethal_values_random():
    rng = random.Random(7)
    for _ in range(20):
        es = [ev(f"e{i}", rng.uniform(-1, 6), rng.uniform(-2, 2), sev=rng.random() * 3)
              for i in range(rng.randint(0, 40))]
        data = field(*es).rasterize(INFO)
        st = grid_stats(data)
        assert st["max_value"] <= 90 < 100
        assert st["out_of_range_cells"] == 0


def test_rasterized_cell_matches_point_query():
    f = field(ev("a", 1.0, 0.5))
    data = f.rasterize(INFO)
    cell = INFO.world_to_cell(1.0, 0.5)
    cx, cy = INFO.cell_center(*cell)
    assert data[INFO.index(*cell)] == f.value_for_risk(f.risk_at(cx, cy))


def test_event_outside_grid_does_not_crash():
    f = field(ev("out", 100.0, 100.0))
    assert set(f.rasterize(INFO)) == {0}


@pytest.mark.parametrize("kw", [dict(max_cell_value=100), dict(max_cell_value=0),
                                dict(radius_m=0.0), dict(radius_m=math.nan),
                                dict(value_per_unit_risk=-1.0), dict(decay_half_life_s=-1)])
def test_invalid_params_rejected(kw):
    with pytest.raises(ValueError):
        RiskFieldParams(**kw)


def test_max_nonlethal_constant_is_below_nav2_lethal():
    assert MAX_NONLETHAL_VALUE < 100
    RiskFieldParams(max_cell_value=MAX_NONLETHAL_VALUE)


def test_path_metrics_through_event():
    f = field(ev("a", 2.0, 0.0))
    straight = [(0.0, 0.0), (4.0, 0.0)]
    m = f.path_metrics(straight)
    assert m["length_m"] == pytest.approx(4.0)
    # integral of (1 - (x/0.8)^2) over [-0.8, 0.8] = 2 * 0.8 * 2/3
    assert m["accumulated_risk"] == pytest.approx(2 * 0.8 * 2 / 3, rel=0.02)
    assert m["max_risk"] == pytest.approx(1.0, abs=1e-3)
    assert m["min_event_distance_m"] == pytest.approx(0.0, abs=1e-6)
    assert m["contributing_events"] == ["a"]
    assert m["risk_length_m"] == pytest.approx(1.6, abs=0.1)


def test_path_metrics_handle_degenerate_input():
    f = field(ev("a", 0, 0))
    assert f.path_metrics([])["length_m"] == 0.0
    assert f.path_metrics([(0, 0)])["accumulated_risk"] == 0.0
    m = f.path_metrics([(0, 0), (math.nan, 1), (1, 0)])
    assert m["nonfinite_points"] == 1


def test_gridinfo_world_to_cell_boundaries():
    assert INFO.world_to_cell(-1.5, -2.0) == (0, 0)
    assert INFO.world_to_cell(-1.5 + 160 * 0.05, 0) is None
    assert INFO.world_to_cell(-1.51, 0) is None
    assert INFO.world_to_cell(math.inf, 0) is None
    last_x = -1.5 + 160 * 0.05 - 1e-9
    assert INFO.world_to_cell(last_x, 0)[0] == 159


def test_gridinfo_rejects_bad_geometry():
    with pytest.raises(ValueError):
        GridInfo(resolution=0.0, width=1, height=1, origin_x=0, origin_y=0)
    with pytest.raises(ValueError):
        GridInfo(resolution=0.05, width=0, height=1, origin_x=0, origin_y=0)
