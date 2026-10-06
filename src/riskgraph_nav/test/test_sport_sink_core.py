"""Tests for the pure sport sink logic. No ROS."""
import json
import math
from types import SimpleNamespace

import pytest

from riskgraph_nav.sport_sink_core import (
    ALLOWED_API_IDS,
    API_MOVE,
    API_STOP_MOVE,
    MODE_ARMED,
    MODE_DRY_RUN,
    MODE_STOP_ONLY,
    SinkLimits,
    SinkLogic,
    SportRequest,
    fill_unitree_request,
)


def fake_msg():
    return SimpleNamespace(
        header=SimpleNamespace(
            identity=SimpleNamespace(id=-1, api_id=-1),
            lease=SimpleNamespace(id=-1),
            policy=SimpleNamespace(priority=-1, noreply=True)),
        parameter='unset')


def test_allowed_ids():
    assert ALLOWED_API_IDS == {1003, 1008}


def test_invalid_mode_raises():
    with pytest.raises(ValueError):
        SinkLogic('bogus')


@pytest.mark.parametrize('mode', [MODE_DRY_RUN, MODE_ARMED])
def test_move_in_dry_run_and_armed(mode):
    r = SinkLogic(mode).on_command(0.1, 0.05, -0.2, 1.0)
    assert r.api_id == API_MOVE and r.reason == 'MOVE'
    assert json.loads(r.parameter) == {'x': 0.1, 'y': 0.05, 'z': -0.2}
    assert (r.x, r.y, r.z) == (0.1, 0.05, -0.2)


def test_stop_only_never_moves():
    s = SinkLogic(MODE_STOP_ONLY)
    for i in range(20):
        r = s.on_command(0.1, 0.0, 0.0, i * 0.01)
        assert r is None or r.api_id == API_STOP_MOVE
    first = SinkLogic(MODE_STOP_ONLY).on_command(0.1, 0.0, 0.0, 0.0)
    assert first.api_id == API_STOP_MOVE and first.reason == 'NOT_ARMED'
    assert first.parameter == ''


def test_zero_transition_then_repeat():
    s = SinkLogic(MODE_ARMED, stop_repeat_hz=2.0)
    assert s.on_command(0.1, 0.0, 0.0, 0.0).api_id == API_MOVE
    r = s.on_command(0.0, 0.0, 0.0, 0.05)
    assert r.api_id == API_STOP_MOVE and r.reason == 'ZERO'
    assert s.on_command(0.0, 0.0, 0.0, 0.10) is None
    assert s.on_command(0.0, 0.0, 0.0, 0.40) is None
    r = s.on_command(0.0, 0.0, 0.0, 0.56)
    assert r is not None and r.api_id == API_STOP_MOVE


def test_move_rate_limit():
    s = SinkLogic(MODE_ARMED, move_hz=20.0)
    assert s.on_command(0.1, 0.0, 0.0, 0.00) is not None
    assert s.on_command(0.1, 0.0, 0.0, 0.02) is None
    assert s.on_command(0.1, 0.0, 0.0, 0.049) is None
    assert s.on_command(0.1, 0.0, 0.0, 0.051) is not None


@pytest.mark.parametrize('cmd', [
    (0.26, 0.0, 0.0), (-0.26, 0.0, 0.0),
    (0.0, 0.21, 0.0), (0.0, -0.21, 0.0),
    (0.0, 0.0, 0.51), (0.0, 0.0, -0.51),
])
@pytest.mark.parametrize('mode', [MODE_DRY_RUN, MODE_STOP_ONLY, MODE_ARMED])
def test_over_limit_rejected_not_clamped(mode, cmd):
    s = SinkLogic(mode)
    s.on_command(0.0, 0.0, 0.0, 0.0)  # already stopped: rejection must still be forced
    r = s.on_command(*cmd, 0.01)
    assert r.api_id == API_STOP_MOVE and r.reason == 'REJECT_OVER_SINK_LIMIT'
    assert r.parameter == ''


def test_at_limit_is_allowed():
    r = SinkLogic(MODE_ARMED).on_command(0.25, 0.20, 0.50, 0.0)
    assert r.api_id == API_MOVE


def test_custom_limits():
    s = SinkLogic(MODE_ARMED, SinkLimits(0.1, 0.1, 0.1))
    assert s.on_command(0.2, 0.0, 0.0, 0.0).reason == 'REJECT_OVER_SINK_LIMIT'


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
@pytest.mark.parametrize('slot', [0, 1, 2])
def test_non_finite_rejected(bad, slot):
    s = SinkLogic(MODE_ARMED)
    s.on_command(0.0, 0.0, 0.0, 0.0)
    cmd = [0.0, 0.0, 0.0]
    cmd[slot] = bad
    r = s.on_command(*cmd, 0.01)
    assert r.api_id == API_STOP_MOVE and r.reason == 'REJECT_NON_FINITE'


def test_deadman_forced_then_repeats_then_resets():
    s = SinkLogic(MODE_ARMED, input_timeout_sec=0.25, stop_repeat_hz=2.0)
    assert s.on_command(0.1, 0.0, 0.0, 0.0).api_id == API_MOVE
    assert s.on_tick(0.20) is None
    r = s.on_tick(0.30)
    assert r.api_id == API_STOP_MOVE and r.reason == 'DEADMAN'
    assert s.deadman_tripped
    assert s.on_tick(0.35) is None
    assert s.on_tick(0.79) is None
    r = s.on_tick(0.81)
    assert r is not None and r.reason == 'DEADMAN'
    # new input resets the trip; next trip is forced again
    assert s.on_command(0.1, 0.0, 0.0, 1.0).api_id == API_MOVE
    assert not s.deadman_tripped
    assert s.on_tick(1.1) is None
    r = s.on_tick(1.30)
    assert r is not None and r.reason == 'DEADMAN'


def test_deadman_with_no_input_ever():
    r = SinkLogic(MODE_DRY_RUN).on_tick(0.0)
    assert r.api_id == API_STOP_MOVE and r.reason == 'DEADMAN'


def test_fill_move():
    from riskgraph_nav.sport_sink_core import move
    m = fill_unitree_request(fake_msg(), move(0.1, 0.0, 0.2), 42)
    assert m.header.identity.id == 42
    assert m.header.identity.api_id == API_MOVE
    assert m.header.lease.id == 0
    assert m.header.policy.priority == 0
    assert m.header.policy.noreply is False
    assert json.loads(m.parameter) == {'x': 0.1, 'y': 0.0, 'z': 0.2}


def test_fill_stop():
    from riskgraph_nav.sport_sink_core import stop_move
    m = fill_unitree_request(fake_msg(), stop_move('ZERO'), 7)
    assert m.header.identity.api_id == API_STOP_MOVE
    assert m.parameter == ''


@pytest.mark.parametrize('bad_id', [1001, 1002, 1004, 0, -1, 1009])
def test_fill_refuses_non_allowed(bad_id):
    msg = fake_msg()
    with pytest.raises(ValueError):
        fill_unitree_request(msg, SportRequest(bad_id, '', 'X'), 1)
    assert msg.header.identity.api_id == -1  # untouched
