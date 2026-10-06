"""Pure logic for the RiskGraph GO2 sport sink. No ROS imports.

The sink is the only RiskGraph publisher on /api/sport/request, and its only
input is Nav2's /nav/cmd_vel. It is a stateless translator plus a deadman:

  zero command          -> StopMove (1003), on the transition and then at
                           stop_repeat_hz while zero
  nonzero, armed/dry_run -> Move (1008) {"x","y","z"} at most move_hz
  nonzero, stop_only    -> StopMove (never Move)
  over sink limits      -> StopMove (rejected, never clamped)
  non-finite command    -> StopMove (rejected)
  input older than input_timeout_sec -> StopMove (DEADMAN)

dry_run makes exactly the armed decisions so its trace shows what would
have been sent; the node never publishes to the robot in dry_run.

Only API ids 1003 and 1008 exist here. Id 1001 on that topic is Damp and drops
the robot; it is refused by construction.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Optional

API_STOP_MOVE = 1003
API_MOVE = 1008
ALLOWED_API_IDS = frozenset({API_STOP_MOVE, API_MOVE})

MODE_DRY_RUN = 'dry_run'      # armed decisions, nothing reaches the robot
MODE_STOP_ONLY = 'stop_only'  # only StopMove reaches the robot
MODE_ARMED = 'armed'          # Move allowed within sink limits
MODES = (MODE_DRY_RUN, MODE_STOP_ONLY, MODE_ARMED)

NODE_NAME = 'riskgraph_sport_sink'
INPUT_TOPIC = '/nav/cmd_vel'
TRACE_TOPIC = '/riskgraph/sink/trace'
#: Idle, the sink traces a StopMove at stop_repeat_hz (2 Hz): older than this means it is gone.
TRACE_MAX_AGE_S = 1.0


@dataclass(frozen=True)
class SinkLimits:
    max_vx: float = 0.25
    max_vy: float = 0.20
    max_wz: float = 0.50


@dataclass(frozen=True)
class SportRequest:
    api_id: int
    parameter: str       # JSON string, '' for StopMove
    reason: str
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


def stop_move(reason: str) -> SportRequest:
    return SportRequest(API_STOP_MOVE, '', reason)


def move(x: float, y: float, z: float) -> SportRequest:
    return SportRequest(API_MOVE, json.dumps({'x': x, 'y': y, 'z': z}), 'MOVE', x, y, z)


class SinkLogic:

    def __init__(self, mode: str, limits: SinkLimits = SinkLimits(),
                 input_timeout_sec: float = 0.25, stop_repeat_hz: float = 2.0,
                 move_hz: float = 20.0) -> None:
        if mode not in MODES:
            raise ValueError(f'mode must be one of {MODES}, got {mode!r}')
        self.mode = mode
        self.limits = limits
        self.input_timeout_sec = input_timeout_sec
        self._stop_period = 1.0 / stop_repeat_hz
        self._move_period = 1.0 / move_hz
        self._last_input_at: Optional[float] = None
        self._last_sent_at: Optional[float] = None
        self._last_api: Optional[int] = None
        self.deadman_tripped = False

    def on_command(self, vx: float, vy: float, wz: float, now: float) -> Optional[SportRequest]:
        self._last_input_at = now
        self.deadman_tripped = False
        vals = (vx, vy, wz)
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals):
            return self._stop('REJECT_NON_FINITE', now, force=True)
        if vx == 0.0 and vy == 0.0 and wz == 0.0:
            return self._stop('ZERO', now)
        if (abs(vx) > self.limits.max_vx or abs(vy) > self.limits.max_vy
                or abs(wz) > self.limits.max_wz):
            return self._stop('REJECT_OVER_SINK_LIMIT', now, force=True)
        if self.mode == MODE_STOP_ONLY:
            return self._stop('NOT_ARMED', now)
        if (self._last_api == API_MOVE and self._last_sent_at is not None
                and now - self._last_sent_at < self._move_period):
            return None
        return self._sent(move(float(vx), float(vy), float(wz)), now)

    def on_tick(self, now: float) -> Optional[SportRequest]:
        """Deadman check; call periodically."""
        if self._last_input_at is None or now - self._last_input_at > self.input_timeout_sec:
            first = not self.deadman_tripped
            self.deadman_tripped = True
            return self._stop('DEADMAN', now, force=first)
        return None

    def _stop(self, reason: str, now: float, force: bool = False) -> Optional[SportRequest]:
        transition = self._last_api != API_STOP_MOVE
        due = self._last_sent_at is None or now - self._last_sent_at >= self._stop_period
        if force or transition or due:
            return self._sent(stop_move(reason), now)
        return None

    def _sent(self, req: SportRequest, now: float) -> SportRequest:
        assert req.api_id in ALLOWED_API_IDS
        self._last_api = req.api_id
        self._last_sent_at = now
        return req


def fill_unitree_request(msg, req: SportRequest, request_id: int):
    """Populate a unitree_api/msg/Request; refuses any id outside the allowlist."""
    if req.api_id not in ALLOWED_API_IDS:
        raise ValueError(f'api_id {req.api_id} refused')
    msg.header.identity.id = int(request_id)
    msg.header.identity.api_id = int(req.api_id)
    msg.header.lease.id = 0
    msg.header.policy.priority = 0
    msg.header.policy.noreply = False
    msg.parameter = req.parameter
    return msg
