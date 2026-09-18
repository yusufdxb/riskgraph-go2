"""Spatial risk field: stored events -> risk value at any map point -> grid.

This is the representation Nav2 consumes. Each stored event contributes a
bounded bump centred on its map position:

    w_e      = aggregate_severity(e) * decay(age_e)
    k(d)     = 1 - (d / r)^2   for d < r, else 0
    risk(p)  = sum_e w_e * k(|p - x_e|)

``risk`` is dimensionless (1.0 is one full-severity event at its centre). A
grid cell's value is ``min(max_cell_value, round(value_per_unit_risk * risk))``
in OccupancyGrid units (0..100).

Why these choices, briefly:

* The kernel has compact support, so an event can never affect a route more
  than ``r`` metres away ("distant incidents do not affect unrelated routes"
  is a property, not a tuning outcome).
* Contributions add, so several incidents near one spot rank above one.
* ``max_cell_value`` is hard-capped below 100 (:data:`MAX_NONLETHAL_VALUE`).
  Nav2 interprets 100 as lethal; risk is a planning cost, never an obstacle,
  so the planner can always still route through risk when nothing else
  exists (the fallback semantics).
* Non-finite inputs are skipped and counted, never propagated into the grid.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from .events import RiskEvent
from .geometry import resample_polyline

#: Largest OccupancyGrid value RiskGraph will ever publish. Nav2's static
#: layer maps 100 to LETHAL; 95 maps to cost ~241, below INSCRIBED (253), so
#: a risk cell is never treated as an obstacle and is never inflated.
MAX_NONLETHAL_VALUE = 95


@dataclass(frozen=True)
class GridInfo:
    """Geometry of an OccupancyGrid. Origin yaw must be zero (map_server's)."""

    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float

    def __post_init__(self) -> None:
        if not (self.resolution > 0 and math.isfinite(self.resolution)):
            raise ValueError(f"resolution must be positive and finite, got {self.resolution}")
        if self.width <= 0 or self.height <= 0:
            raise ValueError(f"grid must be non-empty, got {self.width}x{self.height}")
        if not (math.isfinite(self.origin_x) and math.isfinite(self.origin_y)):
            raise ValueError("grid origin must be finite")

    def cell_center(self, ix: int, iy: int) -> Tuple[float, float]:
        return (self.origin_x + (ix + 0.5) * self.resolution,
                self.origin_y + (iy + 0.5) * self.resolution)

    def world_to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        ix = int(math.floor((x - self.origin_x) / self.resolution))
        iy = int(math.floor((y - self.origin_y) / self.resolution))
        if 0 <= ix < self.width and 0 <= iy < self.height:
            return ix, iy
        return None

    def index(self, ix: int, iy: int) -> int:
        return iy * self.width + ix

    def contains(self, x: float, y: float) -> bool:
        return self.world_to_cell(x, y) is not None

    def same_geometry(self, other: "GridInfo", tol: float = 1e-6) -> bool:
        return (self.width == other.width and self.height == other.height
                and abs(self.resolution - other.resolution) < tol
                and abs(self.origin_x - other.origin_x) < tol
                and abs(self.origin_y - other.origin_y) < tol)


@dataclass(frozen=True)
class RiskFieldParams:
    radius_m: float = 0.8
    decay_half_life_s: float = 0.0          # 0 disables decay
    value_per_unit_risk: float = 90.0       # one full-severity event peaks at 90
    max_cell_value: int = 90
    min_value: int = 1                      # cells below this are published as 0

    def __post_init__(self) -> None:
        if not (self.radius_m > 0 and math.isfinite(self.radius_m)):
            raise ValueError(f"radius_m must be positive, got {self.radius_m}")
        if not (0 < self.max_cell_value <= MAX_NONLETHAL_VALUE):
            raise ValueError(
                f"max_cell_value must be in 1..{MAX_NONLETHAL_VALUE} (100 is lethal in Nav2), "
                f"got {self.max_cell_value}")
        if not (self.value_per_unit_risk > 0 and math.isfinite(self.value_per_unit_risk)):
            raise ValueError("value_per_unit_risk must be positive and finite")
        if self.decay_half_life_s < 0 or not math.isfinite(self.decay_half_life_s):
            raise ValueError("decay_half_life_s must be >= 0 and finite")
        if not (0 <= self.min_value <= self.max_cell_value):
            raise ValueError("min_value must be within 0..max_cell_value")


@dataclass(frozen=True)
class _Bump:
    event_id: str
    x: float
    y: float
    weight: float


@dataclass
class RiskField:
    """Immutable-in-practice risk field over a set of events at time ``now``."""

    bumps: List[_Bump]
    params: RiskFieldParams
    skipped_nonfinite: int = 0
    skipped_frame: int = 0
    skipped_zero: int = 0
    frame_id: str = "map"

    @classmethod
    def from_events(cls, events: Iterable[RiskEvent], params: RiskFieldParams,
                    now: float, frame_id: str = "map") -> "RiskField":
        lam = (math.log(2.0) / params.decay_half_life_s) if params.decay_half_life_s > 0 else 0.0
        bumps: List[_Bump] = []
        nonfinite = wrong_frame = zero = 0
        for ev in events:
            if ev.frame_id != frame_id:
                wrong_frame += 1
                continue
            if not ev.is_finite():
                nonfinite += 1
                continue
            w = ev.aggregate_severity()
            if lam > 0:
                w *= math.exp(-lam * max(0.0, float(now) - ev.timestamp))
            if not math.isfinite(w):
                nonfinite += 1
                continue
            if w <= 0.0:
                zero += 1
                continue
            bumps.append(_Bump(ev.event_id, float(ev.position[0]), float(ev.position[1]), w))
        bumps.sort(key=lambda b: b.event_id)  # deterministic accumulation order
        return cls(bumps=bumps, params=params, skipped_nonfinite=nonfinite,
                   skipped_frame=wrong_frame, skipped_zero=zero, frame_id=frame_id)

    # -- point queries ------------------------------------------------------

    def _kernel(self, d: float) -> float:
        r = self.params.radius_m
        if d >= r:
            return 0.0
        q = d / r
        return 1.0 - q * q

    def risk_at(self, x: float, y: float) -> float:
        if not (math.isfinite(x) and math.isfinite(y)):
            return 0.0
        total = 0.0
        for b in self.bumps:
            total += b.weight * self._kernel(math.hypot(x - b.x, y - b.y))
        return total

    def value_for_risk(self, risk: float) -> int:
        """OccupancyGrid value (0..max_cell_value) for a risk level."""
        if not math.isfinite(risk) or risk <= 0.0:
            return 0
        v = int(round(self.params.value_per_unit_risk * risk))
        if v < self.params.min_value:
            return 0
        return min(self.params.max_cell_value, v)

    def contributing_events(self, x: float, y: float) -> List[str]:
        return [b.event_id for b in self.bumps
                if math.hypot(x - b.x, y - b.y) < self.params.radius_m]

    # -- grid ---------------------------------------------------------------

    def rasterize(self, info: GridInfo) -> List[int]:
        """Row-major OccupancyGrid data, every value in 0..max_cell_value."""
        data = [0] * (info.width * info.height)
        r = self.params.radius_m
        # Accumulate risk per cell only inside each bump's bounding box.
        acc: Dict[int, float] = {}
        for b in self.bumps:
            ix0 = max(0, int(math.floor((b.x - r - info.origin_x) / info.resolution)))
            ix1 = min(info.width - 1, int(math.floor((b.x + r - info.origin_x) / info.resolution)))
            iy0 = max(0, int(math.floor((b.y - r - info.origin_y) / info.resolution)))
            iy1 = min(info.height - 1, int(math.floor((b.y + r - info.origin_y) / info.resolution)))
            for iy in range(iy0, iy1 + 1):
                for ix in range(ix0, ix1 + 1):
                    cx, cy = info.cell_center(ix, iy)
                    k = self._kernel(math.hypot(cx - b.x, cy - b.y))
                    if k > 0.0:
                        idx = info.index(ix, iy)
                        acc[idx] = acc.get(idx, 0.0) + b.weight * k
        for idx, risk in acc.items():
            data[idx] = self.value_for_risk(risk)
        return data

    # -- path metrics -------------------------------------------------------

    def path_metrics(self, points: Sequence[Tuple[float, float]], step: float = 0.05
                     ) -> Dict[str, object]:
        """Risk exposure of a polyline, sampled every ``step`` metres.

        accumulated_risk is the line integral of risk along the path
        (risk x metres); max_risk is the peak; min_event_distance_m is the
        closest approach to any active event centre.
        """
        pts = [(float(x), float(y)) for x, y in points
               if math.isfinite(float(x)) and math.isfinite(float(y))]
        n_bad = len(points) - len(pts)
        if len(pts) < 2:
            return {"length_m": 0.0, "accumulated_risk": 0.0, "max_risk": 0.0,
                    "mean_risk": 0.0, "max_cell_value": 0, "min_event_distance_m": None,
                    "n_samples": len(pts), "nonfinite_points": n_bad,
                    "risk_length_m": 0.0, "contributing_events": []}
        samples = resample_polyline(pts, step)
        risks = [self.risk_at(x, y) for x, y in samples]
        length = 0.0
        acc = 0.0
        risky_len = 0.0
        for (a, ra), (b, rb) in zip(zip(samples, risks), zip(samples[1:], risks[1:])):
            seg = math.hypot(b[0] - a[0], b[1] - a[1])
            length += seg
            acc += 0.5 * (ra + rb) * seg
            if ra > 0.0 or rb > 0.0:
                risky_len += seg
        min_d = None
        contributing = set()
        for bmp in self.bumps:
            d = min(math.hypot(x - bmp.x, y - bmp.y) for x, y in samples)
            min_d = d if min_d is None else min(min_d, d)
            if d < self.params.radius_m:
                contributing.add(bmp.event_id)
        max_r = max(risks) if risks else 0.0
        return {
            "length_m": length,
            "accumulated_risk": acc,
            "max_risk": max_r,
            "mean_risk": (acc / length) if length > 0 else 0.0,
            "max_cell_value": self.value_for_risk(max_r),
            "min_event_distance_m": min_d,
            "n_samples": len(samples),
            "nonfinite_points": n_bad,
            "risk_length_m": risky_len,
            "contributing_events": sorted(contributing),
        }


def grid_stats(data: Sequence[int]) -> Dict[str, object]:
    """Summary of a published grid; used by status output and abort checks."""
    nonzero = [v for v in data if v != 0]
    bad = [v for v in data if not isinstance(v, int) or v < 0 or v > MAX_NONLETHAL_VALUE]
    return {
        "cells": len(data),
        "nonzero_cells": len(nonzero),
        "max_value": max(data) if data else 0,
        "sum_value": int(sum(data)),
        "out_of_range_cells": len(bad),
    }
