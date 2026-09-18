"""Risk event and factor data classes (pure Python, no ROS coupling)."""
from __future__ import annotations

import math
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple


class FactorCategory(str, Enum):
    SLIP = "SLIP"
    SAFETY = "SAFETY"
    DEPTH = "DEPTH"
    AUDIO = "AUDIO"
    FAULT = "FAULT"
    HUMAN = "HUMAN"
    COLLISION = "COLLISION"
    OTHER = "OTHER"

    @classmethod
    def coerce(cls, raw) -> "FactorCategory":
        if isinstance(raw, FactorCategory):
            return raw
        if not isinstance(raw, str):
            return cls.OTHER
        try:
            return cls(raw.upper())
        except ValueError:
            return cls.OTHER


def _clamp01(x: float) -> float:
    """Clamp to [0, 1]. NaN maps to 0.0 so one bad factor cannot poison a sum."""
    x = float(x)
    if x != x:  # NaN
        return 0.0
    if x < 0.0:
        return 0.0
    if x > 1.0:
        return 1.0
    return float(x)


@dataclass
class RiskFactor:
    category: FactorCategory
    severity: float
    source: str
    detail: str = ""

    def __post_init__(self) -> None:
        self.category = FactorCategory.coerce(self.category)
        self.severity = _clamp01(self.severity)


Point3 = Tuple[float, float, float]


class Provenance(str, Enum):
    """Where a risk observation came from.

    Stored on every row so evidence classes can never be mixed up after the
    fact. Only ``HARDWARE_DERIVED`` and ``OPERATOR_INJECTED`` rows may be
    ingested by a node running in ``live`` mode; the rest describe data that
    must never be presented as hardware evidence.
    """

    HARDWARE_DERIVED = "HARDWARE_DERIVED"    # computed from real robot / ROS measurements
    OPERATOR_INJECTED = "OPERATOR_INJECTED"  # deliberate test event at a real, live pose
    REPLAY = "REPLAY"                        # re-ingested from a rosbag
    SYNTHETIC = "SYNTHETIC"                  # fixtures, demos, unit tests
    SIMULATION = "SIMULATION"                # produced by a simulator or rehearsal robot
    UNKNOWN = "UNKNOWN"                      # legacy rows written before provenance existed

    @classmethod
    def coerce(cls, raw) -> "Provenance":
        if isinstance(raw, Provenance):
            return raw
        if not isinstance(raw, str) or not raw:
            return cls.UNKNOWN
        try:
            return cls(raw.upper())
        except ValueError:
            return cls.UNKNOWN


#: Provenances a node in ``live`` run mode accepts. Anything else arriving on
#: the live topic is quarantined rather than stored as risk.
LIVE_PROVENANCES = frozenset({Provenance.HARDWARE_DERIVED, Provenance.OPERATOR_INJECTED})


@dataclass
class RiskEvent:
    event_id: str
    position: Point3
    factors: List[RiskFactor]
    confidence: float = 1.0
    timestamp: float = field(default_factory=time.time)
    frame_id: str = "map"
    segment_id: Optional[str] = None  # assigned at ingestion, after spatial join
    provenance: Provenance = Provenance.UNKNOWN
    source_frame_id: str = ""   # frame the observation arrived in, before any transform
    map_id: str = ""            # map identity the position is expressed against
    run_mode: str = ""          # run mode of the process that ingested it (live, replay, ...)
    ingest_time: float = 0.0    # wall/ROS time at which the memory node stored it
    clock_note: str = ""        # set when the event stamp was replaced (implausible clock)

    def __post_init__(self) -> None:
        if not self.factors:
            raise ValueError("RiskEvent requires at least one RiskFactor")
        self.confidence = _clamp01(self.confidence)
        self.provenance = Provenance.coerce(self.provenance)
        # Coerce factor types if a caller passed dicts/strings.
        self.factors = [
            f if isinstance(f, RiskFactor) else RiskFactor(**f) for f in self.factors
        ]

    @staticmethod
    def new_id() -> str:
        return str(uuid.uuid4())

    def is_finite(self) -> bool:
        """True when position, timestamp and confidence are all finite numbers."""
        try:
            vals = (float(self.position[0]), float(self.position[1]),
                    float(self.position[2]), float(self.timestamp),
                    float(self.confidence))
        except (TypeError, ValueError, IndexError):
            return False
        return all(math.isfinite(v) for v in vals)

    def dominant_category(self) -> FactorCategory:
        return max(self.factors, key=lambda f: f.severity).category

    def aggregate_severity(self) -> float:
        """Severity attributable to this event after confidence weighting.

        We use max-factor-severity rather than sum, on the assumption that the
        factors describe the *same* incident from different sensing modalities;
        summing would double-count a single slip seen by both proprio and IMU.
        """
        if not self.factors:
            return 0.0
        return _clamp01(max(f.severity for f in self.factors) * self.confidence)
