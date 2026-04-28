"""Risk event and factor data classes (pure Python, no ROS coupling)."""
from __future__ import annotations

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


@dataclass
class RiskEvent:
    event_id: str
    position: Point3
    factors: List[RiskFactor]
    confidence: float = 1.0
    timestamp: float = field(default_factory=time.time)
    frame_id: str = "map"
    segment_id: Optional[str] = None  # assigned at ingestion, after spatial join

    def __post_init__(self) -> None:
        if not self.factors:
            raise ValueError("RiskEvent requires at least one RiskFactor")
        self.confidence = _clamp01(self.confidence)
        # Coerce factor types if a caller passed dicts/strings.
        self.factors = [
            f if isinstance(f, RiskFactor) else RiskFactor(**f) for f in self.factors
        ]

    @staticmethod
    def new_id() -> str:
        return str(uuid.uuid4())

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
