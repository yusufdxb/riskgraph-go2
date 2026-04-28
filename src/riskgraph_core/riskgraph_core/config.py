"""YAML config loader for RiskGraph-Go2 weights and store path.

Defaults are calibrated so a partial config still produces a sensible scorer:
geometry weight 1.0, semantic weight 1.0, risk weight 2.0, no decay.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict

import yaml

from .scoring import ScoringWeights


@dataclass
class Config:
    weights: ScoringWeights
    store_path: str = ":memory:"
    raw: Dict[str, Any] = None  # original dict, for diagnostics


def _build_weights(d: Dict[str, Any]) -> ScoringWeights:
    weights_section = d.get("weights", {}) or {}
    return ScoringWeights(
        geometry=float(weights_section.get("geometry", 1.0)),
        semantic=float(weights_section.get("semantic", 1.0)),
        risk=float(weights_section.get("risk", 2.0)),
        decay_half_life_s=float(d.get("decay_half_life_s", 0.0)),
    )


def load_config(path: str) -> Config:
    with open(path, "r") as fh:
        raw = yaml.safe_load(fh) or {}
    if not isinstance(raw, dict):
        raise ValueError(f"Config root must be a mapping, got {type(raw).__name__}")
    weights = _build_weights(raw)
    store_path = str(raw.get("store_path", ":memory:"))
    cfg = Config(weights=weights, store_path=store_path, raw=raw)
    return cfg
