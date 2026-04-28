"""Pure-Python risk model core for RiskGraph-Go2.

This package has no ROS dependency. All ROS coupling lives in the sibling
packages (riskgraph_memory, riskgraph_planner, riskgraph_explainer, etc.),
which import from this module.
"""

from .events import RiskFactor, RiskEvent, FactorCategory
from .segments import RouteSegment, Route, segment_for_point
from .store import RiskStore
from .scoring import ScoringWeights, score_routes, RouteScoreResult
from .explainer import explain_choice, ExplanationTemplate
from .config import load_config, Config

__all__ = [
    "RiskFactor",
    "RiskEvent",
    "FactorCategory",
    "RouteSegment",
    "Route",
    "segment_for_point",
    "RiskStore",
    "ScoringWeights",
    "score_routes",
    "RouteScoreResult",
    "explain_choice",
    "ExplanationTemplate",
    "load_config",
    "Config",
]
