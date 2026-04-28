"""ROS 2 node: route scoring service.

Provides /riskgraph/score_routes (ScoreRoutes.srv). Reads weights and store
location from parameters; opens the same SQLite file the memory node writes to,
so scoring sees the live event log.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from riskgraph_msgs.srv import ScoreRoutes
from riskgraph_msgs.msg import (
    RouteScore as RouteScoreMsg,
    RouteScoreArray as RouteScoreArrayMsg,
    RouteExplanation as RouteExplanationMsg,
)

from riskgraph_core.store import RiskStore
from riskgraph_core.scoring import ScoringWeights, score_routes
from riskgraph_core.explainer import explain_choice

from riskgraph_memory.conversions import msg_route_to_core


class PlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("riskgraph_planner")
        self.declare_parameter("store_path", ":memory:")
        self.declare_parameter("weight_geometry", 1.0)
        self.declare_parameter("weight_semantic", 1.0)
        self.declare_parameter("weight_risk", 2.0)
        self.declare_parameter("decay_half_life_s", 0.0)

        store_path = self.get_parameter("store_path").get_parameter_value().string_value
        self._store = RiskStore(store_path)

        self._srv = self.create_service(
            ScoreRoutes, "/riskgraph/score_routes", self._on_score
        )
        self.get_logger().info(f"riskgraph_planner ready, store_path={store_path}")

    def _weights(self) -> ScoringWeights:
        return ScoringWeights(
            geometry=float(self.get_parameter("weight_geometry").get_parameter_value().double_value),
            semantic=float(self.get_parameter("weight_semantic").get_parameter_value().double_value),
            risk=float(self.get_parameter("weight_risk").get_parameter_value().double_value),
            decay_half_life_s=float(self.get_parameter("decay_half_life_s").get_parameter_value().double_value),
        )

    def _on_score(self, request: ScoreRoutes.Request,
                  response: ScoreRoutes.Response) -> ScoreRoutes.Response:
        candidates = [msg_route_to_core(r) for r in request.candidates]
        weights = self._weights()
        result = score_routes(
            candidates, self._store, weights,
            semantic_objective=str(request.semantic_objective),
        )
        explanation = explain_choice(
            result, candidates, self._store,
            semantic_objective=str(request.semantic_objective),
        )

        arr = RouteScoreArrayMsg()
        arr.header = Header()
        arr.header.frame_id = "map"
        arr.scores = []
        for s in result.scores:
            m = RouteScoreMsg()
            m.route_id = s.route_id
            m.total_cost = float(s.total_cost)
            m.geometry_cost = float(s.geometry_cost)
            m.semantic_cost = float(s.semantic_cost)
            m.risk_cost = float(s.risk_cost)
            m.dominant_segment_ids = list(s.dominant_segment_ids)
            m.dominant_factor_categories = list(s.dominant_factor_categories)
            arr.scores.append(m)
        arr.chosen_route_id = result.chosen_route_id

        exp_msg = RouteExplanationMsg()
        exp_msg.header = Header()
        exp_msg.header.frame_id = "map"
        exp_msg.route_id = explanation.route_id
        exp_msg.text = explanation.text
        exp_msg.evidence_event_ids = list(explanation.evidence_event_ids)

        response.result = arr
        response.explanation = exp_msg
        return response

    def destroy_node(self) -> bool:
        try:
            self._store.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
