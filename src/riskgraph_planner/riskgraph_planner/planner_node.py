"""ROS 2 node: route scoring service.

Provides /riskgraph/score_routes (ScoreRoutes.srv). Reads weights and store
location from parameters; opens the same SQLite file the memory node writes to,
READ-ONLY, so scoring sees the live event log and can never create or modify it.

The store is opened lazily: if the memory node has not created the database
yet, a request fails loudly (empty result, explanation says why) and the next
request retries. ``store_path`` must be absolute; ``expected_map_id`` (if set)
must match the id the database is bound to.

This service scores explicit candidate routes (segment keyed). The live Nav2
integration does not go through it; Nav2 consumes /riskgraph/risk_costmap.
"""
from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header

from riskgraph_msgs.srv import ScoreRoutes
from riskgraph_msgs.msg import (
    RouteScore as RouteScoreMsg,
    RouteScoreArray as RouteScoreArrayMsg,
    RouteExplanation as RouteExplanationMsg,
)

from riskgraph_core.store import RiskStore, StoreError
from riskgraph_core.scoring import ScoringWeights, score_routes
from riskgraph_core.explainer import explain_choice

from riskgraph_memory.conversions import msg_route_to_core


class PlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("riskgraph_planner")
        self.declare_parameter("store_path", "")
        self.declare_parameter("expected_map_id", "")
        self.declare_parameter("weight_geometry", 1.0)
        self.declare_parameter("weight_semantic", 1.0)
        self.declare_parameter("weight_risk", 2.0)
        self.declare_parameter("decay_half_life_s", 0.0)

        store_path = self.get_parameter("store_path").get_parameter_value().string_value
        self._store_path = store_path
        self._expected_map_id = self.get_parameter("expected_map_id").value or None
        self._store = None
        self._store_error = ""
        self._open_store()

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self._scores_pub = self.create_publisher(
            RouteScoreArrayMsg, "/riskgraph/route_scores", qos
        )
        self._srv = self.create_service(
            ScoreRoutes, "/riskgraph/score_routes", self._on_score
        )
        self.get_logger().info(f"riskgraph_planner ready, store_path={store_path}")

    def _open_store(self):
        if self._store is not None:
            return self._store
        try:
            self._store = RiskStore(self._store_path, readonly=True,
                                    map_id=self._expected_map_id)
            self._store_error = ""
        except StoreError as exc:
            self._store_error = str(exc)
            self.get_logger().error(f"risk store unavailable: {exc}")
        return self._store

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
        if self._open_store() is None:
            response.result = RouteScoreArrayMsg()
            response.explanation = RouteExplanationMsg()
            response.explanation.text = f"risk store unavailable: {self._store_error}"
            return response
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
        # Also publish the score array on the topic so streaming consumers
        # (e.g. riskgraph_explainer_node, UI overlays) can react without
        # making their own service call.
        self._scores_pub.publish(arr)
        return response

    def destroy_node(self) -> bool:
        if self._store is not None:
            try:
                self._store.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    # Own SIGINT/SIGTERM handling: rclpy's default handler tears the context
    # down under the executor and the process used to exit non-zero.
    from rclpy.signals import SignalHandlerOptions
    from riskgraph_memory.memory_node import run_until_signal
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PlannerNode()
    try:
        run_until_signal(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
