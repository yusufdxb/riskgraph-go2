# Validation Status

The honesty boundary in `AGENTS.md` requires three categories: verified offline, inferred runtime behavior, and hardware-dependent. This file is the single source of truth for what is in which category as of v0.1.0.

## Verified offline

These claims have been demonstrated on the workstation with concrete commands. Reproduction commands listed for each.

### `riskgraph_core` — pure-Python model

- 28 unit tests pass (`./scripts/run_tests.sh src/riskgraph_core/test`).
  - 6× event tests: severity clamping, category coercion, factor presence requirement, dominant category, aggregate severity (max-not-sum semantics).
  - 5× segment tests: Euclidean length, route total length, nearest-segment join (interior, endpoint-past, empty list).
  - 6× store tests: in-memory round-trip, file persistence across handles, no-decay sum, decay drops old events, dominant factor, unknown-segment zero risk.
  - 6× scoring tests: **safer-longer-route-beats-shorter-risky-route** (the headline regression), geometry-only short-wins, risk-weight zero ignores history, semantic match reduces cost, dominant segment / factor populated, decay reduces old event influence.
  - 3× explainer tests: avoid-risky cites factor + evidence ids, geometry-only mentions distance, semantic-match mentions objective.
  - 2× config tests: minimal load, defaults when missing.

### `riskgraph_memory.conversions` — ROS-msg → core dataclass translation

- 3 unit tests pass without rclpy/riskgraph_msgs (using `SimpleNamespace` mocks): basic round-trip, unknown-category fallback, route segment-order preservation.

### `riskgraph_demo.offline_demo` — end-to-end orchestrator

- 2 unit tests pass: bundled `glossy_hallway` scenario chooses the LONG route, explanation contains "slip", at least one event id is cited.

**Aggregate:** 33 tests, all green, both via `./scripts/run_tests.sh` and via `colcon test --packages-select riskgraph_core riskgraph_memory riskgraph_demo`.

### Offline demo regression

- `./scripts/run_offline_demo.sh` runs the full risk model end-to-end against the bundled scenario and exits with code 0. With the canonical weights (geometry=1.0, risk=4.0, half_life=1800s):
  - SHORT: total cost 14.523 (geom 4.00, risk 10.523, dominant SLIP on `glossy`)
  - LONG:  total cost  7.405 (geom 7.40, risk 0.000)
  - chosen: LONG
  - explanation: "Chose route LONG because the alternative passed through glossy where 3 prior slips have been recorded."
  - evidence ids: `[ev_slip_1, ev_slip_2, ev_safe_1]`

### colcon build + interface generation

- `colcon build --symlink-install` builds all 7 packages cleanly: `riskgraph_msgs`, `riskgraph_core`, `riskgraph_memory`, `riskgraph_planner`, `riskgraph_explainer`, `riskgraph_demo`, `riskgraph_bringup`.
- `ros2 interface list | grep riskgraph` reports all 7 messages and 2 services as expected.
- `ros2 interface show riskgraph_msgs/msg/RiskEvent` and `ros2 interface show riskgraph_msgs/srv/ScoreRoutes` produce well-formed nested IDL.

### Live ROS pipeline (workstation only)

- `ros2 launch riskgraph_bringup demo_offline.launch.py` brings up all 4 demo nodes; the synthetic publisher reports "all events replayed"; no errors in node logs.
- `python3 scripts/ros_end_to_end_check.py` (workstation) publishes 3 synthetic slip events via the `/riskgraph/risk_events` topic, calls `/riskgraph/score_routes` over rclpy, and asserts:
  - `chosen_route_id == "long"` ✓
  - explanation cites the actual published event ids `e2e_slip_0..2` ✓

## Inferred runtime behavior (not yet observed)

These claims are implied by the code but have not been exercised against live data on a robot. They depend on configuration choices that the workstation cannot fully validate.

- **Adapters translate upstream events correctly.** `safety_adapter`, `helix_adapter`, `tactile_adapter` have not been run against live `go2_msgs`, `helix_msgs`, or a real slip_state publisher. The translation tables are unit-tested via mock objects only.
- **Cross-run persistence on Jetson.** SQLite-on-NVMe is expected to behave identically to SQLite-on-/tmp, but not measured. WAL behaviour under power-loss has not been tested.
- **Decay parameters appropriate for real-world session lengths.** The default 1800 s half-life was chosen as a reasonable starting point for a 30-min session; tuning requires data from real traversals.

## Hardware-dependent (unverified)

These claims are explicitly **not** validated and require a CaresLab Go2 + Jetson Orin NX session.

- End-to-end behaviour with the live Go2 stack: that adapters subscribe successfully to upstream topics with the right QoS, that frame_ids match, that pose-bearing events spatially-join to the right segments.
- Performance: latency from event-publish to memory-write, latency of `/riskgraph/score_routes` under realistic candidate counts, SQLite throughput on Jetson NVMe.
- That the canonical weights (geometry=1.0, semantic=1.0, risk=4.0) produce useful behaviour for a low-vision user. This requires user-study data, not just synthetic regression.
- Cross-run memory: that running the stack twice with the same SQLite file produces correct second-run scoring, where the first run's events bias the second.
- Soft-dependency robustness on Jetson: that adapters cleanly no-op when an upstream package is genuinely missing on a real Jetson installation.

The next concrete validation step is a CaresLab session executing the test plan in `docs/hardware_integration.md`. Until that runs, no claim of "running on Go2" should be made anywhere in this repo, in writing, or in conversation.
