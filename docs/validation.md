# Validation Status

The honesty boundary in `AGENTS.md` requires three categories: verified offline, inferred runtime behavior, and hardware-dependent. This file is the single source of truth for what is in which category as of v0.1.0.

## Verified offline

These claims have been demonstrated on the workstation with concrete commands. Reproduction commands listed for each.

### `riskgraph_core`: pure-Python model

- 44 unit tests pass (`./scripts/run_tests.sh src/riskgraph_core/test`).
  - 6× event tests: severity clamping, category coercion, factor presence requirement, dominant category, aggregate severity (max-not-sum semantics).
  - 7× segment tests: Euclidean length, route total length, nearest-segment join (interior, endpoint-past, empty list, degenerate zero-length, near-collinear).
  - 7× store tests: in-memory round-trip, file persistence across handles, no-decay sum, decay drops old events, dominant factor, unknown-segment zero risk, concurrent reader atomicity.
  - 8× scoring tests: **safer-longer-route-beats-shorter-risky-route** (the headline regression), geometry-only short-wins, risk-weight zero ignores history, semantic match reduces cost, dominant segment / factor populated, decay reduces old event influence, plus zero-candidate and tie-resolution.
  - 3× explainer tests: avoid-risky cites factor + evidence ids, geometry-only mentions distance, semantic-match mentions objective.
  - 2× config tests: minimal load, defaults when missing.
  - 11× cross-run memory tests (new in v0.1.1): mock-pose path → segment join, cold start, empty pose stream, warm start, three-run accumulation, risk only elevated near patches, decay-on-restart, lateral-jitter robustness, dominant-category survival, evidence ranking after restart, 30-event stress.

### `riskgraph_memory.conversions`: ROS-msg → core dataclass translation

- 3 unit tests pass without rclpy/riskgraph_msgs (using `SimpleNamespace` mocks): basic round-trip, unknown-category fallback, route segment-order preservation.

### `riskgraph_memory.adapters`: soft-import upstream adapters

- 22 unit tests pass without rclpy or upstream packages, using `sys.modules`-injected stubs for `rclpy`, `geometry_msgs`, `std_msgs`, `riskgraph_msgs`, `go2_msgs`, `helix_msgs`. Both arms of each soft-import (`HAVE_GO2_MSGS`, `HAVE_HELIX_MSGS`) are exercised, and `main()` is asserted to be a clean no-op when the upstream msg package is missing.
  - 9× safety_adapter: alert-table lookup for all 5 known types + unknown fallback, `HAVE_GO2_MSGS` true/false toggling, `main()` no-op when `go2_msgs` is missing, callback publishes RiskEvent with correct severity/category/source/detail for DROP_DETECTED and EMERGENCY_STOP, blank frame_id falls back to "map", unknown alert type uses default factor.
  - 8× helix_adapter: severity-map levels (1/2/3 → 0.3/0.6/1.0), `HAVE_HELIX_MSGS` true/false toggling, `main()` no-op when `helix_msgs` is missing, callback publishes RiskEvent for CRITICAL fault with correct stamp synthesis from float timestamp, unknown severity → default 0.5, WARN severity, integer-second timestamp boundary.
  - 5× tactile_adapter: leading-edge debounce emits one event per rising edge, default severity 0.7 / category SLIP / source `tactile/slip_state`, starting-with-False is a no-op, truthy-int data is coerced to bool.

### `riskgraph_demo.offline_demo`: end-to-end orchestrator

- 2 unit tests pass: bundled `glossy_hallway` scenario chooses the LONG route, explanation contains "slip", at least one event id is cited.

**Aggregate:** 71 tests, all green, both via `./scripts/run_tests.sh` and via `colcon test --packages-select riskgraph_core riskgraph_memory riskgraph_demo`. Five of those tests were added in response to the Codex architecture audit (degenerate / near-collinear segment joins, zero-candidate scoring, tie resolution, concurrent SQLite reader). See `docs/audits/audit_log.md`. The 33 tests added in v0.1.1 cover cross-run memory (mock-pose paths through SQLite restart cycles) and soft-import adapters (both upstream-present and upstream-missing modes), strengthening pre-hardware confidence ahead of the CaresLab session.

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

- **Adapters translate upstream events correctly.** `safety_adapter`, `helix_adapter`, `tactile_adapter` have not been run against live `go2_msgs`, `helix_msgs`, or a real slip_state publisher. As of v0.1.1 each adapter has 5-9 unit tests covering both the upstream-present arm (translation table + callback shape) and the upstream-missing arm (`HAVE_*_MSGS=False`, `main()` no-op without spinning rclpy), but live message wire-format and QoS handshakes still need a hardware run.
- **Cross-run persistence on Jetson.** SQLite-on-NVMe is expected to behave identically to SQLite-on-/tmp, but not measured. WAL behaviour under power-loss has not been tested. The cross-run semantics (warm start sees prior risk, three-run accumulation, decay-on-restart) are now unit-tested against `tmp_path`-backed SQLite files in `test_cross_run_memory.py`.
- **Decay parameters appropriate for real-world session lengths.** The default 1800 s half-life was chosen as a reasonable starting point for a 30-min session; tuning requires data from real traversals.

## Hardware-dependent (unverified)

These claims are explicitly **not** validated and require a CaresLab Go2 + Jetson Orin NX session.

- End-to-end behaviour with the live Go2 stack: that adapters subscribe successfully to upstream topics with the right QoS, that frame_ids match, that pose-bearing events spatially-join to the right segments.
- Performance: latency from event-publish to memory-write, latency of `/riskgraph/score_routes` under realistic candidate counts, SQLite throughput on Jetson NVMe.
- That the canonical weights (geometry=1.0, semantic=1.0, risk=4.0) produce useful behaviour for a low-vision user. This requires user-study data, not just synthetic regression.
- Cross-run memory: that running the stack twice with the same SQLite file produces correct second-run scoring, where the first run's events bias the second.
- Soft-dependency robustness on Jetson: that adapters cleanly no-op when an upstream package is genuinely missing on a real Jetson installation.

The next concrete validation step is a CaresLab session executing the test plan in `docs/hardware_integration.md`. Until that runs, no claim of "running on Go2" should be made anywhere in this repo, in writing, or in conversation.
