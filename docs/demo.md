# Demo Guide

## Bundled scenario: `glossy_hallway`

Two routes between the same start and goal:

| Route   | Segments              | Length | Risk history                                          |
|---------|-----------------------|-------:|-------------------------------------------------------|
| `SHORT` | `glossy`              | 4.0 m  | 2 slip events (severity 0.9, 0.8), 1 safety alert (0.6), 1 audio anomaly (0.4) |
| `LONG`  | `safe-a` + `safe-b`   | 7.4 m  | none                                                  |

Weights: geometry=1.0, semantic=0.0, risk=4.0, decay half-life=1800 s.

**Expected:** The planner picks `LONG`. The explanation cites slip events. With these weights, ignoring the risk history would pick `SHORT` (cost 4.0 vs 7.4). Adding risk pulls SHORT's cost above LONG.

## Run the offline demo (no ROS required)

```bash
./scripts/run_offline_demo.sh
```

Expected output:

```
=== RiskGraph-Go2 offline demo: glossy_hallway ===
loaded 4 risk events into in-memory store
scoring 2 candidate routes with weights=ScoringWeights(...)

--- per-route scores (lower = better) ---
  SHORT: total= 14.523  geom= 4.00  sem= 0.00  risk=10.523  dom_segs=['glossy'] dom_cats=['SLIP']
  LONG:  total=  7.405  geom= 7.40  sem= 0.00  risk= 0.000  dom_segs=[] dom_cats=[]

chosen: LONG  (expected: LONG)
explanation: Chose route LONG because the alternative passed through glossy where 3 prior slips have been recorded. Going with the safer path.
evidence event ids: ['ev_slip_1', 'ev_slip_2', 'ev_safe_1']

choice match:      PASS
explanation match: PASS
overall:           PASS
```

`demo_results.json` is written for inspection. `scripts/export_demo_results.py demo_results.json out.csv` renders a CSV summary suitable for paper tables.

## Run the live ROS pipeline (requires ROS 2 Humble)

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch riskgraph_bringup demo_offline.launch.py
```

This brings up:
- `riskgraph_memory_node` — subscribes `/riskgraph/risk_events`, writes to SQLite (default `:memory:`).
- `riskgraph_planner_node` — exposes `/riskgraph/score_routes` service.
- `riskgraph_explainer_node` — re-broadcasts explanations on `/riskgraph/explanations`.
- `riskgraph_synthetic_publisher` — replays the scenario fixture as `RiskEvent` messages.

The synthetic publisher exits after replaying all events. The other three nodes keep running; you can call the planner service from another terminal:

```bash
source install/setup.bash
ros2 service call /riskgraph/score_routes riskgraph_msgs/srv/ScoreRoutes \
    "{candidates: [...], semantic_objective: ''}"
```

For an automated end-to-end check, `scripts/ros_end_to_end_check.py` spins memory + planner in-process, publishes synthetic events, calls the service, and asserts the chosen route is the safe one. This is the runtime-side counterpart to the offline pytest regression.

## Running with a custom scenario

```bash
./scripts/run_offline_demo.sh path/to/your_scenario.json out.json
```

Or for the ROS path:

```bash
ros2 launch riskgraph_bringup demo_offline.launch.py scenario:=/path/to/your_scenario.json
```

Scenario JSON schema (see `src/riskgraph_demo/fixtures/scenario_glossy_hallway.json` for a working example):

```json
{
  "name": "...",
  "description": "...",
  "segments": {
    "<seg_id>": {"start": [x, y, z], "end": [x, y, z], "label": "..."}
  },
  "routes": [
    {"route_id": "...", "segments": ["<seg_id>", "..."]}
  ],
  "events": [
    {"event_id": "...", "segment_id": "...", "category": "SLIP|SAFETY|DEPTH|AUDIO|FAULT|HUMAN|COLLISION|OTHER",
     "severity": 0.0..1.0, "source": "...", "age_s": 0.0}
  ],
  "weights": {"geometry": 1.0, "semantic": 1.0, "risk": 4.0, "decay_half_life_s": 1800.0},
  "expected_choice": "<route_id>",
  "expected_explanation_keywords": ["slip", "..."]
}
```

`age_s` is "seconds before now"; the loader stamps each event accordingly so decay tests are deterministic.
