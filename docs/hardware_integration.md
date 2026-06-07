# Hardware Integration

This document describes how to wire RiskGraph-Go2 into a live Unitree Go2 + Jetson Orin NX 16 GB stack alongside the existing Go2 repos. **Nothing in this document has been verified on hardware as of this writing.** Steps are inferred from upstream code inspection and unit-test runs against synthetic publishers.

## Topic / message contracts consumed

| Source repo                        | Upstream topic            | Upstream message                      | RiskGraph adapter            |
|-----------------------------------|---------------------------|---------------------------------------|------------------------------|
| `GO2-seeing-eye-dog/go2_msgs`     | `/go2/safety_alert`       | `go2_msgs/SafetyAlert`                | `riskgraph_safety_adapter`   |
| `helix/helix_msgs`                | `/helix/faults`           | `helix_msgs/FaultEvent`               | `riskgraph_helix_adapter`    |
| `neuroskin/neuroskin_msgs` *(or upstream slip_state node)* | `/tactile/slip_state`     | `std_msgs/Bool`                       | `riskgraph_tactile_adapter`  |

Adapters are **soft-dependent**: each does a `try: import upstream_msgs; except ImportError: …` at top-level and exits cleanly if the upstream package is not installed. This means `colcon build` and `ros2 launch riskgraph_bringup integration.launch.py` succeed even when individual upstream stacks are missing, the affected adapter just becomes a no-op.

## Frame conventions

RiskGraph-Go2 expects:
- `map`: global frame; the frame `RiskEvent.header.frame_id` is in by default.
- `odom`, `base_link`, `camera_color_optical_frame`: used by upstream perception; not directly required by RiskGraph but assumed available for adapters that need to transform poses (none currently do).

If upstream nodes publish events with `frame_id != "map"`, the adapters do **not** currently TF-transform them; the planner spatial-join will be wrong. This is a known limitation; see "Known limitations" below.

## Wiring into a live stack

```bash
# 1. Build (alongside the upstream Go2 stack)
cd ~/Projects/personal/riskgraph-go2
source /opt/ros/humble/setup.bash
# If upstream Go2 packages are in a separate workspace, source it first:
source ~/workspace/GO2-seeing-eye-dog/install/setup.bash
source ~/workspace/helix/install/setup.bash
colcon build --symlink-install
source install/setup.bash

# 2. Launch RiskGraph alongside the upstream stack
ros2 launch riskgraph_bringup integration.launch.py \
    enable_safety_adapter:=true \
    enable_helix_adapter:=true \
    enable_tactile_adapter:=true
```

You should see:
- `riskgraph_memory_node` writing to SQLite as upstream events arrive.
- `riskgraph_planner_node` ready to answer `/riskgraph/score_routes` calls.
- `riskgraph_explainer_node` publishing on `/riskgraph/explanations` whenever scoring runs.

## Persistence on Jetson

The default `store_path` is `:memory:`. For cross-run memory, override it to a file on the Jetson's internal NVMe (NOT the SD card):

```yaml
# config/jetson.yaml
riskgraph_memory:
  ros__parameters:
    store_path: "/home/unitree/.local/share/riskgraph/memory.sqlite"
    decay_half_life_s: 7200.0     # 2 h, longer than session
riskgraph_planner:
  ros__parameters:
    store_path: "/home/unitree/.local/share/riskgraph/memory.sqlite"
    weight_geometry: 1.0
    weight_semantic: 1.5
    weight_risk: 4.0
    decay_half_life_s: 7200.0
```

Both nodes must point at the same file; the planner reads, the memory node writes. SQLite handles the concurrent-process case via WAL journaling; no extra config needed for our access pattern.

## Sourcing candidate routes from Nav2

The MVP planner does **not** generate routes; it scores candidates. To wire it into Nav2:

1. Run Nav2's planner to produce a path (`/plan` topic, `nav_msgs/Path`).
2. Discretise the path into segments, one per straight-line leg, with stable ids.
3. Build a `riskgraph_msgs/Route` message and call `/riskgraph/score_routes` with one or more candidate routes.

A small "nav2 bridge" node is the natural next deliverable; it is not in scope for the MVP. For the demo, candidate routes come from the synthetic publisher / test harness.

## Known limitations (hardware-relevant)

1. **No TF transforms in adapters.** Adapters forward upstream messages with the upstream frame_id intact. If `/go2/safety_alert.header.frame_id` is `base_link`, the planner spatial-join will produce wrong segment associations. Either (a) require upstream to publish in `map`, or (b) extend each adapter to wait for TF and transform. Test before relying on cross-frame events.
2. **Adapter pose source is empty.** Adapters set `RiskEvent.position` to `(0,0,0)` because they do not subscribe to `/odom` or query TF. Upstream events do not all carry pose. The memory node's spatial-join will fall back to whatever segments are registered, but the join is unreliable until either the adapters are pose-aware or the upstream events carry their own poses. *This is the highest-priority hardware-readiness gap.*
3. **No retry on SQLite contention.** The store opens with default SQLite settings. Under sustained concurrent writes from multiple adapters this should be fine (single writer, multiple readers via WAL), but it has not been load-tested.
4. **No graceful shutdown of the SQLite handle.** If a node is killed via SIGKILL the WAL may need cleanup on next open; SQLite handles this automatically but it is worth confirming on Jetson.
5. **`length_m` field on RouteSegment is decorative.** The core computes length from `start`/`end` so it is the source of truth; the message field is included for protocol legibility only.

## Hardware test plan (pending CaresLab session)

When a session is available:

1. Build on the Jetson against the upstream Go2 stack as above. Capture `colcon build` log and `ros2 interface list | grep riskgraph` output.
2. With the robot stationary, hand-trigger upstream events (e.g. publish a synthetic SafetyAlert via `ros2 topic pub`) and verify they land in the SQLite file (`sqlite3 memory.sqlite "SELECT count(*) FROM risk_event;"`).
3. Drive a loop course, then call the planner service with two candidate routes spanning the same start/end and confirm the safer one is picked. Capture the explanation.
4. Restart all nodes. Re-run the same planner call with the persisted SQLite file. Confirm the previous-session events still bias the score, validating the cross-run claim.

Each step's expected output should be archived alongside `docs/validation.md` as the session record.
