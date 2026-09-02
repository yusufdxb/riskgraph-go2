# Hardware Integration

How to wire RiskGraph-Go2 into a live Unitree Go2 + Jetson Orin NX 16 GB stack alongside the existing Go2 repos. **Nothing here has been verified on hardware.** Steps are inferred from upstream code inspection and unit-test runs against synthetic publishers.

## Topic / message contracts consumed

| Source repo                        | Upstream topic            | Upstream message                      | RiskGraph adapter            |
|-----------------------------------|---------------------------|---------------------------------------|------------------------------|
| `GO2-seeing-eye-dog/go2_msgs`     | `/go2/safety_alert`       | `go2_msgs/SafetyAlert`                | `riskgraph_safety_adapter`   |
| `helix/helix_msgs`                | `/helix/faults`           | `helix_msgs/FaultEvent`               | `riskgraph_helix_adapter`    |
| `neuroskin/neuroskin_msgs` *(or upstream slip_state node)* | `/tactile/slip_state`     | `std_msgs/Bool`                       | `riskgraph_tactile_adapter`  |

Adapters are **soft-dependent**: each does a `try: import upstream_msgs; except ImportError: …` at top-level and exits cleanly if the upstream package is not installed. This means `colcon build` and `ros2 launch riskgraph_bringup integration.launch.py` succeed even when individual upstream stacks are missing, the affected adapter just becomes a no-op.

## Pose source

Upstream events say what happened, not where. Each adapter subscribes to the robot's odometry, keeps the latest sample in a bounded-age cache, and stamps every outgoing `RiskEvent` with that position.

| Parameter        | Default                | Meaning                                                              |
|------------------|------------------------|----------------------------------------------------------------------|
| `odom_topic`     | `/utlidar/robot_odom`  | `nav_msgs/Odometry` source. `""` declares "this deployment has no odometry". |
| `pose_max_age_s` | `0.5`                  | How stale a sample may be, in either direction, relative to the event. |

The topic contract was measured on the Go2 EDU on 2026-04-17: `/utlidar/robot_odom` at ~150 Hz, `header.frame_id = "odom"`, `child_frame_id = "base_link"`. The subscription is BEST_EFFORT / KEEP_LAST / depth 1, since only the newest sample matters.

When no sample is available within `pose_max_age_s` of the event, the adapter does **not** invent a position and does **not** drop the event. It publishes the event with `header.frame_id` set to the empty string, the "unposed" marker, and the memory node stores it without a segment. An unbound event is honest; a confidently mislocated one is not.

## Frame conventions

RiskGraph-Go2 expects:
- `odom`: the frame adapters stamp events in, because it is the only frame the Go2 SDK provides. Its origin is the robot's boot pose. The SDK publishes no `map` frame and no `/tf`.
- `base_link`, `camera_color_optical_frame`: used by upstream perception; not directly required by RiskGraph.
- `""` (empty): the unposed marker described above. Never spatially joined.

**The segment seed's `frame_id` must equal the frame the adapters stamp** (`odom` for a stock Go2). The memory node refuses to spatially join an event whose frame differs from the seed's, and counts the refusals in `frame_mismatch_event_count`. Joining across frames silently produces a risk map that is confidently wrong, which is worse than one with holes in it. Adapters still do not TF-transform, so a deployment that genuinely has a `map` frame needs either a `map`-framed odometry source on `odom_topic` or a TF-aware adapter; see "Known limitations".

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

1. **No TF transforms in adapters.** An event's frame is whatever frame its pose source publishes in; adapters never transform. Running against a stack that has a `map` frame means pointing `odom_topic` at a `map`-framed odometry source, or extending the adapters to wait for TF. Cross-frame events are not silently joined, they are counted and refused, so a mismatch shows up as `frame_mismatch_event_count` and a launch-time WARN rather than as bad data.
2. **Pose coverage is only as good as the odometry stream.** Events that arrive while odometry is stale or absent are stored unbound, and the planner cannot retrieve them by segment id. The adapter logs a WARN on the first such event and every 50th after it; `posed_event_count` / `unposed_event_count` give the exact split for a run. This is expected behavior, not a bug, but a run with a high unposed fraction means the odometry stream needs attention before the risk map is trustworthy.
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
