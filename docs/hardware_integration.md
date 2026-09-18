# Hardware Integration

Upstream adapter contracts and pose policy. **Nothing here has been verified on hardware.** The live procedure (launch order, Nav2, motion path, preflight, trials) is `docs/HW_VERIFICATION.md`; this file only covers the optional adapters and the pose contract.

## Topic / message contracts consumed

| Source repo                        | Upstream topic            | Upstream message                      | RiskGraph adapter            |
|-----------------------------------|---------------------------|---------------------------------------|------------------------------|
| `GO2-seeing-eye-dog/go2_msgs`     | `/go2/safety_alert`       | `go2_msgs/SafetyAlert`                | `riskgraph_safety_adapter`   |
| `helix/helix_msgs`                | `/helix/faults`           | `helix_msgs/FaultEvent`               | `riskgraph_helix_adapter`    |
| `neuroskin/neuroskin_msgs` *(or upstream slip_state node)* | `/tactile/slip_state`     | `std_msgs/Bool`                       | `riskgraph_tactile_adapter`  |

Adapters are **soft-dependent**: each does a `try: import upstream_msgs; except ImportError: …` at top-level and exits cleanly if the upstream package is not installed. This means `colcon build` and `ros2 launch riskgraph_bringup riskgraph_live.launch.py` succeed even when individual upstream stacks are missing, the affected adapter just becomes a no-op.

## Pose source

Upstream events say what happened, not where. Each adapter subscribes to the robot's odometry, keeps the latest sample in a bounded-age cache, and stamps every outgoing `RiskEvent` with that position.

| Parameter        | Default                | Meaning                                                              |
|------------------|------------------------|----------------------------------------------------------------------|
| `odom_topic`     | `/utlidar/robot_odom`  | `nav_msgs/Odometry` source. `""` declares "this deployment has no odometry". |
| `pose_max_age_s` | `0.5`                  | How stale a sample may be, in either direction, relative to the event. |

The topic contract was measured on the Go2 EDU on 2026-04-17: `/utlidar/robot_odom` at ~150 Hz, `header.frame_id = "odom"`, `child_frame_id = "base_link"`. The subscription is BEST_EFFORT / KEEP_LAST / depth 1, since only the newest sample matters.

When no sample is available within `pose_max_age_s` of the event, the adapter does **not** invent a position and does **not** drop the event. It publishes the event with `header.frame_id` set to the empty string, the "unposed" marker, and the memory node stores it without a segment. An unbound event is honest; a confidently mislocated one is not.

## Frame conventions

- `odom`: the GO2's odometry frame (origin = boot pose). Adapters stamp events in it.
- `base_link`: robot body. `odom->base_link` is republished on `/tf` by
  `riskgraph_localization`, restamped on the payload clock.
- `map`: defined by marker A (`riskgraph_localization` anchors `map->odom`).
  **All stored risk is in `map`.** The memory node transforms every event with
  a timestamped TF lookup and quarantines it if the lookup fails.
- `""` (empty): no pose. Quarantined, never placed on the map.

Pose freshness in the adapters is judged on the adapter's own receipt clock:
the robot's header stamps were measured months behind the payload clock.

Optional adapters are enabled on the RiskGraph launch:
`ros2 launch riskgraph_bringup riskgraph_live.launch.py run_mode:=live enable_tactile_adapter:=true`.
The canonical trial does not use them.

## Known limitations (hardware-relevant)

1. The adapters' upstream wire formats (`go2_msgs/SafetyAlert`, `helix_msgs/FaultEvent`, slip flag) have never been exercised live.
2. A slip / safety detector is not validated here; the canonical trial uses explicitly marked OPERATOR_INJECTED observations at real TF poses instead of provoking hazards.
3. Pose coverage is only as good as the odometry stream; unposed events are quarantined and counted (`quarantine_reasons.UNPOSED` in `/riskgraph/status`).
