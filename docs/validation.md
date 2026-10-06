# Validation Status

Single source of truth for what is proven, and how. Three categories:
**verified off-robot** (with the command that reproduces it), **prepared for
hardware** (procedure and tooling exist, never run on the robot), and
**hardware-dependent** (only the GO2 can answer).

Status as of v0.2.0: **software validated / hardware experiment prepared /
physical hardware behavior unverified.**

## Verified off-robot

### Unit tests (no ROS needed)

`./scripts/run_tests.sh` runs the pure-Python suites of `riskgraph_core`,
`riskgraph_memory`, `riskgraph_demo` and `riskgraph_nav`. Areas covered:

| Area | What is asserted |
|---|---|
| SQLite store v2 | absolute paths only; parent dirs created; v1 migration keeps rows; newer schema refused; map-id and evidence-class binding and mismatch refusal; legacy rows cannot be bound to a map; duplicate ids ignored (first write wins); non-finite rows rejected; malformed rows skipped; interrupted write leaves nothing; committed row survives SIGKILL; 4 writer threads + a second reader never see a partial event; reopen after restart; consistent backup |
| Risk field | empty field; monotone in severity; kernel decreasing; zero at and beyond the radius (unrelated routes untouched); aggregation and cap; NaN/Inf skipped; extreme severity clamped; other frames excluded; decay; deterministic rasterization; never >= 100 (randomized); path metrics |
| Geometry | quaternion/yaw round trip, rotation, transform composition/inverse, anchoring with rotation and translation, polyline helpers |
| Map identity / experiment | stable, path-independent id; changes with image, resolution, origin, anchor; the committed map equals the generator output; corridor geometry; route side; malformed-route detection; route comparison requires geometry change and lower risk; fallback checks |
| Memory ingestion | map events stored; odom events transformed (translation + 90 deg rotation); missing / invalid TF quarantined; skewed robot clock replaced by receipt time; unposed quarantined; live mode quarantines SYNTHETIC / REPLAY / SIMULATION / UNKNOWN; wrong map id quarantined; DB errors reported; restart gives the same grid |
| Adapters | receipt-clock pose freshness (robot stamp months skewed still posed); stale odometry unposed; uuid event ids; provenance |
| Localization | odometry rate / age / stale; frame and non-finite rejection; jump latch (distance and yaw); stationary detection; anchor puts the robot on the marker |
| Preflight | every blocking condition (P01..P38) produces NO-GO on synthetic facts; rehearsal relaxations |
| Report | all 14 criteria; wrong executed corridor fails; missing bag fails; `hardware_pass` impossible for rehearsal or without operator attestation |
| Static guards | only the sport sink creates a sport-request publisher and only the sink stage runner a Twist publisher; sport API ids only as constants in the sink core, 1001 nowhere; Nav2 velocity limits strictly inside the sport sink limits; no bt_navigator / recovery behaviors; risk layer non-lethal and not in the local costmap |

### ROS 2 integration tests (real processes, real Nav2, no robot)

`source install/setup.bash && python3 -m pytest tests/integration -v`
(8 tests; also run in CI in `ros:humble`). A fake odometry source publishes
with the lab robot's clock skew and an odom origin offset and rotated from
the start marker. Asserted:

* RiskGraph started before Nav2 waits for `/map` without failing;
* TF `map->base_link` resolves to the marker (0, 0) and is stamped on the local clock;
* all RiskGraph nodes use one absolute database path;
* an odom-frame event is stored at the correct map position;
* an event in a frame with no TF is quarantined `TF_UNAVAILABLE`;
* **one injected event moves Nav2's NavFn plan from the left corridor to the right**;
* SIGINT exits 0, publishes a blank risk grid, Nav2's costmap clears and the
  plan reverts; the restarted process restores the same incidents from disk
  and the plan moves back;
* with risk on both corridors the plan is valid, identical over 3 requests,
  non-lethal, and a goal inside a risk region is still plannable;
* a database bound to another map makes the memory node exit 2 with FATAL.

### Full rehearsal of the live procedure

`scripts/rehearse_live_trial.sh` runs the lab procedure with
`riskgraph_rehearsal_go2` in place of the robot: sink stages S0-S2, then the
real motion chain in software (Nav2 controller -> velocity_smoother ->
`/nav/cmd_vel` -> riskgraph_sport_sink armed -> `unitree_api` Request ->
rehearsal robot). Results of the rehearsal run
are recorded in the CHANGELOG entry for the version that ran them. Rehearsal
evidence is labelled REHEARSAL and can never set `hardware_pass`.

### Offline demo

`./scripts/run_offline_demo.sh` still chooses LONG with the cited evidence ids.

## Prepared for hardware (never run on the robot)

* The canonical experiment, Trials A-E, preflight, arming, abort monitors and
  evidence bundle: `docs/HW_VERIFICATION.md`.
* Live replay parity: `scripts/replay_trial_bag.sh <run_dir>`.

## Hardware-dependent (unverified)

* That `/utlidar/robot_odom` on the robot keeps localization valid over the
  course (drift, jumps, the robot being walked back by hand).
* That the mcf gait follows Nav2's commands (<= 0.20 m/s with gentle yaw)
  round the box; the GO2 has only been measured at 0.15 m/s straight.
* That the RiskGraph sport sink moves and stops this robot (sink stages S1/S2).
* That Nav2 is installed on the payload and runs there at the configured rates.
* Planner / controller latency and CPU on the Jetson under the full stack.
* Live adapter wire formats (`go2_msgs`, `helix_msgs`, slip flag): the
  canonical trial does not depend on them.

Until a live run passes `docs/HW_VERIFICATION.md` section 8 with
`hardware_pass: true`, no claim of "runs on GO2" is made in this repository.
