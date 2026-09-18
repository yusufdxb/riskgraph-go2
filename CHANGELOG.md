# Changelog

All notable changes to RiskGraph-Go2 are tracked here.
This project does not yet follow strict semver: 0.x.y bumps are operational
milestones, not API contracts.

## [0.2.0] - 2026-09-18

Hardware experiment prepared. Physical behavior on the GO2 is still
UNVERIFIED: everything below was exercised off-robot (unit tests, real ROS 2
+ Nav2 processes, and a full rehearsal against a stand-in robot).

### Added
- **Nav2 integration.** `riskgraph_memory` publishes `/riskgraph/risk_costmap`
  (nav_msgs/OccupancyGrid, same geometry as `/map`, values 0..90, never
  lethal, transient local). Nav2's global costmap reads it through a second
  StaticLayer (`config/nav2_live.yaml`), so remembered risk is a planning cost
  inside Nav2 itself. Verified off-robot: one injected event moves the NavFn
  plan from the left corridor to the right one.
- `riskgraph_core.risk_field`: compact-support spatial risk over stored
  map-frame events, rasterization, path exposure metrics.
- `riskgraph_core.map_identity`: map id = hash of map image + metadata +
  start-marker anchor. The database is bound to it.
- `riskgraph_core.experiment`: the canonical two-corridor course (one file
  drives the map, anchor, goal, tolerances and pass logic) and a map generator.
- `riskgraph_nav` package:
  - `riskgraph_localization`: `odom->base_link` TF restamped on the payload
    clock, `map->odom` anchored on marker A, jump latch, status topic.
  - `riskgraph_preflight` / `scripts/preflight_live.sh`: 38 read-only GO/NO-GO
    checks (environment, data flow, TF, Nav2 lifecycle, single database,
    motion-authority topology, HELIX stage D/E evidence).
  - `riskgraph_live_trial` / `scripts/run_live_trial.sh`: Trials A-E with typed
    arming, abort monitors, goal cancellation and an evidence bundle.
  - `riskgraph_status`, `riskgraph_replay_check` / `scripts/replay_trial_bag.sh`,
    `riskgraph_generate_course_map`, and `riskgraph_rehearsal_go2` +
    `scripts/rehearse_live_trial.sh` for off-robot rehearsal.
- `RiskEvent.msg`: `provenance`, `source_frame_id`, `map_id`.
- ROS integration tests (`tests/integration`, real processes and Nav2) and a
  CI job running them in `ros:humble`.

### Changed
- **SQLite store v2**: `PRAGMA user_version` with in-place v1 migration,
  absolute paths only, map-id and evidence-class binding (a replay or
  rehearsal database cannot be opened as live), first-write-wins duplicates
  (was `INSERT OR REPLACE`), quarantine table, `synchronous=FULL`, read-only
  mode for readers, online backup.
- The memory node TF-transforms every event into `map` at its timestamp and
  quarantines what it cannot place (unposed, no TF, non-finite, wrong map,
  non-live provenance in live mode). Unposed events are no longer stored as
  unbound risk rows.
- One database path for all nodes, computed once by the launch file:
  `~/.local/share/riskgraph/<map_id>/<db_tag>.sqlite`.
- Adapters assign event ids and provenance.

### Fixed
- Adapters judged odometry freshness with the robot's header stamps, which
  were measured months behind the payload clock: every live event would have
  been stored unposed. Freshness now uses the receipt clock.
- The memory node's TF listener shared its node with the main executor, so a
  lookup with a timeout could wait on callbacks it was blocking.
- Planner, explainer, adapters and the synthetic publisher exited non-zero on
  SIGINT.

### Removed
- `integration.launch.py` (replaced by `riskgraph_live.launch.py`) and the
  motors-off `tests/hw` harness (replaced by the live trial).

## [0.1.2] - 2026-09-02

### Added
- Pose-aware adapters. All three adapters (`safety`, `helix`, `tactile`) now
  subscribe to the robot's odometry and stamp each emitted `RiskEvent` with a
  real position instead of `(0, 0, 0)`.
  - New pure-Python module `riskgraph_memory.pose_source` with `OdometryCache`
    (bounded-age, single-slot, rejects blank frames / non-finite coordinates /
    unstamped samples without evicting a good one) and the `UNKNOWN_FRAME`
    marker.
  - New `riskgraph_memory.adapters.pose_tagging.PoseTaggingMixin` holding the
    ROS wiring: a BEST_EFFORT / depth-1 subscription to `odom_topic`
    (default `/utlidar/robot_odom`, the measured Go2 EDU contract), fallback to
    node-clock receive time for firmware that publishes an unset header stamp,
    throttled WARN on unposed events, and `posed_event_count` /
    `unposed_event_count`.
  - New adapter parameters `odom_topic` and `pose_max_age_s` (default 0.5 s),
    wired into `default.yaml` for all three adapters. Setting `odom_topic` to
    `""` declares a deployment with no odometry.
- Join-boundary guards and counters on `RiskMemoryNode`:
  `joined_event_count`, `unposed_event_count`, `frame_mismatch_event_count`.
- `--frame` argument on the glossy-loop hardware scenario so the scripted
  events, the routes, and the segment seed all agree on a frame.
- 46 new tests (111 → 157): the odometry cache's age bound and malformed-input
  rejection, per-adapter posed / unposed / stale behavior, pose-tagging
  configuration (empty topic, custom topic, bad max-age), and the memory
  node's two join refusals.

### Changed
- **The spatial join is now guarded, and this changes stored data.** An event
  is joined to a seeded segment only if it carries a pose (non-blank
  `frame_id`) *and* that frame equals the seed's `frame_id`. Refusals are
  counted and logged, and the event is still persisted, just unbound.
  Previously every unposed event was joined to whichever segment sat nearest
  the origin, which silently attributed hazards to the wrong place.
- `conversions.core_event_from_msg` no longer coerces a blank `frame_id` to
  `"map"`. Blank is the unposed marker and must survive to the join boundary.
- The sample seed `hw_glossy_loop.json` declares `frame_id: "odom"`. The Go2
  SDK publishes no `map` frame and no `/tf`, so `odom` is the only frame
  adapters can stamp; a `map` seed would refuse every live event.
- Adapters no longer copy an upstream alert's `frame_id` onto the event. That
  field names the detecting sensor, not a place; the event's frame comes from
  the pose source alone.

### Fixed
- `riskgraph_memory/setup.py` did not list the `adapters` subpackage, so a
  non-symlink `colcon build` installed three console scripts pointing at a
  module it never copied. Only the symlink-install dev workspace worked.

### Verified
- 157 offline tests green via `./scripts/run_tests.sh`.
- `colcon build --symlink-install` green for the four affected packages.
- Live ROS 2 graph (CycloneDDS, real memory node + tactile adapter +
  synthetic `/utlidar/robot_odom` publisher), three cases: odometry near
  segment A stores `(5.0, 0.1, "odom") → A`; odometry near segment B stores
  `(5.0, 4.9, "odom") → B` (the old code would have bound both to whichever
  segment was nearest the origin); no odometry stores
  `(0, 0, "") → unbound` with the expected WARN from both nodes.
- Still unverified: the same path against the robot's own odometry stream.

## [0.1.1] - 2026-05-12

### Added
- Segment-seeding (closes the phase-1 gap that blocked hardware integration).
  - New pure-Python module `riskgraph_core.seed` with:
    - `SegmentSeedResult` dataclass
    - `SegmentSeedError` for structurally invalid seed files
    - `parse_segment_seed(dict)` and `load_segment_seed(path)` (JSON + YAML)
    - `merge_segment_seeds(...)` for multi-file seeds (last-write-wins on id)
  - `RiskMemoryNode` now reads a `segment_seed_path` ROS parameter at startup
    and loads it into `_known_segments` so events arriving without a stamped
    `segment_id` get spatially-joined via `segment_for_point`. A broken or
    missing seed file is loud (`get_logger().error(...)`) but non-fatal.
  - `RiskMemoryNode.known_segments` and `.segment_seed` properties for inspection.
  - Sample seed at `src/riskgraph_bringup/config/segment_seeds/hw_glossy_loop.json`
    matches the geometry in the v0.1.0 hw harness scenario.
- 36 new unit tests across `riskgraph_core` (31) and `riskgraph_memory` (5)
  covering empty seed, single segment, multiple segments, overlapping
  segments, malformed input (10 flavors), YAML loading, file IO failures,
  merge-across-files, and the end-to-end memory-node spatial-join path.

### Changed
- `src/riskgraph_bringup/config/default.yaml` adds the new
  `segment_seed_path` parameter (default `""`, which disables seeding).
- `riskgraph_bringup/setup.py` now installs `config/segment_seeds/*.json`
  into the package share dir.
- All 7 packages bumped to 0.1.1.

### Compatibility
- Hardware harness `tests/hw/scenario_glossy_loop.py` (df6e51b) is API-
  compatible: it does not set `segment_seed_path`, so the memory node
  behaves exactly as before unless the launch explicitly seeds it. With
  seeding enabled (recommended for v0.1.1 lab sessions), phase 1 events
  published at `(2.0, 0.0)` now spatially-join to `hw_glossy` rather than
  being stored unbound.

## [0.1.0] - 2026-04-28

Initial MVP. See `docs/HW_VERIFICATION.md` for the operator runbook against
the glossy-loop hw scenario.
