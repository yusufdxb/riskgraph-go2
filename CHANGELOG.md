# Changelog

All notable changes to RiskGraph-Go2 are tracked here.
This project does not yet follow strict semver: 0.x.y bumps are operational
milestones, not API contracts.

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
