# Codex Audit Prompt — Midpoint (post-core, post-planner)

**Run with:** `codex exec`.

**Repo state:** core risk model + ROS planner are implemented and tested offline; ROS end-to-end smoke test passes on workstation; hardware not yet exercised.

## Task

Audit the implementation against the architecture document. Verify no drift between `docs/architecture.md` and the actual code. Look for the kinds of bugs that pass unit tests but fail in production: edge cases, boundary conditions, off-by-one in spatial joins, persistence corner cases.

## Specific checks

1. **Scoring drift.** Re-derive `total_cost` from the formula in `docs/architecture.md` and check `riskgraph_core/scoring.py:score_routes` line-by-line. Any case where the implementation diverges from the doc?
2. **Decay correctness.** `RiskStore.segment_risk` and `score_routes` both compute decay. Are they consistent? Any double-decay (decaying once in the store and again in scoring)?
3. **Spatial join edge cases.** `riskgraph_core/segments.py:_point_segment_distance` — what happens when start ≈ end (degenerate segment)? When the point is collinear but past both endpoints? Verify the test coverage in `test_segments.py`.
4. **SQLite atomicity.** `RiskStore.record_event` does `DELETE` + `INSERT` in factor table without an explicit transaction. Is there a window where a reader sees an event with no factors? Check this against `events_for_segment` which filters out events with empty factor lists. Is the filter masking a real bug or a benign race?
5. **Explainer evidence selection.** `explain_choice` cites `evidence_for_segment(seg, max_events=3)`. Does `evidence_for_segment` use the same decay parameter as the scoring run? If they diverge, the explanation may cite events that scoring weighted to ~0.
6. **ROS-side validation.** Run `scripts/ros_end_to_end_check.py` and compare its output to `docs/validation.md`. Anything in validation.md that the script does not actually demonstrate?
7. **Adapter soft-dep paths.** Inspect `safety_adapter.py`, `helix_adapter.py`. Verify the soft-import pattern actually allows colcon build + bringup to succeed when go2_msgs / helix_msgs are not installed. The current code has the import at module top-level — does that defeat the soft-dep claim?

## Output format

Same as the architecture audit prompt. Group findings by severity. End with a "highest-leverage fixes" list of at most 3 items.
