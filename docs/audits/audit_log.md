# Audit Log

Running record of audit checkpoints, findings, and how each was resolved. Three checkpoints are scheduled; each fills in one section.

## Checkpoint 1 — Architecture & Interfaces

**Trigger:** post-`riskgraph_msgs` + `riskgraph_core` + architecture doc.
**Audit prompt:** `docs/audits/codex_architecture_audit_prompt.md`
**Auditor:** internal pass + Codex CLI (`codex exec`) ran 2026-04-28 against the post-implementation tree (HEAD `4046b04`).

### Findings (internal pass)

| ID  | Severity | Where                                              | Finding                                                                                                                                                  | Resolution |
|-----|----------|----------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------|------------|
| A-1 | MED      | `riskgraph_msgs/msg/RouteSegment.msg`              | `length_m` field is present but the core computes length from start/end and treats start/end as source of truth, so the field is decorative.            | DOCUMENTED in `docs/architecture.md` and `docs/hardware_integration.md`. Kept on the wire for protocol legibility; not used by the planner. |
| A-2 | MED      | `riskgraph_memory/adapters/safety_adapter.py`      | Adapters set `RiskEvent.position` to `(0,0,0)` because they have no pose source. Memory node's spatial join then has nothing to chew on.                 | DOCUMENTED as the highest-priority hardware-readiness gap in `docs/hardware_integration.md`. Memory node falls back to registered segments + segment_id-from-stamp where possible. v0.2 will add TF-aware adapters. |
| A-3 | LOW      | `docs/prior_art.md`                                | Initial novelty paragraph claimed "first" cross-run risk memory on Go2; refined to "to our knowledge, not demonstrated end-to-end" to avoid overclaim.   | RESOLVED in commit prior to v0.1.0 freeze. |
| A-4 | LOW      | `riskgraph_core/scoring.py`                        | Semantic match is substring on label only — does not match the upstream `SemanticDetection.embedding` vector path.                                       | DOCUMENTED in `docs/architecture.md` "Scoring model" as MVP scope. v0.2 plug-in slot identified. |
| A-5 | MED      | `riskgraph_core/store.py:RiskStore.record_event`   | `DELETE` + `INSERT` in `risk_factor` is not wrapped in an explicit transaction. A concurrent reader could observe an event with no factors.               | MITIGATED by `events_for_segment` skipping events with empty factor lists. Real fix (BEGIN/COMMIT) tracked in v0.2. |

### Codex findings (HEAD `4046b04`)

| ID    | Severity | Finding                                                                                                                                                       | Resolution |
|-------|----------|---------------------------------------------------------------------------------------------------------------------------------------------------------------|------------|
| C-H1  | HIGH     | ROS-ingested events cannot reliably become segment-keyed: no `segment_id` field in `RiskEvent.msg`, no segment registry topic, unjoined events stored unbound. | FIXED 2026-04-28: added `segment_id` field to `riskgraph_msgs/msg/RiskEvent.msg`; conversions plumb through; synthetic publisher populates it; verified on-disk via `sqlite3` after live launch (4 rows, all with `segment_id='glossy'`). |
| C-H2  | HIGH     | `default.yaml` started memory + planner + explainer with separate `:memory:` SQLite stores → planner could never see the memory node's writes.                | FIXED 2026-04-28: `store_path` in `default.yaml` now `/tmp/riskgraph_store.sqlite` shared by all three nodes. Verified by inspecting the file after `ros2 launch riskgraph_bringup demo_offline.launch.py`. |
| C-H3  | HIGH     | Adapters stamp every event as `map @ (0,0,0)` because they have no pose source. Once segment registration is wired up, this poisons the spatial join.         | PARTIALLY MITIGATED: with `segment_id` now in the IDL, adapters can be extended to set the segment id directly when they know it (e.g. tactile_adapter at the moment of slip can be paired with current segment). For real upstream events without that info, this remains a known v0.2 gap; documented in `docs/hardware_integration.md`. |
| C-H4  | HIGH     | `docs/hardware_integration.md` claimed SQLite WAL handles concurrent processes, but the code never enabled WAL or busy_timeout.                              | FIXED 2026-04-28: `RiskStore.__init__` now sets `PRAGMA journal_mode=WAL`, `busy_timeout=2000`, `synchronous=NORMAL`. Added `test_concurrent_reader_sees_atomic_event_writes` regression. |
| C-M1  | MED      | SQLite write failures could escape the subscription callback and kill ingestion.                                                                              | FIXED: `_on_event` wraps `record_event` in try/except, logs and continues. `record_event` itself uses a `BEGIN IMMEDIATE` transaction with rollback-on-error. |
| C-M2  | MED      | QoS hard-coded `RELIABLE`; may not match upstream best-effort sensor publishers.                                                                              | DEFERRED to v0.2. RELIABLE is the safer default for our event-shaped (low-rate) traffic; mismatch with a real best-effort publisher will manifest as "no events received" and is detectable. To make per-adapter QoS a parameter when a real upstream is wired. |
| C-M3  | MED      | "deployable Go2 stack" / "on Go2" wording overstated novelty.                                                                                                 | FIXED: README and `docs/prior_art.md` now read "Go2-targeted but currently hardware-unverified" with explicit "no claim of running on Go2" line. |
| C-M4  | MED      | `RouteSegment.length_m` is decorative.                                                                                                                        | DOCUMENTED in `docs/architecture.md` and `docs/hardware_integration.md`. Acceptable: keeping the field on the wire is harmless and protocol-clearer than a missing field. |
| C-M5  | MED      | Explainer node subscribed to `/riskgraph/route_scores` but planner never published it.                                                                        | FIXED: `PlannerNode` now publishes the score array on `/riskgraph/route_scores` after each service call so streaming consumers actually receive data. |
| C-L1  | LOW      | Tie resolution in `score_routes` is silent (input order); no tie-aware explanation.                                                                           | DOCUMENTED. `test_score_routes_resolves_ties_by_input_order` pins current behaviour; tie-aware explanation can wait for a real failure case. |
| C-L2  | LOW      | Test coverage missed degenerate spatial joins, concurrent SQLite, zero candidates, ties.                                                                      | FIXED: added `test_segment_for_point_handles_degenerate_zero_length_segment`, `test_segment_for_point_picks_correctly_with_near_collinear_segments`, `test_score_routes_handles_zero_candidates`, `test_score_routes_resolves_ties_by_input_order`, `test_concurrent_reader_sees_atomic_event_writes`. 38 tests total now. |

### Highest-leverage fixes (next pass)

1. Make adapters TF-aware so `RiskEvent.position` is populated *and/or* segment_id-pre-tagged from a paired pose subscription. Without it, real upstream events still don't map to a segment.
2. Replace substring semantic match with embedding-based match once upstream `SemanticDetection.embedding` is consumed.
3. Decide whether `evidence_for_segment` should default to scoring's decay or stay raw — consistency.

## Checkpoint 2 — Midpoint (post-core, post-planner)

**Trigger:** post-ROS-side end-to-end smoke test passing on workstation.
**Audit prompt:** `docs/audits/codex_midpoint_audit_prompt.md`
**Auditor:** internal.

### Findings (internal pass)

| ID  | Severity | Where                                            | Finding                                                                                                                                       | Resolution |
|-----|----------|--------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------|------------|
| M-1 | LOW      | `riskgraph_core/store.py:evidence_for_segment`   | `evidence_for_segment` accepts a `decay_half_life_s` parameter but the explainer call site passes the default (0.0), so evidence ranking is undecayed even when scoring uses decay. Evidence may cite an old event that scoring weighted near zero. | KNOWN, deferred. The explainer documents that it cites events by raw severity; with the canonical 1800 s half-life and an active session, the discrepancy is small. To revisit when decay defaults change. |
| M-2 | LOW      | `riskgraph_core/segments.py:_point_segment_distance` | Degenerate-segment branch (`ab_len_sq <= 1e-12`) returns distance to `a`; correct, but no test covers it.                                  | TEST ADDED is on the v0.2 backlog; behaviour is correct. |
| M-3 | LOW      | `scripts/ros_end_to_end_check.py`                | Reuses `:memory:` SQLite handles by closing the default node-internal store and opening a new one against `/tmp/riskgraph_e2e.sqlite`. This is a test-only hack; production launch uses parameter-based store_path. | NOTE in script header. Production path is unaffected. |
| M-4 | MED      | All adapters                                     | Adapters do `from upstream_msgs.msg import …` AT MODULE TOP-LEVEL inside the `try:` block, but later `from go2_msgs.msg import SafetyAlert as _SafetyAlert` in `__init__` re-imports without a guard. If upstream is missing, the import succeeds at module load (in the try) but the second import explodes at node init. | RESOLVED 2026-04-28: the second import is also guarded by the `HAVE_*` flag and the node early-returns from `main()` before constructing the adapter; module-load remains safe. |

### Highest-leverage fixes

1. (Done) Audit adapter import paths so soft-dep claim is genuinely honored at every entry point.
2. Decide whether `evidence_for_segment` should default to scoring's decay or stay raw — pick one and document.

## Checkpoint 3 — Final (pre-handoff)

**Trigger:** v0.1.0 tag candidate.
**Audit prompt:** `docs/audits/codex_final_audit_prompt.md`
**Auditor:** internal pass complete; external Codex review pending.

### Findings (internal pass)

| ID  | Severity | Where                       | Finding                                                                                                                                                    | Resolution |
|-----|----------|-----------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------|------------|
| F-1 | LOW      | `README.md`                 | "Validation status" makes the verified/inferred/hardware split explicit. No claim of "running on Go2" anywhere. README cross-references `docs/validation.md`. | OK. |
| F-2 | LOW      | `.gitignore`                | `*.sqlite`, `demo_results.json`, `demo_results.csv` are all ignored, so accidental check-in of run artefacts is prevented.                                 | OK. |
| F-3 | MED      | `docs/architecture.md`      | Diagram shows `/riskgraph/route_scores` flowing into the explainer, but the planner currently returns scores via service response only — the topic publication is the explainer's responsibility (it streams from inputs it receives). | DOCUMENTED clearly: planner publishes via service response; explainer node is OPTIONAL streaming convenience. README reflects this. |
| F-4 | LOW      | `docs/validation.md`        | "33 tests pass" is sourced directly from a recorded `pytest` run. Any future test count change should re-run the source command before updating the doc.   | OK with note. |

### Verdict

**Internal: SHIP** at v0.1.0 with the explicit hardware-proof-boundary notice. External Codex audit recommended before any reviewer-facing presentation.

### Outstanding for v0.2

1. TF-aware adapters (gap A-2).
2. Explicit SQLite transaction in `record_event` (gap A-5).
3. Embedding-based semantic match (gap A-4).
4. Decay-consistency between scoring and evidence selection (M-1).
5. Nav2 bridge node — convert `/plan` into candidate `Route` messages.
