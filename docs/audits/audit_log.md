# Audit Log

Running record of audit checkpoints, findings, and how each was resolved. Three checkpoints are scheduled; each fills in one section.

## Checkpoint 1 — Architecture & Interfaces

**Trigger:** post-`riskgraph_msgs` + `riskgraph_core` + architecture doc.
**Audit prompt:** `docs/audits/codex_architecture_audit_prompt.md`
**Auditor:** internal (also queued for `codex exec` review).

### Findings (internal pass)

| ID  | Severity | Where                                              | Finding                                                                                                                                                  | Resolution |
|-----|----------|----------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------|------------|
| A-1 | MED      | `riskgraph_msgs/msg/RouteSegment.msg`              | `length_m` field is present but the core computes length from start/end and treats start/end as source of truth, so the field is decorative.            | DOCUMENTED in `docs/architecture.md` and `docs/hardware_integration.md`. Kept on the wire for protocol legibility; not used by the planner. |
| A-2 | MED      | `riskgraph_memory/adapters/safety_adapter.py`      | Adapters set `RiskEvent.position` to `(0,0,0)` because they have no pose source. Memory node's spatial join then has nothing to chew on.                 | DOCUMENTED as the highest-priority hardware-readiness gap in `docs/hardware_integration.md`. Memory node falls back to registered segments + segment_id-from-stamp where possible. v0.2 will add TF-aware adapters. |
| A-3 | LOW      | `docs/prior_art.md`                                | Initial novelty paragraph claimed "first" cross-run risk memory on Go2; refined to "to our knowledge, not demonstrated end-to-end" to avoid overclaim.   | RESOLVED in commit prior to v0.1.0 freeze. |
| A-4 | LOW      | `riskgraph_core/scoring.py`                        | Semantic match is substring on label only — does not match the upstream `SemanticDetection.embedding` vector path.                                       | DOCUMENTED in `docs/architecture.md` "Scoring model" as MVP scope. v0.2 plug-in slot identified. |
| A-5 | MED      | `riskgraph_core/store.py:RiskStore.record_event`   | `DELETE` + `INSERT` in `risk_factor` is not wrapped in an explicit transaction. A concurrent reader could observe an event with no factors.               | MITIGATED by `events_for_segment` skipping events with empty factor lists. Real fix (BEGIN/COMMIT) tracked in v0.2. |

### Highest-leverage fixes (next pass)

1. Make adapters TF-aware so `RiskEvent.position` is populated; otherwise the spatial-join pillar is structurally weak.
2. Wrap `record_event` in an explicit SQLite transaction.
3. Replace substring semantic match with embedding-based match once upstream `SemanticDetection.embedding` is consumed.

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
