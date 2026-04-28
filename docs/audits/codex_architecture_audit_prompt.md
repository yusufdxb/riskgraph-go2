# Codex Audit Prompt — Architecture & Interfaces

**Run with:** `codex exec` (or paste into a Codex CLI / Codex agent context).

**Repo state:** post-architecture, pre-implementation deepening. Targets: `riskgraph_msgs/*`, `riskgraph_core/*`, `riskgraph_memory/conversions.py`, `docs/architecture.md`.

## Task

Review the architecture and the message/service interfaces of RiskGraph-Go2 as a senior robotics architect from a top-tier lab (NVIDIA, DeepMind, CMU RI tier). Be ruthless. Answer specifically; cite file paths and line numbers.

## Things to look for

1. **Weak novelty claims.** Read `docs/prior_art.md` and `README.md`. Does the novelty boundary hold up? Identify any sentence that overstates what is new vs. what already exists in Nav2 costmap layering, RatSLAM-style experience maps, ConceptGraphs, or DreamFLEX-style fault-aware control. If the answer is "this is just a costmap with persistence," say that.
2. **Unsafe robotics assumptions.**
   - Are frame conventions consistent? Does the memory node make any assumption that `RiskEvent.position` is in `map` even though the adapters set it to `(0,0,0)`?
   - Are QoS choices explicit and correct given the message classes (event-shaped vs. control-loop)?
   - Is anything in the SQLite write path positioned where a malformed message could crash the node instead of being logged and dropped?
3. **Broken ROS interfaces.**
   - Are all msg/srv files self-consistent (parallel arrays of equal length where promised, optional fields documented)?
   - Are there fields that look load-bearing but are decorative (e.g. `RouteSegment.length_m`)? Flag them.
   - Does any service contract risk silent partial-result behaviour (e.g. `QuerySegmentRisk` returning empty arrays on error)?
4. **Missing tests.** Where is the regression coverage thinnest? Specifically: spatial join under near-collinear segments; SQLite under concurrent write/read; explanation path when there are zero candidates; explanation path when ties exist.
5. **Unvalidated hardware claims.** Read `docs/validation.md` and `README.md`. Anywhere a hardware-dependent claim is asserted as verified? Anywhere "running on Go2" is implied?
6. **Overengineering / scope creep.** Anything in the MVP that would be removed in a tight 2-week thesis sprint? `riskgraph_explainer_node` may be on the bubble.
7. **Integration risks.** Look at `docs/hardware_integration.md` "Known limitations". Anything missed? Particular concerns: namespace collisions with upstream Go2 stack, parameter file overrides, and TF assumptions.

## Output format

For each finding, emit:

```
- severity: [HIGH|MED|LOW]
- file: <path>:<line range>
- finding: <one sentence>
- recommendation: <one sentence, concrete>
```

Group by severity. End with a 3-bullet "highest-leverage fixes" list.
