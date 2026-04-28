# Prior Art and Novelty Boundary

A scoped scan of adjacent literature to bound what RiskGraph-Go2 (persistent route-risk memory + explainable route biasing on Unitree Go2) can honestly claim as new. Not exhaustive; intended to set reviewer expectations.

## 1. Risk-aware path planning and hazard costmaps

**Representative work**
- Nav2 costmap layers + Smac/MPPI planners — pluginlib costmap layers (LIDAR, depth, sonar) feed cost-aware penalty functions in search; current production standard for risk-as-cost in ROS 2. ([Nav2 concepts](https://docs.nav2.org/concepts/index.html), [Smac planner](https://docs.ros.org/en/humble/p/nav2_smac_planner/))
- Bio-inspired risk-aware adaptive navigation for quadruped S&R — CNN-based selective attention fuses RGB-D into a per-frame risk map for traversability gating. ([ScienceDirect 2025](https://www.sciencedirect.com/science/article/pii/S2667379726000586))

**State of the art (1 sentence).** Hazard is encoded as instantaneous cost at the costmap-cell level and consumed by a planner; risk is recomputed each run from current sensor input, not accumulated across runs.

**Boundary for RiskGraph-Go2.** Cannot claim risk-aware planning itself. Can claim *persistence and provenance* of risk: cells/segments carry an event history with source modality (slip / safety-alert / depth / audio / near-miss) and survive across runs, not just within a single costmap snapshot.

## 2. Lifelong / experience-based navigation

**Representative work**
- RatSLAM experience maps — hippocampally-inspired graph that accretes experience across long deployments. ([RatSLAM IEEE](https://ieeexplore.ieee.org/document/1307183/))
- GP-based traversability for mapless navigation — sparse GP local map + RRT* for terrain-aware planning. ([arXiv 2403.19010](https://arxiv.org/abs/2403.19010))

**State of the art.** Experience graphs and GP traversability surfaces are mature; both store *geometric/visual* experience, not categorized failure events keyed to semantic route segments.

**Boundary.** Cannot claim experience maps or lifelong mapping. Can claim a *typed event log* (modality + severity + timestamp + pose) attached to topological route segments, queryable at plan time.

## 3. Failure-aware locomotion vs. failure-aware *route selection*

**Representative work**
- DreamFLEX — fault-aware locomotion controller that estimates joint-fault vector and modulates gait online. ([arXiv 2502.05817](https://arxiv.org/html/2502.05817v1))
- Fare: Failure Resilience in Learned Visual Navigation — OOD-aware policy + recovery, 300 m unsupervised run. ([arXiv 2510.24680](https://arxiv.org/html/2510.24680))

**State of the art.** Failure feedback loops close at the *controller* or *policy* level (gait adaptation, recovery, retraining curricula like the Ashfall failure-driven curriculum). They do not persist failure into a global route-graph that biases future high-level path selection.

**Boundary.** Cannot claim failure-aware locomotion or failure curricula. Can claim closing the loop from runtime failure events (slip, safety alert, audio anomaly) back into *map-level route preference* across sessions — an explicit gap in current work.

## 4. Semantic + risk overlays

**Representative work**
- ConceptGraphs — open-vocabulary 3D scene graphs from RGB-D, queryable by LLMs. ([concept-graphs.github.io](https://concept-graphs.github.io/))
- HOV-SG — hierarchical open-vocab scene graphs (floors/rooms/objects) for language-grounded nav. ([arXiv 2403.17846](https://arxiv.org/html/2403.17846v2))

**State of the art.** Open-vocab scene graphs encode *what is where*; risk/affordance is queried zero-shot from the LLM ("is this traversable?") rather than grounded in the robot's own historical incidents.

**Boundary.** Cannot claim semantic scene graphs or open-vocab nav. Can claim *grounding hazard semantics in the agent's own history* — the graph stores "this hallway segment caused IMU-measured slip on 2 of 5 traversals," not a zero-shot LLM guess.

## 5. Explainable navigation

**Representative work**
- Personalized causal explanations of robot behavior — cause/effect store + LLM refinement. ([PMC 12540097](https://pmc.ncbi.nlm.nih.gov/articles/PMC12540097/))
- LLM-integrated nav with CoT justification (EnvNet/RoutePlanner). ([Frontiers 2025](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2025.1627937/full))

**State of the art.** LLM-mediated justifications exist but rationalize *current* plans from *current* scene state; they do not cite a structured incident log as evidence.

**Boundary.** Cannot claim explainable robot behavior. Can claim *evidence-grounded* explanations: the explainer cites specific stored events ("avoided segment N7: 3 prior slip flags, 1 audio alarm") rather than free-form rationalization. Crucially, the MVP explainer is a deterministic template path, so the audit hook (`evidence_event_ids`) is preserved even before any LLM enters the pipeline.

## 6. Slip / terrain memory for legged robots

**Representative work**
- ProNav — proprioceptive traversability estimation for legged outdoor nav. ([arXiv 2307.09754](https://arxiv.org/html/2307.09754v4))
- Resilient Legged Local Navigation (ANYmal) — end-to-end traversal under compromised perception. ([arXiv 2310.03581](https://ar5iv.labs.arxiv.org/html/2310.03581))

**State of the art.** Proprio-derived traversability is well-studied per-episode and per-step; cross-run, segment-keyed slip ledgers shared with a planner are not standard.

**Boundary.** Cannot claim proprioceptive traversability. Can claim a per-segment, cross-run slip incidence record on Go2 with a documented contribution to the global cost.

---

## Novelty boundary

**Legitimate claim.** A persistent, multi-modal *route-risk memory* on Unitree Go2 that
1. ingests heterogeneous runtime events — slip flags, safety-mode triggers, depth-hazard hits, audio anomalies, near-collision counters — keyed to topological route segments;
2. biases segment selection across sessions, not just within a single costmap;
3. emits deterministic, evidence-grounded justifications citing specific stored events.

The contribution is the *integration*: typed cross-run incident log feeding both the planner and the explainer on a deployable quadruped stack. Each ingredient exists; the combination has not been demonstrated end-to-end on Go2 in the surveyed literature.

**Cannot claim.** Risk-aware planning, costmap layering, semantic / open-vocab navigation, scene graphs, experience maps, GP traversability, fault-aware locomotion, OOD recovery, LLM explanations of robot behavior, proprioceptive slip detection — all prior art.

**Strong related-work citations to anchor the prior-art section.**
1. Gu et al., *ConceptGraphs: Open-Vocabulary 3D Scene Graphs for Perception and Planning*, 2023. ([arXiv 2309.16650](https://arxiv.org/abs/2309.16650))
2. Cho et al., *DreamFLEX: Fault-Aware Quadrupedal Locomotion in Rough Terrains*, 2025. ([arXiv 2502.05817](https://arxiv.org/html/2502.05817v1))
3. Milford and Wyeth, *RatSLAM: A hippocampal model for SLAM*, IEEE. ([IEEE 1307183](https://ieeexplore.ieee.org/document/1307183/))

Supporting: Nav2 costmaps ([docs](https://docs.nav2.org/concepts/index.html)); GP traversability ([arXiv 2403.19010](https://arxiv.org/abs/2403.19010)); Fare failure-resilient visual nav ([arXiv 2510.24680](https://arxiv.org/html/2510.24680)); bio-inspired risk-aware quadruped nav ([ScienceDirect](https://www.sciencedirect.com/science/article/pii/S2667379726000586)).
