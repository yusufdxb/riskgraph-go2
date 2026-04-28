# AGENTS.md — RiskGraph-Go2

Authoritative agent policy for this repo. Overrides parent `AGENTS.md` where in conflict.

## Scope

RiskGraph-Go2 is an experience-grounded risk memory + explainable route scoring overlay for the Unitree Go2. It does NOT plan paths from scratch and does NOT replace Nav2; it consumes route candidates and risk events, and emits scored, explained route choices.

## Architecture rules

- The colcon workspace root is this repo. Packages live under `src/`.
- `riskgraph_core` is **pure Python** — must remain importable without sourcing ROS. All ROS coupling lives in the `*_memory`, `*_planner`, `*_explainer`, `*_demo`, `*_bringup` packages.
- Custom interfaces only in `riskgraph_msgs` (ament_cmake). Do not redefine messages already provided by upstream `go2_msgs`, `helix_msgs`, `come_here_msgs`, `neuroskin_msgs`, `go2_semantic_msgs` — adapter nodes translate them into RiskGraph events.
- Adapter dependencies on upstream `*_msgs` packages must be **soft**: `try: import …` with a documented fallback, so colcon build succeeds even when those packages are not present.
- Persistence backend is SQLite (file or `:memory:`). No server-backed databases.

## Honesty boundary

When reporting validation, always separate:
1. **Verified offline** — what `pytest`, `colcon test`, the synthetic demo, or a unit run actually showed.
2. **Inferred runtime behavior** — what the code should do once wired into a live Go2 stack, but has not been observed.
3. **Hardware-dependent** — explicitly unverified until run on a Go2 + Jetson Orin NX with the upstream stack live.

Never collapse those categories.

## Editing norms

- Inspect before editing. Read related files in `src/` first.
- Don't fork upstream Go2 repos. Add adapter nodes here, point at upstream topic names.
- Preserve interface stability after `riskgraph_msgs` is published; bump versions for breaking changes.
- No 3D Gaussian splatting. No custom CUDA. Stay edge-deployable on Jetson Orin NX 16 GB.

## Validation defaults

- `colcon build --symlink-install` from repo root for ROS-side validation.
- `python -m pytest src/riskgraph_core/test src/riskgraph_memory/test src/riskgraph_planner/test src/riskgraph_explainer/test src/riskgraph_demo/test` for offline-friendly unit runs.
- `scripts/run_offline_demo.sh` for the deterministic synthetic demo.
- `scripts/run_audit.sh` (if present) to invoke a Codex review pass.
