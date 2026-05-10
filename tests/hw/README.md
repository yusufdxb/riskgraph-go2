# tests/hw — Hardware verification harness for RiskGraph-Go2 v0.1.0

These are NOT unit tests. They run only on a Go2 + Jetson Orin NX session at
CaresLab, against a live ROS 2 Humble graph that includes the upstream Go2
stack (`go2_msgs`, `helix_msgs`, optional `neuroskin` slip publisher).

The offline pytest suite (`./scripts/run_tests.sh`) and the workstation ROS
smoke (`scripts/ros_end_to_end_check.py`) are sufficient to validate the pure
risk model and the in-process node wiring. They are NOT sufficient to validate:

- adapter wire-format / QoS handshake against live upstream messages,
- spatial join under TF-less adapter pose (currently `(0,0,0)`),
- cross-run SQLite memory on Jetson NVMe across a real session restart,
- end-to-end planner score response under realistic load.

The scripts in this directory are designed to run on the Jetson (or a
workstation that can reach the Go2's ROS_DOMAIN_ID), record a rosbag, and
emit machine-readable PASS/FAIL verdicts that can be archived alongside
`docs/validation.md` as the v0.1.0 hardware proof.

## What's here

- `scenario_glossy_loop.py` — scripted-path driver. Brings up the integration
  launch, registers known segments, drives a short scripted `/cmd_vel` loop
  past a "glossy" zone twice, hand-publishes synthetic safety alerts that
  simulate slip events at known map coordinates, then calls `/riskgraph/score_routes`
  with two candidate routes (the loop arm vs. an alternate arm) and asserts
  the planner picks the safer arm. The same SQLite file is then re-read after
  a node restart to assert cross-run memory.
- `lib/__init__.py`, `lib/scenario_runner.py` — shared helpers (rosbag record,
  service call wrappers, segment registration, verdict printer).
- `run_scenario.sh` — entry point. Sources ROS, sources the workspace, and
  invokes the scenario script with a timestamped output directory.

## Running

On the Jetson (after `colcon build` against an env that has `go2_msgs` and
`helix_msgs` installed):

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/GO2-seeing-eye-dog/install/setup.bash    # for go2_msgs
source ~/workspace/helix/install/setup.bash                  # for helix_msgs
source install/setup.bash                                    # this repo
./tests/hw/run_scenario.sh
```

Output is written to `tests/hw/runs/<timestamp>/` and includes:
- `bag/` — rosbag of the run (all `/riskgraph/*`, `/go2/safety_alert`,
  `/cmd_vel`, `/tf`).
- `verdict.json` — machine-readable PASS/FAIL with chosen_route_id, scores,
  and evidence event ids.
- `console.log` — full stdout/stderr.

## Honesty boundary

Per `AGENTS.md`, no claim of "verified on Go2" should be made until:
1. `verdict.json.pass == true`, AND
2. The bag is reviewable (no missing topics / dropped frames), AND
3. Cross-run assertion (Step 4 in `docs/HW_VERIFICATION.md`) is green
   against a real SQLite file on the Jetson NVMe path.
