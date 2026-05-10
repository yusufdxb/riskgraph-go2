#!/usr/bin/env bash
# Entry point for hardware verification of RiskGraph-Go2 v0.1.0.
#
# This script does NOT bring up the integration launch — that must be done in
# a separate terminal so the operator controls when nodes start/stop (the
# cross-run phase requires a restart). See docs/HW_VERIFICATION.md for the
# operator workflow.
#
# What this script does:
#   1. Sources ROS 2 Humble and the workspace overlay.
#   2. Verifies the planner service is reachable.
#   3. Invokes the glossy_loop scenario, which records a bag, publishes
#      synthetic events, calls score_routes, and writes verdict.json.
#
# Usage:
#   ./tests/hw/run_scenario.sh                  # both phases (interactive restart)
#   ./tests/hw/run_scenario.sh --phase one      # live phase only
#   ./tests/hw/run_scenario.sh --phase two      # cross-run phase only

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

if [ -z "${ROS_DISTRO:-}" ]; then
    if [ -f /opt/ros/humble/setup.bash ]; then
        # shellcheck disable=SC1091
        source /opt/ros/humble/setup.bash
    else
        echo "[run_scenario] ABORT: ROS not sourced and /opt/ros/humble/setup.bash not found" >&2
        exit 2
    fi
fi

if [ -f "$REPO_ROOT/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$REPO_ROOT/install/setup.bash"
else
    echo "[run_scenario] ABORT: $REPO_ROOT/install/setup.bash missing. Run colcon build first." >&2
    exit 2
fi

# Pre-flight: planner service must already be advertised by a running launch.
if ! ros2 service list 2>/dev/null | grep -q "/riskgraph/score_routes"; then
    echo "[run_scenario] ABORT: /riskgraph/score_routes not advertised." >&2
    echo "[run_scenario]   Start the integration launch in another terminal first:" >&2
    echo "[run_scenario]     ros2 launch riskgraph_bringup integration.launch.py \\" >&2
    echo "[run_scenario]         enable_safety_adapter:=true \\" >&2
    echo "[run_scenario]         enable_helix_adapter:=true \\" >&2
    echo "[run_scenario]         enable_tactile_adapter:=true" >&2
    exit 2
fi

LOG_DIR="$REPO_ROOT/tests/hw/runs"
mkdir -p "$LOG_DIR"
TS="$(date +%Y%m%d_%H%M%S)"
CONSOLE_LOG="$LOG_DIR/console_${TS}.log"

echo "[run_scenario] Starting glossy_loop scenario; console log -> $CONSOLE_LOG"
python3 "$REPO_ROOT/tests/hw/scenario_glossy_loop.py" "$@" 2>&1 | tee "$CONSOLE_LOG"
RC="${PIPESTATUS[0]}"
echo "[run_scenario] scenario exit code: $RC"
exit "$RC"
