#!/usr/bin/env bash
# Live preflight: run immediately before a moving trial. Publishes nothing.
# Any FAIL prints NO-GO and exits 1. A GO verifies nothing about the robot.
#
#   ./scripts/preflight_live.sh --helix-session ~/helix_hw/<date>_motion [--expected-branch main]
#
# Default --mode live. All other flags: ros2 run riskgraph_nav riskgraph_preflight -h
set -uo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
if [ -z "${ROS_DISTRO:-}" ]; then source /opt/ros/humble/setup.bash; fi
if ! ros2 pkg prefix riskgraph_nav >/dev/null 2>&1; then
  set +u; source "$REPO/install/setup.bash"; set -u
fi
export RISKGRAPH_REPO="$REPO"
ARGS=("$@")
case " $* " in *" --mode "*) ;; *) ARGS=(--mode live "${ARGS[@]}") ;; esac
mkdir -p "$HOME/riskgraph_runs/preflight"
OUT="$HOME/riskgraph_runs/preflight/preflight_$(date +%Y%m%d_%H%M%S).json"
ros2 run riskgraph_nav riskgraph_preflight "${ARGS[@]}" --out "$OUT"
