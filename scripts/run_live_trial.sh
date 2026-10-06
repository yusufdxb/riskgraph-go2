#!/usr/bin/env bash
# The canonical live RiskGraph GO2 trial (Trials A-E), with evidence bundle.
# Pass the sink stages S0-S2 first, then start the RiskGraph sport sink
# (armed) and riskgraph_nav_live.launch.py (docs/HW_VERIFICATION.md section 5).
#
#   ./scripts/run_live_trial.sh --sink-session ~/riskgraph_sink/<date> [--db-tag trial]
#
# Nothing moves until you type the arming phrase for each route.
set -uo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
if [ -z "${ROS_DISTRO:-}" ]; then source /opt/ros/humble/setup.bash; fi
if ! ros2 pkg prefix riskgraph_nav >/dev/null 2>&1; then
  set +u; source "$REPO/install/setup.bash"; set -u
fi
export RISKGRAPH_REPO="$REPO"
ARGS=("$@")
case " $* " in *" --mode "*) ;; *) ARGS=(--mode live "${ARGS[@]}") ;; esac
exec ros2 run riskgraph_nav riskgraph_live_trial "${ARGS[@]}"
