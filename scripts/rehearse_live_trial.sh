#!/usr/bin/env bash
# OFF-ROBOT rehearsal of the complete live trial (docs/HW_VERIFICATION.md).
#
# Runs the SAME programs as the lab, in the same order, with the REHEARSAL
# robot (riskgraph_rehearsal_go2) in place of the GO2: it speaks the real
# unitree_api sport interface and publishes odometry with the robot clock
# skew measured on the lab GO2. The full motion chain is real software:
#   sink stages S0 (stop_only), S1 + S2 (armed), then
#   Nav2 controller -> velocity_smoother -> /nav/cmd_vel
#   -> riskgraph_sport_sink (armed) -> /api/sport/request -> robot
# Every output is labelled REHEARSAL and can never count as hardware evidence.
#
# Usage: scripts/rehearse_live_trial.sh [evidence_root] [sink_session_dir]
# Needs: this repo and unitree_ros2 (UNITREE_WS, for unitree_api) built.
set -uo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
UNITREE_WS=${UNITREE_WS:-$HOME/workspace/unitree_ros2/cyclonedds_ws}
EVIDENCE_ROOT=${1:-$HOME/riskgraph_runs/rehearsal}
LOGS=$(mktemp -d /tmp/riskgraph_rehearsal_logs.XXXX)
SINK_SESSION=${2:-$LOGS/sink_session}
TAG="r$(date +%Y%m%d%H%M%S)"   # fresh database per rehearsal
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-89}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=${CYCLONEDDS_URI:-file://$REPO/tests/integration/cyclonedds_loopback.xml}
unset ROS_LOCALHOST_ONLY
set +u
source /opt/ros/humble/setup.bash
source "$UNITREE_WS/install/setup.bash"
source "$REPO/install/setup.bash"
set -u
PIDS=()
start() { local n=$1; shift; setsid "$@" >"$LOGS/$n.log" 2>&1 & PIDS+=($!); }
cleanup() { for p in "${PIDS[@]}"; do kill -INT -- "-$p" 2>/dev/null; done; sleep 3;
            for p in "${PIDS[@]}"; do kill -KILL -- "-$p" 2>/dev/null; done; echo "logs: $LOGS"; }
trap cleanup EXIT
ros2 daemon stop >/dev/null 2>&1
start robot ros2 run riskgraph_nav riskgraph_rehearsal_go2
sleep 4
stage() {  # stage <S> <sink mode> <phrase>
  start "sink_$1" ros2 run riskgraph_nav riskgraph_sport_sink --ros-args -p mode:="$2"
  sleep 4
  ros2 run riskgraph_nav riskgraph_sink_stage --stage "$1" --session-dir "$SINK_SESSION" \
      --repo "$REPO" --rehearsal --confirm "$3" || { echo "STAGE $1 did not PASS; stopping."; exit 1; }
  local p=${PIDS[-1]}; kill -INT -- "-$p" 2>/dev/null; sleep 2; kill -KILL -- "-$p" 2>/dev/null
}
stage S0 stop_only "ROBOT STANDING STOP ONLY"
stage S1 armed "AREA CLEAR MOVE"
stage S2 armed "AREA CLEAR DEADMAN"
start sink ros2 run riskgraph_nav riskgraph_sport_sink --ros-args -p mode:=armed
start nav ros2 launch riskgraph_bringup riskgraph_nav_live.launch.py
sleep 12
"$REPO/scripts/preflight_live.sh" --mode rehearsal --sink-session "$SINK_SESSION" \
    --db-root /tmp/riskgraph_rehearsal_db --db-tag "$TAG" --evidence-root "$EVIDENCE_ROOT" \
    || echo "(standalone preflight NO-GO; the runner repeats it and will stop)"
ros2 run riskgraph_nav riskgraph_live_trial --mode rehearsal --auto-confirm --execute-d \
    --sink-session "$SINK_SESSION" --db-root /tmp/riskgraph_rehearsal_db \
    --db-tag "$TAG" --evidence-root "$EVIDENCE_ROOT"
RC=$?
echo "runner exit code: $RC"
exit $RC
