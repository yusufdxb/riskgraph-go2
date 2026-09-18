#!/usr/bin/env bash
# OFF-ROBOT rehearsal of the complete live trial (docs/HW_VERIFICATION.md).
#
# Runs the SAME programs as the lab, in the same order, with the REHEARSAL
# robot (riskgraph_rehearsal_go2) in place of the GO2: it speaks the real
# unitree_api sport interface and publishes odometry with the robot clock
# skew measured on the lab GO2. The full motion chain is real software:
#   Nav2 controller -> velocity_smoother -> /nav/cmd_vel -> helix_arbiter
#   -> /cmd_vel -> helix_go2_sport_sink (armed) -> /api/sport/request -> robot
# Every output is labelled REHEARSAL and can never count as hardware evidence.
#
# Usage: scripts/rehearse_live_trial.sh <helix_session_dir> [evidence_root]
#   helix_session_dir: a HELIX HW_MOTION_TEST session with stage D/E PASS
#   (for a rehearsal, one produced by HELIX's scripts/hw_rehearsal.sh).
# Needs: this repo, HELIX (HELIX_WS) and unitree_ros2 (UNITREE_WS) built.
set -uo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
HELIX_WS=${HELIX_WS:-$HOME/workspace/helix}
UNITREE_WS=${UNITREE_WS:-$HOME/workspace/unitree_ros2/cyclonedds_ws}
HELIX_SESSION=${1:?usage: $0 <helix_session_dir> [evidence_root]}
EVIDENCE_ROOT=${2:-$HOME/riskgraph_runs/rehearsal}
LOGS=$(mktemp -d /tmp/riskgraph_rehearsal_logs.XXXX)
TAG="r$(date +%Y%m%d%H%M%S)"   # fresh database per rehearsal
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-89}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=${CYCLONEDDS_URI:-file://$REPO/tests/integration/cyclonedds_loopback.xml}
unset ROS_LOCALHOST_ONLY
set +u
source /opt/ros/humble/setup.bash
source "$UNITREE_WS/install/setup.bash"
source "$HELIX_WS/install/setup.bash"
source "$REPO/install/setup.bash"
set -u
PIDS=()
start() { local n=$1; shift; setsid "$@" >"$LOGS/$n.log" 2>&1 & PIDS+=($!); }
cleanup() { for p in "${PIDS[@]}"; do kill -INT -- "-$p" 2>/dev/null; done; sleep 3;
            for p in "${PIDS[@]}"; do kill -KILL -- "-$p" 2>/dev/null; done; echo "logs: $LOGS"; }
trap cleanup EXIT
ros2 daemon stop >/dev/null 2>&1
start robot ros2 run riskgraph_nav riskgraph_rehearsal_go2
start helix ros2 launch helix_bringup helix_closedloop.launch.py auto_activate_recovery:=true recovery_enabled:=true
sleep 8
start sink ros2 run helix_arbiter helix_go2_sport_sink --ros-args -p mode:=armed
start nav ros2 launch riskgraph_bringup riskgraph_nav_live.launch.py
sleep 12
"$REPO/scripts/preflight_live.sh" --mode rehearsal --helix-session "$HELIX_SESSION" \
    --db-root /tmp/riskgraph_rehearsal_db --db-tag "$TAG" --evidence-root "$EVIDENCE_ROOT" \
    || echo "(standalone preflight NO-GO; the runner repeats it and will stop)"
ros2 run riskgraph_nav riskgraph_live_trial --mode rehearsal --auto-confirm --execute-d \
    --helix-session "$HELIX_SESSION" --db-root /tmp/riskgraph_rehearsal_db \
    --db-tag "$TAG" --evidence-root "$EVIDENCE_ROOT"
RC=$?
echo "runner exit code: $RC"
exit $RC
