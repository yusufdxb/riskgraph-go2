#!/usr/bin/env bash
# Replay parity for a trial bundle: feed the recorded bag through the SAME
# RiskGraph code (run_mode:=replay, sim time) into a FRESH replay-class
# database, then compare events and risk grid with the trial's own DB.
#
#   ./scripts/replay_trial_bag.sh <run_dir>
#
# The replay database lands in <run_dir>/replay/ and is evidence class
# "replay": it can never be opened by a live node and never counts as
# hardware evidence. Run it off-robot (a workstation is fine).
set -uo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
RUN=${1:?usage: $0 <run_dir>}
RUN=$(cd "$RUN" && pwd)
[ -d "$RUN/bag" ] || { echo "no bag in $RUN"; exit 2; }
[ -f "$RUN/db/riskgraph_after.sqlite" ] || { echo "no db/riskgraph_after.sqlite in $RUN"; exit 2; }
if [ -z "${ROS_DISTRO:-}" ]; then source /opt/ros/humble/setup.bash; fi
set +u; source "$REPO/install/setup.bash"; set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-$((150 + RANDOM % 50))}
OUT="$RUN/replay"; mkdir -p "$OUT"
DB="$OUT/replay_$(date +%Y%m%d_%H%M%S).sqlite"
EXP=$(python3 -c "import json;print(json.load(open('$RUN/manifest.json'))['experiment'])")
setsid ros2 launch riskgraph_bringup riskgraph_live.launch.py run_mode:=replay use_sim_time:=true \
    experiment:="$EXP" store_path:="$DB" > "$OUT/riskgraph_replay.log" 2>&1 &
RG=$!
sleep 6
ros2 bag play "$RUN/bag" --clock --topics /tf /tf_static /map /riskgraph/risk_events \
    > "$OUT/bag_play.log" 2>&1
sleep 3
kill -INT -- "-$RG" 2>/dev/null; wait "$RG" 2>/dev/null
ros2 run riskgraph_nav riskgraph_replay_check --original "$RUN/db/riskgraph_after.sqlite" \
    --replayed "$DB" --experiment "$EXP" | tee "$OUT/replay_parity.txt"
