#!/bin/bash
# ---------------------------------------------------------------------------
# run_headless_lap.sh MODE LABEL [DURATION_S] [WORLD]
#
# One reproducible, recorded lap - the exact recipe behind docs/baselines/.
#   MODE  novlm  publish /maneuver/target_speed at 10 Hz (NOVLM_SPEED, default
#                8.0 = above the MPC's 3 m/s cap, so the planner profile
#                governs); without it the MPC holds at 0
#         vlm    run the real sign chain from the torch venv (VLM_VENV),
#                waiting for Qwen's "VLM ready" before starting the sim
#   LABEL       session label -> data/av_laps/<stamp>_<LABEL>/
#   DURATION_S  wall-clock recording time (default 300)
#   WORLD       default urban_course
# Env: NO_AEB=1 skips aeb_node (by default it runs in shadow mode, as Day 17
#      will run it), NOVLM_SPEED, VLM_VENV.
# Then: ./scripts/analyze_av_lap.sh latest --plots
# Logs: log/headless_lap/<LABEL>/ (stack, aeb, recorder, vlm).
# Kills any running sim/stack processes first and again at the end.
# ---------------------------------------------------------------------------
MODE=${1:?usage: run_headless_lap.sh novlm|vlm LABEL [DURATION_S] [WORLD]}
LABEL=${2:?usage: run_headless_lap.sh novlm|vlm LABEL [DURATION_S] [WORLD]}
DUR=${3:-300}
WORLD=${4:-urban_course}
WS="${VLM_AV_WS:-$HOME/vlm_av_ws}"
VENV="${VLM_VENV:-$HOME/venvs/vlm_robot}"
LOG="$WS/log/headless_lap/$LABEL"
mkdir -p "$LOG"
export PYTHONNOUSERSITE=1
# no `set -u`: the ROS 2 setup scripts reference unbound vars
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
cd "$WS" || exit 1

STACK_PROCS='ign gazebo|gz sim|aeb_node|lane_node|local_planner|mpc_tracker_v2|ekf_node|parameter_bridge|odom_compare|vlm_sign_node|sign_maneuver_node|robot_state_publisher|ros2 topic pub|ros2 launch'
cleanup() {
    pkill -f "$STACK_PROCS" 2>/dev/null
    sleep 3
    pkill -9 -f "$STACK_PROCS" 2>/dev/null
}
cleanup

if [ "$MODE" = vlm ]; then
    ( source "$VENV/bin/activate"
      stdbuf -oL -eL python3 -m vlm_planner_py.vlm_sign_node --ros-args \
          -p use_sim_time:=true -p vlm_query_rate_hz:=0.5 ) > "$LOG/vlm_sign.log" 2>&1 &
    for _ in $(seq 1 120); do
        grep -q "VLM ready" "$LOG/vlm_sign.log" && break
        sleep 3
    done
    grep -q "VLM ready" "$LOG/vlm_sign.log" || {
        echo "VLM never became ready:"; tail -20 "$LOG/vlm_sign.log"; cleanup; exit 1; }
    echo "VLM ready"
    ( source "$VENV/bin/activate"
      python3 -m vlm_planner_py.sign_maneuver_node --ros-args \
          --params-file src/vlm_planner_py/config/sign_maneuver_params.yaml \
          -p use_sim_time:=true ) > "$LOG/sign_maneuver.log" 2>&1 &
elif [ "$MODE" != novlm ]; then
    echo "MODE must be novlm or vlm"; exit 2
fi

ros2 launch robot_bringup av_stack.launch.py world:="$WORLD" rviz:=false > "$LOG/stack.log" 2>&1 &
sleep 14   # EKF up (urban_world starts it at 9 s)
if [ "${NO_AEB:-0}" != 1 ]; then
    ros2 run av_safety_cpp aeb_node --ros-args --params-file src/av_safety_cpp/config/aeb_params.yaml \
        -r nominal_cmd:=/cmd_vel -p use_sim_time:=true > "$LOG/aeb.log" 2>&1 &
fi
sleep 12   # lane_node / local_planner / mpc_tracker_v2 up (launched at 12 s)
echo "aeb_node instances: $(pgrep -fc 'av_safety_cpp/aeb_node')"

"$WS/scripts/record_av_lap.sh" --label "$LABEL" --duration "$DUR" --world "$WORLD" > "$LOG/rec.log" 2>&1 &
REC=$!
sleep 8
kill -0 "$REC" 2>/dev/null || { echo "recorder died:"; cat "$LOG/rec.log"; cleanup; exit 1; }
if [ "$MODE" = novlm ]; then
    ros2 topic pub -r 10 /maneuver/target_speed std_msgs/msg/Float64 \
        "{data: ${NOVLM_SPEED:-8.0}}" > /dev/null 2>&1 &
fi
wait "$REC"
cat "$LOG/rec.log"
echo "QP fallback switches: $(grep -c 'PROPORTIONAL fallback' "$LOG/stack.log")"
cleanup
