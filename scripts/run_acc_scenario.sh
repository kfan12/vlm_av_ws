#!/bin/bash
# ---------------------------------------------------------------------------
# run_acc_scenario.sh ROW [SET_SPEED] [LABEL]
#
# One recorded ACC scenario in the lidar setup (lidar AEB + arbiter + ACC +
# trajectory_coordinator), headless, on urban_course, with a driven lead
# vehicle (tools/avkit/lead_vehicle.py) in the ego lane.
#   ROW  follow      lead at s=25, 1.5 m/s constant; ego set speed above it
#        lead_brake  lead at s=25, 2.0 m/s, then 2 m/s^2 to a stop at t=20 s
#        stop_go     lead at s=25, 2.0 -> stop (t=15-17) -> wait -> 2.0 (t=25-27)
#        curve       lead at s=40, 1.5 m/s through the right turn (s=60-84)
#        cut_in      lead appears at s=35, 1.0 m/s, once the ego passes x=14
#                    (about 12 m gap after the ~1 s spawn, ego already at speed)
#   (lead profile times are sim seconds since its driver started)
#   SET_SPEED  cruise target, /maneuver/target_speed (default 3.0, the MPC cap)
#   LABEL      session label (default acc_ROW_SET_SPEEDmps)
# Writes data/aeb_scenarios/<stamp>_<LABEL>/ with the AEB report and
# acc_metrics.json (+ acc.png). Logs: log/acc_scenario/<LABEL>/.
# Env: DUR (recording seconds, wall; default per row).
# Kills any running sim/stack first and at the end.
# ---------------------------------------------------------------------------
ROW=${1:?usage: run_acc_scenario.sh ROW [SET_SPEED] [LABEL]}
V=${2:-3.0}
LABEL=${3:-acc_${ROW}_${V}mps}
WS="${VLM_AV_WS:-$HOME/vlm_av_ws}"
LOG="$WS/log/acc_scenario/$LABEL"
mkdir -p "$LOG"
export PYTHONNOUSERSITE=1 AEB_SENSOR=lidar STACK_ARGS="acc:=true"
export REQUIRED_NODES="av_features_cpp/acc_node trajectory_coordinator_node constraint_from_maneuver_node"
WORLD=urban_course
# no `set -u`: the ROS 2 setup scripts reference unbound vars
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
source "$WS/scripts/_av_stack_common.sh"
cd "$WS" || exit 1

case "$ROW" in
    follow)     LEAD_S=25; PROFILE="0:1.5";                              ROW_DUR=70 ;;
    lead_brake) LEAD_S=25; PROFILE="0:2.0,20:2.0,21:0.0";                ROW_DUR=50 ;;
    stop_go)    LEAD_S=25; PROFILE="0:2.0,15:2.0,17:0.0,25:0.0,27:2.0";  ROW_DUR=70 ;;
    curve)      LEAD_S=40; PROFILE="0:1.5";                              ROW_DUR=90 ;;
    cut_in)     LEAD_S=35; PROFILE="0:1.0";                              ROW_DUR=50 ;;
    *) echo "unknown ROW $ROW"; exit 2 ;;
esac
DUR=${DUR:-$ROW_DUR}

av_cleanup
av_launch "$WORLD" || exit 1

# the lead's own bridge: its velocity command in, its ground-truth odometry out
ros2 run ros_gz_bridge parameter_bridge \
    "/model/lead_vehicle/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist" \
    "/model/lead_vehicle/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry" \
    --ros-args -r /model/lead_vehicle/cmd_vel:=/lead/cmd_vel \
    -r /model/lead_vehicle/odometry:=/lead/odom_truth > "$LOG/lead_bridge.log" 2>&1 &

"$WS/scripts/record_aeb_scenario.sh" --label "$LABEL" --duration "$DUR" --world "$WORLD" \
    --target-speed "$V" > "$LOG/rec.log" 2>&1 &
REC=$!
sleep 3
kill -0 "$REC" 2>/dev/null || { echo "recorder died:"; cat "$LOG/rec.log"; av_cleanup; exit 1; }

start_lead() {
    # the driver first: it waits for the lead's odometry and starts the profile
    # on the first message, so the lead moves as soon as it exists; started
    # after the spawn, it left the lead standing for about 1.7 s
    python3 tools/avkit/lead_vehicle.py drive --profile "$PROFILE" --world "$WORLD" \
        --ros-args -p use_sim_time:=true > "$LOG/lead_driver.log" 2>&1 &
    python3 tools/avkit/lead_vehicle.py spawn --s "$LEAD_S" --world "$WORLD" || return 1
}

if [ "$ROW" = cut_in ]; then
    ros2 topic pub -r 10 /maneuver/target_speed std_msgs/msg/Float64 "{data: $V}" > /dev/null 2>&1 &
    echo "waiting for truth x >= 14"
    # unbuffered: piped, ros2's Python stdout is block-buffered and awk would
    # see the position seconds late
    PYTHONUNBUFFERED=1 timeout 90 ros2 topic echo /odom_truth nav_msgs/msg/Odometry \
        --field pose.pose.position.x 2>/dev/null | awk '$1+0 >= 14 {exit}'
    start_lead || { echo "lead spawn failed"; av_cleanup; exit 1; }
else
    start_lead || { echo "lead spawn failed"; av_cleanup; exit 1; }
    sleep 2
    ros2 topic pub -r 10 /maneuver/target_speed std_msgs/msg/Float64 "{data: $V}" > /dev/null 2>&1 &
fi

wait "$REC"
# a dead driver stops following the profile (the lead may even keep its
# last velocity command), so the run did not test what its row describes
pgrep -f 'lead_vehicle.py drive' > /dev/null || \
    echo "INVALID RUN: lead driver died before the recording ended (see $LOG/lead_driver.log)"
cat "$LOG/rec.log"
av_check_recording "$LOG/rec.log"
grep -q "'lead_truth': 0," "$LOG/rec.log" && echo "INVALID RUN: no lead odometry (lead bridge? see $LOG/lead_bridge.log)"
grep -q "'acc_state': 0," "$LOG/rec.log" && echo "INVALID RUN: no /feature/acc/state (acc_node died? see $LOG/stack.log)"
grep "control source" "$LOG/stack.log"
av_cleanup
"$WS/scripts/analyze_aeb_scenario.sh" latest --plots > "$LOG/aeb_report.md" 2>&1
python3 tools/avkit/analyze_acc_run.py latest --plots | tee "$LOG/acc_metrics.json"
