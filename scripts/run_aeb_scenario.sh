#!/bin/bash
# ---------------------------------------------------------------------------
# run_aeb_scenario.sh ROW SPEED [LABEL]
#
# One reproducible, recorded AEB scenario in the aeb setup (aeb_node +
# control_arbiter own /cmd_vel), headless, on urban_course (no generated
# road_block, so the spawned box is the only ground-truth obstacle).
#   ROW    inlane        box in the ego lane at s=45
#          outlane       box beside the lane, curb side (lateral -4.7)
#          outlane_left  box in the oncoming lane (lateral +1.75)
#          partial       box half in the lane (lateral -0.9)
#          curve         box mid right turn (s=74)
#          sudden        box at s=45 spawned only once the car is ~2.5 m away
#          mpc_timeout   no box; mpc_tracker_v2 is killed while driving
#          sensor_dropout  box at s=45; ~8 s after it appears the lidar bridge is
#                          killed mid-approach, before AEB sees the box
#                          (AEB_SENSOR=lidar only: the depth camera shares its
#                          bridge with /clock and /cmd_vel). The kill's wall
#                          time goes to $LOG/dropout.json for scoring.
#   SPEED  /maneuver/target_speed [m/s] (the MPC caps at 3.0)
#   LABEL  session label (default ROW_SPEEDmps) -> data/aeb_scenarios/<stamp>_<LABEL>/
# Rows that should brake remove the box 15 s before the recording ends, so
# the STOP_HOLD release and the hand-back to the MPC are recorded too.
# Env: DUR (recording seconds, wall; default per row), SUDDEN_X (truth x that
# triggers the sudden spawn; default from SPEED), AEB_SENSOR (depth|lidar,
# default depth; lidar sessions get a _lidar label suffix).
# STACK_ARGS (extra av_stack.launch.py args, see _av_stack_common.sh; refused
# if it sets aeb_sensor, which is AEB_SENSOR's job and decides the label).
# Logs: log/aeb_scenario/<LABEL>/. Kills any running sim/stack first and at the end.
# ---------------------------------------------------------------------------
ROW=${1:?usage: run_aeb_scenario.sh ROW SPEED [LABEL]}
V=${2:?usage: run_aeb_scenario.sh ROW SPEED [LABEL]}
AEB_SENSOR=${AEB_SENSOR:-depth}
case "$STACK_ARGS" in
    *aeb_sensor:=*) echo "STACK_ARGS must not set aeb_sensor (use AEB_SENSOR=depth|lidar): $STACK_ARGS"; exit 2 ;;
esac
LABEL=${3:-${ROW}_${V}mps$([ "$AEB_SENSOR" = lidar ] && echo _lidar)}

WS="${VLM_AV_WS:-$HOME/vlm_av_ws}"
LOG="$WS/log/aeb_scenario/$LABEL"
mkdir -p "$LOG"
export PYTHONNOUSERSITE=1
# no `set -u`: the ROS 2 setup scripts reference unbound vars
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
source "$WS/scripts/_av_stack_common.sh"
cd "$WS" || exit 1

S=45; LAT=""; BRAKES=1
case "$ROW" in
    inlane)       ;;
    outlane)      LAT="--lateral -4.7"; BRAKES=0 ;;
    outlane_left) LAT="--lateral 1.75"; BRAKES=0 ;;
    partial)      LAT="--lateral -0.9" ;;
    curve)        S=74 ;;
    sudden)       ;;
    mpc_timeout)  S="" ;;
    sensor_dropout)
        [ "$AEB_SENSOR" = lidar ] || { echo "sensor_dropout needs AEB_SENSOR=lidar"; exit 2; }
        DUR=${DUR:-50} ;;
    *) echo "unknown ROW $ROW"; exit 2 ;;
esac
# long enough to reach the box at SPEED (start at s=3, ~6 s to speed) plus a 25 s hold/release
if [ -z "$DUR" ]; then
    if [ -z "$S" ]; then DUR=40; else DUR=$(awk -v s="$S" -v v="$V" 'BEGIN{printf "%d", 35 + s / v}'); fi
fi

av_cleanup
av_launch urban_course || exit 1

OBST=""; [ -n "$S" ] && OBST="--obstacle aeb_test_box"
"$WS/scripts/record_aeb_scenario.sh" --label "$LABEL" --duration "$DUR" --world urban_course \
    --target-speed "$V" $OBST > "$LOG/rec.log" 2>&1 &
REC=$!
T0=$(date +%s)
sleep 3
kill -0 "$REC" 2>/dev/null || { echo "recorder died:"; cat "$LOG/rec.log"; av_cleanup; exit 1; }
ros2 topic pub -r 10 /maneuver/target_speed std_msgs/msg/Float64 "{data: $V}" > /dev/null 2>&1 &

case "$ROW" in
    mpc_timeout)
        sleep 15
        echo "killing mpc_tracker_v2"
        pkill -f 'av_behavior_cpp/mpc_tracker_v2' ;;
    sudden)
        # bumper (x + 2.25) about 2.5 m + one second of travel short of the box face (s - 0.3)
        X=${SUDDEN_X:-$(awk -v s="$S" -v v="$V" 'BEGIN{printf "%.2f", s - 0.3 - 2.25 - 2.5 - v}')}
        echo "waiting for truth x >= $X"
        # unbuffered: piped, ros2's Python stdout is block-buffered and awk would
        # see the position seconds late; --world skips the spawn's world lookup
        # (an extra `ign service -l`), so the box appears close to the trigger
        PYTHONUNBUFFERED=1 timeout 90 ros2 topic echo /odom_truth nav_msgs/msg/Odometry \
            --field pose.pose.position.x 2>/dev/null | awk -v x="$X" '$1+0 >= x {exit}'
        "$WS/scripts/spawn_aeb_obstacle.sh" --s "$S" $LAT --world urban_course ;;
    sensor_dropout)
        sleep 7
        "$WS/scripts/spawn_aeb_obstacle.sh" --s "$S" $LAT
        sleep 8
        # only the lidar's own bridge: /clock, /cmd_vel and the camera stay up
        T_KILL=$(date +%s.%N)
        pkill -f '__node:=gz_ros_bridge_lidar'
        echo "killed the lidar bridge at wall $T_KILL"
        printf '{"kill_wall": %.3f}\n' "$T_KILL" > "$LOG/dropout.json" ;;
    *)
        sleep 7
        "$WS/scripts/spawn_aeb_obstacle.sh" --s "$S" $LAT ;;
esac

if [ -n "$S" ]; then
    # remove 15 s before the end (braking rows) to record the release, else just before the end
    LEAD=15; [ "$BRAKES" = 0 ] && LEAD=3
    while kill -0 "$REC" 2>/dev/null && [ $(( $(date +%s) - T0 )) -lt $(( DUR - LEAD )) ]; do sleep 1; done
    "$WS/scripts/remove_aeb_obstacle.sh"
fi
wait "$REC"
cat "$LOG/rec.log"
av_check_recording "$LOG/rec.log"
grep "control source" "$LOG/stack.log"
av_cleanup
"$WS/scripts/analyze_aeb_scenario.sh" latest --plots > "$LOG/report.md" 2>&1
tail -n +1 "$LOG/report.md" | grep -E 'session|source_sequence|command_gate|false_positive_brakes|"contact"|\*\*contact|first_braking|source_events|min_gt_standoff|fault_reasons|final_truth'
