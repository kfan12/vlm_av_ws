# _av_stack_common.sh - sourced by the scenario runners (run_aeb_scenario.sh,
# run_acc_scenario.sh): stack lifecycle with a health gate, so every runner
# kills, launches and checks the stack the same way.
# Expects WS and LOG set and the ROS 2 + workspace setup sourced. Reads
# AEB_SENSOR (depth|lidar, default depth), STACK_ARGS (extra
# av_stack.launch.py args) and REQUIRED_NODES (pgrep -f patterns of extra
# nodes that must be alive, e.g. "av_features_cpp/acc_node").
# Not executable on its own.

STACK_PROCS='ign gazebo|gz sim|aeb_node|control_arbiter_node|lane_node|local_planner|mpc_tracker_v2|ekf_node|parameter_bridge|odom_compare|odom_noise|robot_state_publisher|trajectory_coordinator|constraint_from_maneuver|acc_node|lead_vehicle.py|ros2 topic pub|ros2 topic echo|ros2 launch|record_aeb_scenario'

# av_cleanup - stop every stack, sim and helper process (TERM, then KILL)
av_cleanup() {
    pkill -f "$STACK_PROCS" 2>/dev/null
    sleep 3
    pkill -9 -f "$STACK_PROCS" 2>/dev/null
}

sim_sec() {
    timeout 10 ros2 topic echo --once /clock rosgraph_msgs/msg/Clock --field clock 2>/dev/null \
        | awk '/^sec:/{s=$2} /^nanosec:/{n=$2} END{if (s != "") printf "%.3f", s + n * 1e-9}'
}

# The software-GL sim can freeze right after start (/clock stuck, no odom or
# depth) while every node keeps publishing on the frozen sim time - and then
# nothing ever looks stale. Only record once sim time advances and odometry
# and depth flow. Every echo names its message type, so it works even when
# the ros2 daemon is stale (it then lists no topics).
stack_healthy() {
    local c1 c2 n
    c1=$(sim_sec); sleep 4; c2=$(sim_sec)
    [ -n "$c1" ] && [ -n "$c2" ] && awk -v a="$c1" -v b="$c2" 'BEGIN{exit !(b - a > 1.0)}' || {
        echo "sim time not advancing ($c1 -> $c2)"; return 1; }
    timeout 10 ros2 topic echo --once /odom_truth nav_msgs/msg/Odometry --field header.stamp > /dev/null 2>&1 || {
        echo "no /odom_truth"; return 1; }
    timeout 10 ros2 topic echo --once /camera/front/depth/image_raw sensor_msgs/msg/Image --field header.stamp \
        > /dev/null 2>&1 || { echo "no depth image"; return 1; }
    if [ "${AEB_SENSOR:-depth}" = lidar ]; then
        timeout 10 ros2 topic echo --once /lidar/front/points sensor_msgs/msg/PointCloud2 --field header.stamp \
            > /dev/null 2>&1 || { echo "no lidar cloud"; return 1; }
    fi
    # checked last, after the waits above: a node that crashes on its first
    # full tick has had time to die by now
    pgrep -f 'av_safety_cpp/aeb_node' > /dev/null || { echo "aeb_node not running (crashed? see $LOG/stack.log)"; return 1; }
    for n in $REQUIRED_NODES; do
        pgrep -f "$n" > /dev/null || { echo "$n not running (crashed? see $LOG/stack.log)"; return 1; }
    done
}

# av_launch WORLD - start the stack headless in the background, wait for the
# arbiter's hand-off to the MPC and a healthy sim; up to 3 attempts. Returns
# non-zero if it never comes up.
av_launch() {
    local world=$1 attempt
    for attempt in 1 2 3; do
        # shellcheck disable=SC2086  # STACK_ARGS is a list of launch args
        ros2 launch robot_bringup av_stack.launch.py world:="$world" rviz:=false \
            aeb_sensor:="${AEB_SENSOR:-depth}" $STACK_ARGS > "$LOG/stack.log" 2>&1 &
        # -s: the first check can run before the backgrounded launch has created the file
        for _ in $(seq 1 60); do
            grep -qs "control source: .* -> mpc" "$LOG/stack.log" && break
            sleep 2
        done
        if ! grep -qs "control source: .* -> mpc" "$LOG/stack.log"; then
            # the arbiter holds fault_aeb_status until aeb_node's input flows
            echo "arbiter never handed over to mpc in 120 s (aeb_node input missing? see $LOG/stack.log)"
        elif stack_healthy; then
            echo "stack up: aeb_node $(pgrep -fc 'av_safety_cpp/aeb_node')  arbiter $(pgrep -fc 'av_control_cpp/control_arbiter_node')"
            return 0
        fi
        echo "stack attempt $attempt unhealthy - relaunching"
        cp "$LOG/stack.log" "$LOG/stack_failed_$attempt.log"
        av_cleanup
    done
    echo "stack never came up healthy"
    return 1
}

# av_check_recording REC_LOG - the INVALID RUN lines every runner prints,
# from the recorder's final row counts
av_check_recording() {
    grep -q "'odom_truth': 0," "$1" && echo "INVALID RUN: no /odom_truth recorded (sim froze mid-run?)"
    grep -q "'aeb_status': 0," "$1" && echo "INVALID RUN: no /safety/aeb_status recorded (aeb_node died? see $LOG/stack.log)"
    return 0
}
