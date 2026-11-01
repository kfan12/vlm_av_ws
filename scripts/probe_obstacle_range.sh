#!/bin/bash
# ---------------------------------------------------------------------------
# probe_obstacle_range.sh [S ...]
#
# Standstill detection range of the depth and the lidar obstacle extractors.
# Brings up the stack with the lidar on (depth AEB running, the car held at
# the spawn point because nothing publishes /maneuver/target_speed), runs
# obstacle_probe, and for each course arc length S (default 8 13 18 23 28 33
# 38 43 53) spawns a box in the ego lane, samples for 8 s, removes it.
# Then scores the probe log against the spawn records:
#   ./tools/avkit/obstacle_range_table.py <log> range_box_<S>...
# Logs and the table: log/obstacle_range/<stamp>/.
# ---------------------------------------------------------------------------
WS="${VLM_AV_WS:-$HOME/vlm_av_ws}"
DISTANCES=${*:-8 13 18 23 28 33 38 43 53}
OUT="$WS/log/obstacle_range/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
cd "$WS" || exit 1

STACK_PROCS='ign gazebo|gz sim|aeb_node|control_arbiter_node|lane_node|local_planner|mpc_tracker_v2|ekf_node|parameter_bridge|odom_compare|odom_noise|robot_state_publisher|obstacle_probe|ros2 launch'
cleanup() { pkill -f "$STACK_PROCS" 2>/dev/null; sleep 3; pkill -9 -f "$STACK_PROCS" 2>/dev/null; }
cleanup

# urban_course: the car never moves, and the boxes reach s=53, past the
# barrier that urban_course_roadblock has at s=35. Software GL: gpu is not
# used on the full course. No windows (gz_gui:=false): closing a Gazebo GUI
# would stop the server mid-sweep.
ros2 launch robot_bringup av_stack.launch.py world:=urban_course rviz:=false gz_gui:=false \
    gz_gl:=software lidar:=true > "$OUT/stack.log" 2>&1 &
sleep 30
ros2 run av_safety_cpp obstacle_probe --ros-args \
    --params-file src/av_safety_cpp/config/aeb_params.yaml -p log_path:="$OUT/probe.jsonl" \
    > "$OUT/probe.log" 2>&1 &
sleep 5
if [ ! -s "$OUT/probe.jsonl" ]; then
    echo "obstacle_probe wrote no samples - see $OUT/probe.log and $OUT/stack.log"
    cleanup; exit 1
fi

NAMES=()
for s in $DISTANCES; do
    name="range_box_$s"
    ./scripts/spawn_aeb_obstacle.sh --s "$s" --name "$name" || continue
    sleep 8
    ./scripts/remove_aeb_obstacle.sh "$name"
    sleep 2
    NAMES+=("$name")
done
cleanup
python3 tools/avkit/obstacle_range_table.py "$OUT/probe.jsonl" "${NAMES[@]}" | tee "$OUT/table.md"
