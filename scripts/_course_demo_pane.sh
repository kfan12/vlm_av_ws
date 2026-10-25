#!/bin/bash
# ---------------------------------------------------------------------------
# _course_demo_pane.sh  N
#
# One pane of the terminator 3x2 course demo (see start_course_demo.sh).
# Sources ROS 2 + the workspace, then runs one piece of the V2 course demo.
# Only vlm-mode pane 1 activates the torch venv (Qwen); every other pane is
# system python so `ros2` CLI works normally.
#
# DEMO_MODE=vlm panes (VLM sign chain, no AEB):
#   1  VLM pipeline (venv): vlm_sign_node loads Qwen, and once the "VLM ready"
#      log appears it drops a marker file and starts sign_maneuver_node.
#   2  AV stack: waits for pane 1's marker (unless STACK_NOWAIT=1), then
#      `ros2 launch robot_bringup av_stack.launch.py aeb:=false` - gazebo +
#      bridge + EKF + RViz + lane_node + local_planner + mpc_tracker_v2, the
#      MPC publishing /cmd_vel directly.
#   3  /vlm/sign_maneuver narrative (maneuver / speed_cap / stop_dist / latch).
#   4  /maneuver/state + /maneuver/target_speed + raw /vlm/sign, 1 Hz.
#
# DEMO_MODE=aeb panes (AEB + arbiter, no VLM):
#   1  fixed /maneuver/target_speed (DEMO_SPEED) at 10 Hz - stands in for the
#      sign chain; without it the MPC holds the car at 0.
#   2  AV stack: `av_stack.launch.py aeb:=true`, no wait - the above plus
#      aeb_node + control_arbiter (the only /cmd_vel publisher).
#   3  /safety/aeb_status, every state change + 1 Hz heartbeat.
#   4  /control/active_source + /cmd_vel + target_speed, same cadence.
#
# Both modes:
#   5  health: topic rates for the camera, depth (lane_node ticks on this),
#      the lane path, and the sign reads (vlm) or AEB status + /cmd_vel (aeb).
#   6  free shell with the handy commands printed.
#
# Env (forwarded by start_course_demo.sh):
#   VLM_AV_WS      workspace root         (default ~/vlm_av_ws)
#   VLM_VENV       torch venv             (default ~/venvs/vlm_robot)
#   DEMO_RUNDIR    per-run scratch dir    (set by the launcher)
#   DEMO_MODE      vlm|aeb                (default vlm)
#   DEMO_WORLD     world / map name       (default urban_course)
#   DEMO_RVIZ      true|false             (default true)
#   DEMO_CURVE_SPLICE true|false          (default false) local_planner curve splice
#   DEMO_SPEED     aeb-mode target speed  (default 8.0, above the MPC cap)
#   STACK_NOWAIT=1 vlm-mode pane 2 starts the stack without waiting for Qwen
# ---------------------------------------------------------------------------
# no `set -u`: the ROS 2 setup scripts reference unbound vars

WS="${VLM_AV_WS:-$HOME/vlm_av_ws}"
VENV="${VLM_VENV:-$HOME/venvs/vlm_robot}"
RUNDIR="${DEMO_RUNDIR:-/tmp/vlm_course_demo}"
MODE="${DEMO_MODE:-vlm}"
WORLD="${DEMO_WORLD:-urban_course}"
CURVE_SPLICE="${DEMO_CURVE_SPLICE:-false}"
SPEED="${DEMO_SPEED:-8.0}"
PARAMS="$WS/src/vlm_planner_py/config/sign_maneuver_params.yaml"
WATCH="$WS/scripts/_course_demo_watch.py"
VLM_LOG="$RUNDIR/vlm_sign.log"
VLM_READY="$RUNDIR/vlm_ready"
mkdir -p "$RUNDIR"

# keep the stray ~/.local numpy 2.x out of the system-python ROS nodes
export PYTHONNOUSERSITE=1

source /opt/ros/humble/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"

hold() {  # keep the pane open after the process exits so errors stay readable
    local rc=$?
    echo
    echo "── pane ${1:-?} exited (rc=$rc) ── Enter for a shell ──"
    read -r _ || true
    exec bash
}

watch_once() { timeout 5 ros2 topic echo --once "$1" 2>/dev/null | head -1; }

case "$MODE:${1:-}" in

  vlm:1)
      echo "[1] VLM pipeline (venv) - loading Qwen, then sign_maneuver_node"
      rm -f "$VLM_READY"
      # shellcheck disable=SC1091
      source "$VENV/bin/activate"
      stdbuf -oL -eL python3 -m vlm_planner_py.vlm_sign_node --ros-args \
          -p use_sim_time:=true -p vlm_query_rate_hz:=0.5 2>&1 | tee "$VLM_LOG" &
      vlm_pid=$!
      echo "[1] waiting for 'VLM ready' (cold load ~2-3 min)..."
      until grep -q "VLM ready" "$VLM_LOG" 2>/dev/null; do
          kill -0 "$vlm_pid" 2>/dev/null || { echo "[1] vlm_sign_node exited during load"; hold 1; }
          sleep 2
      done
      : > "$VLM_READY"                       # release pane 2
      echo "[1] VLM ready -> starting sign_maneuver_node"
      python3 -m vlm_planner_py.sign_maneuver_node --ros-args \
          --params-file "$PARAMS" -p use_sim_time:=true
      hold 1 ;;

  aeb:1)
      echo "[1] no VLM: /maneuver/target_speed = $SPEED m/s at 10 Hz"
      echo "    (stands in for sign_maneuver_node; Ctrl-C here and the MPC holds the car at 0)"
      ros2 topic pub -r 10 /maneuver/target_speed std_msgs/msg/Float64 "{data: $SPEED}" > /dev/null
      hold 1 ;;

  vlm:2|aeb:2)
      if [ "$MODE" = aeb ]; then
          AEB=true
          echo "[2] AV stack: gazebo + bridge + EKF + RViz + lane + planner + MPC + aeb_node + arbiter"
      else
          AEB=false
          echo "[2] AV stack: gazebo + bridge + EKF + RViz + lane + planner + MPC (no AEB)"
      fi
      # a leftover gz server / stale daemon from a previous run breaks the boot
      pkill -9 -f 'ign gazebo|gz sim' 2>/dev/null || true
      pkill -9 -f 'ros2 daemon|_ros2_daemon' 2>/dev/null || true
      sleep 2; ros2 daemon start >/dev/null 2>&1 || true
      if [ "$MODE" = vlm ] && [ "${STACK_NOWAIT:-0}" != "1" ]; then
          echo "[2] waiting for pane 1's 'VLM ready' marker  (STACK_NOWAIT=1 to skip)..."
          while [ ! -e "$VLM_READY" ]; do sleep 2; done
          echo "[2] Qwen up -> launching the stack"
      fi
      ros2 launch robot_bringup av_stack.launch.py \
          world:="$WORLD" rviz:="${DEMO_RVIZ:-true}" \
          curve_splice_enable:="$CURVE_SPLICE" aeb:="$AEB"
      hold 2 ;;

  vlm:3)
      echo "[3] /vlm/sign_maneuver  (1 Hz)  maneuver / speed_cap / stop_dist / sign latch"
      sleep 8
      while true; do
          timeout 5 ros2 topic echo --once --full-length /vlm/sign_maneuver 2>/dev/null \
              | sed -n '1,4p'
          echo '---'
          sleep 1
      done ;;

  aeb:3)
      echo "[3] /safety/aeb_status  (>> = state change, plus a 1 Hz heartbeat)"
      python3 "$WATCH" aeb
      hold 3 ;;

  vlm:4)
      echo "[4] /maneuver setpoint + raw /vlm/sign  (1 Hz)"
      sleep 8
      while true; do
          st=$(watch_once /maneuver/state);        st=${st#data: }
          sp=$(watch_once /maneuver/target_speed); sp=${sp#data: }
          sg=$(watch_once /vlm/sign);              sg=${sg#data: }
          printf 'state:%-10s target_speed:%-7s  vlm:%s\n' "${st:--}" "${sp:--}" "${sg:--}"
          sleep 1
      done ;;

  aeb:4)
      echo "[4] control_arbiter: who owns /cmd_vel  (>> = source change, plus a 1 Hz heartbeat)"
      python3 "$WATCH" ctrl
      hold 4 ;;

  vlm:5|aeb:5)
      if [ "$MODE" = aeb ]; then
          topics="/camera/front/image_raw /camera/front/depth/image_raw /lane/path_odom /safety/aeb_status /cmd_vel"
      else
          topics="/camera/front/image_raw /camera/front/depth/image_raw /lane/path_odom /vlm/sign"
      fi
      echo "[5] health - topic rates"
      echo "    (lane_node ticks on /camera/front/depth/image_raw - watch it stall)"
      sleep 10
      while true; do
          for t in $topics; do
              hz=$(timeout 6 ros2 topic hz "$t" 2>/dev/null | grep -m1 'average rate' || echo 'no data')
              printf '%-34s %s\n' "$t" "$hz"
          done
          echo '---'
      done ;;

  vlm:6|aeb:6)
      echo "[6] free terminal.  Handy checks:"
      echo "     ros2 topic hz /camera/front/depth/image_raw     # lane_node's tick source"
      echo "     ros2 topic info /cmd_vel -v                      # exactly one publisher: $([ "$MODE" = aeb ] && echo control_arbiter || echo mpc_tracker_v2)"
      if [ "$MODE" = aeb ]; then
          echo "     ros2 topic echo /safety/aeb_status               # full AEB status"
          echo "     ros2 topic pub -r 10 /safety/manual_estop std_msgs/msg/Bool '{data: true}'  # arbiter e-stop while running"
          echo "     RViz shows the arbiter's 'ctrl: <source>' label above the car"
      else
          echo "     ros2 topic echo --full-length /vlm/sign          # raw Qwen labels"
          echo "     ros2 param set /vlm_sign vlm_query_enabled false  # pause Qwen live"
          echo "     add an Image display on /maneuver/debug_image in RViz for the annotated FPV"
      fi
      echo "     ros2 launch robot_bringup av_stack.launch.py rviz:=false aeb:=$([ "$MODE" = aeb ] && echo true || echo false)   # headless"
      exec bash ;;

  *)  echo "usage: DEMO_MODE=vlm|aeb _course_demo_pane.sh <1..6>   (got '$MODE:${1:-}')"; exec bash ;;
esac
