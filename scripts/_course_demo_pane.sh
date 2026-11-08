#!/bin/bash
# ---------------------------------------------------------------------------
# _course_demo_pane.sh  N
#
# One pane of the terminator 3x2 course demo (see start_course_demo.sh).
# Sources ROS 2 + the workspace, then runs one piece of the course demo.
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
# DEMO_MODE=aeb panes (depth AEB + arbiter, no VLM):
#   1  fixed /maneuver/target_speed (DEMO_SPEED) at 10 Hz - stands in for the
#      sign chain; without it the MPC holds the car at 0.
#   2  AV stack: `av_stack.launch.py aeb:=true`, no wait - the above plus
#      aeb_node + control_arbiter (the only /cmd_vel publisher).
#   3  /safety/aeb_status, every state change + 1 Hz heartbeat.
#   4  /control/active_source + /cmd_vel + target_speed, same cadence.
#
# DEMO_MODE=lidar panes: the aeb panes, except that pane 2 launches with
# aeb_sensor:=lidar (aeb_node reads the front lidar, which only the lidar
# setups render) and pane 5 also watches /lidar/front/points.
#
# DEMO_MODE=acc panes: the lidar panes, plus ACC:
#   1  waits for the stack (trajectory_coordinator's binding topic), bridges,
#      spawns and drives the lead vehicle (tools/avkit/lead_vehicle.py; DEMO_LEAD=0
#      skips it), then publishes the cruise target DEMO_SPEED.
#   2  launches with aeb_sensor:=lidar acc:=true (acc_node + cruise_constraint +
#      trajectory_coordinator in front of the MPC).
#   3  ACC narrative: lead gap / lead speed / ACC cap, the binding constraint and
#      the AEB state.
#
# Every mode:
#   5  health: topic rates for what that setup depends on.
#   6  free shell with the handy commands printed.
#
# Env (forwarded by start_course_demo.sh):
#   VLM_AV_WS      workspace root         (default ~/vlm_av_ws)
#   VLM_VENV       torch venv             (default ~/venvs/vlm_robot)
#   DEMO_RUNDIR    per-run scratch dir    (set by the launcher)
#   DEMO_MODE      vlm|aeb|lidar|acc      (default vlm)
#   DEMO_WORLD     world / map name       (default urban_course)
#   DEMO_RVIZ      true|false             (default true)
#   DEMO_GZ_GL     gpu|software           (launcher default: aeb/lidar gpu, vlm/acc software; unset here: software)
#                  Gazebo OpenGL backend (gpu: WSLg d3d12, software: llvmpipe)
#   DEMO_CURVE_SPLICE true|false          (default false) local_planner curve splice
#   DEMO_SPEED     aeb/lidar/acc target speed (default 8.0, above the MPC cap)
#   DEMO_LEAD      acc: 1 = spawn + drive a lead vehicle, 0 = none (default 1)
#   DEMO_LEAD_S    acc: lead spawn arc length [m] (default 25)
#   DEMO_LEAD_PROFILE acc: lead speed profile "t0:v0,t1:v1,..." (sim s since
#                  its driver started : m/s; default 0:1.5)
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
LEAD="${DEMO_LEAD:-1}"
LEAD_S="${DEMO_LEAD_S:-25}"
LEAD_PROFILE="${DEMO_LEAD_PROFILE:-0:1.5}"
PARAMS="$WS/src/vlm_planner_py/config/sign_maneuver_params.yaml"
WATCH="$WS/scripts/_course_demo_watch.py"
VLM_LOG="$RUNDIR/vlm_sign.log"
VLM_READY="$RUNDIR/vlm_ready"
mkdir -p "$RUNDIR"

# What each mode runs: AEB + arbiter (every mode but vlm), AEB's obstacle
# sensor, ACC. SAFE groups the panes the three AEB modes share.
case "$MODE" in
    vlm)   AEB=false; AEB_SENSOR=depth; ACC=false ;;
    aeb)   AEB=true;  AEB_SENSOR=depth; ACC=false ;;
    lidar) AEB=true;  AEB_SENSOR=lidar; ACC=false ;;
    acc)   AEB=true;  AEB_SENSOR=lidar; ACC=true ;;
    *)     echo "unknown DEMO_MODE '$MODE' (vlm|aeb|lidar|acc)"; exec bash ;;
esac
FAMILY=vlm; [ "$AEB" = true ] && FAMILY=safe

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

  aeb:1|lidar:1)
      echo "[1] no VLM: /maneuver/target_speed = $SPEED m/s at 10 Hz"
      echo "    (stands in for sign_maneuver_node; Ctrl-C here and the MPC holds the car at 0)"
      ros2 topic pub -r 10 /maneuver/target_speed std_msgs/msg/Float64 "{data: $SPEED}" > /dev/null
      hold 1 ;;

  acc:1)
      # The lead's bridge and driver run in the background next to the speed
      # publisher; every way out of this pane stops all three (the lead model
      # itself goes with Gazebo, which pane 2 stops).
      stop_lead() {
          kill $(jobs -p) 2>/dev/null
          pkill -f 'lead_vehicle.py drive' 2>/dev/null
          pkill -f '/model/lead_vehicle/cmd_vel@' 2>/dev/null
      }
      trap 'stop_lead; exit 129' HUP
      trap 'stop_lead' INT TERM
      echo "[1] ACC demo: waiting for the stack (trajectory_coordinator's /planning/binding_constraint)..."
      until timeout 5 ros2 topic echo --once /planning/binding_constraint std_msgs/msg/String \
              > /dev/null 2>&1; do
          sleep 2
      done
      if [ "$LEAD" = 1 ]; then
          echo "[1] lead vehicle: bridge, spawn at s=$LEAD_S, drive profile $LEAD_PROFILE"
          ros2 run ros_gz_bridge parameter_bridge \
              "/model/lead_vehicle/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist" \
              "/model/lead_vehicle/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry" \
              --ros-args -r /model/lead_vehicle/cmd_vel:=/lead/cmd_vel \
              -r /model/lead_vehicle/odometry:=/lead/odom_truth > "$RUNDIR/lead_bridge.log" 2>&1 &
          if python3 "$WS/tools/avkit/lead_vehicle.py" spawn --s "$LEAD_S" --world "$WORLD"; then
              python3 "$WS/tools/avkit/lead_vehicle.py" drive --profile "$LEAD_PROFILE" --world "$WORLD" \
                  --ros-args -p use_sim_time:=true > "$RUNDIR/lead_driver.log" 2>&1 &
              echo "    driver log: $RUNDIR/lead_driver.log"
              sleep 2   # the lead gets going before the ego is released
          else
              echo "[1] lead spawn failed - cruising without a lead (ACC should stay on 'no lead')"
          fi
      else
          echo "[1] DEMO_LEAD=0: no lead vehicle (ACC should stay on 'no lead')"
      fi
      echo "[1] cruise target: /maneuver/target_speed = $SPEED m/s at 10 Hz"
      echo "    (Ctrl-C here and the coordinator holds the car at 0: the cruise constraint is required)"
      ros2 topic pub -r 10 /maneuver/target_speed std_msgs/msg/Float64 "{data: $SPEED}" > /dev/null &
      wait $!
      wait $! 2>/dev/null   # the first wait returns early when a trap fires
      trap - HUP INT TERM
      stop_lead
      hold 1 ;;

  vlm:2|aeb:2|lidar:2|acc:2)
      if [ "$ACC" = true ]; then
          echo "[2] AV stack: gazebo + bridge + EKF + RViz + lane + planner + ACC + coordinator + MPC + aeb_node (lidar) + arbiter"
      elif [ "$AEB" = true ]; then
          echo "[2] AV stack: gazebo + bridge + EKF + RViz + lane + planner + MPC + aeb_node ($AEB_SENSOR) + arbiter"
      else
          echo "[2] AV stack: gazebo + bridge + EKF + RViz + lane + planner + MPC (no AEB)"
      fi
      echo "[2] Gazebo rendering: ${DEMO_GZ_GL:-software}   (DEMO_GZ_GL=gpu|software)"
      # a leftover gz server / stale daemon from a previous run breaks the boot
      pkill -9 -f 'ign gazebo|gz sim' 2>/dev/null || true
      pkill -9 -f 'ros2 daemon|_ros2_daemon' 2>/dev/null || true
      sleep 2; ros2 daemon start >/dev/null 2>&1 || true
      if [ "$MODE" = vlm ] && [ "${STACK_NOWAIT:-0}" != "1" ]; then
          echo "[2] waiting for pane 1's 'VLM ready' marker  (STACK_NOWAIT=1 to skip)..."
          while [ ! -e "$VLM_READY" ]; do sleep 2; done
          echo "[2] Qwen up -> launching the stack"
      fi
      # Shutdown has to be driven from here. Closing the demo window hangs up
      # this pane's terminal; left alone, ros2 launch and the `ign gazebo`
      # wrapper die from SIGHUP with no shutdown, and with the GUI the wrapper
      # runs `ign gazebo server` / `gui` in process groups of their own, so
      # those two never get the hangup and keep running orphaned.
      # So the launch runs in the background (a trap can interrupt `wait`, not
      # a foreground child), and every way out - window closed, Ctrl-C, launch
      # ending on its own - stops the launch with SIGTERM (a background job
      # starts with SIGINT ignored; launch shuts its processes down cleanly on
      # TERM) and then any Gazebo process it left behind.
      stop_gazebo() {
          pkill -INT -f 'ign gazebo (server|gui)' 2>/dev/null || true
          sleep 3
          pkill -9 -f 'ign gazebo (server|gui)' 2>/dev/null || true
      }
      ros2 launch robot_bringup av_stack.launch.py \
          world:="$WORLD" rviz:="${DEMO_RVIZ:-true}" gz_gl:="${DEMO_GZ_GL:-software}" \
          curve_splice_enable:="$CURVE_SPLICE" aeb:="$AEB" aeb_sensor:="$AEB_SENSOR" acc:="$ACC" &
      LAUNCH=$!
      stop_stack() { kill -TERM "$LAUNCH" 2>/dev/null; wait "$LAUNCH" 2>/dev/null; stop_gazebo; }
      trap 'stop_stack; exit 129' HUP
      trap 'stop_stack' INT TERM
      wait "$LAUNCH"
      wait "$LAUNCH" 2>/dev/null   # the first wait returns early when a trap fires
      trap - HUP INT TERM
      stop_gazebo   # the launch also ends on its own (e.g. a Gazebo crash): never leave a server behind
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

  aeb:3|lidar:3)
      echo "[3] /safety/aeb_status  (>> = state change, plus a 1 Hz heartbeat)"
      python3 "$WATCH" aeb
      hold 3 ;;

  acc:3)
      echo "[3] ACC: lead gap / speed / cap, binding constraint, AEB state"
      echo "    (>> = lead found or lost, binding source or AEB state changed; plus a 1 Hz heartbeat)"
      python3 "$WATCH" acc
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

  aeb:4|lidar:4|acc:4)
      echo "[4] control_arbiter: who owns /cmd_vel  (>> = source change, plus a 1 Hz heartbeat)"
      python3 "$WATCH" ctrl
      hold 4 ;;

  vlm:5|aeb:5|lidar:5|acc:5)
      case "$MODE" in
          vlm)   topics="/camera/front/image_raw /camera/front/depth/image_raw /lane/path_odom /vlm/sign" ;;
          aeb)   topics="/camera/front/image_raw /camera/front/depth/image_raw /lane/path_odom /safety/aeb_status /cmd_vel" ;;
          lidar) topics="/camera/front/image_raw /camera/front/depth/image_raw /lidar/front/points /lane/path_odom /safety/aeb_status /cmd_vel" ;;
          acc)   topics="/camera/front/depth/image_raw /lidar/front/points /lane/path_odom /feature/acc/state /planning/authorized/speed_profile /lead/odom_truth /cmd_vel" ;;
      esac
      echo "[5] health - topic rates"
      echo "    (lane_node ticks on /camera/front/depth/image_raw - watch it stall)"
      sleep 10
      while true; do
          for t in $topics; do
              hz=$(timeout 6 ros2 topic hz "$t" 2>/dev/null | grep -m1 'average rate' || echo 'no data')
              printf '%-36s %s\n' "$t" "$hz"
          done
          echo '---'
      done ;;

  vlm:6|aeb:6|lidar:6|acc:6)
      echo "[6] free terminal.  Handy checks:"
      echo "     ros2 topic hz /camera/front/depth/image_raw     # lane_node's tick source"
      echo "     ros2 topic info /cmd_vel -v                      # exactly one publisher: $([ "$FAMILY" = safe ] && echo control_arbiter || echo mpc_tracker_v2)"
      if [ "$FAMILY" = safe ]; then
          echo "     ros2 topic echo /safety/aeb_status               # full AEB status"
          echo "     ros2 topic pub -r 10 /safety/manual_estop std_msgs/msg/Bool '{data: true}'  # arbiter e-stop while running"
          echo "     RViz shows the arbiter's 'ctrl: <source>' label above the car"
          if [ "$AEB_SENSOR" = lidar ]; then
              echo "     ros2 param get /aeb_node obstacle_source         # must say lidar"
              echo "     ros2 topic hz /lidar/front/points                # ~10 Hz; aeb_node FAULTs 'stale lidar' after 0.5 s without it"
          fi
          if [ "$ACC" = true ]; then
              echo "     ros2 topic echo /planning/binding_constraint     # which constraint limits the speed (cruise / acc / hold)"
              echo "     ros2 topic echo /feature/acc/constraint          # ACC's cap and its reason"
              echo "     ros2 topic info /planning/authorized/path -v     # the MPC is its only subscriber"
              echo "     python3 tools/avkit/lead_vehicle.py remove --world $WORLD   # take the lead away mid-run"
              echo "     pkill -f av_features_cpp/acc_node                # required-source fail-safe: binding turns to hold, the car stops"
          fi
      else
          echo "     ros2 topic echo --full-length /vlm/sign          # raw Qwen labels"
          echo "     ros2 param set /vlm_sign vlm_query_enabled false  # pause Qwen live"
          echo "     add an Image display on /maneuver/debug_image in RViz for the annotated FPV"
      fi
      echo "     ros2 launch robot_bringup av_stack.launch.py rviz:=false aeb:=$AEB aeb_sensor:=$AEB_SENSOR acc:=$ACC   # headless"
      exec bash ;;

  *)  echo "usage: DEMO_MODE=vlm|aeb|lidar|acc _course_demo_pane.sh <1..6>   (got '$MODE:${1:-}')"; exec bash ;;
esac
