#!/bin/bash
# ---------------------------------------------------------------------------
# start_course_demo.sh
#
# Bring up the V2 course demo (world urban_course: straight -> right 90 ->
# winding S -> left 90 -> STOP) in ONE terminator window laid out as a 3x2
# grid of 6 panes. Column 1 is the two things that must be started by hand -
# the venv VLM pipeline and the ROS launch; columns 2-3 are watchers.
#
#   col1 top     1  VLM pipeline (venv): vlm_sign_node -> wait "VLM ready"
#                   -> sign_maneuver_node
#   col1 bottom  2  ros2 launch robot_bringup av_stack.launch.py
#                   (gazebo + bridge + EKF + RViz + lane + planner + MPC),
#                   held until pane 1 signals Qwen is loaded
#   col2 top     3  /vlm/sign_maneuver narrative
#   col2 bottom  4  /maneuver setpoint + raw /vlm/sign
#   col3 top     5  topic-rate health (camera / depth / lane path / sign)
#   col3 bottom  6  free shell
#
# The av_stack launch file and the RViz config are assumed to already exist
# and be good - this script does not touch them.
#
# Usage:
#   ./scripts/start_course_demo.sh
#   STACK_NOWAIT=1 ./scripts/start_course_demo.sh     # don't wait for Qwen
#   DEMO_WORLD=urban_tight ./scripts/start_course_demo.sh
#   DEMO_RVIZ=false ./scripts/start_course_demo.sh
# Teardown: close the terminator window (each pane's job dies with it).
# ---------------------------------------------------------------------------
set -euo pipefail

WS="${VLM_AV_WS:-$HOME/vlm_av_ws}"
VENV="${VLM_VENV:-$HOME/venvs/vlm_robot}"
PANE="$WS/scripts/_course_demo_pane.sh"
RUNDIR="/tmp/vlm_course_demo.$$"

command -v terminator >/dev/null 2>&1 || {
    echo "terminator not found:  sudo apt update && sudo apt install -y terminator"; exit 1; }
[ -f "$PANE" ] || { echo "missing $PANE"; exit 1; }
chmod +x "$PANE"
[ -f "$WS/install/setup.bash" ] || echo "warn: $WS/install/setup.bash missing - run colcon build first"
[ -d "$VENV" ] || echo "warn: venv $VENV missing - pane 1 will fail"
WORLD="${DEMO_WORLD:-urban_course}"
[ -f "$WS/maps/$WORLD.json" ] || echo "warn: maps/$WORLD.json missing - regenerate the world first"

mkdir -p "$RUNDIR"
CFG="$RUNDIR/terminator_config"
trap 'rm -rf "$RUNDIR"' EXIT

pane_cmd() { printf 'bash %s %s' "$PANE" "$1"; }

cat > "$CFG" <<EOF
[global_config]
  suppress_multiple_term_dialog = True
  title_use_system_font = True
[profiles]
  [[default]]
    scrollback_lines = 20000
    scroll_on_output = False
    use_system_font = True
    exit_action = hold
[layouts]
  [[avdemo]]
    [[[window0]]]
      type = Window
      parent = ""
      size = 1920, 1040
      title = vlm course demo
    [[[h0]]]
      type = HPaned
      parent = window0
      order = 0
      ratio = 0.34
    [[[c1]]]
      type = VPaned
      parent = h0
      order = 0
      ratio = 0.5
    [[[t1]]]
      type = Terminal
      parent = c1
      order = 0
      profile = default
      command = $(pane_cmd 1)
    [[[t2]]]
      type = Terminal
      parent = c1
      order = 1
      profile = default
      command = $(pane_cmd 2)
    [[[h1]]]
      type = HPaned
      parent = h0
      order = 1
      ratio = 0.5
    [[[c2]]]
      type = VPaned
      parent = h1
      order = 0
      ratio = 0.5
    [[[t3]]]
      type = Terminal
      parent = c2
      order = 0
      profile = default
      command = $(pane_cmd 3)
    [[[t4]]]
      type = Terminal
      parent = c2
      order = 1
      profile = default
      command = $(pane_cmd 4)
    [[[c3]]]
      type = VPaned
      parent = h1
      order = 1
      ratio = 0.5
    [[[t5]]]
      type = Terminal
      parent = c3
      order = 0
      profile = default
      command = $(pane_cmd 5)
    [[[t6]]]
      type = Terminal
      parent = c3
      order = 1
      profile = default
      command = $(pane_cmd 6)
[keybindings]
EOF

export VLM_AV_WS="$WS" VLM_VENV="$VENV" DEMO_RUNDIR="$RUNDIR"
for v in DEMO_WORLD DEMO_RVIZ STACK_NOWAIT; do
    if [ -n "${!v:-}" ]; then export "${v?}"; fi
done

echo "terminator 3x2 course demo   rundir=$RUNDIR   world=$WORLD"
echo "  1 vlm pipeline | 2 av_stack | 3 sign_maneuver | 4 setpoint | 5 health | 6 free"
echo "  close the window to tear everything down."
exec terminator -g "$CFG" -l avdemo
