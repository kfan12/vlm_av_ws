#!/bin/bash
# ---------------------------------------------------------------------------
# start_course_demo.sh
#
# Bring up the V2 course demo (world urban_course: straight -> right 90 ->
# winding S -> left 90 -> STOP) in ONE terminator window laid out as a 3x2
# grid of 6 panes. Column 1 is the two things that must be started by hand -
# the speed source and the ROS launch; columns 2-3 are watchers.
#
# Two setups, never both at once (MODE = first arg, or DEMO_MODE, default vlm):
#   vlm  VLM sign chain sets the speed cap; av_stack runs with aeb:=false
#        (no aeb_node, no control_arbiter - the MPC publishes /cmd_vel)
#   aeb  no VLM; a fixed /maneuver/target_speed (DEMO_SPEED, default 8.0 =
#        above the MPC cap, so the planner profile governs) and av_stack with
#        aeb:=true (aeb_node + control_arbiter own /cmd_vel). Default world
#        urban_course_roadblock, so there is something to brake for.
#
#   pane              vlm                              aeb
#   col1 top     1    vlm_sign_node -> "VLM ready"     target_speed publisher
#                     -> sign_maneuver_node (venv)
#   col1 bottom  2    av_stack aeb:=false, held        av_stack aeb:=true
#                     until Qwen is loaded
#   col2 top     3    /vlm/sign_maneuver narrative     /safety/aeb_status narrative
#   col2 bottom  4    /maneuver setpoint + /vlm/sign   arbiter source + /cmd_vel
#   col3 top     5    topic-rate health (camera / depth / lane path / sign or aeb)
#   col3 bottom  6    free shell
#
# The av_stack launch file and the RViz config are assumed to already exist
# and be good - this script does not touch them.
#
# Usage:
#   ./scripts/start_course_demo.sh [vlm|aeb]
#   STACK_NOWAIT=1 ./scripts/start_course_demo.sh     # vlm: don't wait for Qwen
#   ./scripts/start_course_demo.sh aeb                # roadblock world, AEB on
#   DEMO_WORLD=urban_course_roadblock_oncoming ./scripts/start_course_demo.sh aeb
#   DEMO_SPEED=2.0 ./scripts/start_course_demo.sh aeb  # cap the aeb-mode speed
#   DEMO_WORLD=urban_tight ./scripts/start_course_demo.sh
#   DEMO_RVIZ=false ./scripts/start_course_demo.sh
#   DEMO_CURVE_SPLICE=true ./scripts/start_course_demo.sh    # enable local_planner curve splice (off by default)
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
DEMO_MODE="${1:-${DEMO_MODE:-vlm}}"
case "$DEMO_MODE" in
    vlm) [ -d "$VENV" ] || echo "warn: venv $VENV missing - pane 1 will fail"
         DEMO_WORLD="${DEMO_WORLD:-urban_course}" ;;
    aeb) DEMO_WORLD="${DEMO_WORLD:-urban_course_roadblock}" ;;
    *)   echo "usage: start_course_demo.sh [vlm|aeb]   (got '$DEMO_MODE')"; exit 2 ;;
esac
WORLD="$DEMO_WORLD"
[ -f "$WS/maps/$WORLD.json" ] || echo "warn: maps/$WORLD.json missing - regenerate the world first"

# DDS should be pinned to loopback (WSL mirrored networking: default Cyclone binds the
# Wi-Fi eth1, and a reconnect mid-run kills every node's sockets and stops the car).
# Unset falls back to Cyclone defaults (works, but Wi-Fi-fragile); any other file is refused.
# All panes inherit this shell's environment, so checking here covers the whole stack.
DDS_CFG="$WS/src/robot_bringup/config/cyclonedds_localhost.xml"
if [ "${RMW_IMPLEMENTATION:-}" != "rmw_cyclonedds_cpp" ]; then
    echo "error: RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION:-}' (need rmw_cyclonedds_cpp) - set it in ~/.bashrc and open a new shell"; exit 1
fi
if [ -z "${CYCLONEDDS_URI:-}" ]; then
    echo "warn: CYCLONEDDS_URI unset - DDS on Cyclone defaults, a Wi-Fi reconnect will break the run"
elif [ "${CYCLONEDDS_URI}" != "file://$DDS_CFG" ]; then
    echo "error: CYCLONEDDS_URI='${CYCLONEDDS_URI}' - unset it or use file://$DDS_CFG"; exit 1
fi

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
      title = $DEMO_MODE course demo
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

export VLM_AV_WS="$WS" VLM_VENV="$VENV" DEMO_RUNDIR="$RUNDIR" DEMO_MODE DEMO_WORLD
for v in DEMO_RVIZ DEMO_CURVE_SPLICE DEMO_SPEED STACK_NOWAIT; do
    if [ -n "${!v:-}" ]; then export "${v?}"; fi
done

echo "terminator 3x2 course demo   mode=$DEMO_MODE   rundir=$RUNDIR   world=$WORLD"
if [ "$DEMO_MODE" = aeb ]; then
    echo "  1 target_speed | 2 av_stack aeb:=true | 3 aeb_status | 4 arbiter | 5 health | 6 free"
else
    echo "  1 vlm pipeline | 2 av_stack aeb:=false | 3 sign_maneuver | 4 setpoint | 5 health | 6 free"
fi
echo "  close the window to tear everything down."
exec terminator -g "$CFG" -l avdemo
