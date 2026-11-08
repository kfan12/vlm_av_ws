#!/bin/bash
# ---------------------------------------------------------------------------
# start_course_demo.sh
#
# Bring up the V2 course demo (world urban_course: straight -> right 90 ->
# winding S -> left 90 -> STOP) in ONE terminator window laid out as a 3x2
# grid of 6 panes. Column 1 is the two things that must be started by hand -
# the speed source and the ROS launch; columns 2-3 are watchers.
#
# Four demos, never more than one at once (MODE = first arg, or DEMO_MODE, default vlm):
#   MODE   speed source                 safety / speed chain              obstacle sensor  world (default)
#   vlm    VLM sign chain (venv)        aeb:=false - MPC owns /cmd_vel    -                urban_course
#   aeb    fixed target speed           aeb_node + control_arbiter        depth camera     urban_course_roadblock
#   lidar  fixed target speed           aeb_node + control_arbiter        front lidar      urban_course_roadblock
#   acc    fixed cruise target +        lidar setup + acc_node +          front lidar      urban_course
#          a driven lead vehicle        trajectory_coordinator
#   vlm    the VLM caps the speed from road signs; no AEB, no arbiter.
#   aeb    no VLM; a fixed /maneuver/target_speed (DEMO_SPEED, default 8.0 =
#          above the MPC cap, so the planner profile governs) and aeb_node on
#          the depth image; the roadblock barrier is something to brake for.
#   lidar  as aeb, but AEB reads the front lidar (av_stack aeb_sensor:=lidar);
#          the lidar exists in the robot model only in the lidar setups.
#   acc    the lidar setup plus adaptive cruise (av_stack aeb_sensor:=lidar
#          acc:=true): pane 1 spawns a lead vehicle in the ego lane and drives
#          it (DEMO_LEAD_S, DEMO_LEAD_PROFILE; DEMO_LEAD=0 for none), and
#          acc_node keeps a time gap behind it by capping the speed. Full
#          urban_course: on the roadblock track ACC would take the barrier for
#          a stopped lead and park in front of it.
#
#   pane              vlm                              aeb | lidar                  acc
#   col1 top     1    vlm_sign_node -> "VLM ready"     target_speed publisher       lead vehicle, then
#                     -> sign_maneuver_node (venv)                                  target_speed publisher
#   col1 bottom  2    av_stack aeb:=false, held        av_stack aeb:=true           av_stack aeb_sensor:=lidar
#                     until Qwen is loaded             (lidar: aeb_sensor:=lidar)   acc:=true
#   col2 top     3    /vlm/sign_maneuver narrative     /safety/aeb_status narrative ACC gap / cap / binding + AEB
#   col2 bottom  4    /maneuver setpoint + /vlm/sign   arbiter source + /cmd_vel    arbiter source + /cmd_vel
#   col3 top     5    topic-rate health for the setup
#   col3 bottom  6    free shell with that setup's handy commands
#
# The av_stack launch file and the RViz config are assumed to already exist
# and be good - this script does not touch them.
#
# Usage:
#   ./scripts/start_course_demo.sh [vlm|aeb|lidar|acc]      (-h prints this header)
#   STACK_NOWAIT=1 ./scripts/start_course_demo.sh     # vlm: don't wait for Qwen
#   ./scripts/start_course_demo.sh aeb                # roadblock world, AEB on
#   ./scripts/start_course_demo.sh lidar              # roadblock world, lidar AEB
#   ./scripts/start_course_demo.sh acc                # full course, lidar AEB + ACC behind a 1.5 m/s lead
#   DEMO_LEAD_PROFILE=0:2.0,20:2.0,21:0.0,30:0.0,32:2.0 ./scripts/start_course_demo.sh acc   # lead stops, goes
#   DEMO_LEAD_S=40 ./scripts/start_course_demo.sh acc # lead starts further ahead
#   DEMO_LEAD=0 ./scripts/start_course_demo.sh acc    # no lead: ACC must stay on 'no lead'
#   DEMO_WORLD=urban_course_roadblock_oncoming ./scripts/start_course_demo.sh aeb
#   DEMO_SPEED=2.0 ./scripts/start_course_demo.sh aeb  # cap the aeb-mode speed
#   DEMO_WORLD=urban_tight ./scripts/start_course_demo.sh
#   DEMO_RVIZ=false ./scripts/start_course_demo.sh
#   DEMO_GZ_GL=gpu|software ./scripts/start_course_demo.sh  # Gazebo sensor rendering: gpu (WSLg d3d12) or
#                                                         # software (llvmpipe); default: aeb/lidar gpu, vlm/acc software
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
    aeb|lidar) DEMO_WORLD="${DEMO_WORLD:-urban_course_roadblock}" ;;
    acc) DEMO_WORLD="${DEMO_WORLD:-urban_course}"
         case "$DEMO_WORLD" in
             *roadblock*) echo "warn: $DEMO_WORLD has a barrier in the lane - ACC will treat it as a stopped lead" ;;
         esac ;;
    -h|--help|help) sed -n '3,/^# ----/p' "$0" | sed -e '$d' -e 's/^# \{0,1\}//'; exit 0 ;;
    *)   echo "usage: start_course_demo.sh [vlm|aeb|lidar|acc]   (got '$DEMO_MODE'; -h for details)"; exit 2 ;;
esac
WORLD="$DEMO_WORLD"
[ -f "$WS/maps/$WORLD.json" ] || echo "warn: maps/$WORLD.json missing - regenerate the world first"

# DDS runs on Cyclone defaults (CYCLONEDDS_URI unset). Under WSL
# networkingMode=nat they bind the virtual eth0, which a host Wi-Fi reconnect
# does not touch. The loopback-only cyclonedds_localhost.xml freezes
# lane_node's camera subscriptions right after startup, so it only gets a
# warning; any other file is refused. All panes inherit this shell's
# environment, so checking here covers the whole stack.
DDS_CFG="$WS/src/robot_bringup/config/cyclonedds_localhost.xml"
if [ "${RMW_IMPLEMENTATION:-}" != "rmw_cyclonedds_cpp" ]; then
    echo "error: RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION:-}' (need rmw_cyclonedds_cpp) - set it in ~/.bashrc and open a new shell"; exit 1
fi
if [ "${CYCLONEDDS_URI:-}" = "file://$DDS_CFG" ]; then
    echo "warn: CYCLONEDDS_URI is the loopback XML - lane_node's camera subscriptions freeze with it; unset it and open a new shell"
elif [ -n "${CYCLONEDDS_URI:-}" ]; then
    echo "error: CYCLONEDDS_URI='${CYCLONEDDS_URI}' - unset it (Cyclone defaults)"; exit 1
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

# Gazebo OpenGL backend: resolved (and validated) here once, so the summary
# below and pane 2's launch always agree on it. gpu is for the short-track aeb
# and lidar demos; the full-track vlm and acc demos stay on software, like the
# recorded baselines.
case "$DEMO_MODE" in
    aeb|lidar) DEMO_GZ_GL="${DEMO_GZ_GL:-gpu}" ;;
    *)         DEMO_GZ_GL="${DEMO_GZ_GL:-software}" ;;
esac
case "$DEMO_GZ_GL" in
    gpu)      GL_DESC="gpu (Mesa d3d12 on the WSLg GPU)" ;;
    software) GL_DESC="software (llvmpipe, CPU)" ;;
    *) echo "DEMO_GZ_GL must be 'gpu' or 'software', got '$DEMO_GZ_GL'" >&2; exit 1 ;;
esac

export VLM_AV_WS="$WS" VLM_VENV="$VENV" DEMO_RUNDIR="$RUNDIR" DEMO_MODE DEMO_WORLD DEMO_GZ_GL
for v in DEMO_RVIZ DEMO_CURVE_SPLICE DEMO_SPEED DEMO_LEAD DEMO_LEAD_S DEMO_LEAD_PROFILE STACK_NOWAIT; do
    if [ -n "${!v:-}" ]; then export "${v?}"; fi
done

echo "terminator 3x2 course demo   mode=$DEMO_MODE   rundir=$RUNDIR   world=$WORLD"
echo "  Gazebo rendering: $GL_DESC   (DEMO_GZ_GL=gpu|software)"
case "$DEMO_MODE" in
    vlm)   echo "  1 vlm pipeline | 2 av_stack aeb:=false | 3 sign_maneuver | 4 setpoint | 5 health | 6 free" ;;
    aeb)   echo "  1 target_speed | 2 av_stack aeb:=true aeb_sensor:=depth | 3 aeb_status | 4 arbiter | 5 health | 6 free" ;;
    lidar) echo "  1 target_speed | 2 av_stack aeb:=true aeb_sensor:=lidar | 3 aeb_status | 4 arbiter | 5 health | 6 free" ;;
    acc)   echo "  1 lead + target_speed | 2 av_stack aeb_sensor:=lidar acc:=true | 3 acc | 4 arbiter | 5 health | 6 free"
           if [ "${DEMO_LEAD:-1}" = 1 ]; then
               echo "  lead: s=${DEMO_LEAD_S:-25}  profile=${DEMO_LEAD_PROFILE:-0:1.5}   (DEMO_LEAD_S / DEMO_LEAD_PROFILE / DEMO_LEAD=0)"
           else
               echo "  lead: none (DEMO_LEAD=0)"
           fi ;;
esac
echo "  close the window to tear everything down."
exec terminator -g "$CFG" -l avdemo
