#!/bin/bash
# stop_av_stack.sh - stop every simulator, stack and helper process, whoever started it.
#
# The one command for "something is still running": a headless run that was
# interrupted, a demo whose Gazebo GUI outlived its terminal (the GUI's server
# and client run in their own process groups), a lead vehicle left driving.
# Uses the same process list and TERM-then-KILL sequence as the scenario runners.
#   ./scripts/stop_av_stack.sh        stop, then list what is left (exit 1 if anything is)
#   ./scripts/stop_av_stack.sh -n     only list what would be stopped
WS="${VLM_AV_WS:-$HOME/vlm_av_ws}"
source "$WS/scripts/_av_stack_common.sh"
# the VLM chain and the lap recorder are not in the runners' list
PROCS="$STACK_PROCS|vlm_sign_node|sign_maneuver_node|record_av_lap"
list() { pgrep -af "$PROCS" | grep -v "stop_av_stack.sh" ; }

if [ "$1" = -n ]; then
    list || echo "nothing running"
    exit 0
fi
if ! list > /dev/null; then
    echo "nothing running"
    exit 0
fi
echo "stopping:"; list | cut -c1-140 | sed 's/^/  /'
pkill -f "$PROCS" 2>/dev/null
sleep 3
pkill -9 -f "$PROCS" 2>/dev/null
sleep 1
if list > /dev/null; then
    echo "still running:"; list | cut -c1-140 | sed 's/^/  /'
    exit 1
fi
echo "all stopped"
