#!/bin/bash
# remove_aeb_obstacle.sh [NAME] [--world W] - undo spawn_aeb_obstacle.sh
# (default NAME aeb_test_box; world auto-detected from the running Gazebo).
set -e
exec python3 "$HOME/vlm_av_ws/tools/avkit/aeb_obstacle.py" remove "$@"
