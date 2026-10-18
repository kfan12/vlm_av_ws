#!/bin/bash
# record_av_lap.sh [--label L --duration S --world W]
# Thin wrapper over tools/avkit/record_av_lap.py (Day 15).
# System python, no venv; use_sim_time so stamps share the sim clock epoch.
set -e
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source "$HOME/vlm_av_ws/install/setup.bash"
exec python3 "$HOME/vlm_av_ws/tools/avkit/record_av_lap.py" "$@" \
  --ros-args -p use_sim_time:=true
