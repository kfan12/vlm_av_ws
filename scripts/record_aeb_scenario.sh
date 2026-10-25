#!/bin/bash
# record_aeb_scenario.sh [--label L --duration S --world W --target-speed V]
# Thin wrapper over tools/avkit/record_aeb_scenario.py.
set -e
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source "$HOME/vlm_av_ws/install/setup.bash"
exec python3 "$HOME/vlm_av_ws/tools/avkit/record_aeb_scenario.py" "$@" \
  --ros-args -p use_sim_time:=true
