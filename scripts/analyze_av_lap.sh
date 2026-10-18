#!/bin/bash
# analyze_av_lap.sh <session|latest> [--map maps/<world>.json] [--plots]
# Thin wrapper over tools/avkit/analyze_av_lap.py (no ROS needed).
set -e
export PYTHONNOUSERSITE=1
exec python3 "$HOME/vlm_av_ws/tools/avkit/analyze_av_lap.py" "$@"
