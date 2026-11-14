#!/bin/bash
# check_param_sync.sh [--if-listed FILE] [--list]
# Verifies that every hand-duplicated number (camera and lidar extrinsics,
# vehicle and lead lengths) agrees with its owner, per tools/avkit/param_sync_rules.yaml.
# Thin wrapper over tools/avkit/param_sync.py (no ROS needed). Exit 1 on a mismatch.
set -e
export PYTHONNOUSERSITE=1
exec python3 "$HOME/vlm_av_ws/tools/avkit/param_sync.py" "$@"
