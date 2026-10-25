#!/bin/bash
# spawn_aeb_obstacle.sh X Y [NAME] | --s S [--lateral L] [--name NAME]
#   [--size ALONG ACROSS HEIGHT] [--world W]
# Drop a static 0.6x0.6x1.0 m box into the running Gazebo world (auto-detected)
# at world-frame X,Y or at course arc length S. See
# tools/avkit/aeb_obstacle.py.
set -e
exec python3 "$HOME/vlm_av_ws/tools/avkit/aeb_obstacle.py" spawn "$@"
