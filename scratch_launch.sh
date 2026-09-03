#!/bin/bash
source /opt/ros/humble/setup.bash
cd ~/vlm_av_ws
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
exec ros2 launch robot_bringup urban_world.launch.py rviz:=false > /tmp/av_clean_launch.log 2>&1
