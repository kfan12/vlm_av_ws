#!/usr/bin/env bash
# cv_bridge is banned in v2: the venv's NumPy 2 breaks its Python binding, and
# C++ nodes wrap sensor_msgs data zero-copy into cv::Mat (img_access.hpp) so
# they don't need it either. Day 01 guard.
WS="$(cd "$(dirname "$0")/.." && pwd)"
# match real usage (include/import), not comments that mention the ban
hits=$(grep -rnE "include.*cv_bridge|import cv_bridge|from cv_bridge" \
  "$WS/src/av_common_cpp" "$WS/src/av_perception_cpp" "$WS/src/av_behavior_cpp" \
  "$WS/src/urban_gazebo" "$WS/src/sedan_description" \
  "$WS/src/vlm_planner_py/vlm_planner_py/vlm_server.py" \
  "$WS/src/vlm_planner_py/vlm_planner_py/vlm_scene_node.py" \
  "$WS/src/vlm_planner_py/vlm_planner_py/mission_node.py" \
  2>/dev/null | grep -v "\.md")
if [ -n "$hits" ]; then
  echo "cv_bridge found in v2 code:"
  echo "$hits"
  exit 1
fi
echo "OK: no cv_bridge in v2 code"