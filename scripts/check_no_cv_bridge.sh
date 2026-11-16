#!/usr/bin/env bash
# cv_bridge is banned: the venv's NumPy 2 breaks its Python binding, and
# C++ nodes wrap sensor_msgs data zero-copy into cv::Mat (img_access.hpp) so
# they don't need it either.
# Scans every package under src/, plus tools/ and scripts/, so a new package is
# covered without editing this file.
WS="$(cd "$(dirname "$0")/.." && pwd)"
# match real usage (include/import/dependency), not comments that mention the ban
hits=$(grep -rnIE "include.*cv_bridge|import cv_bridge|from cv_bridge|<depend>cv_bridge|find_package\(cv_bridge" \
  "$WS/src" "$WS/tools" "$WS/scripts" \
  --include=*.cpp --include=*.hpp --include=*.h --include=*.py --include=*.xml --include=CMakeLists.txt \
  --exclude-dir=__pycache__ 2>/dev/null)
if [ -n "$hits" ]; then
  echo "cv_bridge found:"
  echo "$hits"
  exit 1
fi
echo "OK: no cv_bridge in src/, tools/ or scripts/"
