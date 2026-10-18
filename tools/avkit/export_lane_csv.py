#!/usr/bin/env python3
"""export_lane_csv.py - write a map's right-hand travel lane as "x,y" lines.

  python3 tools/avkit/export_lane_csv.py maps/urban_course.json > /tmp/lane.csv

Same lane construction as analyze_av_lap.py (course centerline offset half a
lane right of travel), world frame. Input for av_behavior_cpp's
mpc_closed_loop_sim.
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_av_lap import right_lane  # noqa: E402


def main():
    if len(sys.argv) != 2:
        sys.exit('usage: export_lane_csv.py <map.json>')
    with open(sys.argv[1]) as f:
        mapj = json.load(f)
    lane = right_lane(np.asarray(mapj['course']['centerline'], dtype=float),
                      mapj['lane_width'] / 2.0)
    for x, y in lane:
        print(f'{x:.4f},{y:.4f}')


if __name__ == '__main__':
    main()
