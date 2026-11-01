#!/usr/bin/env python3
"""obstacle_range_table.py - score obstacle_probe's log against spawned boxes.

  python3 tools/avkit/obstacle_range_table.py PROBE.jsonl NAME [NAME ...]

For each box NAME (a data/aeb_obstacles/NAME.json spawn record) it takes the
probe samples from 2 s after the spawn until the remove, and reports per
sensor: detection rate (a cluster in the lane band at all), median measured
range, and its error against the truth - the box's near face minus the car's
/odom_truth x, both in the world frame, valid on a straight with the car
facing +x (the urban_course start). Ranges are base_link x of the nearest
point, i.e. from the chassis centre.
"""
import json
import os
import statistics
import sys

OBSTACLE_DIR = os.path.expanduser('~/vlm_av_ws/data/aeb_obstacles')


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    rows = [json.loads(l) for l in open(sys.argv[1]) if l.strip()]
    print('| box s [m] | true range [m] | depth seen | depth range | depth err | depth pts | '
          'lidar seen | lidar range | lidar err | lidar pts | depth ms | lidar ms |')
    print('|' + '---|' * 12)
    for name in sys.argv[2:]:
        rec = json.load(open(os.path.join(OBSTACLE_DIR, name + '.json')))
        t0, t1 = rec['spawned_wall'] + 2.0, rec['removed_wall'] or float('inf')
        win = [r for r in rows if t0 <= r['wall'] <= t1]
        if not win:
            print(f"| {rec['s']} | - | no samples |")
            continue
        face_x = rec['x'] - rec['size'][0] / 2.0
        truth = statistics.median(face_x - r['car_x'] for r in win)
        cells = [f"{rec['s']:.0f}", f'{truth:.2f}']
        for sensor in ('depth', 'lidar'):
            seen = [r[sensor] for r in win if r[sensor]['x'] >= 0]
            rate = len(seen) / len(win)
            if seen:
                rng = statistics.median(s['x'] for s in seen)
                pts = statistics.median(s['n_points'] for s in seen)
                cells += [f'{rate:.2f}', f'{rng:.2f}', f'{rng - truth:+.2f}', f'{pts:.0f}']
            else:
                cells += [f'{rate:.2f}', '-', '-', '-']
        for sensor in ('depth', 'lidar'):
            cells.append(f"{statistics.median(r[sensor]['ms'] for r in win):.1f}")
        print('| ' + ' | '.join(cells) + ' |')


if __name__ == '__main__':
    main()
