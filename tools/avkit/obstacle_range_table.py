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

Samples where a sensor was stale (obstacle_probe logged "stale": true - no
message within sensor_stale_sec, e.g. a frozen render) are neither detections
nor misses for that sensor: they are left out of its rate, range and timing,
and counted in its "stale" column. Logs without the field count as fresh.
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
    print('| box s [m] | true range [m] | depth seen | depth range | depth err | depth pts | depth stale | '
          'lidar seen | lidar range | lidar err | lidar pts | lidar stale | depth ms | lidar ms |')
    print('|' + '---|' * 14)
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
        fresh = {sensor: [r[sensor] for r in win if not r[sensor].get('stale', False)]
                 for sensor in ('depth', 'lidar')}
        for sensor in ('depth', 'lidar'):
            f = fresh[sensor]
            n_stale = len(win) - len(f)
            seen = [x for x in f if x['x'] >= 0]
            if not f:
                cells += ['-', '-', '-', '-', f'{n_stale}']
            elif seen:
                rng = statistics.median(x['x'] for x in seen)
                pts = statistics.median(x['n_points'] for x in seen)
                cells += [f'{len(seen) / len(f):.2f}', f'{rng:.2f}', f'{rng - truth:+.2f}', f'{pts:.0f}',
                          f'{n_stale}']
            else:
                cells += ['0.00', '-', '-', '-', f'{n_stale}']
        for sensor in ('depth', 'lidar'):
            f = fresh[sensor]
            cells.append(f"{statistics.median(x['ms'] for x in f):.1f}" if f else '-')
        print('| ' + ' | '.join(cells) + ' |')

if __name__ == '__main__':
    main()
