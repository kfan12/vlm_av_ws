#!/usr/bin/env python3
"""compare_aeb_sessions.py - one markdown table, depth vs lidar, per scenario.

  python3 tools/avkit/compare_aeb_sessions.py [--root data/aeb_scenarios] [--since 20261108]

Pairs every <row>_<v>mps_lidar session with the newest <row>_<v>mps (depth)
session of the same row and speed, and prints the key metrics of each
analyzed metrics.json side by side. Sessions without a partner are listed
alone. Needs analyze_aeb_scenario.py to have run on every session.

Pass --since with the day the matrix was run, so only sessions recorded in
one environment pair up (older sessions ran on other builds and networking).
"""
import argparse
import glob
import json
import os
import re


def _first_stop(m):
    """(source, time to standstill) of the first non-mpc source event, if any."""
    for e in m.get('source_timeline', []):
        if e['source'] != 'mpc':
            ev = (m.get('source_events') or {}).get(e['source']) or {}
            return e['source'], ev.get('time_to_standstill_s')
    return None, None


FIELDS = [
    ('sequence', lambda m: ' > '.join(m.get('source_sequence', []))),
    ('first seen gt [m]', lambda m: (m.get('first_detection') or {}).get('gt_standoff_m')),
    ('braking at [m/s]', lambda m: (m.get('first_braking') or {}).get('truth_speed_mps')),
    ('stop margin [m]', lambda m: (m.get('first_braking') or {}).get('stop_margin_m')),
    ('stopped short by [m]', lambda m: ((m.get('source_events') or {}).get('aeb') or {}).get('standstill_gt_standoff_m')),
    ('gate [ms]', lambda m: None if m.get('command_gate_response_sec') is None
        else round(1000 * m['command_gate_response_sec'])),
    ('to standstill [s]', lambda m: _first_stop(m)[1]),
    ('contact', lambda m: m.get('contact')),
    ('FP brakes', lambda m: m.get('false_positive_brakes')),
    ('range err mean [m]', lambda m: (m.get('distance_error_m') or {}).get('mean')),
    ('fault_aeb_status entries', lambda m: sum(1 for e in m.get('source_timeline', [])
                                               if e['source'] == 'fault_aeb_status')),
    ('fault reasons', lambda m: ', '.join(m.get('fault_reasons', [])) or '-'),
]
LABEL = re.compile(r'^\d{8}_\d{6}_(?P<row>.+?)_(?P<v>[\d.]+)mps(?P<lidar>_lidar)?$')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default=os.path.expanduser('~/vlm_av_ws/data/aeb_scenarios'))
    ap.add_argument('--since', default='', help='only sessions whose stamp sorts >= this prefix')
    args = ap.parse_args()

    latest = {}   # (row, v, sensor) -> (session, metrics); sorted, so the newest wins
    for d in sorted(glob.glob(os.path.join(args.root, '*'))):
        name = os.path.basename(d)
        m = LABEL.match(name)
        if not m or name < args.since or not os.path.exists(os.path.join(d, 'metrics.json')):
            continue
        with open(os.path.join(d, 'metrics.json')) as f:
            latest[(m['row'], m['v'], 'lidar' if m['lidar'] else 'depth')] = (name, json.load(f))

    keys = sorted({(r, v) for r, v, _ in latest})
    print('| row | speed | sensor | ' + ' | '.join(f for f, _ in FIELDS) + ' | session |')
    print('|' + '---|' * (4 + len(FIELDS)))
    for row, v in keys:
        for sensor in ('depth', 'lidar'):
            if (row, v, sensor) not in latest:
                continue
            name, m = latest[(row, v, sensor)]
            cells = [row, v, sensor] + ['-' if (x := fn(m)) is None else str(x) for _, fn in FIELDS] + [name]
            print('| ' + ' | '.join(cells) + ' |')


if __name__ == '__main__':
    main()
