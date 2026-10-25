#!/usr/bin/env python3
"""aeb_obstacle.py - spawn / remove a static AEB test box in the running Gazebo world.

  python3 tools/avkit/aeb_obstacle.py spawn X Y [NAME]            # world-frame x, y
  python3 tools/avkit/aeb_obstacle.py spawn --s 20 [--lateral -1.75] [--name N]
  python3 tools/avkit/aeb_obstacle.py remove [NAME]

Wrapped by scripts/spawn_aeb_obstacle.sh and scripts/remove_aeb_obstacle.sh.

--s/--lateral place the box by course arc length and left offset from the
centerline (same convention as generate_course_world.py --road-block; default
lateral is the right-lane center, -lane_width/2), using maps/<world>.json, so
in-lane / out-of-lane / on-curve positions come from the map, not guesses.
The box is yawed to the local road heading.

The world is auto-detected from the running Gazebo (`ign service -l`), so it
works in urban_course, urban_course_roadblock, etc.; --world overrides it.

Every spawn/remove is written to data/aeb_obstacles/<NAME>.json (pose in the
map frame = Gazebo world frame, size, wall-clock spawn/remove times).
record_aeb_scenario.py --obstacle NAME copies that record into meta.json, and
analyze_aeb_scenario.py uses it as ground truth for the time it was present.

No ROS needed: talks to Gazebo through the `ign service` CLI only.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

WS = os.path.expanduser('~/vlm_av_ws')
STATE_DIR = os.path.join(WS, 'data', 'aeb_obstacles')
DEFAULT_NAME = 'aeb_test_box'
DEFAULT_SIZE = (0.6, 0.6, 1.0)       # along road, across road, height [m]
RGBA = '0.9 0.1 0.1 1'               # red: outside lane_node's white/yellow HSV gates


def detect_world():
    try:
        out = subprocess.run(['ign', 'service', '-l'], capture_output=True, text=True,
                             timeout=10).stdout
    except (OSError, subprocess.TimeoutExpired) as e:
        sys.exit(f'cannot list Gazebo services ({e}); is Gazebo running?')
    worlds = sorted({line.split('/')[2] for line in out.split()
                     if line.startswith('/world/') and line.endswith('/create')})
    if len(worlds) != 1:
        sys.exit(f'expected exactly one running Gazebo world, found {worlds or "none"}; '
                 'pass --world')
    return worlds[0]


def load_course(world):
    path = os.path.join(WS, 'maps', world + '.json')
    try:
        with open(path) as f:
            mapj = json.load(f)
    except OSError:
        return None, None
    pts = mapj['course']['centerline']
    s = [0.0]
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        s.append(s[-1] + math.hypot(x1 - x0, y1 - y0))
    return [(si, p[0], p[1]) for si, p in zip(s, pts)], mapj.get('lane_width', 3.5)


def pose_at_s(course, s_query):
    """(x, y, heading) on the centerline at arc length s_query."""
    for (s0, x0, y0), (s1, x1, y1) in zip(course, course[1:]):
        if s1 >= s_query or (s1, x1, y1) == course[-1]:
            t = min(max((s_query - s0) / max(1e-9, s1 - s0), 0.0), 1.0)
            return x0 + t * (x1 - x0), y0 + t * (y1 - y0), math.atan2(y1 - y0, x1 - x0)
    sys.exit('course centerline has fewer than two points')


def project(course, x, y):
    """(s, lateral-left, heading) of the centerline point nearest (x, y)."""
    best = None
    for (s0, x0, y0), (s1, x1, y1) in zip(course, course[1:]):
        dx, dy = x1 - x0, y1 - y0
        L2 = max(1e-12, dx * dx + dy * dy)
        t = min(max(((x - x0) * dx + (y - y0) * dy) / L2, 0.0), 1.0)
        px, py = x0 + t * dx, y0 + t * dy
        d2 = (x - px) ** 2 + (y - py) ** 2
        if best is None or d2 < best[0]:
            h = math.atan2(dy, dx)
            lat = -(x - px) * math.sin(h) + (y - py) * math.cos(h)
            best = (d2, s0 + t * (s1 - s0), lat, h)
    return best[1], best[2], best[3]


def ign_call(world, service, reqtype, req):
    cmd = ['ign', 'service', '-s', f'/world/{world}/{service}', '--reqtype', reqtype,
           '--reptype', 'ignition.msgs.Boolean', '--timeout', '5000', '--req', req]
    r = subprocess.run(cmd, capture_output=True, text=True)
    ok = r.returncode == 0 and 'data: true' in r.stdout
    return ok, (r.stdout + r.stderr).strip()


def model_names(world):
    """Top-level models in the running world. The create/remove services reply
    true even when nothing happens (duplicate name, unknown name), so success is
    judged from this list instead. scene/info answers in about 1 s; `ign model
    --list` takes about 8 s, too slow for a sudden-appearance spawn."""
    r = subprocess.run(['ign', 'service', '-s', f'/world/{world}/scene/info',
                        '--reqtype', 'ignition.msgs.Empty', '--reptype', 'ignition.msgs.Scene',
                        '--timeout', '5000', '--req', ''],
                       capture_output=True, text=True, timeout=15)
    lines = r.stdout.splitlines()
    return {nxt.split('"')[1] for cur, nxt in zip(lines, lines[1:])
            if cur == 'model {' and nxt.startswith('  name: "')}


def wait_for(world, name, present, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if (name in model_names(world)) == present:
            return True
        time.sleep(0.25)
    return False


def state_path(name):
    return os.path.join(STATE_DIR, name + '.json')


def spawn(a):
    world = a.world or detect_world()
    course, lane_width = load_course(world)
    size = tuple(a.size)

    if a.s is not None:
        if a.x is not None:
            sys.exit('give either X Y or --s, not both')
        if course is None:
            sys.exit(f'--s needs maps/{world}.json')
        lat = a.lateral if a.lateral is not None else -lane_width / 2.0
        cx, cy, yaw = pose_at_s(course, a.s)
        x, y, s = cx - lat * math.sin(yaw), cy + lat * math.cos(yaw), a.s
    elif a.x is not None and a.y is not None:
        x, y = a.x, a.y
        if course is not None:
            s, lat, yaw = project(course, x, y)
        else:
            s, lat, yaw = None, None, 0.0
            print(f'warn: maps/{world}.json missing - box yaw 0, no s/lateral recorded')
    else:
        sys.exit('usage: spawn X Y [NAME]  or  spawn --s S [--lateral L]')

    sdf = (f"<sdf version='1.9'><model name='{a.name}'><static>true</static><link name='l'>"
           f"<collision name='c'><geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box>"
           f"</geometry></collision><visual name='v'><geometry><box><size>{size[0]} {size[1]} "
           f"{size[2]}</size></box></geometry><material><ambient>{RGBA}</ambient>"
           f"<diffuse>{RGBA}</diffuse></material></visual></link></model></sdf>")
    req = (f'sdf: "{sdf}" name: "{a.name}" allow_renaming: false '
           f'pose {{ position {{ x: {x:.4f} y: {y:.4f} z: {size[2] / 2.0:.4f} }} '
           f'orientation {{ z: {math.sin(yaw / 2.0):.6f} w: {math.cos(yaw / 2.0):.6f} }} }}')
    if a.name in model_names(world):
        sys.exit(f'{a.name} already exists in {world}; remove it first: '
                 f'scripts/remove_aeb_obstacle.sh {a.name}')
    t_req = time.time()   # the box cannot exist before the request
    ok, out = ign_call(world, 'create', 'ignition.msgs.EntityFactory', req)
    if not ok or not wait_for(world, a.name, present=True):
        sys.exit(f'spawn of {a.name} failed in world {world}: {out or "no reply"}')

    rec = {'name': a.name, 'world': world, 'x': round(x, 3), 'y': round(y, 3),
           'yaw': round(yaw, 4), 's': None if s is None else round(s, 2),
           'lateral': None if lat is None else round(lat, 3), 'size': list(size),
           'spawned_wall': round(t_req, 3), 'removed_wall': None}
    os.makedirs(STATE_DIR, exist_ok=True)
    with open(state_path(a.name), 'w') as f:
        json.dump(rec, f, indent=1)
    print(json.dumps(rec))
    print(f'spawned {a.name} in {world}; record it with --obstacle {a.name}, '
          f'remove with: scripts/remove_aeb_obstacle.sh {a.name}')


def remove(a):
    world = a.world or detect_world()
    if a.name not in model_names(world):
        sys.exit(f'no model named {a.name} in {world}')
    t_req = time.time()
    ok, out = ign_call(world, 'remove', 'ignition.msgs.Entity',
                       f'name: "{a.name}" type: MODEL')
    if not ok or not wait_for(world, a.name, present=False):
        sys.exit(f'remove of {a.name} failed in world {world}: {out or "no reply"}')
    try:
        with open(state_path(a.name)) as f:
            rec = json.load(f)
        rec['removed_wall'] = round(t_req, 3)
        with open(state_path(a.name), 'w') as f:
            json.dump(rec, f, indent=1)
    except OSError:
        pass  # not spawned by this tool (e.g. the generated road_block)
    print(f'removed {a.name} from {world}')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)

    sp = sub.add_parser('spawn')
    sp.add_argument('x', type=float, nargs='?')
    sp.add_argument('y', type=float, nargs='?')
    sp.add_argument('name_pos', nargs='?', metavar='NAME')
    sp.add_argument('--name', default=None)
    sp.add_argument('--s', type=float, default=None, help='course arc length [m]')
    sp.add_argument('--lateral', type=float, default=None,
                    help='left offset from the centerline [m] (default -lane_width/2)')
    sp.add_argument('--size', type=float, nargs=3, default=DEFAULT_SIZE,
                    metavar=('ALONG', 'ACROSS', 'HEIGHT'))
    sp.add_argument('--world', default=None)

    rp = sub.add_parser('remove')
    rp.add_argument('name', nargs='?', default=DEFAULT_NAME)
    rp.add_argument('--world', default=None)

    a = ap.parse_args()
    if a.cmd == 'spawn':
        a.name = a.name or a.name_pos or DEFAULT_NAME
        spawn(a)
    else:
        remove(a)


if __name__ == '__main__':
    main()
