#!/usr/bin/env python3
"""build.py - regenerate local/kb/architecture.html from the code.

Publishers and subscribers come from tools/avkit/kb_index.py's scan of the sources (not the
index file, so the page cannot lag the code), setups and descriptions from
local/kb/index.yaml, launch remaps per node from layout.py (checked against
av_stack.launch.py), positions and routes from layout.py.

  python3 local/kb/architecture/build.py [bare_page.html]

Exit 1 when the drawing no longer matches the code: an edge with no route in layout.py,
a node in a demo setup with no position, a launch remap with no rule, an unresolved
topic expression, or a planned node that now exists in the code.
"""
import datetime
import json
import os
import re
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(WS, 'tools', 'avkit'))
import kb_index  # noqa: E402
import layout as Lo  # noqa: E402

problems = []
code = kb_index.scan()
index = yaml.safe_load(open(os.path.join(WS, 'local', 'kb', 'index.yaml')))['nodes']
DEMO = set(Lo.SETUPS) - {'acc_ot'}


def setups_of(name):
    s = list(index.get(name, {}).get('setups') or [])
    return s + (['acc_ot'] if 'acc' in s else [])


def split(entry):
    t, _, ty = entry.partition(' ')
    return t, ty


def norm(t):
    return t if t.startswith('/') or t.startswith('<') else '/' + t


def resolve(node, topic, src_path):
    """'<expr>' -> list of topics: from a config placeholder or the literal default in the source."""
    if not topic.startswith('<'):
        return [topic]
    expr = topic[1:-1]
    ph = Lo.PLACEHOLDERS.get((node, expr))
    if ph:
        cfg = yaml.safe_load(open(os.path.join(WS, ph['config'])))
        params = cfg['/**']['ros__parameters'] if '/**' in cfg else next(iter(cfg.values()))['ros__parameters']
        return list(params[ph['key']])
    text = open(os.path.join(WS, src_path)).read()
    m = re.search(re.escape(expr) + r'\s*=\s*[^\n]*?["\'](/[^"\']+)["\']', text)
    if m:
        return [m.group(1)]
    problems.append(f'{node}: cannot resolve topic expression {topic} in {src_path}')
    return []


# ---------------------------------------------------------------- every drawn node's topics
nodes = {}
for nid, lo in Lo.NODES.items():
    ext = Lo.EXTERNAL.get(nid)
    if lo.get('planned') and nid in code:
        problems.append(f'{nid} is now in the code: drop planned=True and its EXTERNAL entry in layout.py')
    pubs, subs = [], []  # (topic, type, setups)
    if ext:
        for e, st in ext.get('pub', []):
            pubs.append((*split(e), st))
        for e, st in ext.get('sub', []):
            subs.append((*split(e), st))
        for path, st in ext.get('bridges', []):
            for b in yaml.safe_load(open(os.path.join(WS, path))):
                ty = b['ros_type_name'].replace('/msg/', '/')
                (pubs if b['direction'] == 'GZ_TO_ROS' else subs).append((b['ros_topic_name'], ty, st))
        if ext.get('ekf_config'):
            cfg = yaml.safe_load(open(os.path.join(WS, ext['ekf_config'])))
            p = next(iter(cfg.values()))['ros__parameters']
            for k, ty in (('odom0', 'nav_msgs/Odometry'), ('imu0', 'sensor_msgs/Imu')):
                if k in p:
                    subs.append((p[k], ty, Lo.SETUPS))
        setups = sorted({s for _, _, st in pubs + subs for s in st}, key=Lo.SETUPS.index)
        pkg, desc = ext['pkg'], ext['d']
    elif nid in code:
        c = code[nid]
        setups = setups_of(nid)
        if not setups:
            problems.append(f'{nid}: no setups in local/kb/index.yaml')
        for e in c['publishes']:
            t, ty = split(e)
            pubs += [(x, ty, setups) for x in resolve(nid, t, c['source'])]
        for e in c['subscribes']:
            t, ty = split(e)
            subs += [(x, ty, Lo.ONLY_IN.get((nid, norm(x)), setups)) for x in resolve(nid, t, c['source'])]
        for e, st in Lo.EXTRA_SUBS.get(nid, []):
            subs.append((*split(e), st))
        pkg, desc = c['package'], index.get(nid, {}).get('description', '')
    else:
        problems.append(f'{nid}: in layout.py but neither in the code nor EXTERNAL')
        continue
    nodes[nid] = dict(x=lo['x'], y=lo['y'], w=lo['w'], h=lo.get('h', Lo.H), t=lo.get('t', nid), s=lo.get('s', ''),
                      sim=lo.get('sim', False), planned=lo.get('planned', False), pkg=pkg, d=desc,
                      setups=setups, pubs=pubs, subs=subs)

# a node in a demo setup that the drawing does not place
for (n, t) in Lo.ONLY_IN:
    if n in code and not any(norm(resolve(n, split(e)[0], code[n]['source'])[0] if split(e)[0].startswith('<') else split(e)[0]) == t
                             for e in code[n]['publishes'] + code[n]['subscribes']):
        problems.append(f'ONLY_IN rule ({n}, {t}) matches no topic of that node any more')
for name in code:
    if name not in Lo.NODES and name not in Lo.IGNORE and DEMO & set(setups_of(name)):
        problems.append(f'{name} runs in {sorted(DEMO & set(setups_of(name)))} but has no position in layout.py (or add it to IGNORE)')

# every launch remap covered by a rule
rules = {(a, b) for _, a, b, _ in Lo.REMAPS}
for r in kb_index.launch_remaps():
    a, b = [x.strip() for x in r.split('->')]
    if (a, b) not in rules:
        problems.append(f'launch remap {a} -> {b} has no rule in layout.py REMAPS')


def remapped(nid, topic, setup):
    for n, a, b, st in Lo.REMAPS:
        if n == nid and setup in st and (topic == a or norm(topic) == norm(a)):
            return b
    return norm(topic)


# ---------------------------------------------------------------- edges per setup
edges, bus = {}, {}
for s in Lo.SETUPS:
    pub_of, sub_of = {}, {}
    for nid, n in nodes.items():
        if s not in n['setups']:
            continue
        for t, ty, st in n['pubs']:
            if s in st:
                pub_of.setdefault(remapped(nid, t, s), []).append((nid, ty))
        for t, ty, st in n['subs']:
            if s in st:
                sub_of.setdefault(remapped(nid, t, s), []).append((nid, ty))
    for t, ps in pub_of.items():
        subs = [n for n, _ in sub_of.get(t, []) if n not in [p for p, _ in ps]]
        if t in Lo.BUS:
            if subs:
                bus.setdefault(t, {'from': ps[0][0], **Lo.BUS[t], 'per': {}})['per'][s] = sorted(set(subs))
            continue
        for p, ty in ps:
            for q in subs:
                e = edges.setdefault(f'{p}>{q}', {'from': p, 'to': q, 'per': {}})
                e['per'].setdefault(s, [])
                if [t, ty] not in e['per'][s]:
                    e['per'][s].append([t, ty])

TYPE_KIND = [('Image', 'sensor'), ('PointCloud2', 'sensor'), ('Imu', 'sensor'), ('CameraInfo', 'sensor'),
             ('Odometry', 'sensor'), ('Path', 'path'), ('Float32MultiArray', 'path'), ('VelocityConstraint', 'speed'),
             ('Twist', 'cmd'), ('AebStatus', 'cmd')]


def kind_of(topics):
    for t, ty in topics:
        if t in Lo.TOPIC_KIND:
            return Lo.TOPIC_KIND[t]
        for k, v in TYPE_KIND:
            if ty.endswith(k):
                return v
    return 'fb'


def auto_label(topics):
    out = []
    for t, _ in topics:
        lab = Lo.TOPIC_LABEL.get(t, t)
        if lab and lab not in out:
            out.append(lab)
    return ' + '.join(out)


out_edges = []
for key, e in sorted(edges.items()):
    r = Lo.ROUTES.get(key)
    a, b = nodes[e['from']], nodes[e['to']]
    if not r:
        problems.append(f'edge {key} ({", ".join(sorted({t for v in e["per"].values() for t, _ in v}))}) has no route in layout.py: auto-routed')
        x1, y1 = a['x'] + a['w'], a['y'] + a['h'] / 2
        x2, y2 = b['x'], b['y'] + b['h'] / 2
        if x2 < x1:
            x1, x2 = a['x'], b['x'] + b['w']
        mx = (x1 + x2) / 2
        r = dict(d=f'M{x1:.0f},{y1:.0f} C{mx:.0f},{y1:.0f} {mx:.0f},{y2:.0f} {x2:.0f},{y2:.0f}', lx=mx, ly=(y1 + y2) / 2)
    all_topics = [tt for v in e['per'].values() for tt in v]
    per = {s: {'label': r['label'] if 'label' in r else auto_label(v), 'topics': v} for s, v in e['per'].items()}
    out_edges.append(dict(id=key, frm=e['from'], to=e['to'], d=r['d'], lx=r.get('lx', 0), ly=r.get('ly', 0),
                          kind=r.get('kind') or kind_of(all_topics), planned=a['planned'] or b['planned'], per=per))
for t, bdef in bus.items():
    per = {s: {'label': f'{t} to ' + ', '.join(nodes[n]['t'] for n in subs),
               'topics': [[t, 'nav_msgs/Odometry']], 'to': ', '.join(nodes[n]['t'] for n in subs)} for s, subs in bdef['per'].items()}
    out_edges.append(dict(id='bus' + t.replace('/', '-'), frm=bdef['from'], to='*', d=bdef['d'], lx=bdef['lx'], ly=bdef['ly'],
                          kind='fb', planned=False, bus=True, per=per))
for key in Lo.ROUTES:
    if key not in edges:
        problems.append(f'route {key} in layout.py matches no edge in any setup (stale?)')

# ---------------------------------------------------------------- page
page_nodes = {}
for nid, n in nodes.items():
    page_nodes[nid] = dict(x=n['x'], y=n['y'], w=n['w'], h=n['h'], t=n['t'], s=n['s'], sim=n['sim'], planned=n['planned'],
                           pkg=n['pkg'], d=n['d'], setups=n['setups'],
                           pub=sorted({t for t, _, _ in n['pubs']}), sub=sorted({t for t, _, _ in n['subs']}))
data = dict(nodes=page_nodes, edges=out_edges, notes=Lo.NOTES, ports=Lo.PORTS, built=datetime.date.today().isoformat())
tpl = open(os.path.join(HERE, 'template.html'), encoding='utf-8').read()
page = tpl.replace('/*__DATA__*/null', json.dumps(data, ensure_ascii=False).replace('</', '<\\/'))
if chr(0x2014) in page:
    problems.append('em dash in the page (layout.py or a description)')
if len(sys.argv) > 1:
    open(sys.argv[1], 'w', encoding='utf-8', newline='\n').write(page)
head_end = page.index('</style>') + len('</style>')
note = ('<!-- Generated by local/kb/architecture/build.py from the code (topics), local/kb/index.yaml (setups,\n'
        '     descriptions) and layout.py (positions, routes). Do not edit: change layout.py and rebuild. -->\n')
full = ('<!doctype html>\n<html lang="en">\n<head>\n<meta charset="utf-8">\n'
        '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">\n'
        + note + page[:head_end] + '\n<style>body{margin:0}</style>\n</head>\n<body>' + page[head_end:] + '\n</body>\n</html>\n')
open(os.path.join(WS, 'local', 'kb', 'architecture.html'), 'w', encoding='utf-8', newline='\n').write(full)
print(f'architecture: {len(nodes)} nodes, {len(out_edges)} edges -> local/kb/architecture.html')
if problems:
    print(f'architecture: {len(problems)} problem(s)')
    print('\n'.join('  ' + p for p in problems))
    sys.exit(1)
