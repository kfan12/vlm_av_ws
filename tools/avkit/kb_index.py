#!/usr/bin/env python3
"""kb_index.py - the node index in local/kb/index.yaml, generated where it can be.

  python3 tools/avkit/kb_index.py --update   # regenerate the code-derived fields in place
  python3 tools/avkit/kb_index.py --check    # exit 1 if the file lags the code or names missing files
  python3 tools/avkit/kb_index.py --show NODE

Per node the file holds two kinds of fields. Generated from the sources (never
edit by hand): source, publishes, subscribes, parameters. Hand-written:
package, description, setups, plans, notes, reports, invariants. --update keeps
the hand fields and rewrites the generated ones; a node found in the code but
not in the file is added with empty hand fields, so --check then asks for them.
The launch file's remappings are recorded under launch_remaps.
Topic strings are what the code passes (relative names stay relative).
Needs PyYAML.
"""
import argparse
import glob
import os
import re
import sys

import yaml

WS = os.path.expanduser('~/vlm_av_ws')
INDEX = os.path.join(WS, 'local', 'kb', 'index.yaml')
GENERATED = ('source', 'publishes', 'subscribes', 'parameters')
HAND = ('package', 'description', 'setups', 'plans', 'notes', 'reports', 'invariants')

# topic as a literal, or an expression (odom_noise passes in_topic_) recorded as <expression>
CPP_PUB = re.compile(r'create_publisher\s*<\s*([^>]+?)\s*>\s*\(\s*([^,]+?)\s*,', re.S)
CPP_SUB = re.compile(r'create_subscription\s*<\s*([^>]+?)\s*>\s*\(\s*([^,]+?)\s*,', re.S)
CPP_PARAM = re.compile(r'declare_parameter\s*(?:<[^>]+>)?\s*\(\s*"([^"]+)"')
# the topic is a literal or an expression (vlm_sign_node passes self.sign_topic); an
# expression is recorded as <expression> so the index still names the publisher
PY_PUB = re.compile(r"create_publisher\(\s*(\w+)\s*,\s*([^,]+?)\s*,", re.S)
PY_SUB = re.compile(r"create_subscription\(\s*(\w+)\s*,\s*([^,]+?)\s*,", re.S)
PY_PARAM = re.compile(r"declare_parameter\(\s*['\"]([^'\"]+)['\"]")
# a remapping tuple anywhere in the launch file: ('from', '/to'), the target always absolute
REMAP = re.compile(r"\(\s*'([^']+)'\s*,\s*'(/[^']+)'\s*\)")

CPP_NODE_NAMES = {  # file stem -> executable / node name where they differ
    'mpc_tracker_v2_node': 'mpc_tracker_v2',
}
SKIP_PY = {'__init__', 'aeb_obstacle', 'analyze_aeb_scenario', 'analyze_av_lap', 'analyze_acc_run',
           'compare_aeb_sessions', 'export_lane_csv', 'extract_frames', 'obstacle_range_table',
           'kb_index', 'param_sync', 'img_convert', 'json_parser', 'vlm_prompts', 'sign_latch',
           'maneuver', 'model_runner'}


def py_topic(expr):
    expr = ' '.join(expr.split())
    m = re.fullmatch(r"['\"]([^'\"]+)['\"]", expr)
    return m.group(1) if m else f'<{expr}>'


def short_type(t):
    t = t.strip().replace('::msg::', '/').replace('::', '/')
    return t.split('/')[-2] + '/' + t.split('/')[-1] if '/' in t else t


def scan():
    nodes = {}
    for path in sorted(glob.glob(os.path.join(WS, 'src', '*', 'src', '*.cpp'))):
        text = open(path).read()
        if 'create_publisher' not in text and 'create_subscription' not in text:
            continue
        stem = os.path.splitext(os.path.basename(path))[0]
        name = CPP_NODE_NAMES.get(stem, stem)
        pkg = path.split(os.sep)[-3]
        pubs = {f'{py_topic(t)} {short_type(ty)}' for ty, t in CPP_PUB.findall(text)}
        # av_common's DebugTap publishes /debug/<node name> (a String per tick) from the header
        if 'DebugTap' in text:
            pubs.add(f'/debug/{name} std_msgs/String')
        nodes[name] = {
            'package': pkg,
            'source': os.path.relpath(path, WS),
            'publishes': sorted(pubs),
            'subscribes': sorted({f'{py_topic(t)} {short_type(ty)}' for ty, t in CPP_SUB.findall(text)}),
            'parameters': sorted(set(CPP_PARAM.findall(text))),
        }
    py_files = glob.glob(os.path.join(WS, 'src', 'vlm_planner_py', 'vlm_planner_py', '*.py'))
    py_files += glob.glob(os.path.join(WS, 'tools', 'avkit', '*.py'))
    py_files += glob.glob(os.path.join(WS, 'scripts', '*.py'))
    for path in sorted(py_files):
        stem = os.path.splitext(os.path.basename(path))[0]
        if stem in SKIP_PY:
            continue
        text = open(path).read()
        pubs = [(ty, py_topic(t)) for ty, t in PY_PUB.findall(text)]
        subs = [(ty, py_topic(t)) for ty, t in PY_SUB.findall(text)]
        if not pubs and not subs:
            continue
        rel = os.path.relpath(path, WS)
        pkg = 'vlm_planner_py' if 'vlm_planner_py' in rel else rel.split('/')[0] + '/' + rel.split('/')[1]
        nodes[stem] = {
            'package': pkg,
            'source': rel,
            'publishes': sorted({f'{t} {ty}' for ty, t in pubs}),
            'subscribes': sorted({f'{t} {ty}' for ty, t in subs}),
            'parameters': sorted(set(PY_PARAM.findall(text))),
        }
    return nodes


def launch_remaps():
    path = os.path.join(WS, 'src', 'robot_bringup', 'launch', 'av_stack.launch.py')
    text = open(path).read()
    return sorted({f'{a} -> {b}' for a, b in REMAP.findall(text)})


def load():
    if not os.path.exists(INDEX):
        return {'nodes': {}, 'launch_remaps': []}
    with open(INDEX) as f:
        return yaml.safe_load(f) or {'nodes': {}, 'launch_remaps': []}


def merged(current):
    code = scan()
    out = {'nodes': {}, 'launch_remaps': launch_remaps()}
    for name in sorted(set(code) | set(current.get('nodes', {}))):
        hand = current.get('nodes', {}).get(name, {})
        entry = {}
        gen = code.get(name)
        entry['package'] = hand.get('package') or (gen or {}).get('package', '')
        entry['description'] = hand.get('description', '')
        if gen:
            for k in GENERATED:
                entry[k] = gen[k]
        else:
            entry['source'] = hand.get('source', '')
            entry['removed'] = 'no source found; delete this entry or fix source'
        for k in ('setups', 'plans', 'notes', 'reports', 'invariants'):
            entry[k] = hand.get(k, [])
        out['nodes'][name] = entry
    return out


def dump(data):
    header = ('# Node index for vlm_av_ws. Fields source/publishes/subscribes/parameters are\n'
              '# generated by tools/avkit/kb_index.py --update (do not edit); package,\n'
              '# description, setups, plans, notes, reports, invariants are hand-written.\n'
              '# --check fails when the generated fields lag the code or a listed file is missing.\n')
    with open(INDEX, 'w') as f:
        f.write(header)
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True, width=120)


def check(current):
    problems = []
    want = merged(current)
    for name, entry in want['nodes'].items():
        have = current.get('nodes', {}).get(name)
        if have is None:
            problems.append(f'{name}: in the code but not in the index (run --update, then fill the hand fields)')
            continue
        for k in GENERATED:
            if have.get(k) != entry.get(k):
                problems.append(f'{name}.{k}: index lags the code (run --update)')
        if 'removed' in entry:
            problems.append(f'{name}: {entry["removed"]}')
        for k in ('plans', 'notes', 'reports'):
            for p in have.get(k, []) or []:
                if not os.path.exists(os.path.join(WS, p.split('#')[0])):
                    problems.append(f'{name}.{k}: missing file {p}')
        if not have.get('description'):
            problems.append(f'{name}: no description')
    if current.get('launch_remaps') != want['launch_remaps']:
        problems.append('launch_remaps lag the launch file (run --update)')
    return problems


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--update', action='store_true')
    g.add_argument('--check', action='store_true')
    g.add_argument('--show', metavar='NODE')
    a = ap.parse_args()
    current = load()
    if a.update:
        dump(merged(current))
        print(f'wrote {INDEX}: {len(merged(current)["nodes"])} nodes')
        return 0
    if a.show:
        print(yaml.safe_dump({a.show: current.get('nodes', {}).get(a.show)}, sort_keys=False))
        return 0
    problems = check(current)
    if problems:
        print('kb index: ' + str(len(problems)) + ' problem(s)')
        print('\n'.join('  ' + p for p in problems))
        return 1
    print(f'kb index: {len(current.get("nodes", {}))} nodes match the code')
    return 0


if __name__ == '__main__':
    sys.exit(main())
