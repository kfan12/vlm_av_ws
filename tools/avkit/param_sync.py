#!/usr/bin/env python3
"""param_sync.py - check hand-duplicated numbers against their owner.

  python3 tools/avkit/param_sync.py                 # every rule in tools/avkit/param_sync_rules.yaml
  python3 tools/avkit/param_sync.py --if-listed F   # only if file F appears in a rule (else exit 0)
  python3 tools/avkit/param_sync.py --list          # the files any rule touches

Sensor extrinsics and vehicle dimensions are repeated in the URDF, the YAML
configs, node defaults and analysis tools on purpose (no TF lookups), so a
change in one place must be made in every other. Each rule names the owner
and the copies with a regex capturing the value; a copy must equal
owner * scale (default 1). A pattern that no longer matches is an error too,
since it means a parameter was renamed under the rule.
Exit 0 when everything agrees, 1 otherwise. Needs PyYAML.
"""
import argparse
import os
import re
import sys

import yaml

WS = os.path.expanduser('~/vlm_av_ws')
RULES = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'param_sync_rules.yaml')


def first_value(spec):
    path = os.path.join(WS, spec['file'])
    try:
        with open(path) as f:
            text = f.read()
    except OSError:
        return None, f'{spec["file"]}: missing file'
    m = re.search(spec['regex'], text, re.M)
    if not m:
        return None, f'{spec["file"]}: pattern not found: {spec["regex"]}'
    line = text[:m.start()].count('\n') + 1
    return float(m.group(1)), f'{spec["file"]}:{line}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--if-listed', metavar='FILE')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--rules', default=RULES)
    a = ap.parse_args()
    with open(a.rules) as f:
        rules = yaml.safe_load(f)['rules']
    files = {r['owner']['file'] for r in rules} | {c['file'] for r in rules for c in r['copies']}
    if a.list:
        print('\n'.join(sorted(files)))
        return 0
    if a.if_listed is not None and a.if_listed.replace('\\', '/') not in files:
        return 0

    problems = []
    for r in rules:
        owner, where = first_value(r['owner'])
        if owner is None:
            problems.append(f'[{r["name"]}] owner {where}')
            continue
        want = owner * float(r.get('scale', 1.0))
        for c in r['copies']:
            got, cwhere = first_value(c)
            if got is None:
                problems.append(f'[{r["name"]}] {cwhere}')
            elif abs(got - want) > 1e-6:
                problems.append(f'[{r["name"]}] {cwhere} has {got:g}, owner {where} gives {want:g}')
    if problems:
        print('param sync: ' + str(len(problems)) + ' problem(s)')
        print('\n'.join('  ' + p for p in problems))
        return 1
    print(f'param sync: {len(rules)} rules, {sum(len(r["copies"]) for r in rules)} copies agree')
    return 0


if __name__ == '__main__':
    sys.exit(main())
