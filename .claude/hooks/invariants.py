#!/usr/bin/env python3
"""invariants.py - the rule registry (local/kb/invariants.yaml) against the files.

  python3 .claude/hooks/invariants.py --check   exit 1 when a rule's text is gone from a file that should state it
  python3 .claude/hooks/invariants.py --table   the coverage table: which rule is enforced by what
  python3 .claude/hooks/invariants.py --show ID one rule: its owner and every restatement to edit together

Each rule has an owner file and an anchor (a phrase that file must contain), a
list of restatements (file + anchor) and what enforces it. --check verifies that
every anchor is still present, that the scripts, skill checks and evals named
under enforced_by exist, and that every invariant id a node carries in
local/kb/index.yaml is registered. A decision that changes a rule is applied to
the owner and to every restatement: --show lists them.
Enforcement kinds: hook (hook.py refuses or asks), script (a checker exits
non-zero), launch (the launch file raises), test (a hook test or a skill check),
scenario (a recorded row proves it), eval (a prompt eval), agent (a reviewer
reads for it), none.
"""
import os
import sys

import yaml

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
REGISTRY = os.path.join(WS, 'local', 'kb', 'invariants.yaml')
MECHANICAL = ('hook', 'script', 'launch', 'test', 'scenario')


def load():
    with open(REGISTRY) as f:
        return yaml.safe_load(f)['invariants']


def has(rel, anchor):
    try:
        with open(os.path.join(WS, rel), encoding='utf-8') as f:
            return anchor in f.read()
    except OSError:
        return None


def kinds(rule):
    return [str(e).split(':', 1)[0].strip() for e in rule.get('enforced_by') or ['none']]


def level(rule):
    k = kinds(rule)
    if any(x in MECHANICAL for x in k):
        return 'mechanical'
    if 'eval' in k:
        return 'eval'
    if 'agent' in k:
        return 'review only'
    return 'none'


def check():
    rules = load()
    problems = []
    for rid, r in rules.items():
        for where, rel, anchor in [('owner', r.get('owner'), r.get('anchor'))] + \
                [('restated_in', x.get('file'), x.get('anchor')) for x in r.get('restated_in') or []]:
            if not rel or not anchor:
                problems.append(f'{rid}: {where} needs a file and an anchor')
                continue
            found = has(rel, anchor)
            if found is None:
                problems.append(f'{rid}: {where} file {rel} is missing')
            elif not found:
                problems.append(f'{rid}: {rel} no longer contains "{anchor}" (the rule moved or was reworded: update the registry)')
        if not r.get('statement'):
            problems.append(f'{rid}: no statement')
        for e in r.get('enforced_by') or []:
            kind, _, ref = str(e).partition(':')
            ref = ref.strip().split(' ')[0]
            if kind == 'script' and not os.path.exists(os.path.join(WS, ref)):
                problems.append(f'{rid}: enforced_by script {ref} does not exist')
            if kind == 'agent' and not os.path.exists(os.path.join(WS, '.claude', 'agents', ref + '.md')):
                problems.append(f'{rid}: enforced_by agent {ref} does not exist')
            if kind == 'eval':
                evals = os.path.join(WS, 'local', 'harness', 'evals', 'evals.yaml')
                if os.path.exists(evals) and f'id: {ref}\n' not in open(evals).read():
                    problems.append(f'{rid}: enforced_by eval {ref} is not in evals.yaml')
    index = os.path.join(WS, 'local', 'kb', 'index.yaml')
    if os.path.exists(index):
        for node, entry in (yaml.safe_load(open(index)).get('nodes') or {}).items():
            for rid in entry.get('invariants') or []:
                if rid not in rules:
                    problems.append(f'index.yaml node {node}: invariant "{rid}" is not in the registry')
    return rules, problems


def table(rules):
    rows = ['| Rule | Enforced by | Level |', '| --- | --- | --- |']
    order = {'none': 0, 'review only': 1, 'eval': 2, 'mechanical': 3}
    for rid, r in sorted(rules.items(), key=lambda kv: (order[level(kv[1])], kv[0])):
        rows.append(f'| `{rid}`: {r["statement"]} | {"; ".join(str(e) for e in r.get("enforced_by") or ["none"])} | {level(r)} |')
    counts = {}
    for r in rules.values():
        counts[level(r)] = counts.get(level(r), 0) + 1
    rows.append('')
    rows.append(f'{len(rules)} rules: ' + ', '.join(f'{counts.get(k, 0)} {k}' for k in ('mechanical', 'eval', 'review only', 'none')))
    return '\n'.join(rows)


def main():
    args = sys.argv[1:]
    if args == ['--table']:
        print(table(load()))
        return 0
    if args[:1] == ['--show'] and len(args) == 2:
        r = load().get(args[1])
        if not r:
            print(f'no rule {args[1]}')
            return 1
        print(f'{args[1]}: {r["statement"]}\n  owner: {r["owner"]}  ("{r["anchor"]}")')
        for x in r.get('restated_in') or []:
            print(f'  restated in: {x["file"]}  ("{x["anchor"]}")')
        print('  enforced by: ' + '; '.join(str(e) for e in r.get('enforced_by') or ['none']))
        return 0
    if args == ['--check']:
        rules, problems = check()
        if problems:
            print(f'invariants: {len(problems)} problem(s)')
            print('\n'.join('  ' + p for p in problems))
            return 1
        n = sum(1 for r in rules.values() if level(r) == 'mechanical')
        print(f'invariants: {len(rules)} rules, every anchor present; {n} enforced mechanically')
        return 0
    print(__doc__)
    return 2


if __name__ == '__main__':
    sys.exit(main())
