"""extract_kb.py - per node: its source files (CMake sources + project headers it includes),
each file's header comment, and every function/method with line, signature and the comment above it."""
import ast
import json
import os
import re
import sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cpp_scan

WS = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
os.chdir(WS)

NODES = {
    # node: (package, [entry sources])
    'lane_node': ('av_perception_cpp', ['src/av_perception_cpp/src/lane_node.cpp']),
    'local_planner': ('av_behavior_cpp', ['src/av_behavior_cpp/src/local_planner.cpp']),
    'mpc_tracker_v2': ('av_behavior_cpp', None),
    'aeb_node': ('av_safety_cpp', ['src/av_safety_cpp/src/aeb_node.cpp']),
    'control_arbiter_node': ('av_control_cpp', ['src/av_control_cpp/src/control_arbiter_node.cpp']),
    'acc_node': ('av_features_cpp', ['src/av_features_cpp/src/acc_node.cpp']),
    'trajectory_coordinator_node': ('av_planning_coordinator', ['src/av_planning_coordinator/src/trajectory_coordinator_node.cpp']),
    'constraint_from_maneuver_node': ('av_planning_coordinator', ['src/av_planning_coordinator/src/constraint_from_maneuver_node.cpp']),
    'odom_noise': ('robot_bringup', ['src/robot_bringup/src/odom_noise.cpp']),
    'odom_compare': ('robot_bringup', ['src/robot_bringup/src/odom_compare.cpp']),
    'vlm_sign_node': ('vlm_planner_py', ['src/vlm_planner_py/vlm_planner_py/vlm_sign_node.py']),
    'sign_maneuver_node': ('vlm_planner_py', ['src/vlm_planner_py/vlm_planner_py/sign_maneuver_node.py']),
    'lead_vehicle': ('tools/avkit', ['tools/avkit/lead_vehicle.py']),
    'mpc_closed_loop_sim': ('av_behavior_cpp', None),
    'obstacle_probe': ('av_safety_cpp', ['src/av_safety_cpp/src/obstacle_probe.cpp']),
}


def cmake_sources(pkg, exe):
    txt = open(f'src/{pkg}/CMakeLists.txt').read()
    m = re.search(r'add_executable\(\s*' + exe + r'\b(.*?)\)', txt, re.S)
    return [f'src/{pkg}/' + s for s in m.group(1).split() if s.endswith(('.cpp', '.hpp'))]


INC_DIRS = ['src/av_common_cpp/include'] + [f'src/{p}/include' for p in os.listdir('src') if os.path.isdir(f'src/{p}/include')]


def resolve_include(name, from_file):
    cand = [os.path.join(os.path.dirname(from_file), name)] + [os.path.join(d, name) for d in INC_DIRS]
    for c in cand:
        if os.path.exists(c):
            return os.path.normpath(c)
    return None


def cpp_closure(entries):
    seen, order = set(), []
    stack = list(entries)
    while stack:
        f = stack.pop(0)
        if f in seen:
            continue
        seen.add(f)
        order.append(f)
        for name in re.findall(r'#include\s*[<"]([^>"]+)[>"]', open(f).read()):
            r = resolve_include(name, f)
            if r:
                stack.append(r)
    return order


def py_closure(entries):
    seen, order, stack = set(), [], list(entries)
    while stack:
        f = stack.pop(0)
        if f in seen:
            continue
        seen.add(f)
        order.append(f)
        d = os.path.dirname(f)
        for m in re.finditer(r'^\s*(?:from\s+(?:\.|vlm_planner_py\.)?(\w+)\s+import|import\s+(\w+))', open(f).read(), re.M):
            mod = m.group(1) or m.group(2)
            c = os.path.join(d, mod + '.py')
            if os.path.exists(c) and c != f:
                stack.append(c)
    return order


def comment_above(lines, i):
    out = []
    j = i - 1
    while j >= 0 and lines[j].strip().startswith(('//', '*', '/*')):
        s = lines[j].strip()
        s = re.sub(r'^(///?|/\*+|\*+/?)\s?', '', s).rstrip('*/ ').strip()
        out.insert(0, s)
        j -= 1
    return ' '.join(x for x in out if x)


def file_header(lines, py=False):
    out = []
    if py:
        return ''
    for ln in lines[:40]:
        s = ln.strip()
        if s.startswith('#') or not s:
            if out and not s:
                break
            continue
        if s.startswith(('//', '/*', '*')):
            s = re.sub(r'^(///?|/\*+|\*+/?)\s?', '', s).rstrip('*/ ').strip()
            out.append(s)
        else:
            break
    return ' '.join(x for x in out if x)


KW = {'if', 'for', 'while', 'switch', 'return', 'catch', 'sizeof', 'else', 'do', 'delete', 'new', 'throw'}
FUNC = re.compile(r'^\s*(?:template\s*<[^>]*>\s*)?((?:[\w:<>,\*&~\[\]]+\s+)*?)([~\w]+(?:::[~\w]+)*)\s*\(([^;{}]*)\)\s*((?:const|override|noexcept|final|\s)*)(?::[^{;]*)?\{?\s*(//.*)?$')


def cpp_functions(path):
    lines = open(path).read().split('\n')
    out = []
    cls_stack = []
    depth = 0
    for i, ln in enumerate(lines):
        code = re.sub(r'//.*', '', ln)
        s = code.strip()
        mcls = re.match(r'^\s*(class|struct)\s+(\w+)[^;]*$', code)
        if mcls and not s.endswith(';'):
            out.append({'kind': mcls.group(1), 'name': mcls.group(2), 'line': i + 1, 'sig': s.rstrip('{ ').strip(), 'doc': comment_above(lines, i)})
        m = FUNC.match(code)
        if m and m.group(2).split('::')[-1] not in KW and not s.endswith(';') and not s.startswith(('#', 'return', 'else', '}')):
            # the body must open on this line or the next non-empty one
            nxt = s.endswith('{') or s.endswith(')') or s.endswith('const') or s.endswith('override')
            k = i + 1
            while k < len(lines) and not lines[k].strip():
                k += 1
            opens = s.endswith('{') or (k < len(lines) and (lines[k].strip().startswith('{') or lines[k].strip().startswith(':')))
            ret = m.group(1).strip()
            name = m.group(2)
            if opens and nxt and (ret or '::' in name or name[0].isupper() or name.startswith('~')):
                if ret in ('', 'else') and not ('::' in name or name[0].isupper() or name.startswith('~')):
                    continue
                if ret.split() and ret.split()[-1] in KW:
                    continue
                sig = (ret + ' ' + name + '(' + ' '.join(m.group(3).split()) + ')').strip()
                out.append({'kind': 'fn', 'name': name, 'line': i + 1, 'sig': sig, 'doc': comment_above(lines, i)})
    return out


def py_functions(path):
    src = open(path).read()
    tree = ast.parse(src)
    out = []

    def doc(n):
        d = ast.get_docstring(n)
        return ' '.join(d.split()) if d else ''

    def comment(n):
        lines = src.split('\n')
        j = n.lineno - 2
        c = []
        while j >= 0 and lines[j].strip().startswith('#'):
            c.insert(0, lines[j].strip().lstrip('#').strip())
            j -= 1
        return ' '.join(c)

    def args(n):
        return ', '.join(a.arg for a in n.args.args)

    for n in tree.body:
        if isinstance(n, ast.ClassDef):
            out.append({'kind': 'class', 'name': n.name, 'line': n.lineno, 'end': n.end_lineno, 'sig': f'class {n.name}', 'doc': doc(n) or comment(n)})
            for m in n.body:
                if isinstance(m, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    out.append({'kind': 'fn', 'name': f'{n.name}.{m.name}', 'line': m.lineno, 'end': m.end_lineno, 'sig': f'def {m.name}({args(m)})', 'doc': doc(m) or comment(m)})
        elif isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)):
            out.append({'kind': 'fn', 'name': n.name, 'line': n.lineno, 'end': n.end_lineno, 'sig': f'def {n.name}({args(n)})', 'doc': doc(n) or comment(n)})
    return out, (ast.get_docstring(tree) or '').strip()


# every other node the code scan finds joins automatically (entry = its CMake sources or its file),
# so a new node appears in the guide without editing this file
sys.path.insert(0, os.path.join(WS, 'tools', 'avkit'))
import kb_index  # noqa: E402
SKIP = {'_course_demo_watch', 'record_aeb_scenario', 'record_av_lap', 'sensor_rates', 'pitch_check', 'aeb_test_driver'}
for name, c in kb_index.scan().items():
    if name in NODES or name in SKIP:
        continue
    pkg, src = c['package'], c['source']
    entries = [src]
    if src.endswith('.cpp') and os.path.exists(f'src/{pkg}/CMakeLists.txt'):
        exe = os.path.splitext(os.path.basename(src))[0]
        if re.search(r'add_executable\(\s*' + re.escape(name) + r'\s', open(f'src/{pkg}/CMakeLists.txt').read()):
            exe = name
        try:
            entries = cmake_sources(pkg, exe) or [src]
        except AttributeError:
            entries = [src]
    NODES[name] = (pkg, entries)

result = {}
files = {}
for node, (pkg, entries) in NODES.items():
    if entries is None:
        exe = node
        entries = cmake_sources(pkg, exe)
    if entries[0].endswith('.py'):
        flist = py_closure(entries)
    else:
        flist = cpp_closure(entries)
    result[node] = {'package': pkg, 'files': flist}
    for f in flist:
        if f in files:
            continue
        lines = open(f).read().split('\n')
        if f.endswith('.py'):
            fns, hdr = py_functions(f)
        else:
            fns, hdr = cpp_scan.scan(f), file_header(lines)
        files[f] = {'loc': len(lines), 'header': hdr, 'items': fns}

json.dump({'nodes': result, 'files': files}, open(os.path.join(HERE, 'extracted.json'), 'w'), indent=1)
print('extracted', len(result), 'nodes,', len(files), 'files')
