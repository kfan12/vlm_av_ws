#!/usr/bin/env python3
"""Claude Code hooks for vlm_av_ws, run inside WSL.

  python3 hook.py post-edit      PostToolUse on Edit|Write|MultiEdit: exec bit, text lint,
                                 cv_bridge check, duplicated-parameter check
  python3 hook.py guard-bash     PreToolUse on Bash|PowerShell: refuse known-bad commands
  python3 hook.py session-start  SessionStart: working tree, status rows, leftover processes
  python3 hook.py stop-check     Stop: once per session, ask for a knowledge-base sync
                                 when code changed and no note did

Reads the hook's JSON on stdin. Exit 2 with a message on stderr is what Claude sees
(a refusal for PreToolUse, a finding for the others); exit 0 is silent.
Paths arrive as Windows UNC paths and are mapped to this workspace.
"""
import json
import os
import re
import subprocess
import sys

WS = os.environ.get('VLM_AV_WS', '/home/kefan/vlm_av_ws')   # overridable for the hook's own tests
EM_DASH = chr(0x2014)
PLAN_REF = re.compile(r'\bDay \d+\b|\bday\d{2}\.md\b|local/plans/')
# shipped files: never a plan reference; these prefixes and names may cite plans
PLAN_OK_PREFIXES = ('local/', 'docs/reports/', '.claude/')
EXEC_FILES = re.compile(r'^(scripts/[^/]+\.(sh|py)|tools/avkit/[^/]+\.py)$')
IMAGE_CODE = re.compile(r'^(src/.*\.(cpp|hpp|py)|tools/.*\.py|scripts/.*\.py)$')


def to_wsl(path):
    if not path:
        return path
    p = path.replace('\\', '/')
    m = re.match(r'^//wsl(\.localhost|\$)/[^/]+(/.*)$', p)
    if m:
        return m.group(2)
    m = re.match(r'^([A-Za-z]):/(.*)$', p)
    if m:
        return f'/mnt/{m.group(1).lower()}/{m.group(2)}'
    if not p.startswith('/'):
        return os.path.join(WS, p)
    return p


def rel(path):
    p = to_wsl(path)
    return os.path.relpath(p, WS) if p and p.startswith(WS + '/') else None


def read_input():
    try:
        raw = sys.stdin.read()
        return json.loads(raw) if raw.strip() else {}
    except (ValueError, OSError):
        return {}


def run(cmd):
    r = subprocess.run(cmd, cwd=WS, capture_output=True, text=True)
    # rstrip only: `git status --porcelain` lines start with a significant space
    return r.returncode, (r.stdout + r.stderr).rstrip()


def edited_texts(data):
    """The text an edit adds (not the whole file), so legacy content never trips the lint."""
    ti = data.get('tool_input', {})
    tool = data.get('tool_name', '')
    if tool == 'Write':
        return [ti.get('content', '')]
    if tool == 'Edit':
        return [ti.get('new_string', '')]
    if tool == 'MultiEdit':
        return [e.get('new_string', '') for e in ti.get('edits', [])]
    return []


def post_edit(data):
    r = rel(data.get('tool_input', {}).get('file_path', ''))
    if not r:
        return
    problems = []
    path = os.path.join(WS, r)

    # 1. executable bit (the Edit/Write tools drop it on the share)
    if EXEC_FILES.match(r) and os.path.exists(path):
        mode = os.stat(path).st_mode
        if not mode & 0o111:
            os.chmod(path, mode | 0o755)

    # 2. text lint on the added text only
    texts = edited_texts(data)
    for text in texts:
        for n, line in enumerate(text.splitlines(), 1):
            if EM_DASH in line:
                problems.append(f'em dash in added text (line {n} of the edit): {line.strip()[:100]}')
            if not r.startswith(PLAN_OK_PREFIXES) and os.path.basename(r) != 'CLAUDE.md' and PLAN_REF.search(line):
                problems.append(f'plan reference in a shipped file ({r}, edit line {n}): {line.strip()[:100]}')

    # 3. no cv_bridge in image code
    if IMAGE_CODE.match(r):
        code, out = run(['bash', 'scripts/check_no_cv_bridge.sh'])
        if code != 0:
            problems.append('scripts/check_no_cv_bridge.sh failed:\n' + out)

    # 4. duplicated parameters
    if os.path.exists(os.path.join(WS, 'scripts/check_param_sync.sh')):
        code, out = run(['bash', 'scripts/check_param_sync.sh', '--if-listed', r])
        if code != 0:
            problems.append('scripts/check_param_sync.sh failed:\n' + out)

    if problems:
        sys.stderr.write(f'[hook post-edit] {r}\n' + '\n'.join(problems) + '\n')
        sys.stderr.write('Fix these before going on (no em dashes; no Day N / dayNN.md / local/plans references in shipped files).\n')
        sys.exit(2)


def guard_bash(data):
    cmd = data.get('tool_input', {}).get('command', '') or ''
    if '--headless-rendering' in cmd:
        refuse('never --headless-rendering on WSL2 (the camera freezes after 40-90 s); use -r, or gz_gui:=false')
    if re.search(r'ros2 topic echo[^|\n]*\|', cmd) and 'PYTHONUNBUFFERED' not in cmd:
        refuse('a piped `ros2 topic echo` is block-buffered: prefix PYTHONUNBUFFERED=1 and use `timeout -s INT N` (wsl-ros-ops skill)')
    if 'wsl.exe' in cmd and '<<' in cmd:
        refuse('a heredoc through wsl.exe gets mangled: write a script file under /home/kefan/<step>_work/ and run that (wsl-ros-ops skill)')
    if 'wsl.exe' in cmd and re.search(r'bash\s+-l?c\s', cmd) and re.search(r'\$\(|\$[A-Za-z_{]', cmd):
        refuse('$VAR / $(...) inside `wsl.exe -- bash -c "..."` is expanded or mangled before WSL sees it: put the logic in a script file (wsl-ros-ops skill)')


def refuse(msg):
    sys.stderr.write('[hook guard-bash] ' + msg + '\n')
    sys.exit(2)


def session_start(data):
    out = []
    code, status = run(['git', 'status', '--short'])
    lines = status.splitlines()
    out.append(f'working tree: {len(lines)} changed file(s)' + (':\n  ' + '\n  '.join(lines[:15]) if lines else ''))
    code, log = run(['git', 'log', '--oneline', '-3'])
    out.append('last commits:\n  ' + log.replace('\n', '\n  '))
    for note, pattern in (('local/notes/lidar_implementation.md', r'^\| L[0-9]'),
                          ('local/notes/aeb_status_and_lessons.md', r'^\| Day 1[6-9]')):
        try:
            with open(os.path.join(WS, note)) as f:
                rows = [ln.rstrip() for ln in f if re.match(pattern, ln)]
            out.append(f'{note} status rows:\n  ' + '\n  '.join(r[:160] for r in rows))
        except OSError:
            out.append(f'{note}: missing')
    code, procs = run(['bash', '-c', "pgrep -af 'ign gazebo|parameter_bridge|lead_vehicle.py|ros2 launch|record_a' | grep -v pgrep || true"])
    out.append('leftover sim processes: ' + ('none' if not procs else '\n  ' + procs.replace('\n', '\n  ')))
    out.append('rules: CLAUDE.md (root + per directory), skills in .claude/skills (wsl-ros-ops for any WSL command).')
    print('\n'.join(out))


def newest_mtime(subdirs, exts=('.md', '.yaml')):
    newest = 0.0
    for sub in subdirs:
        for root, _, files in os.walk(os.path.join(WS, sub)):
            for fn in files:
                if fn.endswith(exts):
                    newest = max(newest, os.stat(os.path.join(root, fn)).st_mtime)
    return newest


def stop_check(data):
    sid = data.get('session_id', 'nosession')
    marker = f'/tmp/claude_stop_check_{sid}'
    if os.path.exists(marker):
        return
    code, status = run(['git', 'status', '--porcelain'])
    changed = [ln[3:] for ln in status.splitlines()]
    code_changed = [p for p in changed if p.startswith(('src/', 'scripts/', 'tools/')) and not p.endswith('CLAUDE.md')]
    # local/ and docs/reports/ are gitignored, so a note edit is invisible to git:
    # a knowledge-base file written after the newest changed code file counts as a sync
    code_mtime = max((os.stat(os.path.join(WS, p)).st_mtime for p in code_changed if os.path.exists(os.path.join(WS, p))), default=0.0)
    kb_mtime = newest_mtime(('local/notes', 'local/kb', 'local/plans', 'docs/reports'))
    kb_changed = kb_mtime >= code_mtime - 3600
    if code_changed and not kb_changed:
        open(marker, 'w').close()
        sys.stderr.write('[hook stop-check] code changed (' + ', '.join(code_changed[:6]) +
                         ') but no note, plan, kb or report did. Run the sync-knowledge-base skill, '
                         'or say in one line why no knowledge-base change is needed, then stop.\n')
        sys.exit(2)


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else ''
    data = read_input()
    {'post-edit': post_edit, 'guard-bash': guard_bash,
     'session-start': session_start, 'stop-check': stop_check}.get(cmd, lambda d: None)(data)


if __name__ == '__main__':
    main()
