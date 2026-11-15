#!/usr/bin/env python3
"""Claude Code hooks for vlm_av_ws, run inside WSL.

  python3 hook.py post-edit      PostToolUse on Edit|Write|MultiEdit: exec bit, text lint,
                                 cv_bridge check, duplicated-parameter check
  python3 hook.py guard-bash     PreToolUse on Bash|PowerShell: refuse known-bad commands
  python3 hook.py session-start  SessionStart: working tree, status rows, leftover processes,
                                 and the code commits since the last knowledge-base sync
  python3 hook.py mark-synced    by hand, at the end of a sync: the knowledge base matches HEAD
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

# the workspace is two levels above this file (.claude/hooks/hook.py); overridable for the hook's own tests
WS = os.environ.get('VLM_AV_WS', os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
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

    # 5. agent and skill frontmatter must be valid YAML (GitHub's preview and
    # strict parsers reject an unquoted value containing ": ")
    if re.match(r'^\.claude/(agents/[^/]+|skills/[^/]+/SKILL)\.md$', r) and os.path.exists(path):
        problems += frontmatter_problems(path)

    if problems:
        sys.stderr.write(f'[hook post-edit] {r}\n' + '\n'.join(problems) + '\n')
        sys.stderr.write('Fix these before going on (no em dashes; no Day N / dayNN.md / local/plans references in shipped files).\n')
        sys.exit(2)


def frontmatter_problems(path):
    text = open(path).read()
    if not text.startswith('---\n') or '\n---\n' not in text[4:]:
        return ['no YAML frontmatter (--- ... ---) at the top']
    head = text[4:].split('\n---\n', 1)[0]
    try:
        import yaml
    except ImportError:
        return []
    try:
        data = yaml.safe_load(head)
    except yaml.YAMLError as e:
        return ['frontmatter is not valid YAML (quote a value that contains ": "): ' + str(e).splitlines()[0]]
    if not isinstance(data, dict) or not {'name', 'description'} <= set(data):
        return ['frontmatter needs name and description']
    return []


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
    pending = unsynced_commits()
    if pending:
        out.append(pending)
    print('\n'.join(out))


# the commit the knowledge base was last synced to; inside .git, so it is never tracked
SYNC_PATHS = ['src', 'scripts', 'tools']


def synced_head_file():
    code, gitdir = run(['git', 'rev-parse', '--absolute-git-dir'])
    return os.path.join(gitdir, 'kb_synced_head') if code == 0 else None


def head():
    code, sha = run(['git', 'rev-parse', 'HEAD'])
    return sha if code == 0 else None


def mark_synced(data=None, sha=None):
    path, sha = synced_head_file(), sha or head()
    if path and sha:
        with open(path, 'w') as f:
            f.write(sha + '\n')
        print(f'knowledge base marked as synced to {sha[:7]}')


def unsynced_commits():
    """Commits since the last knowledge-base sync that touched code (the user commits by
    hand between sessions). Empty when in sync; the marker moves on silently when the new
    commits touched no code. The first run starts the marker at HEAD."""
    path, now = synced_head_file(), head()
    if not path or not now:
        return ''
    try:
        last = open(path).read().strip()
    except OSError:
        mark_synced(sha=now)
        return ''
    if last == now:
        return ''
    if run(['git', 'merge-base', '--is-ancestor', last, now])[0] != 0:
        return (f'KNOWLEDGE-BASE SYNC NEEDED: the last synced commit {last[:7]} is no longer an ancestor of HEAD '
                f'(history was rewritten). Compare `git diff {last[:7]} HEAD --stat -- {" ".join(SYNC_PATHS)}`, '
                'run the sync-knowledge-base skill for what changed, then `python3 .claude/hooks/hook.py mark-synced`.')
    code, log = run(['git', 'log', '--oneline', '--no-merges', f'{last}..{now}', '--', *SYNC_PATHS])
    if code != 0:
        return ''
    if not log:
        mark_synced(sha=now)
        return ''
    commits = log.splitlines()
    return ('KNOWLEDGE-BASE SYNC NEEDED: ' + str(len(commits)) + f' commit(s) since the last sync ({last[:7]}) touched code:\n  ' +
            '\n  '.join(commits[:20]) +
            f'\nBefore other work, run the sync-knowledge-base skill for `git log -p {last[:7]}..HEAD -- {" ".join(SYNC_PATHS)}`, '
            'then `python3 .claude/hooks/hook.py mark-synced`. Commit nothing without asking.')


def newest_mtime(subdirs, exts=('.md', '.yaml')):
    newest = 0.0
    for sub in subdirs:
        for root, _, files in os.walk(os.path.join(WS, sub)):
            for fn in files:
                if fn.endswith(exts):
                    newest = max(newest, os.stat(os.path.join(root, fn)).st_mtime)
    return newest


# the two generated knowledge-base pages, their build, and what they are built from
KB_PAGES = [
    ('local/kb/architecture.html', ['python3', 'local/kb/architecture/build.py'],
     ['src', 'tools/avkit', 'scripts', 'local/kb/index.yaml', 'local/kb/architecture']),
    ('local/kb/source_guide.html', ['bash', 'local/kb/source_guide/build.sh'],
     ['src', 'tools/avkit', 'scripts', 'local/kb/source_guide']),
]
KB_SRC_EXT = ('.cpp', '.hpp', '.h', '.py', '.yaml', '.txt', '.html', '.sh', '.xacro')


def newest_source(paths):
    newest = 0.0
    for rel_path in paths:
        p = os.path.join(WS, rel_path)
        if os.path.isfile(p):
            newest = max(newest, os.stat(p).st_mtime)
            continue
        for root, dirs, files in os.walk(p):
            dirs[:] = [d for d in dirs if d not in ('__pycache__', 'build', 'install', 'log')]
            for fn in files:
                if fn.endswith(KB_SRC_EXT) and fn != 'extracted.json':
                    newest = max(newest, os.stat(os.path.join(root, fn)).st_mtime)
    return newest


def kb_pages(sid):
    """Rebuild a generated page when anything it is built from is newer than it. A build that
    reports a gap (an edge with no route, a function without a description, ...) is shown to
    Claude once per session per distinct report."""
    reports = []
    for page, cmd, sources in KB_PAGES:
        out = os.path.join(WS, page)
        if os.path.exists(out) and os.stat(out).st_mtime >= newest_source(sources):
            continue
        try:
            r = subprocess.run(cmd, cwd=WS, capture_output=True, text=True, timeout=40)
            code, text = r.returncode, (r.stdout + r.stderr).strip()
        except subprocess.TimeoutExpired:
            code, text = 1, 'build timed out'
        if code != 0:
            lines = [ln for ln in text.splitlines() if ln.startswith('  ') or 'problem' in ln or 'Error' in ln]
            reports.append(f'{page} ({" ".join(cmd)}):\n' + '\n'.join(lines[-25:] or text.splitlines()[-10:]))
    if not reports:
        return
    msg = '\n'.join(reports)
    import hashlib
    marker = f'/tmp/claude_kb_pages_{sid}_{hashlib.sha1(msg.encode()).hexdigest()[:12]}'
    if os.path.exists(marker):
        return
    open(marker, 'w').close()
    sys.stderr.write('[hook stop-check] a knowledge-base page was rebuilt and no longer matches the code:\n' + msg +
                     '\nFix layout.py / descriptions.py (sync-knowledge-base skill, item 4), rebuild, then stop.\n')
    sys.exit(2)


def stop_check(data):
    sid = data.get('session_id', 'nosession')
    kb_pages(sid)
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
     'session-start': session_start, 'stop-check': stop_check,
     'mark-synced': mark_synced}.get(cmd, lambda d: None)(data)


if __name__ == '__main__':
    main()
