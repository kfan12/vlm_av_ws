#!/usr/bin/env python3
"""Claude Code hooks for vlm_av_ws, run inside WSL.

  python3 hook.py post-edit      PostToolUse on Edit|Write|MultiEdit: file mode, text lint,
                                 cv_bridge check, duplicated-parameter check, frontmatter
  python3 hook.py guard-edit     PreToolUse on Edit|Write|MultiEdit: protected paths
                                 (protected_paths.yaml: deny or ask, each with its reason)
  python3 hook.py guard-bash     PreToolUse on Bash|PowerShell: refuse known-bad commands,
                                 ask before git commands that write history
  python3 hook.py session-start  SessionStart: working tree, status tables, leftover processes,
                                 the WSL operating rules, the code commits since the last
                                 knowledge-base sync; writes the session stamp
  python3 hook.py mark-synced    by hand, at the end of a sync: the knowledge base matches HEAD
  python3 hook.py stop-check     Stop: rebuild the generated pages, sweep the files this
                                 session changed (mode, lint), report simulator processes the
                                 session left behind, ask for a build and a knowledge-base sync
  python3 hook.py subagent-stop  SubagentStop: simulator processes left behind

Reads the hook's JSON on stdin. Exit 2 with a message on stderr is what Claude sees
(a refusal for PreToolUse, a finding for the others); exit 0 is silent. An "ask" is JSON
on stdout with exit 0. Paths arrive as Windows UNC paths and are mapped to this workspace.
Tests: bash .claude/hooks/tests/run.sh
"""
import fnmatch
import hashlib
import json
import os
import re
import subprocess
import sys
import time

# the workspace is two levels above this file (.claude/hooks/hook.py); overridable for the hook's own tests
WS = os.environ.get('VLM_AV_WS', os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
HERE = os.path.dirname(os.path.abspath(__file__))
TMP = os.environ.get('VLM_AV_HOOK_TMP', '/tmp')
EM_DASH = chr(0x2014)
PLAN_REF = re.compile(r'\bDay \d+\b|\bday\d{2}\.md\b|local/plans/')
# plan step names (the lidar and overtake tracks) only with a context word: a bare L2 is a variable
STEP_REF = re.compile(r'\b[LO]\d (?:tasks?|steps?|plan|track|matrix|accept\w*|spike|runs?)\b'
                      r'|\b(?:task|step) \d+ of [LO]\d\b')
# shipped files: never a plan reference; these prefixes and names may cite plans
PLAN_OK_PREFIXES = ('local/', 'docs/', '.claude/')
IMAGE_CODE = re.compile(r'^(src/.*\.(cpp|hpp|py)|tools/.*\.py|scripts/.*\.py)$')
TEXT_EXT = ('.cpp', '.hpp', '.h', '.py', '.sh', '.yaml', '.yml', '.md', '.xacro', '.txt', '.xml',
            '.launch', '.sdf', '.json', '.rviz', '.cmake', '.msg', '.ps1')
CODE_PREFIXES = ('src/', 'scripts/', 'tools/')
KB_DIRS = ('local/notes', 'local/kb', 'local/plans', 'local/reports', 'local/research_notes', 'docs/reports')
SIM_PROCS = 'ign gazebo|parameter_bridge|lead_vehicle.py|ros2 launch|record_a'


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


def once(kind, sid, text):
    """True the first time this (session, report) pair is seen. A stable SHA-1, because
    Python's hash() changes per process."""
    marker = os.path.join(TMP, f'claude_{kind}_{sid}_{hashlib.sha1(text.encode()).hexdigest()[:12]}')
    if os.path.exists(marker):
        return False
    open(marker, 'w').close()
    return True


# ---------------------------------------------------------------- session stamp

def stamp_file(sid):
    return os.path.join(TMP, f'claude_session_{sid}.json')


def read_stamp(sid):
    try:
        with open(stamp_file(sid)) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def sim_pids():
    """{pid: command line} of simulator and stack processes."""
    code, out = run(['bash', '-c', f"pgrep -af '{SIM_PROCS}' | grep -v pgrep || true"])
    procs = {}
    for ln in out.splitlines():
        pid, _, cmdline = ln.strip().partition(' ')
        if pid.isdigit():
            procs[pid] = cmdline
    return procs


# ---------------------------------------------------------------- lint and file mode

def is_shipped(r):
    return not r.startswith(PLAN_OK_PREFIXES) and os.path.basename(r) != 'CLAUDE.md'


def lint_lines(r, lines, where):
    problems = []
    shipped = is_shipped(r)
    for n, line in lines:
        if EM_DASH in line:
            problems.append(f'em dash ({r}, {where} {n}): {line.strip()[:100]}')
        if shipped and (PLAN_REF.search(line) or STEP_REF.search(line)):
            problems.append(f'plan reference in a shipped file ({r}, {where} {n}): {line.strip()[:100]}')
    return problems


def lint_file(r, added_texts=None):
    """A shipped file is checked whole (the tree carries no legacy em dash or plan reference);
    any other file only on the text an edit adds, so old notes never trip the lint."""
    path = os.path.join(WS, r)
    if is_shipped(r) or added_texts is None:
        if not r.endswith(TEXT_EXT) or not os.path.isfile(path) or os.path.getsize(path) > 2_000_000:
            return []
        try:
            with open(path, encoding='utf-8') as f:
                return lint_lines(r, enumerate(f.read().splitlines(), 1), 'line')[:12]
        except (OSError, UnicodeDecodeError):
            return []
    problems = []
    for text in added_texts:
        problems += lint_lines(r, enumerate(text.splitlines(), 1), 'line of the edit')
    return problems[:12]


def fix_mode(r):
    """The Edit and Write tools drop the executable bit on the share. Restore what git
    records; a file git does not know gets +x only when it starts with a shebang."""
    path = os.path.join(WS, r)
    if not os.path.isfile(path):
        return
    mode = os.stat(path).st_mode
    if mode & 0o111:
        return
    code, out = run(['git', 'ls-files', '-s', '--', r])
    if code == 0 and out:
        want = out.split()[0] == '100755'
    else:
        try:
            with open(path, 'rb') as f:
                want = f.read(2) == b'#!'
        except OSError:
            want = False
    if want:
        os.chmod(path, mode | 0o755)


def edited_texts(data):
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
    path = os.path.join(WS, r)
    fix_mode(r)
    problems = lint_file(r, edited_texts(data))

    if IMAGE_CODE.match(r) and os.path.exists(os.path.join(WS, 'scripts/check_no_cv_bridge.sh')):
        code, out = run(['bash', 'scripts/check_no_cv_bridge.sh'])
        if code != 0:
            problems.append('scripts/check_no_cv_bridge.sh failed:\n' + out)

    if os.path.exists(os.path.join(WS, 'scripts/check_param_sync.sh')):
        code, out = run(['bash', 'scripts/check_param_sync.sh', '--if-listed', r])
        if code != 0:
            problems.append('scripts/check_param_sync.sh failed:\n' + out)

    # agent and skill frontmatter must be valid YAML (GitHub's preview and
    # strict parsers reject an unquoted value containing ": ")
    if re.match(r'^\.claude/(agents/[^/]+|skills/[^/]+/SKILL)\.md$', r) and os.path.exists(path):
        problems += frontmatter_problems(path)

    if problems:
        sys.stderr.write(f'[hook post-edit] {r}\n' + '\n'.join(problems) + '\n')
        sys.stderr.write('Fix these before going on (no em dashes; no plan-step references in shipped files).\n')
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


# ---------------------------------------------------------------- PreToolUse

def ask(tag, msg):
    """Hand the decision to the user: the permission prompt shows the reason."""
    print(json.dumps({'hookSpecificOutput': {'hookEventName': 'PreToolUse', 'permissionDecision': 'ask',
                                             'permissionDecisionReason': f'[hook {tag}] {msg}'}}))
    sys.exit(0)


def refuse(msg, tag='guard-bash'):
    sys.stderr.write(f'[hook {tag}] {msg}\n')
    sys.exit(2)


def protected_rules():
    try:
        import yaml
        with open(os.path.join(HERE, 'protected_paths.yaml')) as f:
            return (yaml.safe_load(f) or {}).get('rules', [])
    except (ImportError, OSError, ValueError):
        return []


def guard_edit(data):
    r = rel(data.get('tool_input', {}).get('file_path', ''))
    if not r:
        return
    for rule in protected_rules():
        if any(fnmatch.fnmatch(r, pat) for pat in rule.get('paths', [])):
            msg = f'{r}: {rule.get("reason", "protected path")}'
            if rule.get('action') == 'deny':
                refuse(msg, 'guard-edit')
            ask('guard-edit', msg)


GIT_WRITE = re.compile(r'\bgit\b[^|;&\n]*?\s(commit|push|rebase|filter-repo|filter-branch|reset\s+--hard)(\s|$)')


def guard_bash(data):
    cmd = data.get('tool_input', {}).get('command', '') or ''
    if '--headless-rendering' in cmd:
        refuse('never --headless-rendering on WSL2 (the camera freezes after 40-90 s); use -r, or gz_gui:=false')
    if re.search(r'ros2 topic echo[^|\n]*\|', cmd) and 'PYTHONUNBUFFERED' not in cmd:
        refuse('a piped `ros2 topic echo` is block-buffered: prefix PYTHONUNBUFFERED=1 and use `timeout -s INT N`, '
               'or use `python3 tools/avkit/ros_probe.py field TOPIC FIELD --for N`')
    if 'wsl.exe' in cmd and '<<' in cmd:
        refuse('a heredoc through wsl.exe gets mangled: write a script file under /home/kefan/<step>_work/ and run that (wsl-ros-ops skill)')
    if 'wsl.exe' in cmd and re.search(r'bash\s+-l?c\s', cmd) and re.search(r'\$\(|\$[A-Za-z_{]', cmd):
        refuse('$VAR / $(...) in a command that also runs `wsl.exe -- bash -c "..."` is expanded or mangled before WSL '
               'sees it (this rule cannot tell a Windows-side substitution from one meant for WSL): put the whole '
               'command in a script file and run `wsl.exe -d Ubuntu-22.04 -- bash /home/kefan/<step>_work/x.sh`')
    if (data.get('tool_name') == 'Bash' and 'MSYS_NO_PATHCONV' not in cmd
            and re.search(r'wsl\.exe\b[^|;&<>\n]*\s/[A-Za-z]', cmd)):
        refuse('Git Bash rewrites a /posix/path argument of wsl.exe into C:/Program Files/Git/...: '
               'prefix the command with MSYS_NO_PATHCONV=1')
    m = GIT_WRITE.search(cmd)
    if m:
        if re.search(r'co-authored-by', cmd, re.I):
            refuse('no agent co-author line in a commit message (global rule); remove the Co-Authored-By trailer')
        ask('git-guard', f'`git {" ".join(m.group(1).split())}`: the user commits and pushes by hand in this '
            'repository. Approve only if you asked for it; before anything that rewrites history, fetch origin first.')
    if re.search(r'\bgit\b[^|;&\n]*\spush\b[^|;&\n]*(--force|\s-f\b)', cmd):
        ask('git-guard', 'forced push')


# ---------------------------------------------------------------- SessionStart

def status_rows(note):
    """Every row of the first table after the note's section 1 heading: no step-name
    pattern, so a new track shows up without a hook change."""
    with open(os.path.join(WS, note)) as f:
        lines = f.read().splitlines()
    start = next((i for i, ln in enumerate(lines) if re.match(r'^## 1\b', ln)), 0)
    rows, in_table = [], False
    for ln in lines[start:]:
        if ln.startswith('|'):
            in_table = True
            if not re.match(r'^\|\s*-{3}', ln):
                rows.append(ln.rstrip())
        elif in_table:
            break
    return rows[1:]  # without the header row


WSL_RULES = [
    'WSL operating rules (wsl-ros-ops skill; a subagent gets them from here):',
    '  - one-liners: MSYS_NO_PATHCONV=1 wsl.exe -d Ubuntu-22.04 -- bash -lc "cd ~/vlm_av_ws && CMD" < /dev/null; '
    'anything with quotes, $, pipes or several lines goes into a script file under /home/kefan/<step>_work/',
    '  - every script sources /opt/ros/humble/setup.bash and install/setup.bash; PYTHONNOUSERSITE=1 for cv2; CYCLONEDDS_URI unset',
    '  - long runs: one foreground call per run with a log file (a subagent), or run_in_background (main session); look at the plot, not only the metric',
    '  - leftovers: bash scripts/stop_av_stack.sh; never start a GUI Gazebo unattended (gz_gui:=false); do not commit, the user does',
]


def session_start(data):
    sid = data.get('session_id', 'nosession')
    procs = sim_pids()
    if not (data.get('source') in ('resume', 'compact') and read_stamp(sid)):
        try:
            with open(stamp_file(sid), 'w') as f:
                json.dump({'start': time.time(), 'pids': sorted(procs)}, f)
        except OSError:
            pass
    out = []
    code, status = run(['git', 'status', '--short'])
    lines = status.splitlines()
    out.append(f'working tree: {len(lines)} changed file(s)' + (':\n  ' + '\n  '.join(lines[:15]) if lines else ''))
    code, log = run(['git', 'log', '--oneline', '-3'])
    out.append('last commits:\n  ' + log.replace('\n', '\n  '))
    for note in ('local/notes/lidar_implementation.md', 'local/notes/aeb_status_and_lessons.md'):
        try:
            out.append(f'{note} status rows:\n  ' + '\n  '.join(r[:160] for r in status_rows(note)))
        except OSError:
            out.append(f'{note}: missing')
    out.append('leftover sim processes: ' + ('none' if not procs else
                                             '\n  ' + '\n  '.join(f'{p} {c}' for p, c in procs.items())))
    out.append('rules: CLAUDE.md (root + per directory), skills in .claude/skills.')
    out += WSL_RULES
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


def commit_lint(last, now):
    """The same lint over the files the hand commits touched (they never pass post-edit)."""
    code, names = run(['git', 'diff', '--name-only', '--diff-filter=AM', last, now, '--', *SYNC_PATHS])
    if code != 0:
        return ''
    problems = []
    for r in names.splitlines():
        problems += lint_file(r)
    return ('\nLint findings in those commits (report them, fix only if asked):\n  ' +
            '\n  '.join(problems[:15])) if problems else ''


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
            f'\nBefore other work, run the sync-knowledge-base skill for `git log -p {last[:7]}..HEAD -- {" ".join(SYNC_PATHS)}` '
            '(the check-hand-build skill when the commits are a plan task built by hand), '
            'then `python3 .claude/hooks/hook.py mark-synced`. Commit nothing without asking.' + commit_lint(last, now))


# ---------------------------------------------------------------- Stop

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
        if not os.path.exists(os.path.join(WS, cmd[1])):
            continue
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
    if once('kb_pages', sid, msg):
        sys.stderr.write('[hook stop-check] a knowledge-base page was rebuilt and no longer matches the code:\n' + msg +
                         '\nFix layout.py / descriptions.py (sync-knowledge-base skill, item 4), rebuild, then stop.\n')
        sys.exit(2)


def changed_files():
    """Working-tree changes, one path per file (-uall: a new package is not one directory line)."""
    code, status = run(['git', 'status', '--porcelain', '-uall'])
    out = []
    for ln in status.splitlines():
        p = ln[3:].split(' -> ')[-1].strip('"')
        if os.path.isfile(os.path.join(WS, p)):
            out.append(p)
    return out


def leftovers(data, sid, tag):
    """Simulator processes that were not there when the session started. Skipped while a
    background task of this session is still running (a lap in progress is not an orphan)."""
    stamp = read_stamp(sid)
    if stamp is None or data.get('background_tasks'):
        return
    new = {p: c for p, c in sim_pids().items() if p not in set(stamp.get('pids', []))}
    if not new:
        return
    msg = '\n  '.join(f'{p} {c[:140]}' for p, c in sorted(new.items()))
    if once('leftovers', sid, ' '.join(sorted(new))):
        sys.stderr.write(f'[hook {tag}] simulator processes started in this session are still running:\n  {msg}\n'
                         'Stop them with `bash scripts/stop_av_stack.sh`, or say in one line why they must stay '
                         '(a demo the user is watching), then stop.\n')
        sys.exit(2)


def hand_build_paths():
    return [pat for rule in protected_rules() if rule.get('hand_build') for pat in rule.get('paths', [])]


def unbuilt_packages(code_files):
    """Packages whose C++ changed after their last colcon build (build/<pkg>/colcon_build.rc)."""
    skip = hand_build_paths()
    stale = set()
    for p in code_files:
        m = re.match(r'^src/([^/]+)/.*\.(cpp|hpp|h)$', p)
        if not m or any(fnmatch.fnmatch(p, pat) for pat in skip):
            continue
        rc = os.path.join(WS, 'build', m.group(1), 'colcon_build.rc')
        if os.path.exists(rc) and os.stat(os.path.join(WS, p)).st_mtime > os.stat(rc).st_mtime:
            stale.add(m.group(1))
    return sorted(stale)


def stop_check(data):
    sid = data.get('session_id', 'nosession')
    kb_pages(sid)
    leftovers(data, sid, 'stop-check')
    stamp = read_stamp(sid)
    start = stamp['start'] if stamp else 0.0
    mtime = lambda p: os.stat(os.path.join(WS, p)).st_mtime
    session_files = [p for p in changed_files() if mtime(p) >= start]

    # sweep: whatever tool wrote the file (Edit, a shell redirect, a subagent), same mode and lint rules
    problems = []
    for p in session_files:
        fix_mode(p)
        code, diff = run(['git', 'diff', '-U0', 'HEAD', '--', p])
        added = [ln[1:] for ln in diff.splitlines() if ln.startswith('+') and not ln.startswith('+++')]
        problems += lint_file(p, ['\n'.join(added)] if added else None)
    if problems and once('sweep', sid, '\n'.join(problems)):
        sys.stderr.write('[hook stop-check] files changed in this session break the text rules:\n  ' +
                         '\n  '.join(problems[:20]) + '\nFix them (or say why a line must stay), then stop.\n')
        sys.exit(2)

    code_changed = [p for p in session_files if p.startswith(CODE_PREFIXES) and not p.endswith('CLAUDE.md')]
    stale = unbuilt_packages(code_changed)
    if stale and once('build', sid, ' '.join(stale)):
        sys.stderr.write('[hook stop-check] C++ changed after the last build of: ' + ', '.join(stale) +
                         '. Run `colcon build --symlink-install --packages-select ' + ' '.join(stale) +
                         '` and read the warnings, or say in one line why no build is needed, then stop.\n')
        sys.exit(2)

    marker = os.path.join(TMP, f'claude_stop_check_{sid}')
    if os.path.exists(marker) or not code_changed:
        return
    # local/ and docs/reports/ are gitignored, so a note edit is invisible to git. With a session
    # stamp: a knowledge-base file written since the session began is the sync. Without one (an
    # old session): a file written within an hour before the newest code change, or after it.
    kb_mtime = newest_mtime(KB_DIRS)
    kb_changed = kb_mtime >= start if stamp else kb_mtime >= max(mtime(p) for p in code_changed) - 3600
    if not kb_changed:
        open(marker, 'w').close()
        sys.stderr.write('[hook stop-check] code changed (' + ', '.join(code_changed[:6]) +
                         ') but no note, plan, kb or report did. Run the sync-knowledge-base skill, '
                         'or say in one line why no knowledge-base change is needed, then stop.\n')
        sys.exit(2)


def subagent_stop(data):
    leftovers(data, data.get('session_id', 'nosession'), 'subagent-stop')


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else ''
    data = read_input()
    {'post-edit': post_edit, 'guard-edit': guard_edit, 'guard-bash': guard_bash,
     'session-start': session_start, 'stop-check': stop_check, 'subagent-stop': subagent_stop,
     'mark-synced': mark_synced}.get(cmd, lambda d: None)(data)


if __name__ == '__main__':
    main()
