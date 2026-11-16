#!/bin/bash
# Tests of .claude/hooks/hook.py. Hermetic: every case runs against a scratch repository
# (VLM_AV_WS) and a scratch marker directory (VLM_AV_HOOK_TMP); the real workspace is
# only read. One PASS or FAIL line per case, exit 1 on any failure.
#   bash .claude/hooks/tests/run.sh [-v]
HOOK="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/hook.py"
REAL_WS="$(cd "$(dirname "$HOOK")/../.." && pwd)"
VERBOSE=${1:-}
T=$(mktemp -d)
trap 'rm -rf "$T"' EXIT
export VLM_AV_WS="$T/ws" VLM_AV_HOOK_TMP="$T/tmp"
mkdir -p "$VLM_AV_WS" "$VLM_AV_HOOK_TMP"
EM=$(printf '\xe2\x80\x94')
pass=0; fail=0

# case NAME WANT_RC JSON ENTRY [GREP]: run one hook entry, compare the exit code and
# optionally require a pattern in its output
case_() {
    local name=$1 want=$2 json=$3 entry=$4 pat=${5:-}
    local out rc
    out=$(printf '%s' "$json" | python3 "$HOOK" "$entry" 2>&1); rc=$?
    # the pattern '^$' means "no output at all" (grep sees no line in an empty string)
    if [ "$rc" = "$want" ] && { [ -z "$pat" ] || { [ "$pat" = '^$' ] && [ -z "$out" ]; } || printf '%s' "$out" | grep -qE -- "$pat"; }; then
        pass=$((pass + 1)); echo "PASS  $name"
    else
        fail=$((fail + 1)); echo "FAIL  $name (rc=$rc, want $want${pat:+, pattern '$pat'})"; printf '%s\n' "$out" | sed 's/^/      /' | head -12
    fi
    [ -n "$VERBOSE" ] && printf '%s\n' "$out" | sed 's/^/      /' | head -6
    return 0
}
check() {  # check NAME COMMAND...: a plain assertion
    local name=$1; shift
    if "$@" >/dev/null 2>&1; then pass=$((pass + 1)); echo "PASS  $name"; else fail=$((fail + 1)); echo "FAIL  $name"; fi
}
edit() { printf '{"tool_name":"%s","tool_input":{"file_path":"%s","%s":"%s"}}' "$1" "$2" "$3" "$4"; }
bash_() { python3 -c 'import json,sys; print(json.dumps({"tool_name": sys.argv[1], "tool_input": {"command": sys.argv[2]}}))' "$1" "$2"; }

# ---------------------------------------------------------------- scratch workspace
cd "$VLM_AV_WS" || exit 1
git init -q . && git config user.email t@t && git config user.name t
mkdir -p src/pkg/src scripts tools/avkit local/notes local/kb local/plans docs/baselines/lap1 .claude/hooks build/pkg
printf 'local/\nbuild/\n' > .gitignore
echo 'int a;' > src/pkg/src/a.cpp
printf '#!/bin/bash\necho run\n' > scripts/run.sh; chmod +x scripts/run.sh
printf '# sourced helper\nf() { :; }\n' > scripts/_common.sh
# stand-in for the parameter check: fails for a file whose name contains "bad"
printf '#!/bin/bash\ncase "$2" in *bad*) echo "mismatch in $2"; exit 1;; esac\nexit 0\n' > scripts/check_param_sync.sh
printf '# notes\n\n## 1. Status\n\n| Step | State |\n| --- | --- |\n| L0 spike | Done |\n| O1 truck overtake | In progress |\n\n## 2. Other\n\n| x | y |\n| --- | --- |\n| not | status |\n' > local/notes/lidar_implementation.md
printf '# notes\n\n## 1. Status\n\n| Part | State |\n| --- | --- |\n| Day 16 AEB | Built |\n' > local/notes/aeb_status_and_lessons.md
git add -A && git commit -qm init
W="//wsl.localhost/Ubuntu-22.04$VLM_AV_WS"   # how a path arrives from the Windows side

# ---------------------------------------------------------------- post-edit: lint
case_ 'em dash in a shipped file is refused'            2 "$(echo "int b; // x $EM y" >> src/pkg/src/a.cpp; edit Edit "$W/src/pkg/src/a.cpp" new_string 'x')" post-edit 'em dash'
git checkout -q src/pkg/src/a.cpp
case_ 'legacy-free shipped file passes'                 0 "$(edit Edit "$W/src/pkg/src/a.cpp" new_string 'int c;')" post-edit
echo '# per Day 16 of the plan' >> scripts/run.sh
case_ 'plan reference in a shipped file is refused'     2 "$(edit Edit "$W/scripts/run.sh" new_string 'x')" post-edit 'plan reference'
git checkout -q scripts/run.sh
echo '# see L5 task 8' >> scripts/run.sh
case_ 'step wording with a context word is refused'     2 "$(edit Edit "$W/scripts/run.sh" new_string 'x')" post-edit 'plan reference'
git checkout -q scripts/run.sh
echo 'L2 = dx * dx + dy * dy  # squared length' > tools/avkit/geom.py
case_ 'a variable named L2 is not a step reference'     0 "$(edit Write "$W/tools/avkit/geom.py" content 'x')" post-edit
echo "old note $EM kept" > local/notes/old.md
case_ 'legacy em dash in a note is not re-reported'     0 "$(edit Edit "$W/local/notes/old.md" new_string 'a new plain line, see local/plans/day16.md')" post-edit
case_ 'em dash added to a note is refused'              2 "$(edit Edit "$W/local/notes/old.md" new_string "new $EM line")" post-edit 'em dash'
case_ 'a path outside the workspace is ignored'         0 "$(edit Write "C:\\\\Users\\\\x\\\\notes.md" content "a $EM b")" post-edit

# ---------------------------------------------------------------- post-edit: file mode follows git
chmod -x scripts/run.sh
case_ 'edit of a tracked executable'                    0 "$(edit Edit "$W/scripts/run.sh" new_string 'x')" post-edit
check 'mode restored where git records 100755'          test -x scripts/run.sh
case_ 'edit of a tracked sourced file'                  0 "$(edit Edit "$W/scripts/_common.sh" new_string 'x')" post-edit
check 'mode left alone where git records 100644'        test ! -x scripts/_common.sh
printf '#!/usr/bin/env python3\nprint(1)\n' > tools/avkit/new_tool.py
case_ 'new file with a shebang'                         0 "$(edit Write "$W/tools/avkit/new_tool.py" content 'x')" post-edit
check 'new shebang file becomes executable'             test -x tools/avkit/new_tool.py
printf 'x = 1\n' > tools/avkit/module.py
case_ 'new file without a shebang'                      0 "$(edit Write "$W/tools/avkit/module.py" content 'x')" post-edit
check 'new module stays non-executable'                 test ! -x tools/avkit/module.py
check 'no mode change shows in git'                     test -z "$(git diff --summary)"

# ---------------------------------------------------------------- post-edit: parameter check wiring, frontmatter
touch src/pkg/bad_params.yaml
case_ 'parameter mismatch is reported'                  2 "$(edit Edit "$W/src/pkg/bad_params.yaml" new_string 'x')" post-edit 'check_param_sync'
rm src/pkg/bad_params.yaml
mkdir -p .claude/agents
printf -- '---\nname: x\ndescription: says: this\n---\nbody\n' > .claude/agents/x.md
case_ 'frontmatter with an unquoted ": " is refused'    2 "$(edit Write "$W/.claude/agents/x.md" content 'x')" post-edit 'frontmatter'
printf -- '---\nname: x\ndescription: "says: this"\n---\nbody\n' > .claude/agents/x.md
case_ 'valid frontmatter passes'                        0 "$(edit Write "$W/.claude/agents/x.md" content 'x')" post-edit

# ---------------------------------------------------------------- guard-edit (the real rule file)
case_ 'generated page: denied'                          2 "$(edit Edit "$W/local/kb/architecture.html" new_string 'x')" guard-edit 'generated page'
case_ 'baseline session file: denied'                   2 "$(edit Write "$W/docs/baselines/lap1/metrics.json" content 'x')" guard-edit 'baseline'
case_ 'baselines README: allowed'                       0 "$(edit Edit "$W/docs/baselines/README.md" new_string 'x')" guard-edit
case_ 'recorded session: denied'                        2 "$(edit Write "$W/data/av_laps/s1/odom.jsonl" content 'x')" guard-edit 'evidence'
case_ 'driving chain: ask'                              0 "$(edit Edit "$W/src/av_perception_cpp/src/lane_node.cpp" new_string 'x')" guard-edit '"permissionDecision": "ask"'
case_ 'hand-build target: ask'                          0 "$(edit Write "$W/src/av_overtake_cpp/src/overtake_manager.cpp" content 'x')" guard-edit '"permissionDecision": "ask"'
case_ 'ordinary source file: silent'                    0 "$(edit Edit "$W/src/pkg/src/a.cpp" new_string 'x')" guard-edit '^$'

# ---------------------------------------------------------------- guard-bash
case_ 'headless rendering refused'                      2 "$(bash_ Bash 'ign gazebo --headless-rendering x.sdf')" guard-bash
case_ 'buffered echo in a pipe refused'                 2 "$(bash_ Bash 'timeout 10 ros2 topic echo /odom_truth --field pose.pose.position.x | head')" guard-bash
case_ 'unbuffered echo allowed'                         0 "$(bash_ Bash 'PYTHONUNBUFFERED=1 timeout -s INT 10 ros2 topic echo /odom_truth | head')" guard-bash '^$'
case_ 'heredoc through wsl.exe refused'                 2 "$(bash_ Bash 'MSYS_NO_PATHCONV=1 wsl.exe -d Ubuntu-22.04 -- bash -lc "python3 - <<EOF
print(1)
EOF"')" guard-bash 'heredoc'
case_ 'substitution next to wsl bash -c refused'        2 "$(bash_ Bash 'MSYS_NO_PATHCONV=1 wsl.exe -d Ubuntu-22.04 -- bash -lc "echo $(date)"')" guard-bash 'script file'
case_ 'script file form allowed'                        0 "$(bash_ Bash 'MSYS_NO_PATHCONV=1 wsl.exe -d Ubuntu-22.04 -- bash /home/kefan/x_work/x.sh < /dev/null')" guard-bash '^$'
case_ 'posix path without MSYS_NO_PATHCONV refused'     2 "$(bash_ Bash 'wsl.exe -d Ubuntu-22.04 --exec python3 /home/kefan/vlm_av_ws/x.py')" guard-bash 'MSYS_NO_PATHCONV'
case_ 'same command from PowerShell allowed'            0 "$(bash_ PowerShell 'wsl.exe -d Ubuntu-22.04 --exec python3 /home/kefan/vlm_av_ws/x.py')" guard-bash '^$'
case_ 'wsl.exe with only a redirect path allowed'       0 "$(bash_ Bash 'wsl.exe -d Ubuntu-22.04 -- ls < /dev/null')" guard-bash '^$'
case_ 'git commit: ask'                                 0 "$(bash_ Bash 'git commit -m "fix the lint"')" guard-bash '"permissionDecision": "ask"'
case_ 'git -C path push: ask'                           0 "$(bash_ PowerShell 'git -C . push origin master')" guard-bash '"permissionDecision": "ask"'
case_ 'git reset --hard: ask'                           0 "$(bash_ Bash 'git reset --hard HEAD~1')" guard-bash '"permissionDecision": "ask"'
case_ 'commit with a co-author trailer refused'         2 "$(bash_ Bash 'git commit -m "x

Co-Authored-By: Someone <a@b.c>"')" guard-bash 'co-author'
case_ 'read-only git is silent'                         0 "$(bash_ Bash 'git log --oneline -5 && git status --short && git diff --stat')" guard-bash '^$'
case_ 'git show of a commit named push-fix is silent'   0 "$(bash_ Bash 'git log --grep=commit --oneline')" guard-bash '^$'

# ---------------------------------------------------------------- session-start
git add -A; git commit -qm 'tests so far'
out=$(echo '{"session_id":"s1","source":"startup"}' | python3 "$HOOK" session-start)
check 'session start prints the overtake row'           grep -q 'O1 truck overtake' <<<"$out"
check 'session start prints the first track row'        grep -q 'L0 spike' <<<"$out"
check 'session start does not print a later table'      bash -c '! grep -q "not | status" <<<"$1"' _ "$out"
check 'session start prints the WSL rules'              grep -q 'MSYS_NO_PATHCONV=1' <<<"$out"
check 'session stamp written'                           test -s "$VLM_AV_HOOK_TMP/claude_session_s1.json"
check 'first run starts the sync marker at HEAD'        test "$(cat .git/kb_synced_head)" = "$(git rev-parse HEAD)"
before=$(cat "$VLM_AV_HOOK_TMP/claude_session_s1.json")
echo '{"session_id":"s1","source":"compact"}' | python3 "$HOOK" session-start >/dev/null
check 'a compact restart keeps the session stamp'       test "$before" = "$(cat "$VLM_AV_HOOK_TMP/claude_session_s1.json")"

# sync marker: hand commits since the last sync
echo 'int d;' >> src/pkg/src/a.cpp; echo "// note $EM here" >> src/pkg/src/a.cpp
git commit -qam 'hand commit touching code'
out=$(echo '{"session_id":"s2"}' | python3 "$HOOK" session-start)
check 'a code commit since the sync is listed'          grep -q 'SYNC NEEDED: 1 commit' <<<"$out"
check 'the hand commit is linted'                       grep -q 'em dash (src/pkg/src/a.cpp' <<<"$out"
python3 "$HOOK" mark-synced </dev/null >/dev/null
out=$(echo '{"session_id":"s3"}' | python3 "$HOOK" session-start)
check 'mark-synced silences it'                         bash -c '! grep -q "SYNC NEEDED" <<<"$1"' _ "$out"
echo 'doc' > README.md; git add -A; git commit -qm 'docs only'
out=$(echo '{"session_id":"s4"}' | python3 "$HOOK" session-start)
check 'a commit without code moves the marker silently' bash -c '! grep -q "SYNC NEEDED" <<<"$1" && test "$(cat .git/kb_synced_head)" = "$(git rev-parse HEAD)"' _ "$out"
git commit -q --allow-empty -m tmp; git rev-parse HEAD > .git/kb_synced_head; git reset -q --hard HEAD~1
echo 'int e;' >> src/pkg/src/a.cpp; git commit -qam 'after rewrite'
out=$(echo '{"session_id":"s5"}' | python3 "$HOOK" session-start)
check 'a rewritten history asks for a diff-based sync'  grep -q 'no longer an ancestor' <<<"$out"
python3 "$HOOK" mark-synced </dev/null >/dev/null
sed -i "s/ $EM here/ here/" src/pkg/src/a.cpp; git commit -qam 'plain dash'

# ---------------------------------------------------------------- stop-check without a session stamp (old sessions)
echo 'int f;' >> src/pkg/src/a.cpp
touch -d '3 hours ago' local/notes/*.md
case_ 'no stamp: code changed, notes 3 h old'           2 '{"session_id":"n1"}' stop-check 'sync-knowledge-base'
case_ 'no stamp: fires once per session'                0 '{"session_id":"n1"}' stop-check
touch local/notes/lidar_implementation.md
case_ 'no stamp: note touched'                          0 '{"session_id":"n2"}' stop-check
git checkout -q src/pkg/src/a.cpp; touch -d '3 hours ago' local/notes/*.md
case_ 'no stamp: no code change'                        0 '{"session_id":"n3"}' stop-check

# ---------------------------------------------------------------- stop-check with a session stamp
stamp() { printf '{"start": %s, "pids": []}' "$(date -d "$2" +%s)" > "$VLM_AV_HOOK_TMP/claude_session_$1.json"; }
stamp k1 '10 minutes ago'
mkdir -p src/newpkg/src; echo 'int g;' > src/newpkg/src/g.cpp
touch -d '30 minutes ago' local/notes/*.md
case_ 'new untracked package, note older than session'  2 '{"session_id":"k1"}' stop-check 'src/newpkg/src/g.cpp'
stamp k2 '10 minutes ago'; touch local/notes/lidar_implementation.md
case_ 'note written since the session began'            0 '{"session_id":"k2"}' stop-check
stamp k3 '10 minutes ago'; touch -d '30 minutes ago' local/notes/*.md src/newpkg/src/g.cpp
case_ 'code older than the session is not this session' 0 '{"session_id":"k3"}' stop-check
rm -rf src/newpkg

# sweep: a file written through a shell (never seen by post-edit)
stamp w1 '10 minutes ago'; touch local/notes/lidar_implementation.md
printf '#!/bin/bash\n# a %s b\n' "$EM" > scripts/by_shell.sh
case_ 'sweep reports a shell-written file'              2 '{"session_id":"w1"}' stop-check 'scripts/by_shell.sh'
check 'sweep set the mode of the shell-written script'  test -x scripts/by_shell.sh
case_ 'sweep reports once per session'                  0 '{"session_id":"w1"}' stop-check
rm scripts/by_shell.sh

# build freshness
stamp b1 '10 minutes ago'; touch local/notes/lidar_implementation.md
touch -d '5 minutes ago' build/pkg/colcon_build.rc; echo 'int h;' >> src/pkg/src/a.cpp
case_ 'C++ newer than the last build'                   2 '{"session_id":"b1"}' stop-check 'packages-select pkg'
touch build/pkg/colcon_build.rc
stamp b2 '10 minutes ago'
case_ 'built after the change'                          0 '{"session_id":"b2"}' stop-check
git checkout -q src/pkg/src/a.cpp

# leftover simulator processes
stamp p1 '10 minutes ago'
( exec -a 'ign gazebo server -r test.sdf' sleep 30 ) &
SIM=$!; sleep 0.3
case_ 'a simulator started in this session is reported' 2 '{"session_id":"p1"}' stop-check 'stop_av_stack.sh'
case_ 'reported once for the same processes'            0 '{"session_id":"p1"}' stop-check
stamp p2 '10 minutes ago'
case_ 'not reported while a background task runs'       0 '{"session_id":"p2","background_tasks":[{"id":"x"}]}' stop-check
case_ 'subagent stop reports it too'                    2 '{"session_id":"p2"}' subagent-stop 'subagent-stop'
printf '{"start": 0, "pids": ["%s"]}' "$(pgrep -f 'ign gazebo server -r test.sdf' | head -1)" > "$VLM_AV_HOOK_TMP/claude_session_p3.json"
case_ 'a process that predates the session is ignored'  0 '{"session_id":"p3"}' subagent-stop
kill $SIM 2>/dev/null; wait $SIM 2>/dev/null

# ---------------------------------------------------------------- the real workspace (read-only)
check 'hook.py compiles'                                python3 -m py_compile "$HOOK"
check 'protected_paths.yaml parses with rules'          python3 -c "import yaml,sys; r=yaml.safe_load(open('$REAL_WS/.claude/hooks/protected_paths.yaml'))['rules']; sys.exit(0 if len(r) >= 5 and all(x['action'] in ('deny','ask') and x['paths'] and x['reason'] for x in r) else 1)"
check 'settings.json registers every entry point'       python3 -c "
import json,sys
s=open('$REAL_WS/.claude/settings.json').read(); json.loads(s)
sys.exit(0 if all(e in s for e in ('post-edit','guard-edit','guard-bash','session-start','stop-check','subagent-stop')) else 1)"

echo "$pass passed, $fail failed"
[ "$fail" = 0 ]
