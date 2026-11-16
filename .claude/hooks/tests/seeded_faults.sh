#!/bin/bash
# seeded_faults.sh - every checker must catch the fault it exists for.
# A checker that has only ever run on a clean tree proves nothing; here each one is
# pointed at a scratch copy of the workspace with one known fault planted, and must
# fail there (and pass on the clean copy first).
#   bash .claude/hooks/tests/seeded_faults.sh
# The real workspace is only read. No model, no simulator. Exit 1 if a fault goes unnoticed.
REAL="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
T=$(mktemp -d)
trap 'rm -rf "$T"' EXIT
C="$T/ws"
mkdir -p "$C"
# the tree the checkers read: sources, tooling, documents, the harness; no build output, no recordings
( cd "$REAL" && tar cf - --exclude=./build --exclude=./install --exclude=./log --exclude=./data \
    --exclude=./.git --exclude='__pycache__' --exclude=./local/harness/evals/runs . ) | tar xf - -C "$C"
export VLM_AV_WS="$C" PYTHONNOUSERSITE=1
cd "$C" || exit 1
EM=$(printf '\xe2\x80\x94')
pass=0; fail=0

must() {  # must pass|fail NAME COMMAND...
    local want=$1 name=$2 rc; shift 2
    "$@" > "$T/out.txt" 2>&1; rc=$?
    if { [ "$want" = pass ] && [ "$rc" = 0 ]; } || { [ "$want" = fail ] && [ "$rc" != 0 ]; }; then
        pass=$((pass + 1)); echo "PASS  $name"
    else
        fail=$((fail + 1)); echo "FAIL  $name (exit $rc, wanted to $want)"; tail -6 "$T/out.txt" | sed 's/^/        /'
    fi
}
restore() { ( cd "$REAL" && tar cf - "$@" ) | tar xf - -C "$C"; }
param_sync() { python3 tools/avkit/param_sync.py "$@"; }
[ -f tools/avkit/param_sync.py ] && grep -q "VLM_AV_WS" tools/avkit/param_sync.py || PARAM_SYNC_FIXED_WS=1

echo "== clean copy: every checker passes"
must pass "clean: reference lint"      python3 .claude/hooks/ref_lint.py
must pass "clean: invariant registry"  python3 .claude/hooks/invariants.py --check
must pass "clean: node index"          python3 tools/avkit/kb_index.py --check
must pass "clean: cv_bridge check"     bash scripts/check_no_cv_bridge.sh
[ -z "$PARAM_SYNC_FIXED_WS" ] && must pass "clean: parameter sync" param_sync

echo "== seeded faults: each must be caught"
# 1. a skill names a script that does not exist
echo 'Stop leftovers with `scripts/stop_everything.sh`.' >> .claude/skills/run-av-stack/SKILL.md
must fail "a skill names a missing script"                 python3 .claude/hooks/ref_lint.py
restore .claude/skills/run-av-stack/SKILL.md
# 2. a skill claims a report is still to be written although it exists
echo 'See `docs/reports/lidar_acc.md` once it exists.' >> .claude/skills/run-scenario/SKILL.md
must fail "a skill says an existing report does not exist" python3 .claude/hooks/ref_lint.py
restore .claude/skills/run-scenario/SKILL.md
# 3. a skill names a launch argument that was renamed
echo 'Launch with `gz_render:=gpu`.' >> .claude/skills/run-av-stack/SKILL.md
must fail "a skill names an undeclared launch argument"    python3 .claude/hooks/ref_lint.py
restore .claude/skills/run-av-stack/SKILL.md
# 4. a rule is reworded away in a file that restates it (the decision-versus-rule drift)
sed -i 's/ACC only lowers speed/ACC shapes speed/' src/av_features_cpp/CLAUDE.md
must fail "a restated rule vanished from its file"         python3 .claude/hooks/invariants.py --check
restore src/av_features_cpp/CLAUDE.md
# 5. a node gains a subscription the index does not know
sed -i '0,/create_subscription</s//create_subscription<std_msgs::msg::String>("\/seeded\/topic", 10, [](std_msgs::msg::String::ConstSharedPtr){});\n        sub_x_ = create_subscription</' src/av_control_cpp/src/control_arbiter_node.cpp
must fail "the index lags a new subscription"              python3 tools/avkit/kb_index.py --check
restore src/av_control_cpp/src/control_arbiter_node.cpp
# 6. a report nobody names
mkdir -p docs/reports; echo '# orphan' > docs/reports/seeded_orphan.md
must fail "a report named by no index entry"               python3 tools/avkit/kb_index.py --check
rm docs/reports/seeded_orphan.md
# 7. cv_bridge in a package the old check never scanned
echo '#include <cv_bridge/cv_bridge.h>' >> src/av_features_cpp/src/acc_node.cpp
must fail "cv_bridge in a feature package"                 bash scripts/check_no_cv_bridge.sh
restore src/av_features_cpp/src/acc_node.cpp
# 8. one copy of a duplicated extrinsic moved
if [ -z "$PARAM_SYNC_FIXED_WS" ]; then
    sed -i 's/^    lidar_x: 2.30/    lidar_x: 2.31/' src/av_features_cpp/config/acc_params.yaml
    must fail "one copy of lidar_x moved"                  param_sync
    restore src/av_features_cpp/config/acc_params.yaml
fi
# 9. the text rules, through the hook's sweep of a file written by a shell
git init -q . && git add -A > /dev/null 2>&1 && git -c user.email=t@t -c user.name=t commit -qm base
mkdir -p "$T/tmp"; printf '{"start": 0, "pids": []}' > "$T/tmp/claude_session_seed.json"
printf '#!/bin/bash\n# see the %s plan\n' "$EM" > scripts/seeded.sh
must fail "an em dash in a shell-written script"           env VLM_AV_HOOK_TMP="$T/tmp" python3 .claude/hooks/hook.py stop-check <<<'{"session_id":"seed"}'
rm scripts/seeded.sh

echo "$pass passed, $fail failed"
[ "$fail" = 0 ]
