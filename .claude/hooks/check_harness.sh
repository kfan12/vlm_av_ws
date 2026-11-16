#!/bin/bash
# check_harness.sh - every check of the AI harness that needs no model and no simulator.
#   bash .claude/hooks/check_harness.sh [-v]
# One PASS / FAIL / SKIP line per check, the failing check's output underneath, exit 1 on
# any failure. Run it after changing a skill, a hook, an agent, a CLAUDE.md, the index or
# an analyzer, and at the end of every knowledge-base sync.
# A skill whose check.sh passes gets today's date as its "Last verified" date.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS" || exit 1
VERBOSE=${1:-}
export PYTHONNOUSERSITE=1
pass=0; fail=0; skip=0
EM=$(printf '\xe2\x80\x94')

run() {  # run NAME COMMAND...
    local name=$1 out rc; shift
    out=$("$@" 2>&1); rc=$?
    if [ "$rc" = 0 ]; then
        pass=$((pass + 1)); echo "PASS  $name"
        [ -n "$VERBOSE" ] && printf '%s\n' "$out" | tail -3 | sed 's/^/        /'
    else
        fail=$((fail + 1)); echo "FAIL  $name"; printf '%s\n' "$out" | tail -25 | sed 's/^/        /'
    fi
    return "$rc"
}
skip() { skip=$((skip + 1)); echo "SKIP  $1 ($2)"; }

echo "== hooks"
run "hook tests" bash .claude/hooks/tests/run.sh
run "seeded faults: every checker catches its fault" bash .claude/hooks/tests/seeded_faults.sh

echo "== skills and agents"
frontmatter() {
    python3 - <<'EOF'
import glob, sys, yaml
bad = []
for p in sorted(glob.glob('.claude/skills/*/SKILL.md') + glob.glob('.claude/agents/*.md')):
    t = open(p).read()
    if not t.startswith('---\n') or '\n---\n' not in t[4:]:
        bad.append(f'{p}: no frontmatter'); continue
    try:
        d = yaml.safe_load(t[4:].split('\n---\n', 1)[0])
    except yaml.YAMLError as e:
        bad.append(f'{p}: {str(e).splitlines()[0]}'); continue
    if not isinstance(d, dict) or not {'name', 'description'} <= set(d):
        bad.append(f'{p}: needs name and description')
    elif '/skills/' in p and d['name'] != p.split('/')[-2]:
        bad.append(f'{p}: name "{d["name"]}" differs from its directory')
print('\n'.join(bad)); sys.exit(1 if bad else 0)
EOF
}
run "frontmatter of every skill and agent" frontmatter
today=$(date +%Y-%m-%d)
for d in .claude/skills/*/; do
    name=$(basename "$d")
    if [ ! -f "$d/check.sh" ]; then
        fail=$((fail + 1)); echo "FAIL  skill $name: no check.sh"; continue
    fi
    if run "skill $name" bash "$d/check.sh"; then
        # the date is written by the check, not typed: the first "Last verified ... YYYY-MM-DD" in the frontmatter
        sed -i -E "1,6s/(Last verified[a-z ]*) [0-9]{4}-[0-9]{2}-[0-9]{2}/\1 $today/" "$d/SKILL.md"
    fi
done
run "reference lint (skills, agents, CLAUDE.md)" python3 .claude/hooks/ref_lint.py

echo "== knowledge base"
run "invariant registry" python3 .claude/hooks/invariants.py --check
run "node index" python3 tools/avkit/kb_index.py --check
run "duplicated parameters" bash scripts/check_param_sync.sh
run "no cv_bridge" bash scripts/check_no_cv_bridge.sh
run "architecture page builds" python3 local/kb/architecture/build.py
run "source guide builds" bash local/kb/source_guide/build.sh
order() {  # decisions.md: newest at the bottom
    grep -o '^- 20[0-9-]*' local/kb/decisions.md | sort -c 2>&1
}
if [ -f local/kb/decisions.md ]; then run "decisions in date order" order; else skip "decisions in date order" "local/kb/decisions.md is local only"; fi

echo "== shipped files"
text_rules() {
    local bad=0
    if grep -rnI --exclude-dir=__pycache__ "$EM" src scripts tools README.md; then bad=1; fi
    if grep -rnIE 'Day [0-9]|day[0-9]{2}\.md|local/plans/' src scripts tools README.md \
        --include=*.py --include=*.cpp --include=*.hpp --include=*.sh --include=*.yaml --include=*.md \
        --include=*.xacro --include=*.txt --exclude=CLAUDE.md --exclude-dir=__pycache__; then bad=1; fi
    return $bad
}
run "no em dash, no plan reference" text_rules
modes() {  # a file that starts with a shebang under scripts/ or tools/avkit/ and is tracked non-executable is listed, not failed
    git ls-files -s scripts tools/avkit | awk '$1 == "100644" {print $4}' | while read -r f; do
        head -c2 "$f" | grep -q '#!' && echo "tracked 100644 with a shebang: $f"
    done
    return 0
}
[ -n "$VERBOSE" ] && modes

echo "== analyzers"
if [ -f local/harness/golden/golden.py ]; then
    run "analyzer golden test" python3 local/harness/golden/golden.py --check
else
    skip "analyzer golden test" "local/harness is local only"
fi

echo "== evals"
if [ -f local/harness/evals/results.jsonl ]; then
    last=$(tail -1 local/harness/evals/results.jsonl | python3 -c 'import json,sys; print(json.loads(sys.stdin.read()).get("harness_commit", ""))')
    paths=$(git ls-files .claude ':(glob)**/CLAUDE.md' 2>/dev/null | head -400)
    behind=$(git rev-list --count "$last"..HEAD -- .claude 2>/dev/null || echo "?")
    dirty=$(git status --porcelain -- .claude | wc -l)
    echo "INFO  evals last ran at $last: $behind commit(s) and $dirty uncommitted file(s) under .claude/ since then"
    echo "      (powershell -File local/harness/run_evals.ps1 from Windows; about 50 short sessions for three trials)"
else
    echo "INFO  no eval result recorded yet (local/harness/evals/results.jsonl)"
fi

echo
echo "$pass passed, $fail failed, $skip skipped"
[ "$fail" = 0 ]
