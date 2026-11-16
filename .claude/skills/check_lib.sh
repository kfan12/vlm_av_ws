# check_lib.sh - sourced by every skill's check.sh.
# A check asserts what its skill claims about the code and exits non-zero when a
# claim no longer holds, so a stale skill is found by a script, not by a reader.
# Paths a skill names in backticks are checked for all skills at once by
# .claude/hooks/ref_lint.py; a check.sh holds what a path check cannot see.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS" || exit 1
SKILL_MD="$(dirname "$(readlink -f "$0")")/SKILL.md"
_fail=0
ok()   { :; }
bad()  { echo "  FAIL: $*"; _fail=1; }
# need_file PATH...: each exists
need_file() { local p; for p in "$@"; do [ -e "$p" ] || bad "missing $p"; done; }
# need_grep PATTERN FILE...: the pattern occurs in at least one of the files
need_grep() { local pat=$1; shift; grep -qE -- "$pat" "$@" 2>/dev/null || bad "'$pat' not found in $*"; }
# skill_says PATTERN: the skill text itself still carries a claim this check depends on
skill_says() { grep -qE -- "$1" "$SKILL_MD" || bad "the skill no longer mentions '$1' (update check.sh with the skill)"; }
# case_labels FILE: the labels of the first `case "$ROW" in` block, one per line
case_labels() { awk '/^case "\$ROW" in/{n++} n==1 && /^esac/{exit} n==1' "$1" | grep -oE '^\s+[a-z_|]+\)' | tr -d ' )' | tr '|' '\n' | sort; }
# same_set NAME "expected words" "actual words"
same_set() {
    local a b
    a=$(tr ' ' '\n' <<<"$2" | awk 'NF' | sort | tr '\n' ' '); b=$(tr ' ' '\n' <<<"$3" | awk 'NF' | sort | tr '\n' ' ')
    [ "$a" = "$b" ] || bad "$1: the skill lists [$a] but the code has [$b]"
}
# launch_args: every argument av_stack.launch.py declares
launch_args() { grep -o "DeclareLaunchArgument('[a-z_]*'" src/robot_bringup/launch/av_stack.launch.py | sed "s/.*('//; s/'//" | sort; }
done_() { [ "$_fail" = 0 ] && echo "  ok"; exit "$_fail"; }
