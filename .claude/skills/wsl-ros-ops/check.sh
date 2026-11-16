#!/bin/bash
source "$(dirname "$(readlink -f "$0")")/../check_lib.sh"
# each rule the skill says a hook enforces is in the hook
H=.claude/hooks/hook.py
need_grep 'MSYS_NO_PATHCONV' "$H"
need_grep 'PYTHONUNBUFFERED' "$H"
need_grep 'headless-rendering' "$H"
need_grep 'GIT_WRITE' "$H"
need_grep 'def fix_mode' "$H"
need_grep 'def leftovers' "$H"
need_file .claude/hooks/protected_paths.yaml scripts/stop_av_stack.sh scripts/run_scenario_batch.sh tools/avkit/ros_probe.py
python3 -m py_compile tools/avkit/ros_probe.py || bad "ros_probe.py does not compile"
for sub in field rate graph; do need_grep "add_parser\('$sub'\)|'$sub'" tools/avkit/ros_probe.py; done
bash scripts/stop_av_stack.sh -n > /dev/null || bad "stop_av_stack.sh -n failed"
# the same rules are printed at session start, for subagents
need_grep 'WSL_RULES' "$H"
done_
