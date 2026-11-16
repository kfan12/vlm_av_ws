#!/bin/bash
source "$(dirname "$(readlink -f "$0")")/../check_lib.sh"
help=$(./scripts/start_course_demo.sh -h 2>&1) || bad "start_course_demo.sh -h failed"
for m in vlm aeb lidar acc; do grep -qE "^  $m " <<<"$help" || bad "demo mode $m not in the help"; done
# six panes, as the gate table assumes
for p in 1 2 3 4 5 6; do grep -qE "^  col[123] (top|bottom) +$p " <<<"$help" || bad "pane $p not in the help's pane table"; done
# the pane script takes the env the headless check passes, and pane 6 runs with no stack
need_grep 'DEMO_MODE' scripts/_course_demo_pane.sh
need_grep 'DEMO_RUNDIR' scripts/_course_demo_pane.sh
out=$(DEMO_MODE=acc timeout 20 bash scripts/_course_demo_pane.sh 6 < /dev/null 2>&1 | head -5)
[ -n "$out" ] || bad "pane 6 printed nothing without a stack"
# what the gate table expects each pane to show
need_grep 'VLM ready' scripts/_course_demo_pane.sh
need_grep 'ctrl' scripts/_course_demo_watch.py scripts/_course_demo_pane.sh
need_grep 'OpenGL context' "$SKILL_MD"
need_file scripts/_course_demo_watch.py scripts/_av_stack_common.sh src/robot_bringup/config/av_stack.rviz
done_
