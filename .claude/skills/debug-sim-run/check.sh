#!/bin/bash
source "$(dirname "$(readlink -f "$0")")/../check_lib.sh"
need_file tools/avkit/ros_probe.py local/harness/golden/golden.py docs/baselines/README.md \
          .claude/agents/sim-debugger.md local/notes/aeb_status_and_lessons.md local/notes/lidar_implementation.md
need_grep 'mpc_closed_loop_sim' src/av_behavior_cpp/CMakeLists.txt
# the signals step 2 tells the reader to look at are recorded
need_grep 'active_source' tools/avkit/record_aeb_scenario.py tools/avkit/record_av_lap.py
need_grep 'binding' tools/avkit/record_aeb_scenario.py
need_grep 'acc_state' tools/avkit/record_aeb_scenario.py
# the validity markers of step 1
need_grep 'INVALID RUN' scripts/_av_stack_common.sh
need_grep 'PROPORTIONAL fallback' scripts/run_headless_lap.sh
done_
