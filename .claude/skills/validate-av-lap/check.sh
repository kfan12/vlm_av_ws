#!/bin/bash
source "$(dirname "$(readlink -f "$0")")/../check_lib.sh"
# the rows the command block names, against the runners
skill_says 'rows: inlane outlane outlane_left partial curve sudden mpc_timeout'
same_set "AEB rows" "inlane outlane outlane_left partial curve sudden mpc_timeout sensor_dropout" "$(case_labels scripts/run_aeb_scenario.sh | tr '\n' ' ')"
skill_says 'follow\|lead_brake\|stop_go\|curve\|cut_in'
same_set "ACC rows" "follow lead_brake stop_go curve cut_in" "$(case_labels scripts/run_acc_scenario.sh | tr '\n' ' ')"
# run_headless_lap.sh still defaults to the full course and takes STACK_ARGS (the skill says to pass the world)
need_grep 'WORLD=\$\{4:-urban_course\}' scripts/run_headless_lap.sh
need_grep 'STACK_ARGS' scripts/run_headless_lap.sh
need_grep 'AEB_SENSOR' scripts/run_aeb_scenario.sh
need_grep -- '--since' tools/avkit/compare_aeb_sessions.py
need_grep -- '--plots' tools/avkit/analyze_av_lap.py
# the lap metrics step 3 names
for m in crosstrack_rmse_m crosstrack_max_m speed_rmse_ms; do need_grep "$m" tools/avkit/analyze_av_lap.py; done
need_grep 'mpc_closed_loop_sim' src/av_behavior_cpp/CMakeLists.txt
need_file docs/baselines/README.md docs/reports/lidar_acc.md tools/avkit/export_lane_csv.py scripts/run_scenario_batch.sh
done_
