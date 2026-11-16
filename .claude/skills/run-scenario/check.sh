#!/bin/bash
source "$(dirname "$(readlink -f "$0")")/../check_lib.sh"
AEB="inlane outlane outlane_left partial curve sudden mpc_timeout sensor_dropout"
ACC="follow lead_brake stop_go curve cut_in"
skill_says "\`$AEB\`"
skill_says "\`$ACC\`"
same_set "AEB rows" "$AEB" "$(case_labels scripts/run_aeb_scenario.sh | tr '\n' ' ')"
same_set "ACC rows" "$ACC" "$(case_labels scripts/run_acc_scenario.sh | tr '\n' ' ')"
# the validity lines step 3 tells the reader to look for
need_grep 'stack never came up healthy' scripts/_av_stack_common.sh
need_grep 'arbiter never handed over' scripts/_av_stack_common.sh
need_grep 'INVALID RUN' scripts/_av_stack_common.sh scripts/run_acc_scenario.sh
need_grep 'sensor_dropout needs AEB_SENSOR=lidar' scripts/run_aeb_scenario.sh
# both runners are fixed to the full course
need_grep 'av_launch urban_course' scripts/run_aeb_scenario.sh
need_grep '^WORLD=urban_course$' scripts/run_acc_scenario.sh
# the metric names of step 4 are what the analyzers write
for m in source_sequence standstill_gt_standoff_m command_gate_response_sec time_to_standstill_s false_positive_brakes; do
    need_grep "$m" tools/avkit/analyze_aeb_scenario.py
done
for m in min_gap_m steady_gap_error_m steady_time_gap_s acc_gap_error_m cap_drop_latency_s restart_lag_s max_over_set_speed_mps binding_counts; do
    need_grep "$m" tools/avkit/analyze_acc_run.py
done
need_file docs/reports/aeb_scenario_matrix.md docs/reports/lidar_aeb_matrix.md docs/reports/lidar_acc.md
done_
