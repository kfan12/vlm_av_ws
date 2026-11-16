#!/bin/bash
source "$(dirname "$(readlink -f "$0")")/../check_lib.sh"
args=$(launch_args | tr '\n' ' ')
# every launch argument the skill uses is declared
for a in aeb aeb_sensor acc world rviz gz_gl gz_gui; do
    skill_says "$a:="
    grep -qw "$a" <<<"$args" || bad "launch argument $a is not declared in av_stack.launch.py"
done
# the four demo modes and their default worlds
help=$(./scripts/start_course_demo.sh -h 2>&1) || bad "start_course_demo.sh -h failed"
for m in vlm aeb lidar acc; do grep -qE "^  $m " <<<"$help" || bad "demo mode $m not in start_course_demo.sh -h"; done
grep -qE '^  vlm .*urban_course$' <<<"$help" || bad "vlm demo default world is no longer urban_course"
grep -qE '^  aeb .*urban_course_roadblock$' <<<"$help" || bad "aeb demo default world is no longer urban_course_roadblock"
grep -qE '^  lidar .*urban_course_roadblock$' <<<"$help" || bad "lidar demo default world is no longer urban_course_roadblock"
grep -qE '^  acc .*urban_course$' <<<"$help" || bad "acc demo default world is no longer urban_course"
# the demo env vars the skill lists
for v in DEMO_WORLD DEMO_SPEED DEMO_GZ_GL DEMO_RVIZ STACK_NOWAIT DEMO_LEAD DEMO_LEAD_S DEMO_LEAD_PROFILE; do
    skill_says "$v"
    need_grep "$v" scripts/start_course_demo.sh scripts/_course_demo_pane.sh
done
# the launch guards and the health signals it describes
need_grep 'acc' src/robot_bringup/launch/av_stack.launch.py
need_grep 'stack_healthy' scripts/_av_stack_common.sh
need_grep '/control/active_source' src/av_control_cpp/src/control_arbiter_node.cpp
need_grep '/planning/binding_constraint' src/av_planning_coordinator/src/trajectory_coordinator_node.cpp
need_grep 'obstacle_source' src/av_safety_cpp/src/aeb_node.cpp
need_grep 'VLM ready' scripts/run_headless_lap.sh
need_file scripts/stop_av_stack.sh tools/avkit/ros_probe.py src/vlm_planner_py/config/sign_maneuver_params.yaml
done_
