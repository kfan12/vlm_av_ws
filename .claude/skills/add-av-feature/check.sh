#!/bin/bash
source "$(dirname "$(readlink -f "$0")")/../check_lib.sh"
# the extension points the class table names
need_file src/av_feature_msgs/msg/VelocityConstraint.msg src/av_feature_msgs/msg/AebStatus.msg
for f in maximum_speed_mps valid_until stop_distance_m; do need_grep "$f" src/av_feature_msgs/msg/VelocityConstraint.msg; done
need_grep 'constraint_topics' src/av_planning_coordinator/config/coordinator_lidar_acc.yaml src/av_planning_coordinator/src/trajectory_coordinator_node.cpp
need_grep 'required_constraint_topics' src/av_planning_coordinator/src/trajectory_coordinator_node.cpp
# class 2 is still an extension to build: the merge does not use stop_distance_m
if grep -q 'stop_distance_m' src/av_planning_coordinator/src/trajectory_coordinator_node.cpp; then
    bad "the coordinator now reads stop_distance_m: rewrite the class 2 row and step 2 (it says the merge does not use it yet)"
fi
# the worked example
need_grep '/feature/acc/constraint' src/av_features_cpp/src/acc_node.cpp
need_grep 'mpc_remaps' src/robot_bringup/launch/av_stack.launch.py
need_grep '/planning/authorized/' src/robot_bringup/launch/av_stack.launch.py
need_grep '_ONLY_DISPLAYS' src/robot_bringup/launch/av_stack.launch.py
need_grep 'control/aeb_cmd' src/av_control_cpp/src/control_arbiter_node.cpp
# the warning flags step 3 prescribes for a new package (not every older package sets them)
need_grep 'Wall -Wextra -Wpedantic' src/av_control_cpp/CMakeLists.txt src/av_planning_coordinator/CMakeLists.txt
# class 4 points at the overtake design and its decision
need_file local/plans/overtake/README.md src/av_planning_coordinator/src/constraint_from_maneuver_node.cpp
need_grep 'overtake_manager' local/kb/decisions.md
done_
