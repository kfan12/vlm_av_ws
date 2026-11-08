cmake_minimum_required(VERSION 3.8)
project(av_planning_coordinator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(av_feature_msgs REQUIRED)

add_executable(trajectory_coordinator_node src/trajectory_coordinator_node.cpp)
ament_target_dependencies(trajectory_coordinator_node rclcpp std_msgs nav_msgs av_feature_msgs)

add_executable(constraint_from_maneuver_node src/constraint_from_maneuver_node.cpp)
ament_target_dependencies(constraint_from_maneuver_node rclcpp std_msgs av_feature_msgs)

install(TARGETS trajectory_coordinator_node constraint_from_maneuver_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
