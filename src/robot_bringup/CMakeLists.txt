cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# odom_compare: republish every odometry source into one common frame for RViz.
add_executable(odom_compare src/odom_compare.cpp)
ament_target_dependencies(odom_compare rclcpp nav_msgs geometry_msgs)
install(TARGETS odom_compare DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
