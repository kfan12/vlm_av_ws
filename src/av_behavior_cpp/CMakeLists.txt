cmake_minimum_required(VERSION 3.8)
project(av_behavior_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  # av_common/projection.hpp -> CameraInfo
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(av_common_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# OSQP / osqp-eigen were built from source into /usr/local (CLAUDE.md gotcha)
set(OsqpEigen_DIR "/usr/local/lib/cmake/OsqpEigen" CACHE PATH "")
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

include_directories(src)  # for "mpc/..."

# --- local planner (curve splice + speed profile) --------------------------
add_executable(local_planner src/local_planner.cpp)
target_link_libraries(local_planner av_common_cpp::av_common)
ament_target_dependencies(local_planner rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs)

# --- MPC v2 (solver in src/mpc/)--------
add_executable(model_probe
  src/mpc/model_probe.cpp
  src/mpc/mpc_solver.cpp
  src/mpc/path_utils.cpp
  src/mpc/vehicle_model.cpp)
target_link_libraries(model_probe av_common_cpp::av_common
  Eigen3::Eigen tf2::tf2 osqp::osqp OsqpEigen::OsqpEigen)
ament_target_dependencies(model_probe
  rclcpp std_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)


add_executable(mpc_probe
  src/mpc/mpc_probe.cpp
  src/mpc/mpc_solver.cpp
  src/mpc/vehicle_model.cpp
  src/mpc/path_utils.cpp)
target_link_libraries(mpc_probe Eigen3::Eigen tf2::tf2 osqp::osqp OsqpEigen::OsqpEigen)
ament_target_dependencies(mpc_probe
 rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# --- MPC v2 tracker node (runtime) -----------------------------------------
add_executable(mpc_tracker_v2
  src/mpc_tracker_v2_node.cpp
  src/mpc/mpc_solver.cpp
  src/mpc/vehicle_model.cpp
  src/mpc/path_utils.cpp)
target_link_libraries(mpc_tracker_v2
  Eigen3::Eigen tf2::tf2 osqp::osqp OsqpEigen::OsqpEigen)
ament_target_dependencies(mpc_tracker_v2
  rclcpp std_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS model_probe
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS mpc_probe
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS mpc_tracker_v2
  DESTINATION lib/${PROJECT_NAME})

  
install(TARGETS local_planner
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
