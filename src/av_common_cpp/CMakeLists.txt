cmake_minimum_required(VERSION 3.8)
project(av_common_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Eigen3 REQUIRED)

# Header-only interface library.
add_library(av_common INTERFACE)
target_include_directories(av_common INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(av_common INTERFACE
  nlohmann_json::nlohmann_json Eigen3::Eigen)

# Smoke target: forces every header to actually compile.
# TODO(day01 task 6): re-enable once src/header_check.cpp is written.
add_executable(header_check src/header_check.cpp)
target_link_libraries(header_check av_common)
ament_target_dependencies(header_check rclcpp std_msgs sensor_msgs)

# add_executable(test_arc_length test/test_arc_length.cpp)
# target_link_libraries(test_arc_length av_common)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS av_common EXPORT export_av_common)
ament_export_targets(export_av_common HAS_LIBRARY_TARGET)
ament_export_dependencies(nlohmann_json Eigen3 rclcpp std_msgs sensor_msgs)

ament_package()