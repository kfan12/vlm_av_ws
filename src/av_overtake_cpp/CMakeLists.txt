cmake_minimum_required(VERSION 3.8)
project(av_overtake_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(av_common_cpp REQUIRED)
find_package(av_feature_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(overtake_manager src/overtake_manager.cpp)
target_include_directories(overtake_manager PRIVATE include)
target_link_libraries(overtake_manager av_common_cpp::av_common)
ament_target_dependencies(overtake_manager rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs
  visualization_msgs av_feature_msgs)

install(TARGETS overtake_manager DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
