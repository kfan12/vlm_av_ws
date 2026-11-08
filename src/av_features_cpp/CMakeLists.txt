cmake_minimum_required(VERSION 3.8)
project(av_features_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(av_common_cpp REQUIRED)
find_package(av_feature_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(acc_node src/acc_node.cpp)
target_link_libraries(acc_node av_common_cpp::av_common)
ament_target_dependencies(acc_node rclcpp std_msgs sensor_msgs nav_msgs visualization_msgs av_feature_msgs)

install(TARGETS acc_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
