cmake_minimum_required(VERSION 3.8)
project(av_safety_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(aeb_node src/aeb_node.cpp)
target_include_directories(aeb_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(aeb_node av_common_cpp::av_common ${OpenCV_LIBS})
ament_target_dependencies(aeb_node
  rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs visualization_msgs av_feature_msgs)

install(TARGETS aeb_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
