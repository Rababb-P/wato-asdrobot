cmake_minimum_required(VERSION 3.8)
project(control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(control_node src/control_node.cpp)
target_include_directories(control_node PUBLIC include)
ament_target_dependencies(control_node rclcpp nav_msgs geometry_msgs tf2)

install(TARGETS control_node DESTINATION lib/${PROJECT_NAME})

ament_package()
