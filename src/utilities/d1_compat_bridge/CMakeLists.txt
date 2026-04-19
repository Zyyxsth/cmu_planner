cmake_minimum_required(VERSION 3.8)
project(d1_compat_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(motion_msgs REQUIRED)
find_package(ception_msgs REQUIRED)

add_executable(d1_compat_bridge_node src/d1_compat_bridge_node.cpp)
ament_target_dependencies(
  d1_compat_bridge_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  diagnostic_msgs
  tf2
  tf2_geometry_msgs
  motion_msgs
  ception_msgs
)

install(TARGETS
  d1_compat_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
