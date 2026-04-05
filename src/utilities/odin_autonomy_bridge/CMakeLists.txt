cmake_minimum_required(VERSION 3.8)
project(odin_autonomy_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(odin_autonomy_bridge_node src/odin_autonomy_bridge_node.cpp)
ament_target_dependencies(
  odin_autonomy_bridge_node
  rclcpp
  nav_msgs
  sensor_msgs
  std_msgs
)

install(TARGETS
  odin_autonomy_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
