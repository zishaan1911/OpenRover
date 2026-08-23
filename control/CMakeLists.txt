cmake_minimum_required(VERSION 3.8)
project(openrover_control)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

add_executable(command_interface_node
  src/main.cpp
  src/command_interface_node.cpp
)

target_include_directories(command_interface_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(command_interface_node
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
  ros_gz_interfaces
)

install(TARGETS
  command_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
