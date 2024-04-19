cmake_minimum_required(VERSION 3.22)
project(twist_to_ackermann)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(twist_to_ackermann_node
  src/twist_to_ackermann_node.cpp
)

ament_auto_package()
