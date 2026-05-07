cmake_minimum_required(VERSION 3.8)
project(differential_drive_robot_control)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS nodes/teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleop
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
