cmake_minimum_required(VERSION 3.8)
project(differential_drive_robot_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch world
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
