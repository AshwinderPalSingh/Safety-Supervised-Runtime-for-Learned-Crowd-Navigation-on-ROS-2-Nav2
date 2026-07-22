cmake_minimum_required(VERSION 3.16)
project(crowd_nav_control)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
