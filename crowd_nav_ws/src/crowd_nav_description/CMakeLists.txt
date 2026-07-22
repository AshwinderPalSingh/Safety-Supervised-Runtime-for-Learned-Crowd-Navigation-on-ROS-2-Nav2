cmake_minimum_required(VERSION 3.16)
project(crowd_nav_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf rviz
  DESTINATION share/${PROJECT_NAME}
)
install(
  FILES MEASUREMENTS.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
