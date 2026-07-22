cmake_minimum_required(VERSION 3.16)
project(crowd_nav_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
