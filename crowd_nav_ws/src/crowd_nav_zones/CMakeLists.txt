cmake_minimum_required(VERSION 3.16)
project(crowd_nav_zones)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddZone.srv"
  "srv/RemoveZone.srv"
)

install(PROGRAMS
  scripts/zone_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
