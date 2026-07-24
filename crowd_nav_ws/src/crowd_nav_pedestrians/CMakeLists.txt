cmake_minimum_required(VERSION 3.16)
project(crowd_nav_pedestrians)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pedestrian.msg"
  "msg/PedestrianArray.msg"
  DEPENDENCIES std_msgs
)

install(PROGRAMS
  scripts/pedestrian_sim_node.py
  scripts/actor_mirror_node.py
  scripts/robot_pose_extractor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
