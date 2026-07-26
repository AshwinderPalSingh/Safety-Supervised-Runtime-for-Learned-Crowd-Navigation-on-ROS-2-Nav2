cmake_minimum_required(VERSION 3.16)
project(crowd_nav_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(crowd_nav_pedestrians REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# SHARED: this workspace's CMake default is static, which lacks -fPIC and can't be linked into
# a downstream .so - found the hard way when crowd_nav_controller (a pluginlib plugin, which
# MUST be a real .so for dlopen()) failed to link against this package as a static dependency
# (Phase 7 findings).
add_library(${PROJECT_NAME} SHARED
  src/ground_truth_human_source.cpp
  src/lidar_clustering.cpp
  src/lidar_tracker.cpp
  src/lidar_human_tracker_source.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp geometry_msgs crowd_nav_pedestrians tf2 tf2_ros tf2_geometry_msgs sensor_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp geometry_msgs crowd_nav_pedestrians tf2 tf2_ros tf2_geometry_msgs sensor_msgs
)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ground_truth_human_source test/test_ground_truth_human_source.cpp)
  target_link_libraries(test_ground_truth_human_source ${PROJECT_NAME})
  ament_target_dependencies(test_ground_truth_human_source rclcpp geometry_msgs crowd_nav_pedestrians tf2 tf2_geometry_msgs)

  ament_add_gtest(test_lidar_clustering test/test_lidar_clustering.cpp)
  target_link_libraries(test_lidar_clustering ${PROJECT_NAME})

  ament_add_gtest(test_lidar_tracker test/test_lidar_tracker.cpp)
  target_link_libraries(test_lidar_tracker ${PROJECT_NAME})
  ament_target_dependencies(test_lidar_tracker rclcpp)

  ament_add_gtest(test_lidar_human_tracker_source test/test_lidar_human_tracker_source.cpp)
  target_link_libraries(test_lidar_human_tracker_source ${PROJECT_NAME})
  ament_target_dependencies(test_lidar_human_tracker_source rclcpp tf2_ros geometry_msgs)
endif()

ament_package()
