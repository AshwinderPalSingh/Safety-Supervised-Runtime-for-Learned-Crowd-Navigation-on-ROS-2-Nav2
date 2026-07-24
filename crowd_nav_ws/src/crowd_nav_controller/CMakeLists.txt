cmake_minimum_required(VERSION 3.16)
project(crowd_nav_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(angles REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(crowd_nav_observation REQUIRED)
find_package(crowd_nav_perception REQUIRED)
find_package(crowd_nav_policy_adapters REQUIRED)
find_package(crowd_nav_onnxruntime_vendor REQUIRED)
find_package(crowd_nav_safety_supervisor REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
  geometry_msgs
  nav_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  angles
  ament_index_cpp
  crowd_nav_observation
  crowd_nav_perception
  crowd_nav_policy_adapters
  crowd_nav_safety_supervisor
)

# SHARED, not the CMake default in this workspace (static) - pluginlib's class_loader loads
# plugins via dlopen() at runtime, which requires a .so. Confirmed the hard way: a static
# libcrowd_nav_controller.a built and gtest-linked fine, but controller_server failed with
# "Could not find library corresponding to plugin... Make sure that the library
# 'crowd_nav_controller' actually exists" - a unit test alone would never catch this, only the
# live pluginlib load path does (IMPLEMENTATION_PLAN.md S4.6's whole reason for a live
# Gazebo/Nav2 verification, not just gtest, for this phase).
add_library(${PROJECT_NAME} SHARED
  src/controller_decision_core.cpp
  src/crowd_nav_controller.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${crowd_nav_onnxruntime_vendor_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} ${crowd_nav_onnxruntime_vendor_LIBRARIES})

pluginlib_export_plugin_description_file(nav2_core crowd_nav_controller_plugin.xml)

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
ament_export_dependencies(${dependencies})
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_controller_decision_core test/test_controller_decision_core.cpp)
  target_link_libraries(test_controller_decision_core ${PROJECT_NAME})
  ament_target_dependencies(test_controller_decision_core rclcpp crowd_nav_policy_adapters)
endif()

ament_package()
