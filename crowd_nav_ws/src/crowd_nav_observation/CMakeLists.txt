cmake_minimum_required(VERSION 3.16)
project(crowd_nav_observation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(crowd_nav_perception REQUIRED)

add_library(${PROJECT_NAME}
  src/observation_builder.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp crowd_nav_perception
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
ament_export_dependencies(rclcpp crowd_nav_perception)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  install(
    DIRECTORY test/fixtures/
    DESTINATION share/${PROJECT_NAME}/test/fixtures
  )
  ament_add_gtest(test_observation_builder test/test_observation_builder.cpp)
  target_link_libraries(test_observation_builder ${PROJECT_NAME})
  target_compile_definitions(test_observation_builder PRIVATE
    CROWD_NAV_OBSERVATION_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures"
  )
  ament_target_dependencies(test_observation_builder rclcpp crowd_nav_perception)
endif()

ament_package()
