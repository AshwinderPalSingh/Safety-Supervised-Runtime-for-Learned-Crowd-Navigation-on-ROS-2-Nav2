cmake_minimum_required(VERSION 3.16)
project(crowd_nav_policy_adapters)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(crowd_nav_observation REQUIRED)
find_package(crowd_nav_onnxruntime_vendor REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}
  src/candidate_action_space.cpp
  src/candidate_propagation.cpp
  src/shape_validation.cpp
  src/onnx_inference.cpp
  src/dummy_adapter.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${crowd_nav_onnxruntime_vendor_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp crowd_nav_observation
)
target_link_libraries(${PROJECT_NAME}
  ${crowd_nav_onnxruntime_vendor_LIBRARIES}
  yaml-cpp
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)
install(
  PROGRAMS scripts/generate_dummy_model.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp crowd_nav_observation crowd_nav_onnxruntime_vendor yaml_cpp_vendor)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_policy_adapters
    test/test_candidate_action_space.cpp
    test/test_candidate_propagation.cpp
    test/test_shape_validation.cpp
    test/test_dummy_adapter.cpp
  )
  target_link_libraries(test_policy_adapters ${PROJECT_NAME})
  target_include_directories(test_policy_adapters PRIVATE ${crowd_nav_onnxruntime_vendor_INCLUDE_DIRS})
  target_compile_definitions(test_policy_adapters PRIVATE
    CROWD_NAV_POLICY_ADAPTERS_PACKAGE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )
  ament_target_dependencies(test_policy_adapters rclcpp crowd_nav_observation)
endif()

ament_package()
