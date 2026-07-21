cmake_minimum_required(VERSION 3.16)
project(crowd_nav_onnxruntime_vendor)

find_package(ament_cmake REQUIRED)

# Pinned per IMPLEMENTATION_PLAN.md §1.3/§1.9: CPU-only Linux x64 release. Originally pinned
# to 1.17.3 (mature, well within Humble's window) but that only supports ONNX IR version <= 9;
# PyTorch's current exporter (this machine: torch 2.13, dynamo-based) emits IR version 10,
# which 1.17.3 rejects outright at load time ("Unsupported model IR version: 10, max
# supported IR version: 9") - confirmed by actually loading an exported model, not assumed.
# 1.19+ added IR version 10 support; using 1.20.1 for headroom. Bump this single line to move
# to a newer release, but re-run test_onnxruntime_link against a freshly-exported model first.
set(ONNXRUNTIME_VERSION "1.20.1")
set(ONNXRUNTIME_ARCHIVE "onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}.tgz")
set(ONNXRUNTIME_URL
  "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/${ONNXRUNTIME_ARCHIVE}")
set(ONNXRUNTIME_DOWNLOAD_DIR "${CMAKE_CURRENT_BINARY_DIR}/onnxruntime_download")
set(ONNXRUNTIME_EXTRACT_DIR "${ONNXRUNTIME_DOWNLOAD_DIR}/onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}")

if(NOT EXISTS "${ONNXRUNTIME_EXTRACT_DIR}/lib/libonnxruntime.so")
  file(MAKE_DIRECTORY "${ONNXRUNTIME_DOWNLOAD_DIR}")
  set(ONNXRUNTIME_TARBALL "${ONNXRUNTIME_DOWNLOAD_DIR}/${ONNXRUNTIME_ARCHIVE}")

  message(STATUS "crowd_nav_onnxruntime_vendor: downloading ONNX Runtime ${ONNXRUNTIME_VERSION}")
  file(DOWNLOAD "${ONNXRUNTIME_URL}" "${ONNXRUNTIME_TARBALL}"
    STATUS ONNXRUNTIME_DOWNLOAD_STATUS
    SHOW_PROGRESS
  )
  list(GET ONNXRUNTIME_DOWNLOAD_STATUS 0 ONNXRUNTIME_DOWNLOAD_STATUS_CODE)
  if(NOT ONNXRUNTIME_DOWNLOAD_STATUS_CODE EQUAL 0)
    list(GET ONNXRUNTIME_DOWNLOAD_STATUS 1 ONNXRUNTIME_DOWNLOAD_STATUS_MSG)
    message(FATAL_ERROR
      "crowd_nav_onnxruntime_vendor: failed to download ${ONNXRUNTIME_URL}: ${ONNXRUNTIME_DOWNLOAD_STATUS_MSG}")
  endif()

  message(STATUS "crowd_nav_onnxruntime_vendor: extracting ${ONNXRUNTIME_ARCHIVE}")
  file(ARCHIVE_EXTRACT INPUT "${ONNXRUNTIME_TARBALL}" DESTINATION "${ONNXRUNTIME_DOWNLOAD_DIR}")
endif()

if(NOT EXISTS "${ONNXRUNTIME_EXTRACT_DIR}/lib/libonnxruntime.so")
  message(FATAL_ERROR
    "crowd_nav_onnxruntime_vendor: expected ${ONNXRUNTIME_EXTRACT_DIR}/lib/libonnxruntime.so after extraction, not found")
endif()

# Install headers/libs into this package's own install prefix and expose them as a
# normal imported target, so downstream packages just do:
#   find_package(crowd_nav_onnxruntime_vendor REQUIRED)
#   target_link_libraries(my_target PRIVATE crowd_nav_onnxruntime_vendor::onnxruntime)
install(
  DIRECTORY "${ONNXRUNTIME_EXTRACT_DIR}/include/"
  DESTINATION include/crowd_nav_onnxruntime_vendor
)
install(
  DIRECTORY "${ONNXRUNTIME_EXTRACT_DIR}/lib/"
  DESTINATION lib
  FILES_MATCHING PATTERN "libonnxruntime*.so*"
)

# IMPORTED target for use within this package's own build (the test executable below).
# NOTE: plain `install(TARGETS ... EXPORT ...)` does not work for IMPORTED targets in CMake,
# so downstream packages consume this vendor package the older, plainer ament way below
# (ament_export_include_directories / ament_export_libraries) rather than via this target.
add_library(onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime PROPERTIES
  IMPORTED_LOCATION "${ONNXRUNTIME_EXTRACT_DIR}/lib/libonnxruntime.so"
  INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_EXTRACT_DIR}/include"
)

# Phase 0 plumbing check: prove the vendored runtime actually links and runs inference
# (IMPLEMENTATION_PLAN.md Phase 0 / Phase 6), not just that CMake found the headers.
add_executable(test_onnxruntime_link test/test_onnxruntime_link.cpp)
target_link_libraries(test_onnxruntime_link PRIVATE onnxruntime)
install(TARGETS test_onnxruntime_link DESTINATION lib/${PROJECT_NAME})
install(FILES test_models/trivial.onnx DESTINATION share/${PROJECT_NAME}/test_models)

# Downstream packages (crowd_nav_policy_adapters, crowd_nav_controller, from Phase 6 onward):
#   find_package(crowd_nav_onnxruntime_vendor REQUIRED)
#   target_include_directories(mytarget PRIVATE ${crowd_nav_onnxruntime_vendor_INCLUDE_DIRS})
#   target_link_libraries(mytarget PRIVATE ${crowd_nav_onnxruntime_vendor_LIBRARIES})
ament_export_include_directories(include/crowd_nav_onnxruntime_vendor)
ament_export_libraries(onnxruntime)

ament_package()
