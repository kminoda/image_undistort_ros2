cmake_minimum_required(VERSION 3.5)
project(image_undistort)

# OpenCV
find_package(OpenCV REQUIRED)

## Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

## Targets
ament_auto_add_library(image_undistort_node_component SHARED
  src/image_undistort/image_undistort_node.cpp
)

target_include_directories(
  image_undistort_node_component PRIVATE
  SYSTEM
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
  image_undistort_node_component
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(image_undistort_node_component
  PLUGIN "image_undistort::ImageUndistortNode"
  EXECUTABLE image_undistort_node
)

## Tests
if(BUILD_TESTING)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## Package
ament_auto_package(
  INSTALL_TO_SHARE
    launch
)