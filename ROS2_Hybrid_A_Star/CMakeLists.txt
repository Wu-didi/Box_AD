cmake_minimum_required(VERSION 3.8)
project(ros2_hybrid_a_star)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Find glog using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLOG REQUIRED libglog)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
)

# Hybrid A* library
add_library(hybrid_a_star_lib
  src/hybrid_a_star.cpp
  src/rs_path.cpp
)

target_link_libraries(hybrid_a_star_lib
  ${EIGEN3_LIBRARIES}
  ${GLOG_LIBRARIES}
)

# Hybrid A* node executable
add_executable(hybrid_a_star_node
  src/hybrid_a_star_node.cpp
)

ament_target_dependencies(hybrid_a_star_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
)

target_link_libraries(hybrid_a_star_node
  hybrid_a_star_lib
)

# Install
install(TARGETS
  hybrid_a_star_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
