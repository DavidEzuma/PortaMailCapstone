cmake_minimum_required(VERSION 3.8)
project(portamail_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# We need C++17 for filesystem support (used in saving map logic)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(PkgConfig REQUIRED)

# Use PkgConfig to find yaml-cpp (standard linux library)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

# --- Executable ---
add_executable(coordinator src/navigation_coordinator.cpp)

# Include directories for ROS packages + YAML
target_include_directories(coordinator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${YAML_CPP_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(coordinator
  ${YAML_CPP_LIBRARIES}
)

# Link ROS 2 dependencies
ament_target_dependencies(coordinator
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav2_msgs
  slam_toolbox
)

# --- Install Rules ---
install(TARGETS coordinator
  DESTINATION lib/${PROJECT_NAME})

# Install the config/launch folders if you have them (optional but good practice)
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()