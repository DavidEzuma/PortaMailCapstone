cmake_minimum_required(VERSION 3.8)
project(portamail_coordinator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
find_package(yaml-cpp REQUIRED)

# --- Executable: Coordinator ---
add_executable(coordinator src/navigation_coordinator.cpp)

target_include_directories(coordinator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link generic system libraries (yaml-cpp)
target_link_libraries(coordinator yaml-cpp)

# Link ROS 2 packages
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

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()