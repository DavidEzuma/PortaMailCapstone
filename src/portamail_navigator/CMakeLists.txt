cmake_minimum_required(VERSION 3.8)
project(portamail_navigator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# --- Executable: Mock Driver (Simulation Fallback) ---
add_executable(mock_driver src/mock_driver.cpp)
ament_target_dependencies(mock_driver 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  tf2 
  tf2_ros
  tf2_geometry_msgs)

# --- Install Executables ---
install(TARGETS mock_driver
  DESTINATION lib/${PROJECT_NAME})

# --- Install ALL Resource Directories ---
# Note: portamail.urdf now works for both hardware and Gazebo simulation
install(DIRECTORY 
  launch 
  config 
  firmware 
  urdf 
  meshes
  worlds       # Gazebo world files
  models       # Gazebo SDF models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
