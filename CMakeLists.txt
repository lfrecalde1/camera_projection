cmake_minimum_required(VERSION 3.8)
project(camera_projection)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# List of dependencies
set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  tf2_ros
  Eigen3
)

# Create a library for the node
add_library(camera_projection_node SHARED
  src/camera_projection_node.cpp
)

# Specify the include directories
target_include_directories(camera_projection_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies to the library
ament_target_dependencies(camera_projection_node ${dependencies})

# Register the node as a component
rclcpp_components_register_node(camera_projection_node
  PLUGIN "camera_projection_node::CameraProjectionNode"
  EXECUTABLE camera_projection_node_exec)

# Install the library and executable
install(TARGETS camera_projection_node camera_projection_node_exec
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Export the package
ament_package()