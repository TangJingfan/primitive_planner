cmake_minimum_required(VERSION 3.0.2)
project(path_finder)

# Find catkin and required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Add executable
add_executable(path_finder_node
  src/path_finder.cpp
  src/path_finder_node.cpp
)

# Link libraries
target_link_libraries(path_finder_node
  ${catkin_LIBRARIES}
)

# Add dependencies
add_dependencies(path_finder_node
  ${catkin_EXPORTED_TARGETS}
)