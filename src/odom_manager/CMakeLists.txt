cmake_minimum_required(VERSION 3.0.2)
project(odom_manager)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES odom_manager
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs visualization_msgs tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(odometry_manager_node src/odometry_manager_node.cpp src/vehicle.cpp src/agv.cpp src/odometry_manager.cpp)

target_link_libraries(odometry_manager_node
  ${catkin_LIBRARIES}
)
