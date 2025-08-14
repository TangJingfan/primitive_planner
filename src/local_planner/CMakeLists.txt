cmake_minimum_required(VERSION 3.0.2)
project(local_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(primitive_planner_node src/planner.cpp src/planner_node.cpp)

target_link_libraries(primitive_planner_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_dependencies(primitive_planner_node ${catkin_EXPORTED_TARGETS})
