cmake_minimum_required(VERSION 3.0.2)
project(map_builder)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  octomap_ros       
  octomap_msgs
)

find_package(PCL REQUIRED)
find_package(octomap REQUIRED)   

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs pcl_ros pcl_conversions octomap_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}      
)

link_directories(${PCL_LIBRARY_DIRS} include)
add_definitions(${PCL_DEFINITIONS})

add_executable(map_builder_node 
  src/map_builder.cpp src/map_builder_node.cpp
)

target_link_libraries(map_builder_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}         
)

add_dependencies(map_builder_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)