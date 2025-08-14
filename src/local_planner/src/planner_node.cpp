#include "planner.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "primitive_planner_node");
  ros::NodeHandle nh("~");
  PrimitivePlanner node(nh);
  ros::spin();
  return 0;
}