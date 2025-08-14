#include "path_finder.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_finder_node");
  ros::NodeHandle nh("~");
  PathFinder node(nh);
  ros::spin();
  return 0;
}