#include "map_builder.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_builder_node");
  ros::NodeHandle nh("~");
  MapBuilder mgr(nh);
  ros::spin();
  return 0;
}
