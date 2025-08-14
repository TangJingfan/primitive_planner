#include "trajectory_tracker.h"
#include "vehicle.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_tracker_node");
  ros::NodeHandle nh("~");

  TrajectoryTracker tracker(nh);
  ros::spin();
  return 0;
}