#include "agv.h"
#include "odometry_manager.h"
#include "vehicle.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <map>
#include <math.h>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry_manager_node");
  OdometryManager manager;
  manager.run();
  return 0;
}
