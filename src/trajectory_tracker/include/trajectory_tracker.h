#ifndef TRAJECTORY_TRACKER_H
#define TRAJECTORY_TRACKER_H

#include "vehicle.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

class TrajectoryTracker {
public:
  TrajectoryTracker(ros::NodeHandle &nh_);

private:
  ros::NodeHandle nh;
  std::vector<std::string> robot_names;
  std::map<std::string, Vehicle> vehicles;

  void odom_callback(const nav_msgs::Odometry::ConstPtr &msg,
                     const std::string &name);
  void path_callback(const nav_msgs::Path::ConstPtr &msg,
                     const std::string &name);
  void adjust_callback(const std_msgs::String::ConstPtr &msg,
                       const std::string &name);
};

#endif // TRAJECTORY_TRACKER_H