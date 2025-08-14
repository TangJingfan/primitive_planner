#ifndef ODOMETRY_MANAGER_H
#define ODOMETRY_MANAGER_H

#include "agv.h"
#include "vehicle.h"
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

class OdometryManager {
public:
  OdometryManager();
  void run();

private:
  ros::NodeHandle nh;
  ros::Rate rate;
  std::map<std::string, std::shared_ptr<Vehicle>> vehicles;
  std::map<std::string, InitialStatus> vehicle_definitions;

  void cmdCallback(const geometry_msgs::Twist::ConstPtr &msg,
                   const std::string &vehicle_name);
};

#endif