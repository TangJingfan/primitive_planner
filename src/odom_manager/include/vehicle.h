#ifndef VEHICLE_H
#define VEHICLE_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

enum class VehicleType { AGV, UAV };

struct InitialStatus {
  double x_init;
  double y_init;
  double z_init;
  double theta_init;
  VehicleType type;
  bool use_model;
  std::string mesh_resource;
};

struct Vehicle {
  std::string name;
  double x, y, z, theta;
  double vx, vy, vz, angular_velocity;

  ros::Publisher odom_pub;
  ros::Publisher marker_pub;
  ros::Publisher trajectory_pub;

  ros::Subscriber cmd_vel_sub;

  visualization_msgs::Marker trajectory_marker;

  bool use_model;
  std::string mesh_resource;

  Vehicle(const std::string &vehicle_name, ros::NodeHandle &nh, double x_init,
          double y_init, double z_init, double theta_init,
          bool use_model_flag = false, const std::string &mesh_res = "");
  virtual ~Vehicle();

  virtual void updatePose(double dt) = 0;
  virtual void publishData(const ros::Time &current_time,
                           tf2_ros::TransformBroadcaster &broadcaster) = 0;
};

#endif