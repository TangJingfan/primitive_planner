#include "vehicle.h"
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

Vehicle::Vehicle(const std::string &vehicle_name, ros::NodeHandle &nh,
                 double x_init, double y_init, double z_init, double theta_init,
                 bool use_model_flag, const std::string &mesh_res)
    : name(vehicle_name), x(x_init), y(y_init), z(z_init), theta(theta_init),
      vx(0.0), vy(0.0), vz(0.0), angular_velocity(0.0),
      use_model(use_model_flag), mesh_resource(mesh_res) {
  odom_pub = nh.advertise<nav_msgs::Odometry>("/" + name + "/odom", 10);
  marker_pub = nh.advertise<visualization_msgs::Marker>(
      "/" + name + "/vehicle_marker", 10);
  trajectory_pub = nh.advertise<visualization_msgs::Marker>(
      "/" + name + "/vehicle_trajectory", 10);

  trajectory_marker.header.frame_id = "map";
  trajectory_marker.header.stamp = ros::Time::now();
  trajectory_marker.ns = name + "_trajectory";
  trajectory_marker.id = 0;
  trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;
  trajectory_marker.action = visualization_msgs::Marker::ADD;
  trajectory_marker.pose.orientation.w = 1.0; // 无旋转

  trajectory_marker.color.r = 0.0;
  trajectory_marker.color.g = 0.0;
  trajectory_marker.color.b = 1.0;
  trajectory_marker.color.a = 1.0;

  trajectory_marker.scale.x = 0.05;
  trajectory_marker.points.clear();
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  trajectory_marker.points.push_back(p);
}

Vehicle::~Vehicle() {}