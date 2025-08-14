#include "agv.h"
#include "vehicle.h"
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

AGV::AGV(const std::string &vehicle_name, ros::NodeHandle &nh, double x_init,
         double y_init, double theta_init, bool use_model_flag,
         const std::string &mesh_res)
    : Vehicle::Vehicle(vehicle_name, nh, x_init, y_init, 0.3, theta_init,
                       use_model_flag, mesh_res) {}

void AGV::updatePose(double dt) {
  double delta_x = (vx * cos(theta) - vy * sin(theta)) * dt;
  double delta_y = (vx * sin(theta) + vy * cos(theta)) * dt;
  double delta_z = 0.0;

  // update pose
  x += delta_x;
  y += delta_y;
  theta += angular_velocity * dt;
  theta = fmod(theta + M_PI, 2 * M_PI) - M_PI;

  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  trajectory_marker.points.push_back(p);
  if (trajectory_marker.points.size() > 1000) {
    trajectory_marker.points.erase(trajectory_marker.points.begin());
  }
}

void AGV::publishData(const ros::Time &current_time,
                      tf2_ros::TransformBroadcaster &broadcaster) {
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "map";
  odom.child_frame_id = name + "_base_link";

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = z;

  tf2::Quaternion q_tf2;
  q_tf2.setRPY(0, 0, theta);
  geometry_msgs::Quaternion q = tf2::toMsg(q_tf2);
  odom.pose.pose.orientation = q;

  for (int i = 0; i < 36; i++) {
    odom.pose.covariance[i] = 0.0;
    odom.twist.covariance[i] = 0.0;
  }

  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.linear.z = 0.0;
  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = angular_velocity;

  odom_pub.publish(odom);

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "map";
  odom_trans.child_frame_id = name + "_base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = z;
  odom_trans.transform.rotation = q;

  broadcaster.sendTransform(odom_trans);

  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = name + "_marker";
  marker.id = 0;

  if (use_model && !mesh_resource.empty()) {
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = mesh_resource;
    marker.mesh_use_embedded_materials = true;
  } else {
    marker.type = visualization_msgs::Marker::ARROW;
  }

  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation = q;

  if (use_model && !mesh_resource.empty()) {
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
  } else {
    marker.scale.x = 1.0;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
  }

  marker_pub.publish(marker);
  trajectory_pub.publish(trajectory_marker);
}