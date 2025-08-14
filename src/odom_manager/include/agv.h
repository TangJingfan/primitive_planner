#ifndef AGV_H
#define AGV_H

#include "vehicle.h"
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

struct AGV : public Vehicle {
  AGV(const std::string &vehicle_name, ros::NodeHandle &nh, double x_init,
      double y_init, double theta_init, bool use_model_flag = false,
      const std::string &mesh_res = "");

  void updatePose(double dt) override;
  void publishData(const ros::Time &current_time,
                   tf2_ros::TransformBroadcaster &broadcaster) override;
};

#endif