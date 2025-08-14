#include "pcd_publisher.h"
#include <Eigen/Dense>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <unordered_map>

PcdPublisher::PcdPublisher(ros::NodeHandle &nh, const std::string &pcd_file_,
                           double local_radius_, double scan_resolution_,
                           double sample_step_)
    : nh(nh), local_radius(local_radius_), scan_resolution(scan_resolution_),
      sample_step(sample_step_) {
  // read pointcloud file
  if (pcl::io::loadPCDFile<PointT>(pcd_file_, global_cloud) == -1) {
    ROS_ERROR("[Pcd Publisher]couldn't read PCD file: %s", pcd_file_.c_str());
    ros::shutdown();
  }
  ROS_INFO("[Pcd Publisher]loaded %lu points from %s",
           global_cloud.points.size(), pcd_file_.c_str());

  // publish global pointcloud
  global_pcd_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/global_pcd", 1, true);

  // subscribe to odom
  names = {"agv1"};
  for (const auto &name : names) {
    // E.g: /agv1/odom -> agv1_odom_local_pcd
    std::string local_topic = "/" + name + "/local_pcd";
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(local_topic, 1);
    local_pcd_pubs[name] = pub;
    ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>(
        "/" + name + "/odom", 10,
        std::bind(&PcdPublisher::odomCallback, this, std::placeholders::_1,
                  name));
    ROS_INFO("[Pcd Publisher]subscribed to %s", name.c_str());
    odom_subs.push_back(sub);
  }

  // initialize kdtree
  kdtree.setInputCloud(global_cloud.makeShared());

  // initialize timer
  timer = nh.createTimer(ros::Duration(1), &PcdPublisher::timerCallback, this);
}

void PcdPublisher::odomCallback(const nav_msgs::Odometry::ConstPtr &msg,
                                const std::string &name) {
  // update position
  Eigen::Vector3f position(msg->pose.pose.position.x, msg->pose.pose.position.y,
                           msg->pose.pose.position.z);
  std::lock_guard<std::mutex> lock(mutex);
  current_positions[name] = position;
}

void PcdPublisher::timerCallback(const ros::TimerEvent &) {
  // publish global pointcloud
  sensor_msgs::PointCloud2 global_msg;
  pcl::toROSMsg(global_cloud, global_msg);
  global_msg.header.stamp = ros::Time::now();
  global_msg.header.frame_id = "map";
  global_pcd_pub.publish(global_msg);

  // publish local pointcloud
  std::lock_guard<std::mutex> lock(mutex);
  for (const auto &name : names) {
    auto it = current_positions.find(name);
    if (it != current_positions.end() && it->second.size() > 0) {
      Eigen::Vector3f position = it->second;

      // create local cloud
      pcl::PointCloud<PointT>::Ptr local_cloud(new pcl::PointCloud<PointT>());
      for (float angle = 0.0f; angle < 360.0f;
           angle += static_cast<float>(scan_resolution)) {
        float yaw_rad = angle * static_cast<float>(M_PI) / 180.0f;
        for (float v_angle = -60.0f;
             v_angle <= static_cast<float>(vertical_angle); v_angle += 10.0f) {
          float pitch_rad = v_angle * static_cast<float>(M_PI) / 180.0f;
          Eigen::Vector3f ray_direction(std::cos(yaw_rad) * std::cos(pitch_rad),
                                        std::sin(yaw_rad) * std::cos(pitch_rad),
                                        std::sin(pitch_rad));
          ray_direction.normalize();

          // along the ray, sample
          for (float t = sample_step; t <= local_radius;) {
            Eigen::Vector3f sample_point = position + t * ray_direction;
            PointT search_point;
            search_point.x = sample_point[0];
            search_point.y = sample_point[1];
            search_point.z = sample_point[2];
            std::vector<int> point_idx;
            std::vector<float> point_dist;
            int num_neighbors = kdtree.radiusSearch(search_point, 12.0f,
                                                    point_idx, point_dist, 250);
            if (num_neighbors > 0) {
              PointT collision_point;
              for (int i = 0; i < num_neighbors; ++i) {
                collision_point = global_cloud.points[point_idx[i]];
                local_cloud->points.push_back(collision_point);
              }
              int num_noise_points = 0;
              float noise_range = 0.1f;
              for (int i = 0; i < num_noise_points; ++i) {
                PointT noise_point;
                noise_point.x =
                    collision_point.x +
                    ((rand() / static_cast<float>(RAND_MAX)) * 2 - 1) *
                        noise_range;
                noise_point.y =
                    collision_point.y +
                    ((rand() / static_cast<float>(RAND_MAX)) * 2 - 1) *
                        noise_range;
                noise_point.z =
                    collision_point.z +
                    ((rand() / static_cast<float>(RAND_MAX)) * 2 - 1) *
                        noise_range;
                local_cloud->points.push_back(noise_point);
              }
              // when collision found, break
              break;
            } else {
            }
            // add step
            t += 4.0f;
          } // for t
        }   // for v_angle
      }     // for angle

      // publish local pointcloud
      if (local_cloud->points.size() > 0) {
        sensor_msgs::PointCloud2 local_msg;
        pcl::toROSMsg(*local_cloud, local_msg);
        local_msg.header.stamp = ros::Time::now();
        local_msg.header.frame_id = "map";
        local_pcd_pubs[name].publish(local_msg);
      } else {
      }
    } else {
    }
  }
}