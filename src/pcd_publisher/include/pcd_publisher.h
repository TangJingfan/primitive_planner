#ifndef PCD_PUBLISHER_H
#define PCD_PUBLISHER_H

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

using PointT = pcl::PointXYZ;

class PcdPublisher {
public:
  PcdPublisher(ros::NodeHandle &nh_, const std::string &pcd_file_,
               double local_radius_, double scan_resolution_,
               double sample_step_);

private:
  ros::NodeHandle nh;

  // publish global pointcloud and local pointcloud
  ros::Publisher global_pcd_pub;
  std::unordered_map<std::string, ros::Publisher> local_pcd_pubs;
  pcl::PointCloud<PointT> global_cloud;
  pcl::KdTreeFLANN<PointT> kdtree;

  // manage robots, subscribe to odom
  std::vector<ros::Subscriber> odom_subs;
  std::mutex mutex;
  std::vector<std::string> names;
  std::unordered_map<std::string, Eigen::Vector3f> current_positions;
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg,
                    const std::string &name);

  // timer
  ros::Timer timer;
  void timerCallback(const ros::TimerEvent &);

  // lidar params
  double local_radius;
  double scan_resolution;
  double sample_step;
  double vertical_angle = 100.0; // maximum angle to look up
  double vertical_resolution;
};

#endif