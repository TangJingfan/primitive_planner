#ifndef MAP_BUILDER_H_
#define MAP_BUILDER_H_

#include <memory>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <unordered_map>
#include <vector>

using PclCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = PclCloud::Ptr;

class MapBuilder {
public:
  MapBuilder(ros::NodeHandle nh_);

private:
  ros::NodeHandle nh;

  // subscribe to global pointcloud
  ros::Subscriber global_cloud_sub;
  void globalCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

  // manage robots, subscribe to odometry
  std::vector<std::string> robot_names;
  std::map<std::string, ros::Subscriber> odom_subs;
  void odomCallback(const nav_msgs::OdometryConstPtr &msg,
                    const std::string &robot_name);

  // map of ground truth
  ros::Publisher agv_map_pub, uav_map_pub;
  nav_msgs::OccupancyGrid agv_grid, uav_grid;
  bool map_processed;

  // map visible to robots
  ros::Publisher agv_vis_pub, uav_vis_pub;
  nav_msgs::OccupancyGrid agv_visible, uav_visible;
  std::shared_ptr<octomap::OcTree> octree_map;
  // set header for map
  void setupGridHeader(nav_msgs::OccupancyGrid &grid);

  // map params
  double resolution, origin_x, origin_y;
  int grid_size_x, grid_size_y;
};

#endif