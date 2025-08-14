#ifndef PLANNER_H_
#define PLANNER_H_

#include <cmath>
#include <fstream>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

// 调整信息
enum STATE { IN, LEFT, RIGHT };

struct Path {
  int id;
  std::vector<pcl::PointXYZ> points;
};

using PointCloudRGB = pcl::PointCloud<pcl::PointXYZRGB>;

struct Robot {
  std::string name;
  ros::Subscriber odom_sub;
  geometry_msgs::Pose current_pose;
  // 订阅路径
  ros::Subscriber path_sub;
  ros::Publisher cloud_pub;
  ros::Publisher local_path_pub;
  nav_msgs::Path a_star_path;
  // 调整姿态
  ros::Publisher adjust_pub;
  // 记录目标点index
  int target_index;
  // 离散化路径
  std::vector<geometry_msgs::Point> discrete_path;
};

class PrimitivePlanner {
public:
  PrimitivePlanner(ros::NodeHandle nh_);

private:
  ros::NodeHandle nh;
  // 机器人
  std::string name;
  Robot r;
  // 订阅地图
  ros::Subscriber map_sub;
  nav_msgs::OccupancyGrid map, costmap;
  bool map_received;
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  // 路径回调
  void pathCallback(const nav_msgs::Path::ConstPtr &msg);
  // 订阅里程计
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  // 里程计定时检查
  ros::Timer timer;
  void timerCallback(const ros::TimerEvent &);

  // primitive的参数
  double extra_rotation_rad = 0.0;
  double front_angle_range = 0.0;
  double minimum_distance = 0.0;
  std::vector<Path> all_paths;
  std::vector<std::tuple<uint8_t, uint8_t, uint8_t>> color_table;
  std::vector<int> blocked_path_ids;
  // 检查是否可通行
  bool isPathCollisionFree(const std::vector<pcl::PointXYZ> &pts) const;
  // 路径坐标转换
  pcl::PointXYZ transformPoint(const pcl::PointXYZ &pt, double a, double tx,
                               double ty) const;
  // 坐标转换检查
  bool worldToGridSafe(double x, double y, int &mx, int &my) const;
  // 是否在机器人眼前
  STATE isInFov(double robot_x, double robot_y, double yaw, double goal_x,
                double goal_y);
};

#endif