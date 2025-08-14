#ifndef FINDER_H_
#define FINDER_H_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

struct Robot {
  std::string name;
  // 发布路径
  ros::Publisher path_pub;
  // 订阅里程计
  ros::Subscriber odom_sub;
  geometry_msgs::Pose current_pose;
};

struct Point2D {
  int x, y;
  // 为std::map设计一个比较函数
  bool operator<(const Point2D &other) const {
    if (x != other.x)
      return x < other.x;
    return y < other.y;
  }
  bool operator==(const Point2D &other) const {
    return x == other.x && y == other.y;
  }
};

// A Star 节点
struct Node2D {
  int x, y;
  double f, g, h;
  std::shared_ptr<Node2D> parent;
  Node2D(int x_, int y_, double g_ = 0.0)
      : x(x_), y(y_), g(g_), h(0.0), f(g + h), parent(nullptr){};
  int64_t to_key(int width) const {
    return static_cast<int64_t>(y) * width + x;
  };
};

// 构建min heap
struct NodeComparator {
  bool operator()(const Node2D &a, const Node2D &b) const { return a.f > b.f; };
};

// 订阅2D goal，发布A star路径
class PathFinder {
public:
  PathFinder(ros::NodeHandle nh_);

private:
  ros::NodeHandle nh;
  // 机器人
  std::string name;
  Robot r;
  // 订阅器
  ros::Subscriber goal_sub;
  ros::Subscriber map_sub;
  // 地图
  nav_msgs::OccupancyGrid map, costmap;
  bool map_received;
  // 目标点
  geometry_msgs::Point goal;
  // 邻居
  std::vector<std::pair<int, int>> neighbor_dirs;
  // 订阅终点
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  // 订阅地图
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  // 里程计回调
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  // 搜索路径
  bool aStarSearch(int start_x, int start_y, int goal_x, int goal_y,
                   std::vector<Point2D> &path);
  // 生成代价地图
  nav_msgs::OccupancyGrid
  generateCostmap(const nav_msgs::OccupancyGrid &original_map);
};

#endif