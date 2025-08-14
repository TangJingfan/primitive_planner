#include "path_finder.h"
#include <nav_msgs/Path.h>
#include <queue>
#include <unordered_map>
#include <unordered_set>

PathFinder::PathFinder(ros::NodeHandle nh_) : nh(nh_), map_received(false) {
  // 机器人初始化
  name = "agv1";
  r.name = name;
  r.odom_sub =
      nh.subscribe("/" + name + "/odom", 10, &PathFinder::odomCallback, this);
  r.path_pub = nh.advertise<nav_msgs::Path>("/" + name + "/path", 10);
  // 地图
  map_sub = nh.subscribe("/agv_map", 1, &PathFinder::mapCallback, this);
  // 目标点
  goal_sub = nh.subscribe("/move_base_simple/goal", 10,
                          &PathFinder::goalCallback, this);
  // 初始化邻居
  neighbor_dirs = {{1, 0}, {-1, 0}, {0, 1},   {0, -1},
                   {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};
}

void PathFinder::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  map = *msg;
  map_received = true;
  costmap = generateCostmap(map);
}

void PathFinder::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  goal = msg->pose.position;
  // 搜寻a star路径
  nav_msgs::Path a_star_path;
  a_star_path.header.frame_id = "map";
  a_star_path.header.stamp = ros::Time::now();
  // 坐标转换
  // Snap world coordinates to grid cell centers
  double snapped_x =
      std::round(r.current_pose.position.x / map.info.resolution) *
      map.info.resolution;
  double snapped_y =
      std::round(r.current_pose.position.y / map.info.resolution) *
      map.info.resolution;
  double snapped_goal_x =
      std::round(goal.x / map.info.resolution) * map.info.resolution;
  double snapped_goal_y =
      std::round(goal.y / map.info.resolution) * map.info.resolution;

  // Convert to grid coordinates
  int robot_x = static_cast<int>((snapped_x + (double)map.info.width / 2) /
                                 map.info.resolution);
  int robot_y = static_cast<int>((snapped_y + (double)map.info.height / 2) /
                                 map.info.resolution);
  int goal_x = static_cast<int>((snapped_goal_x + (double)map.info.width / 2) /
                                map.info.resolution);
  int goal_y = static_cast<int>((snapped_goal_y + (double)map.info.height / 2) /
                                map.info.resolution);
  ROS_INFO_STREAM("Try to find a path from (" << robot_x << ", " << robot_y
                                              << ") to (" << goal_x << ", "
                                              << goal_y << ").");
  // 检查是否可通行
  std::vector<Point2D> a_star_path_in_point;
  if (!aStarSearch(robot_x, robot_y, goal_x, goal_y, a_star_path_in_point)) {
    ROS_ERROR("No acceptable path for robot!");
    return;
  }
  ROS_INFO_STREAM("map resolution: " << map.info.resolution);
  ROS_INFO_STREAM("width: " << map.info.width);
  ROS_INFO_STREAM("height: " << map.info.height);
  ROS_INFO_STREAM("origin x: " << (-(double)map.info.width / 2));
  ROS_INFO_STREAM("origin y: " << (-(double)map.info.height / 2));
  // 坐标转换
  for (int i = 0; i < a_star_path_in_point.size(); i++) {
    geometry_msgs::PoseStamped pose;
    pose.header = a_star_path.header;
    pose.pose.position.x = (-(double)map.info.width / 2) +
                           a_star_path_in_point[i].x * map.info.resolution +
                           0.5;
    pose.pose.position.y = (-(double)map.info.height / 2) +
                           a_star_path_in_point[i].y * map.info.resolution +
                           0.5;
    pose.pose.position.z = 0.0;
    ROS_INFO_STREAM("(" << a_star_path_in_point[i].x << ", "
                        << a_star_path_in_point[i].y << ") ===> ("
                        << pose.pose.position.x << ", " << pose.pose.position.y
                        << ").");
    pose.pose.orientation.w = 1.0;
    a_star_path.poses.push_back(pose);
  }
  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header = a_star_path.header;
  goal_pose.pose.position.x = goal.x;
  goal_pose.pose.position.y = goal.y;
  goal_pose.pose.position.z = 0.0;
  goal_pose.pose.orientation.w = 1.0;
  a_star_path.poses.push_back(goal_pose);
    // 发布
  r.path_pub.publish(a_star_path);
}

void PathFinder::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  if (!msg) {
    return;
  }
  r.current_pose = msg->pose.pose;
}


bool PathFinder::aStarSearch(int start_x, int start_y, int goal_x, int goal_y,
                             std::vector<Point2D> &path) {
  int width = map.info.width;
  int height = map.info.height;
  if (start_x < 0 || start_x >= width || start_y < 0 || start_y >= height ||
      goal_x < 0 || goal_x >= width || goal_y < 0 || goal_y >= height) {
    ROS_WARN("%s: Start (%d, %d) or Goal (%d, %d) out of bounds", name.c_str(),
             start_x, start_y, goal_x, goal_y);
    return false;
  }

  int start_idx = start_y * width + start_x;
  int goal_idx = goal_y * width + goal_x;
  if (map.data[start_idx] >= 50 || map.data[goal_idx] >= 50) {
    ROS_WARN("%s: Start (%d, %d) or Goal (%d, %d) in obstacle", name.c_str(),
             start_x, start_y, goal_x, goal_y);
    return false;
  }

  std::priority_queue<Node2D, std::vector<Node2D>, NodeComparator> open;
  std::unordered_set<int64_t> closed;
  std::unordered_map<int64_t, std::shared_ptr<Node2D>> nodes;
  std::unordered_map<int64_t, double> gtable;

  Node2D start(start_x, start_y, 0.0);
  start.h = std::hypot(start_x - goal_x, start_y - goal_y);
  start.f = start.g + start.h;
  int64_t start_key = start.to_key(width);
  nodes[start_key] = std::make_shared<Node2D>(start);
  gtable[start_key] = start.g;
  open.push(start);

  while (!open.empty()) {
    Node2D current = open.top();
    open.pop();
    int64_t current_key = current.to_key(width);

    if (closed.find(current_key) != closed.end()) {
      continue;
    }
    closed.insert(current_key);

    if (current.x == goal_x && current.y == goal_y) {
      std::vector<Point2D> rev;
      std::shared_ptr<Node2D> node = nodes[current_key];
      while (node) {
        rev.push_back({node->x, node->y});
        node = node->parent;
      }
      std::reverse(rev.begin(), rev.end());
      path = rev;
      return true;
    }

    for (const auto &dir : neighbor_dirs) {
      int nx = current.x + dir.first;
      int ny = current.y + dir.second;
      if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

      int idx = ny * width + nx;
      if (costmap.data[idx] >= 80 || map.data[idx] < 0) continue;

      // 对角线检查
      bool is_diagonal = (dir.first != 0 && dir.second != 0);
      bool can_move = true;
      if (is_diagonal) {
        int dx = dir.first, dy = dir.second;
        int n1x = current.x + dx, n1y = current.y;
        int n2x = current.x, n2y = current.y + dy;
        if (!(n1x >= 0 && n1x < width && n1y >= 0 && n1y < height && map.data[n1y * width + n1x] < 50))
          can_move = false;
        if (!(n2x >= 0 && n2x < width && n2y >= 0 && n2y < height && map.data[n2y * width + n2x] < 50))
          can_move = false;
      }
      if (!can_move) continue;

      int64_t neighbor_key = ny * width + nx;
      if (closed.find(neighbor_key) != closed.end()) continue;

      double step = (is_diagonal ? 1.414 : 1.0);
      if (costmap.data[idx] >= 75) step += 30.0;

      double tentative_g = current.g + step;

      if (gtable.find(neighbor_key) == gtable.end() || tentative_g < gtable[neighbor_key]) {
        gtable[neighbor_key] = tentative_g;

        Node2D neighbor(nx, ny, tentative_g);
        neighbor.h = std::hypot(nx - goal_x, ny - goal_y);
        neighbor.f = neighbor.g + neighbor.h;

        if (nodes.find(current_key) == nodes.end())
          nodes[current_key] = std::make_shared<Node2D>(current);

        neighbor.parent = nodes[current_key];
        nodes[neighbor_key] = std::make_shared<Node2D>(neighbor);

        open.push(neighbor);
      }
    }
  }

  ROS_WARN("%s: No feasible path from (%d, %d) to (%d, %d)", name.c_str(),
           start_x, start_y, goal_x, goal_y);
  return false;
}

nav_msgs::OccupancyGrid
PathFinder::generateCostmap(const nav_msgs::OccupancyGrid &original_map) {
  // 生成输出用的地图
  // 复制所有信息
  nav_msgs::OccupancyGrid new_map = original_map;
  new_map.data = original_map.data;

  // 检查输入是否有效
  if (original_map.info.width <= 0 || original_map.info.height <= 0 ||
      original_map.data.empty()) {
    ROS_WARN("Invalid input map: empty or invalid dimensions");
    return new_map;
  }

  // 修改地图
  for (int y = 0; y < original_map.info.height; y++) {
    for (int x = 0; x < original_map.info.width; x++) {
      int index = y * original_map.info.width + x;
      // 如果被占据
      if (original_map.data[index] == 100) {
        // 8邻居检查
        for (int dy = -1; dy <= 1; dy++) {
          for (int dx = -1; dx <= 1; dx++) {
            int new_y = y + dy;
            int new_x = x + dx;
            // 检查边界
            if (new_x >= 0 && new_x < original_map.info.width && new_y >= 0 &&
                new_y < original_map.info.height) {
              int new_index = new_y * original_map.info.width + new_x;
              // 修改free cell
              if (new_map.data[new_index] != 100 &&
                  new_map.data[new_index] == 0) {
                new_map.data[new_index] = 75;
              }
            }
          }
        }
      }
    }
  }

  return new_map;
}
