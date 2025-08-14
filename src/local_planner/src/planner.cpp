#include "planner.h"
#include <ros/package.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

PrimitivePlanner::PrimitivePlanner(ros::NodeHandle nh_)
    : nh(nh_), map_received(false) {
  // 机器人初始化
  name = "agv1";
  r.name = name;
  r.odom_sub = nh.subscribe("/" + name + "/odom", 10,
                            &PrimitivePlanner::odomCallback, this);
  r.path_sub = nh.subscribe("/" + name + "/path", 10,
                            &PrimitivePlanner::pathCallback, this);
  r.adjust_pub =
      nh.advertise<std_msgs::String>("/" + name + "/adjust_pose", 10, true);
  r.local_path_pub =
      nh.advertise<nav_msgs::Path>("/" + name + "/local_path", 10, true);
  r.cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(
      "/" + name + "/trajectory_cloud", 1);
  // 地图
  map_sub = nh.subscribe("/agv_map", 1, &PrimitivePlanner::mapCallback, this);
  // 定时器
  timer = nh.createTimer(ros::Duration(0.5), &PrimitivePlanner::timerCallback,
                         this);
  // 初始化规划器
  std::string path_file;
  double rotation_angle_deg;
  nh.param<std::string>("path_file", path_file,
                        "package://local_planner/paths/paths.txt");
  nh.param<double>("rotation_angle_deg", rotation_angle_deg, 0.0);
  extra_rotation_rad = rotation_angle_deg * M_PI / 180.0;
  nh.param<double>("front_angle", front_angle_range, 120);
  nh.param<double>("distance_tolerance", minimum_distance, 1.0);
  std::string package_path = ros::package::getPath("local_planner");
  if (package_path.empty()) {
    ROS_ERROR("Cannot find package: local_planner");
    return;
  }
  path_file = package_path + "/paths/paths.txt";
  // Load paths
  std::ifstream file(path_file);
  if (!file.is_open()) {
    ROS_ERROR("Cannot open path file: %s", path_file.c_str());
  } else {
    std::string line;
    while (std::getline(file, line)) {
      std::stringstream ss(line);
      Path p;
      ss >> p.id;
      double x, y;
      while (ss >> x >> y) {
        p.points.emplace_back(x, y, 0.0);
      }
      if (!p.points.empty()) {
        all_paths.push_back(p);
      }
    }
    ROS_INFO("Loaded %lu paths", all_paths.size());
  }

  color_table = {std::make_tuple(255, 0, 0),   std::make_tuple(0, 255, 0),
                 std::make_tuple(0, 0, 255),   std::make_tuple(255, 255, 0),
                 std::make_tuple(255, 0, 255), std::make_tuple(0, 255, 255),
                 std::make_tuple(255, 128, 0), std::make_tuple(128, 0, 255),
                 std::make_tuple(0, 128, 255), std::make_tuple(0, 0, 0)};
}

void PrimitivePlanner::mapCallback(
    const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  map = *msg;
  map_received = true;
}

void PrimitivePlanner::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  if (!msg) {
    return;
  }
  r.current_pose = msg->pose.pose;
}

void PrimitivePlanner::timerCallback(const ros::TimerEvent &) {
  if (r.discrete_path.empty() == true) {
    return;
  }

  // 计算机器人朝向
  double tx = r.current_pose.position.x;
  double ty = r.current_pose.position.y;
  double target_x = r.discrete_path[r.target_index].x;
  double target_y = r.discrete_path[r.target_index].y;
  double final_x = r.discrete_path.back().x;
  double final_y = r.discrete_path.back().y;
  double final_dist = std::hypot(final_x - tx, final_y - ty);

  // 打印距离信息（无论是否满足 < 0.5）
  // ROS_INFO("[PrimitivePlanner] Distance to goal: %.2f meters", final_dist);

  if (final_dist < 0.8) {
    nav_msgs::Path final_path;
    final_path.header.frame_id = "map";
    final_path.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped final_pose;
    final_pose.header = final_path.header;
    final_pose.pose.position.x = final_x;
    final_pose.pose.position.y = final_y;
    final_pose.pose.position.z = 0.0;
    final_pose.pose.orientation.w = 1.0;

    final_path.poses.push_back(final_pose);
    r.local_path_pub.publish(final_path);

    ROS_INFO_ONCE(
        "[PrimitivePlanner] Close to goal. Directly sending final goal pose.");
    return;
  }

  tf2::Quaternion q(r.current_pose.orientation.x, r.current_pose.orientation.y,
                    r.current_pose.orientation.z, r.current_pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  // ROS_INFO_STREAM("x: " << tx << " y: " << ty << " yaw: " << yaw);
  double distance_to_current_target = std::hypot(tx - target_x, ty - target_y);
  if (distance_to_current_target < 2 &&
      r.target_index < r.discrete_path.size() - 1) {
    r.target_index++;
  }

  // 计算目标点是否在机器人的FOV内，若在则继续，若不在则先旋转
  // ROS_INFO_STREAM(
  //     "is goal in FOV: " << isInFov(tx, ty, yaw, target_x, target_y));
  if (isInFov(tx, ty, yaw, target_x, target_y) == IN) {
    std_msgs::String msg;
    msg.data = "IN";
    r.adjust_pub.publish(msg);
  } else if (isInFov(tx, ty, yaw, target_x, target_y) == LEFT) {
    std_msgs::String msg;
    msg.data = "LEFT";
    r.adjust_pub.publish(msg);
    return;
  } else if (isInFov(tx, ty, yaw, target_x, target_y) == RIGHT) {
    std_msgs::String msg;
    msg.data = "RIGHT";
    r.adjust_pub.publish(msg);
    return;
  }
  // 清空不合格路径id
  blocked_path_ids.clear();
  // 初始化cloud
  PointCloudRGB::Ptr cloud(new PointCloudRGB);
  cloud->header.frame_id = "map";
  cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
  // 保存所有可行路径和颜色
  std::vector<std::vector<pcl::PointXYZ>> all_valid_paths;
  std::vector<std::tuple<uint8_t, uint8_t, uint8_t>> path_colors;
  // 保存最佳路径
  double min_dist = std::numeric_limits<double>::max();
  std::vector<pcl::PointXYZ> best_transformed_path;
  uint8_t best_r = 0, best_g = 0, best_b = 0;
  int best_path_id = -1;
  for (size_t i = 0; i < all_paths.size(); ++i) {
    const auto &path = all_paths[i];

    std::vector<pcl::PointXYZ> transformed_points;

    // ====== 第一步：路径旋转 + 坐标变换 ======
    for (const auto &pt : path.points) {
      pcl::PointXYZ pt_rotated(pt.x * std::cos(extra_rotation_rad) -
                                   pt.y * std::sin(extra_rotation_rad),
                               pt.x * std::sin(extra_rotation_rad) +
                                   pt.y * std::cos(extra_rotation_rad),
                               pt.z);

      pcl::PointXYZ pt_world = transformPoint(pt_rotated, yaw, tx, ty);
      transformed_points.push_back(pt_world);
    }

    // ====== 第二步：路径截断（根据到目标的距离） ======
    double gx = tx - target_x;
    double gy = ty - target_y;
    double dist_to_goal = std::hypot(gx, gy);

    if (dist_to_goal < 6.0) {
      std::vector<pcl::PointXYZ> truncated_points;
      double accum_dist = 0.0;
      double max_keep_dist = 1.5 + 6.0 * std::tanh(dist_to_goal / 8.0);

      for (size_t j = 1; j < transformed_points.size(); ++j) {
        double dx = transformed_points[j].x - transformed_points[j - 1].x;
        double dy = transformed_points[j].y - transformed_points[j - 1].y;
        double step_dist = std::hypot(dx, dy);
        accum_dist += step_dist;

        truncated_points.push_back(transformed_points[j - 1]);
        if (accum_dist >= max_keep_dist)
          break;
      }

      transformed_points = truncated_points;
    }

    // ====== 第三步：碰撞检测 ======
    bool is_blocked = false;
    for (const auto &pt : transformed_points) {
      int mx, my;
      if (!worldToGridSafe(pt.x, pt.y, mx, my)) {
        is_blocked = true;
        break;
      }
      int idx = my * map.info.width + mx;
      if (idx < 0 || idx >= static_cast<int>(map.data.size()) ||
          map.data[idx] == 100) {
        is_blocked = true;
        break;
      }
    }

    if (is_blocked || transformed_points.empty()) {
      blocked_path_ids.push_back(path.id);
      continue;
    }

    // ====== 可行路径记录与可视化 ======
    all_valid_paths.push_back(transformed_points);
    path_colors.push_back(color_table[i % color_table.size()]);

    // ====== 更新最佳路径 ======
    const auto &end = transformed_points.back();
    double dx = end.x - target_x;
    double dy = end.y - target_y;
    double dist = std::hypot(dx, dy);
    if (dist < min_dist) {
      min_dist = dist;
      best_transformed_path = transformed_points;
      std::tie(best_r, best_g, best_b) = color_table[i % color_table.size()];
      best_path_id = path.id;
    }
  }

  // 可行路径可视化为点云
  for (size_t i = 0; i < all_valid_paths.size(); ++i) {
    const auto &path = all_valid_paths[i];
    auto [r, g, b] = path_colors[i];

    for (const auto &pt : path) {
      pcl::PointXYZRGB pt_rgb;
      pt_rgb.x = pt.x;
      pt_rgb.y = pt.y;
      pt_rgb.z = pt.z;
      pt_rgb.r = r;
      pt_rgb.g = g;
      pt_rgb.b = b;
      cloud->points.push_back(pt_rgb);
    }
  }
  r.cloud_pub.publish(*cloud);
  // 最优路径发布为 nav_msgs::Path
  if (!best_transformed_path.empty()) {
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();
    ROS_INFO_STREAM("Best Path ID: " << best_path_id);
    double goal_x = r.a_star_path.poses.back().pose.position.x;
    double goal_y = r.a_star_path.poses.back().pose.position.y;
    for (const auto &pt : best_transformed_path) {
      // TODO: 判断当前点是否接近目标点，若接近则不继续发
      double dx = pt.x - goal_x;
      double dy = pt.y - goal_y;
      double dist_check = std::hypot(dx, dy);
      if (dist_check < minimum_distance) {
        break;
      }
      geometry_msgs::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = pt.x;
      pose.pose.position.y = pt.y;
      pose.pose.position.z = pt.z;
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
      // 打印每个路径点
      // ROS_INFO("  (%.2f, %.2f)", pt.x, pt.y);
    }
    r.local_path_pub.publish(path_msg);
    // ROS_INFO("Published nav_msgs::Path with %lu poses. Goal distance: %.2f",
    //          path_msg.poses.size(), min_dist);
  } else {
    ROS_WARN_THROTTLE(2.0, "No valid path near goal to publish.");
  }
}

void PrimitivePlanner::pathCallback(const nav_msgs::Path::ConstPtr &msg) {
  if (msg->poses.empty() == true) {
    return;
  }
  r.discrete_path.clear();
  r.a_star_path = *msg;
  r.target_index = 0;
  for (int i = 0; i < msg->poses.size(); i++) {
    if (6 * (i + 1) >= msg->poses.size()) {
      r.discrete_path.push_back(msg->poses.back().pose.position);
      break;
    }
    r.discrete_path.push_back(msg->poses[3 * (i + 1)].pose.position);
  }
}

pcl::PointXYZ PrimitivePlanner::transformPoint(const pcl::PointXYZ &pt,
                                               double a, double tx,
                                               double ty) const {
  double c = std::cos(a), s = std::sin(a);
  double xr = pt.x * c - pt.y * s;
  double yr = pt.x * s + pt.y * c;
  return pcl::PointXYZ(xr + tx, yr + ty, pt.z);
}

bool PrimitivePlanner::isPathCollisionFree(
    const std::vector<pcl::PointXYZ> &pts) const {
  for (const auto &pt : pts) {
    int mx, my;
    if (!worldToGridSafe(pt.x, pt.y, mx, my))
      return false;
    int idx = my * map.info.width + mx;
    if (idx < 0 || idx >= static_cast<int>(map.data.size()))
      return false;
    if (map.data[idx] == 100)
      return false;
  }
  return true;
}

STATE PrimitivePlanner::isInFov(double robot_x, double robot_y, double yaw,
                                double goal_x, double goal_y) {
  double dx = goal_x - robot_x;
  double dy = goal_y - robot_y;
  double dist = std::hypot(dx, dy);

  // Handle case where robot is at the goal
  if (dist < 0.5) {
    ROS_INFO_STREAM("Robot is at the goal point.");
    return IN;
  }

  // Compute angle to goal using atan2
  double goal_angle = std::atan2(dy, dx);
  // ROS_INFO_STREAM("Goal_x: " << goal_x << " Goal_y: " << goal_y);
  // ROS_INFO_STREAM("Goal angle: " << goal_angle << " radians");

  // Normalize angles to [0, 2π)
  auto normalize_angle = [](double angle) {
    while (angle >= 2.0 * M_PI)
      angle -= 2.0 * M_PI;
    while (angle < 0.0)
      angle += 2.0 * M_PI;
    return angle;
  };
  yaw = normalize_angle(yaw);
  goal_angle = normalize_angle(goal_angle);

  // Compute FOV boundaries
  double fov_half = (front_angle_range / 180.0) * M_PI / 2.0;
  double fov_min = normalize_angle(yaw - fov_half);
  double fov_max = normalize_angle(yaw + fov_half);

  // Compute angular difference for left/right decision
  double angle_diff = goal_angle - yaw;
  while (angle_diff > M_PI)
    angle_diff -= 2.0 * M_PI;
  while (angle_diff < -M_PI)
    angle_diff += 2.0 * M_PI;

  ROS_INFO_STREAM("Normalized yaw: " << yaw << " rad");
  ROS_INFO_STREAM("Normalized goal_angle: " << goal_angle << " rad");
  ROS_INFO_STREAM("Raw angle diff: " << angle_diff << " rad");

  // Check if goal is in FOV
  bool in_fov;
  if (fov_min < fov_max) {
    in_fov = (goal_angle >= fov_min && goal_angle <= fov_max);
  } else {
    in_fov = (goal_angle >= fov_min || goal_angle <= fov_max);
  }

  if (in_fov) {
    return IN;
  } else {
    // Return LEFT if goal is to the right (positive angle_diff), RIGHT if to
    // the left
    return (angle_diff > 0) ? LEFT : RIGHT;
  }
}

bool PrimitivePlanner::worldToGridSafe(double x, double y, int &mx,
                                       int &my) const {
  if (map.info.resolution <= 0.0 || map.info.width <= 0 || map.info.height <= 0)
    return false;
  double snapped_x = std::round(x / map.info.resolution) * map.info.resolution;
  double snapped_y = std::round(y / map.info.resolution) * map.info.resolution;

  // Convert to grid coordinates
  mx = static_cast<int>((snapped_x + (double)map.info.width / 2) /
                        map.info.resolution);
  my = static_cast<int>((snapped_y + (double)map.info.height / 2) /
                        map.info.resolution);
  return mx >= 0 && mx < map.info.width && my >= 0 && my < map.info.height;
}