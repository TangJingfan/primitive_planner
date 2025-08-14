#include "map_builder.h"

MapBuilder::MapBuilder(ros::NodeHandle nh_) : nh(nh_), map_processed(false) {
  // initialize map params
  nh.getParam("resolution", resolution);
  nh.getParam("size_x", grid_size_x);
  nh.getParam("size_y", grid_size_y);
  origin_x = -(grid_size_x * resolution) / 2.0f;
  origin_y = -(grid_size_y * resolution) / 2.0f;

  // initialize OctoMap
  ROS_INFO("[Map Builder]origin_x: %.2f, origin_y: %.2f, resolution: %.4f",
           origin_x, origin_y, resolution);
  octree_map = std::make_shared<octomap::OcTree>(resolution);

  // initialize map of ground truth
  setupGridHeader(agv_grid);
  setupGridHeader(uav_grid);
  agv_grid.data.assign(grid_size_x * grid_size_y, 0);
  uav_grid.data.assign(grid_size_x * grid_size_y, 0);

  // initialize map visible to robots
  setupGridHeader(agv_visible);
  setupGridHeader(uav_visible);
  agv_visible.data.assign(grid_size_x * grid_size_y, -1);
  uav_visible.data.assign(grid_size_x * grid_size_y, -1);

  // subscribe to global pointcloud
  global_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(
      "/global_pcd", 1, &MapBuilder::globalCloudCallback, this);

  // subscribe to odom
  robot_names = {"agv1"};
  for (auto &name : robot_names) {
    odom_subs[name] = nh.subscribe<nav_msgs::Odometry>(
        "/" + name + "/odom", 10,
        std::bind(&MapBuilder::odomCallback, this, std::placeholders::_1,
                  name));
  }

  // publish map of ground truth
  agv_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/agv_grid_map", 1, true);
  uav_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/uav_grid_map", 1, true);
  // publish map visible to robots
  agv_vis_pub = nh.advertise<nav_msgs::OccupancyGrid>("/agv_map", 10, true);
  uav_vis_pub = nh.advertise<nav_msgs::OccupancyGrid>("/uav_map", 10, true);

  // initialize map builder node
  ROS_INFO("[Map Builder]node initialized.");
}

void MapBuilder::setupGridHeader(nav_msgs::OccupancyGrid &grid) {
  grid.header.frame_id = "map";
  grid.info.resolution = resolution;
  grid.info.width = grid_size_x;
  grid.info.height = grid_size_y;
  grid.info.origin.position.x = origin_x;
  grid.info.origin.position.y = origin_y;
  grid.info.origin.orientation.w = 1.0;
}

void MapBuilder::globalCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr &msg) {
  // if map is processed, return
  if (map_processed)
    return;
  map_processed = true;

  // initialize pointcloud
  PointCloudPtr cloud(new PclCloud);
  pcl::fromROSMsg(*msg, *cloud);

  // modify octomap
  octomap::Pointcloud octo_cloud;
  for (auto &pt : cloud->points) {
    if (pt.z >= 0.3 && pt.z <= 2.5)
      octo_cloud.push_back(pt.x, pt.y, pt.z);
  }
  octomap::point3d origin_pt(0, 0, 0.0);
  octree_map->insertPointCloud(octo_cloud, origin_pt);
  double min_x, min_y, min_z, max_x, max_y, max_z;
  octree_map->getMetricMin(min_x, min_y, min_z);
  octree_map->getMetricMax(max_x, max_y, max_z);
  ROS_INFO("[Map Builder]octoMap bounds: x=[%.2f, %.2f], y=[%.2f, %.2f], "
           "z=[%.2f,%.2f]",
           min_x, max_x, min_y, max_y, min_z, max_z);

  // generate map of ground truth
  for (int y = 0; y < grid_size_y; ++y) {
    for (int x = 0; x < grid_size_x; ++x) {
      int idx = y * grid_size_x + x;
      double wx = origin_x + (x + 0.5) * resolution;
      double wy = origin_y + (y + 0.5) * resolution;
      if (wx < min_x || wx > max_x || wy < min_y || wy > max_y) {
        continue;
      }
      bool low_occ = false, high_occ = false;
      octomap::point3d p_low(wx, wy, 0.5), p_high(wx, wy, 1.5);
      auto n1 = octree_map->search(p_low);
      if (n1 && octree_map->isNodeOccupied(*n1))
        low_occ = true;
      auto n2 = octree_map->search(p_high);
      if (n2 && octree_map->isNodeOccupied(*n2))
        high_occ = true;
      if (low_occ && high_occ) {
        agv_grid.data[idx] = 100;
        uav_grid.data[idx] = 100;
      } else if (low_occ) {
        agv_grid.data[idx] = 100;
        uav_grid.data[idx] = 0;
      } else if (high_occ) {
        agv_grid.data[idx] = 100;
        uav_grid.data[idx] = 100;
      } else {
        agv_grid.data[idx] = 0;
        uav_grid.data[idx] = 0;
      }
    }
  }

  // publish map of fround truth
  agv_grid.header.stamp = ros::Time::now();
  uav_grid.header.stamp = ros::Time::now();
  agv_map_pub.publish(agv_grid);
  uav_map_pub.publish(uav_grid);
  ROS_INFO("[Map Builder]map of ground truth generated.");
}

void MapBuilder::odomCallback(const nav_msgs::OdometryConstPtr &msg,
                              const std::string &robot_name) {
  // record position
  double rx = msg->pose.pose.position.x;
  double ry = msg->pose.pose.position.y;

  // if map is processed, return
  if (!map_processed)
    return;

  // params for ray simulation
  const int num_rays = 120;
  const double max_range = 12.0;

  // simulate lidar
  for (int i = 0; i < num_rays; ++i) {
    double ang = 2.0 * M_PI * i / num_rays;
    double dx = cos(ang), dy = sin(ang);
    for (double r = 0; r <= max_range; r += 1.0) {
      double ix = rx + dx * r;
      double iy = ry + dy * r;
      int gx = static_cast<int>((ix - origin_x) / resolution);
      int gy = static_cast<int>((iy - origin_y) / resolution);
      if (gx < 0 || gx >= grid_size_x || gy < 0 || gy >= grid_size_y)
        break;
      int idx = gy * grid_size_x + gx;

      // check that type of robot
      bool is_agv = robot_name.find("agv") != std::string::npos;

      if (is_agv) {
        // set obstacle for agv
        if (agv_grid.data[idx] == 100) {
          agv_visible.data[idx] = 100;

          if (uav_grid.data[idx] == 100) {
            uav_visible.data[idx] = 100;
          } else {
            uav_visible.data[idx] = 0;
          }
          break;
        } else {
          agv_visible.data[idx] = 0;
          uav_visible.data[idx] = 0;
        }
      } else {
        // consider obstacle for uav
        if (uav_grid.data[idx] == 100) {
          agv_visible.data[idx] = 100;
          uav_visible.data[idx] = 100;
          break;
        } else {
          uav_visible.data[idx] = 0;
          if (agv_grid.data[idx] == 100) {
            agv_visible.data[idx] = 100;
          } else {
            agv_visible.data[idx] = 0;
          }
        }
      }
    }
  }

  // publish map visible to robots
  agv_visible.header.stamp = ros::Time::now();
  uav_visible.header.stamp = ros::Time::now();
  agv_vis_pub.publish(agv_visible);
  uav_vis_pub.publish(uav_visible);
}
