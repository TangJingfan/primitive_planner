#include "pcd_publisher.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcd_publisher_node");
  ros::NodeHandle nh;

  // read param of pointcloud file
  std::string pcd_file;
  nh.param<std::string>("pcd_file", pcd_file,
                        "package://pcd_publisher/pcd_file/output.pcd");
  if (pcd_file.find("package://") == 0) {
    std::string package_name = "pcd_publisher";
    std::string package_path = ros::package::getPath(package_name);
    if (package_path.empty()) {
      ROS_ERROR("[Pcd Publisher]could not find package: %s",
                package_name.c_str());
    } else {
      pcd_file.replace(0, package_name.length() + 10, package_path);
    }
  }

  PcdPublisher publisher(nh, pcd_file, 18, 10, 2);
  ros::spin();
  return 0;
}