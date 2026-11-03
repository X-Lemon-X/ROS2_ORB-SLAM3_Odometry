#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "stereo_inertial_slam_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<orbslam3_ros2::StereoInertialSlamNode>();
  if (!node->start()) {
    node.reset();
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  node.reset();  // stops SLAM (and saves the map if configured) before ROS goes down
  rclcpp::shutdown();
  return 0;
}
