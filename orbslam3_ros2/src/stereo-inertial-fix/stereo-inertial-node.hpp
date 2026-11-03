#ifndef STEREO_INERTIAL_NODE_HPP_
#define STEREO_INERTIAL_NODE_HPP_

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "ImuTypes.h"
#include "System.h"

class StereoInertialNode : public rclcpp::Node
{
 public:
  // strDoRectify / strDoEqual are passed as "true"/"false" strings, matching argv in
  // main.cpp
  StereoInertialNode(ORB_SLAM3::System* pSLAM,
                     const std::string& strSettingsFile,
                     const std::string& strDoRectify,
                     const std::string& strDoEqual);
  ~StereoInertialNode();

 private:
  // --- ROS callbacks ---
  void GrabImageLeft(const sensor_msgs::msg::Image::SharedPtr msg);
  void GrabImageRight(const sensor_msgs::msg::Image::SharedPtr msg);
  void GrabImu(const sensor_msgs::msg::Imu::SharedPtr msg);

  // --- helpers ---
  cv::Mat GetImage(const sensor_msgs::msg::Image::SharedPtr msg);
  void LoadRectificationMaps(const std::string& strSettingsFile);
  void SyncWithImu(); // runs in its own thread
  void PublishPose(const Sophus::SE3f& Tcw, const rclcpp::Time& stamp);

  // --- SLAM system (owned by caller, not this node) ---
  ORB_SLAM3::System* SLAM_;

  // --- worker thread that matches stereo pairs with IMU data ---
  std::thread syncThread_;
  std::atomic<bool> running_{ true };

  // --- image buffers (filled by subscription callbacks) ---
  std::queue<sensor_msgs::msg::Image::SharedPtr> imgLeftBuf_;
  std::queue<sensor_msgs::msg::Image::SharedPtr> imgRightBuf_;
  std::mutex mBufMutexLeft_;
  std::mutex mBufMutexRight_;

  // --- imu buffer ---
  std::queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf_;
  std::mutex mBufMutexImu_;

  // --- condition variable to wake the sync thread instead of busy-polling ---
  std::condition_variable dataCond_;
  std::mutex dataCondMutex_;

  // --- rectification / preprocessing config ---
  bool doRectify_{ false };
  bool doEqual_{ false };
  cv::Mat M1l_, M2l_, M1r_, M2r_;

  // --- ROS interfaces ---
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImgLeft_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImgRight_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // --- pose accumulation state (see PublishPose) ---
  bool has_prev_pose_{ false };
  Sophus::SE3f prev_pose_;
  Sophus::SE3f accumulated_pose_;

  // max allowed timestamp gap (seconds) between left/right frame before we drop the older
  // one
  static constexpr double kMaxImageTimeDiff = 0.01;
};

#endif // STEREO_INERTIAL_NODE_HPP_