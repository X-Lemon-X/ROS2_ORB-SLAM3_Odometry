#ifndef ORBSLAM3_ROS2_STEREO_INERTIAL_SLAM_NODE_HPP_
#define ORBSLAM3_ROS2_STEREO_INERTIAL_SLAM_NODE_HPP_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "imu_frame_sync.hpp"
#include "pose_utils.hpp"

namespace ORB_SLAM3
{
class System;
}

namespace orbslam3_ros2
{

class StereoInertialSlamNode : public rclcpp::Node
{
 public:
  explicit StereoInertialSlamNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~StereoInertialSlamNode() override;

  // Creates ORB-SLAM3 (loads vocabulary, optionally an atlas) and starts processing.
  // Separate from the constructor so a failure can be reported cleanly from main().
  bool start();

 private:
  using Trigger = std_srvs::srv::Trigger;
  using SetBool = std_srvs::srv::SetBool;

  // ----- ROS input
  void onLeft(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void onRight(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
  cv::Mat toMono(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

  // ----- SLAM worker
  void workerLoop();
  void handleResult(const StereoFrame<cv::Mat>& frame, const Iso3& T_C_W, int state);
  void publishPose(const rclcpp::Time& stamp, const Iso3& T_M_B, bool degraded);
  void publishState(const std::string& state);
  bool lookupBaseToCamera(const std::string& camera_frame);
  void stopWorker();

  // ----- services
  void srvPause(const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res);
  void srvResume(const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res);
  void srvReset(const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res);
  void srvResetActiveMap(const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res);
  void srvLocalization(const SetBool::Request::SharedPtr req, SetBool::Response::SharedPtr res);
  void srvSaveMap(const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res);

  // ----- helpers
  std::string prepareSettingsFile();
  bool shutdownSlam(bool save);
  static const char* stateName(int state);

  // ----- parameters
  std::string vocabulary_file_, settings_file_;
  std::string left_topic_, right_topic_, imu_topic_;
  std::string map_frame_, odom_frame_, base_frame_, camera_frame_override_;
  bool use_viewer_{ false };
  bool equalize_{ false };
  bool publish_tf_{ true };
  bool anchor_to_start_{ true };
  bool publish_before_imu_init_{ false };
  bool publish_recently_lost_{ true };
  double imu_timeout_{ 0.5 };
  double pos_stddev_{ 0.05 }, rot_stddev_{ 0.02 };
  int path_max_length_{ 2000 };
  std::string map_directory_, load_map_name_, save_map_name_;
  bool save_on_exit_{ true };
  bool start_in_localization_mode_{ false };

  // ----- ORB-SLAM3
  std::unique_ptr<ORB_SLAM3::System> slam_;
  std::mutex slam_mutex_;  // guards slam_ lifetime vs. services
  std::string runtime_settings_file_;

  // ----- buffering
  ImuFrameSync<cv::Mat> sync_;
  std::mutex sync_mutex_;
  std::condition_variable sync_cv_;
  std::thread worker_;
  std::atomic<bool> running_{ false };
  std::atomic<bool> paused_{ false };
  std::atomic<bool> slam_alive_{ false };
  cv::Ptr<cv::CLAHE> clahe_;

  // ----- pose bookkeeping (worker thread only)
  std::optional<Iso3> T_B_C_;  // camera pose in base frame (from TF)
  std::string camera_frame_;
  std::string imu_frame_;
  bool imu_was_initialized_{ false };
  bool anchored_{ false };
  Iso3 T_M_W_{ Iso3::Identity() };
  std::optional<Iso3> last_T_M_B_;
  double last_pose_t_{ 0.0 };
  std::string last_state_str_;
  std::mutex state_mutex_;
  std::atomic<bool> request_reanchor_{ false };
  nav_msgs::msg::Path path_;

  // ----- ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_left_, sub_right_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
  rclcpp::Service<Trigger>::SharedPtr srv_pause_, srv_resume_, srv_reset_, srv_reset_map_,
    srv_save_;
  rclcpp::Service<SetBool>::SharedPtr srv_loc_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace orbslam3_ros2

#endif  // ORBSLAM3_ROS2_STEREO_INERTIAL_SLAM_NODE_HPP_
