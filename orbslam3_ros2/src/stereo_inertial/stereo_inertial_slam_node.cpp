#include "stereo_inertial_slam_node.hpp"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <unistd.h>

#include <cv_bridge/cv_bridge.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

// ORB-SLAM3
#include "ImuTypes.h"
#include "System.h"

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace orbslam3_ros2
{

namespace
{
// Same numbering as ORB_SLAM3::Tracking::eTrackingState
constexpr int kSystemNotReady = -1;
constexpr int kNoImagesYet = 0;
constexpr int kNotInitialized = 1;
constexpr int kOk = 2;
constexpr int kRecentlyLost = 3;
constexpr int kLost = 4;
constexpr int kOkKlt = 5;

std::string expandUser(const std::string& p)
{
  if (!p.empty() && p[0] == '~') {
    const char* home = std::getenv("HOME");
    if (home)
      return std::string(home) + p.substr(1);
  }
  return p;
}

Iso3 sophusToIso(const Sophus::SE3f& T)
{
  Iso3 out = Iso3::Identity();
  out.linear() = T.rotationMatrix().cast<double>();
  out.translation() = T.translation().cast<double>();
  return sanitize(out);
}

geometry_msgs::msg::Pose isoToPose(const Iso3& T)
{
  geometry_msgs::msg::Pose p;
  const Eigen::Quaterniond q(T.linear());
  p.position.x = T.translation().x();
  p.position.y = T.translation().y();
  p.position.z = T.translation().z();
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

geometry_msgs::msg::Transform isoToTf(const Iso3& T)
{
  geometry_msgs::msg::Transform t;
  const Eigen::Quaterniond q(T.linear());
  t.translation.x = T.translation().x();
  t.translation.y = T.translation().y();
  t.translation.z = T.translation().z();
  t.rotation.x = q.x();
  t.rotation.y = q.y();
  t.rotation.z = q.z();
  t.rotation.w = q.w();
  return t;
}

bool isFinitePose(const Sophus::SE3f& T)
{
  return T.matrix().allFinite();
}
}  // namespace

// ============================================================================ setup

StereoInertialSlamNode::StereoInertialSlamNode(const rclcpp::NodeOptions& options)
  : Node("orbslam3", options)
{
  vocabulary_file_ = declare_parameter<std::string>("vocabulary_file", "");
  settings_file_ = declare_parameter<std::string>("settings_file", "");

  left_topic_ = declare_parameter<std::string>("left_topic", "/oak/left_rect/image_raw");
  right_topic_ = declare_parameter<std::string>("right_topic", "/oak/right_rect/image_raw");
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/oak/imu/data");

  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  camera_frame_override_ = declare_parameter<std::string>("camera_frame", "");
  publish_tf_ = declare_parameter<bool>("publish_tf", true);
  anchor_to_start_ = declare_parameter<bool>("anchor_to_start", true);
  publish_before_imu_init_ = declare_parameter<bool>("publish_before_imu_init", false);
  publish_recently_lost_ = declare_parameter<bool>("publish_recently_lost", true);
  pos_stddev_ = declare_parameter<double>("position_stddev", 0.05);
  rot_stddev_ = declare_parameter<double>("orientation_stddev", 0.02);
  path_max_length_ = declare_parameter<int>("path_max_length", 2000);

  use_viewer_ = declare_parameter<bool>("use_viewer", false);
  equalize_ = declare_parameter<bool>("equalize", false);

  SyncConfig sc;
  sc.max_stereo_dt = declare_parameter<double>("max_stereo_dt", 0.003);
  sc.max_pending_frames =
    static_cast<std::size_t>(declare_parameter<int>("max_pending_frames", 2));
  imu_timeout_ = declare_parameter<double>("imu_timeout", 0.5);
  sync_ = ImuFrameSync<cv::Mat>(sc);

  map_directory_ = expandUser(declare_parameter<std::string>("map_directory", "~/.ros/orbslam3_maps"));
  load_map_name_ = declare_parameter<std::string>("load_map", "");
  save_map_name_ = declare_parameter<std::string>("save_map", "");
  save_on_exit_ = declare_parameter<bool>("save_on_exit", true);
  start_in_localization_mode_ = declare_parameter<bool>("localization_mode", false);
  paused_ = declare_parameter<bool>("start_paused", false);

  clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));

  // Outputs exist from the start so tools can see them while the vocabulary loads.
  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);
  pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 10);
  pub_path_ = create_publisher<nav_msgs::msg::Path>("~/path", 1);
  pub_state_ = create_publisher<std_msgs::msg::String>(
    "~/state", rclcpp::QoS(1).transient_local().reliable());

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  path_.header.frame_id = map_frame_;
  publishState("LOADING");
}

StereoInertialSlamNode::~StereoInertialSlamNode()
{
  stopWorker();
  if (slam_alive_) {
    if (save_on_exit_ || save_map_name_.empty()) {
      shutdownSlam(!save_map_name_.empty());
    } else {
      RCLCPP_INFO(get_logger(), "save_on_exit is false: exiting without saving the map");
    }
  }
}

std::string StereoInertialSlamNode::prepareSettingsFile()
{
  if (load_map_name_.empty() && save_map_name_.empty())
    return settings_file_;

  // ORB-SLAM3 reads atlas file names from the settings yaml. Write a copy with the
  // values from the ROS parameters so the yaml does not need editing.
  std::ifstream in(settings_file_);
  std::stringstream out;
  const std::regex atlas_key(R"(^\s*System\.(LoadAtlasFromFile|SaveAtlasToFile)\s*:)");
  std::string line;
  while (std::getline(in, line)) {
    if (!std::regex_search(line, atlas_key))
      out << line << "\n";
  }
  out << "\n# --- added by orbslam3 ROS node ---\n";
  if (!load_map_name_.empty())
    out << "System.LoadAtlasFromFile: \"" << load_map_name_ << "\"\n";
  if (!save_map_name_.empty())
    out << "System.SaveAtlasToFile: \"" << save_map_name_ << "\"\n";

  const fs::path tmp =
    fs::temp_directory_path() / ("orbslam3_ros2_" + std::to_string(::getpid()) + ".yaml");
  std::ofstream f(tmp);
  f << out.str();
  return tmp.string();
}

bool StereoInertialSlamNode::start()
{
  if (vocabulary_file_.empty() || settings_file_.empty()) {
    RCLCPP_FATAL(get_logger(), "Parameters 'vocabulary_file' and 'settings_file' are required");
    return false;
  }
  vocabulary_file_ = fs::absolute(expandUser(vocabulary_file_)).string();
  settings_file_ = fs::absolute(expandUser(settings_file_)).string();
  for (const auto& f : { vocabulary_file_, settings_file_ }) {
    if (!fs::exists(f)) {
      RCLCPP_FATAL(get_logger(), "File not found: %s", f.c_str());
      return false;
    }
  }

  // ORB-SLAM3 reads and writes the atlas as "./<name>.osa", i.e. relative to the
  // working directory, so the process works from the map directory.
  if (!load_map_name_.empty() || !save_map_name_.empty()) {
    std::error_code ec;
    fs::create_directories(map_directory_, ec);
    fs::current_path(map_directory_, ec);
    if (ec) {
      RCLCPP_FATAL(get_logger(), "Cannot use map_directory '%s': %s", map_directory_.c_str(),
                   ec.message().c_str());
      return false;
    }
    if (!load_map_name_.empty() && !fs::exists(load_map_name_ + ".osa")) {
      RCLCPP_WARN(get_logger(), "Map '%s/%s.osa' not found: starting with an empty map",
                  map_directory_.c_str(), load_map_name_.c_str());
      load_map_name_.clear();
    }
  }
  runtime_settings_file_ = prepareSettingsFile();

  RCLCPP_INFO(get_logger(), "Loading ORB-SLAM3 (vocabulary loading takes a while)...");
  RCLCPP_INFO(get_logger(), "  vocabulary: %s", vocabulary_file_.c_str());
  RCLCPP_INFO(get_logger(), "  settings:   %s", settings_file_.c_str());
  if (!load_map_name_.empty())
    RCLCPP_INFO(get_logger(), "  load map:   %s/%s.osa", map_directory_.c_str(),
                load_map_name_.c_str());
  if (!save_map_name_.empty())
    RCLCPP_INFO(get_logger(), "  save map:   %s/%s.osa", map_directory_.c_str(),
                save_map_name_.c_str());

  slam_ = std::make_unique<ORB_SLAM3::System>(
    vocabulary_file_, runtime_settings_file_, ORB_SLAM3::System::IMU_STEREO, use_viewer_);
  slam_alive_ = true;

  if (start_in_localization_mode_) {
    slam_->ActivateLocalizationMode();
    RCLCPP_INFO(get_logger(), "Starting in localization mode (map is not extended)");
  }

  running_ = true;
  worker_ = std::thread(&StereoInertialSlamNode::workerLoop, this);

  // Inputs. IMU depth is large because some drivers publish in bursts.
  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, rclcpp::SensorDataQoS().keep_last(1000),
    std::bind(&StereoInertialSlamNode::onImu, this, std::placeholders::_1));
  sub_left_ = create_subscription<sensor_msgs::msg::Image>(
    left_topic_, rclcpp::SensorDataQoS(),
    std::bind(&StereoInertialSlamNode::onLeft, this, std::placeholders::_1));
  sub_right_ = create_subscription<sensor_msgs::msg::Image>(
    right_topic_, rclcpp::SensorDataQoS(),
    std::bind(&StereoInertialSlamNode::onRight, this, std::placeholders::_1));

  using std::placeholders::_1;
  using std::placeholders::_2;
  srv_pause_ = create_service<Trigger>(
    "~/pause", std::bind(&StereoInertialSlamNode::srvPause, this, _1, _2));
  srv_resume_ = create_service<Trigger>(
    "~/resume", std::bind(&StereoInertialSlamNode::srvResume, this, _1, _2));
  srv_reset_ = create_service<Trigger>(
    "~/reset", std::bind(&StereoInertialSlamNode::srvReset, this, _1, _2));
  srv_reset_map_ = create_service<Trigger>(
    "~/reset_active_map", std::bind(&StereoInertialSlamNode::srvResetActiveMap, this, _1, _2));
  srv_loc_ = create_service<SetBool>(
    "~/set_localization_mode", std::bind(&StereoInertialSlamNode::srvLocalization, this, _1, _2));
  srv_save_ = create_service<Trigger>(
    "~/save_map", std::bind(&StereoInertialSlamNode::srvSaveMap, this, _1, _2));

  RCLCPP_INFO(get_logger(), "Ready. Waiting for %s, %s and %s", left_topic_.c_str(),
              right_topic_.c_str(), imu_topic_.c_str());
  publishState(paused_ ? "PAUSED" : "WAITING_FOR_DATA");
  return true;
}

// ============================================================================ input

cv::Mat StereoInertialSlamNode::toMono(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  try {
    return cv_bridge::toCvShare(msg, "mono8")->image.clone();
  } catch (const std::exception& e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Image conversion failed (%s): %s",
                          msg->encoding.c_str(), e.what());
    return {};
  }
}

void StereoInertialSlamNode::onLeft(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  if (paused_)
    return;
  cv::Mat img = toMono(msg);
  if (img.empty())
    return;
  {
    std::lock_guard<std::mutex> lk(sync_mutex_);
    if (camera_frame_.empty())
      camera_frame_ = camera_frame_override_.empty() ? msg->header.frame_id : camera_frame_override_;
    sync_.addLeft(rclcpp::Time(msg->header.stamp).seconds(), std::move(img));
  }
  sync_cv_.notify_one();
}

void StereoInertialSlamNode::onRight(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  if (paused_)
    return;
  cv::Mat img = toMono(msg);
  if (img.empty())
    return;
  {
    std::lock_guard<std::mutex> lk(sync_mutex_);
    sync_.addRight(rclcpp::Time(msg->header.stamp).seconds(), std::move(img));
  }
  sync_cv_.notify_one();
}

void StereoInertialSlamNode::onImu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
{
  if (paused_)
    return;
  ImuSample s;
  s.t = rclcpp::Time(msg->header.stamp).seconds();
  s.ax = static_cast<float>(msg->linear_acceleration.x);
  s.ay = static_cast<float>(msg->linear_acceleration.y);
  s.az = static_cast<float>(msg->linear_acceleration.z);
  s.gx = static_cast<float>(msg->angular_velocity.x);
  s.gy = static_cast<float>(msg->angular_velocity.y);
  s.gz = static_cast<float>(msg->angular_velocity.z);
  bool ok;
  {
    std::lock_guard<std::mutex> lk(sync_mutex_);
    if (imu_frame_.empty()) {
      imu_frame_ = msg->header.frame_id;
      RCLCPP_INFO(get_logger(), "IMU frame: '%s' (IMU.T_b_c1 must be expressed in this frame)",
                  imu_frame_.c_str());
    }
    ok = sync_.addImu(s);
  }
  if (!ok) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Dropping IMU sample with non-increasing timestamp. If this repeats, "
                         "the IMU driver publishes out of order or duplicate stamps.");
  }
  sync_cv_.notify_one();
}

// ============================================================================ worker

void StereoInertialSlamNode::workerLoop()
{
  double waiting_stamp = -1.0;
  auto waiting_since = std::chrono::steady_clock::now();

  while (running_) {
    std::optional<StereoFrame<cv::Mat>> frame;
    {
      std::unique_lock<std::mutex> lk(sync_mutex_);
      frame = sync_.popReady();
      if (!frame) {
        sync_cv_.wait_for(lk, 20ms);
        if (!running_)
          break;
        frame = sync_.popReady();
      }

      if (!frame) {
        // Frames waiting for IMU that never comes -> warn and drop them.
        const auto oldest = sync_.oldestPendingStamp();
        if (oldest) {
          if (*oldest != waiting_stamp) {
            waiting_stamp = *oldest;
            waiting_since = std::chrono::steady_clock::now();
          } else if (std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                                   waiting_since)
                       .count() > imu_timeout_) {
            sync_.dropOldestPending();
            const std::string why =
              sync_.hasImu()
                ? "IMU is " + std::to_string(*oldest - sync_.lastImuStamp()) + " s behind"
                : std::string("no IMU received yet");
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 3000,
              "No IMU data covering image time (%s). Check that %s is publishing and that "
              "IMU and camera use the same clock.",
              why.c_str(), imu_topic_.c_str());
          }
        }
      }
      if (sync_.takeImuOverflowFlag()) {
        RCLCPP_WARN(get_logger(), "IMU buffer overflow: images stopped arriving while IMU kept "
                                  "streaming. Old IMU samples were discarded.");
      }
    }
    if (!frame)
      continue;

    if (frame->dropped_before > 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "SLAM is slower than the camera: skipped %zu frame(s). Lower the "
                           "camera FPS or ORBextractor.nFeatures if this is constant.",
                           frame->dropped_before);
    }
    if (frame->imu.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Frame without IMU samples: IMU rate is too low or IMU stamps lag.");
    }

    std::vector<ORB_SLAM3::IMU::Point> imu;
    imu.reserve(frame->imu.size());
    for (const auto& s : frame->imu) {
      imu.emplace_back(cv::Point3f(s.ax, s.ay, s.az), cv::Point3f(s.gx, s.gy, s.gz), s.t);
    }

    if (equalize_) {
      clahe_->apply(frame->left, frame->left);
      clahe_->apply(frame->right, frame->right);
    }

    Sophus::SE3f Tcw;
    int state;
    bool imu_init;
    bool map_changed;
    {
      std::lock_guard<std::mutex> lk(slam_mutex_);
      if (!slam_alive_)
        break;
      Tcw = slam_->TrackStereo(frame->left, frame->right, frame->t, imu);
      state = slam_->GetTrackingState();
      imu_init = slam_->GetTimeFromIMUInit() > 0.0;
      map_changed = slam_->MapChanged();
    }

    // --- IMU initialisation / new-map bookkeeping
    if (!imu_was_initialized_ && imu_init) {
      RCLCPP_INFO(get_logger(), "IMU initialised: map is now metric and gravity aligned");
      anchored_ = false;  // ORB-SLAM3 rotated the whole map
    } else if (imu_was_initialized_ && !imu_init) {
      RCLCPP_WARN(get_logger(),
                  "ORB-SLAM3 started a new map (reset or tracking lost). Pose output resumes "
                  "after the IMU re-initialises; the frame is re-anchored.");
      anchored_ = false;
    }
    imu_was_initialized_ = imu_init;
    if (request_reanchor_.exchange(false))
      anchored_ = false;
    if (map_changed) {
      RCLCPP_INFO(get_logger(), "Loop closure / map merge: map corrected, pose may jump");
    }

    // --- state topic
    std::string st = stateName(state);
    if ((state == kOk || state == kOkKlt) && !imu_init)
      st = "OK_WAITING_IMU_INIT";
    if (paused_)
      st = "PAUSED";
    bool changed;
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      changed = (st != last_state_str_);
    }
    if (changed) {
      publishState(st);
      if (state == kLost)
        RCLCPP_WARN(get_logger(), "Tracking LOST");
    }

    if (isFinitePose(Tcw))
      handleResult(*frame, sophusToIso(Tcw), state);
  }
}

void StereoInertialSlamNode::handleResult(const StereoFrame<cv::Mat>& frame,
                                          const Iso3& T_C_W,
                                          int state)
{
  const bool tracking = (state == kOk || state == kOkKlt);
  const bool recently_lost = (state == kRecentlyLost);
  if (!tracking && !(recently_lost && publish_recently_lost_))
    return;
  if (!imu_was_initialized_ && !publish_before_imu_init_)
    return;
  if (!T_C_W.matrix().allFinite())
    return;

  // Camera pose in base (static, from TF; looked up once).
  if (!T_B_C_) {
    std::string cam;
    {
      std::lock_guard<std::mutex> lk(sync_mutex_);
      cam = camera_frame_;
    }
    if (!lookupBaseToCamera(cam))
      return;
  }

  const Iso3 T_W_B = baseInWorld(T_C_W.inverse(), *T_B_C_);

  if (!anchored_) {
    T_M_W_ = anchor_to_start_ ? makeAnchor(T_W_B) : Iso3::Identity();
    anchored_ = true;
    last_T_M_B_.reset();
    path_.poses.clear();
    if (anchor_to_start_)
      RCLCPP_INFO(get_logger(), "Map frame '%s' anchored at current robot pose",
                  map_frame_.c_str());
  }

  const Iso3 T_M_B = sanitize(T_M_W_ * T_W_B);
  const rclcpp::Time stamp(static_cast<int64_t>(std::llround(frame.t * 1e9)), RCL_ROS_TIME);
  publishPose(stamp, T_M_B, recently_lost || !imu_was_initialized_);
}

bool StereoInertialSlamNode::lookupBaseToCamera(const std::string& camera_frame)
{
  if (camera_frame.empty() || camera_frame == base_frame_) {
    T_B_C_ = Iso3::Identity();
    if (camera_frame.empty())
      RCLCPP_WARN(get_logger(), "Images have no frame_id: assuming base_frame == camera frame");
    return true;
  }
  try {
    const auto tf = tf_buffer_->lookupTransform(base_frame_, camera_frame, tf2::TimePointZero);
    T_B_C_ = tf2::transformToEigen(tf);
    const auto& t = T_B_C_->translation();
    RCLCPP_INFO(get_logger(), "Using TF %s -> %s: t = [%.3f %.3f %.3f]", base_frame_.c_str(),
                camera_frame.c_str(), t.x(), t.y(), t.z());
    return true;
  } catch (const tf2::TransformException& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                         "Waiting for TF %s -> %s (publish a static transform from your base "
                         "to the camera): %s",
                         base_frame_.c_str(), camera_frame.c_str(), e.what());
    return false;
  }
}

void StereoInertialSlamNode::publishPose(const rclcpp::Time& stamp,
                                         const Iso3& T_M_B,
                                         bool degraded)
{
  const double t = stamp.seconds();
  const double k = degraded ? 100.0 : 1.0;  // inflate covariance when not fully trusted
  const double pv = pos_stddev_ * pos_stddev_ * k;
  const double rv = rot_stddev_ * rot_stddev_ * k;

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = map_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose = isoToPose(T_M_B);
  for (int i = 0; i < 3; ++i) {
    odom.pose.covariance[i * 7] = pv;
    odom.pose.covariance[(i + 3) * 7] = rv;
  }

  if (last_T_M_B_ && t > last_pose_t_ && t - last_pose_t_ < 1.0) {
    Eigen::Vector3d v, w;
    bodyTwist(*last_T_M_B_, T_M_B, t - last_pose_t_, v, w);
    odom.twist.twist.linear.x = v.x();
    odom.twist.twist.linear.y = v.y();
    odom.twist.twist.linear.z = v.z();
    odom.twist.twist.angular.x = w.x();
    odom.twist.twist.angular.y = w.y();
    odom.twist.twist.angular.z = w.z();
    for (int i = 0; i < 3; ++i) {
      odom.twist.covariance[i * 7] = 0.05 * k;
      odom.twist.covariance[(i + 3) * 7] = 0.05 * k;
    }
  } else {
    odom.twist.covariance[0] = -1.0;  // unknown
  }
  last_T_M_B_ = T_M_B;
  last_pose_t_ = t;
  pub_odom_->publish(odom);

  geometry_msgs::msg::PoseStamped ps;
  ps.header = odom.header;
  ps.pose = odom.pose.pose;
  pub_pose_->publish(ps);

  path_.header.stamp = stamp;
  path_.poses.push_back(ps);
  if (path_max_length_ > 0 && static_cast<int>(path_.poses.size()) > path_max_length_)
    path_.poses.erase(path_.poses.begin());
  if (pub_path_->get_subscription_count() > 0)
    pub_path_->publish(path_);

  if (!publish_tf_)
    return;

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = map_frame_;
  if (odom_frame_.empty()) {
    tf.child_frame_id = base_frame_;
    tf.transform = isoToTf(T_M_B);
  } else {
    // REP-105: we own map -> odom, something else (wheel odometry, EKF) owns odom -> base.
    try {
      const auto odom_base = tf_buffer_->lookupTransform(odom_frame_, base_frame_, stamp, 50ms);
      const Iso3 T_O_B = tf2::transformToEigen(odom_base);
      tf.child_frame_id = odom_frame_;
      tf.transform = isoToTf(sanitize(T_M_B * T_O_B.inverse()));
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "odom_frame is set but TF %s -> %s is unavailable: %s",
                           odom_frame_.c_str(), base_frame_.c_str(), e.what());
      return;
    }
  }
  tf_broadcaster_->sendTransform(tf);
}

void StereoInertialSlamNode::publishState(const std::string& state)
{
  std::lock_guard<std::mutex> lk(state_mutex_);
  last_state_str_ = state;
  std_msgs::msg::String m;
  m.data = state;
  pub_state_->publish(m);
}

const char* StereoInertialSlamNode::stateName(int state)
{
  switch (state) {
    case kSystemNotReady:
      return "SYSTEM_NOT_READY";
    case kNoImagesYet:
      return "NO_IMAGES_YET";
    case kNotInitialized:
      return "NOT_INITIALIZED";
    case kOk:
      return "OK";
    case kRecentlyLost:
      return "RECENTLY_LOST";
    case kLost:
      return "LOST";
    case kOkKlt:
      return "OK_KLT";
    default:
      return "UNKNOWN";
  }
}

void StereoInertialSlamNode::stopWorker()
{
  running_ = false;
  sync_cv_.notify_all();
  if (worker_.joinable())
    worker_.join();
}

bool StereoInertialSlamNode::shutdownSlam(bool save)
{
  stopWorker();
  std::lock_guard<std::mutex> lk(slam_mutex_);
  if (!slam_alive_)
    return false;
  if (save) {
    // Give local mapping / loop closing a moment to finish the last keyframes:
    // this ORB-SLAM3 build does not wait for its threads before saving.
    RCLCPP_INFO(get_logger(), "Saving map to %s/%s.osa ...", map_directory_.c_str(),
                save_map_name_.c_str());
    std::this_thread::sleep_for(2s);
    std::error_code ec;
    fs::current_path(map_directory_, ec);
  }
  slam_->Shutdown();  // saves the atlas when System.SaveAtlasToFile is set
  slam_alive_ = false;
  publishState(save ? "STOPPED_MAP_SAVED" : "STOPPED");
  return true;
}

// ============================================================================ services

void StereoInertialSlamNode::srvPause(const Trigger::Request::SharedPtr,
                                      Trigger::Response::SharedPtr res)
{
  paused_ = true;
  {
    std::lock_guard<std::mutex> lk(sync_mutex_);
    sync_.clear();
  }
  publishState("PAUSED");
  res->success = slam_alive_;
  res->message = "Paused: images and IMU are ignored. If the camera moves more than ~1 s "
                 "while paused, ORB-SLAM3 starts a new map on resume and merges it when it "
                 "recognises a known place.";
}

void StereoInertialSlamNode::srvResume(const Trigger::Request::SharedPtr,
                                       Trigger::Response::SharedPtr res)
{
  paused_ = false;
  publishState("WAITING_FOR_DATA");
  res->success = slam_alive_;
  res->message = slam_alive_ ? "Resumed" : "SLAM is stopped (map was saved); restart the node";
}

void StereoInertialSlamNode::srvReset(const Trigger::Request::SharedPtr,
                                      Trigger::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lk(slam_mutex_);
  if (!slam_alive_) {
    res->success = false;
    res->message = "SLAM is stopped";
    return;
  }
  slam_->Reset();
  request_reanchor_ = true;
  res->success = true;
  res->message = "Whole atlas will be cleared on the next frame";
}

void StereoInertialSlamNode::srvResetActiveMap(const Trigger::Request::SharedPtr,
                                               Trigger::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lk(slam_mutex_);
  if (!slam_alive_) {
    res->success = false;
    res->message = "SLAM is stopped";
    return;
  }
  slam_->ResetActiveMap();
  request_reanchor_ = true;
  res->success = true;
  res->message = "Active map will be cleared on the next frame (other maps are kept)";
}

void StereoInertialSlamNode::srvLocalization(const SetBool::Request::SharedPtr req,
                                             SetBool::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lk(slam_mutex_);
  if (!slam_alive_) {
    res->success = false;
    res->message = "SLAM is stopped";
    return;
  }
  if (req->data)
    slam_->ActivateLocalizationMode();
  else
    slam_->DeactivateLocalizationMode();
  res->success = true;
  res->message = req->data ? "Localization only (mapping stopped) from the next frame"
                           : "SLAM mode (mapping) from the next frame";
}

void StereoInertialSlamNode::srvSaveMap(const Trigger::Request::SharedPtr,
                                        Trigger::Response::SharedPtr res)
{
  if (save_map_name_.empty()) {
    res->success = false;
    res->message = "Parameter 'save_map' is empty. ORB-SLAM3 needs the file name at startup: "
                   "relaunch with save_map:=<name>.";
    return;
  }
  if (!slam_alive_) {
    res->success = false;
    res->message = "SLAM is already stopped";
    return;
  }
  // ORB-SLAM3 can only save its atlas as part of Shutdown(), so saving ends the session.
  const bool ok = shutdownSlam(true);
  const fs::path file = fs::path(map_directory_) / (save_map_name_ + ".osa");
  res->success = ok && fs::exists(file);
  res->message = res->success
                   ? "Saved " + file.string() + ". SLAM stopped: relaunch with load_map:=" +
                       save_map_name_ + " to continue from this map."
                   : "Saving failed, see node log";
}

}  // namespace orbslam3_ros2
