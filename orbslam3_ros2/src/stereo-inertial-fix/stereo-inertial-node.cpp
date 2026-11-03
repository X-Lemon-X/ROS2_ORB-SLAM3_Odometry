#include "stereo-inertial-node.hpp"

#include <chrono>

using std::placeholders::_1;

StereoInertialNode::StereoInertialNode(ORB_SLAM3::System* pSLAM,
                                       const std::string& strSettingsFile,
                                       const std::string& strDoRectify,
                                       const std::string& strDoEqual)
  : Node("orbslam3_stereo_inertial")
  , SLAM_(pSLAM)
{
  doRectify_ = (strDoRectify == "true");
  doEqual_ = (strDoEqual == "true");

  if (doRectify_) {
    LoadRectificationMaps(strSettingsFile);
  }

  // --- subscriptions ---
  // Topic names: adjust to match your camera/IMU driver, or make these node parameters.
  subImgLeft_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/left",
    rclcpp::SensorDataQoS(),
    std::bind(&StereoInertialNode::GrabImageLeft, this, _1));

  subImgRight_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/right",
    rclcpp::SensorDataQoS(),
    std::bind(&StereoInertialNode::GrabImageRight, this, _1));

  subImu_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu", rclcpp::SensorDataQoS(), std::bind(&StereoInertialNode::GrabImu, this, _1));

  // --- publishers ---
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  accumulated_pose_ = Sophus::SE3f(); // identity

  // --- start the worker thread that syncs stereo+imu and calls TrackStereo ---
  syncThread_ = std::thread(&StereoInertialNode::SyncWithImu, this);

  RCLCPP_INFO(this->get_logger(),
              "StereoInertialNode initialized (rectify=%d, equalize=%d)",
              doRectify_,
              doEqual_);
}

StereoInertialNode::~StereoInertialNode()
{
  running_ = false;
  dataCond_.notify_all();
  if (syncThread_.joinable())
    syncThread_.join();

  SLAM_->Shutdown();
}

void
StereoInertialNode::LoadRectificationMaps(const std::string& strSettingsFile)
{
  cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to open settings file: %s", strSettingsFile.c_str());
    rclcpp::shutdown();
    return;
  }

  cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
  fsSettings["LEFT.K"] >> K_l;
  fsSettings["RIGHT.K"] >> K_r;
  fsSettings["LEFT.P"] >> P_l;
  fsSettings["RIGHT.P"] >> P_r;
  fsSettings["LEFT.R"] >> R_l;
  fsSettings["RIGHT.R"] >> R_r;
  fsSettings["LEFT.D"] >> D_l;
  fsSettings["RIGHT.D"] >> D_r;

  int rows_l = fsSettings["LEFT.height"];
  int cols_l = fsSettings["LEFT.width"];
  int rows_r = fsSettings["RIGHT.height"];
  int cols_r = fsSettings["RIGHT.width"];

  if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() ||
      R_r.empty() || D_l.empty() || D_r.empty() || rows_l == 0 || cols_l == 0 ||
      rows_r == 0 || cols_r == 0) {
    RCLCPP_ERROR(this->get_logger(), "Calibration parameters missing in settings file");
    rclcpp::shutdown();
    return;
  }

  cv::initUndistortRectifyMap(K_l,
                              D_l,
                              R_l,
                              P_l.rowRange(0, 3).colRange(0, 3),
                              cv::Size(cols_l, rows_l),
                              CV_32F,
                              M1l_,
                              M2l_);
  cv::initUndistortRectifyMap(K_r,
                              D_r,
                              R_r,
                              P_r.rowRange(0, 3).colRange(0, 3),
                              cv::Size(cols_r, rows_r),
                              CV_32F,
                              M1r_,
                              M2r_);
}

void
StereoInertialNode::GrabImageLeft(const sensor_msgs::msg::Image::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(mBufMutexLeft_);
    imgLeftBuf_.push(msg);
  }
  dataCond_.notify_one();
}

void
StereoInertialNode::GrabImageRight(const sensor_msgs::msg::Image::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(mBufMutexRight_);
    imgRightBuf_.push(msg);
  }
  dataCond_.notify_one();
}

void
StereoInertialNode::GrabImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(mBufMutexImu_);
    imuBuf_.push(msg);
  }
  dataCond_.notify_one();
}

cv::Mat
StereoInertialNode::GetImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return cv::Mat();
  }

  if (cv_ptr->image.type() == 0) // CV_8UC1
  {
    return cv_ptr->image.clone();
  } else {
    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    return gray;
  }
}

void
StereoInertialNode::SyncWithImu()
{
  const double maxTimeDiff = kMaxImageTimeDiff;

  while (running_ && rclcpp::ok()) {
    cv::Mat imLeft, imRight;
    double tImLeft = 0, tImRight = 0;
    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;

    // --- wait until we have data to look at ---
    {
      std::unique_lock<std::mutex> lk(dataCondMutex_);
      dataCond_.wait_for(lk, std::chrono::milliseconds(50), [this] {
        return !running_ || (!imgLeftBuf_.empty() && !imgRightBuf_.empty());
      });
    }

    if (!running_)
      break;

    if (imgLeftBuf_.empty() || imgRightBuf_.empty())
      continue;

    // --- pop a timestamp-matched stereo pair ---
    {
      std::lock_guard<std::mutex> lockL(mBufMutexLeft_);
      std::lock_guard<std::mutex> lockR(mBufMutexRight_);

      tImLeft = rclcpp::Time(imgLeftBuf_.front()->header.stamp).seconds();
      tImRight = rclcpp::Time(imgRightBuf_.front()->header.stamp).seconds();

      // drop stale frames on whichever side is lagging
      while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf_.size() > 1) {
        imgRightBuf_.pop();
        tImRight = rclcpp::Time(imgRightBuf_.front()->header.stamp).seconds();
      }
      while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf_.size() > 1) {
        imgLeftBuf_.pop();
        tImLeft = rclcpp::Time(imgLeftBuf_.front()->header.stamp).seconds();
      }

      if (std::abs(tImLeft - tImRight) > maxTimeDiff)
        continue; // still not matched, try again next loop

      imLeft = GetImage(imgLeftBuf_.front());
      imRight = GetImage(imgRightBuf_.front());
      imgLeftBuf_.pop();
      imgRightBuf_.pop();
    }

    if (imLeft.empty() || imRight.empty())
      continue;

    // --- gather IMU measurements up to this frame's timestamp ---
    {
      std::lock_guard<std::mutex> lock(mBufMutexImu_);
      while (!imuBuf_.empty() &&
             rclcpp::Time(imuBuf_.front()->header.stamp).seconds() <= tImLeft) {
        auto& m = imuBuf_.front();
        cv::Point3f acc(
          m->linear_acceleration.x, m->linear_acceleration.y, m->linear_acceleration.z);
        cv::Point3f gyr(
          m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z);
        double t = rclcpp::Time(m->header.stamp).seconds();
        vImuMeas.emplace_back(acc, gyr, t);
        imuBuf_.pop();
      }
    }

    if (vImuMeas.empty())
      continue; // ORB_SLAM3 IMU_STEREO needs at least some IMU data between frames

    // --- optional preprocessing ---
    if (doRectify_) {
      cv::remap(imLeft, imLeft, M1l_, M2l_, cv::INTER_LINEAR);
      cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR);
    }
    if (doEqual_) {
      static cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
      clahe->apply(imLeft, imLeft);
      clahe->apply(imRight, imRight);
    }

    // --- run SLAM ---
    Sophus::SE3f Tcw = SLAM_->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);

    if (Tcw.matrix().isZero(0)) {
      RCLCPP_WARN(this->get_logger(), "Invalid pose from ORB-SLAM3");
      continue;
    }

    PublishPose(Tcw, rclcpp::Time(static_cast<int64_t>(tImLeft * 1e9)));
  }
}

void
StereoInertialNode::PublishPose(const Sophus::SE3f& Tcw, const rclcpp::Time& /*stamp*/)
{
  // 1. Invert to get camera-in-world
  Sophus::SE3f Twc = Tcw.inverse();

  // --- Compute delta and accumulate ---
  if (!has_prev_pose_) {
    prev_pose_ = Twc;
    has_prev_pose_ = true;
  }

  Sophus::SE3f delta = prev_pose_.inverse() * Twc;
  accumulated_pose_ = accumulated_pose_ * delta;
  prev_pose_ = Twc;

  // 2. Extract translation & rotation
  Eigen::Vector3f t_slam = accumulated_pose_.translation();
  Eigen::Matrix3f R_slam = accumulated_pose_.rotationMatrix();

  // 3. Axis conversion SLAM -> ROS
  // ORB-SLAM3: X=right, Y=down, Z=forward
  // ROS REP-103: X=forward, Y=left, Z=up
  Eigen::Matrix3f R_slam_to_ros;
  R_slam_to_ros << 0, 0, 1, -1, 0, 0, 0, -1, 0;

  Eigen::Vector3f t_ros = R_slam_to_ros * t_slam;
  Eigen::Matrix3f R_ros = R_slam_to_ros * R_slam * R_slam_to_ros.transpose();

  // 4. Convert to quaternion
  Eigen::Quaternionf q_ros(R_ros);
  q_ros.normalize();

  // 5. Publish Odometry
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = this->get_clock()->now();
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";

  odom_msg.pose.pose.position.x = t_ros.x();
  odom_msg.pose.pose.position.y = t_ros.y();
  odom_msg.pose.pose.position.z = t_ros.z();

  odom_msg.pose.pose.orientation.x = q_ros.x();
  odom_msg.pose.pose.orientation.y = q_ros.y();
  odom_msg.pose.pose.orientation.z = q_ros.z();
  odom_msg.pose.pose.orientation.w = q_ros.w();

  odom_pub_->publish(odom_msg);

  // 6. Publish TF
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header = odom_msg.header;
  tf_msg.child_frame_id = odom_msg.child_frame_id;

  tf_msg.transform.translation.x = t_ros.x();
  tf_msg.transform.translation.y = t_ros.y();
  tf_msg.transform.translation.z = t_ros.z();

  tf_msg.transform.rotation.x = q_ros.x();
  tf_msg.transform.rotation.y = q_ros.y();
  tf_msg.transform.rotation.z = q_ros.z();
  tf_msg.transform.rotation.w = q_ros.w();

  tf_broadcaster_->sendTransform(tf_msg);
}