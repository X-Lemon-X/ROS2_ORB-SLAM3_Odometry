// Frame math for turning ORB-SLAM3 camera poses into ROS poses. Eigen only.
//
// Frames used below:
//   W  ORB-SLAM3 world. In stereo-inertial mode, once the IMU is initialised, W is
//      gravity aligned with +Z up (ORB-SLAM3 rotates the map so g = (0, 0, -9.81)).
//      Its origin and yaw are arbitrary (they come from the first frames).
//      Before IMU initialisation W is the IMU body frame of the first frame and
//      is NOT gravity aligned.
//   C  left camera optical frame (x right, y down, z forward), as ORB-SLAM3 sees it.
//   B  robot base frame (REP-103: x forward, y left, z up).
//   M  published map frame. M = W, optionally shifted/rotated about Z so that the
//      robot starts at the origin looking along +X ("anchoring"). Rotation is about Z
//      only, so M stays gravity aligned.
//
// No optical->ROS axis swap is applied to W: in inertial mode W is already z-up.

#ifndef ORBSLAM3_ROS2_POSE_UTILS_HPP_
#define ORBSLAM3_ROS2_POSE_UTILS_HPP_

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace orbslam3_ros2
{

using Iso3 = Eigen::Isometry3d;

// T_W_B = T_W_C * T_C_B
inline Iso3 baseInWorld(const Iso3& T_W_C, const Iso3& T_B_C)
{
  return T_W_C * T_B_C.inverse();
}

// Yaw of the base x-axis projected on the gravity-aligned XY plane.
inline double yawOf(const Eigen::Matrix3d& R)
{
  const Eigen::Vector3d x = R.col(0);
  return std::atan2(x.y(), x.x());
}

// Anchor transform T_M_W chosen so that the given base pose maps to the origin
// with zero yaw. Roll and pitch are preserved (gravity alignment kept).
inline Iso3 makeAnchor(const Iso3& T_W_B0)
{
  Iso3 T_W_A = Iso3::Identity();
  T_W_A.linear() =
    Eigen::AngleAxisd(yawOf(T_W_B0.linear()), Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T_W_A.translation() = T_W_B0.translation();
  return T_W_A.inverse();
}

// Body-frame twist from two consecutive poses (finite difference).
inline void bodyTwist(const Iso3& T_prev,
                      const Iso3& T_curr,
                      double dt,
                      Eigen::Vector3d& v_body,
                      Eigen::Vector3d& w_body)
{
  const Iso3 dT = T_prev.inverse() * T_curr;  // motion expressed in previous body frame
  v_body = dT.translation() / dt;
  const Eigen::AngleAxisd aa(dT.linear());
  w_body = aa.axis() * aa.angle() / dt;
}

// Re-orthonormalise a rotation coming from float math.
inline Iso3 sanitize(const Iso3& T)
{
  Iso3 out = T;
  Eigen::Quaterniond q(T.linear());
  q.normalize();
  out.linear() = q.toRotationMatrix();
  return out;
}

}  // namespace orbslam3_ros2

#endif  // ORBSLAM3_ROS2_POSE_UTILS_HPP_
