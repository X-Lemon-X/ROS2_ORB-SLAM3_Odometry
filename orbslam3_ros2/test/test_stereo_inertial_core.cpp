#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include "imu_frame_sync.hpp"
#include "pose_utils.hpp"

using orbslam3_ros2::ImuFrameSync;
using orbslam3_ros2::ImuSample;
using orbslam3_ros2::Iso3;
using orbslam3_ros2::SyncConfig;

namespace
{
ImuSample imu(double t)
{
  ImuSample s;
  s.t = t;
  s.az = 9.81f;
  return s;
}

// Feed IMU at 200 Hz in [t0, t1]
void feedImu(ImuFrameSync<std::string>& s, double t0, double t1)
{
  for (int i = static_cast<int>(std::lround(t0 * 200)); i <= std::lround(t1 * 200); ++i)
    s.addImu(imu(i / 200.0));
}
}  // namespace

// ------------------------------------------------------------------ sync

TEST(ImuFrameSync, WaitsUntilImuCoversFrame)
{
  ImuFrameSync<std::string> s;
  feedImu(s, 0.0, 0.095);
  s.addLeft(0.100, "L1");
  s.addRight(0.100, "R1");
  EXPECT_FALSE(s.popReady().has_value()) << "IMU only up to 0.095, frame at 0.100";

  s.addImu(imu(0.100));
  auto f = s.popReady();
  ASSERT_TRUE(f.has_value());
  EXPECT_EQ(f->left, "L1");
  EXPECT_EQ(f->right, "R1");
  EXPECT_DOUBLE_EQ(f->imu.back().t, 0.100);
}

TEST(ImuFrameSync, ImuSplitBetweenConsecutiveFrames)
{
  ImuFrameSync<std::string> s;
  feedImu(s, 0.0, 0.2);
  s.addLeft(0.100, "L1");
  s.addRight(0.100, "R1");
  auto f1 = s.popReady();
  ASSERT_TRUE(f1);
  EXPECT_LE(f1->imu.back().t, 0.100);

  s.addLeft(0.150, "L2");
  s.addRight(0.150, "R2");
  auto f2 = s.popReady();
  ASSERT_TRUE(f2);
  ASSERT_FALSE(f2->imu.empty());
  EXPECT_GT(f2->imu.front().t, 0.100);  // nothing reused from frame 1
  EXPECT_LE(f2->imu.back().t, 0.150);
  EXPECT_EQ(f2->imu.size(), 10u);  // 0.105 ... 0.150 at 200 Hz
}

TEST(ImuFrameSync, FirstFrameKeepsOnlyRecentImu)
{
  ImuFrameSync<std::string> s;
  feedImu(s, 0.0, 3.0);
  s.addLeft(3.0, "L");
  s.addRight(3.0, "R");
  auto f = s.popReady();
  ASSERT_TRUE(f);
  EXPECT_GE(f->imu.front().t, 2.5 - 1e-9);
}

TEST(ImuFrameSync, StereoPairingDropsUnmatched)
{
  SyncConfig cfg;
  cfg.max_stereo_dt = 0.003;
  ImuFrameSync<std::string> s(cfg);
  feedImu(s, 0.0, 1.0);
  s.addLeft(0.100, "L1");  // right for 0.100 lost
  s.addLeft(0.133, "L2");
  s.addRight(0.1335, "R2");
  auto f = s.popReady();
  ASSERT_TRUE(f);
  EXPECT_EQ(f->left, "L2");
  EXPECT_EQ(f->right, "R2");
  EXPECT_EQ(s.unmatchedImages(), 1u);
}

TEST(ImuFrameSync, SlowSlamSkipsFramesButKeepsImu)
{
  SyncConfig cfg;
  cfg.max_pending_frames = 5;
  ImuFrameSync<std::string> s(cfg);
  feedImu(s, 0.0, 0.05);
  s.addLeft(0.05, "L0");
  s.addRight(0.05, "R0");
  ASSERT_TRUE(s.popReady());

  // three frames arrive while SLAM is busy
  feedImu(s, 0.055, 0.2);
  for (double t : { 0.1, 0.133, 0.166 }) {
    s.addLeft(t, "L" + std::to_string(t));
    s.addRight(t, "R" + std::to_string(t));
  }
  auto f = s.popReady();
  ASSERT_TRUE(f);
  EXPECT_DOUBLE_EQ(f->t, 0.166);
  EXPECT_EQ(f->dropped_before, 2u);
  EXPECT_GT(f->imu.front().t, 0.05);
  EXPECT_LT(f->imu.front().t, 0.06) << "IMU of skipped frames must be handed over";
  EXPECT_FALSE(s.popReady());
}

TEST(ImuFrameSync, RejectsNonMonotonicImu)
{
  ImuFrameSync<std::string> s;
  EXPECT_TRUE(s.addImu(imu(1.0)));
  EXPECT_FALSE(s.addImu(imu(1.0)));
  EXPECT_FALSE(s.addImu(imu(0.9)));
  EXPECT_TRUE(s.addImu(imu(1.005)));
  EXPECT_EQ(s.rejectedImu(), 2u);
}

TEST(ImuFrameSync, IgnoresImagesOlderThanLastFrame)
{
  ImuFrameSync<std::string> s;
  feedImu(s, 0.0, 1.0);
  s.addLeft(0.5, "L");
  s.addRight(0.5, "R");
  ASSERT_TRUE(s.popReady());
  s.addLeft(0.4, "old");
  s.addRight(0.4, "old");
  EXPECT_FALSE(s.popReady());
  EXPECT_EQ(s.pendingFrames(), 0u);
}

TEST(ImuFrameSync, PendingQueueIsBounded)
{
  SyncConfig cfg;
  cfg.max_pending_frames = 2;
  ImuFrameSync<std::string> s(cfg);
  for (int i = 1; i <= 5; ++i) {
    s.addLeft(i * 0.1, "L");
    s.addRight(i * 0.1, "R");
  }
  EXPECT_EQ(s.pendingFrames(), 2u);
  EXPECT_DOUBLE_EQ(*s.oldestPendingStamp(), 0.4);
}

// ------------------------------------------------------------------ pose math

namespace
{
// Optical camera looking along base +X: cam z = base x, cam x = -base y, cam y = -base z
Iso3 opticalInBase(const Eigen::Vector3d& t)
{
  Iso3 T = Iso3::Identity();
  Eigen::Matrix3d R;
  R << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  T.linear() = R;
  T.translation() = t;
  return T;
}

Iso3 yawPose(double yaw, const Eigen::Vector3d& t)
{
  Iso3 T = Iso3::Identity();
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() = t;
  return T;
}
}  // namespace

TEST(PoseUtils, BaseFromCameraRoundTrip)
{
  const Iso3 T_B_C = opticalInBase({ 0.2, 0.0, 0.3 });
  const Iso3 T_W_B_true = yawPose(0.7, { 1.0, 2.0, 0.0 });
  const Iso3 T_W_C = T_W_B_true * T_B_C;  // what ORB-SLAM3 reports (inverted Tcw)
  const Iso3 T_W_B = orbslam3_ros2::baseInWorld(T_W_C, T_B_C);
  EXPECT_TRUE(T_W_B.isApprox(T_W_B_true, 1e-9));
}

TEST(PoseUtils, AnchorPutsStartAtOriginKeepingGravity)
{
  // start pose: yawed, offset, and slightly pitched (robot on a ramp)
  Iso3 T_W_B0 = yawPose(1.2, { 3.0, -1.0, 0.5 });
  T_W_B0.linear() =
    T_W_B0.linear() * Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY()).toRotationMatrix();

  const Iso3 T_M_W = orbslam3_ros2::makeAnchor(T_W_B0);
  const Iso3 T_M_B0 = T_M_W * T_W_B0;
  EXPECT_NEAR(T_M_B0.translation().norm(), 0.0, 1e-9);
  EXPECT_NEAR(orbslam3_ros2::yawOf(T_M_B0.linear()), 0.0, 1e-9);
  // pitch preserved => anchor rotates about gravity only
  const Eigen::Vector3d z_world_in_M = T_M_W.linear() * Eigen::Vector3d::UnitZ();
  EXPECT_TRUE(z_world_in_M.isApprox(Eigen::Vector3d::UnitZ(), 1e-9));

  // driving 1 m forward from the start shows up as +X in the map frame
  const Iso3 T_W_B1 = T_W_B0 * yawPose(0.0, { 1.0, 0.0, 0.0 });
  const Eigen::Vector3d p = (T_M_W * T_W_B1).translation();
  EXPECT_NEAR(p.x(), std::cos(0.1), 1e-9);
  EXPECT_NEAR(p.y(), 0.0, 1e-9);
  EXPECT_NEAR(p.z(), -std::sin(0.1), 1e-9);
}

TEST(PoseUtils, BodyTwist)
{
  const Iso3 a = yawPose(0.5, { 1.0, 1.0, 0.0 });
  const Iso3 b = a * yawPose(0.1, { 0.2, 0.0, 0.0 });
  Eigen::Vector3d v, w;
  orbslam3_ros2::bodyTwist(a, b, 0.1, v, w);
  EXPECT_NEAR(v.x(), 2.0, 1e-9);
  EXPECT_NEAR(v.y(), 0.0, 1e-9);
  EXPECT_NEAR(w.z(), 1.0, 1e-9);
}
