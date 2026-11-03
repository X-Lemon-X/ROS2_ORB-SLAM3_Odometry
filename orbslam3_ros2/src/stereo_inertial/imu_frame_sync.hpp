// Pure C++ (no ROS, no ORB-SLAM3) buffering logic for a stereo-inertial front end.
//
// Responsibilities:
//   * pair left/right images by timestamp,
//   * keep IMU samples strictly ordered,
//   * release a stereo frame only once IMU data covering its timestamp has arrived,
//     together with every IMU sample since the previous released frame,
//   * drop stale frames when SLAM falls behind (IMU samples of dropped frames are kept,
//     so preintegration stays continuous).
//
// Kept free of ROS types so it can be unit tested on its own.

#ifndef ORBSLAM3_ROS2_IMU_FRAME_SYNC_HPP_
#define ORBSLAM3_ROS2_IMU_FRAME_SYNC_HPP_

#include <cmath>
#include <cstddef>
#include <deque>
#include <optional>
#include <utility>
#include <vector>

namespace orbslam3_ros2
{

struct ImuSample
{
  double t{ 0.0 };  // seconds
  float ax{ 0.f }, ay{ 0.f }, az{ 0.f };  // m/s^2, IMU frame
  float gx{ 0.f }, gy{ 0.f }, gz{ 0.f };  // rad/s, IMU frame
};

template<typename ImageT>
struct StereoFrame
{
  double t{ 0.0 };
  ImageT left{};
  ImageT right{};
  std::vector<ImuSample> imu;  // samples in (t_prev_frame, t]
  std::size_t dropped_before{ 0 };  // stale frames skipped to reach this one
};

struct SyncConfig
{
  double max_stereo_dt{ 0.003 };  // s, max |t_left - t_right| for a valid pair
  std::size_t max_image_queue{ 10 };  // per side, oldest images are discarded beyond this
  std::size_t max_pending_frames{ 2 };  // matched pairs waiting for IMU / SLAM
  std::size_t max_imu_queue{ 4000 };  // ~10 s at 400 Hz
};

template<typename ImageT>
class ImuFrameSync
{
 public:
  explicit ImuFrameSync(SyncConfig cfg = SyncConfig{}) : cfg_(cfg) {}

  // ---------------------------------------------------------------- input
  // Returns false if the sample was rejected (non-monotonic timestamp).
  bool addImu(const ImuSample& s)
  {
    if (!std::isfinite(s.t) || (have_last_imu_ && s.t <= last_imu_t_)) {
      ++rejected_imu_;
      return false;
    }
    last_imu_t_ = s.t;
    have_last_imu_ = true;
    imu_.push_back(s);
    while (imu_.size() > cfg_.max_imu_queue) {
      imu_.pop_front();
      imu_overflow_ = true;
    }
    return true;
  }

  void addLeft(double t, ImageT img)
  {
    pushImage(left_, t, std::move(img));
    matchPairs();
  }

  void addRight(double t, ImageT img)
  {
    pushImage(right_, t, std::move(img));
    matchPairs();
  }

  // --------------------------------------------------------------- output
  // Returns the next frame that is ready for SLAM, or nothing if we are still
  // waiting for IMU data. Frames older than the newest ready frame are dropped
  // (their IMU samples are kept and handed out with the returned frame).
  std::optional<StereoFrame<ImageT>> popReady()
  {
    if (pending_.empty() || !have_last_imu_)
      return std::nullopt;

    // Newest pending frame whose timestamp is already covered by IMU data.
    std::ptrdiff_t ready = -1;
    for (std::size_t i = 0; i < pending_.size(); ++i) {
      if (pending_[i].t <= last_imu_t_)
        ready = static_cast<std::ptrdiff_t>(i);
    }
    if (ready < 0)
      return std::nullopt;

    StereoFrame<ImageT> out = std::move(pending_[static_cast<std::size_t>(ready)]);
    out.dropped_before = static_cast<std::size_t>(ready);
    pending_.erase(pending_.begin(), pending_.begin() + ready + 1);

    // Discard IMU from before the very first frame: ORB-SLAM3 only needs data
    // between consecutive frames.
    if (!have_last_frame_) {
      while (!imu_.empty() && imu_.front().t < out.t - kFirstFrameImuWindow)
        imu_.pop_front();
    }

    while (!imu_.empty() && imu_.front().t <= out.t) {
      out.imu.push_back(imu_.front());
      imu_.pop_front();
    }

    last_frame_t_ = out.t;
    have_last_frame_ = true;
    dropped_frames_ += out.dropped_before;
    return out;
  }

  // Oldest pending frame timestamp (to detect a dead IMU stream).
  std::optional<double> oldestPendingStamp() const
  {
    if (pending_.empty())
      return std::nullopt;
    return pending_.front().t;
  }

  // Drop the oldest pending frame (used when IMU never arrives for it).
  void dropOldestPending()
  {
    if (!pending_.empty()) {
      pending_.pop_front();
      ++dropped_frames_;
    }
  }

  void clear()
  {
    left_.clear();
    right_.clear();
    pending_.clear();
    imu_.clear();
    have_last_imu_ = false;
    have_last_frame_ = false;
    last_imu_t_ = 0.0;
    last_frame_t_ = 0.0;
  }

  // ------------------------------------------------------------- statistics
  bool hasImu() const { return have_last_imu_; }
  double lastImuStamp() const { return last_imu_t_; }
  std::size_t pendingFrames() const { return pending_.size(); }
  std::size_t imuQueueSize() const { return imu_.size(); }
  std::size_t droppedFrames() const { return dropped_frames_; }
  std::size_t rejectedImu() const { return rejected_imu_; }
  std::size_t unmatchedImages() const { return unmatched_images_; }
  bool takeImuOverflowFlag()
  {
    bool f = imu_overflow_;
    imu_overflow_ = false;
    return f;
  }

 private:
  struct Stamped
  {
    double t;
    ImageT img;
  };

  static constexpr double kFirstFrameImuWindow = 0.5;  // s of IMU kept before frame 0

  void pushImage(std::deque<Stamped>& q, double t, ImageT img)
  {
    if (!std::isfinite(t))
      return;
    // Ignore images not newer than the last released / queued frame.
    if (have_last_frame_ && t <= last_frame_t_)
      return;
    if (!q.empty() && t <= q.back().t)
      return;
    q.push_back(Stamped{ t, std::move(img) });
    while (q.size() > cfg_.max_image_queue) {
      q.pop_front();
      ++unmatched_images_;
    }
  }

  void matchPairs()
  {
    while (!left_.empty() && !right_.empty()) {
      const double tl = left_.front().t;
      const double tr = right_.front().t;
      const double dt = tl - tr;
      if (std::abs(dt) <= cfg_.max_stereo_dt) {
        StereoFrame<ImageT> f;
        f.t = tl;  // ORB-SLAM3 uses the left image time
        f.left = std::move(left_.front().img);
        f.right = std::move(right_.front().img);
        left_.pop_front();
        right_.pop_front();
        if (!pending_.empty() && f.t <= pending_.back().t)
          continue;
        pending_.push_back(std::move(f));
        while (pending_.size() > cfg_.max_pending_frames) {
          pending_.pop_front();
          ++dropped_frames_;
        }
      } else if (dt > 0) {
        // right is older and has no partner
        right_.pop_front();
        ++unmatched_images_;
      } else {
        left_.pop_front();
        ++unmatched_images_;
      }
    }
  }

  SyncConfig cfg_;
  std::deque<Stamped> left_, right_;
  std::deque<StereoFrame<ImageT>> pending_;
  std::deque<ImuSample> imu_;

  bool have_last_imu_{ false };
  double last_imu_t_{ 0.0 };
  bool have_last_frame_{ false };
  double last_frame_t_{ 0.0 };

  std::size_t dropped_frames_{ 0 };
  std::size_t rejected_imu_{ 0 };
  std::size_t unmatched_images_{ 0 };
  bool imu_overflow_{ false };
};

}  // namespace orbslam3_ros2

#endif  // ORBSLAM3_ROS2_IMU_FRAME_SYNC_HPP_
