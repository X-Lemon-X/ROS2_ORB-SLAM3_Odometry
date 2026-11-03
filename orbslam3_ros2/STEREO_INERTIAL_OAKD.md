# Stereo-inertial SLAM node (OAK-D, ROS 2 Jazzy)

`stereo_inertial_slam` is a rewrite of the stereo-inertial node. It replaces
`stereo-inertial` / `stereo-inertial-fix`, which are kept only for reference.

## What was wrong with the old node

| Problem | Effect | Fix |
|---|---|---|
| Frames were processed before IMU data up to the image time had arrived | IMU samples landed in the wrong frame interval → inconsistent preintegration → IMU init fails or the trajectory diverges | Frames wait until IMU covers their timestamp (`imu_frame_sync.hpp`, unit tested) |
| Camera-optical → ROS axis swap applied to the world frame | In inertial mode ORB-SLAM3's world is already gravity aligned, Z up; the swap put "up" on X | World used as is; base pose from TF `base_link → camera` |
| Pose stamped with `now()` | Wrong time for TF / fusion | Stamped with the image time |
| No IMU config for the camera (`RealSense_D445.yaml` doesn't exist, D455 file has no `IMU.T_b_c1`) | ORB-SLAM3 aborts or runs with wrong extrinsics | `make_orbslam3_config.py` builds the yaml from the camera itself |
| Unbounded queues, no frame skipping | Latency grows when SLAM is slower than the camera | Bounded queues; stale frames skipped, their IMU kept |

## Quick start

```bash
# 0. driver
sudo apt install ros-jazzy-depthai-ros

# 1. build (ORB-SLAM3 core built as before)
cd ~/ros2_ws && colcon build --packages-select orbslam3 --symlink-install
source install/setup.bash

# 2. start the camera alone, check topic names, generate the settings file
ros2 launch depthai_ros_driver camera.launch.py \
    params_file:=$(ros2 pkg prefix orbslam3)/share/orbslam3/config/oakd/depthai_stereo_imu.yaml
ros2 topic list | grep -E "rect|imu"          # adjust topic args below if names differ
ros2 run orbslam3 make_orbslam3_config.py --out ~/orbslam3_oakd.yaml
# (stop the camera afterwards)

# 3. run everything
ros2 launch orbslam3 oakd_stereo_inertial.launch.py settings:=$HOME/orbslam3_oakd.yaml \
    cam_x:=0.10 cam_z:=0.25            # camera mount position on the robot
```

Move the camera gently for the first ~10–15 s (translate, don't only rotate). The node
publishes nothing until ORB-SLAM3 reports that the IMU is initialised; watch the state:

```bash
ros2 topic echo /orbslam3/state      # LOADING → OK_WAITING_IMU_INIT → OK
```

## Interface

Published (all stamped with the image time):

| Topic / TF | Type | |
|---|---|---|
| `/orbslam3/odom` | `nav_msgs/Odometry` | `map_frame` → `base_frame`, with finite-difference twist |
| `/orbslam3/pose` | `geometry_msgs/PoseStamped` | |
| `/orbslam3/path` | `nav_msgs/Path` | only when subscribed |
| `/orbslam3/state` | `std_msgs/String` | latched; `OK`, `RECENTLY_LOST`, `LOST`, `PAUSED`, ... |
| TF | | `map → base_link`, or `map → odom` when `odom_frame` is set |

Services:

```bash
ros2 service call /orbslam3/pause  std_srvs/srv/Trigger
ros2 service call /orbslam3/resume std_srvs/srv/Trigger
ros2 service call /orbslam3/reset  std_srvs/srv/Trigger              # clear the whole atlas
ros2 service call /orbslam3/reset_active_map std_srvs/srv/Trigger    # clear current map only
ros2 service call /orbslam3/set_localization_mode std_srvs/srv/SetBool "{data: true}"
ros2 service call /orbslam3/save_map std_srvs/srv/Trigger            # needs save_map:=<name>
```

### Maps

```bash
# build a map
ros2 launch orbslam3 oakd_stereo_inertial.launch.py settings:=... save_map:=lab
# ...drive around, then either Ctrl-C (saves on exit) or call /orbslam3/save_map

# reuse it (localization only)
ros2 launch orbslam3 oakd_stereo_inertial.launch.py settings:=... load_map:=lab localization_mode:=true
```

Maps are stored as `<map_directory>/<name>.osa` (default `~/.ros/orbslam3_maps`).
ORB-SLAM3 can only write its atlas during shutdown, so **`save_map` ends the SLAM
session**: relaunch with `load_map:=<name>` to continue. After loading, the robot must
see a mapped place before poses are published in that map's frame.

## Parameters (node)

| Name | Default | |
|---|---|---|
| `vocabulary_file`, `settings_file` | — | required |
| `left_topic`, `right_topic`, `imu_topic` | `/oak/left_rect/image_raw`, `/oak/right_rect/image_raw`, `/oak/imu/data` | |
| `map_frame`, `base_frame` | `map`, `base_link` | |
| `odom_frame` | `""` | set when another node publishes `odom → base_link` (wheel odometry / EKF) |
| `camera_frame` | `""` | default: `frame_id` of the left images |
| `anchor_to_start` | `true` | map origin = robot start pose (yaw only; stays gravity aligned) |
| `publish_before_imu_init` | `false` | poses before IMU init are not gravity aligned and jump at init |
| `publish_recently_lost` | `true` | IMU-predicted poses during short visual loss, with 100× covariance |
| `position_stddev`, `orientation_stddev` | `0.05`, `0.02` | covariance reported in odom |
| `max_stereo_dt` | `0.003` | s, left/right pairing tolerance |
| `max_pending_frames` | `2` | older frames are skipped when SLAM is slower than the camera |
| `imu_timeout` | `0.5` | s, frames waiting longer for IMU are dropped with a warning |
| `use_viewer`, `equalize` | `false` | Pangolin viewer, CLAHE |
| `map_directory`, `load_map`, `save_map`, `save_on_exit` | `~/.ros/orbslam3_maps`, `""`, `""`, `true` | |
| `localization_mode`, `start_paused` | `false` | |

## Getting good IMU results

1. **Calibration matters more than code.** The generated yaml uses the OAK-D factory
   calibration and generic IMU noise values. For best results calibrate camera–IMU with
   [Kalibr](https://github.com/ethz-asl/kalibr) and IMU noise with Allan variance, then
   paste `IMU.T_b_c1` and the four noise values into the yaml.
2. **`IMU.T_b_c1` must be in the IMU message frame.** The node prints that frame at
   startup; the generator uses the same one.
3. **`IMU.Frequency` must be the real rate** (the generator measures it).
4. **Keep the OAK-D Pro dot projector off** (default in the provided driver config).
   Projected dots move with the camera and look like features.
5. **Limit exposure** in dim rooms: motion blur kills tracking faster than anything else.
6. If IMU init takes long or the pose is jittery, increase the four IMU noise values 2–10×.

## Caveats

- The driver parameter and topic names follow depthai-ros v2 for Jazzy; check
  `ros2 param list /oak` and `ros2 topic list` and override the launch arguments if they differ.
- The rectification rotation of the left camera is taken from `camera_info.R` when the
  driver provides it; otherwise it is ignored (well under a degree on factory-calibrated OAKs).
- The ORB-SLAM3 build used here doesn't wait for its threads before saving the atlas;
  the node pauses 2 s before saving to let local mapping finish.
