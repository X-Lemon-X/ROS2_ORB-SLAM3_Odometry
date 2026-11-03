#!/usr/bin/env python3
"""Generate an ORB-SLAM3 stereo-inertial settings file from a running camera driver.

Reads, while the camera driver is running:
  * left/right rectified camera_info  -> fx, fy, cx, cy, image size, baseline
  * image rate                        -> Camera.fps
  * IMU messages                      -> IMU frame name and real IMU.Frequency
  * TF (IMU frame <- left camera)     -> IMU.T_b_c1

The IMU noise values written are starting points only. For best results, calibrate
camera-IMU extrinsics and IMU noise with Kalibr / allan_variance_ros and paste them in.

Usage:
  ros2 run orbslam3 make_orbslam3_config.py --out ~/orbslam3_oakd.yaml
  ros2 run orbslam3 make_orbslam3_config.py --left /oak/left_rect/image_raw \
      --right /oak/right_rect/image_raw --imu /oak/imu/data --out oakd.yaml
"""

import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu
from tf2_ros import Buffer, TransformListener


def info_topic(image_topic: str) -> str:
    return image_topic.rsplit('/', 1)[0] + '/camera_info'


def quat_to_rot(x, y, z, w):
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def matmul3(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def transpose3(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def is_identity(r, tol=1e-6):
    return all(abs(r[i][j] - (1.0 if i == j else 0.0)) < tol for i in range(3) for j in range(3))


class Collector(Node):

    def __init__(self, args):
        super().__init__('orbslam3_config_generator')
        self.args = args
        self.left_info = None
        self.right_info = None
        self.imu_stamps = []
        self.img_stamps = []
        self.imu_frame = None
        self.cam_frame = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, args.left_info or info_topic(args.left),
                                 self.on_left_info, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, args.right_info or info_topic(args.right),
                                 self.on_right_info, qos_profile_sensor_data)
        self.create_subscription(Imu, args.imu, self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Image, args.left, self.on_img, qos_profile_sensor_data)

    def on_left_info(self, msg):
        self.left_info = msg

    def on_right_info(self, msg):
        self.right_info = msg

    def on_imu(self, msg):
        self.imu_frame = msg.header.frame_id
        self.imu_stamps.append(Time.from_msg(msg.header.stamp).nanoseconds * 1e-9)

    def on_img(self, msg):
        self.cam_frame = msg.header.frame_id
        self.img_stamps.append(Time.from_msg(msg.header.stamp).nanoseconds * 1e-9)


def rate(stamps):
    s = sorted(set(stamps))
    if len(s) < 10:
        return None
    return (len(s) - 1) / (s[-1] - s[0])


def fail(msg):
    print('ERROR: ' + msg, file=sys.stderr)
    sys.exit(1)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--left', default='/oak/left_rect/image_raw')
    p.add_argument('--right', default='/oak/right_rect/image_raw')
    p.add_argument('--left-info', default='', help='default: <left ns>/camera_info')
    p.add_argument('--right-info', default='', help='default: <right ns>/camera_info')
    p.add_argument('--imu', default='/oak/imu/data')
    p.add_argument('--camera-frame', default='',
                   help='TF frame of the left camera (default: frame_id of left images)')
    p.add_argument('--imu-frame', default='',
                   help='TF frame of the IMU (default: frame_id of IMU messages)')
    p.add_argument('--seconds', type=float, default=5.0, help='how long to listen')
    p.add_argument('--features', type=int, default=1200)
    p.add_argument('--out', required=True)
    args, ros_args = p.parse_known_args()

    rclpy.init(args=ros_args)
    node = Collector(args)
    t_end = time.time() + args.seconds
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)

    li, ri = node.left_info, node.right_info
    if li is None or ri is None:
        fail('no camera_info received on %s / %s. Check the topics with "ros2 topic list".'
             % (args.left_info or info_topic(args.left), args.right_info or info_topic(args.right)))
    if node.imu_frame is None:
        fail('no IMU messages on ' + args.imu)
    if node.cam_frame is None:
        fail('no images on ' + args.left)

    imu_frame = args.imu_frame or node.imu_frame
    cam_frame = args.camera_frame or node.cam_frame or li.header.frame_id
    right_frame = ri.header.frame_id

    # --- intrinsics of the rectified pair (projection matrix P)
    fx, fy, cx, cy = li.p[0], li.p[5], li.p[2], li.p[6]
    if fx <= 0:
        fail('left camera_info has an empty P matrix: is this the rectified stream?')
    if any(abs(c) > 1e-6 for c in li.d):
        print('WARNING: left camera_info has non-zero distortion. ORB-SLAM3 "Rectified" mode '
              'expects rectified, undistorted images (use the *_rect topics).')

    # --- TF lookups (static, so time 0)
    def lookup(target, source):
        end = time.time() + 5.0
        while time.time() < end:
            try:
                return node.tf_buffer.lookup_transform(target, source, Time())
            except Exception:
                rclpy.spin_once(node, timeout_sec=0.1)
        return None

    # --- baseline: prefer P of the right camera, cross-check with TF
    b_p = -ri.p[3] / ri.p[0] if ri.p[0] > 0 else 0.0
    b_tf = None
    tf_lr = lookup(cam_frame, right_frame) if right_frame and right_frame != cam_frame else None
    if tf_lr is not None:
        t = tf_lr.transform.translation
        b_tf = math.sqrt(t.x ** 2 + t.y ** 2 + t.z ** 2)
    baseline = b_p
    if b_p <= 0 or (b_tf and abs(b_p - b_tf) / b_tf > 0.2):
        if b_tf:
            print('NOTE: baseline from right P (%.4f m) is unusable or disagrees with TF '
                  '(%.4f m). Using TF.' % (b_p, b_tf))
            baseline = b_tf
        elif b_p > 1.0:  # probably millimetres
            baseline = b_p / 1000.0
            print('NOTE: baseline from P looks like millimetres, converted to %.4f m' % baseline)
    if baseline <= 0:
        fail('could not determine the stereo baseline from camera_info or TF')

    # --- IMU <- camera extrinsic
    tf_bc = lookup(imu_frame, cam_frame)
    if tf_bc is None:
        fail('no TF between IMU frame "%s" and camera frame "%s". Is the driver publishing '
             'its URDF / robot_state_publisher?' % (imu_frame, cam_frame))
    q = tf_bc.transform.rotation
    t = tf_bc.transform.translation
    R_bc = quat_to_rot(q.x, q.y, q.z, q.w)
    # Rectification rotation: rectified camera = R * raw camera  ->  R_b_rect = R_b_raw * R^T
    R_rect = [list(li.r[0:3]), list(li.r[3:6]), list(li.r[6:9])]
    if not is_identity(R_rect) and any(abs(v) > 0 for v in li.r):
        R_bc = matmul3(R_bc, transpose3(R_rect))
        print('Applied left rectification rotation from camera_info.R to IMU.T_b_c1')

    imu_hz = rate(node.imu_stamps)
    cam_hz = rate(node.img_stamps)
    if imu_hz is None:
        fail('not enough IMU messages to measure the rate')
    if imu_hz < 100:
        print('WARNING: IMU rate is only %.0f Hz. ORB-SLAM3 works best with >= 200 Hz.' % imu_hz)
    fps = int(round(cam_hz)) if cam_hz else 30

    def row(r, tt):
        return '%.6f, %.6f, %.6f, %.6f' % (r[0], r[1], r[2], tt)

    tbc = ',\n         '.join([
        row(R_bc[0], t.x), row(R_bc[1], t.y), row(R_bc[2], t.z), '0.0, 0.0, 0.0, 1.0'])

    yaml = f"""%YAML:1.0
# ORB-SLAM3 stereo-inertial settings generated by make_orbslam3_config.py
#   left:  {args.left}  (frame {cam_frame})
#   right: {args.right} (frame {right_frame})
#   imu:   {args.imu}  (frame {imu_frame}, measured {imu_hz:.1f} Hz)
# Intrinsics/extrinsics come from the device's factory calibration. For best accuracy
# replace IMU.T_b_c1 and the IMU noise values with a Kalibr calibration.

File.version: "1.0"

#--------------------------------------------------------------------------------------------
# Camera (rectified stereo pair from the driver)
#--------------------------------------------------------------------------------------------
Camera.type: "Rectified"

Camera1.fx: {fx:.6f}
Camera1.fy: {fy:.6f}
Camera1.cx: {cx:.6f}
Camera1.cy: {cy:.6f}

Stereo.b: {baseline:.6f}

Camera.width: {li.width}
Camera.height: {li.height}
Camera.fps: {fps}
# 0: BGR, 1: RGB (ignored for grayscale)
Camera.RGB: 0

# Close/far threshold, in baselines
Stereo.ThDepth: 40.0

#--------------------------------------------------------------------------------------------
# IMU
#--------------------------------------------------------------------------------------------
# Transformation from left camera to IMU (body) frame: p_imu = T_b_c1 * p_cam
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [{tbc}]

# Noise densities / random walks (continuous time). Starting values; tune with Allan variance.
# If initialisation is slow or the trajectory is jittery, increase these by 2-10x.
IMU.NoiseGyro: 1.0e-3   # rad/s/sqrt(Hz)
IMU.NoiseAcc: 1.0e-2    # m/s^2/sqrt(Hz)
IMU.GyroWalk: 1.0e-6    # rad/s^2/sqrt(Hz)
IMU.AccWalk: 1.0e-4     # m/s^3/sqrt(Hz)
IMU.Frequency: {imu_hz:.1f}

#--------------------------------------------------------------------------------------------
# ORB features
#--------------------------------------------------------------------------------------------
ORBextractor.nFeatures: {args.features}
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

#--------------------------------------------------------------------------------------------
# Viewer (only used with use_viewer:=true)
#--------------------------------------------------------------------------------------------
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -3.5
Viewer.ViewpointF: 500.0
"""
    out = os.path.abspath(os.path.expanduser(args.out))
    with open(out, 'w') as f:
        f.write(yaml)

    print('\nWrote ' + out)
    print('  fx=%.2f fy=%.2f cx=%.2f cy=%.2f  %dx%d @ %d fps' % (fx, fy, cx, cy, li.width,
                                                              li.height, fps))
    print('  baseline=%.4f m   IMU %.1f Hz in frame "%s"' % (baseline, imu_hz, imu_frame))
    print('  T_b_c1 translation = [%.4f %.4f %.4f]' % (t.x, t.y, t.z))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
