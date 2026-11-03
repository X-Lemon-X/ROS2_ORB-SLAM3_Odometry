"""ORB-SLAM3 stereo-inertial on a Luxonis OAK-D.

  ros2 launch orbslam3 oakd_stereo_inertial.launch.py settings:=~/orbslam3_oakd.yaml

Starts (optionally) the depthai driver, a static TF base_link -> camera, and the SLAM node.
Generate the settings file once with:
  ros2 run orbslam3 make_orbslam3_config.py --out ~/orbslam3_oakd.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('orbslam3')

    args = [
        DeclareLaunchArgument('settings', description='ORB-SLAM3 settings yaml for your camera'),
        DeclareLaunchArgument('vocabulary',
                              default_value=os.path.join(share, 'vocabulary', 'ORBvoc.txt')),
        # camera driver
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('camera_name', default_value='oak'),
        DeclareLaunchArgument('camera_params',
                              default_value=os.path.join(share, 'config', 'oakd',
                                                         'depthai_stereo_imu.yaml')),
        DeclareLaunchArgument('left_topic', default_value='/oak/left_rect/image_raw'),
        DeclareLaunchArgument('right_topic', default_value='/oak/right_rect/image_raw'),
        DeclareLaunchArgument('imu_topic', default_value='/oak/imu/data'),
        # frames
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='',
                              description='Set (e.g. "odom") if wheel odometry/EKF publishes '
                                          'odom->base_link; SLAM then publishes map->odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('publish_camera_tf', default_value='true',
                              description='Publish base_frame -> camera_parent_frame below'),
        DeclareLaunchArgument('camera_parent_frame', default_value='oak-d-base-frame'),
        DeclareLaunchArgument('cam_x', default_value='0.0'),
        DeclareLaunchArgument('cam_y', default_value='0.0'),
        DeclareLaunchArgument('cam_z', default_value='0.0'),
        DeclareLaunchArgument('cam_roll', default_value='0.0'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0'),
        DeclareLaunchArgument('cam_yaw', default_value='0.0'),
        # SLAM behaviour
        DeclareLaunchArgument('use_viewer', default_value='false'),
        DeclareLaunchArgument('map_directory', default_value='~/.ros/orbslam3_maps'),
        DeclareLaunchArgument('load_map', default_value=''),
        DeclareLaunchArgument('save_map', default_value=''),
        DeclareLaunchArgument('localization_mode', default_value='false'),
    ]

    lc = LaunchConfiguration

    camera = OpaqueFunction(function=_camera)

    camera_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='base_to_camera_tf',
        arguments=['--x', lc('cam_x'), '--y', lc('cam_y'), '--z', lc('cam_z'),
                   '--roll', lc('cam_roll'), '--pitch', lc('cam_pitch'), '--yaw', lc('cam_yaw'),
                   '--frame-id', lc('base_frame'), '--child-frame-id', lc('camera_parent_frame')],
        condition=IfCondition(lc('publish_camera_tf')),
    )

    def s(name):
        return ParameterValue(lc(name), value_type=str)

    def b(name):
        return ParameterValue(lc(name), value_type=bool)

    slam = Node(
        package='orbslam3', executable='stereo_inertial_slam', name='orbslam3', output='screen',
        parameters=[{
            'vocabulary_file': s('vocabulary'),
            'settings_file': s('settings'),
            'left_topic': s('left_topic'),
            'right_topic': s('right_topic'),
            'imu_topic': s('imu_topic'),
            'map_frame': s('map_frame'),
            'odom_frame': s('odom_frame'),
            'base_frame': s('base_frame'),
            'use_viewer': b('use_viewer'),
            'map_directory': s('map_directory'),
            'load_map': s('load_map'),
            'save_map': s('save_map'),
            'localization_mode': b('localization_mode'),
        }],
    )

    return LaunchDescription(args + [camera, camera_tf, slam])


def _camera(context):
    if LaunchConfiguration('start_camera').perform(context).lower() != 'true':
        return []
    try:
        depthai = get_package_share_directory('depthai_ros_driver')
    except Exception:
        raise RuntimeError('depthai_ros_driver is not installed: '
                           'sudo apt install ros-jazzy-depthai-ros  (or start_camera:=false)')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(depthai, 'launch', 'camera.launch.py')),
        launch_arguments={
            'name': LaunchConfiguration('camera_name'),
            'params_file': LaunchConfiguration('camera_params'),
        }.items(),
    )]
