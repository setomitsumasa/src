"""Survey indoor ArUco landmark centres directly into the prior-map frame.

The rig must begin at the same pose used for mapping, where local GICP is known to
lock reliably.  Global ArUco initialization is disabled to avoid circularly using the
coordinates being calibrated.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('ares_erc_bringup')
    localize = os.path.join(share, 'launch', 'aruco_localize.launch.py')

    pcd = LaunchConfiguration('pcd')
    output_file = LaunchConfiguration('output_file')
    duration = LaunchConfiguration('duration_sec')
    fitness_max = LaunchConfiguration('fitness_max')
    marker_size = LaunchConfiguration('marker_size')
    marker_dict = LaunchConfiguration('marker_dict')
    template_file = LaunchConfiguration('template_file')

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localize),
        launch_arguments={
            'pcd': pcd,
            'marker_size': marker_size,
            'marker_dict': marker_dict,
            'global_init': 'false',
            'run_anchor': 'false',
            'rviz': 'true',
        }.items(),
    )
    calibrator = Node(
        package='ares_erc_bringup',
        executable='aruco_map_calibrator.py',
        name='aruco_map_calibrator',
        output='screen',
        parameters=[{
            'output_file': output_file,
            'duration_sec': duration,
            'fitness_max': fitness_max,
            'template_file': template_file,
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'pcd',
            default_value=os.path.expanduser(
                '~/real_ws/src/ares_erc_bringup/maps/prior_map.pcd')),
        DeclareLaunchArgument(
            'output_file',
            default_value='/tmp/aruco_anchors_measured.yaml'),
        DeclareLaunchArgument('duration_sec', default_value='15.0'),
        DeclareLaunchArgument(
            'fitness_max', default_value='0.02',
            description='collect only while GICP fitness is at most this value'),
        DeclareLaunchArgument('marker_size', default_value='0.150'),
        DeclareLaunchArgument('marker_dict', default_value='5X5_100'),
        DeclareLaunchArgument(
            'template_file',
            default_value=os.path.join(
                share, 'config', 'aruco_anchors_erc.yaml'),
            description='copy gates/datum settings from this file, replacing markers'),
        stack,
        calibrator,
    ])
