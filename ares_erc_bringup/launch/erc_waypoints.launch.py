# ERC Phase 1 — start just the datum/waypoint node.
# Broadcasts static TF camera_init->datum and publishes waypoint markers + poses.
# Overlay this on a live map (mapping_mid360.launch.py) or the offline demo
# (place_waypoints.launch.py).

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('ares_erc_bringup')
    default_wp = os.path.join(share, 'config', 'waypoints_erc.yaml')

    waypoints_file = LaunchConfiguration('waypoints_file')
    declare_wp = DeclareLaunchArgument(
        'waypoints_file', default_value=default_wp,
        description='ERC datum-frame waypoints yaml')

    node = Node(
        package='ares_erc_bringup',
        executable='erc_waypoints.py',
        name='erc_waypoints',
        output='screen',
        parameters=[{'waypoints_file': waypoints_file}],
    )
    return LaunchDescription([declare_wp, node])
