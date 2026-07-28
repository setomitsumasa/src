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
    # Phase 3: set false to let aruco_map_anchor own the (dynamic) map->datum TF.
    declare_datum_tf = DeclareLaunchArgument(
        'publish_datum_tf', default_value='true',
        description='publish the static map->datum TF (false when aruco anchor runs)')

    node = Node(
        package='ares_erc_bringup',
        executable='erc_waypoints.py',
        name='erc_waypoints',
        output='screen',
        parameters=[{
            'waypoints_file': waypoints_file,
            'publish_datum_tf': LaunchConfiguration('publish_datum_tf'),
        }],
    )
    return LaunchDescription([declare_wp, declare_datum_tf, node])
