# ERC Phase 1 — OFFLINE placement demo (no hardware).
# Publishes the saved scan map (.pcd) as a PointCloud2 in camera_init, overlays the
# datum-frame waypoints, and opens RViz so you can see WHERE the 4 datum points land
# on the scanned map.
#
# Run:  ros2 launch ares_erc_bringup place_waypoints.launch.py
#       (optionally  pcd:=/path/to/other.pcd  waypoints_file:=/path/to/wp.yaml)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('ares_erc_bringup')
    default_pcd = os.path.expanduser('~/real_ws/src/ares_erc_bringup/maps/prior_map.pcd')
    default_rviz = os.path.join(share, 'rviz', 'erc_waypoints.rviz')

    pcd = LaunchConfiguration('pcd')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_pcd = DeclareLaunchArgument(
        'pcd', default_value=default_pcd, description='Prior map .pcd to display')
    declare_rviz = DeclareLaunchArgument('rviz_cfg', default_value=default_rviz)

    # Saved map -> PointCloud2 on /erc/prior_map in the `map` frame (offline, no FAST-LIO).
    prior_map = Node(
        package='pcl_ros',
        executable='pcd_to_pointcloud',
        name='prior_map_publisher',
        output='screen',
        parameters=[{
            'file_name': pcd,
            'tf_frame': 'map',
            'publishing_period_ms': 2000,
        }],
        remappings=[('cloud_pcd', '/erc/prior_map')],
    )

    waypoints = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'erc_waypoints.launch.py'))
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_cfg], output='screen',
    )

    return LaunchDescription([declare_pcd, declare_rviz, prior_map, waypoints, rviz])
