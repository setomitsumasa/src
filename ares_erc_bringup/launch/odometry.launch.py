# ERC Phase 2 (Milestone 2a) — live global odometry output + display.
#
# Runs the full Phase-1b localization stack (FAST-LIO + map_anchor GICP + prior map +
# waypoints) with its own RViz OFF, then adds:
#   * erc_odometry -> composes the live map->body TF into /erc/odometry (nav_msgs/Odometry,
#     frame_id=map) and /erc/trajectory (nav_msgs/Path),
#   * RViz with erc_odometry.rviz -> prior map (grey) + live cloud (red) + waypoints
#     + the odometry pose arrow + the accumulated trajectory.
#
# Run:  ros2 launch ares_erc_bringup odometry.launch.py
# The robot's drift-corrected global pose (map frame) is what Nav2 will consume in 2b.

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
    default_rviz = os.path.join(share, 'rviz', 'erc_odometry.rviz')

    pcd = LaunchConfiguration('pcd')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_pcd = DeclareLaunchArgument('pcd', default_value=default_pcd)
    declare_rviz = DeclareLaunchArgument('rviz_cfg', default_value=default_rviz)

    # Phase 1b localization stack, but let THIS launch own RViz.
    localize = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'localize.launch.py')),
        launch_arguments={'pcd': pcd, 'rviz': 'false'}.items(),
    )

    # Global odometry: compose map->body from TF -> /erc/odometry + /erc/trajectory.
    odometry = Node(
        package='ares_erc_bringup', executable='erc_odometry.py', name='erc_odometry',
        output='screen',
        parameters=[{'map_frame': 'map', 'body_frame': 'body', 'publish_rate': 30.0}],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_cfg], output='screen',
    )

    return LaunchDescription([declare_pcd, declare_rviz, localize, odometry, rviz])
