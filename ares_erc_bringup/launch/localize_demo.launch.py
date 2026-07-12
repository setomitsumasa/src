# ERC Phase 1b — OFFLINE visualization of relocalization (no hardware).
#
# There is no LiDAR here, so we FAKE the live cloud by replaying the prior map itself
# as /cloud_registered (in camera_init). map_anchor then GICP-aligns it to the prior
# map. We deliberately start with an offset (init_xyz/init_yaw) so you can WATCH the
# red "live" cloud slide onto the grey prior map as the correction converges.
#
# Run:  ros2 launch ares_erc_bringup localize_demo.launch.py
# In RViz (Fixed Frame = map): grey = prior map (fixed), red = fake live cloud.
#   They start offset, then GICP pulls red onto grey within ~15 s.
#
# For the REAL thing (live Mid-360), use localize.launch.py instead.

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
    default_rviz = os.path.join(share, 'rviz', 'erc_localize.rviz')

    pcd = LaunchConfiguration('pcd')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_pcd = DeclareLaunchArgument('pcd', default_value=default_pcd)
    declare_rviz = DeclareLaunchArgument('rviz_cfg', default_value=default_rviz)

    # Prior map (grey, fixed) in `map`.
    prior_map = Node(
        package='pcl_ros', executable='pcd_to_pointcloud', name='prior_map_publisher',
        output='screen',
        parameters=[{'file_name': pcd, 'tf_frame': 'map', 'publishing_period_ms': 2000}],
        remappings=[('cloud_pcd', '/erc/prior_map')],
    )

    # Fake "live" cloud (red) = the same map replayed in camera_init.
    fake_live = Node(
        package='pcl_ros', executable='pcd_to_pointcloud', name='fake_live_publisher',
        output='screen',
        parameters=[{'file_name': pcd, 'tf_frame': 'camera_init', 'publishing_period_ms': 1000}],
        remappings=[('cloud_pcd', '/cloud_registered')],
    )

    # map_anchor with a deliberate initial offset so the convergence is visible.
    map_anchor = Node(
        package='ares_erc_bringup', executable='map_anchor', name='map_anchor',
        output='screen',
        parameters=[os.path.join(share, 'config', 'localization.yaml'), {
            'prior_map_path': pcd,
            'init_xyz': [0.6, 0.0, 0.0],
            'init_yaw': 0.15,
            'voxel_leaf': 0.5,
            'accum_window_sec': 1.2,
            'update_period': 1.0,
            'lowpass_alpha': 0.3,
            'fitness_max': 1.0,
        }],
    )

    waypoints = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'erc_waypoints.launch.py'))
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_cfg], output='screen',
    )

    return LaunchDescription([
        declare_pcd, declare_rviz, prior_map, fake_live, map_anchor, waypoints, rviz,
    ])
