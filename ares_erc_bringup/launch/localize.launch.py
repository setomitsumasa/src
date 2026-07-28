# ERC Phase 1b — live prior-map relocalization.
#
# Layers the localization stack on top of the Phase-0 mapping bring-up:
#   1) mapping_mid360.launch.py (rviz off) -> Livox Mid-360 + FAST-LIO2
#         publishes camera_init->body (odom) and /cloud_registered (camera_init frame)
#   2) map_anchor -> GICP-aligns /cloud_registered to the saved prior .pcd and
#         broadcasts the low-passed  map -> camera_init  correction (§6.3)
#   3) erc_waypoints -> static map->datum (yaw) + waypoint markers/poses (Phase 1a)
#   4) pcd_to_pointcloud -> the prior map as /erc/prior_map in `map` (for RViz overlay)
#   5) RViz (Fixed Frame = map): prior map (fixed) vs live /cloud_registered should overlap
#
# Run:   ros2 launch ares_erc_bringup localize.launch.py
# TF authority (single, no doubling): map->camera_init = map_anchor,
#   camera_init->body = FAST-LIO, map->datum = erc_waypoints.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('ares_erc_bringup')
    default_pcd = os.path.expanduser('~/real_ws/src/ares_erc_bringup/maps/prior_map.pcd')
    default_loc = os.path.join(share, 'config', 'localization.yaml')
    default_rviz = os.path.join(share, 'rviz', 'erc_localize.rviz')

    pcd = LaunchConfiguration('pcd')
    loc_cfg = LaunchConfiguration('loc_config')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_pcd = DeclareLaunchArgument(
        'pcd', default_value=default_pcd, description='Prior map .pcd (map frame)')
    declare_loc = DeclareLaunchArgument('loc_config', default_value=default_loc)
    declare_rviz = DeclareLaunchArgument('rviz_cfg', default_value=default_rviz)
    # Allow a parent launch (e.g. odometry.launch.py) to own RViz itself.
    declare_use_rviz = DeclareLaunchArgument(
        'rviz', default_value='true', description='start RViz (false when included)')
    # Phase 3: aruco_localize sets this false so aruco_map_anchor owns map->datum.
    declare_datum_tf = DeclareLaunchArgument(
        'datum_tf', default_value='true',
        description='let erc_waypoints publish the static map->datum TF')
    declare_wait_aruco = DeclareLaunchArgument(
        'wait_for_aruco_init', default_value='false',
        description='hold GICP until a consistent multi-marker global pose is available')

    # 1) Mid-360 + FAST-LIO2 (own RViz off; we use our map-frame RViz below).
    # Scope the child launch argument. Without GroupAction(scoped=True), its
    # rviz:=false leaked into this launch and disabled the map-frame RViz below.
    mapping = GroupAction(
        scoped=True,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'mapping_mid360.launch.py')),
            launch_arguments={'rviz': 'false'}.items(),
        )],
    )

    # 2) Relocalization: map -> camera_init correction via GICP on the prior map.
    map_anchor = Node(
        package='ares_erc_bringup',
        executable='map_anchor',
        name='map_anchor',
        output='screen',
        parameters=[loc_cfg, {
            'prior_map_path': pcd,
            'wait_for_aruco_initialization': ParameterValue(
                LaunchConfiguration('wait_for_aruco_init'), value_type=bool),
        }],
    )

    # 3) datum static TF + waypoint markers/poses (Phase 1a node, map-frame default).
    waypoints = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'erc_waypoints.launch.py')),
        launch_arguments={'publish_datum_tf': LaunchConfiguration('datum_tf')}.items(),
    )

    # 4) Prior map as a PointCloud2 in `map` for the RViz overlay.
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

    # 5) RViz, Fixed Frame = map.
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_cfg], output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        declare_pcd, declare_loc, declare_rviz, declare_use_rviz, declare_datum_tf,
        declare_wait_aruco,
        mapping, map_anchor, waypoints, prior_map, rviz,
    ])
