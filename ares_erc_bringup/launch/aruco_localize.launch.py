# ERC Phase 3 — live prior-map localization + ArUco map->datum anchoring.
#
# Layers on the Phase-1b/2 stack:
#   1) localize.launch.py (rviz off, datum_tf off) -> FAST-LIO + map_anchor(GICP) +
#      prior map + waypoint markers. erc_waypoints does NOT publish map->datum here.
#   2) realsense_from_lib -> D435i RGB/Depth/CameraInfo + camera_link->optical TF
#   3) static body->camera_link (measured, config/extrinsics_erc.yaml)
#   4) aruco_tracker_autostart (shared aruco_opencv, always-on) -> /aruco_detections
#   5) aruco_map_anchor -> corrects & OWNS map->datum from known-coordinate markers
#   6) RViz (erc_aruco.rviz, Fixed Frame = map)
#
# Run:  ros2 launch ares_erc_bringup aruco_localize.launch.py
# TF authority (single): map->camera_init = map_anchor, camera_init->body = FAST-LIO,
#   body->camera_link = static (this), map->datum = aruco_map_anchor.

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('ares_erc_bringup')
    aruco_share = get_package_share_directory('aruco_opencv')
    rs_share = get_package_share_directory('realsense_from_lib')

    default_pcd = os.path.expanduser('~/real_ws/src/ares_erc_bringup/maps/prior_map.pcd')
    anchors = os.path.join(share, 'config', 'aruco_anchors_erc.yaml')
    rviz_cfg = os.path.join(share, 'rviz', 'erc_aruco.rviz')
    aruco_yaml = os.path.join(aruco_share, 'config', 'aruco_tracker.yaml')
    board_yaml = os.path.join(aruco_share, 'config', 'board_descriptions.yaml')

    pcd = LaunchConfiguration('pcd')
    marker_size = LaunchConfiguration('marker_size')
    marker_dict = LaunchConfiguration('marker_dict')
    global_init = LaunchConfiguration('global_init')
    anchors_file = LaunchConfiguration('anchors_file')
    run_anchor = LaunchConfiguration('run_anchor')
    use_rviz = LaunchConfiguration('rviz')
    declare_pcd = DeclareLaunchArgument('pcd', default_value=default_pcd)
    # ERC Rules Rev.3 spec: 150x150mm (+-2mm). Measured to match on the wall-mounted test
    # markers (id 51/52); override with marker_size:=X for a different-sized test marker.
    declare_ms = DeclareLaunchArgument(
        'marker_size', default_value='0.150', description='ArUco marker side length [m]')
    # ERC Rules Rev.3: real landmarks use a 5x5-grid dictionary (not the shared package's
    # default 4X4_50), IDs 51-64 tentative -- 5X5_100 (IDs 0-99) covers that range.
    declare_md = DeclareLaunchArgument(
        'marker_dict', default_value='5X5_100', description='aruco_opencv dictionary name')
    declare_global_init = DeclareLaunchArgument(
        'global_init', default_value='true',
        description='initialize arbitrary start pose from 2+ known ArUco landmarks before GICP')
    declare_anchors = DeclareLaunchArgument(
        'anchors_file', default_value=anchors,
        description='known ArUco landmark coordinates and gates')
    declare_run_anchor = DeclareLaunchArgument(
        'run_anchor', default_value='true',
        description='run global ArUco localization (false while surveying map anchors)')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true', description='start ERC ArUco RViz')

    # measured body -> camera_link extrinsic
    ext_path = os.path.join(share, 'config', 'extrinsics_erc.yaml')
    with open(ext_path) as f:
        e = (yaml.safe_load(f) or {}).get('body_to_camera_link', {})
    ext = {k: str(float(e.get(k, 0.0))) for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')}

    # 1) localization stack, our RViz, no static datum TF (anchor owns it).
    # Scope the nested ``rviz:=false``. Without the group, that launch configuration
    # leaks back into this file and also disables the ERC ArUco RViz node below.
    localize = GroupAction(
        scoped=True,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'localize.launch.py')),
            launch_arguments={
                'pcd': pcd,
                'rviz': 'false',
                'datum_tf': 'false',
                'wait_for_aruco_init': global_init,
            }.items(),
        )],
    )

    # 2) RealSense D435i.
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rs_share, 'launch', 'publish_realsense.launch.py')),
    )

    # 3) measured body -> camera_link.
    body_to_cam = Node(
        package='tf2_ros', executable='static_transform_publisher', name='body_to_camera_link',
        arguments=['--x', ext['x'], '--y', ext['y'], '--z', ext['z'],
                   '--roll', ext['roll'], '--pitch', ext['pitch'], '--yaw', ext['yaw'],
                   '--frame-id', 'body', '--child-frame-id', 'camera_link'],
    )

    # 4) ArUco detection, always-on (autostart), shared config + overrides (no fork).
    # respawn: a degenerate marker view can throw an uncaught OpenCV exception inside
    # aruco_opencv (solvePnP on a near-collinear corner set) and kill this process outright
    # -- without respawn, ArUco stays dead for the rest of a 20-min run. This restarts the
    # unmodified shared node rather than forking it to add a try/catch (CLAUDE.md §1.4).
    aruco = Node(
        package='aruco_opencv', executable='aruco_tracker_autostart', name='aruco_tracker',
        output='screen', respawn=True, respawn_delay=1.0,
        parameters=[aruco_yaml, {
            'cam_base_topic': '/camera/color/image_raw',
            'marker_size': marker_size,
            'marker_dict': marker_dict,
            'board_descriptions_path': board_yaml,
            # autostart only activates the lifecycle node; image processing itself stays
            # gated by this separate flag (default false) until something publishes true
            # to /aruco/enabled. CLAUDE.md §6.5: ERC wants always-on detection.
            'enabled_by_default': True,
            # Mitigate the recurring cv::solve() "under-determined linear systems" crash
            # (degenerate/near-collinear detected corners feeding solvePnP): SUBPIX is
            # more numerically stable than CONTOUR, and rejecting small/edge-of-frame
            # detections up front avoids the marginal corners most likely to be degenerate.
            'publish_tf': False,                    # ERC adapter publishes correct per-ID TF
            'aruco.cornerRefinementMethod': 1,      # 1=SUBPIX
            'aruco.minDistanceToBorder': 3,
            'aruco.minMarkerPerimeterRate': 0.03,
        }],
        remappings=[('aruco_detections', '/aruco_detections_raw')],
    )

    # 5) Normalize the shared detector's mixed frame convention and publish one
    # correctly-oriented TF per ID.
    adapter = Node(
        package='ares_erc_bringup', executable='aruco_detection_adapter.py',
        name='aruco_detection_adapter', output='screen',
    )

    # 6) ArUco global anchor -> owns map->datum and provides map->camera_init candidates.
    anchor = Node(
        package='ares_erc_bringup', executable='aruco_map_anchor.py', name='aruco_map_anchor',
        output='screen', parameters=[{'anchors_file': anchors_file}],
        condition=IfCondition(run_anchor),
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_cfg], output='screen',
        condition=IfCondition(use_rviz),
    )

    actions = [
        declare_pcd, declare_ms, declare_md, declare_global_init,
        declare_anchors, declare_run_anchor, declare_rviz,
        localize, realsense, body_to_cam, aruco, adapter, anchor, rviz,
    ]
    if all(abs(float(ext[k])) < 1e-9 for k in ext):
        actions.insert(
            4,
            LogInfo(msg=(
                '[WARNING] body->camera_link extrinsic is all zero. '
                'ArUco global localization will be biased until config/extrinsics_erc.yaml '
                'is replaced with measured values.')))
    return LaunchDescription(actions)
