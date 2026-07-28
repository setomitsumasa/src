# ERC Phase 3 — OFFLINE ArUco anchor convergence demo (no camera / no LiDAR).
#
# aruco_fake_detections injects synthetic /aruco_detections placing the known markers at
# a ground-truth map->datum (default 30 deg / 0.2,-0.1). aruco_map_anchor, seeded wrong
# (0 deg), must converge map->datum to that truth. In RViz (Fixed Frame = map) the
# yellow known-marker cubes (datum frame) slide onto the blue observed spheres (map frame).
#
# Run:  ros2 launch ares_erc_bringup aruco_anchor_demo.launch.py
#       (optionally  true_yaw_deg:=45.0  true_xy:="[0.3, 0.2]")
# Check: ros2 run tf2_ros tf2_echo map datum   -> converges to the ground truth
#        ros2 topic echo /erc/aruco_anchor_residual  -> ~0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('ares_erc_bringup')
    anchors = os.path.join(share, 'config', 'aruco_anchors_demo.yaml')
    rviz_cfg = os.path.join(share, 'rviz', 'erc_aruco.rviz')

    declare_yaw = DeclareLaunchArgument('true_yaw_deg', default_value='30.0')
    declare_xy = DeclareLaunchArgument('true_xy', default_value='[0.2, -0.1]')

    fake = Node(
        package='ares_erc_bringup', executable='aruco_fake_detections.py',
        name='aruco_fake_detections', output='screen',
        parameters=[{
            'anchors_file': anchors,
            'true_yaw_deg': LaunchConfiguration('true_yaw_deg'),
            'true_xy': LaunchConfiguration('true_xy'),
        }],
    )
    anchor = Node(
        package='ares_erc_bringup', executable='aruco_map_anchor.py',
        name='aruco_map_anchor', output='screen',
        parameters=[{'anchors_file': anchors}],
    )
    # waypoints markers only (no static map->datum: the anchor owns it here).
    waypoints = Node(
        package='ares_erc_bringup', executable='erc_waypoints.py', name='erc_waypoints',
        output='screen',
        parameters=[{
            'waypoints_file': os.path.join(share, 'config', 'waypoints_erc.yaml'),
            'publish_datum_tf': False,
        }],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_cfg], output='screen',
    )
    return LaunchDescription([declare_yaw, declare_xy, fake, anchor, waypoints, rviz])
