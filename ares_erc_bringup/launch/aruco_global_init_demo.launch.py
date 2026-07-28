# Headless arbitrary-start localization test (no camera / no LiDAR).
#
# A synthetic camera_init is really at (1.0, 2.0, 10 deg) in map, while map_anchor
# starts at identity and disables GICP. Three known markers must recover the hidden
# pose and set /erc/localization_initialized=true.
#
# Run:
#   ros2 launch ares_erc_bringup aruco_global_init_demo.launch.py
# Check:
#   ros2 run tf2_ros tf2_echo map camera_init
#   ros2 topic echo /erc/localization_initialized --once

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('ares_erc_bringup')
    anchors = os.path.join(share, 'config', 'aruco_anchors_global_demo.yaml')

    map_anchor = Node(
        package='ares_erc_bringup', executable='map_anchor',
        name='map_anchor', output='screen',
        parameters=[{
            # The demo exercises ArUco initialization only; no prior cloud is needed.
            'prior_map_path': '',
            'wait_for_aruco_initialization': True,
            'aruco_init_min_candidates': 3,
            'aruco_init_consistency_trans': 0.05,
            'aruco_init_consistency_rot': 0.05,
            'aruco_residual_max': 0.05,
        }],
    )
    anchor = Node(
        package='ares_erc_bringup', executable='aruco_map_anchor.py',
        name='aruco_map_anchor', output='screen',
        parameters=[{'anchors_file': anchors}],
    )
    fake = Node(
        package='ares_erc_bringup', executable='aruco_fake_detections.py',
        name='aruco_fake_detections', output='screen',
        parameters=[{
            'anchors_file': anchors,
            'parent_frame': 'camera_init',
            'true_yaw_deg': 30.0,
            'true_xy': [0.2, -0.1],
            'true_parent_yaw_deg': 10.0,
            'true_parent_xy': [1.0, 2.0],
        }],
    )
    return LaunchDescription([map_anchor, anchor, fake])
