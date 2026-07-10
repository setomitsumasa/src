# ERC Phase 0 — one-shot bring-up for Livox Mid-360 + FAST-LIO2 mapping.
#
# Starts:
#   1) livox_ros_driver2 node (Mid-360) -> /livox/lidar (CustomMsg) + /livox/imu
#      using ares_erc_bringup/config/MID360_config_erc.json (driver extrinsic = 0).
#   2) fast_lio (mapping.launch.py) -> odometry + registered cloud, saves .pcd
#      using ares_erc_bringup/config/mid360_mapping.yaml + fast_lio's RViz.
#
# Run:   ros2 launch ares_erc_bringup mapping_mid360.launch.py
# Stop:  Ctrl-C  -> on shutdown FAST-LIO writes the map to <FAST_LIO_ROS2>/PCD/scans.pcd
#
# WHY a dedicated Mid-360 config (MID360_config_erc.json): the workspace's
# livox_ros_driver2/config/MID360_config.json bakes in a rover-mount extrinsic
# (roll/pitch/yaw = 0/180/180, z=300mm). The driver applies that ONLY to the point
# cloud, NOT the IMU, so LiDAR and IMU disagree by 180deg -> FAST-LIO tracks fine when
# still but DIVERGES the moment you move/rotate. Our ERC config zeroes the driver
# extrinsic (standard Mid-360 + FAST-LIO setup); the small LiDAR<->IMU offset is handled
# by FAST-LIO's own extrinsic_T/R in mid360_mapping.yaml. URC config is left untouched.
#
# NOTE: URC stack is untouched. Do NOT run this together with the URC bring-up:
# FAST-LIO owns map->odom(->body), which must stay a single authority (CLAUDE.md §6).

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fast_lio_share = get_package_share_directory('fast_lio')
    erc_share = get_package_share_directory('ares_erc_bringup')

    rviz = LaunchConfiguration('rviz')

    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz to monitor mapping (uses fast_lio/rviz/fastlio.rviz)'
    )

    # 1) Livox Mid-360 driver -> CustomMsg on /livox/lidar (+ /livox/imu)
    #    Same params as livox_ros_driver2/launch_ROS2/msg_MID360_launch.py, but the
    #    user_config_path points at our ERC config with a ZEROED driver extrinsic.
    mid360_cfg = os.path.join(erc_share, 'config', 'MID360_config_erc.json')
    livox = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {'xfer_format': 1},        # 1 = Livox CustomMsg (required by FAST-LIO)
            {'multi_topic': 0},
            {'data_src': 0},
            {'publish_freq': 10.0},
            {'output_data_type': 0},
            {'frame_id': 'livox_frame'},
            {'lvx_file_path': '/home/livox/livox_test.lvx'},
            {'user_config_path': mid360_cfg},
            {'cmdline_input_bd_code': 'livox0000000001'},
        ],
    )

    # 2) FAST-LIO2 mapping with our ERC config
    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_share, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'config_path': os.path.join(erc_share, 'config'),
            'config_file': 'mid360_mapping.yaml',
            'rviz': rviz,
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_rviz)
    ld.add_action(livox)
    ld.add_action(fast_lio)
    return ld
