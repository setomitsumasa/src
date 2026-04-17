import json
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node


CONFIG_FILENAME = 'livox_mount_config.json'


def _load_livox_mount_config():
    share_dir = get_package_share_directory('pointcloud_to_laserscan')
    config_path = os.path.join(share_dir, 'config', CONFIG_FILENAME)
    with open(config_path, 'r', encoding='utf-8') as config_file:
        return json.load(config_file)


def _deg_to_rad_string(value_deg):
    return str(math.radians(float(value_deg)))


def generate_launch_description():
    livox_mount_config = _load_livox_mount_config()
    translation = livox_mount_config['translation_m']
    rotation_deg = livox_mount_config['rotation_deg']

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='livox_mount_tf_publisher',
            arguments=[
                '--x', str(translation['x']),
                '--y', str(translation['y']),
                '--z', str(translation['z']),
                '--roll', _deg_to_rad_string(rotation_deg['roll']),
                '--pitch', _deg_to_rad_string(rotation_deg['pitch']),
                '--yaw', _deg_to_rad_string(rotation_deg['yaw']),
                '--frame-id', livox_mount_config['mount_parent_frame'],
                '--child-frame-id', livox_mount_config['sensor_frame'],
            ],
            condition=IfCondition(str(livox_mount_config['publish_static_tf']).lower()),
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', '/converted_pointcloud2'),
                ('scan', '/scan_from_converted_pointcloud'),
                ('angle_filtered_pointcloud', '/angle_filtered_pointcloud'),
                ('corrected_pointcloud', '/corrected_pointcloud'),
            ],
            parameters=[{
                # config/livox_mount_config.json の target_frame と rotation_deg を編集すると、
                # pointcloud_to_laserscan と RViz 表示の補正をまとめて調整できる。
                'target_frame': livox_mount_config['target_frame'],
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -1.57,
                'angle_max': 1.57,
                'angle_increment': 0.005,
                'scan_time': 0.01,
                'range_min': 1.3,
                'range_max': 8.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'apply_slope_filter': True
            }],
            name='pointcloud_angle_filter'
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', '/angle_filtered_pointcloud'),
                ('scan', '/scan'),
                ('angle_filtered_pointcloud', '/angle_filtered_pointcloud_to_scan_debug'),
                ('corrected_pointcloud', '/angle_filtered_corrected_pointcloud'),
            ],
            parameters=[{
                'target_frame': livox_mount_config['target_frame'],
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -1.57,
                'angle_max': 1.57,
                'angle_increment': 0.005,
                'scan_time': 0.01,
                'range_min': 1.3,
                'range_max': 8.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'apply_slope_filter': False,
                'voxel_leaf_size': 0.0
            }],
            name='angle_filtered_pointcloud_to_scan'
        )
    ])
