import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('realsense_from_lib')
    default_json_file = os.path.join(
        package_share,
        'config',
        'ADP_D435_TEST_JSON_USB2.1.json',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'json_file_path',
            default_value=default_json_file,
            description='RealSense Viewer から保存した Advanced Mode JSON 設定ファイル',
        ),
        Node(
            package='realsense_from_lib',
            executable='publish_realsense',
            name='realsense_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'color_width': 640,
                    'color_height': 480,
                    'depth_width': 640,
                    'depth_height': 480,
                    'fps': 15,
                    'color_topic': 'camera/color/image_raw',
                    'depth_topic': 'camera/depth/image_raw',
                    'json_file_path': LaunchConfiguration('json_file_path'),
                    'enable_advanced_mode': True,
                }
            ],
        )
    ])
