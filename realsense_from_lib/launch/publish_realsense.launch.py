from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
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
                    'fps': 30,
                    'color_topic': 'camera/color/image_raw',
                    'depth_topic': 'camera/depth/image_raw',
                }
            ],
        )
    ])
