from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """make_direction ノード（publish_direction）を起動する launch ファイル。"""
    return LaunchDescription([
        Node(
            package="YOLO_detection_v2",
            executable="publish_direction",
            name="make_direction",
            output="screen",
        ),
    ])
