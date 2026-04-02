from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """rover_controller ノードを起動する launch ファイル。"""
    return LaunchDescription([
        Node(
            package="rover_controller",
            executable="rover_controller_node",
            name="rover_controller_node",
            output="screen",
        ),
    ])
