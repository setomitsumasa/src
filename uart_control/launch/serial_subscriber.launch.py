from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uart_control',
            executable='serial_subscriber',
            name='uart_serial_subscriber',
            output='screen',
            emulate_tty=True,
            parameters=[],
        )
    ])

