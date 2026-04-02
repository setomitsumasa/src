from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uart_control',
            executable='serial_publiasher',
            name='uart_serial_publiasher',
            output='screen',
            emulate_tty=True,
            parameters=[],
        )
    ]) 