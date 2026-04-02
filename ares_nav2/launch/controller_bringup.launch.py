import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    uart_control_share = get_package_share_directory('uart_control')
    ares_sensor_share = get_package_share_directory('ares_sensor')
    rover_controller_share = get_package_share_directory('rover_controller')
    livox_share = get_package_share_directory('livox_ros_driver2')
    realsense_share = get_package_share_directory('realsense_from_lib')

    serial_subscriber = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(uart_control_share, 'launch', 'serial_subscriber.launch.py')
        )
    )

    serial_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(uart_control_share, 'launch', 'serial_publiasher.launch.py')
        )
    )

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, 'launch', 'publish_realsense.launch.py')
        )
    )

    sensor_data = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ares_sensor_share, 'launch', 'sensor_data_publisher.launch.py')
        )
    )

    rover_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_controller_share, 'launch', 'rover_controller.launch.py')
        )
    )

    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_share, 'launch_ROS2', 'msg_MID360_launch.py')
        )
    )

    livox_to_pointcloud2 = Node(
        package='livox_to_pointcloud2',
        executable='livox_to_pointcloud2_node',
        name='livox_to_pointcloud2',
        output='screen',
        remappings=[
            ('/livox_pointcloud', '/livox/lidar'),
        ],
    )

    return LaunchDescription([
        serial_subscriber,
        TimerAction(period=2.0, actions=[serial_publisher]),
        TimerAction(period=4.0, actions=[realsense]),
        TimerAction(period=6.0, actions=[sensor_data]),
        TimerAction(period=8.0, actions=[rover_controller]),
        TimerAction(period=10.0, actions=[livox_driver]),
        TimerAction(period=12.0, actions=[livox_to_pointcloud2]),
    ])
