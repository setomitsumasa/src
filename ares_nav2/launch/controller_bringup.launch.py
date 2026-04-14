import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    uart_control_share = get_package_share_directory('uart_control')
    ares_sensor_share = get_package_share_directory('ares_sensor')
    rover_controller_share = get_package_share_directory('rover_controller')
    livox_share = get_package_share_directory('livox_ros_driver2')
    realsense_share = get_package_share_directory('realsense_from_lib')
    aruco_share = get_package_share_directory('aruco_opencv')

    sim = LaunchConfiguration('sim')

    serial_subscriber = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(uart_control_share, 'launch', 'serial_subscriber.launch.py')
        ),
        condition=UnlessCondition(sim),
    )

    serial_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(uart_control_share, 'launch', 'serial_publiasher.launch.py')
        ),
        condition=UnlessCondition(sim),
    )

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, 'launch', 'publish_realsense.launch.py')
        ),
        condition=UnlessCondition(sim),
    )

    aruco_tracker = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(aruco_share, 'launch', 'aruco_tracker.launch.xml')
        )
    )

    sensor_data = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ares_sensor_share, 'launch', 'sensor_data_publisher.launch.py')
        ),
        launch_arguments={'sim': sim}.items(),
    )

    rover_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_controller_share, 'launch', 'rover_controller.launch.py')
        )
    )

    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_share, 'launch_ROS2', 'msg_MID360_launch.py')
        ),
        condition=UnlessCondition(sim),
    )

    livox_to_pointcloud2 = Node(
        package='livox_to_pointcloud2',
        executable='livox_to_pointcloud2_node',
        name='livox_to_pointcloud2',
        output='screen',
        remappings=[
            ('/livox_pointcloud', '/livox/lidar'),
        ],
        parameters=[{
            'sim': sim,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='true のとき sensor_data_publisher の UART GPS/IMU ノードを起動しない',
        ),
        serial_subscriber,
        TimerAction(period=2.0, actions=[serial_publisher]),
        TimerAction(period=4.0, actions=[realsense]),
        TimerAction(period=7.0, actions=[aruco_tracker]),
        TimerAction(period=6.0, actions=[sensor_data]),
        TimerAction(period=8.0, actions=[rover_controller]),
        TimerAction(period=10.0, actions=[livox_driver]),
        TimerAction(period=12.0, actions=[livox_to_pointcloud2]),
    ])
