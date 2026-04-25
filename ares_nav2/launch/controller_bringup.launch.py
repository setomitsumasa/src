import os
import importlib.util

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node


_logger_utils_path = os.path.join(os.path.dirname(__file__), 'logger_utils.py')
_logger_utils_spec = importlib.util.spec_from_file_location(
    'ares_nav2_launch_logger_utils',
    _logger_utils_path,
)
_logger_utils = importlib.util.module_from_spec(_logger_utils_spec)
_logger_utils_spec.loader.exec_module(_logger_utils)
make_logger_actions = _logger_utils.make_logger_actions


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

    yolo_tf = Node(
        package='YOLO_detection_v2',
        executable='publish_YOLOtf',
        name='publish_YOLOtf',
        output='screen',
    )

    ld = LaunchDescription()

    for action in make_logger_actions(
        'controller_bringup',
        [
            '/uart_data',
            '/uart_command',
            '/gps/fix',
            '/imu/data',
            '/imu/yaw',
            '/cmd_vel',
            '/livox/lidar',
            '/converted_pointcloud2',
            '/scan',
            '/aruco/id',
            '/aruco/enabled',
            '/tf',
            '/tf_static',
        ],
    ):
        ld.add_action(action)

    ld.add_action(
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='true のとき sensor_data_publisher の UART GPS/IMU ノードを起動しない',
        )
    )
    ld.add_action(serial_subscriber)
    ld.add_action(TimerAction(period=2.0, actions=[serial_publisher]))
    ld.add_action(TimerAction(period=4.0, actions=[realsense]))
    ld.add_action(TimerAction(period=7.0, actions=[aruco_tracker]))
    ld.add_action(TimerAction(period=6.0, actions=[sensor_data]))
    ld.add_action(TimerAction(period=8.0, actions=[rover_controller]))
    ld.add_action(TimerAction(period=9.0, actions=[yolo_tf]))
    ld.add_action(TimerAction(period=10.0, actions=[livox_driver]))
    ld.add_action(TimerAction(period=12.0, actions=[livox_to_pointcloud2]))

    return ld
