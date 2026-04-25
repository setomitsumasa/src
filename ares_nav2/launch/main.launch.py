import importlib.util
import os

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch
import launch_ros.actions


_logger_utils_path = os.path.join(os.path.dirname(__file__), 'logger_utils.py')
_logger_utils_spec = importlib.util.spec_from_file_location(
    'ares_nav2_launch_logger_utils',
    _logger_utils_path,
)
_logger_utils = importlib.util.module_from_spec(_logger_utils_spec)
_logger_utils_spec.loader.exec_module(_logger_utils)
make_logger_actions = _logger_utils.make_logger_actions

def generate_launch_description():
    ld = launch.LaunchDescription()

    for action in make_logger_actions(
        'main',
        [
            '/gps/fix',
            '/imu/data',
            '/imu/yaw',
            '/cmd_vel',
            '/uart_command',
            '/aruco/id',
            '/aruco/enabled',
            '/aruco/target_marker_id',
            '/aruco/goal_reached',
            '/yolo/target_frame',
            '/yolo/goal_reached',
            '/tf',
            '/tf_static',
        ],
    ):
        ld.add_action(action)

    ld.add_action(
        DeclareLaunchArgument(
            'mission_log_directory',
            default_value='mission_logs',
            description='gps_waypoint_follower mission log output directory',
        )
    )
    ld.add_action(
        launch_ros.actions.Node(
            package='ares_nav2',
            executable='gps_waypoint_follower_node',
            name='gps_waypoint_follower',
            output='screen',
            parameters=[{
                'mission_log_directory': LaunchConfiguration('mission_log_directory'),
                'mission_log_to_file': True,
            }])
    )
    ld.add_action(
        launch_ros.actions.Node(
            package='ares_nav2',
            executable='aruco_nav2_goal_node',
            name='aruco_nav2_goal',
            output='screen')
    )
    ld.add_action(
        launch_ros.actions.Node(
            package='ares_nav2',
            executable='yolo_tf_nav2_goal_node',
            name='yolo_tf_nav2_goal',
            output='screen')
    )

    return ld
