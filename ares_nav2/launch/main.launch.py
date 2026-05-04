import importlib.util
import os

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
import launch
import launch_ros.actions
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


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
    pkg_share = get_package_share_directory('ares_nav2')
    real_waypoints_file = os.path.join(pkg_share, 'config', 'waypoints.yaml')
    sim_waypoints_file = os.path.join(pkg_share, 'config', 'waypoints_sim.yaml')
    waypoint_file = PythonExpression([
        "'",
        LaunchConfiguration('sim'),
        "'.lower() in ['true', '1', 'yes'] and '",
        sim_waypoints_file,
        "' or '",
        real_waypoints_file,
        "'",
    ])

    for action in make_logger_actions(
        'main',
        [
            '/gps/fix',
            '/imu/data',
            '/imu/yaw',
            '/cmd_vel',
            '/cmd_vel_force_stop',
            '/uart_command',
            '/aruco/id',
            '/aruco/enabled',
            '/aruco/target_marker_id',
            '/aruco/goal_reached',
            '/yolo/target_frame',
            '/yolo/goal_reached',
            '/mission/status',
            '/tf',
            '/tf_static',
        ],
    ):
        ld.add_action(action)

    ld.add_action(
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Use simulation waypoint file when true',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'mission_log_directory',
            default_value=PathJoinSubstitution([
                LaunchConfiguration('main_logger_session_dir'),
                'mission_logs',
            ]),
            description='gps_waypoint_follower mission log output directory',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'spiral_spin_scan_enabled',
            default_value='true',
            description='Enable in-place spin scan at each spiral search point',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'spiral_spin_scan_total_angle_rad',
            default_value='12.5663706144',  # 4 full rotations
            description='Total spin angle at each spiral search point',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'spiral_spin_scan_angular_speed_rad_s',
            default_value='1.0',
            description='cmd_vel angular.z used during spiral spin scan',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'spiral_spin_scan_linear_speed_m_s',
            default_value='0.3',
            description='cmd_vel linear.x used during spiral spin scan',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'spiral_spin_scan_direction',
            default_value='1',
            description='Spin direction, 1 or -1',
        )
    )
    ld.add_action(
        launch_ros.actions.Node(
            package='ares_nav2',
            executable='gps_waypoint_follower_node',
            name='gps_waypoint_follower',
            output='both',
            arguments=['--waypoints-file', waypoint_file],
            parameters=[{
                'mission_log_directory': LaunchConfiguration('mission_log_directory'),
                'mission_log_to_file': True,
                'mission_status_topic': '/mission/status',
                'mission_status_period_sec': 1.0,
                'spiral_spin_scan_enabled': ParameterValue(
                    LaunchConfiguration('spiral_spin_scan_enabled'), value_type=bool),
                'spiral_spin_scan_total_angle_rad': ParameterValue(
                    LaunchConfiguration('spiral_spin_scan_total_angle_rad'), value_type=float),
                'spiral_spin_scan_angular_speed_rad_s': ParameterValue(
                    LaunchConfiguration('spiral_spin_scan_angular_speed_rad_s'), value_type=float),
                'spiral_spin_scan_linear_speed_m_s': ParameterValue(
                    LaunchConfiguration('spiral_spin_scan_linear_speed_m_s'), value_type=float),
                'spiral_spin_scan_direction': ParameterValue(
                    LaunchConfiguration('spiral_spin_scan_direction'), value_type=int),
            }])
    )
    ld.add_action(
        launch_ros.actions.Node(
            package='ares_nav2',
            executable='aruco_nav2_goal_node',
            name='aruco_nav2_goal',
            output='both')
    )
    ld.add_action(
        launch_ros.actions.Node(
            package='ares_nav2',
            executable='yolo_tf_nav2_goal_node',
            name='yolo_tf_nav2_goal',
            output='both',
            parameters=[{
                'goal_tolerance': 3.0,
            }])
    )

    return ld
