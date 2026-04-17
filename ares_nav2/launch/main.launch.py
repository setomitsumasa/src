from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'mission_log_directory',
            default_value='mission_logs',
            description='gps_waypoint_follower mission log output directory',
        ),
        launch_ros.actions.Node(
            package='ares_nav2',
            executable='gps_waypoint_follower_node',
            name='gps_waypoint_follower',
            output='screen',
            parameters=[{
                'mission_log_directory': LaunchConfiguration('mission_log_directory'),
                'mission_log_to_file': True,
            }]),
        launch_ros.actions.Node(
            package='ares_nav2',
            executable='aruco_nav2_goal_node',
            name='aruco_nav2_goal',
            output='screen'),
        launch_ros.actions.Node(
            package='ares_nav2',
            executable='yolo_tf_nav2_goal_node',
            name='yolo_tf_nav2_goal',
            output='screen'),
  ])
