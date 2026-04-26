from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    target_frame_arg = DeclareLaunchArgument(
        "target_frame",
        default_value="map",
        description="Nav2 goal の基準フレーム",
    )
    goal_send_interval_arg = DeclareLaunchArgument(
        "goal_send_interval_sec",
        default_value="1.0",
        description="YOLO TF goal を再送する最小間隔（秒）",
    )
    goal_update_threshold_arg = DeclareLaunchArgument(
        "goal_update_threshold",
        default_value="1.0",
        description="TFがこの距離(m)以上動いたらゴール更新する",
    )
    max_tf_age_arg = DeclareLaunchArgument(
        "max_tf_age_sec",
        default_value="0.5",
        description="この秒数より古い TF は無視する",
    )

    yolo_tf_nav2_goal_node = Node(
        package="ares_nav2",
        executable="yolo_tf_nav2_goal_node",
        name="yolo_tf_nav2_goal",
        output="screen",
        parameters=[
            {
                "target_frame": LaunchConfiguration("target_frame"),
                "goal_send_interval_sec": LaunchConfiguration("goal_send_interval_sec"),
                "goal_update_threshold": LaunchConfiguration("goal_update_threshold"),
                "max_tf_age_sec": LaunchConfiguration("max_tf_age_sec"),
            }
        ],
    )

    ld = LaunchDescription()
    ld.add_action(target_frame_arg)
    ld.add_action(goal_send_interval_arg)
    ld.add_action(goal_update_threshold_arg)
    ld.add_action(max_tf_age_arg)
    ld.add_action(yolo_tf_nav2_goal_node)
    return ld
