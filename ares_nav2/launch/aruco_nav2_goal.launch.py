# Copyright 2025 ares_nav2
# ArUco マーカー TF を Nav2 のゴールとして送信するノードを起動する launch ファイル

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("ares_nav2")

    target_frame_arg = DeclareLaunchArgument(
        "target_frame",
        default_value="map",
        description="Nav2 ゴールの基準フレーム（map または odom）",
    )
    aruco_frame_arg = DeclareLaunchArgument(
        "aruco_frame",
        default_value="aruco_marker",
        description="ArUco マーカーの TF フレーム名",
    )
    robot_base_frame_arg = DeclareLaunchArgument(
        "robot_base_frame",
        default_value="base_link",
        description="ローバー本体の基準フレーム",
    )
    goal_send_interval_arg = DeclareLaunchArgument(
        "goal_send_interval_sec",
        default_value="5.0",
        description="ゴール送信の最小間隔（秒）",
    )
    goal_update_threshold_arg = DeclareLaunchArgument(
        "goal_update_threshold",
        default_value="0.3",
        description="TFがこの距離(m)以上移動したらゴールを更新する",
    )
    send_only_once_arg = DeclareLaunchArgument(
        "send_only_once",
        default_value="false",
        description="true の場合、マーカー検出時に1回だけゴールを送信",
    )
    goal_tolerance_arg = DeclareLaunchArgument(
        "goal_tolerance",
        default_value="2.0",
        description="ArUco タグからこの距離(m)以内に入れば到達扱いにする",
    )

    aruco_nav2_goal_node = Node(
        package="ares_nav2",
        executable="aruco_nav2_goal_node",
        name="aruco_nav2_goal",
        output="screen",
        parameters=[
            {
                "target_frame": LaunchConfiguration("target_frame"),
                "aruco_frame": LaunchConfiguration("aruco_frame"),
                "robot_base_frame": LaunchConfiguration("robot_base_frame"),
                "goal_send_interval_sec": LaunchConfiguration("goal_send_interval_sec"),
                "goal_update_threshold": LaunchConfiguration("goal_update_threshold"),
                "send_only_once": LaunchConfiguration("send_only_once"),
                "goal_tolerance": LaunchConfiguration("goal_tolerance"),
            }
        ],
    )

    ld = LaunchDescription()
    ld.add_action(target_frame_arg)
    ld.add_action(aruco_frame_arg)
    ld.add_action(robot_base_frame_arg)
    ld.add_action(goal_send_interval_arg)
    ld.add_action(goal_update_threshold_arg)
    ld.add_action(send_only_once_arg)
    ld.add_action(goal_tolerance_arg)
    ld.add_action(aruco_nav2_goal_node)
    return ld
