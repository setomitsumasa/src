# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import shutil
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, LaunchLogDir
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, RegisterEventHandler, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_share = get_package_share_directory("ares_nav2")
    launch_dir = os.path.join(pkg_share, 'launch')
    params_dir = os.path.join(pkg_share, "config")
    nav2_params = os.path.join(params_dir, "nav2_no_map_params.yaml")
    rviz_config_file = os.path.join(params_dir, 'nav2_rviz.rviz')

    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites={}, convert_types=True
    )

    # Add: Nav2パラメータとBT XMLのパス
    nav2_param_file = os.path.join(pkg_share, 'config', 'nav2_no_map_params.yaml')
    nav_to_pose_bt = os.path.join(pkg_share, 'config', 'custom_nav_to_pose.xml')
    nav_through_poses_bt = os.path.join(pkg_share, 'config', 'custom_nav_through_poses.xml')

    # Nav2のBT XMLをカスタムファイルに置き換えるための設定
    bt_substitutions = {
        'default_nav_to_pose_bt_xml': nav_to_pose_bt,
        'default_nav_through_poses_bt_xml': nav_through_poses_bt,
    }
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_param_file,
        root_key='',
        param_rewrites=bt_substitutions,
        convert_types=True
    )

    log_timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    dated_log_filename = f'navigation_sim_{log_timestamp}.log'

    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_use_mapviz_cmd = DeclareLaunchArgument(
        'use_mapviz',
        default_value='False',
        description='Whether to start mapviz')

    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'dual_ekf_navsat.launch.py'))
    )
    
    sample_pointcloud_to_laserscan_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('pointcloud_to_laserscan'),
                         'launch',
                         'sample_pointcloud_to_laserscan_launch.py')
        )
    )

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "False",
            "params_file": configured_nav2_params,  # <= 変更: configured_params -> configured_nav2_params
            "autostart": "True",
        }.items(),
    )

    # Sim用 sensor_data 起動（sim_imu_node, imu_rpy_publisher, sensor_tf_node）
    sensor_data_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ares_sensor'), 'launch', 'sensor_data_publisher.launch.py')
        ),
        launch_arguments={'sim': 'true'}.items(),
    )

    # ArUco tracker launch include
    aruco_tracker_cmd = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('aruco_opencv'), 'launch', 'aruco_tracker.launch.xml')
        )
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'rviz_config': rviz_config_file
        }.items()
    )

    # Force all subprocess output to be shown on screen and persisted into launch.log.
    set_process_output_mode_cmd = SetEnvironmentVariable(
        'OVERRIDE_LAUNCH_PROCESS_OUTPUT', 'both'
    )

    def save_terminal_log(context, *args, **kwargs):
        log_dir = LaunchLogDir().perform(context)
        launch_log_path = os.path.join(log_dir, 'launch.log')
        dated_log_path = os.path.join(log_dir, dated_log_filename)
        if os.path.exists(launch_log_path):
            shutil.copy2(launch_log_path, dated_log_path)
            print(f'[navigation_sim] saved terminal log: {dated_log_path}')
        else:
            print(f'[navigation_sim] launch.log not found: {launch_log_path}')
        return []

    save_terminal_log_on_shutdown_cmd = RegisterEventHandler(
        OnShutdown(on_shutdown=[OpaqueFunction(function=save_terminal_log)])
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(set_process_output_mode_cmd)

    ld.add_action(robot_localization_cmd)

    # navigation2 launch
    ld.add_action(navigation2_cmd)
    # sensor data launch for simulation
    ld.add_action(sensor_data_publisher_cmd)
    # aruco tracker launch
    ld.add_action(aruco_tracker_cmd)
    # sample pointcloud_to_laserscan launch
    ld.add_action(sample_pointcloud_to_laserscan_cmd)

    # rviz launch
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(declare_use_mapviz_cmd)
    ld.add_action(save_terminal_log_on_shutdown_cmd)

    return ld
