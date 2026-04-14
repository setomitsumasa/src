from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/converted_pointcloud2'),
                        ('scan', '/scan')],
            parameters=[{
                # 空にすると点群と同じフレームでLaserScanを出力。TF変換・MessageFilterを使わず軽量に。
                # Nav2は/scanのframe_idをTFでcostmapのglobal_frameへ変換するため、このままで可。
                'target_frame': 'livox_stabilized',
                'transform_tolerance': 0.01,
                'min_height': 0.4,   # 20cm（この高さ範囲の点だけをLaserScanに投影）
                'max_height': 1.5,    # 2.0m
                'angle_min': -1.54,  # -180度
                'angle_max': 1.54,   # 180度
                'angle_increment': 0.003,
                'scan_time': 0.01,     # 100Hz
                'range_min': 2.0,     # 0.5m
                'range_max': 8.0,     # 8m
                'use_inf': True,     # False: 障害物なし→range_max+inf_epsilon（有限値）。Trueだとinfになる
                'inf_epsilon': 1.0    # use_inf=Falseのとき、空き方向は range_max + 1.0 = 9.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])
