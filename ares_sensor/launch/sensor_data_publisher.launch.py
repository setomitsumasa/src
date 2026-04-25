from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch
import launch_ros.actions


def generate_launch_description() -> LaunchDescription:
    """
    IMU と GPS などセンサ関連ノードをまとめて起動する launch ファイル。

    - UART 経由 IMU publisher: ares_sensor/imu_node (imu.cpp)
    - IMU RPY publisher: ares_sensor/sensor_node
    - UART 経由 GPS publisher: ares_sensor/gps_node (gps.cpp)
    - TF publisher: ares_sensor/sensor_tf_node (map->odom, odom->base_link, odom->imu_link)
    """
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "sim",
                default_value="false",
                description="true のとき UART GPS/IMU ノードを起動しない",
            ),
            # UART-IMU ノード（/uart_data -> /imu/data: sensor_msgs/Imu）
            launch_ros.actions.Node(
                package="ares_sensor",
                executable="imu_node",
                name="uart_imu_node",
                condition=UnlessCondition(LaunchConfiguration("sim")),
            ),
            # シミュレータ用 IMU ノード（/imu -> /imu/data）
            launch_ros.actions.Node(
                package="ares_sensor",
                executable="sim_imu_node",
                name="sim_imu_node",
                condition=IfCondition(LaunchConfiguration("sim")),
            ),
            # 既存 IMU RPY ノード
            launch_ros.actions.Node(
                package="ares_sensor",
                executable="sensor_node",
                name="imu_rpy_publisher",
            ),
            # UART-GPS ノード
            launch_ros.actions.Node(
                package="ares_sensor",
                executable="gps_node",
                name="uart_gps_node",
                condition=UnlessCondition(LaunchConfiguration("sim")),
            ),
            # TF: map->odom, odom->base_link, odom->imu_link
            launch_ros.actions.Node(
                package="ares_sensor",
                executable="sensor_tf_node",
                name="sensor_tf_node",
            ),
        ]
    )
